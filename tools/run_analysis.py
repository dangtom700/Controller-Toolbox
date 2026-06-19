"""
tools/run_analysis.py  --  Batch analysis runner for Python case studies.

Discovers every Python case study that has config/analysis.json, imports its
sim/main.py module, then runs analyses for every controller:

  1. Monte Carlo     -- parameter robustness sweep using run_single()
  2. Fault sweep     -- sensor / actuator fault injection using run_with_fault()
  3. WCET            -- per-step compute() timing, only if the study already has
                       raw case-study/<study>/logs/wcet_*.csv (needs a study-specific
                       run_wcet_profile() hook in sim/main.py; skipped otherwise)
  4. Mu analysis     -- ARMA-based robustness margins from existing run_*.csv logs

Each study's output files are written inside its own case-study/<study>/ folder
(same place generate_report.py reads them from):
  mc_summary.csv
  fault_sweep.csv
  wcet_summary.csv  (only if raw wcet_*.csv data exists)
  mu_analysis.csv

Usage (from project root):
    conda run -n soft_robotics python tools/run_analysis.py [options]

Options:
    --study   NAME   run only this study (partial-match on folder name; default: all)
    --skip    LIST   comma-separated analyses to skip: mc, fault, wcet, mu
    --mc-n    INT    override mc_n_samples from analysis.json
    --sigma   FLOAT  override mc_sigma from analysis.json
    --dry-run        print what would run without actually running

See tools/study_protocol.py for the hook contract every sim/main.py must honour.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import random
import sys
import time
import traceback
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent

sys.path.insert(0, str(_ROOT))


# ---------------------------------------------------------------------------
# Study discovery
# ---------------------------------------------------------------------------

def discover_studies(study_filter: str = "") -> list[dict]:
    """Return list of study-info dicts for every Python study with analysis.json."""
    case_root = _ROOT / "case-study"
    studies = []
    for d in sorted(case_root.iterdir()):
        if not d.is_dir() or d.name.startswith("."):
            continue
        main_py = d / "sim" / "main.py"
        analysis_json = d / "config" / "analysis.json"
        if not (main_py.exists() and analysis_json.exists()):
            continue
        if study_filter and study_filter.lower() not in d.name.lower():
            continue
        with open(analysis_json) as fh:
            cfg = json.load(fh)
        studies.append({
            "name":     d.name,
            "slug":     d.name.replace(" ", "_").replace("/", "_")[:40],
            "dir":      d,
            "main_py":  main_py,
            "cfg":      cfg,
        })
    return studies


# ---------------------------------------------------------------------------
# Module loader
# ---------------------------------------------------------------------------

def load_sim_module(main_py: Path, study_dir: Path):
    """Dynamically import sim/main.py and return (module_or_None, cleanup_fn).

    The caller MUST call cleanup_fn() once it is done with this study (after
    MC/fault/WCET processing), even if module loading failed. Every case
    study's sim/ package commonly reuses the same generic file names
    (controllers.py, simulation_runner.py, ...); Python caches imports by
    short module name in sys.modules, not by path, so without this cleanup
    the *next* study's `from controllers import make_controllers` would
    silently resolve to *this* study's already-cached controllers module
    instead of its own (confirmed empirically: running multiple studies in
    one process without this caused 5 of 7 to fail with the wrong study's
    plant-params schema, surfacing as "CONTROLLER_NAMES empty or not found").
    """
    sim_dir = str(study_dir / "sim")
    sim_dir_norm = os.path.normcase(os.path.normpath(sim_dir))
    sys.path.insert(0, sim_dir)
    pre_existing_modules = set(sys.modules.keys())

    def cleanup():
        if sim_dir in sys.path:
            sys.path.remove(sim_dir)
        # Only purge modules that actually live under *this* study's sim/
        # directory (controllers.py, simulation_runner.py, ...) - NOT
        # third-party packages like numpy/pandas that happened to be
        # first-imported while loading this study; some C-extension
        # packages (numpy in particular) cannot be re-imported into a
        # fresh module object within the same process.
        for name in list(sys.modules.keys()):
            if name in pre_existing_modules:
                continue
            mod_obj  = sys.modules.get(name)
            mod_file = getattr(mod_obj, "__file__", None) or ""
            if os.path.normcase(os.path.normpath(mod_file)).startswith(sim_dir_norm):
                del sys.modules[name]

    spec = importlib.util.spec_from_file_location("_analysis_sim", str(main_py))
    mod  = importlib.util.module_from_spec(spec)
    mod.__dict__["__file__"] = str(main_py)
    try:
        spec.loader.exec_module(mod)
    except SystemExit:
        # Some studies do sys.exit(0) when ctrl_toolbox is absent
        return None, cleanup
    return mod, cleanup


# ---------------------------------------------------------------------------
# Parameter perturbation
# ---------------------------------------------------------------------------

def _perturb_params(nominal: dict, sigma: float,
                    keys: list[str], rng: random.Random) -> dict:
    """Return a copy of nominal with the listed scalar keys perturbed by +/-sigma."""
    import copy
    result = copy.deepcopy(nominal)
    for k in keys:
        v = result.get(k)
        if v is None:
            continue
        if isinstance(v, (int, float)) and not isinstance(v, bool) and v != 0:
            factor = 1.0 + rng.gauss(0.0, sigma)
            result[k] = v * max(factor, 0.1)
    return result


# ---------------------------------------------------------------------------
# Monte Carlo
# ---------------------------------------------------------------------------

def run_mc(mod, study: dict, n_samples: int, sigma: float, dry_run: bool) -> Path | None:
    """Run MC for all controllers using run_single(); write mc_summary CSV."""
    ctrl_names: list[str] = getattr(mod, "CONTROLLER_NAMES", [])
    if not ctrl_names:
        print(f"    [MC] SKIP - CONTROLLER_NAMES empty or not found")
        return None

    keys = study["cfg"].get("mc_perturb_keys", [])
    sid  = study["cfg"].get("nominal_scenario_id", None)
    rng  = random.Random(42)

    nom_params: dict = getattr(mod, "_H_NOM", {})
    if not nom_params and hasattr(mod, "_h_nom"):
        try:
            nom_params = mod._h_nom()
        except Exception:
            pass

    if dry_run:
        print(f"    [MC] DRY RUN  {len(ctrl_names)} ctrls x {n_samples} samples "
              f"  sigma={sigma}  keys={keys[:4]}{'...' if len(keys)>4 else ''}")
        return None

    rows = []
    total = len(ctrl_names) * n_samples
    done  = 0
    t0    = time.time()
    for ctrl_name in ctrl_names:
        for i in range(n_samples):
            done += 1
            perturbed = _perturb_params(nom_params, sigma, keys, rng)
            try:
                result = mod.run_single(ctrl_name, params=perturbed,
                                        scenario_id=sid)
                iae  = float(result.get("IAE", float("nan")))
                row  = {
                    "study":      study["name"],
                    "controller": ctrl_name,
                    "sample":     i,
                    "iae":        iae,
                }
                # Copy any extra numeric keys
                for rk, rv in result.items():
                    if rk not in row and isinstance(rv, (int, float)):
                        row[rk] = rv
                rows.append(row)
            except Exception as exc:
                rows.append({
                    "study": study["name"], "controller": ctrl_name,
                    "sample": i, "iae": float("nan"), "error": str(exc),
                })
        elapsed = time.time() - t0
        print(f"    [MC] {ctrl_name:<24}  {n_samples} samples  "
              f"elapsed {elapsed:.1f}s")

    # Write CSV
    out = study["dir"] / "mc_summary.csv"
    _write_csv(out, rows)
    print(f"    [MC] Written: {out.relative_to(_ROOT)}  ({len(rows)} rows)")
    return out


# ---------------------------------------------------------------------------
# Fault sweep
# ---------------------------------------------------------------------------

def run_fault_sweep(mod, study: dict, dry_run: bool) -> Path | None:
    """Run fault sweep for all controllers using run_with_fault(); write CSV."""
    ctrl_names: list[str] = getattr(mod, "CONTROLLER_NAMES", [])
    if not ctrl_names:
        print(f"    [FAULT] SKIP - CONTROLLER_NAMES empty or not found")
        return None

    fault_specs = study["cfg"].get("fault_specs", [])
    if not fault_specs:
        print(f"    [FAULT] SKIP - no fault_specs in analysis.json")
        return None

    sid = study["cfg"].get("nominal_scenario_id", None)

    if dry_run:
        print(f"    [FAULT] DRY RUN  {len(ctrl_names)} ctrls x {len(fault_specs)} "
              f"fault types")
        return None

    try:
        from tools.fault_injector import FaultSpec
    except ImportError:
        print("    [FAULT] SKIP - tools.fault_injector not importable")
        return None

    rows = []
    t0   = time.time()
    for ctrl_name in ctrl_names:
        for fspec in fault_specs:
            kind       = fspec["kind"]
            magnitudes = fspec.get("magnitudes", [0.0])
            for mag in magnitudes:
                fault = FaultSpec(kind=kind, fault_time=0.0, magnitude=mag)
                try:
                    result = mod.run_with_fault(ctrl_name, fault,
                                                scenario_id=sid)
                    iae  = float(result.get("IAE", float("nan")))
                    row  = {
                        "study":      study["name"],
                        "controller": ctrl_name,
                        "fault_kind": kind,
                        "magnitude":  mag,
                        "iae":        iae,
                    }
                    for rk, rv in result.items():
                        if rk not in row and isinstance(rv, (int, float)):
                            row[rk] = rv
                    rows.append(row)
                except Exception as exc:
                    rows.append({
                        "study": study["name"], "controller": ctrl_name,
                        "fault_kind": kind, "magnitude": mag,
                        "iae": float("nan"), "error": str(exc),
                    })
        elapsed = time.time() - t0
        print(f"    [FAULT] {ctrl_name:<24}  {len(fault_specs)} fault types  "
              f"elapsed {elapsed:.1f}s")

    out = study["dir"] / "fault_sweep.csv"
    _write_csv(out, rows)
    print(f"    [FAULT] Written: {out.relative_to(_ROOT)}  ({len(rows)} rows)")
    return out


# ---------------------------------------------------------------------------
# WCET (CSV-based; only runs for studies that already have raw wcet_*.csv
# data under their logs/ - that data comes from a study-specific
# run_wcet_profile()-style hook in sim/main.py, which this script does not
# generate on its own)
# ---------------------------------------------------------------------------

def run_wcet(study: dict, dry_run: bool) -> Path | None:
    has_raw = any(study["dir"].joinpath("logs").glob("wcet_*.csv"))
    if not has_raw:
        print(f"    [WCET] SKIP - no logs/wcet_*.csv (needs a run_wcet_profile() hook in sim/main.py)")
        return None
    if dry_run:
        print(f"    [WCET] DRY RUN  (would aggregate existing logs/wcet_*.csv)")
        return None
    try:
        from tools.wcet_report import main as wcet_main
        wcet_main(["--study", study["name"]])
        out = study["dir"] / "wcet_summary.csv"
        print(f"    [WCET] Written: {out.relative_to(_ROOT)}")
        return out
    except SystemExit as exc:
        # wcet_report.py calls sys.exit() on "no data" - not an Exception
        # subclass, so it would otherwise escape uncaught and abort the
        # whole batch run.
        if exc.code in (0, None):
            print("    [WCET] SKIP - no data after filtering")
        else:
            print(f"    [WCET] WARN: tool exited with code {exc.code}")
        return None
    except Exception as exc:
        print(f"    [WCET] WARN: {exc}")
        return None


# ---------------------------------------------------------------------------
# Mu analysis (CSV-based, no hooks needed; runs once after all studies)
# ---------------------------------------------------------------------------

def run_mu_all(study_filter: str, dry_run: bool) -> None:
    """Run mu_analysis across all case-study CSVs; writes mu_analysis.csv per study."""
    if dry_run:
        print(f"\n  [MU] DRY RUN  (would read all logs/run_*.csv)")
        return
    try:
        from tools.mu_analysis import main as mu_main
        mu_args = ["--study", study_filter] if study_filter else []
        mu_main(mu_args)
    except SystemExit as exc:
        # mu_analysis.py calls sys.exit() on "no CSVs found" - same
        # uncaught-by-Exception hazard as wcet_main() above.
        if exc.code in (0, None):
            print("  [MU] SKIP - no run_*.csv data found")
        else:
            print(f"  [MU] WARN: tool exited with code {exc.code}")
    except Exception as exc:
        print(f"  [MU] WARN: {exc}")


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    import csv
    # Collect all keys across rows (preserving insertion order for common keys)
    all_keys: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for k in row:
            if k not in seen:
                all_keys.append(k)
                seen.add(k)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=all_keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study",   default="",
                    help="Partial-match filter on study folder name")
    ap.add_argument("--skip",    default="",
                    help="Comma-separated analyses to skip: mc, fault, wcet, mu")
    ap.add_argument("--mc-n",    type=int, default=None,
                    help="Override mc_n_samples from analysis.json")
    ap.add_argument("--sigma",   type=float, default=None,
                    help="Override mc_sigma from analysis.json")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print what would run without executing")
    args = ap.parse_args(argv)

    skip = {s.strip().lower() for s in args.skip.split(",") if s.strip()}
    studies = discover_studies(args.study)

    print(f"\n{'='*60}")
    print(f"  run_analysis.py  -  {len(studies)} study/studies with MC/fault hooks found")
    print(f"{'='*60}\n")

    if not studies:
        print(f"No Python studies with config/analysis.json found"
              + (f" matching '{args.study}'" if args.study else "")
              + " - skipping MC/fault/WCET (mu analysis runs regardless, below).")

    for study in studies:
        print(f"\n[{study['name']}]")

        # Load module
        mod, cleanup = load_sim_module(study["main_py"], study["dir"])
        try:
            if mod is None:
                print("  SKIP - module failed to load (ctrl_toolbox absent?)")
                continue

            # MC
            if "mc" not in skip:
                n_samples = args.mc_n  or study["cfg"].get("mc_n_samples", 30)
                sigma     = args.sigma or study["cfg"].get("mc_sigma", 0.10)
                try:
                    run_mc(mod, study, n_samples, sigma, args.dry_run)
                except Exception:
                    print("  [MC] ERROR:")
                    traceback.print_exc()

            # Fault sweep
            if "fault" not in skip:
                try:
                    run_fault_sweep(mod, study, args.dry_run)
                except Exception:
                    print("  [FAULT] ERROR:")
                    traceback.print_exc()

            # WCET (only if the study already has raw timing data)
            if "wcet" not in skip:
                try:
                    run_wcet(study, args.dry_run)
                except Exception:
                    print("  [WCET] ERROR:")
                    traceback.print_exc()
        finally:
            # Purge this study's sim/ sys.path entry + freshly-cached modules
            # before moving to the next study (see load_sim_module docstring).
            cleanup()

    # Mu analysis works directly off logs/run_*.csv - no sim/main.py or
    # analysis.json needed, so it must run even when the hook-based loop
    # above found nothing (e.g. a C++ study, or a Python study missing
    # config/analysis.json).
    if "mu" not in skip:
        run_mu_all(args.study, args.dry_run)

    print(f"\n{'='*60}")
    print(f"  run_analysis.py complete.")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
