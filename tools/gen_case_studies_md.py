#!/usr/bin/env python3
"""
gen_case_studies_md.py
======================
Auto-generate / verify case-study counts in docs/CASE_STUDIES.md.

All counts are extracted from the actual source files — never from documentation.

Modes
-----
  (default)   Print a ground-truth summary table and show any discrepancies with
              docs/CASE_STUDIES.md.

  --patch     Patch the count fields in docs/CASE_STUDIES.md in-place.
              Touches only lines matching "| **Controllers** |", "| **Scenarios** |",
              "| **Total runs** |" inside each study block.  All other content is
              preserved verbatim.

  --generate  Overwrite docs/CASE_STUDIES.md with a freshly generated file.
              WARNING: erases all hand-written caveats / plant descriptions.
              Use --patch for routine updates; --generate for a full rebuild.

Usage
-----
  conda run -n soft_robotics -- python tools/gen_case_studies_md.py
  conda run -n soft_robotics -- python tools/gen_case_studies_md.py --patch
  conda run -n soft_robotics -- python tools/gen_case_studies_md.py --generate
"""

import os
import re
import sys
import glob
import argparse

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CASE_DIR = os.path.join(ROOT, "case-study")
DOCS_DIR = os.path.join(ROOT, "docs")
CASE_STUDIES_MD = os.path.join(DOCS_DIR, "CASE_STUDIES.md")
COMPILE_BAT = os.path.join(ROOT, "compile.bat")
CASE_CMAKE = os.path.join(CASE_DIR, "CMakeLists.txt")


# ---------------------------------------------------------------------------
# Classification helpers
# ---------------------------------------------------------------------------

def _cmake_registered_studies():
    """Return set of study folder names in case-study/CMakeLists.txt add_subdirectory lines."""
    registered = set()
    try:
        with open(CASE_CMAKE, encoding="utf-8") as f:
            for line in f:
                m = re.search(r'add_subdirectory\("([^"]+)"\)', line)
                if m:
                    registered.add(m.group(1))
    except FileNotFoundError:
        pass
    return registered


def _sim_targets_from_compile_bat():
    """Return dict mapping sim-target names from compile.bat (heuristic: *_sim entries)."""
    targets = []
    try:
        with open(COMPILE_BAT, encoding="utf-8", errors="replace") as f:
            for line in f:
                m = re.match(r'\s+(\w+_sim)\b', line)
                if m:
                    targets.append(m.group(1))
    except FileNotFoundError:
        pass
    return targets


# ---------------------------------------------------------------------------
# Count extractors
# ---------------------------------------------------------------------------

def _cpp_controller_count(study_dir):
    """Return (count, source) from main.cpp: prefers N_CONTROLLERS/kNumControllers constants."""
    main_cpp = os.path.join(study_dir, "sim", "src", "main.cpp")
    if not os.path.exists(main_cpp):
        return None, "no main.cpp"
    with open(main_cpp, encoding="utf-8") as f:
        content = f.read()

    # 1. Named constant
    m = re.search(r'(?:N_CONTROLLERS|kNumControllers)\s*=\s*(\d+)', content)
    if m:
        return int(m.group(1)), "N_CONTROLLERS constant"

    # 2. Print statement
    m = re.search(r'Controllers per scenario:\s*(\d+)', content)
    if m:
        return int(m.group(1)), "cout print"

    # 3. Count controller push_backs (exclude scenario push_backs)
    n = len(re.findall(r'controllers\.push_back\(std::make_unique<', content))
    if n:
        return n, f"{n} push_back lines"

    return None, "not found"


def _cpp_scenario_count(study_dir):
    """Return (count, source) for a C++ study."""
    # Try JSON scenario directories (two common layouts)
    for scen_path in [
        os.path.join(study_dir, "config", "scenarios"),
        os.path.join(study_dir, "sim", "config", "scenarios"),
    ]:
        jsons = glob.glob(os.path.join(scen_path, "*.json"))
        if jsons:
            return len(jsons), f"JSON files in {os.path.relpath(scen_path, ROOT)}"

    # Hardcoded scenario names in main.cpp
    main_cpp = os.path.join(study_dir, "sim", "src", "main.cpp")
    if os.path.exists(main_cpp):
        with open(main_cpp, encoding="utf-8") as f:
            content = f.read()
        scenarios = sorted(set(re.findall(r'"(s\d{2}_[^"]+)"', content)))
        if scenarios:
            return len(scenarios), f"hardcoded ({', '.join(scenarios)})"

    return None, "not found"


def _python_controller_count(study_dir):
    """Return (count, source) from controllers.py/planners.py.

    Tries in order:
      1. factory docstring  "Return ... all N <word> instances"
      2. count class instantiations in make_controllers() return list
      3. main.py header comment
    """
    for fname in ["controllers.py", "planners.py"]:
        path = os.path.join(study_dir, "sim", fname)
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as f:
            content = f.read()

        # 1. Docstring: """Return ... all N \w+ instances."""
        m = re.search(r'Return.*?all\s+(\d+)\s+\w', content, re.DOTALL)
        if m:
            return int(m.group(1)), f"factory docstring in {fname}"

        # 2. Count class instantiations inside make_controllers/make_planners return block.
        #    Match from 'def make_...' to the end of the outer list literal.
        m_fn = re.search(
            r'def make_(?:controllers|planners)\b.*?return \[(.+?)(?:\n\s*\])',
            content, re.DOTALL
        )
        if m_fn:
            block = m_fn.group(1)
            # Each controller is one ClassName(...) call
            instances = re.findall(r'\b\w+(?:Ctrl|Drop|Planner|Controller)\(', block)
            if instances:
                return len(instances), f"counted {len(instances)} items in {fname} return list"

    # 3. main.py header comment
    main_py = os.path.join(study_dir, "sim", "main.py")
    if os.path.exists(main_py):
        with open(main_py, encoding="utf-8") as f:
            for line in f:
                m = re.search(r'(\d+)\s+(?:controller|planner)', line)
                if m:
                    return int(m.group(1)), "main.py header"

    return None, "not found"


def _python_scenario_count(study_dir):
    """Return (count, source) for a Python study."""
    scen_path = os.path.join(study_dir, "config", "scenarios")
    jsons = glob.glob(os.path.join(scen_path, "*.json"))
    if jsons:
        return len(jsons), f"JSON files in config/scenarios/ [{', '.join(sorted(os.path.basename(j) for j in jsons))}]"
    return None, "no config/scenarios/ dir"


def _python_controller_names(study_dir):
    """Return list of controller/planner class names from factory return statement."""
    names = []
    for fname in ["controllers.py", "planners.py"]:
        path = os.path.join(study_dir, "sim", fname)
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as f:
            content = f.read()
        # Find make_controllers return block
        m = re.search(r'def make_(?:controllers|planners)[^)]*\).*?return \[(.*?)\]',
                      content, re.DOTALL)
        if m:
            block = m.group(1)
            # Extract class instantiations
            for cls in re.findall(r'\b(\w+Ctrl|\w+Planner|\w+Drop)\(', block):
                if cls not in names:
                    names.append(cls)
    return names


# ---------------------------------------------------------------------------
# Study scanner
# ---------------------------------------------------------------------------

def scan_all_studies():
    """Return list of dicts, one per case-study directory."""
    cmake_registered = _cmake_registered_studies()
    results = []

    for entry in sorted(os.listdir(CASE_DIR)):
        study_dir = os.path.join(CASE_DIR, entry)
        if not os.path.isdir(study_dir):
            continue

        has_main_py = os.path.exists(os.path.join(study_dir, "sim", "main.py"))
        has_main_cpp = os.path.exists(os.path.join(study_dir, "sim", "src", "main.cpp"))
        has_readme = os.path.exists(os.path.join(study_dir, "README.md"))
        is_cpp = entry in cmake_registered
        is_python = has_main_py and not is_cpp
        is_stub = not is_cpp and not is_python

        study = {
            "name": entry,
            "type": "C++" if is_cpp else ("Python" if is_python else "stub"),
            "has_readme": has_readme,
            "has_sim": has_main_py or has_main_cpp,
            "ctrl_count": None,
            "ctrl_source": "N/A",
            "scen_count": None,
            "scen_source": "N/A",
            "runs": None,
        }

        if is_cpp:
            study["ctrl_count"], study["ctrl_source"] = _cpp_controller_count(study_dir)
            study["scen_count"], study["scen_source"] = _cpp_scenario_count(study_dir)
        elif is_python:
            study["ctrl_count"], study["ctrl_source"] = _python_controller_count(study_dir)
            study["scen_count"], study["scen_source"] = _python_scenario_count(study_dir)

        if study["ctrl_count"] and study["scen_count"]:
            study["runs"] = study["ctrl_count"] * study["scen_count"]

        results.append(study)

    return results


# ---------------------------------------------------------------------------
# CASE_STUDIES.md parser — extract documented counts per study
# ---------------------------------------------------------------------------

_CTRL_ROW  = re.compile(r'\|\s*\*\*Controllers\*\*\s*\|\s*(\d+)\s*\|')
_SCEN_ROW  = re.compile(r'\|\s*\*\*Scenarios\*\*\s*\|\s*(\d+)\s*\|')
_RUNS_ROW  = re.compile(r'\|\s*\*\*Total runs\*\*\s*\|\s*[\d,×x*]+\s*\((\d+)[×x*](\d+)\)\s*\|')
_STUDY_HDR = re.compile(r'^###\s+(.+?)\s+(?:✅|🔲)')


def _parse_documented_counts(md_path):
    """Parse CASE_STUDIES.md and return {study_name: {ctrl, scen, runs}} for documented studies."""
    docs = {}
    if not os.path.exists(md_path):
        return docs

    current_study = None
    with open(md_path, encoding="utf-8") as f:
        for line in f:
            m = _STUDY_HDR.match(line)
            if m:
                current_study = m.group(1).strip()
                docs[current_study] = {}
                continue
            if current_study:
                mc = _CTRL_ROW.search(line)
                if mc:
                    docs[current_study]["ctrl"] = int(mc.group(1))
                ms = _SCEN_ROW.search(line)
                if ms:
                    docs[current_study]["scen"] = int(ms.group(1))
                mr = _RUNS_ROW.search(line)
                if mr:
                    docs[current_study]["runs"] = int(mr.group(1)) * int(mr.group(2))
    return docs


# ---------------------------------------------------------------------------
# Patch mode: update count rows in existing CASE_STUDIES.md
# ---------------------------------------------------------------------------

def _patch_case_studies_md(studies):
    """Patch count fields in existing CASE_STUDIES.md, preserving all other content."""
    if not os.path.exists(CASE_STUDIES_MD):
        print("ERROR: docs/CASE_STUDIES.md not found. Use --generate to create it.")
        return False

    with open(CASE_STUDIES_MD, encoding="utf-8") as f:
        lines = f.readlines()

    # Build name→counts map from scanned studies (implemented only)
    counts = {}
    for s in studies:
        if s["type"] in ("C++", "Python") and s["ctrl_count"] and s["scen_count"]:
            counts[s["name"]] = s

    current_study = None
    patched_lines = []
    changed = 0

    for line in lines:
        m = _STUDY_HDR.match(line)
        if m:
            current_study = m.group(1).strip()

        if current_study and current_study in counts:
            info = counts[current_study]
            ctrl, scen, runs = info["ctrl_count"], info["scen_count"], info["runs"]

            mc = _CTRL_ROW.match(line.strip())
            if mc and int(mc.group(1)) != ctrl:
                line = f"| **Controllers** | {ctrl} |\n"
                changed += 1

            ms = _SCEN_ROW.match(line.strip())
            if ms and int(ms.group(1)) != scen:
                line = f"| **Scenarios** | {scen} |\n"
                changed += 1

            mr = _RUNS_ROW.search(line)
            if mr:
                old_ctrl = int(mr.group(1))
                old_scen = int(mr.group(2))
                if old_ctrl != ctrl or old_scen != scen:
                    sep = "×"
                    line = f"| **Total runs** | {runs} ({ctrl}{sep}{scen}) |\n"
                    changed += 1

        patched_lines.append(line)

    if changed:
        with open(CASE_STUDIES_MD, "w", encoding="utf-8") as f:
            f.writelines(patched_lines)
        print(f"Patched {changed} count field(s) in docs/CASE_STUDIES.md.")
    else:
        print("All count fields in docs/CASE_STUDIES.md already match — no changes needed.")
    return True


# ---------------------------------------------------------------------------
# Generate mode: full rebuild of CASE_STUDIES.md from scan results
# ---------------------------------------------------------------------------

def _generate_case_studies_md(studies):
    """Write a fresh docs/CASE_STUDIES.md. Loses hand-written caveats."""
    sim_targets = _sim_targets_from_compile_bat()
    cpp = [s for s in studies if s["type"] == "C++"]
    py  = [s for s in studies if s["type"] == "Python"]
    stub = [s for s in studies if s["type"] == "stub"]

    lines = [
        "# Case Studies — Implementation Tracker",
        "",
        "> Auto-generated by `tools/gen_case_studies_md.py`. Count fields are",
        "> extracted from source files. For per-study caveats and plant details",
        "> keep this file checked-in and run `--patch` when counts change.",
        "",
        "---",
        "",
        "## Status Legend",
        "",
        "| Symbol | Meaning |",
        "|--------|---------|",
        "| ✅ C++ | Built executable, registered in `CMakeLists.txt` + `compile.bat`, runs in Phase 4 |",
        "| ✅ Python | `sim/main.py` present, runs in Phase 6 via conda |",
        "| 🔲 Stub | `README.md` spec only — `sim/` not present, not registered, not built |",
        "",
        "---",
        "",
        f"## C++ Built Studies ({len(cpp)})",
        "",
    ]

    for s in cpp:
        ctrl  = s["ctrl_count"] or "?"
        scen  = s["scen_count"] or "?"
        runs  = s["runs"]       or "?"
        # Guess build target from compile.bat: match first *_sim target that overlaps name tokens
        name_tokens = {t.lower() for t in re.split(r'\W+', s["name"]) if len(t) > 3}
        target = next(
            (t for t in sim_targets if any(tok in t for tok in name_tokens)),
            "???"
        )
        lines += [
            f"",
            f"### {s['name']} ✅ C++",
            f"",
            f"| Field | Value |",
            f"|-------|-------|",
            f"| **Plant** | *(see README.md)* |",
            f"| **Reference** | *(see README.md)* |",
            f"| **Controllers** | {ctrl} |",
            f"| **Scenarios** | {scen} |",
            f"| **Total runs** | {runs} ({ctrl}×{scen}) |",
            f"| **Build target** | `{target}` |",
            f"| **Logs** | `case-study/{s['name']}/logs/` |",
            f"",
            f"---",
        ]

    lines += [
        "",
        f"## Python-Only Studies ({len(py)})",
        "",
        "Discovered by `run.py` Phase 6 via `case-study/*/sim/main.py`.",
        "Not in `CMakeLists.txt` or `compile.bat`.",
        "",
    ]

    for s in py:
        ctrl = s["ctrl_count"] or "?"
        scen = s["scen_count"] or "?"
        runs = s["runs"]       or "?"
        lines += [
            f"",
            f"### {s['name']} ✅ Python",
            f"",
            f"| Field | Value |",
            f"|-------|-------|",
            f"| **Plant** | *(see README.md)* |",
            f"| **Integration** | *(see README.md)* |",
            f"| **Controllers** | {ctrl} |",
            f"| **Scenarios** | {scen} |",
            f"| **Total runs** | {runs} ({ctrl}×{scen}) |",
            f"| **Entry point** | `case-study/{s['name']}/sim/main.py` |",
            f"| **Logs** | `case-study/{s['name']}/logs/` |",
            f"",
            f"---",
        ]

    lines += [
        "",
        f"## Spec-Only Stubs ({len(stub)})",
        "",
        "No `sim/` directory. README (or PDF/TXT) present as spec.",
        "",
    ]

    for s in stub:
        readme_note = "README present" if s["has_readme"] else "PDF/TXT only, no README"
        lines += [
            f"",
            f"### {s['name']} 🔲 Stub",
            f"",
            f"- **Spec:** {readme_note}",
            f"",
            f"---",
        ]

    with open(CASE_STUDIES_MD, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Generated docs/CASE_STUDIES.md  ({len(cpp)} C++ + {len(py)} Python + {len(stub)} stubs).")


# ---------------------------------------------------------------------------
# Default mode: compare ground-truth counts with documented counts
# ---------------------------------------------------------------------------

def _check_and_report(studies):
    """Print summary table and highlight any discrepancies with CASE_STUDIES.md."""
    docs = _parse_documented_counts(CASE_STUDIES_MD)

    print("\n=== Case Study Ground-Truth Counts (from source files) ===\n")
    print(f"{'Type':<8} {'Study':<62} {'Ctrl':>5} {'Scen':>5} {'Runs':>6}")
    print("-" * 90)

    issues = []
    total_runs = 0

    for s in studies:
        if s["type"] == "stub":
            readme = "README ✓" if s["has_readme"] else "no README"
            print(f"{'stub':<8} {s['name']:<62} [{readme}]")
            continue

        ctrl  = s["ctrl_count"]
        scen  = s["scen_count"]
        runs  = s["runs"]
        label = f"{ctrl}×{scen}={runs}" if ctrl and scen else "?"
        print(f"{s['type']:<8} {s['name']:<62} {str(ctrl):>5} {str(scen):>5} {str(runs):>6}")
        if runs:
            total_runs += runs

        # Compare with documented counts
        doc = docs.get(s["name"], {})
        if doc:
            if "ctrl" in doc and doc["ctrl"] != ctrl:
                issues.append(
                    f"  CTRL MISMATCH  {s['name']}: "
                    f"code={ctrl}, CASE_STUDIES.md={doc['ctrl']}"
                )
            if "scen" in doc and doc["scen"] != scen:
                issues.append(
                    f"  SCEN MISMATCH  {s['name']}: "
                    f"code={scen}, CASE_STUDIES.md={doc['scen']}"
                )
            if "runs" in doc and runs and doc["runs"] != runs:
                issues.append(
                    f"  RUNS MISMATCH  {s['name']}: "
                    f"code={runs}, CASE_STUDIES.md={doc['runs']}"
                )
        else:
            if s["type"] in ("C++", "Python"):
                issues.append(
                    f"  NOT IN DOCS    {s['name']} ({s['type']}) — "
                    f"not found in CASE_STUDIES.md"
                )

    print("-" * 90)
    cpp_count  = sum(1 for s in studies if s["type"] == "C++")
    py_count   = sum(1 for s in studies if s["type"] == "Python")
    stub_count = sum(1 for s in studies if s["type"] == "stub")
    print(
        f"Totals: {cpp_count} C++  {py_count} Python  {stub_count} stubs  "
        f"— {total_runs} total simulation runs"
    )

    if issues:
        print(f"\n{'='*60}")
        print(f"DISCREPANCIES FOUND ({len(issues)}) — run --patch to fix:")
        for iss in issues:
            print(iss)
        print('='*60)
        return False
    else:
        print("\nAll documented counts match source files. ✓")
        return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--patch",    action="store_true",
                        help="Patch count fields in docs/CASE_STUDIES.md in-place")
    parser.add_argument("--generate", action="store_true",
                        help="Full overwrite of docs/CASE_STUDIES.md (loses hand-written caveats)")
    parser.add_argument("--ci",       action="store_true",
                        help="Exit 1 if any discrepancy found (for use in CI)")
    args = parser.parse_args()

    studies = scan_all_studies()

    if args.generate:
        _generate_case_studies_md(studies)
    elif args.patch:
        _patch_case_studies_md(studies)
        _check_and_report(studies)
    else:
        ok = _check_and_report(studies)
        if args.ci and not ok:
            sys.exit(1)


if __name__ == "__main__":
    main()
