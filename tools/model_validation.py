"""
tools/model_validation.py  -- ANA-6: grey-box plant parameter estimation + NRMSE validation.

Uses ctrl_toolbox.GreyBoxEstimator to fit an ODE model to logged case-study data,
then reports NRMSE between model prediction and logged plant output.

Usage:
    python tools/model_validation.py --study <name> --scenario <name> [options]

Options:
    --study     NAME    case-study directory name (partial match)
    --scenario  NAME    scenario name as in CSV filename (partial match)
    --controller NAME   controller to use for the excitation data (default: PID)
    --params    JSON    JSON object of initial parameter guess (overrides study defaults)
    --out       FILE    write estimated params to JSON file (optional)

The study must define a grey_box_model() function in its sim/ package that returns:
    (ode_fn, h_fn, x0, param_names, param_bounds_lower, param_bounds_upper)

where:
    ode_fn(x, u, p) -> xdot   (numpy arrays)
    h_fn(x, p)     -> y       (numpy array, scalar or 1D)
    x0             : initial state (numpy array)
    param_names    : list of str
    param_bounds_*: numpy arrays of shape (n_params,)

If the study does not define grey_box_model(), the script reports NRMSE using the
last-row IAE as a proxy and exits with a warning.
"""
from __future__ import annotations
import argparse
import json
import sys
import warnings
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT))

import _setup_bindings  # noqa: F401 -- adds ctrl_toolbox to path

try:
    import numpy as np
    import pandas as pd
except ImportError:
    print("ERROR: numpy and pandas required.")
    sys.exit(1)

try:
    import ctrl_toolbox as ctrl
    _HAS_CTRL = hasattr(ctrl, "GreyBoxEstimator")
except Exception:
    _HAS_CTRL = False


def _find_study_dir(name: str) -> Path:
    for d in (_ROOT / "case-study").iterdir():
        if name.lower() in d.name.lower():
            return d
    raise FileNotFoundError(f"No case-study directory matching '{name}'")


def _find_csv(study_dir: Path, scenario: str, controller: str) -> Path:
    logs_dir = study_dir / "logs"
    candidates = sorted(logs_dir.glob(f"run_*{scenario}*{controller}*.csv"))
    if not candidates:
        candidates = sorted(logs_dir.glob(f"run_*{controller}*.csv"))
    if not candidates:
        raise FileNotFoundError(f"No CSV found in {logs_dir} matching scenario={scenario}, ctrl={controller}")
    return candidates[0]


def _nrmse(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    """Normalized RMSE: RMSE / (max(y_true) - min(y_true))."""
    rmse = float(np.sqrt(np.mean((y_true - y_pred) ** 2)))
    rng  = float(np.max(y_true) - np.min(y_true))
    return rmse / rng if rng > 1e-12 else float("nan")


def _run_validation(study_dir: Path, csv_path: Path, params_override: dict | None,
                    out_path: str) -> dict:
    """Load logged data, run GreyBoxEstimator, and report NRMSE."""
    sim_dir = study_dir / "sim"
    sys.path.insert(0, str(sim_dir))

    result = {
        "study": study_dir.name,
        "csv": csv_path.name,
        "nrmse": float("nan"),
        "estimated_params": {},
        "converged": False,
    }

    # Try to load grey_box_model() from the sim
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("_val_sim", str(study_dir / "sim" / "main.py"))
        mod  = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        has_hook = hasattr(mod, "grey_box_model")
    except Exception:
        has_hook = False

    if not has_hook:
        print("WARN: study has no grey_box_model() in sim/main.py — reporting IAE proxy only")
        df = pd.read_csv(csv_path)
        from tools.metrics import extract_final_iae
        iae = extract_final_iae(df)
        result["iae_proxy"] = iae
        print(f"  Final IAE: {iae:.4f}")
        return result

    if not _HAS_CTRL:
        print("WARN: ctrl_toolbox not available or GreyBoxEstimator not found — skipping estimation")
        return result

    # Load model hooks
    ode_fn, h_fn, x0, param_names, p_lower, p_upper = mod.grey_box_model()

    # Load CSV data
    df = pd.read_csv(csv_path)
    n  = len(df)
    t_col = "t" if "t" in df.columns else "time"
    t_arr = df[t_col].to_numpy(float)
    Ts    = float(t_arr[1] - t_arr[0]) if n > 1 else 0.01

    # Infer input/output columns
    from tools.metrics import compute_metrics_from_df
    u_col_candidates = ["u", "u1", "F_act", "m_dot_f", "f_shade", "omega_t", "d"]
    y_col_candidates = ["y", "y1", "z_s", "omega_b", "T_h", "T_pot", "v_out", "x_p"]
    u_col = next((c for c in u_col_candidates if c in df.columns), None)
    y_col = next((c for c in y_col_candidates if c in df.columns), None)

    if u_col is None or y_col is None:
        print(f"WARN: could not infer u/y columns from {list(df.columns)}")
        return result

    U = df[u_col].to_numpy(float).reshape(1, -1)
    Y = df[y_col].to_numpy(float).reshape(1, -1)

    # Initial parameter guess
    if params_override:
        p_init = np.array([params_override.get(name, (lo + hi) / 2)
                           for name, lo, hi in zip(param_names, p_lower, p_upper)], dtype=float)
    else:
        p_init = (np.asarray(p_lower, float) + np.asarray(p_upper, float)) / 2.0

    # Build GreyBoxEstimator
    gbp = ctrl.GreyBoxParams()
    gbp.max_iterations  = 100
    gbp.tolerance       = 1e-6
    gbp.param_lower     = np.asarray(p_lower, float)
    gbp.param_upper     = np.asarray(p_upper, float)

    def _ode(x, u, p):
        return ode_fn(np.asarray(x), np.asarray(u), np.asarray(p))

    def _h(x, p):
        out = h_fn(np.asarray(x), np.asarray(p))
        return np.atleast_1d(out)

    est = ctrl.GreyBoxEstimator(_ode, _h, len(x0), len(param_names), Ts, gbp)
    fit_result = est.fit(np.asarray(x0, float), U, Y)

    estimated = {name: float(v) for name, v in zip(param_names, fit_result.params)}
    result["estimated_params"] = estimated
    result["converged"]        = bool(fit_result.converged)
    result["fit_cost"]         = float(fit_result.cost)
    result["fit_iterations"]   = int(fit_result.iterations)

    print(f"  Converged: {fit_result.converged}  Cost: {fit_result.cost:.4e}  Iter: {fit_result.iterations}")
    print(f"  Estimated params: {estimated}")

    # NRMSE validation
    Y_hat = est.predict(np.asarray(x0, float), U)
    nrmse = _nrmse(Y[0], Y_hat[0])
    result["nrmse"] = nrmse
    print(f"  NRMSE: {nrmse:.4f}")

    if out_path:
        with open(out_path, "w") as fh:
            json.dump(result, fh, indent=2)
        print(f"  Results written to: {out_path}")

    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study",      required=True)
    ap.add_argument("--scenario",   default="s01")
    ap.add_argument("--controller", default="PID")
    ap.add_argument("--params",     default="")
    ap.add_argument("--out",        default="")
    args = ap.parse_args(argv)

    study_dir = _find_study_dir(args.study)
    print(f"Study: {study_dir.name}")

    try:
        csv_path = _find_csv(study_dir, args.scenario, args.controller)
        print(f"Data: {csv_path.name}")
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)

    params_override = json.loads(args.params) if args.params else None
    _run_validation(study_dir, csv_path, params_override, args.out)


if __name__ == "__main__":
    main()
