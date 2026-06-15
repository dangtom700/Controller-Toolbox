"""
tools/mu_analysis.py  -- ANA-7: unstructured mu upper bound (sigma_max) analysis.

For each case-study CSV that has enough column information to construct a frequency-domain
closed-loop transfer function, computes the peak singular value of the sensitivity function
S(z) = (I + L(z))^{-1} and the maximum sigma of the closed-loop T(z) = L(z)(I+L(z))^{-1}.

For the SISO case (the majority of case studies):
    L(z) = P(z) * C(z)   (identified via linear regression on the logged data)

The sigma_max of T (= infinity norm of T) is an upper bound on the unstructured mu.

Usage:
    python tools/mu_analysis.py [options]

Options:
    --study    NAME   filter by study (partial match)
    --scenario NAME   scenario to use (default: s01 or S1)
    --n_freq   INT    number of frequency points (default: 512)
    --out      FILE   write results CSV (default: mu_summary.csv)
    --plot           emit bode/sigma PNG per study

The analysis is approximate: it identifies a discrete-time ARMA(2,2) model from the
closed-loop data using least-squares and wraps it as a transfer function.
"""
from __future__ import annotations
import argparse
import sys
import warnings
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT))

try:
    import numpy as np
    import pandas as pd
except ImportError:
    print("ERROR: numpy and pandas required.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# ARMA identification from closed-loop data
# ---------------------------------------------------------------------------

def _identify_arma(y: np.ndarray, u: np.ndarray, na: int = 2, nb: int = 2) -> tuple:
    """Identify discrete ARMA(na, nb) model y(k) = -a1*y(k-1) - ... + b0*u(k) + ...

    Returns (A_coeffs, B_coeffs) both 1D arrays of length na+1 / nb+1.
    A_coeffs[0] = 1, A_coeffs[1..na] are the denominator AR coefficients.
    B_coeffs[0..nb] are the numerator MA coefficients.
    """
    N = len(y)
    max_lag = max(na, nb)
    rows = []
    targets = []
    for k in range(max_lag, N):
        row = []
        for i in range(1, na + 1):
            row.append(-y[k - i])
        for j in range(nb + 1):
            row.append(u[k - j])
        rows.append(row)
        targets.append(y[k])
    if not rows:
        return np.array([1.0]), np.array([0.0])
    Phi = np.array(rows)
    Theta = np.array(targets)
    try:
        coeffs, _, _, _ = np.linalg.lstsq(Phi, Theta, rcond=None)
    except np.linalg.LinAlgError:
        return np.array([1.0]), np.array([0.0])
    A = np.zeros(na + 1)
    A[0] = 1.0
    A[1:na + 1] = coeffs[:na]
    B = coeffs[na:]
    return A, B


def _tf_frequency_response(B: np.ndarray, A: np.ndarray,
                            freqs_rad_sample: np.ndarray) -> np.ndarray:
    """Evaluate SISO DT transfer function B(z)/A(z) at z = exp(j*omega)."""
    H = np.zeros(len(freqs_rad_sample), dtype=complex)
    for i, w in enumerate(freqs_rad_sample):
        z = np.exp(1j * w)
        Bval = sum(B[k] * z ** (-k) for k in range(len(B)))
        Aval = sum(A[k] * z ** (-k) for k in range(len(A)))
        H[i] = Bval / Aval if abs(Aval) > 1e-12 else complex(0)
    return H


def _peak_sigma(L: np.ndarray) -> tuple[float, float]:
    """Return (peak_S, peak_T) where S = 1/(1+L), T = L/(1+L)."""
    S = 1.0 / (1.0 + L + 1e-30)
    T = L / (1.0 + L + 1e-30)
    return float(np.max(np.abs(S))), float(np.max(np.abs(T)))


# ---------------------------------------------------------------------------
# CSV discovery
# ---------------------------------------------------------------------------

def _discover_csvs(root: Path, study_filter: str, scenario_filter: str) -> list[Path]:
    all_csvs = sorted((root / "case-study").rglob("logs/run_*.csv"))
    result = []
    seen_study_scenario: set = set()
    for p in all_csvs:
        study = p.parent.parent.name
        stem  = p.stem[4:] if p.stem.startswith("run_") else p.stem
        parts = stem.split("_")
        controller = parts[-1] if parts else "unknown"
        scenario   = "_".join(parts[:-1])

        if study_filter and study_filter.lower() not in study.lower():
            continue
        if scenario_filter and scenario_filter.lower() not in scenario.lower():
            continue
        # One CSV per (study, scenario): pick the first controller
        key = (study, scenario)
        if key in seen_study_scenario:
            continue
        seen_study_scenario.add(key)
        result.append(p)
    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study",    default="")
    ap.add_argument("--scenario", default="s01")
    ap.add_argument("--n_freq",   type=int, default=512)
    ap.add_argument("--out",      default="mu_summary.csv")
    ap.add_argument("--plot",     action="store_true")
    args = ap.parse_args(argv)

    csvs = _discover_csvs(_ROOT, args.study, args.scenario)
    if not csvs:
        print("No CSVs found matching filters.")
        sys.exit(1)

    print(f"Analysing {len(csvs)} (study, scenario) pairs ...")

    freqs = np.linspace(0, np.pi, args.n_freq, endpoint=False)
    rows = []

    for csv_path in csvs:
        study = csv_path.parent.parent.name
        stem  = csv_path.stem[4:] if csv_path.stem.startswith("run_") else csv_path.stem
        parts = stem.split("_")
        scenario = "_".join(parts[:-1])

        try:
            df = pd.read_csv(csv_path)
        except Exception as exc:
            print(f"  SKIP {csv_path.name}: {exc}")
            continue

        # Pick y and u columns
        cols = list(df.columns)
        from tools.metrics import compute_metrics_from_df  # just for column heuristics
        y_candidates = ["y", "y1", "z_s", "omega_b", "T_h", "T_pot", "v_out", "x_p"]
        u_candidates = ["u", "u1", "F_act", "m_dot_f", "f_shade", "omega_t", "d"]
        e_candidates = ["error", "e1", "e"]
        y_col = next((c for c in y_candidates if c in cols), None)
        u_col = next((c for c in u_candidates if c in cols), None)
        e_col = next((c for c in e_candidates if c in cols), None)

        if (y_col is None or u_col is None) and e_col is None:
            print(f"  SKIP {csv_path.name}: cannot identify y/u columns")
            rows.append({"study": study, "scenario": scenario,
                         "peak_S": float("nan"), "peak_T": float("nan"), "status": "no_columns"})
            continue

        if y_col and u_col:
            y = df[y_col].to_numpy(float)
            u = df[u_col].to_numpy(float)
        else:
            # Use error as a proxy for output
            y = df[e_col].to_numpy(float)
            u = np.zeros_like(y)

        # Remove NaN/Inf
        mask = np.isfinite(y) & np.isfinite(u)
        y, u = y[mask], u[mask]

        if len(y) < 10:
            rows.append({"study": study, "scenario": scenario,
                         "peak_S": float("nan"), "peak_T": float("nan"), "status": "too_short"})
            continue

        # Identify ARMA model
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            A, B = _identify_arma(y, u)

        L = _tf_frequency_response(B, A, freqs)
        peak_S, peak_T = _peak_sigma(L)
        rows.append({"study": study, "scenario": scenario,
                     "peak_S": peak_S, "peak_T": peak_T, "status": "ok"})
        print(f"  {study[:30]:30s} {scenario:20s}  peak_S={peak_S:.3f}  peak_T={peak_T:.3f}")

    import csv as csv_mod
    with open(args.out, "w", newline="") as fh:
        w = csv_mod.DictWriter(fh, fieldnames=["study", "scenario", "peak_S", "peak_T", "status"])
        w.writeheader()
        w.writerows(rows)
    print(f"\nMu summary written to: {args.out}  ({len(rows)} rows)")

    if args.plot and rows:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            df_sum = pd.DataFrame(rows).dropna(subset=["peak_T"])
            fig, ax = plt.subplots(figsize=(10, 5))
            ax.barh(range(len(df_sum)), df_sum["peak_T"], color="steelblue")
            ax.axvline(1.0, color="red", linestyle="--", label="||T||_inf = 1 (stability margin)")
            ax.set_yticks(range(len(df_sum)))
            ax.set_yticklabels([f"{r['study'][:25]}/{r['scenario']}" for _, r in df_sum.iterrows()], fontsize=7)
            ax.set_xlabel("Peak |T(z)| (mu upper bound)")
            ax.set_title("Unstructured mu upper bound per study/scenario")
            ax.legend()
            fig.tight_layout()
            plot_path = Path(args.out).with_suffix(".png")
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            print(f"Plot saved: {plot_path}")
        except ImportError:
            print("WARN: matplotlib not found — skipping plot")


if __name__ == "__main__":
    main()
