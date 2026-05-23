"""
Verification utilities for checking data alignment and controller correctness.
"""

import numpy as np
from typing import Optional


def check_alignment(*arrays: np.ndarray, labels: Optional[list[str]] = None) -> bool:
    """
    Verify all arrays have identical length.  Prints a pass/fail summary.
    Returns True if all aligned.
    """
    lengths = [len(a) for a in arrays]
    ok = len(set(lengths)) == 1
    tag = "[PASS]" if ok else "[FAIL]"
    if labels:
        detail = ", ".join(f"{l}={n}" for l, n in zip(labels, lengths))
    else:
        detail = ", ".join(str(n) for n in lengths)
    print(f"  {tag} alignment check - lengths: {detail}")
    return ok


def dc_gain_check(y: np.ndarray, expected: float,
                  tol: float = 0.01, tail_frac: float = 0.1) -> bool:
    """
    Estimate steady-state gain from the last tail_frac fraction of the signal.
    Passes if |mean(y_tail) - expected| / |expected| < tol.
    """
    tail = y[int(len(y) * (1.0 - tail_frac)):]
    measured = float(np.mean(tail))
    if expected == 0.0:
        ok = abs(measured) < tol
    else:
        ok = abs(measured - expected) / abs(expected) < tol
    tag = "[PASS]" if ok else "[FAIL]"
    print(f"  {tag} DC gain - measured={measured:.6f}, expected={expected:.6f}, "
          f"rel_err={abs(measured-expected)/max(abs(expected),1e-12):.4%}")
    return ok


def snr_db(signal: np.ndarray, noisy: np.ndarray) -> float:
    """Signal-to-noise ratio in dB given clean and noisy arrays."""
    noise = noisy - signal
    ps = np.mean(signal**2)
    pn = np.mean(noise**2)
    if pn == 0.0:
        return np.inf
    return 10.0 * np.log10(ps / pn)


def rmse(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    """Root-mean-square error."""
    return float(np.sqrt(np.mean((y_true - y_pred)**2)))


def ise(error: np.ndarray, Ts: float) -> float:
    """Integral of squared error: ISE = Ts * sum(e^2)."""
    return float(Ts * np.sum(error**2))


def itae(error: np.ndarray, Ts: float) -> float:
    """Integral of time-weighted absolute error: ITAE = Ts * sum(t_k * |e_k|)."""
    t = np.arange(len(error)) * Ts
    return float(Ts * np.sum(t * np.abs(error)))


def assert_close(a: float, b: float, tol: float = 1e-6, label: str = "") -> bool:
    """Absolute difference check with pass/fail print."""
    ok = abs(a - b) <= tol
    tag = "[PASS]" if ok else "[FAIL]"
    suffix = f" ({label})" if label else ""
    print(f"  {tag} assert_close{suffix}: |{a:.8g} - {b:.8g}| = {abs(a-b):.3e}  (tol={tol:.3e})")
    return ok


def print_summary(results: dict[str, bool]) -> None:
    """Print a pass/fail summary table from a dict of {label: bool}."""
    passed = sum(v for v in results.values())
    total = len(results)
    print(f"\n{'='*48}")
    print(f"  Verification: {passed}/{total} checks passed")
    print(f"{'='*48}")
    for label, ok in results.items():
        tag = "PASS" if ok else "FAIL"
        print(f"  [{tag}]  {label}")
    print(f"{'='*48}\n")
