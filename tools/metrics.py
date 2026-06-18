"""
tools/metrics.py  -- ANA-1: shared metric computation for all case studies.

Usage:
    from tools.metrics import compute_metrics

    result = compute_metrics(t, y, u, ref)
    # result is a dict with keys: iae, rms_error, settle_time_s,
    #                              overshoot_pct, max_u, energy_var
"""
from __future__ import annotations
import numpy as np


def compute_metrics(
    t: np.ndarray,
    y: np.ndarray,
    u: np.ndarray,
    ref: np.ndarray,
    settle_band: float = 0.02,
    settle_hysteresis: int = 10,
) -> dict[str, float]:
    """Compute a standard set of performance metrics for one controller run.

    Parameters
    ----------
    t          : 1-D time array [s]
    y          : 1-D plant output array (same length as t)
    u          : 1-D control input array (same length as t)
    ref        : 1-D reference/setpoint array (same length as t)
    settle_band: fraction of initial error inside which the output is considered settled
                 (default 0.02 -> 2 %).
    settle_hysteresis: number of consecutive samples that must be inside the band
                       before settling is declared (default 10).

    Returns
    -------
    dict with keys:
        iae          : integral absolute error  int|e|dt
        rms_error    : sqrt(mean(e^2))
        settle_time_s: time [s] when |e| first enters and stays inside settle_band
                       fraction of the initial error magnitude; -1.0 if never settles.
        overshoot_pct: (max(y) - final_ref) / |final_ref| * 100 if y exceeds ref,
                       else 0.0. Returns 0.0 when |final_ref| < 1e-9.
        max_u        : max|u(t)|
        energy_var   : variance of u(t) (actuator wear proxy)
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    u = np.asarray(u, dtype=float)
    ref = np.asarray(ref, dtype=float)

    e = ref - y
    N = len(t)

    # --- IAE ---------------------------------------------------------------
    dt = np.diff(t)
    iae = float(np.sum(np.abs(e[:-1]) * dt)) if N > 1 else 0.0

    # --- RMS error ---------------------------------------------------------
    rms_error = float(np.sqrt(np.mean(e ** 2)))

    # --- Settling time -----------------------------------------------------
    final_ref = float(ref[-1]) if N > 0 else 0.0
    e0_abs = float(np.abs(e[0])) if N > 0 else 0.0
    band_abs = settle_band * e0_abs if e0_abs > 1e-12 else settle_band * max(abs(final_ref), 1e-9)

    settle_time_s = -1.0
    if N >= settle_hysteresis:
        abs_e = np.abs(e)
        in_band = abs_e <= band_abs
        # Find first index where the next `settle_hysteresis` samples are all in band
        for i in range(N - settle_hysteresis + 1):
            if np.all(in_band[i : i + settle_hysteresis]):
                settle_time_s = float(t[i])
                break

    # --- Overshoot ---------------------------------------------------------
    overshoot_pct = 0.0
    if abs(final_ref) > 1e-9:
        if final_ref > 0:
            peak = float(np.max(y))
            if peak > final_ref:
                overshoot_pct = (peak - final_ref) / abs(final_ref) * 100.0
        else:
            peak = float(np.min(y))
            if peak < final_ref:
                overshoot_pct = (final_ref - peak) / abs(final_ref) * 100.0

    # --- Actuator metrics --------------------------------------------------
    max_u = float(np.max(np.abs(u)))
    energy_var = float(np.var(u))

    return {
        "iae": iae,
        "rms_error": rms_error,
        "settle_time_s": settle_time_s,
        "overshoot_pct": overshoot_pct,
        "max_u": max_u,
        "energy_var": energy_var,
    }


def compute_metrics_from_df(df, time_col: str = "time",
                             output_col: str | None = None,
                             input_col: str | None = None,
                             ref_col: str | None = None,
                             **kwargs) -> dict[str, float]:
    """Convenience wrapper: extract columns from a pandas DataFrame and call compute_metrics.

    If column names are not provided, a heuristic scan picks likely candidates:
    - time:   first of ['t', 'time']
    - output: first of ['y', 'y1', 'z_s', 'omega_b', 'T_h', 'v_out', 'x_p', 'T_pot', ...]
    - input:  first of ['u', 'u1', 'F_act', 'f_shade', 'm_dot_f', 'd', ...]
    - ref:    first of ['ref', 'r', 'ref_y1', 'omega_ref', 'T_ref', 'v_ref', 'x_ref', ...]

    Unknown schemas fall back to NaN vectors (metrics return nan).
    """
    cols = list(df.columns)

    def _pick(candidates):
        for c in candidates:
            if c in cols:
                return c
        return None

    t_col = _pick([time_col, 't', 'time'])
    if output_col is None:
        output_col = _pick(['y', 'y1', 'z_s', 'omega_b', 'T_h', 'v_out', 'x_p',
                            'T_pot', 'u_head', 'x_rel', 'z', 'phi'])
    if input_col is None:
        input_col = _pick(['u', 'u1', 'F_act', 'f_shade', 'm_dot_f', 'd', 'F_pto',
                           'omega_t', 'u_ctrl'])
    if ref_col is None:
        ref_col = _pick(['ref', 'r', 'ref_y1', 'omega_ref', 'T_h_ref', 'T_ref',
                         'v_ref', 'x_ref', 'z_r'])

    t = df[t_col].to_numpy(float) if t_col else np.arange(len(df))
    y = df[output_col].to_numpy(float) if output_col else np.full(len(df), float('nan'))
    u = df[input_col].to_numpy(float) if input_col else np.full(len(df), float('nan'))
    ref_arr = df[ref_col].to_numpy(float) if ref_col else np.full(len(df), float('nan'))

    return compute_metrics(t, y, u, ref_arr, **kwargs)


# ---------------------------------------------------------------------------
# IAE column auto-detection for compare_controllers.py
# ---------------------------------------------------------------------------

IAE_COL_CANDIDATES = [
    'iae_cumulative', 'iae', 'IAE_cumulative',
    'IAE_y1', 'IAE_y2', 'IAE_y3',
]

def extract_final_iae(df) -> float:
    """Return the final (last-row) cumulative IAE from a case-study DataFrame.

    For MIMO studies (multiple IAE_y* columns), returns the sum.
    Returns float('nan') if no recognised IAE column is found.
    """
    cols = list(df.columns)
    # Try scalar columns first
    for c in ['iae_cumulative', 'iae', 'IAE_cumulative']:
        if c in cols:
            return float(df[c].iloc[-1])
    # MIMO sum
    mimo_cols = [c for c in cols if c.startswith('IAE_y') or c.startswith('iae_y')]
    if mimo_cols:
        return float(sum(df[c].iloc[-1] for c in mimo_cols))
    # Fallback: try to compute from error column
    e_col = None
    for c in ['error', 'e1', 'e']:
        if c in cols:
            e_col = c
            break
    if e_col:
        t_col = 't' if 't' in cols else ('time' if 'time' in cols else None)
        e_arr = df[e_col].to_numpy(float)
        if t_col:
            t_arr = df[t_col].to_numpy(float)
            dt = np.diff(t_arr)
            return float(np.sum(np.abs(e_arr[:-1]) * dt))
        else:
            return float(np.sum(np.abs(e_arr)))
    return float('nan')
