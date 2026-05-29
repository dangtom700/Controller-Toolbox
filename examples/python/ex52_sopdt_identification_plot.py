"""
ex52_sopdt_identification_plot.py
SOPDT identification from noisy step-response data.

Adds realistic measurement noise to the step response and verifies that
the optimization method identifies parameters within acceptable tolerances.
"""

import numpy as np
from scipy.optimize import minimize_scalar


def sopdt_step(t, K, tau1, tau2, theta, step_mag=1.0):
    dt = t - theta
    mask = dt > 0.0
    y = np.zeros_like(t, dtype=float)
    tol = 1e-9 * (tau1 + tau2 + 1e-30)
    if abs(tau1 - tau2) < tol:
        tau = 0.5 * (tau1 + tau2)
        y[mask] = K * step_mag * (1.0 - (1.0 + dt[mask] / tau) * np.exp(-dt[mask] / tau))
    else:
        y[mask] = K * step_mag * (
            1.0 - (tau1 * np.exp(-dt[mask] / tau1)
                 - tau2 * np.exp(-dt[mask] / tau2)) / (tau1 - tau2)
        )
    return y


def identify_opt_noisy(t, y_noisy, step_mag):
    y0   = y_noisy[0]
    yinf = np.mean(y_noisy[-10:])  # average last 10 samples for robust final value
    K    = (yinf - y0) / step_mag
    T    = t[-1]
    Ts   = t[1] - t[0]

    def sse(tau1, tau2, theta):
        y_hat = sopdt_step(t, K, tau1, max(tau2, Ts), theta, step_mag)
        return float(np.sum((y_noisy - y0 - y_hat) ** 2))

    def best_tau2(theta, tau1):
        r = minimize_scalar(
            lambda tau2: sse(tau1, tau2, theta),
            bounds=(Ts, max(tau1, Ts + 1e-9)), method='bounded')
        return float(np.squeeze(r.x)), r.fun

    def best_tau1(theta):
        r = minimize_scalar(
            lambda tau1: best_tau2(theta, tau1)[1],
            bounds=(Ts, max(T - theta, 2 * Ts)), method='bounded')
        t1 = float(np.squeeze(r.x))
        t2, s = best_tau2(theta, t1)
        return t1, t2, s

    r_th = minimize_scalar(
        lambda th: best_tau1(th)[2],
        bounds=(0.0, 0.45 * T), method='bounded')
    th = float(np.squeeze(r_th.x))
    t1, t2, sse_val = best_tau1(th)
    if t1 < t2: t1, t2 = t2, t1
    rmse = float(np.sqrt(sse_val / len(t)))
    return K, t1, t2, th, rmse


def main():
    np.random.seed(17)
    K_true, tau1_true, tau2_true, th_true = 1.5, 4.0, 1.5, 1.0
    step_mag = 1.0
    Ts = 0.2
    noise_std = 0.05  # 5% noise relative to final output

    t = np.arange(0, 40.0, Ts)
    y_clean = sopdt_step(t, K_true, tau1_true, tau2_true, th_true, step_mag)
    y_noisy = y_clean + noise_std * np.random.randn(len(t))

    K_id, t1_id, t2_id, th_id, rmse = identify_opt_noisy(t, y_noisy, step_mag)

    print(f"True:  K={K_true} tau1={tau1_true} tau2={tau2_true} theta={th_true}")
    print(f"Ident: K={K_id:.3f} tau1={t1_id:.3f} tau2={t2_id:.3f} theta={th_id:.3f} RMSE={rmse:.4f}")

    # Tolerances are wider than noise-free case (5% noise added)
    assert abs(K_id - K_true) < 0.3,             f"K error: {K_id:.4f}"
    assert abs(t1_id + t2_id - (tau1_true + tau2_true)) < 2.0, \
        f"tau_sum error: {t1_id + t2_id:.3f}"
    assert abs(th_id - th_true) < 1.0,           f"theta error: {th_id:.4f}"
    assert rmse < 0.2,                            f"RMSE too large: {rmse:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
