"""
ex50_sopdt_identification.py
Second-Order Plus Dead-Time (SOPDT) step-response identification.

Pure-Python mirror of lib/SOPDTIdentifier. Identifies (K, tau1, tau2, theta)
from synthetic step data using both graphical and optimization methods.

Reference: pdc-master/Module_07/1_Topic/FirstOrderOptimization_*.py
"""

import numpy as np
from scipy.optimize import minimize_scalar


# ---------------------------------------------------------------------------
# SOPDT model
# ---------------------------------------------------------------------------

def sopdt_step(t, K, tau1, tau2, theta, step_mag=1.0):
    """Evaluate SOPDT step response (overdamped or critically damped)."""
    dt = t - theta
    mask = dt > 0.0
    y = np.zeros_like(t, dtype=float)
    if np.abs(tau1 - tau2) < 1e-9 * (tau1 + tau2 + 1e-30):
        # Critically damped
        tau = 0.5 * (tau1 + tau2)
        y[mask] = K * step_mag * (1.0 - (1.0 + dt[mask] / tau) * np.exp(-dt[mask] / tau))
    else:
        y[mask] = K * step_mag * (
            1.0 - (tau1 * np.exp(-dt[mask] / tau1)
                 - tau2 * np.exp(-dt[mask] / tau2)) / (tau1 - tau2)
        )
    return y


# ---------------------------------------------------------------------------
# Graphical identification
# ---------------------------------------------------------------------------

def identify_graphical(t, y, step_mag):
    y0   = y[0]
    yinf = y[-1]
    K    = (yinf - y0) / step_mag
    y_norm = y - y0

    # ZN tangent -> theta
    slopes = np.gradient(y_norm, t)
    idx_max = int(np.argmax(slopes))
    slope_max = slopes[idx_max]
    theta = max(0.0, t[idx_max] - y_norm[idx_max] / max(slope_max, 1e-12))

    # 63.2% crossing -> tau_sum
    y632 = 0.632 * (yinf - y0)
    t632 = t[-1]
    for i in range(len(t) - 1):
        if y_norm[i] <= y632 <= y_norm[i + 1]:
            frac = (y632 - y_norm[i]) / (y_norm[i + 1] - y_norm[i] + 1e-30)
            t632 = t[i] + frac * (t[i + 1] - t[i])
            break
    tau_sum = max(t632 - theta, 2.0 * (t[1] - t[0]))

    # 28.3% crossing -> split estimate
    y283 = 0.283 * (yinf - y0)
    t283 = 0.5 * t632
    for i in range(len(t) - 1):
        if y_norm[i] <= y283 <= y_norm[i + 1]:
            frac = (y283 - y_norm[i]) / (y_norm[i + 1] - y_norm[i] + 1e-30)
            t283 = t[i] + frac * (t[i + 1] - t[i])
            break

    r = max(0.0, (t283 - theta) / (tau_sum + 1e-30))
    alpha = 0.5 * np.clip((r - 0.530) / (0.332 - 0.530 + 1e-10), 0.0, 0.45)
    tau1 = tau_sum * (0.5 + alpha)
    tau2 = max(tau_sum * (0.5 - alpha), t[1] - t[0])
    return K, tau1, tau2, theta


# ---------------------------------------------------------------------------
# Optimization identification
# ---------------------------------------------------------------------------

def identify_optimization(t, y, step_mag):
    y0   = y[0]
    yinf = y[-1]
    K    = (yinf - y0) / step_mag
    T    = t[-1]
    Ts   = t[1] - t[0]

    def sse(K_n, tau1, tau2, theta):
        y_hat = sopdt_step(t, K_n, tau1, tau2, theta, step_mag)
        return float(np.sum((y - y0 - y_hat) ** 2))

    def best_tau2_sse(theta, tau1):
        res = minimize_scalar(
            lambda tau2: sse(K, tau1, max(tau2, Ts), theta),
            bounds=(Ts, max(tau1, Ts + 1e-9)), method='bounded')
        tau2_opt = float(np.squeeze(res.x))
        return tau2_opt, res.fun

    def best_tau1_sse(theta):
        res = minimize_scalar(
            lambda tau1: best_tau2_sse(theta, tau1)[1],
            bounds=(Ts, max(T - theta, 2 * Ts)), method='bounded')
        tau1_opt = float(np.squeeze(res.x))
        tau2_opt, sse_val = best_tau2_sse(theta, tau1_opt)
        return tau1_opt, tau2_opt, sse_val

    res_th = minimize_scalar(
        lambda th: best_tau1_sse(th)[2],
        bounds=(0.0, 0.5 * T), method='bounded')
    theta_opt = float(np.squeeze(res_th.x))
    tau1_opt, tau2_opt, sse_val = best_tau1_sse(theta_opt)

    if tau1_opt < tau2_opt:
        tau1_opt, tau2_opt = tau2_opt, tau1_opt

    rmse = float(np.sqrt(sse_val / len(t)))
    return K, tau1_opt, tau2_opt, theta_opt, rmse


# ---------------------------------------------------------------------------
# IMC-PID tuning for SOPDT
# ---------------------------------------------------------------------------

def imc_tuning_sopdt(K, tau1, tau2, theta, lambda_c, pi_only=False):
    tau_eq = tau1 + tau2
    Ti = tau_eq
    Td = 0.0 if pi_only else (tau1 * tau2) / tau_eq
    Kp = tau_eq / (K * (lambda_c + 0.5 * theta))
    Ki = Kp / Ti if Ti > 1e-12 else 0.0
    Kd = Kp * Td
    return {"Kp": Kp, "Ki": Ki, "Kd": Kd, "Ti": Ti, "Td": Td}


# ---------------------------------------------------------------------------
# Main validation
# ---------------------------------------------------------------------------

def main():
    K_true, tau1_true, tau2_true, theta_true = 2.0, 5.0, 2.0, 1.5
    step_mag = 0.5
    Ts_data  = 0.25

    t = np.arange(0, 50.0, Ts_data)
    y = sopdt_step(t, K_true, tau1_true, tau2_true, theta_true, step_mag)

    # Graphical
    Kg, tau1g, tau2g, thg = identify_graphical(t, y, step_mag)
    print(f"Graphical: K={Kg:.3f} tau1={tau1g:.3f} tau2={tau2g:.3f} theta={thg:.3f}")
    assert abs(Kg - K_true) < 0.2,        f"Graphical K error: {Kg:.4f}"
    assert tau1g + tau2g > 2.0,            "tau_sum must be > 2"
    assert tau1g >= tau2g,                 "tau1 must be dominant"

    # Optimization
    Ko, tau1o, tau2o, tho, rmse = identify_optimization(t, y, step_mag)
    print(f"Optimization: K={Ko:.3f} tau1={tau1o:.3f} tau2={tau2o:.3f} theta={tho:.3f} RMSE={rmse:.6f}")
    assert abs(Ko - K_true) < 0.05,              f"Opt K error: {Ko:.4f}"
    assert abs(tau1o + tau2o - (tau1_true + tau2_true)) < 1.0, \
        f"Opt tau_sum error: {tau1o+tau2o:.3f} vs {tau1_true+tau2_true}"
    assert abs(tho - theta_true) < 0.5,          f"Opt theta error: {tho:.4f}"
    assert rmse < 0.01,                           f"Opt RMSE too large: {rmse:.6f}"

    # IMC-PID
    pp = imc_tuning_sopdt(Ko, tau1o, tau2o, tho, 2.0 * tho)
    print(f"IMC-PID: Kp={pp['Kp']:.4f} Ti={pp['Ti']:.4f} Td={pp['Td']:.4f}")
    assert pp["Kp"] > 0
    assert pp["Ki"] > 0

    # Closed-loop validation
    tauEq = tau1o + tau2o
    y_sim, integ, ref = 0.0, 0.0, 1.0
    dt = Ts_data
    Kp, Ki = pp["Kp"], pp["Ki"]
    for _ in range(3000):
        e = ref - y_sim
        integ += e * dt
        u = Kp * e + Ki * integ
        a = np.exp(-dt / tauEq)
        y_sim = a * y_sim + (1.0 - a) * Ko * u
    assert abs(y_sim - ref) < 0.05, f"Closed-loop did not converge: y={y_sim:.4f}"
    print(f"Closed-loop final y = {y_sim:.4f}  (ref=1.0)")

    print("PASS")


if __name__ == "__main__":
    main()
