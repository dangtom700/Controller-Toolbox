"""
ex48_fopdt_identification.py
First-Order Plus Dead-Time (FOPDT) step-response identification.

Demonstrates the FOPDTIdentifier algorithm (lib/FOPDTIdentifier.h) using a
pure-Python implementation that mirrors the C++ graphical + optimization methods.
Validates against the expected K, tau, theta for a synthetic response.

Reference: pdc-master/Module_06/1_Topic/FirstOrderPlusDeadTime_1.py
"""

import numpy as np
from scipy.optimize import minimize_scalar, minimize

# ---------------------------------------------------------------------------
# FOPDT model and simulator
# ---------------------------------------------------------------------------

def fopdt_step(t, K, tau, theta, step_mag=1.0, t0=0.0):
    """Evaluate FOPDT step response y(t) = K*step*(1-exp(-(t-t0-theta)/tau))."""
    dt = t - t0 - theta
    return np.where(dt > 0.0, K * step_mag * (1.0 - np.exp(-dt / tau)), 0.0)


# ---------------------------------------------------------------------------
# Graphical identification (PDC Module 06 method)
# ---------------------------------------------------------------------------

def identify_graphical(t, y, step_mag):
    """Identify (K, tau, theta) using the Ziegler-Nichols tangent method."""
    y0   = y[0]
    yinf = y[-1]
    K    = (yinf - y0) / step_mag

    y_norm = y - y0  # zero-referenced

    # Steepest tangent (max slope via centred finite differences)
    slopes = np.gradient(y_norm, t)
    idx_max = int(np.argmax(slopes))
    slope_max = slopes[idx_max]

    # Tangent intercept with y=0
    theta = t[idx_max] - y_norm[idx_max] / max(slope_max, 1e-12)
    theta = max(theta, 0.0)

    # Time to 63.2% of final value
    y632 = 0.632 * (yinf - y0)
    t632 = t[-1]
    for i in range(len(t) - 1):
        if y_norm[i] <= y632 <= y_norm[i + 1]:
            frac = (y632 - y_norm[i]) / (y_norm[i+1] - y_norm[i] + 1e-30)
            t632 = t[i] + frac * (t[i+1] - t[i])
            break

    tau = max(t632 - theta, (t[1] - t[0]))
    return K, tau, theta


# ---------------------------------------------------------------------------
# Optimization identification (golden-section / scipy minimize)
# ---------------------------------------------------------------------------

def identify_optimization(t, y, step_mag):
    """Identify (K, tau, theta) by minimising SSE via nested golden-section."""
    y0   = y[0]
    yinf = y[-1]
    K    = (yinf - y0) / step_mag
    T    = t[-1]
    Ts   = t[1] - t[0]

    def sse(theta, tau):
        y_hat = fopdt_step(t, K, tau, theta, step_mag)
        return float(np.sum((y - y0 - y_hat) ** 2))

    def best_tau_sse(theta):
        res = minimize_scalar(lambda tau: sse(theta, tau),
                              bounds=(Ts, max(T - theta, 2*Ts)),
                              method='bounded')
        return res.x, res.fun

    res_th = minimize_scalar(lambda th: best_tau_sse(th)[1],
                             bounds=(0.0, 0.5 * T), method='bounded')
    opt_theta = res_th.x
    opt_tau, _ = best_tau_sse(opt_theta)
    rmse = np.sqrt(sse(opt_theta, opt_tau) / len(t))
    return K, opt_tau, opt_theta, rmse


# ---------------------------------------------------------------------------
# IMC-PID tuning (Rivera 1986)
# ---------------------------------------------------------------------------

def imc_tuning(K, tau, theta, lambda_c, pi_only=False):
    """Return PID parameters from IMC tuning rules."""
    Ti = tau + 0.5 * theta
    Td = 0.0 if pi_only else (tau * theta) / (2.0 * tau + theta)
    Kp = Ti / (K * (lambda_c + 0.5 * theta))
    Ki = Kp / Ti if Ti > 1e-12 else 0.0
    Kd = Kp * Td
    return {"Kp": Kp, "Ki": Ki, "Kd": Kd, "Ti": Ti, "Td": Td}


# ---------------------------------------------------------------------------
# Main validation
# ---------------------------------------------------------------------------

def main():
    # True FOPDT parameters
    K_true, tau_true, theta_true = 2.0, 5.0, 1.5
    step_mag = 0.5
    Ts_data  = 0.1

    t = np.arange(0, 30.0, Ts_data)
    y = fopdt_step(t, K_true, tau_true, theta_true, step_mag)

    # --- Graphical ---
    Kg, taug, thetag = identify_graphical(t, y, step_mag)
    assert abs(Kg - K_true) < 0.1,        f"Graphical K error: {Kg:.4f} vs {K_true}"
    assert abs(taug - tau_true) < 1.0,    f"Graphical tau error: {taug:.4f} vs {tau_true}"
    assert abs(thetag - theta_true) < 0.5,f"Graphical theta error: {thetag:.4f} vs {theta_true}"
    print(f"Graphical:     K={Kg:.3f}  tau={taug:.3f}  theta={thetag:.3f}")

    # --- Optimization ---
    Ko, tauo, thetao, rmse = identify_optimization(t, y, step_mag)
    assert abs(Ko - K_true) < 0.05,       f"Opt K error: {Ko:.4f} vs {K_true}"
    assert abs(tauo - tau_true) < 0.5,    f"Opt tau error: {tauo:.4f} vs {tau_true}"
    assert abs(thetao - theta_true) < 0.3,f"Opt theta error: {thetao:.4f} vs {theta_true}"
    assert rmse < 0.01,                   f"Opt RMSE too large: {rmse:.6f}"
    print(f"Optimization:  K={Ko:.3f}  tau={tauo:.3f}  theta={thetao:.3f}  RMSE={rmse:.6f}")

    # --- IMC-PID tuning ---
    pp = imc_tuning(Ko, tauo, thetao, lambda_c=2.0 * thetao)
    assert pp["Kp"] > 0, "IMC Kp must be positive"
    assert pp["Ki"] > 0, "IMC Ki must be positive"
    print(f"IMC-PID:       Kp={pp['Kp']:.4f}  Ti={pp['Ti']:.4f} s  Td={pp['Td']:.4f} s")

    # --- Validate IMC-PID with a quick closed-loop simulation ---
    Kp, Ki = pp["Kp"], pp["Ki"]
    dt = Ts_data
    # Discretise PI in Euler form (no dead-time in sim for simplicity)
    y_sim, integral, ref = 0.0, 0.0, 1.0
    for _ in range(2000):
        e = ref - y_sim
        integral += e * dt
        u = Kp * e + Ki * integral
        # First-order plant (no dead-time): y+ = exp(-dt/tau)*y + (1-exp(-dt/tau))*K*u
        a = np.exp(-dt / K_true)  # using K_true as time constant proxy... actually use tau_true
        a = np.exp(-dt / tau_true)
        y_sim = a * y_sim + (1.0 - a) * K_true * u
    assert abs(y_sim - ref) < 0.05, f"IMC closed-loop did not converge: y={y_sim:.4f}"
    print(f"IMC closed-loop final y = {y_sim:.4f}  (ref=1.0)")

    print("PASS")


if __name__ == "__main__":
    main()
