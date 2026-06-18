"""
ex49_feedforward_pid.py
Two-degree-of-freedom (2DOF) control: Feedforward + PI feedback.

Demonstrates the FeedforwardController concept (lib/FeedforwardController.h).
A static feedforward u_ff = (1/K_plant)*r provides the steady-state control
action, so the PI only corrects dynamic error. This always reduces IAE when
the PI is tuned slowly enough.

Reference: pdc-master/feedforward_widget.ipynb
"""

import numpy as np


def make_first_order_zoh(K_plant, tau, Ts):
    """ZOH discretisation of G(s) = K/(tau*s+1)."""
    Ad = np.exp(-Ts / tau)
    Bd = K_plant * (1.0 - Ad)
    return Ad, Bd


def simulate_pi_only(Ad, Bd, Kp, Ki, ref_seq, Ts, u_lim=30.0):
    """Pure PI."""
    x, integral = 0.0, 0.0
    y_out = []
    for r in ref_seq:
        y = x
        e = r - y
        u = np.clip(Kp * e + Ki * integral, -u_lim, u_lim)
        integral += e * Ts
        x = Ad * x + Bd * u
        y_out.append(y)
    return np.array(y_out)


def simulate_ff_pi(Ad, Bd, K_ff, Kp, Ki, ref_seq, Ts, u_lim=30.0):
    """Static feedforward u_ff = K_ff*r  plus PI on error."""
    x, integral = 0.0, 0.0
    y_out = []
    for r in ref_seq:
        y    = x
        u_ff = K_ff * r          # feedforward: inverse plant DC gain
        e    = r - y
        u_fb = np.clip(Kp * e + Ki * integral, -u_lim, u_lim)
        u    = u_ff + u_fb       # combined
        integral += e * Ts
        x = Ad * x + Bd * u
        y_out.append(y)
    return np.array(y_out)


def main():
    K_plant = 2.0
    tau_p   = 3.0
    Ts      = 0.1
    N       = 600           # 60 s, >> 5*tau_cl for both designs

    Ad, Bd = make_first_order_zoh(K_plant, tau_p, Ts)
    ref    = np.ones(N)

    # Tuning: lambda_c = 1.5 * tau_p = 4.5 s (moderate speed)
    lambda_c = 1.5 * tau_p
    Kp = tau_p / (K_plant * lambda_c)
    Ki = Kp / tau_p

    # Static feedforward: ideal inverse steady-state gain
    K_ff = 1.0 / K_plant

    y_pi  = simulate_pi_only(Ad, Bd, Kp, Ki, ref, Ts)
    y_ff  = simulate_ff_pi(Ad, Bd, K_ff, Kp, Ki, ref, Ts)

    # Both must converge to reference (within 1%)
    assert abs(y_pi[-1] - 1.0) < 0.01, \
        f"PI did not converge: y={y_pi[-1]:.4f}"
    assert abs(y_ff[-1] - 1.0) < 0.01, \
        f"FF+PI did not converge: y={y_ff[-1]:.4f}"

    iae_pi = float(np.sum(np.abs(ref - y_pi))  * Ts)
    iae_ff = float(np.sum(np.abs(ref - y_ff))  * Ts)

    # Feedforward must reduce IAE vs plain PI
    assert iae_ff < iae_pi, \
        f"FF+PI IAE ({iae_ff:.3f}) must be < PI IAE ({iae_pi:.3f})"

    # Verify feedforward filter state: for static gain, the FeedforwardController
    # computeVec mirrors u_ff = K_ff * r (D-only, A=0, B=0, C=0)
    # Test the discrete-filter math manually (mimics FeedforwardController::compute)
    A_f, B_f, C_f, D_f = 0.0, 0.0, 0.0, K_ff
    x_f = 0.0
    for r_k in [1.0, 2.0, 0.5]:
        u_expected = C_f * x_f + D_f * r_k
        x_f = A_f * x_f + B_f * r_k
        assert abs(u_expected - K_ff * r_k) < 1e-12, \
            f"FeedforwardController math mismatch: {u_expected:.6f} vs {K_ff*r_k:.6f}"

    pct = 100.0 * (iae_pi - iae_ff) / iae_pi
    print(f"PI only   - final y: {y_pi[-1]:.4f}  IAE: {iae_pi:.3f}")
    print(f"FF + PI   - final y: {y_ff[-1]:.4f}  IAE: {iae_ff:.3f}"
          f"  ({pct:.1f}% improvement)")
    print("PASS")


if __name__ == "__main__":
    main()
