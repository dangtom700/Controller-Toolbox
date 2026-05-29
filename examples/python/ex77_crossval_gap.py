"""
ex77_crossval_gap.py
---------------------
Cross-validates the C++ nuGap binding against a Python reference implementation.

The nu-gap upper bound (chordal-distance formula) for SISO discrete systems is:
    delta(P1, P2) = max_omega |P1(z) - P2(z)| / sqrt((1+|P1|^2)(1+|P2|^2))
where z = exp(j*omega*Ts).

We compute this independently in Python using numpy and compare against ctrl.nu_gap().

Expected output: PASS
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
import ctrl_toolbox as ctrl

# --------------------------------------------------------------------------- #
# Python reference implementation
# --------------------------------------------------------------------------- #

def freq_resp_ss(A, B, C, D, omega, Ts):
    """Evaluate C*(z*I - A)^{-1}*B + D at z = exp(j*omega*Ts) for scalar SISO."""
    n = A.shape[0]
    result = np.zeros(len(omega), dtype=complex)
    for i, w in enumerate(omega):
        z  = np.exp(1j * w * Ts)
        zIA = z * np.eye(n) - A
        H  = C @ np.linalg.solve(zIA, B) + D
        result[i] = H[0, 0]
    return result


def nu_gap_python(sys1, sys2, Ts, n_freqs=200):
    """Python reference: nu-gap upper bound via chordal metric."""
    omega = np.logspace(-2, np.log10(np.pi / Ts), n_freqs)

    H1 = freq_resp_ss(sys1['A'], sys1['B'], sys1['C'], sys1['D'], omega, Ts)
    H2 = freq_resp_ss(sys2['A'], sys2['B'], sys2['C'], sys2['D'], omega, Ts)

    num = np.abs(H1 - H2)
    den = np.sqrt((1.0 + np.abs(H1)**2) * (1.0 + np.abs(H2)**2))
    return float(np.max(num / den))


# --------------------------------------------------------------------------- #
# Test pairs
# --------------------------------------------------------------------------- #

Ts = 0.1

test_cases = [
    # (P1 pole, P2 pole, description)
    (0.90, 0.90, "identical systems -> gap ~ 0"),
    (0.90, 0.89, "very similar systems -> small gap"),
    (0.90, 0.50, "moderately different"),
    (0.90, 0.10, "very different"),
]

print(f"{'Description':<35} {'Python':>10} {'C++':>10} {'|diff|':>10}")
print("-" * 70)

tol = 5e-3  # 200 vs 200 freq points; tiny difference due to floating-point order

for pole1, pole2, desc in test_cases:
    # Construct StateSpace objects
    def make_ss_dict(pole, Ts):
        return {'A': np.array([[pole]]),
                'B': np.array([[1.0 - pole]]),
                'C': np.array([[1.0]]),
                'D': np.array([[0.0]])}

    def make_ss_ctrl(pole, Ts):
        return ctrl.StateSpace(
            np.array([[pole]]),
            np.array([[1.0 - pole]]),
            np.array([[1.0]]),
            np.array([[0.0]]),
            Ts
        )

    s1_dict = make_ss_dict(pole1, Ts)
    s2_dict = make_ss_dict(pole2, Ts)
    s1_ctrl = make_ss_ctrl(pole1, Ts)
    s2_ctrl = make_ss_ctrl(pole2, Ts)

    gap_py  = nu_gap_python(s1_dict, s2_dict, Ts, n_freqs=200)
    gap_cpp = ctrl.nu_gap(s1_ctrl, s2_ctrl, 200)
    diff    = abs(gap_py - gap_cpp)

    print(f"{desc:<35} {gap_py:>10.5f} {gap_cpp:>10.5f} {diff:>10.2e}")
    assert diff < tol, (
        f"nu_gap mismatch for '{desc}': python={gap_py:.6f}, cpp={gap_cpp:.6f}"
    )

# Additional: nu_gap_matrix symmetry and zero diagonal
print("\nnuGapMatrix symmetry check:")
poles  = [0.90, 0.70, 0.30]
models = [ctrl.StateSpace(np.array([[p]]),
                          np.array([[1.0 - p]]),
                          np.array([[1.0]]),
                          np.array([[0.0]]),
                          Ts) for p in poles]

G = ctrl.nu_gap_matrix(models, 150)
assert G.shape == (3, 3), "Expected 3x3 gap matrix"
for i in range(3):
    assert abs(G[i, i]) < 1e-10, f"Diagonal ({i},{i}) not zero: {G[i,i]}"
for i in range(3):
    for j in range(3):
        assert abs(G[i, j] - G[j, i]) < 1e-10, f"Matrix not symmetric at ({i},{j})"
print("  3x3 matrix: diagonal=0, symmetric - OK")

print("\nPASS")
