"""
ex129_subspace_id_variants.py

Phase 3 (SI3): MOESP/N4SID/CVA subspace-ID weighting variants.

Mirrors ex112_subspace_id_variants.cpp -- identifies a known 2-output system with
mismatched per-channel measurement noise, demonstrating CVA's reliable advantage over N4SID
averaged over many independent trials (a single noise draw was found, during design
prototyping, to flip unpredictably between CVA winning and losing). Neither CVA nor N4SID is
claimed to beat plain MOESP; prototyping found unweighted MOESP the strongest performer on
this synthetic system.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'subspace_id'):
        raise AttributeError("subspace_id not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

A_true = np.array([[0.9, 0.1], [-0.05, 0.85]])
B_true = np.array([[0.5], [0.2]])
C_true = np.array([[1.0, 0.0], [0.0, 1.0]])
D_true = np.zeros((2, 1))
Ts = 0.1
N = 2000

freqs = [0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 16.0, 20.0]
# get_frequency_response() is SISO-only -- evaluate channel 1 (the high-noise channel)
# via a per-channel SISO sub-system (channel identity survives the similarity
# transform; only the state basis changes).
sys_true_chan1 = ctrl.StateSpace(A_true, B_true, C_true[1:2, :], D_true[1:2, :], Ts)
resp_true = np.array(ctrl.SystemAnalysis.get_frequency_response(sys_true_chan1, freqs))


def chan_freq_error(model):
    model_chan1 = ctrl.StateSpace(model.A, model.B, model.C[1:2, :], model.D[1:2, :], Ts)
    resp_est = np.array(ctrl.SystemAnalysis.get_frequency_response(model_chan1, freqs))
    return float(np.mean(np.abs(np.abs(resp_est) - np.abs(resp_true))))


print("=== Subspace ID method variants (Phase 3 SI3) ===")
print("Mismatched output noise: channel 0 std=0.005, channel 1 std=0.3")
print("Averaging high-noise-channel freq-response error over 20 independent trials\n")

n_trials = 20
master_rng = np.random.default_rng(2026)
total_n4sid, total_cva = 0.0, 0.0
ok = True

for trial in range(n_trials):
    rng_u = np.random.default_rng(master_rng.integers(0, 2**31))
    rng_n = np.random.default_rng(master_rng.integers(0, 2**31))
    U = rng_u.normal(0.0, 1.0, size=(1, N))

    noise_std = np.array([0.005, 0.3])  # channel 1 has 60x channel 0's noise
    x = np.zeros(2)
    Y = np.zeros((2, N))
    for k in range(N):
        u_k = U[:, k]
        y_k = C_true @ x + D_true @ u_k
        y_k = y_k + rng_n.normal(0.0, 1.0, size=2) * noise_std
        Y[:, k] = y_k
        x = A_true @ x + B_true @ u_k

    res_n4sid = ctrl.subspace_id(Y, U, n_order=2, i_horizon=10, Ts=Ts, method=ctrl.SubspaceMethod.N4SID)
    res_cva = ctrl.subspace_id(Y, U, n_order=2, i_horizon=10, Ts=Ts, method=ctrl.SubspaceMethod.CVA)
    if not (res_n4sid.success and res_cva.success):
        print(f"trial {trial} failed")
        ok = False
        continue

    total_n4sid += chan_freq_error(res_n4sid.get_model())
    total_cva += chan_freq_error(res_cva.get_model())

mean_n4sid = total_n4sid / n_trials
mean_cva = total_cva / n_trials
print(f"N4SID: mean high-noise-channel freq-response error = {mean_n4sid:.5f}")
print(f"CVA:   mean high-noise-channel freq-response error = {mean_cva:.5f}")

ok = ok and (mean_cva < mean_n4sid)
print("\n[PASS] All checks passed." if ok else "\n[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
