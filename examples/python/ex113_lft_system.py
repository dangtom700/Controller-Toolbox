"""
ex113_lft_system.py

Phase 3 (RC1): two simultaneous, coupled uncertainty blocks via LFTSystem.

Mirrors ex96_lft_system.cpp - builds a small 2-channel open-loop map with two coupled
uncertainty mechanisms and compares LFTSystem's combined structured-singular-value bound
against the naive "worst of each channel taken independently" comparison.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'LFTSystem'):
        raise AttributeError("LFTSystem not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

A = np.array([[0.4, 0.0], [0.0, 0.6]])
B = np.array([[1.0, 0.2], [0.15, 1.0]])
C = np.array([[1.0, 0.1], [0.1, 1.0]])
D = np.zeros((2, 2))
M0 = ctrl.StateSpace(A, B, C, D, 0.1)

struc = ctrl.UncertaintyStructure()
b1 = ctrl.UncertaintyBlock(); b1.type = ctrl.UncertaintyBlock.Type.ComplexFull; b1.r_out = 1; b1.r_in = 1
b2 = ctrl.UncertaintyBlock(); b2.type = ctrl.UncertaintyBlock.Type.ComplexFull; b2.r_out = 1; b2.r_in = 1
struc.blocks = [b1, b2]

chmap = ctrl.LFTChannelMap()
chmap.row_start = [0, 1]
chmap.col_start = [0, 1]

lft = ctrl.LFTSystem(M0, struc, chmap)
combined = lft.peak_mu(freq_points=100, omega_min=1e-2)
print(f"Combined (coupled) peak mu upper bound = {combined.peak.upper:.4f} "
      f"at omega = {combined.peak_omega_rad_s:.4f} rad/s")

single = ctrl.UncertaintyStructure()
single.blocks = [b1]
map1 = ctrl.LFTChannelMap(); map1.row_start = [0]; map1.col_start = [0]
map2 = ctrl.LFTChannelMap(); map2.row_start = [1]; map2.col_start = [1]
lft1 = ctrl.LFTSystem(M0, single, map1)
lft2 = ctrl.LFTSystem(M0, single, map2)
peak1 = lft1.peak_mu(100, 1e-2).peak.upper
peak2 = lft2.peak_mu(100, 1e-2).peak.upper
naive_worst = max(peak1, peak2)
print(f"Independent-channel comparison: channel1={peak1:.4f}  channel2={peak2:.4f}  "
      f"worst={naive_worst:.4f}")

ok = np.isfinite(combined.peak.upper) and combined.peak.upper > 0.0 and np.isfinite(naive_worst)
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
