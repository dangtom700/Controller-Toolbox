"""
ex106_frequency_domain_plots.py

Phase 4 (Iteration 1): classical frequency-domain analysis & plotting.

Exercises bode/nyquist/nichols/root_locus/sigma_plot from
tools/freq_domain_plots.py against the README's minimal-example plant.

    Plant: TransferFunction([0.0048, 0.0047], [1.0, -1.81, 0.819], Ts=0.01)

SystemAnalysis operates on StateSpace algebra, not IController time-stepping,
so root_locus uses a static proportional gain (Kp=1.0, matching the README's
DiscretePID example) as the open-loop forward path - the same stand-in
pattern used by ex103/ex104's static-gain controller fixtures.
"""

import sys
import os
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'SystemAnalysis') or not hasattr(ctrl.SystemAnalysis, 'get_singular_values'):
        raise AttributeError("SystemAnalysis.get_singular_values not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _ROOT)
from tools import freq_domain_plots as fdp
import matplotlib.figure
import matplotlib.pyplot as plt

Ts = 0.01
tf = ctrl.TransferFunction([0.0048, 0.0047], [1.0, -1.81, 0.819], Ts)
plant = ctrl.tf2ss(tf)

Kp = 1.0
open_loop = ctrl.StateSpace(plant.A, plant.B, plant.C * Kp, plant.D * Kp, Ts)

freqs = list(np.linspace(0.5, np.pi / Ts - 0.5, 200))
gains = list(np.linspace(0.1, 5.0, 25))

ok = True
try:
    figures = {
        "bode": fdp.bode(plant, freqs),
        "nyquist": fdp.nyquist(plant, freqs),
        "nichols": fdp.nichols(plant, freqs),
        "root_locus": fdp.root_locus(open_loop, gains),
        "sigma_plot": fdp.sigma_plot(plant, freqs),
    }
except Exception as _e:
    print(f"[FAIL] plotting raised an exception: {_e}")
    ok = False
    figures = {}

for name, fig in figures.items():
    if not isinstance(fig, matplotlib.figure.Figure) or len(fig.axes) == 0:
        print(f"[FAIL] {name} did not return a populated Figure")
        ok = False
    else:
        print(f"  {name}: OK ({len(fig.axes)} axes)")

plt.close('all')

print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
