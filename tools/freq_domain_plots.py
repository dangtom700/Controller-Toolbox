"""
tools/freq_domain_plots.py -- classical frequency-domain analysis plots.

Unlike the other tools/*_plots.py modules (mu_plots.py, fault_plots.py, mc_plots.py),
this is not a CSV-driven CLI: there is no natural sweep-result file to point at, since
the input is a live ctrl_toolbox.StateSpace evaluated on the fly. Each function below
takes a StateSpace (+ a frequency or gain array) and returns a matplotlib Figure for
the caller to savefig/show.
"""
from __future__ import annotations
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT))

try:
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: numpy, matplotlib required.")
    sys.exit(1)

import ctrl_toolbox as ctrl


def bode(sys, freqs):
    resp = np.array(ctrl.SystemAnalysis.get_frequency_response(sys, list(freqs)))
    mag_db = 20.0 * np.log10(np.abs(resp))
    phase_deg = np.unwrap(np.angle(resp)) * 180.0 / np.pi

    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, sharex=True, figsize=(7, 6))
    ax_mag.semilogx(freqs, mag_db)
    ax_mag.set_ylabel("Magnitude [dB]")
    ax_mag.set_title("Bode plot")
    ax_mag.grid(True, which="both", alpha=0.3)

    ax_phase.semilogx(freqs, phase_deg)
    ax_phase.set_ylabel("Phase [deg]")
    ax_phase.set_xlabel("Frequency [rad/s]")
    ax_phase.grid(True, which="both", alpha=0.3)

    fig.tight_layout()
    return fig


def nyquist(sys, freqs):
    resp = np.array(ctrl.SystemAnalysis.get_frequency_response(sys, list(freqs)))

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(resp.real, resp.imag, label="omega > 0")
    ax.plot(resp.real, -resp.imag, linestyle="--", label="omega < 0 (mirror)")
    ax.plot([-1], [0], "r+", markersize=12, markeredgewidth=2, label="-1")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.axvline(0, color="gray", linewidth=0.5)
    ax.set_xlabel("Re")
    ax.set_ylabel("Im")
    ax.set_title("Nyquist plot")
    ax.legend(fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    return fig


def nichols(sys, freqs):
    resp = np.array(ctrl.SystemAnalysis.get_frequency_response(sys, list(freqs)))
    mag_db = 20.0 * np.log10(np.abs(resp))
    phase_deg = np.unwrap(np.angle(resp)) * 180.0 / np.pi

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(phase_deg, mag_db)
    ax.set_xlabel("Phase [deg]")
    ax.set_ylabel("Magnitude [dB]")
    ax.set_title("Nichols plot")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def root_locus(open_loop, gains):
    fig, ax = plt.subplots(figsize=(6, 6))
    for k in gains:
        scaled = ctrl.StateSpace(open_loop.A, open_loop.B,
                                  open_loop.C * k, open_loop.D * k, open_loop.Ts)
        closed = ctrl.SystemAnalysis.feedback(scaled)
        poles = ctrl.SystemAnalysis.get_poles(closed)
        ax.scatter([p.real for p in poles], [p.imag for p in poles],
                   s=10, color="steelblue")

    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(np.cos(theta), np.sin(theta), "k--", linewidth=0.8, label="unit circle")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.axvline(0, color="gray", linewidth=0.5)
    ax.set_xlabel("Re")
    ax.set_ylabel("Im")
    ax.set_title("Root locus")
    ax.legend(fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    return fig


def sigma_plot(sys, freqs):
    svs = ctrl.SystemAnalysis.get_singular_values(sys, list(freqs))
    n_sv = len(svs[0])
    svs_db = 20.0 * np.log10(np.array([list(sv) for sv in svs]) + 1e-300)

    fig, ax = plt.subplots(figsize=(7, 5))
    for i in range(n_sv):
        ax.semilogx(freqs, svs_db[:, i], label=f"sigma_{i + 1}")
    ax.set_xlabel("Frequency [rad/s]")
    ax.set_ylabel("Singular value [dB]")
    ax.set_title("Singular-value plot")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    return fig
