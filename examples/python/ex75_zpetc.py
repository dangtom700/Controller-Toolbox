"""
ex75_zpetc.py
Zero-Phase Error Tracking Control (ZPETC) via pybind11 binding.

System 1 (min-phase):  G1(z) = 0.2*(z+0.5) / ((z-0.8)*(z-0.6))
  -> exact causal inversion: G1*G_ff = z^{-1} (unit magnitude everywhere)

System 2 (NMP, zero at 1.5): G2(z) = 0.1*(z-1.5) / ((z-0.9)*(z-0.7))
  -> partial inversion: G2*G_ff has unit DC gain, amplitude error away from DC
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.01

    # System 1: minimum-phase
    tf1 = ctrl.TransferFunction([0.2, 0.1], [1.0, -1.4, 0.48], Ts)
    sys1 = ctrl.tf2ss(tf1)
    res1 = ctrl.design_zpetc(sys1)

    print(f"System 1 - hasNMPZeros: {res1.has_nmp_zeros},  "
          f"dcAmplitudeError: {res1.dc_amplitude_error:.2e}")
    assert not res1.has_nmp_zeros, "System 1 should have no NMP zeros"
    assert res1.dc_amplitude_error < 1e-8, \
        f"Min-phase ZPETC should have near-zero amplitude error, got {res1.dc_amplitude_error}"

    # System 2: non-minimum-phase
    tf2 = ctrl.TransferFunction([0.1, -0.15], [1.0, -1.6, 0.63], Ts)
    sys2 = ctrl.tf2ss(tf2)
    res2 = ctrl.design_zpetc(sys2)

    print(f"System 2 - hasNMPZeros: {res2.has_nmp_zeros},  "
          f"dcAmplitudeError: {res2.dc_amplitude_error:.4f}")
    print(f"  NMP zeros: {[f'{z.real:.3f}+{z.imag:.3f}j' for z in res2.nmp_zeros]}")

    assert res2.has_nmp_zeros, "System 2 should have NMP zeros"

    # Verify DC gain of composite G2*G_ff approx = 1
    freqs_low = [1e-4]
    g2_dc   = ctrl.SystemAnalysis.get_frequency_response(sys2,   freqs_low)[0]
    gff_dc  = ctrl.SystemAnalysis.get_frequency_response(res2.filter, freqs_low)[0]
    composite_dc = abs(g2_dc * gff_dc)
    print(f"  Composite DC gain: {composite_dc:.4f}  (need approx = 1.0)")
    assert abs(composite_dc - 1.0) < 0.05, \
        f"Composite DC gain too far from 1: {composite_dc:.4f}"

    # Verify transmission_zeros utility returns the NMP zero
    zeros = ctrl.transmission_zeros(sys2)
    zero_mags = [abs(z) for z in zeros]
    print(f"  Transmission zeros magnitudes: {[f'{m:.3f}' for m in zero_mags]}")
    assert any(m >= 1.0 for m in zero_mags), "Expected at least one NMP zero"

    print("PASS")


if __name__ == "__main__":
    main()
