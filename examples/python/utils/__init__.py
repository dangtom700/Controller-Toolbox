"""
Utility package mirroring the C++ controller_toolbox for Python-side verification.

All implementations match the discrete-time math in lib/ exactly so that C++ and
Python outputs can be compared sample-by-sample.
"""
from .plant import StateSpace, tf2ss, ss_step
from .controllers import (
    DiscretePID, DiscreteLQR, KalmanFilter, DiscreteLQG,
    DiscreteMPC, LeadLag, DiscreteSMC, DiscreteADRC,
    ExtremumSeeker, SmithPredictor
)
from .data_gen import prbs, chirp_signal, multi_sine, add_noise, step_signal
from .verify import (
    check_alignment, dc_gain_check, snr_db, rmse,
    ise, itae, assert_close, print_summary
)

__all__ = [
    "StateSpace", "tf2ss", "ss_step",
    "DiscretePID", "DiscreteLQR", "KalmanFilter", "DiscreteLQG",
    "DiscreteMPC", "LeadLag", "DiscreteSMC", "DiscreteADRC",
    "ExtremumSeeker", "SmithPredictor",
    "prbs", "chirp_signal", "multi_sine", "add_noise", "step_signal",
    "check_alignment", "dc_gain_check", "snr_db", "rmse",
    "ise", "itae", "assert_close", "print_summary",
]
