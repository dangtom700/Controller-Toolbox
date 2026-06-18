"""controllers.py - controller roster for PCM Thermal Energy Storage Control (TEMPLATE).

ctrl_toolbox is imported with a graceful fallback; if unavailable, only the
OpenLoop stub runs.
"""
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
# sim/ -> StudyName/ -> case-study/ -> repo root
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_THIS)))
sys.path.insert(0, _ROOT)
try:
    import _setup_bindings  # noqa: F401  (sets ctrl_toolbox DLL/.so path)
    import ctrl_toolbox as ctrl
    _HAVE_CTRL = True
except Exception as _e:  # pragma: no cover
    print("WARNING: ctrl_toolbox not available (%s) - only OpenLoop will run." % _e)
    _HAVE_CTRL = False


class OpenLoop:
    """Zero-input baseline."""
    def reset(self):
        pass
    def compute(self, error: float) -> float:
        return 0.0


def make_controllers(Ts: float):
    """Return list of (name, controller) tuples (12 entries)."""
    roster = [("OpenLoop", OpenLoop())]
    if _HAVE_CTRL:
        # TODO: add the real roster; the paper's proposed method must be present.
        # roster.append(("PID", ctrl.DiscretePID(Kp, Ki, Kd, Ts)))
        pass
    return roster
