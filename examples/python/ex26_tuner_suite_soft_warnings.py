"""
ex26 - TunerSuite Soft-Warning Dispatch (Python Mirror)
=========================================================
Goal     : Replicate the TunerSuite tier-dispatch logic in Python and verify
           that IDEAL pairings emit no warning, SOFT pairings emit one warning
           to stderr, and FALLBACK pairings log a fallback notice.
           This validates the C++ TunerSuite design without requiring compilation.

Data generation : Synthetic FOPDT parameters; no plant simulation needed.
Verification    :
  - PID + RelayZN -> IDEAL (no warning).
  - SMC + RelayZN -> FALLBACK (warning emitted).
  - LeadLag + RelayZN -> SOFT (warning emitted).
  - Result objects always have success=True for well-formed inputs.

Run:
    conda activate soft_robotics
    python ex26_tuner_suite_soft_warnings.py
"""

import sys, io
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.verify import print_summary

# --- Python TunerSuite mirror ---
from enum import Enum, auto
from dataclasses import dataclass, field

class CtrlKind(Enum):
    PID = auto(); LQR = auto(); LQG = auto(); MPC = auto()
    LeadLag = auto(); SMC = auto(); ADRC = auto()
    ESC = auto(); Smith = auto(); Generic = auto()

class Tier(Enum):
    IDEAL = auto(); SOFT = auto(); FALLBACK = auto()

@dataclass
class TuneResult:
    success: bool = False
    warned:  bool = False
    warning: str  = ""
    Kp: float = 0.0; Ki: float = 0.0; Kd: float = 0.0

def _tier_relay_zn(kind: CtrlKind) -> Tier:
    if kind == CtrlKind.PID:    return Tier.IDEAL
    if kind == CtrlKind.Smith:  return Tier.SOFT
    if kind == CtrlKind.LeadLag: return Tier.SOFT
    return Tier.FALLBACK

def _tier_imc_pid(kind: CtrlKind) -> Tier:
    if kind == CtrlKind.PID:    return Tier.IDEAL
    if kind == CtrlKind.Smith:  return Tier.SOFT
    return Tier.FALLBACK

def _tier_bryson(kind: CtrlKind) -> Tier:
    if kind in (CtrlKind.LQR, CtrlKind.LQG): return Tier.IDEAL
    if kind == CtrlKind.MPC:                   return Tier.SOFT
    return Tier.FALLBACK

_TIER_MAP = {"RelayZN": _tier_relay_zn, "IMC": _tier_imc_pid, "Bryson": _tier_bryson}

def dispatch(tuner_name: str, kind: CtrlKind,
             Kp=1.0, Ki=0.5, Kd=0.1) -> TuneResult:
    tier_fn = _TIER_MAP.get(tuner_name)
    if tier_fn is None:
        raise ValueError(f"Unknown tuner: {tuner_name}")
    t = tier_fn(kind)
    result = TuneResult(success=True, Kp=Kp, Ki=Ki, Kd=Kd)
    if t == Tier.SOFT:
        result.warned  = True
        result.warning = (f"[soft_robotics::TunerSuite] {tuner_name} is sub-optimal "
                          f"for {kind.name}; consider a dedicated tuner.")
        print(result.warning, file=sys.stderr)
    elif t == Tier.FALLBACK:
        result.warned  = True
        result.warning = (f"[soft_robotics::TunerSuite] {tuner_name} has no specialisation "
                          f"for {kind.name}; Nelder-Mead fallback will be used.")
        print(result.warning, file=sys.stderr)
    return result

# ---------- Test cases ----------
print("=" * 60)
print("ex26 - TunerSuite Soft-Warning Dispatch")
print("=" * 60)

cases = [
    ("RelayZN", CtrlKind.PID,     "IDEAL - no warning"),
    ("RelayZN", CtrlKind.Smith,   "SOFT  - 1 warning"),
    ("RelayZN", CtrlKind.LeadLag, "SOFT  - 1 warning"),
    ("RelayZN", CtrlKind.SMC,     "FALLBACK - 1 warning"),
    ("IMC",     CtrlKind.PID,     "IDEAL - no warning"),
    ("IMC",     CtrlKind.LQR,     "FALLBACK - 1 warning"),
    ("Bryson",  CtrlKind.LQR,     "IDEAL - no warning"),
    ("Bryson",  CtrlKind.MPC,     "SOFT  - 1 warning"),
    ("Bryson",  CtrlKind.PID,     "FALLBACK - 1 warning"),
]

results = {}
print("\n  Tuner     | CtrlKind | Expected          | warned | success")
print("  " + "-"*66)

for tuner, kind, expected in cases:
    # Capture stderr to count warnings without cluttering stdout
    buf = io.StringIO()
    old_err = sys.stderr
    sys.stderr = buf
    r = dispatch(tuner, kind)
    sys.stderr = old_err
    warn_count = buf.getvalue().count("[soft_robotics")

    expect_warn = "IDEAL" not in expected
    ok = (r.success and
          r.warned == expect_warn and
          warn_count == (1 if expect_warn else 0))
    results[f"{tuner}+{kind.name}"] = ok
    print(f"  {tuner:<10}| {kind.name:<9}| {expected:<19}| "
          f"{'Yes' if r.warned else 'No':>6} | {'PASS' if ok else 'FAIL'}")

print_summary(results)
