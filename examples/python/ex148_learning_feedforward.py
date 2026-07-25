"""
ex148_learning_feedforward.py -- LearningFeedforwardController: two-phase ILC + PID.

Demonstrates:
  1. The trial state machine (step index, wrap at trialLength, updateFeedforward at
     the boundary, switch from recording to applying) handled inside the class - the
     block that ten case studies carry as copy-pasted code.
  2. Per-trial RMS error falling monotonically on a repeating task.
  3. learnTrials controls how many trials record before the feedforward is applied.
  4. The trialLength / ILCParams.N contract is enforced at construction.

Plant:  y[k+1] = a*y[k] + (1-a)*(u[k] + LOAD),  tau = 0.2 s
Task :  r[k] = sin(2*pi*k / N_TRIAL), repeating every trial (the ILC precondition).

Sign convention: mirrors the nominal controller; with DiscretePID that is r - y.
"""

import sys
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'LearningFeedforwardController'):
        raise AttributeError("LearningFeedforwardController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex148: LearningFeedforwardController (two-phase ILC) ===")

TS = 0.01
N_TRIAL = 200
N_TRIALS = 6
LOAD = 0.30
A = np.exp(-TS / 0.2)


def pid_params():
    p = ctrl.PIDParams()
    p.Kp, p.Ki, p.Kd = 1.2, 2.0, 0.0
    p.uMin, p.uMax = -10.0, 10.0
    return p


def ilc_params():
    p = ctrl.ILCParams()
    p.N, p.Ts, p.mode = N_TRIAL, TS, ctrl.ILCMode.PType
    p.Lp, p.Q_filter = 0.6, 0.98
    p.uMin, p.uMax = -10.0, 10.0
    return p


def run(learn_trials):
    """Run N_TRIALS trials.

    Returns (per-trial RMS list, max |u_ff| seen during the recording trials,
    controller). The plant state carries over between trials, so recording-trial
    RMS values are NOT identical - the invariant to check is that the learned
    feedforward contributes exactly nothing until learnTrials have elapsed.
    """
    lp = ctrl.LearningFFParams()
    lp.trialLength, lp.learnTrials, lp.autoAdvance = N_TRIAL, learn_trials, True
    lp.uMin, lp.uMax = -10.0, 10.0
    lff = ctrl.LearningFeedforwardController(ctrl.DiscretePID(pid_params(), TS),
                                             ilc_params(), lp, TS)
    y, rms, ff_while_recording = 0.0, [], 0.0
    for t in range(N_TRIALS):
        sq = 0.0
        for k in range(N_TRIAL):
            e = np.sin(2.0 * np.pi * k / N_TRIAL) - y
            sq += e * e
            u = lff.compute(e)
            if t < learn_trials:
                ff_while_recording = max(ff_while_recording, abs(lff.feedforward_term()))
            y = A * y + (1.0 - A) * (u + LOAD)
        rms.append(np.sqrt(sq / N_TRIAL))
    return rms, ff_while_recording, lff


# ---------------------------------------------------------------------------
# 1. Learning curve
# ---------------------------------------------------------------------------
rms, ff_rec, lff = run(learn_trials=1)

print("\n-- Per-trial RMS tracking error (repeating sine + constant load) --")
print(f"  {'trial':>6}{'RMS error':>14}{'change':>12}")
for t, v in enumerate(rms):
    delta = "" if t == 0 else f"{100.0 * (v / rms[t - 1] - 1.0):+.2f} %"
    print(f"  {t:>6}{v:>14.6f}{delta:>12}")

print(f"\n  trials completed : {lff.trial_index()}   still learning : {lff.learning()}")
print(f"  reduction trial 0 -> {N_TRIALS - 1} : {100.0 * (1.0 - rms[-1] / rms[0]):.2f} %")

learned_ok = np.isfinite(rms[-1]) and rms[-1] < rms[0]
first_ok = rms[1] < rms[0]
monotone_ok = all(rms[t] <= rms[t - 1] + 1e-9 for t in range(1, N_TRIALS))

# ---------------------------------------------------------------------------
# 2. learnTrials: recording-only trials leave the error untouched
# ---------------------------------------------------------------------------
rms3, ff_rec3, lff3 = run(learn_trials=3)
print("\n-- learnTrials = 3 (trials 0..2 record only) --")
print(f"  trial 0 RMS = {rms3[0]:.6f}   trial 2 RMS = {rms3[2]:.6f}"
      f"   trial 5 RMS = {rms3[5]:.6f}")
print(f"  max |u_ff| during trials 0..2 = {ff_rec3:.1e}  (must be exactly 0)")
print(f"  max |u_ff| during trial 0 with learnTrials=1 = {ff_rec:.1e}")
# RMS across recording trials is not identical (the plant state carries over), so the
# invariant is that the learned term contributes nothing until learnTrials elapse.
recording_ok = ff_rec3 == 0.0 and ff_rec == 0.0 and rms3[5] < rms3[2]

# ---------------------------------------------------------------------------
# 3. The trialLength / ILCParams.N contract
# ---------------------------------------------------------------------------
threw = False
try:
    bad = ctrl.LearningFFParams()
    bad.trialLength, bad.learnTrials = N_TRIAL + 1, 1   # disagrees with ilc_params().N
    ctrl.LearningFeedforwardController(ctrl.DiscretePID(pid_params(), TS),
                                       ilc_params(), bad, TS)
except (ValueError, RuntimeError):
    threw = True
print(f"\n-- Contract check --\n  mismatched trialLength rejected : {threw}")

if not (learned_ok and first_ok and monotone_ok and recording_ok and threw):
    print("\n[FAIL] LearningFeedforwardController demo did not meet its criteria.")
    sys.exit(1)

print("\n[PASS] LearningFeedforwardController demo complete.")
