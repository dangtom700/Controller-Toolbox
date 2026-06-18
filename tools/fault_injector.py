"""
tools/fault_injector.py  -- ANA-3: fault injection wrapper for case-study simulations.

Provides FaultInjector, a callable that wraps a simulation step function and injects
faults at specified times.  Faults are composable: multiple can be active simultaneously.

Fault types
-----------
sensor_bias    -- adds a constant offset to the plant output y
sensor_noise   -- adds zero-mean Gaussian noise to y (sigma specified)
actuator_loss  -- multiplies the control input u by (1 - loss_fraction)
actuator_stuck -- clamps u to a fixed value after fault_time
setpoint_step  -- steps the reference signal r by a specified amount at fault_time
parameter_jump -- not handled here; handled via PlantParamOverride in study sims

Usage
-----
    from tools.fault_injector import FaultInjector, FaultSpec

    faults = [
        FaultSpec(kind="sensor_bias",  fault_time=5.0, magnitude=0.5),
        FaultSpec(kind="actuator_loss", fault_time=10.0, magnitude=0.3),
    ]
    fi = FaultInjector(faults, seed=42)

    # In simulation loop:
    y_obs = fi.inject_sensor(t, y_true)
    u_cmd = controller.compute(r - y_obs)
    u_act = fi.inject_actuator(t, u_cmd)
    plant.step(u_act)
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Optional

try:
    import numpy as np
except ImportError:
    import sys
    print("ERROR: numpy required.")
    sys.exit(1)


FAULT_KINDS = frozenset([
    "none",
    "sensor_bias",
    "sensor_noise",
    "actuator_loss",
    "actuator_stuck",
    "setpoint_step",
])


@dataclass
class FaultSpec:
    """Specification for one fault event.

    Attributes
    ----------
    kind        : one of FAULT_KINDS
    fault_time  : simulation time [s] at which the fault activates (default 0 = always)
    magnitude   : fault magnitude (bias offset, noise sigma, loss fraction, stuck value, step amount)
    end_time    : fault deactivates at this time (default inf = permanent)
    """
    kind:       str   = "none"
    fault_time: float = 0.0
    magnitude:  float = 0.0
    end_time:   float = float("inf")

    def __post_init__(self):
        if self.kind not in FAULT_KINDS:
            raise ValueError(f"Unknown fault kind '{self.kind}'.  Must be one of: {sorted(FAULT_KINDS)}")

    def is_active(self, t: float) -> bool:
        return self.fault_time <= t < self.end_time


class FaultInjector:
    """Wraps a simulation loop to inject faults at specified times.

    Parameters
    ----------
    faults : list of FaultSpec
    seed   : RNG seed for noise faults
    """

    def __init__(self, faults: list[FaultSpec], seed: int = 0):
        self._faults = list(faults)
        self._rng    = np.random.default_rng(seed)
        self._stuck_val: Optional[float] = None

    def inject_sensor(self, t: float, y: float) -> float:
        """Return the possibly-corrupted sensor reading at time t."""
        y_obs = float(y)
        for f in self._faults:
            if not f.is_active(t):
                continue
            if f.kind == "sensor_bias":
                y_obs += f.magnitude
            elif f.kind == "sensor_noise":
                y_obs += float(self._rng.normal(0.0, abs(f.magnitude)))
        return y_obs

    def inject_actuator(self, t: float, u: float) -> float:
        """Return the possibly-corrupted actuator signal at time t."""
        u_act = float(u)
        for f in self._faults:
            if not f.is_active(t):
                continue
            if f.kind == "actuator_loss":
                loss = max(0.0, min(1.0, f.magnitude))
                u_act *= (1.0 - loss)
            elif f.kind == "actuator_stuck":
                # Capture the value at the moment the fault activates
                if t == f.fault_time or (self._stuck_val is None and t >= f.fault_time):
                    self._stuck_val = u_act
                if self._stuck_val is not None:
                    u_act = self._stuck_val
        return u_act

    def inject_reference(self, t: float, r: float) -> float:
        """Return the possibly-stepped reference signal at time t."""
        r_out = float(r)
        for f in self._faults:
            if f.kind == "setpoint_step" and f.is_active(t):
                r_out += f.magnitude
        return r_out

    def reset(self):
        """Reset internal state (stuck value, RNG position unchanged)."""
        self._stuck_val = None
