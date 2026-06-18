"""satellite_launch_vehicle_systems_plant.py - plant model for Satellite Launch Vehicle Systems (TEMPLATE)."""
import numpy as np


class Plant:
    """Discrete-time plant: x[k+1] = f(x[k], u[k]); y = h(x)."""

    def __init__(self, params: dict):
        self.Ts = params.get("Ts", 0.01)
        self.a = params.get("param_a", 1.0)
        self.b = params.get("param_b", 1.0)
        self.u_max = params.get("u_max", 1.0)
        self.u_min = params.get("u_min", -1.0)
        self.x = np.zeros(1)          # TODO: real state dimension

    def reset(self, x0=None):
        self.x = np.zeros_like(self.x) if x0 is None else np.asarray(x0, float)

    def step(self, u: float):
        u = float(np.clip(u, self.u_min, self.u_max))
        # TODO: real dynamics. Placeholder first-order: x' = -a*x + b*u
        dx = -self.a * self.x[0] + self.b * u
        self.x[0] += self.Ts * dx

    def output(self) -> float:
        return float(self.x[0])
