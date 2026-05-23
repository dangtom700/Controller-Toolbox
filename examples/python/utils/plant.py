"""
Discrete-time state-space plant - Python mirror of lib/StateSpace.h.

tf2ss() uses controllable canonical form, matching C++ tf2ss exactly:
  A : companion matrix (nxn)
  B : [1, 0, ..., 0]'  (only first row = 1)
  C : [b1-a1*b0, b2-a2*b0, ...]
  D : [[b0]]

ss_step() follows the C++ ssStep convention:
  output FIRST  (y = C x + D u),  THEN  state update (x = A x + B u).
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class StateSpace:
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray
    x: np.ndarray = field(default=None, repr=False)   # current state

    def __post_init__(self):
        n = self.A.shape[0]
        if self.x is None:
            self.x = np.zeros(n)

    def reset(self):
        self.x[:] = 0.0


def tf2ss(num: list[float], den: list[float]) -> StateSpace:
    """
    Convert transfer function (num, den) to controllable canonical form SS.

    num / den must be length-matched; leading den[0] is normalised to 1.
    Example plant G(s)=1/(s^2+1.5s+1), ZOH Ts=0.01 ->
        num = [0, 4.9625e-5, 4.9125e-5]
        den = [1, -1.98511,  0.98522]
    """
    num = np.asarray(num, dtype=float)
    den = np.asarray(den, dtype=float)
    assert den[0] != 0.0, "den[0] must be non-zero"
    den = den / den[0]
    num = num / den[0] if den[0] != 1.0 else num  # already normalised above

    n = len(den) - 1   # order
    assert len(num) == n + 1, "len(num) must equal len(den)"

    b0 = num[0]
    a_coeffs = den[1:]   # a1..an
    b_coeffs = num[1:]   # b1..bn

    # Companion / controllable canonical A
    A = np.zeros((n, n))
    A[0, :] = -a_coeffs
    for i in range(1, n):
        A[i, i - 1] = 1.0

    # B = [1, 0, ..., 0]'
    B = np.zeros((n, 1))
    B[0, 0] = 1.0

    # C = [b1 - a1*b0, b2 - a2*b0, ...]
    C = np.zeros((1, n))
    for i in range(n):
        C[0, i] = b_coeffs[i] - a_coeffs[i] * b0

    # D = [[b0]]
    D = np.array([[b0]])

    return StateSpace(A=A, B=B, C=C, D=D)


def ss_step(plant: StateSpace, u: float) -> float:
    """
    Advance one timestep.  Output-before-update to match C++ ssStep().
    Returns scalar output y.
    """
    u_vec = np.array([[u]])
    y = float(plant.C @ plant.x + plant.D @ u_vec[0])
    plant.x = plant.A @ plant.x + (plant.B @ u_vec).ravel()
    return y


# ---------------------------------------------------------------------------
# Convenience: build the canonical example plant (G(s)=1/(s^2+1.5s+1), Ts=0.01)
# ---------------------------------------------------------------------------
EXAMPLE_NUM = [0.0,      4.9625e-5, 4.9125e-5]
EXAMPLE_DEN = [1.0, -1.98511,      0.98522    ]

def example_plant() -> StateSpace:
    """Return a fresh instance of the example second-order plant."""
    return tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
