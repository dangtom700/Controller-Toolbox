"""
ex01 - Plant Construction and Transfer-Function to State-Space Conversion
=========================================================================
Audience : Experienced developers familiar with the C++ controller_toolbox.
Goal     : Reproduce lib/StateSpace tf2ss() in Python and verify the resulting
           matrices match the analytically expected controllable canonical form
           for the example plant G(s) = 1 / (s^2 + 1.5s + 1), Ts = 0.01 s.

Data generation: algebraically from known ZOH-discretised coefficients.
Verification   : element-wise matrix comparison against hand-computed values.

Run:
    conda activate soft_robotics
    python ex01_plant_construction.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN
from utils.verify import assert_close, print_summary

# ---------------------------------------------------------------------------
# Expected matrices (controllable canonical form, verified by hand)
# G(z): num=[0, 4.9625e-5, 4.9125e-5], den=[1, -1.98511, 0.98522]
#
#   A = [ -a1   -a2 ]  =  [ 1.98511  -0.98522 ]
#       [  1     0  ]     [ 1.0       0.0      ]
#
#   B = [ 1 ]    C = [ b1 - a1*b0,  b2 - a2*b0 ]   D = [ b0 ]
#       [ 0 ]
# ---------------------------------------------------------------------------
a1, a2 = -1.98511, 0.98522
b0, b1, b2 = 0.0, 4.9625e-5, 4.9125e-5

A_expected = np.array([[ -a1, -a2],
                        [  1.0, 0.0]])
B_expected = np.array([[1.0], [0.0]])
C_expected = np.array([[b1 - a1*b0, b2 - a2*b0]])
D_expected = np.array([[b0]])

print("=" * 60)
print("ex01 - Plant Construction (tf2ss)")
print("=" * 60)

ss = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)

print("\nConstructed A:\n", ss.A)
print("Expected   A:\n", A_expected)
print("\nConstructed B:\n", ss.B)
print("Expected   B:\n", B_expected)
print("\nConstructed C:\n", ss.C)
print("Expected   C:\n", C_expected)
print("\nConstructed D:\n", ss.D)
print("Expected   D:\n", D_expected)

results = {}
for (r, c), val in np.ndenumerate(A_expected):
    results[f"A[{r},{c}]"] = assert_close(ss.A[r,c], val, tol=1e-9, label=f"A[{r},{c}]")
for r in range(2):
    results[f"B[{r},0]"] = assert_close(ss.B[r,0], B_expected[r,0], tol=1e-12, label=f"B[{r},0]")
for c in range(2):
    results[f"C[0,{c}]"] = assert_close(ss.C[0,c], C_expected[0,c], tol=1e-12, label=f"C[0,{c}]")
results["D[0,0]"] = assert_close(ss.D[0,0], D_expected[0,0], tol=1e-15, label="D[0,0]")

# Verify initial state is zero
results["x0_zero"] = assert_close(np.max(np.abs(ss.x)), 0.0, tol=0.0, label="x0 = 0")

print_summary(results)
