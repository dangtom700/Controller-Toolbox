# Innovations State-Space (N4SID) Method

## Overview
Subspace state-space system identification (such as the N4SID algorithm) provides a direct way to estimate reliable state-space models for multivariable (MIMO) systems from input-output data. Unlike prediction error methods (PEM) which require iterative, non-convex optimization and careful parameter initialization, subspace methods rely on robust numerical linear algebra tools like Singular Value Decomposition (SVD) and QR factorization. 

The resulting model is in the "innovations" form:
```
x(k+1) = A*x(k) + B*u(k) + K*e(k)
y(k)   = C*x(k) + D*u(k) + e(k)
```
Where `K` is the Kalman gain matrix, making this structure immediately ready for optimal control design (LQR/LQG or MPC).

## Input Requirements
- **Data Source**: CSV file containing time-series input and output data.
- **Format / Content**: For an `m`-input, `p`-output system, the expected format is:
  ```csv
  time,u1,u2,y1,y2
  0.0,1.0,-0.5,0.1,0.0
  0.1,1.0,-0.5,0.3,0.2
  ...
  ```

## Return Value
- Returns a **State-Space Model** object containing the matrices `A`, `B`, `C`, `D`, and `K`.
- The state dimension `n` is either specified by the user or automatically determined by analyzing the singular values during the identification process.
- The object includes the estimated initial state `x0`.

## Example Usage
While the target deployment is C++, Python is frequently used offline to perform subspace identification via libraries like `SIPPY`.

```python
import pandas as pd
import numpy as np
from sippy import system_identification

# 1. Load multivariable data
data = pd.read_csv('data/mimo_plant.csv')
# Assuming 2 inputs and 2 outputs
u = data[['u1', 'u2']].values.T # shape: (inputs, samples)
y = data[['y1', 'y2']].values.T # shape: (outputs, samples)

# 2. Perform N4SID Subspace Identification
# 'N4SID' is the method. 'IC' means information criterion for order selection.
sys_id = system_identification(y, u, id_method='N4SID', SS_fixed_order=4)

# 3. Extract the state-space matrices
A, B, C, D = sys_id.A, sys_id.B, sys_id.C, sys_id.D
K = sys_id.K # Kalman gain

print("Identified A matrix:")
print(A)

# These matrices can now be written to a header file or JSON to be loaded 
# into the C++ ControllerToolbox `StateSpace` struct.
```
