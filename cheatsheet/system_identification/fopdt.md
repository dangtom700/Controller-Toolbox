# Step-Response (FOPDT) Identification Method

## Overview
The First-Order Plus Dead-Time (FOPDT) model is the workhorse of classical industrial process control. It approximates complex, high-order, overdamped processes using just three parameters: process gain ($K$), time constant ($\tau$), and dead time ($\theta$). It is primarily derived by analyzing the open-loop response of a system to a step input.

The continuous-time transfer function is:
`G(s) = [K / (\tau s + 1)] * e^{-\theta s}`

This structure is directly compatible with widely used tuning rules like Ziegler-Nichols, Cohen-Coon, and IMC-PID.

## Input Requirements
- **Data Source**: CSV file containing the open-loop response to a step change in the input.
- **Format / Content**: 
  ```csv
  time,u,y
  0.0,0.0,0.0
  0.1,5.0,0.0  <- Step change applied to input 'u'
  0.2,5.0,0.1
  ...
  10.0,5.0,8.0 <- System reaches new steady state
  ```
  The data must capture the entire transient response from the step change until the output settles.

## Return Value
- Returns a **FOPDT Model** dictionary or object containing:
  - `K`: Static process gain ($\Delta y / \Delta u$)
  - `tau` ($\tau$): Process time constant (time to reach 63.2% of final response after dead time)
  - `theta` ($\theta$): Process dead time (delay before the process starts responding)

## Example Usage
Our C++ library already contains `ctrl::StepResponseTuner::identify()` for this purpose. If you need to perform it in Python offline, you can use the tangent method or nonlinear curve fitting.

```python
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit

# Define the FOPDT step response function
def fopdt_step_response(t, K, tau, theta, step_amp, t_step):
    y = np.zeros_like(t)
    # Only calculate response for t > (t_step + theta)
    idx = t > (t_step + theta)
    y[idx] = K * step_amp * (1 - np.exp(-(t[idx] - t_step - theta) / tau))
    return y

# 1. Load step response data
data = pd.read_csv('data/step_test.csv')
t = data['time'].values
y = data['y'].values
# Assuming step from 0 to 5 occurred at t=0.1
step_amplitude = 5.0 
step_time = 0.1

# 2. Fit the FOPDT parameters using non-linear least squares
# Initial guesses: K=1, tau=1, theta=0
p0 = [1.0, 1.0, 0.0] 
bounds = ([0, 0, 0], [np.inf, np.inf, np.inf]) # constrain to positive values

popt, _ = curve_fit(
    lambda t, K, tau, theta: fopdt_step_response(t, K, tau, theta, step_amplitude, step_time),
    t, y, p0=p0, bounds=bounds
)

K, tau, theta = popt
print(f"Identified FOPDT: K={K:.3f}, tau={tau:.3f}s, theta={theta:.3f}s")
```
