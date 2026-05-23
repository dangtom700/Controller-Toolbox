# ARMAX Identification Method

## Overview
ARMAX (AutoRegressive Moving Average with eXogenous inputs) is a linear parametric model structure used for system identification. It extends the simpler ARX model by including a moving average (MA) term to model colored (correlated) noise disturbances, which are common in real-world physical systems. This makes ARMAX particularly suitable when measurement noise or unmeasured disturbances do not behave like pure white noise.

The general mathematical form is:
`A(q)y(t) = B(q)u(t) + C(q)e(t)`

Where `A(q)`, `B(q)`, and `C(q)` are polynomials in the shift operator `q`, `u(t)` is the input, `y(t)` is the output, and `e(t)` is white noise.

## Input Requirements
- **Data Source**: CSV file containing time-series input and output data.
- **Format / Content**: The file must contain columns representing the system inputs and outputs. For a SISO (Single-Input Single-Output) system, the expected format is:
  ```csv
  time,u,y
  0.0,1.5,0.1
  0.1,1.5,0.4
  ...
  ```
  *Note: The time column is optional but highly recommended for specifying the sampling interval.*

## Return Value
- Returns an **ARMAX Model** object (e.g., from `sysidentpy` or a custom C++ structure depending on the implementation backend).
- The model object contains the identified polynomial coefficients for `A`, `B`, and `C`, along with statistical metrics of the fit.
- It provides methods to simulate the system response (`predict()`) or generate a state-space representation.

## Example Usage
This example demonstrates how one might use Python to fit an ARMAX model. In the context of our C++ controller project, this identified model would then be passed to the C++ tuning utilities.

```python
import pandas as pd
from sysidentpy.model_structure_selection import FROLS
from sysidentpy.basis_function import Polynomial
from sysidentpy.metrics import root_relative_squared_error

# 1. Load data
data = pd.read_csv('data/plant_response.csv')
u = data['u'].values.reshape(-1, 1)
y = data['y'].values.reshape(-1, 1)

# 2. Configure ARMAX Identification (using sysidentpy as an example framework)
basis_function = Polynomial(degree=1)
model = FROLS(
    order_selection=True,
    n_terms=5,
    extended_least_squares=True, # Enables the Moving Average (C polynomial) estimation
    ylag=2, xlag=2, elag=2,      # Orders of A, B, and C polynomials
    info_criteria='aic',
    estimator='least_squares',
    basis_function=basis_function
)

# 3. Fit the model
model.fit(X=u, y=y)

# 4. View results
print(model.results())
```
