# System Identification Model Evaluation

## Overview
Estimating a model is only the first step of system identification; validating that the model accurately represents the physical system is equally important. An invalid model will lead to poor controller performance or instability. 

Model evaluation consists of two primary phases:
1. **Residual Analysis**: Examining the prediction errors (residuals) to ensure they resemble white noise. If residuals are correlated with past inputs or themselves, the model has failed to capture some dynamics.
2. **Performance Metrics**: Quantifying the simulation or prediction accuracy using data *not* used during the training phase (cross-validation).

## 1. Residual Analysis
The residual $e(t)$ is the difference between the true measured output $y(t)$ and the one-step-ahead predicted output $\hat{y}(t|t-1)$.

For a model to be considered valid:
- **Autocorrelation function of residuals, $R_e(\tau)$**: Should be an impulse at $\tau = 0$ and zero everywhere else (within a 95% confidence interval). This implies the residuals are white noise.
- **Cross-correlation between input and residuals, $R_{ue}(\tau)$**: Should be zero for all lags $\tau$. If $R_{ue}(\tau)$ is significant, the model structure is likely too simple (e.g., missing poles/zeros or dead-time).

### Python Example
```python
import numpy as np
import matplotlib.pyplot as plt
from statsmodels.tsa.stattools import acf, ccf

def plot_residual_diagnostics(u, y_true, y_pred):
    residuals = y_true - y_pred
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Autocorrelation of residuals
    acf_vals = acf(residuals, nlags=30)
    ax1.stem(range(len(acf_vals)), acf_vals)
    ax1.axhline(1.96/np.sqrt(len(residuals)), color='r', linestyle='--')
    ax1.axhline(-1.96/np.sqrt(len(residuals)), color='r', linestyle='--')
    ax1.set_title("Autocorrelation of Residuals")
    
    # Cross-correlation between input and residuals
    ccf_vals = ccf(u.flatten(), residuals.flatten())[:30]
    ax2.stem(range(len(ccf_vals)), ccf_vals)
    ax2.axhline(1.96/np.sqrt(len(residuals)), color='r', linestyle='--')
    ax2.axhline(-1.96/np.sqrt(len(residuals)), color='r', linestyle='--')
    ax2.set_title("Cross-correlation between Input and Residuals")
    
    plt.tight_layout()
    plt.show()
```

## 2. Performance Metrics (Cross-Validation)
Always evaluate these metrics on a **validation dataset** that was not used to estimate the model parameters.

- **RMSE (Root Mean Square Error)**: Measures the absolute fit. Smaller is better.
  $$RMSE = \sqrt{\frac{1}{N}\sum_{t=1}^N (y(t) - \hat{y}(t))^2}$$

- **MAPE (Mean Absolute Percentage Error)**: Provides a scale-independent measure of error. Not suitable if $y(t)$ crosses zero.

- **FIT (Normalized Root Mean Square Error, NRMSE)**: Frequently used in control literature (e.g., MATLAB `compare`). Ranges from $-\infty$ to 100%. A fit of 100% is perfect; 0% is no better than predicting the mean of the data.
  $$FIT = \left(1 - \frac{\sqrt{\sum(y(t) - \hat{y}(t))^2}}{\sqrt{\sum(y(t) - \bar{y})^2}}\right) \times 100\%$$

### Python Example
```python
import numpy as np

def evaluate_model_fit(y_true, y_pred):
    y_true = np.array(y_true).flatten()
    y_pred = np.array(y_pred).flatten()
    
    rmse = np.sqrt(np.mean((y_true - y_pred)**2))
    
    y_mean = np.mean(y_true)
    fit_percent = (1 - (np.linalg.norm(y_true - y_pred) / np.linalg.norm(y_true - y_mean))) * 100
    
    # Safe MAPE calculation
    mask = y_true != 0
    mape = np.mean(np.abs((y_true[mask] - y_pred[mask]) / y_true[mask])) * 100
    
    print(f"RMSE: {rmse:.4f}")
    print(f"FIT : {fit_percent:.2f}%")
    print(f"MAPE: {mape:.2f}%")
    
    return {'rmse': rmse, 'fit': fit_percent, 'mape': mape}
```
