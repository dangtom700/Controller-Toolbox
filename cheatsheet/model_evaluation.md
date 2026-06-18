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

---

## 3. Kalman Innovation Whiteness Test

When the estimator is a Kalman Filter (KF, EKF, or UKF), the innovation sequence
$$\nu_k = y_k - C\hat{x}_{k|k-1}$$
should be zero-mean white noise with covariance $S_k = CP_{k|k-1}C^T + R$ if the model is correct. Deviations from this indicate model mismatch, unmodelled dynamics, or wrong noise statistics.

**Tests to apply:**

- **Normalised innovation squared (NIS):** $\epsilon_k = \nu_k^T S_k^{-1} \nu_k$ should follow a $\chi^2(p)$ distribution, where $p$ is the output dimension. Mean NIS should be near 1.0; sustained values above 1.5 indicate overconfidence in the model.
- **ANEES (Average Normalised Estimation Error Squared):** applies to state estimation benchmarks where the true state is known.
- **ACF of innovations:** same test as residual ACF in Section 1 - the innovations should be uncorrelated across time.

```python
import numpy as np
from scipy import stats

def nis_test(innovations, S_matrices, p_output):
    """
    innovations: list of numpy arrays (p,) - one per step
    S_matrices:  list of numpy arrays (p, p) - innovation covariance per step
    """
    N = len(innovations)
    nis_vals = np.array([
        innovations[k] @ np.linalg.solve(S_matrices[k], innovations[k])
        for k in range(N)
    ])
    
    mean_nis = np.mean(nis_vals)
    # 95% confidence interval for mean NIS under chi2(p)
    ci_lo = stats.chi2.ppf(0.025, df=p_output) / p_output
    ci_hi = stats.chi2.ppf(0.975, df=p_output) / p_output
    
    consistent = ci_lo <= mean_nis <= ci_hi
    print(f"Mean NIS: {mean_nis:.3f}  (95% CI: [{ci_lo:.3f}, {ci_hi:.3f}])")
    print(f"Filter consistent: {consistent}")
    return mean_nis, consistent
```

---

## 4. Real-Time Mismatch Detection (Toolbox-Native)

The tests in Sections 1-3 are applied **offline** to a batch of data. For closed-loop systems that must detect model drift **online**, the toolbox provides `MismatchDetector` (D1) - a CUSUM chart on the normalised innovation norm that raises a sticky alarm when the model and plant diverge.

### Enabling on KalmanFilter

```python
import ctrl_toolbox as ctrl

kf = ctrl.KalmanFilter(A, B, C, Q, R, P0, x0)

# Calibrate sigma from a nominal run: std(||innov||/sqrt(p)) over 200+ steps
kf.enable_mismatch_detection(
    sigma=1.0,        # expected std of ||innov||/sqrt(p) under nominal model
    k_cusum=0.5,      # CUSUM reference value (half expected shift)
    h_threshold=5.0   # alarm threshold
)

for k in range(N):
    kf.predict(u[k])
    kf.update(y[k])
    
    if kf.mismatch_detected():
        score = kf.mismatch_score()
        print(f"Mismatch at step {k}, CUSUM score={score:.2f}")
        kf.reset_mismatch_detector()
        # trigger GreyBoxEstimator re-fit or HybridModelTrainer update
```

### Enabling on MovingHorizonEstimator

```python
mhe = ctrl.MovingHorizonEstimator(...)
mhe.enable_mismatch_detection(sigma=1.0, k_cusum=0.5, h_threshold=5.0)
# identical API: mismatch_detected(), mismatch_score(), reset_mismatch_detector()
```

### Threshold calibration

| Context | `sigma` | `k_cusum` | `h_threshold` | Expected ARL0 |
|---|---|---|---|---|
| Standard (moderate sensitivity) | from data | 0.5 * sigma | 5 * sigma | ~500 steps |
| Sensitive (catch slow drift early) | from data | 0.25 * sigma | 3 * sigma | ~150 steps |
| Conservative (only catch large jumps) | from data | 0.75 * sigma | 8 * sigma | ~2000 steps |

ARL0 = Average Run Length under null (no mismatch) - the expected number of steps before a false alarm.

*See also:* `mismatch_detection.md` for full API, CUSUM tuning, integration with DAE-EKF, and the monitoring + re-identification loop.
