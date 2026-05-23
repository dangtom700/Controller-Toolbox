# Advanced Model Estimation Techniques
### Classical Machine Learning and Soft Computing Methods for System Identification

---

## Introduction

The polynomial and subspace structures surveyed in *System Identification Methods* assume that the dominant dynamics are linear, or that nonlinearities can be separated into a known structural form (Hammerstein, Wiener). In practice, many industrial plants are mildly to strongly nonlinear, operate across multiple regimes, or generate data whose noise statistics violate the white-noise assumption that underpins prediction-error minimisation (PEM).

This document surveys the complementary landscape: regularised linear estimators that handle high-dimensional regressors, kernel-based methods that generalise without committing to a parametric structure, neural network architectures that embed dynamical state, fuzzy and hybrid inference systems that blend interpretability with learning capacity, and ensemble methods drawn from classical machine learning. Each is described in terms of its estimation mechanism, the model structure it implicitly defines, and its suitability for control-oriented identification. Large-language-model and attention-based architectures are outside scope.

The target reader is assumed to be comfortable with state-space models, PEM, and matrix least squares; mathematical notation follows the discrete-time convention of the companion cheatsheet.

---

## Part I - Regularised and Bias-Corrected Linear Methods

---

## 1. Ridge, LASSO, and Elastic-Net Regression

**Mechanism.** Standard ARX/FIR identification minimises the unregularised squared residual. When the regressor dimension is large relative to the data length, or when regressors are nearly collinear, the ordinary least-squares solution exhibits high variance. Regularised estimators add a penalty on the parameter vector $\theta$:

$$\hat{\theta} = \underset{\theta}{\arg\min}\; \|\Phi\theta - Y\|_2^2 + \lambda \cdot \Omega(\theta)$$

| Method | $\Omega(\theta)$ | Effect |
|---|---|---|
| **Ridge** (Tikhonov) | $\|\theta\|_2^2$ | Shrinks all coefficients uniformly; dense solution |
| **LASSO** | $\|\theta\|_1$ | Promotes sparsity; selects a subset of regressors |
| **Elastic-Net** | $\alpha\|\theta\|_1 + (1-\alpha)\|\theta\|_2^2$ | Sparsity with grouping; LASSO + Ridge combined |

**Estimation.** Ridge has the closed-form solution $\hat{\theta} = (\Phi^T\Phi + \lambda I)^{-1}\Phi^T Y$, which regularises the matrix inversion and is always numerically stable. LASSO requires a convex but non-smooth solver (coordinate descent, ADMM, or LARS). Elastic-Net inherits both properties and is solved by the same methods as LASSO.

**Hyperparameter selection.** Cross-validation on a held-out validation segment; or, for Ridge, generalised cross-validation (GCV) provides a computationally efficient closed-form criterion.

**Applications.** High-dimensional FIR identification (long impulse responses); sparse ARX structure selection (LASSO automatically discards irrelevant lag terms); building energy models with many correlated weather inputs; batch process identification with many candidate product quality predictors.

**Pros.** Closed-form or convex; numerically robust; LASSO provides automatic order selection; no iterative optimisation.

**Cons.** Ridge does not perform variable selection; LASSO selects at most $N$ regressors (where $N$ = data length); neither captures nonlinear dynamics; $\lambda$ must be tuned.

---

## 2. Instrumental Variable (IV) Estimation

**Mechanism.** ARX-OLS is statistically biased whenever the noise $e(k)$ is correlated with the regressor vector $\varphi(k)$, which occurs when the true noise is coloured and the noise model is misspecified (common in practice). The IV estimator replaces $\Phi^T\Phi$ with $Z^T\Phi$ where $Z$ is an instrument matrix uncorrelated with the noise but correlated with the regressors:

$$\hat{\theta}_{\text{IV}} = (Z^T \Phi)^{-1} Z^T Y$$

**Choice of instruments.** Two standard constructions:
- *Delayed IV*: use $\varphi(k-d)$ shifted by $d$ steps (breaks noise correlation while preserving signal correlation).
- *Simulation-based IV (SRIVC)*: use predicted outputs from a preliminary model as instruments; this yields the refined IV with continuous-time parameterisation (SRIVC, Young 1970).

**Estimation.** IV is a single matrix inversion per iteration; SRIVC (and the discrete SRIV) iterate between parameter update and instrument generation until convergence, typically in fewer than 10 iterations.

**Applications.** Closed-loop identification where the feedback path correlates noise with the input; identification under correlated disturbances (vibration, electrical interference); MATLAB `iv4()`, `ivx()`.

**Pros.** Consistent (asymptotically unbiased) under mild conditions; computationally cheap; SRIV achieves maximum-likelihood efficiency for SISO OE models.

**Cons.** Efficiency loss relative to ML when instruments are weak; requires a preliminary model for simulation-based variants; MIMO IV is non-trivial.

---

## Part II - Kernel-Based and Probabilistic Methods

---

## 3. Support Vector Regression (SVR / epsilon-SVR)

**Mechanism.** SVR fits a function $f(x) = \langle w, \phi(x) \rangle + b$ such that all training residuals lie within an $\varepsilon$-insensitive tube, minimising model complexity (the 2-norm of $w$) subject to the constraint:

$$\min_{w,b,\xi}\; \tfrac{1}{2}\|w\|^2 + C \sum_k (\xi_k + \xi_k^*), \quad |y_k - f(x_k)| \leq \varepsilon + \xi_k, \; \xi_k \geq 0$$

The dual of this QP is solved in terms of kernel inner products $K(x_i, x_j) = \langle \phi(x_i), \phi(x_j) \rangle$, allowing the feature map $\phi$ to be implicit. Common kernels for dynamical systems:

| Kernel | Formula | Suited for |
|---|---|---|
| RBF (Gaussian) | $\exp(-\gamma\|x-x'\|^2)$ | Smooth nonlinearities |
| Polynomial | $(\langle x,x'\rangle + c)^d$ | Interaction terms |
| ANOVA | $\prod_{i}\kappa_i(x_i, x_i')$ | Structured input spaces |

**System ID usage.** For an $n$-step-ahead predictor, the regressor vector is $x_k = [y(k-1), \ldots, y(k-na), u(k-1), \ldots, u(k-nb)]^\top$, yielding a nonlinear ARX (NARX) predictor via SVR.

**Estimation.** Standard convex QP; $\mathcal{O}(N^3)$ for a dense kernel matrix; sparse solvers (SMO, LIBSVM) are $\mathcal{O}(N^2)$ in practice. Three hyperparameters: $C$ (margin trade-off), $\varepsilon$ (tube width), $\gamma$ (kernel bandwidth).

**Applications.** Soft-sensor identification (process quality from secondary measurements); engine emission models; short-horizon prediction in MPC when the dataset is small (hundreds to low thousands of samples).

**Pros.** Global optimum guaranteed; sparse solution (only support vectors matter); good generalisation with small datasets; kernel selection encodes prior knowledge.

**Cons.** $\mathcal{O}(N^2)$ memory for kernel matrix; no built-in noise model; extrapolation outside training data is poorly characterised; three hyperparameters to tune jointly.

---

## 4. Gaussian Process Regression (GPR)

**Mechanism.** GPR places a prior directly over functions. A Gaussian process is fully specified by a mean function $m(x)$ and covariance (kernel) function $k(x, x')$:

$$f(x) \sim \mathcal{GP}\bigl(m(x),\; k(x, x')\bigr)$$

Given noisy observations $y_k = f(x_k) + \varepsilon_k$, $\varepsilon_k \sim \mathcal{N}(0, \sigma_n^2)$, the posterior predictive distribution at a test point $x_*$ is analytically Gaussian:

$$p(f_* | x_*, X, Y) = \mathcal{N}\bigl(\mu_*, \sigma_*^2\bigr)$$

$$\mu_* = k_*^T(K + \sigma_n^2 I)^{-1}Y, \quad \sigma_*^2 = k_{**} - k_*^T(K + \sigma_n^2 I)^{-1}k_*$$

where $K_{ij} = k(x_i, x_j)$ and $k_* = [k(x_*, x_i)]$.

**Hyperparameter learning.** Kernel hyperparameters (lengthscales, signal variance, noise variance) are estimated by maximising the log marginal likelihood:

$$\log p(Y|X,\theta) = -\tfrac{1}{2}Y^T(K+\sigma_n^2 I)^{-1}Y - \tfrac{1}{2}\log|K+\sigma_n^2 I| - \tfrac{N}{2}\log 2\pi$$

This balances data fit against model complexity without held-out data.

**Applications in system ID.** Direct dynamics models $x_{k+1} = f(x_k, u_k) + \varepsilon$; Gaussian process state-space models (GP-SSM, Frigola 2015); MPC with uncertainty propagation (GP-MPC, Hewing 2020); active learning - query the operating point where $\sigma_*^2$ is largest.

**Pros.** Calibrated uncertainty (posterior variance usable in robust/stochastic MPC); automatic relevance determination (ARD kernel identifies irrelevant inputs); non-parametric (no structural commitment).

**Cons.** $\mathcal{O}(N^3)$ training cost; $\mathcal{O}(N^2)$ memory; degrades with large datasets (sparse GP approximations: Nyström, FITC, VFE address this); multi-step prediction requires uncertainty propagation (moment matching or sigma-point methods).

---

## 5. Relevance Vector Machine (RVM)

**Mechanism.** The RVM (Tipping 2001) is a sparse Bayesian linear model in a kernel basis. Rather than the SVR $\varepsilon$-tube, it places independent zero-mean Gaussian priors over each weight $w_i$ with a separate precision hyperparameter $\alpha_i$. Automatic Relevance Determination (ARD) drives irrelevant $\alpha_i \to \infty$, which zeros the corresponding weight - yielding a sparser solution than SVR, typically.

$$p(\theta | \alpha) = \prod_i \mathcal{N}(\theta_i | 0, \alpha_i^{-1}), \quad \hat{\alpha}_{\text{ML-II}} = \underset{\alpha}{\arg\max}\; p(Y | X, \alpha)$$

**Estimation.** Expectation-Maximisation or sequential sparse Bayesian learning (Tipping & Faul 2003); converges in fewer basis evaluations than SVM QPs for comparable sparsity.

**Applications.** Same domain as SVR, but where probabilistic output (predictive variance) is needed without full GPR cost; classification in fault detection.

**Pros.** Sparser than SVR; produces calibrated predictive variance; kernel is not required to be Mercer (RVM is not constrained to PSD kernels).

**Cons.** Non-convex hyperparameter optimisation; sensitive to initialisation; less community support than GPR or SVR.

---

## Part III - Neural Network Architectures for Dynamics

---

## 6. Multilayer Perceptron as Static Nonlinear Block

**Mechanism.** A feedforward MLP with one hidden layer is a universal function approximator (Cybenko 1989). As a system ID component it identifies the static nonlinearity in Hammerstein or Wiener structures:

$$\hat{y}(k) = f_{\text{NN}}\bigl(\varphi(k)\bigr), \quad \varphi(k) = [y(k-1), \ldots, u(k-nb)]^\top$$

where $f_{\text{NN}}$ is a composition of affine maps and pointwise activation functions (ReLU, tanh, sigmoid).

**Estimation.** Mini-batch stochastic gradient descent (SGD/Adam) on the mean squared prediction error; backpropagation computes gradients. Regularisation: L2 weight decay (equivalent to Gaussian weight prior), dropout, or early stopping on a validation split.

**Network sizing.** For dynamics, hidden-layer width $h \approx 2(na + nb) + 1$ is a common starting heuristic; cross-validation or BIC-analogues determine final architecture.

**Applications.** Static soft sensors; gain-scheduling lookup replacement; nonlinear inverse-model feed-forward in mechatronics.

**Pros.** Universal approximation; mature tooling (PyTorch, TensorFlow, scikit-learn); GPU acceleration for large datasets.

**Cons.** No built-in dynamics (memory must be supplied via the regressor vector); extrapolation is unpredictable; non-convex training; model uncertainty not quantified.

---

## 7. NARX Neural Networks

**Mechanism.** The NARX (Nonlinear AutoRegressive with eXogenous Input) neural network extends the MLP to full dynamic identification by including past outputs in the regressor:

$$\hat{y}(k) = f_{\text{NN}}\bigl(y(k-1), \ldots, y(k-na),\; u(k-1), \ldots, u(k-nb)\bigr)$$

Two training modes:

| Mode | Description | Bias/variance |
|---|---|---|
| **Series-parallel** (open-loop) | True past outputs used during training | Low bias; may not generalise to closed-loop |
| **Parallel** (simulation) | Own predicted outputs fed back | Unbiased for long-range prediction; harder to train |

**Estimation.** Backpropagation-through-time (BPTT) for the parallel (recurrent) mode; standard backprop for series-parallel. Dynamic backpropagation is numerically stiff for large $na$.

**Applications.** Engine torque/emission modelling; chemical batch dynamics; MPC internal model when a physics-based model is unavailable; real-time adaptive control with online NARX update.

**Pros.** Captures arbitrary nonlinear dynamics; series-parallel training is convex per layer; well-studied stability conditions for closed-loop use (Suykens, De Moor 1996).

**Cons.** Parallel mode training is slow and unstable for long time series; no explicit uncertainty; requires care to avoid divergence in long-horizon simulation.

---

## 8. Echo State Networks and Reservoir Computing

**Mechanism.** An Echo State Network (ESN, Jaeger 2001) is a recurrent neural network where the internal (reservoir) weights are fixed at random initialisation; only the output (readout) weights $W_{\text{out}}$ are trained:

$$x(k+1) = (1-\alpha)x(k) + \alpha\,\sigma\bigl(W_{\text{res}} x(k) + W_{\text{in}} u(k)\bigr)$$
$$\hat{y}(k) = W_{\text{out}} x(k)$$

where $\alpha$ is the leaking rate and $\sigma$ is a pointwise nonlinearity (tanh). The reservoir state $x(k)$ is a high-dimensional nonlinear transformation of the input history; the output layer is trained by ridge regression - a single linear least-squares solve.

**Echo State Property.** The spectral radius of $W_{\text{res}}$ must be less than 1 (typically 0.7-0.99) to ensure the network fades memory of initial conditions and converges to a unique trajectory driven by the input.

**Applications.** Chaotic system identification (Lorenz, Mackey-Glass); time-series forecasting when training data is abundant; real-time adaptive filtering; surrogate models for fluid dynamics.

**Pros.** Training is a single ridge regression - convex, fast, no gradient required; captures long-range temporal dependencies; reservoir size easily scaled.

**Cons.** Reservoir weights are not optimised - large reservoirs may be needed; hyperparameters (spectral radius, leaking rate, input scaling) require tuning; no uncertainty quantification.

---

## 9. Extreme Learning Machines (ELM)

**Mechanism.** ELM (Huang et al. 2006) is a single-hidden-layer feedforward network where the input-to-hidden weights are randomly assigned and fixed; only the hidden-to-output weights $\beta$ are solved by least squares:

$$H\beta = Y, \quad \hat{\beta} = H^\dagger Y$$

where $H_{ki} = g(w_i^T x_k + b_i)$ is the hidden-layer activation matrix, $g$ is a nonlinear activation, and $H^\dagger$ is the Moore-Penrose pseudoinverse. Ridge regularisation is standard: $\hat{\beta} = (H^TH + \lambda I)^{-1}H^TY$.

**Applications.** Rapid prototype identification when training time is constrained; soft sensors updated online (OS-ELM, sequential update without full retraining); fault diagnosis classifiers.

**Pros.** Training is one matrix inversion - extremely fast; no gradient, no local minima; OS-ELM variant supports streaming data.

**Cons.** Universal approximation requires very wide hidden layers; random initialisation introduces variance between runs; accuracy generally inferior to tuned MLP on the same dataset; no uncertainty.

---

## Part IV - Fuzzy and Hybrid Systems

---

## 10. Takagi-Sugeno (TS) Fuzzy Model Identification

**Mechanism.** A TS fuzzy model represents a nonlinear system as a blended collection of local linear models, each active in a fuzzy region of the operating space. For $r$ rules:

$$\text{Rule } i: \text{ IF } x \text{ is } A_i \text{ THEN } \hat{y} = a_i^T x + b_i$$

$$\hat{y}(x) = \frac{\sum_{i=1}^{r} \mu_i(x)\,(a_i^T x + b_i)}{\sum_{i=1}^{r} \mu_i(x)}$$

where $\mu_i(x)$ is the membership degree to cluster $i$ (typically Gaussian or trapezoidal).

**Identification procedure:**
1. **Structure identification** - cluster the input-output data space using Fuzzy C-Means (FCM) or Gustafson-Kessel (ellipsoidal clusters) to determine the number of rules $r$ and the antecedent membership functions $\mu_i$.
2. **Consequence identification** - for each cluster, solve a weighted least-squares problem (weight = $\mu_i$) to obtain $a_i, b_i$.
3. **Validation and simplification** - merge clusters whose local models are nearly identical; prune rules with low firing strength.

**Applications.** Gain-scheduled MPC linearisation; nonlinear valve and actuator models; rule-based control systems where interpretability is a regulatory requirement (pharmaceutical, aerospace).

**Pros.** Interpretable rules; bridges engineering knowledge and data; consequent parameters are solved by linear regression; stability analysis available (Lyapunov, LMI).

**Cons.** Cluster number $r$ must be pre-specified (or found by iterating FCM with increasing $r$); curse of dimensionality in antecedent space; blending may smooth out sharp transitions.

---

## 11. ANFIS - Adaptive Neuro-Fuzzy Inference System

**Mechanism.** ANFIS (Jang 1993) implements a Takagi-Sugeno fuzzy system in a five-layer feedforward network, enabling gradient-based learning of both the antecedent membership parameters and the consequent linear coefficients simultaneously. The network layers are:

| Layer | Function |
|---|---|
| 1. Fuzzification | Compute $\mu_i(x)$ - sigmoid, Gaussian, or generalised bell |
| 2. Rule firing | $w_i = \prod_j \mu_{ij}(x_j)$ |
| 3. Normalisation | $\bar{w}_i = w_i / \sum w_i$ |
| 4. Defuzzification | $\bar{w}_i (a_i^T x + b_i)$ |
| 5. Summation | $\hat{y} = \sum_i \bar{w}_i (a_i^T x + b_i)$ |

**Estimation.** Hybrid learning: consequent parameters ($a_i, b_i$) solved by least squares in the forward pass (fast); antecedent parameters ($\mu$ shape) updated by backpropagation in the backward pass. Converges faster than pure gradient descent on TS models.

**Applications.** Nonlinear actuator models (hydraulics, pneumatics); battery state-of-charge estimation; inverse kinematics for robot arms; any regression task where TS fuzzy structure is desired with automatic membership tuning.

**Pros.** Combines interpretability of TS rules with gradient-based membership learning; hybrid training is faster than pure backprop; well-supported in MATLAB `anfis()`.

**Cons.** Requires a fixed grid or pre-clustered antecedent partitioning; scales poorly to high input dimension ($r$ grows exponentially with the number of inputs unless restricted to a first-order TS structure); prone to overfitting on small datasets without regularisation.

---

## Part V - Ensemble and Evolutionary Methods

---

## 12. Random Forests for System Identification

**Mechanism.** A Random Forest (Breiman 2001) is an ensemble of $T$ regression trees, each trained on a bootstrap sample of the data and using a random subset of $m$ features at each split. The prediction is the average across trees:

$$\hat{y}(x) = \frac{1}{T}\sum_{t=1}^{T} h_t(x)$$

For dynamical identification, $x = \varphi(k) = [y(k-1), \ldots, u(k-nb)]^\top$, yielding a nonlinear NARX-type predictor. Out-of-bag (OOB) samples provide an unbiased estimate of generalisation error without a separate validation set.

**Feature importance.** Mean decrease in impurity (MDI) or permutation importance across trees measures each lag's contribution to predictive accuracy - a data-driven lag selection criterion.

**Applications.** Nonlinear process control surrogate models; fault classification from sensor time series; gain-schedule identification where the scheduling variable is unknown.

**Pros.** Strong generalisation with default hyperparameters; built-in OOB error estimate; interpretable feature importance; robust to outliers and irrelevant inputs; trivially parallelisable.

**Cons.** Discontinuous piecewise-constant prediction surface (not smooth); poor extrapolation outside training envelope; large memory footprint for many trees; no explicit uncertainty model (prediction intervals require quantile forests).

---

## 13. Gradient Boosted Trees (GBT / XGBoost)

**Mechanism.** GBT (Friedman 2001) builds an additive model by sequentially fitting shallow regression trees to the negative gradient of the loss:

$$F_m(x) = F_{m-1}(x) + \eta \cdot h_m(x), \quad h_m = \underset{h}{\arg\min}\sum_k \left[\partial_{F}L(y_k, F_{m-1}(x_k)) - h(x_k)\right]^2$$

where $\eta$ is the learning rate (shrinkage) and $L$ is typically squared error for regression. XGBoost and LightGBM add L1/L2 regularisation on tree weights and leaf values.

**Applications.** Kaggle-style tabular regression; soft-sensor development when data volume is moderate (thousands to tens of thousands of samples); ranking and anomaly scoring in predictive maintenance.

**Pros.** Best-in-class on tabular data (benchmark results); handles missing values; native feature importance; extensive hyperparameter control.

**Cons.** More hyperparameters than Random Forest (depth, $\eta$, number of trees, regularisation); slower to train (sequential boosting); same extrapolation and discontinuity weaknesses as Random Forest; no probabilistic output without post-processing.

---

## 14. Evolutionary and Genetic Algorithm Identification

**Mechanism.** Genetic Algorithms (GA) and related methods (Differential Evolution, CMA-ES) treat the parameter vector $\theta$ as a chromosome and apply selection, crossover, and mutation operators to evolve a population toward minimum prediction error:

$$J(\theta) = \sum_k \bigl(y(k) - \hat{y}(k|\theta)\bigr)^2 + \lambda\,\Omega(\theta)$$

Beyond parameter estimation for fixed structures, Genetic Programming (GP, not to be confused with Gaussian Process) evolves the *model structure* itself - operators, functions, and lags are all subject to evolution.

| Method | Searches | Strengths |
|---|---|---|
| GA / DE | Parameter space of fixed structure | Escapes local minima; handles integer variables (model order) |
| CMA-ES | Continuous parameter space, adapts covariance | Best for moderate-dimensional smooth landscapes |
| Genetic Programming | Structure + parameters simultaneously | Symbolic regression; interpretable closed-form models |

**Applications.** Global PEM when gradient-based optimisation stalls in local minima; simultaneous model-order and parameter selection; symbolic regression for explainable process models; multi-objective identification (accuracy vs. complexity).

**Pros.** Gradient-free; naturally handles discrete structure choices; parallel evaluation of population; produces diverse candidate models.

**Cons.** Expensive in function evaluations; no convergence guarantee; solution quality depends strongly on operator design; Genetic Programming produces complex expressions that may overfit.

---

## Part VI - Non-Parametric and Local Methods

---

## 15. Locally Weighted Regression (LWR / LOESS)

**Mechanism.** LWR (Cleveland 1979; Atkeson et al. 1997) fits a separate local polynomial model at each query point $x_*$, weighting training points by their distance:

$$\hat{\theta}(x_*) = \underset{\theta}{\arg\min}\sum_k w_k(x_*)\, (y_k - \phi_k^T \theta)^2$$

$$w_k(x_*) = K\!\left(\frac{\|x_k - x_*\|}{\beta}\right), \quad K(u) = \max(0, (1-u^3)^3) \quad \text{(tricube)}$$

The bandwidth $\beta$ controls the locality. Setting $\beta \to \infty$ recovers global OLS; $\beta \to 0$ interpolates exactly.

For on-line use, LWPR (Locally Weighted Projection Regression, Vijayakumar & Schaal 2000) maintains a set of receptive fields and updates them incrementally without storing the full dataset.

**Applications.** Real-time adaptive inverse dynamics for robot manipulators; operating-point-dependent linearisation for gain-scheduled MPC; exploratory identification of unknown nonlinear maps before committing to a parametric structure.

**Pros.** No global structure assumed; adapts automatically to varying local dynamics; LWPR supports real-time online learning.

**Cons.** Requires storing or summarising the training dataset for each query; bandwidth selection is critical; high-dimensional input spaces require large datasets to achieve adequate local coverage (curse of dimensionality).

---

## Comparison and Selection Guide

| Method | Data need | Uncertainty | Structure | Dynamics | Interpretability |
|---|---|---|---|---|---|
| Ridge / LASSO | Low | No (point est.) | Linear | Via lagged regressors | High |
| IV | Low | No | Linear | Yes | High |
| SVR | Low-Med | No | Kernel | Via NARX setup | Low |
| GPR | Low-Med | **Yes** | Kernel | Via NARX setup | Low |
| RVM | Low-Med | **Yes** | Sparse kernel | Via NARX setup | Low |
| MLP | Med-High | No | NN | Via NARX setup | Low |
| NARX-NN | Med-High | No | NN | **Native** | Low |
| ESN | High | No | Random reservoir | **Native** | Low |
| ELM | Med | No | Random hidden | Via NARX setup | Low |
| TS Fuzzy | Med | No | Blended linear | Via TS-NARX | **High** |
| ANFIS | Med | No | TS + gradient | Via NARX setup | **Medium** |
| Random Forest | Med-High | Partial (QRF) | Ensemble tree | Via NARX setup | Medium |
| GBT / XGBoost | Med-High | Partial | Ensemble tree | Via NARX setup | Medium |
| Evolutionary | Any | No | Any | Any | Variable |
| LWR / LWPR | High | Partial | Local linear | Via local NARX | **High** |

**Decision heuristic:**

1. **Start linear.** Ridge or LASSO for high-dimensional regressors; IV when closed-loop or coloured noise bias is suspected. Always validate on held-out data before adding complexity.

2. **Add structure for known nonlinearity.** TS fuzzy or ANFIS when operating regimes are physically interpretable. Hammerstein-Wiener (see companion cheatsheet) when input or output nonlinearity location is known.

3. **Go kernel or probabilistic if data is scarce.** GPR provides calibrated uncertainty for MPC constraint tightening; SVR generalises well with small datasets.

4. **Use neural networks when data volume allows.** NARX-NN or ESN for strongly nonlinear dynamics with thousands of samples or more; regularise aggressively to avoid overfitting.

5. **Use ensembles for robustness.** Random Forests or GBT when a reliable point prediction with no physics knowledge is needed; accept the loss of extrapolation and smoothness.

6. **Use evolutionary methods as a last resort.** When gradient-based PEM fails to converge, or when simultaneous structure and parameter search is required.

---

## Conclusion

The methods in this document extend the classical identification toolbox toward data-driven territory. The key principle remains unchanged from parametric identification: **model complexity must be matched to data informativeness**. A GPR model with 15 data points can outperform a 10-layer NARX network, and a well-specified ARX can outperform a Random Forest when the dynamics are genuinely linear.

Validation protocol does not change: always separate training, validation (for hyperparameter selection), and test data. Simulate the identified model in open loop over the test set and compare the simulated output against measured output - not one-step-ahead predictions, which are optimistic indicators of true model quality for control applications.

For control-oriented identification specifically, prefer models that generalise well in simulation (OE-like training criterion) over models that minimise one-step-ahead error (ARX-like). Uncertainty-aware models (GPR, RVM) enable robust MPC constraint tightening and active exploration - a capability that purely point-estimate models cannot provide.

---

*See also:* `system_identification.md` (classical parametric structures), `tuning_methods.md` (using identified models for controller design).
