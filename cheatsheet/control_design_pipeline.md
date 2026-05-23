### 1. Requirements & Specifications (precedes any modeling)
Before touching data or equations, you must define **what "good control" means**:
- Time-domain specs: overshoot <= 10%, settling time <= 2 s, zero steady-state error.
- Frequency-domain specs: bandwidth >= 1 rad/s, phase margin >= 45^\circ, gain margin >= 6 dB.
- Constraint specs: actuator limits, rate limits, safety limits on states.
- Disturbance rejection and noise sensitivity bounds.
- Robustness: allowed performance degradation under parameter variations (e.g., mass +/-20%).

*Without this, you cannot choose the right identification or controller structure, nor evaluate success objectively.*

---

### 2. Experiment Design for Identification
Collecting data is not a passive step. You must design **the input signal and experiment conditions**:
- Excitation signal: PRBS, swept sine, multisine, chirp - chosen to persistently excite all relevant dynamics.
- Amplitude and duration: avoid nonlinear regions but achieve acceptable signal-to-noise ratio.
- Safety constraints: cannot break the plant during identification (amplitude limits, override trips).
- Open-loop vs. closed-loop data: if plant is unstable or integrator, you must identify under a stabilizing controller.

*You hinted at "collect data after multiple run", but the design of that experiment determines model quality.*

---

### 3. Model Validation (not just "estimate model")
After estimating parameters, you must **prove the model is good enough**:
- Residual analysis (whiteness, independence from input) to check for unmodeled dynamics.
- Cross-validation on a fresh dataset not used for estimation.
- Simulation response (output error) vs. one-step-ahead prediction - crucial for control.
- Uncertainty quantification: error bounds (frequency-domain uncertainty discs, parameter confidence intervals) for robust design.

*Without validation, a poorly fitted model will yield an apparently brilliant controller in simulation that fails in reality.*

---

### 4. Control-Oriented Model Transformation
The identified model is rarely in the exact form your control method needs:
- Convert continuous \leftrightarrow discrete (with proper sampling time selection).
- Model reduction (balanced truncation, Hankel norm) to get a low-order design model.
- Augmentation with disturbance models (integrators for offset-free tracking, ARIMA for MPC).
- Delay approximation (Padé, or exact state augmentation for MPC).
- Uncertainty representation (extracting multiplicative/additive weight for Hinf).

*This bridge between identification and controller design is often messy and iterative.*

---

### 5. Controller Structure Selection
Based on the requirements, plant complexity, and implementation constraints, you must **pick the controller architecture** (PID, LQG, MPC, Hinf, fuzzy, adaptive...). This is not a tuning step but a critical architectural decision:
- Does the plant have large dead time? -> Smith Predictor or MPC.
- Are there strong nonlinearities? -> gain scheduling, fuzzy, NMPC.
- Are all states measurable? -> state feedback or observer needed.
- Is computational power limited? -> fixed-point PID vs. heavy optimization.

*You mentioned "depending on criteria and budget", but the mapping from specs to controller type is a missing explicit step.*

---

### 6. Robustness & Worst-Case Analysis (beyond nominal performance)
Evaluating only nominal step response (MSE, ITAE) can be dangerously optimistic. You must also:
- **Monte Carlo simulations**: vary identified parameters within uncertainty bounds.
- **Worst-case analysis**: find parameter combinations that maximize sensitivity peak or cause instability.
- **Disc margins**: check how much simultaneous gain/phase variation the loop can tolerate.
- **Structured singular value (mu)** if you have a rigorous uncertainty model.

*Your "evaluation" step is good, but must include robustness measures beyond error integrals.*

---

### 7. Implementation & Real-Time Considerations
Moving from a mathematical control law to running code on hardware introduces new issues:
- **Discretization method**: Tustin, ZOH, matched pole-zero - chosen to preserve stability margins.
- **Anti-windup** (essential if the controller has integral action and actuators saturate).
- **Bumpless transfer** (between manual and automatic mode, or between gain-scheduled controllers).
- **Derivative filtering** and noise amplification (derivative on measurement not error).
- **Real-time scheduling**: computational delay, jitter, sample rate mismatch.
- **Finite word length effects** (fixed-point vs. floating-point).

*These are part of the design pipeline, not an afterthought, because they can completely change the effective dynamics.*

---

### 8. Commissioning and Fine-Tuning on Real Plant
Simulation is never perfect. The final step is **plant-side validation and tweaking**:
- Start with conservative gains (lower bandwidth, lower authority).
- Slowly increase performance while monitoring signals.
- Perform step tests, load disturbances, and setpoint changes on the real system.
- Fine-tune parameters using data-driven methods (IFT, VRFT) if the initial design is off.

---

### 9. Iteration and Feedback Loops (every step can send you back)
The full pipeline is **not linear**:
- If model validation fails -> redesign experiment or change model structure.
- If robust analysis shows poor stability margins -> relax performance specs or change controller structure.
- If implementation shows chatter or limit cycles -> add filter, adjust sampling rate, or return to design.

Your 3-step pipeline is essentially a single pass with evaluation at the end. Real design loops back repeatedly.

---

### 10. Performance Monitoring & Maintenance
After commissioning, the system should be monitored:
- Detect performance degradation (increased variance, sluggish response) and trigger re-identification or adaptive tuning.
- Auto-tuning routines to compensate for slow plant changes (wear, fouling, seasonal).

This "closes the loop" across the plant's entire lifecycle.

---

## Expanded Pipeline (Summary)
```
[Requirements & Specs]
    v
[Experiment Design & Data Collection]
    v
[System Identification] -> [Model Validation] (anticlockwise) (if fail, back to experiment/model structure)
    v
[Control-Oriented Model Transformation]
    v
[Controller Structure Selection]
    v
[Controller Design & Tuning] (using model + specs)
    v
[Robustness & Worst-Case Analysis] (Monte Carlo, mu) (anticlockwise) (if fail, back to specs or controller)
    v
[Implementation: discretization, anti-windup, bumpless transfer, real-time constraints]
    v
[Commissioning & Real-Plant Fine-Tuning]
    v
[Performance Monitoring & Adaptive Maintenance]
```