"""Quick smoke test for ctrl_toolbox Python bindings."""
import sys
import os

# On Windows, the MinGW runtime DLLs (libstdc++, libgcc, libwinpthread) must be
# reachable before importing the .pyd.  Add the MinGW bin dir explicitly.
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

# Load the .pyd / .so from the build output directory.
# Try build_py/bindings first (CI workflow), then build/bindings (local dev).
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _build_dir in ['build_py', 'build']:
    _p = os.path.join(_root, _build_dir, 'bindings')
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
        break

import ctrl_toolbox as ctrl
import numpy as np

# 1. Feature flags
f = ctrl.features()
print('features:', f)
assert isinstance(f, dict), "features() should return a dict"
assert 'fuzzy' in f, "features() should contain 'fuzzy'"

# 2. TransferFunction + tf2ss
tf = ctrl.TransferFunction([0.2], [1.0, -0.8], 0.01)
sys_ss = ctrl.tf2ss(tf)
print(f'StateSpace n={sys_ss.state_size()} m={sys_ss.input_size()} p={sys_ss.output_size()}')
assert sys_ss.state_size() == 1

# 3. ssStepCopy (non-mutating)
x = np.zeros(sys_ss.state_size())
y, x_next = ctrl.ss_step_copy(sys_ss, x, np.array([1.0]))
print(f'ssStepCopy y={float(y[0]):.4f}  x_next={x_next}')
assert abs(float(y[0]) - 0.0) < 1e-9    # D=0, y = C*0 + D*u = 0
assert abs(float(x_next[0]) - 1.0) < 1e-9  # controllable canonical: B=1, x_next = A*0 + B*1 = 1

# 4. DiscretePID
p = ctrl.PIDParams()
p.Kp = 2.0
p.Ki = 0.5
p.Kd = 0.0
pid = ctrl.DiscretePID(p, 0.01)
u = pid.compute(1.0)
print(f'PID compute(1.0) = {u:.4f}')
assert u > 0

# 5. KalmanFilter plain-reference step overload
Q = np.eye(1) * 1e-4
R = np.eye(1) * 0.01
kf = ctrl.KalmanFilter(sys_ss, Q, R)
kf.step(np.array([0.1]), np.array([0.5]), np.array([0.5]))  # 3-arg overload
print(f'KF state after step: {kf.state()}')
assert kf.state().shape == (1,)

# 6. Observer shared_ptr lifetime (Python subclass)
class Logger(ctrl.IControllerObserver):
    def __init__(self):
        super().__init__()
        self.calls = []
    def on_compute(self, u, signal):
        self.calls.append((u, signal))

obs = Logger()
pid2 = ctrl.DiscretePID(p, 0.01)
pid2.attach_observer(obs)
pid2.compute(0.5)
assert len(obs.calls) == 1, f"Expected 1 callback, got {len(obs.calls)}"
print(f'Observer got u={obs.calls[0][0]:.4f} signal={obs.calls[0][1]:.4f}')

# 7. Python subclass of IController
class Gain(ctrl.IController):
    def __init__(self, k, ts):
        super().__init__()
        self._k = k
        self._ts = ts
    def compute(self, signal):
        return self._k * signal
    def reset(self):
        pass
    def sample_time(self):
        return self._ts

gain = Gain(3.0, 0.01)
assert abs(gain.compute(2.0) - 6.0) < 1e-12
print(f'Python IController subclass: gain.compute(2.0) = {gain.compute(2.0)}')

# ---------------------------------------------------------------------------
# 8. DiscreteMPC
# ---------------------------------------------------------------------------
# Stable 2nd-order plant: damped oscillator ZOH @ Ts=0.1s
# Continuous: x_dot = Ac*x + Bc*u,  Ac=[[0,1],[-1,-1.5]],  Bc=[[0],[1]]
# ZOH @ 0.1s gives discrete poles at ~0.926+-0.062j (stable)
Ts2 = 0.1
Ac = np.array([[0.0, 1.0], [-1.0, -1.5]])
Bc = np.array([[0.0], [1.0]])
Cc = np.array([[1.0, 0.0]])
Dc = np.zeros((1, 1))
plant2 = ctrl.c2d(ctrl.StateSpace(Ac, Bc, Cc, Dc, 0.0), Ts2, ctrl.C2dMethod.ZOH)
mp = ctrl.MPCParams()
mp.Np = 15; mp.Nc = 4; mp.rho_y = 1.0; mp.rho_u = 0.01
mp.uMin = -5.0; mp.uMax = 5.0
mpc = ctrl.DiscreteMPC(plant2, mp)

x0 = np.zeros(plant2.state_size())
r0 = np.ones(plant2.output_size())
u_mpc = mpc.compute_ref(x0, r0)
print(f'MPC compute_ref -> u={u_mpc}')
assert u_mpc.shape == (plant2.input_size(),), "MPC output wrong shape"
assert mpc.last_qp_converged(), "MPC QP did not converge"
assert mp.Np == 15
print(f'MPCParams: Np={mp.Np}, Nc={mp.Nc}, rho_y={mp.rho_y}')

# ---------------------------------------------------------------------------
# 9. DiscreteLQR + LQRAdapter
# ---------------------------------------------------------------------------
lqr_p = ctrl.LQRParams()
lqr_p.Q = np.diag([10.0, 1.0])
lqr_p.R = np.array([[0.1]])
lqr = ctrl.DiscreteLQR(plant2, lqr_p)
print(f'LQR dare_converged={lqr.dare_converged()}, K={lqr.gain_matrix()}')
assert lqr.dare_converged(), "LQR DARE did not converge"
assert lqr.gain_matrix().shape == (plant2.input_size(), plant2.state_size())

x_state = np.zeros(plant2.state_size())
u_lqr = lqr.compute(x_state)
print(f'LQR compute(zeros) = {u_lqr}')
assert u_lqr.shape == (plant2.input_size(),)

# LQRAdapter with Python lambda callbacks
x_current = np.array([0.1, 0.05])
x_ref_val  = np.zeros(plant2.state_size())
adapter = ctrl.LQRAdapter(lqr,
    state_fn=lambda: x_current,
    ref_fn=lambda: x_ref_val)
u_adapt = adapter.compute_vec()
print(f'LQRAdapter compute_vec() = {u_adapt}')
assert u_adapt.shape == (plant2.input_size(),)
assert adapter.is_healthy()

# ---------------------------------------------------------------------------
# 10. DiscreteLQG
# ---------------------------------------------------------------------------
Q_noise = np.eye(plant2.state_size()) * 1e-4
R_noise = np.eye(plant2.output_size()) * 0.01
lqg = ctrl.DiscreteLQG(plant2, lqr_p, Q_noise, R_noise)
u_prev_lqg = np.zeros(plant2.input_size())
y_meas = np.array([0.1])
u_lqg = lqg.step(y_meas, u_prev_lqg)
print(f'LQG step -> u={u_lqg}, x^={lqg.state_estimate()}')
assert u_lqg.shape == (plant2.input_size(),)
assert lqg.state_estimate().shape == (plant2.state_size(),)

# ---------------------------------------------------------------------------
# 11. GeneralizedPredictiveController (GPC)
# ---------------------------------------------------------------------------
gp = ctrl.GPCParams()
gp.Np = 15; gp.Nu = 4; gp.rho_y = 1.0; gp.rho_u = 0.1; gp.alpha = 0.2
gpc = ctrl.GeneralizedPredictiveController(plant2, gp)
u_gpc = gpc.compute_ref(0.0, 1.0)   # y=0 (at origin), r=1 (step)
print(f'GPC compute_ref(y=0, r=1) = {u_gpc:.4f}')
assert isinstance(u_gpc, float), "GPC output should be scalar"
assert gpc.last_qp_converged(), "GPC QP did not converge"
aug_state = gpc.augmented_state()
print(f'GPC augmented state shape: {aug_state.shape}')

# ---------------------------------------------------------------------------
# 12. ControllerStack (Supervisory fallback)
# ---------------------------------------------------------------------------
stack = ctrl.ControllerStack(ctrl.StackMode.Supervisory, 0.01)
pid3 = ctrl.DiscretePID(p, 0.01)
stack.add_controller(
    pid3, "PID_fallback", weight=1.0, condition=None)
u_stack = stack.compute(0.5)
print(f'ControllerStack(Supervisory) compute(0.5) = {u_stack:.4f}')
assert isinstance(u_stack, float)
print(f'Active controller: {stack.active_controller_name()}')

# Weighted stack
stack_w = ctrl.ControllerStack(ctrl.StackMode.Weighted, 0.01)
pid_a = ctrl.DiscretePID(p, 0.01)
pid_b = ctrl.DiscretePID(p, 0.01)
stack_w.add_controller(pid_a, "A", weight=0.7)
stack_w.add_controller(pid_b, "B", weight=0.3)
u_w = stack_w.compute(1.0)
print(f'ControllerStack(Weighted) compute(1.0) = {u_w:.4f}')
assert abs(u_w - pid_a.last_output()) < 0.01, "Weighted blend should match single PID (both equal)"

# StackMode enum values
assert ctrl.StackMode.Supervisory != ctrl.StackMode.Additive
assert ctrl.StackMode.Weighted != ctrl.StackMode.Supervisory
print(f'StackMode.Supervisory = {ctrl.StackMode.Supervisory}')

# ---------------------------------------------------------------------------
# 13. EKF + UKF  (if CTRL_HAS_ADVANCED_KALMAN)
# ---------------------------------------------------------------------------
feats = ctrl.features()
if feats.get('advanced_kalman', False):
    Ts_ekf = 0.01
    A_lin = np.array([[1.0, Ts_ekf], [0.0, 1.0]])
    B_lin = np.array([[0.0], [Ts_ekf]])
    C_lin = np.array([[1.0, 0.0]])

    def f_lin(x, u): return A_lin @ x + B_lin @ u
    def h_lin(x, u): return C_lin @ x
    def Fj(x, u):   return A_lin
    def Hj(x, u):   return C_lin

    Q_ekf = np.eye(2) * 1e-4
    R_ekf = np.eye(1) * 0.01

    ekf = ctrl.ExtendedKalmanFilter(n=2, p=1,
        f=f_lin, h=h_lin, F_jac=Fj, H_jac=Hj,
        Q=Q_ekf, R=R_ekf, Ts=Ts_ekf)

    ekf.step(np.array([0.1]), np.array([0.5]))
    x_ekf = ekf.state()
    print(f'EKF state after step: {x_ekf}')
    assert x_ekf.shape == (2,), "EKF state wrong shape"

    # Numerical Jacobian utility
    Fj_num = ctrl.ExtendedKalmanFilter.numerical_jacobian(
        lambda xx: A_lin @ xx, np.zeros(2))
    print(f'EKF numerical_jacobian:\n{Fj_num}')
    assert Fj_num.shape == (2, 2)
    assert np.allclose(Fj_num, A_lin, atol=1e-6), "Numerical Jacobian inaccurate"

    # UKF
    ukf = ctrl.UnscentedKalmanFilter(n=2, p=1,
        f=f_lin, h=h_lin, Q=Q_ekf, R=R_ekf, Ts=Ts_ekf)
    ukf.step(np.array([0.1]), np.array([0.5]))
    x_ukf = ukf.state()
    print(f'UKF state after step: {x_ukf}')
    assert x_ukf.shape == (2,), "UKF state wrong shape"
    print('EKF + UKF smoke tests passed.')
else:
    print('Skipping EKF/UKF: advanced_kalman not compiled in.')

# ---------------------------------------------------------------------------
# 14. RepetitiveController
# ---------------------------------------------------------------------------
pid_inner = ctrl.DiscretePID(p, 0.01)
rp = ctrl.RepetitiveParams()
rp.period_steps = 50; rp.Krc = 0.5; rp.Q = 0.98
rp.uMin = -10.0; rp.uMax = 10.0
rc = ctrl.RepetitiveController(pid_inner, rp, 0.01)
u_rc = rc.compute(1.0)
print(f'RepetitiveController compute(1.0) = {u_rc:.4f}  correction={rc.correction():.4f}')
assert isinstance(u_rc, float), "RC output should be float"
assert abs(rc.correction()) < abs(u_rc) + 0.01   # correction starts at 0

# ---------------------------------------------------------------------------
# 15. Fuzzy (FuzzyPD, FuzzyPID, FuzzySupervisor)
# ---------------------------------------------------------------------------
if feats.get('fuzzy', False):
    # FuzzyPD
    fpd_p = ctrl.FuzzyPDParams()
    fpd_p.e_scale = 1.0; fpd_p.de_scale = 0.1; fpd_p.u_scale = 5.0
    fpd_p.uMin = -5.0; fpd_p.uMax = 5.0
    fpd = ctrl.FuzzyPD(fpd_p, 0.01)
    u_fpd = fpd.compute(0.5)
    print(f'FuzzyPD compute(0.5) = {u_fpd:.4f}')
    assert fpd_p.uMin <= u_fpd <= fpd_p.uMax, "FuzzyPD output out of range"

    # FuzzyPID
    fpid_p = ctrl.FuzzyPIDParams()
    fpid_p.pd.e_scale = 1.0; fpid_p.pd.u_scale = 5.0; fpid_p.Ki = 0.1
    fpid_p.uMin = -5.0; fpid_p.uMax = 5.0
    fpid = ctrl.FuzzyPID(fpid_p, 0.01)
    u_fpid = fpid.compute(0.5)
    print(f'FuzzyPID compute(0.5) = {u_fpid:.4f}')
    assert fpid_p.uMin <= u_fpid <= fpid_p.uMax, "FuzzyPID output out of range"

    # FuzzySupervisor
    sp = ctrl.SupervisorParams()
    sp.e_threshold = 5.0; sp.trend_threshold = 0.5; sp.signal_threshold = 0.5
    sup = ctrl.FuzzySupervisor(sp, 0.01)
    # Warm up: call several times with constant small error to zero the trend
    for _ in range(20):
        sup.update(0.1)
    dec = sup.update(0.1)   # steady small error + zero trend: should NOT relinearise
    print(f'FuzzySupervisor: signal={dec.relinearize_signal:.3f} relinearize={dec.relinearize}')
    assert not dec.relinearize, "Steady small error should not trigger relinearisation"
    dec_big = sup.update(10.0)   # large error
    assert isinstance(dec_big.relinearize_signal, float)
    print('Fuzzy smoke tests passed.')
else:
    print('Skipping Fuzzy: not compiled in.')

# ---------------------------------------------------------------------------
# 16. SubspaceID (n4sid + suggest_order)
# ---------------------------------------------------------------------------
if feats.get('subspace', False):
    # Generate data from a known first-order system: y[k+1] = 0.8*y + 0.2*u
    rng_id = np.random.default_rng(1)
    N_id = 500
    y_id = np.zeros(N_id); u_id = rng_id.standard_normal(N_id)
    for k in range(1, N_id):
        y_id[k] = 0.8*y_id[k-1] + 0.2*u_id[k-1] + 0.01*rng_id.standard_normal()
    Y_mat = y_id[np.newaxis, :]   # (1 x N)
    U_mat = u_id[np.newaxis, :]   # (1 x N)

    res = ctrl.n4sid(Y_mat, U_mat, n_order=1, i_horizon=10, Ts=0.01)
    print(f'n4sid: success={res.success}, sv_count={len(res.singular_values)}')
    assert res.success, f"n4sid failed: {res.message}"
    model = res.get_model()
    assert model.state_size() == 1
    order = ctrl.suggest_order(res.singular_values, threshold=0.01)
    print(f'suggest_order -> {order}')
    assert 1 <= order <= 5, f"suggest_order returned unexpected value: {order}"
    print('SubspaceID smoke tests passed.')
else:
    print('Skipping SubspaceID: not compiled in.')

# ---------------------------------------------------------------------------
# 17. DiscreteHinf + MixedSensitivity
# ---------------------------------------------------------------------------
if feats.get('hinf', False):
    # Simple stable first-order plant: G(s) = 1/(s+1), ZOH @ Ts=0.1s
    Ts_h = 0.1
    G_h = ctrl.c2d(ctrl.StateSpace(
        np.array([[-1.0]]), np.array([[1.0]]),
        np.array([[1.0]]), np.zeros((1,1)), 0.0), Ts_h, ctrl.C2dMethod.ZOH)

    W1 = ctrl.MixedSensitivity.make_W1(omega_B=2.0, M=2.0, eps=0.01, Ts=Ts_h)
    W2 = ctrl.MixedSensitivity.make_W2_constant(gain=0.1, Ts=Ts_h)
    W3 = ctrl.MixedSensitivity.make_W3(omega_T=30.0, Mt=1.5, eps=0.01, Ts=Ts_h)
    P  = ctrl.MixedSensitivity.build(G_h, W1, W2, W3)
    print(f'GeneralisedPlant: n={P.state_size()}, nw={P.nw()}, nu={P.nu()}, nz={P.nz()}, ny={P.ny()}')

    hp = ctrl.HinfParams(); hp.gamma_init = 50.0
    result = ctrl.DiscreteHinf.solve(P, hp)
    print(f'Hinf: feasible={result.feasible}  gamma={result.achieved_gamma:.3f}')
    if result.feasible:
        hinf = ctrl.DiscreteHinf(result)
        u_h = hinf.compute(0.1)   # measurement input (NOT error)
        print(f'DiscreteHinf compute(0.1) = {u_h:.4f}')
        assert isinstance(u_h, float)
        assert hinf.achieved_gamma() > 0
    print('Hinf smoke tests passed.')

    # HinfFilter - H-infinity-optimal state filter (Phase 3 EF1)
    assert hasattr(ctrl, 'HinfFilter'), "HinfFilter not bound"
    assert ctrl.registry_has('hinf_filter'), "hinf_filter not registered"
    Qw_h = np.eye(1) * 0.01
    Rv_h = np.eye(1) * 0.1
    hf_result = ctrl.HinfFilter.solve(G_h, Qw_h, Rv_h)
    print(f'HinfFilter: feasible={hf_result.feasible}  gamma={hf_result.achieved_gamma:.3f}')
    if hf_result.feasible:
        hf = ctrl.HinfFilter(hf_result)
        hf.predict(np.array([0.0]))
        hf.update(np.array([0.1]))
        x_hat = hf.state()
        assert np.all(np.isfinite(x_hat)), "HinfFilter state not finite"
        assert hf.achieved_gamma() > 0
    print('HinfFilter smoke test passed.')
else:
    print('Skipping Hinf: not compiled in.')

# ---------------------------------------------------------------------------
# 17b. DiscreteH2 (Phase 4 Iteration 3 - discrete H2/LQG synthesis)
# ---------------------------------------------------------------------------
if feats.get('h2_synthesis', False):
    # Hand-built D11=0 generalised plant (MixedSensitivity-built plants have D11 != 0
    # from their W1/W3 weight gains and are rejected by DiscreteH2 - see lib/DiscreteH2.h).
    # nw=2 deliberately - a single noise channel (nw=1) is a degenerate case for the filter
    # Riccati (S2^2 == Q2*R2 always for a scalar B1/D21), forcing Y=0 and an overly
    # aggressive, destabilising observer gain.
    P_h2 = ctrl.GeneralisedPlant()
    P_h2.Ts  = 0.1
    P_h2.A   = np.array([[0.9]])
    P_h2.B1  = np.array([[0.3, 0.1]])
    P_h2.B2  = np.array([[1.0]])
    P_h2.C1  = np.array([[1.0], [0.3]])
    P_h2.C2  = np.array([[1.0]])
    P_h2.D11 = np.zeros((2, 2))
    P_h2.D12 = np.array([[0.2], [1.0]])
    P_h2.D21 = np.array([[0.1, 0.4]])
    P_h2.D22 = np.zeros((1, 1))

    result_h2 = ctrl.DiscreteH2.solve(P_h2)
    print(f'H2: feasible={result_h2.feasible}  achieved_h2_norm={result_h2.achieved_h2_norm:.3f}')
    if result_h2.feasible:
        h2 = ctrl.DiscreteH2(result_h2)
        u_h2 = h2.compute(0.1)
        print(f'DiscreteH2 compute(0.1) = {u_h2:.4f}')
        assert isinstance(u_h2, float)
        assert h2.achieved_h2_norm() > 0
    print('DiscreteH2 smoke tests passed.')
else:
    print('Skipping DiscreteH2: not compiled in.')

# ---------------------------------------------------------------------------
# Part 18: SOPDTIdentifier
# ---------------------------------------------------------------------------
import math
t_sop  = [i * 0.2 for i in range(100)]
y_sop  = [0.0] * 100
for i, ti in enumerate(t_sop):
    dt = ti - 1.0
    if dt > 0:
        y_sop[i] = 2.0 * 1.0 * (1.0 - (3.0*math.exp(-dt/3.0) - 1.0*math.exp(-dt/1.0))/(3.0-1.0))

sop_id = ctrl.SOPDTIdentifier(t_sop, y_sop, 1.0, 0.0)
sop_m  = sop_id.identify(ctrl.SOPDTMethod.Optimization)
print(f'SOPDT Opt: K={sop_m.K:.3f} tau1={sop_m.tau1:.3f} tau2={sop_m.tau2:.3f} theta={sop_m.theta:.3f}')
assert sop_m.K > 0, "SOPDT K must be > 0"
assert sop_m.tau1 >= sop_m.tau2, "tau1 must be >= tau2"
pp_sop = ctrl.SOPDTIdentifier.imc_tuning(sop_m, 2.0 * sop_m.theta, 0.1)
assert pp_sop.Kp > 0, "IMC-SOPDT Kp must be > 0"
print('SOPDTIdentifier smoke test passed.')

# ---------------------------------------------------------------------------
# Part 18: MovingHorizonEstimator
# ---------------------------------------------------------------------------
mhe_plant = ctrl.StateSpace(
    np.array([[0.8]]),
    np.array([[1.0]]),
    np.array([[1.0]]),
    np.zeros((1, 1)), 0.1)

mhe_params = ctrl.MHEParams()
mhe_params.N       = 5
mhe_params.wMin    = -1.0
mhe_params.wMax    =  1.0

mhe = ctrl.MovingHorizonEstimator(mhe_plant,
    0.01 * np.eye(1), 0.1 * np.eye(1), mhe_params)
mhe.initialize(np.zeros(1), 10.0 * np.eye(1))

for _ in range(10):
    x_mhe = mhe.estimate(np.array([0.5]), np.array([1.0]))

print(f'MHE state after 10 steps: {x_mhe}')
assert x_mhe.shape == (1,),           "MHE state wrong shape"
assert np.isfinite(x_mhe[0]),         "MHE state not finite"
assert mhe.last_converged(),           "MHE QP did not converge"

# E4: polytopic constraint fields accessible
mhe_params2 = ctrl.MHEParams()
mhe_params2.C_ineq = np.array([[1.0]])   # x_0 <= 0.8
mhe_params2.d_ineq = np.array([0.8])
mhe_params2.ineq_proj_iters = 10
assert mhe_params2.C_ineq.shape == (1, 1), "MHEParams.C_ineq shape wrong"
assert mhe_params2.d_ineq.shape == (1,),   "MHEParams.d_ineq shape wrong"
assert mhe_params2.ineq_proj_iters == 10,  "MHEParams.ineq_proj_iters wrong"

print('MovingHorizonEstimator smoke test passed.')

# ---- GainScheduledController + nu_gap + LPVModel (Part 20) ----------------
import ctrl_toolbox as ctrl
Ts_s = 0.1
pid0 = ctrl.DiscretePID(ctrl.PIDParams(), Ts_s)
pid1 = ctrl.DiscretePID(ctrl.PIDParams(), Ts_s)
gs = ctrl.GainScheduledController(Ts_s)
gs.add_schedule_point(0.0, pid0)
gs.add_schedule_point(1.0, pid1)
assert gs.num_points == 2, "Expected 2 schedule points"
gs.set_scheduling_param(0.5)
u_gs = gs.compute(1.0)
assert np.isfinite(u_gs), "GainScheduledController output not finite"
print('GainScheduledController smoke test passed.')

sys_a = ctrl.StateSpace(np.array([[0.9]]), np.array([[0.1]]),
                        np.array([[1.0]]), np.array([[0.0]]), Ts_s)
sys_b = ctrl.StateSpace(np.array([[0.5]]), np.array([[0.5]]),
                        np.array([[1.0]]), np.array([[0.0]]), Ts_s)
gap = ctrl.nu_gap(sys_a, sys_b, 50)
assert 0.0 <= gap <= 1.0, f"nu_gap out of range: {gap}"
gap_self = ctrl.nu_gap(sys_a, sys_a, 50)
assert gap_self < 1e-9, f"nu_gap(P,P) not zero: {gap_self}"
G_mat = ctrl.nu_gap_matrix([sys_a, sys_b], 50)
assert G_mat.shape == (2, 2), "nu_gap_matrix wrong shape"
assert abs(G_mat[0,0]) < 1e-9 and abs(G_mat[1,1]) < 1e-9, "diagonal not zero"
res = ctrl.cluster_by_gap(G_mat, 0.5)
assert res.num_clusters in (1, 2), "unexpected cluster count"
print('nuGap + clusterByGap smoke tests passed.')

# Check that identify_lpv binding is accessible; full numerical test in ex77
assert hasattr(ctrl, 'LPVModel'),        "LPVModel class not accessible"
assert hasattr(ctrl, 'identify_lpv'),    "identify_lpv not accessible"
assert hasattr(ctrl, 'identify_lpv_from_io'), "identify_lpv_from_io not accessible"
print('LPVModel smoke test passed.')

# NonlinearMPC smoke test
nmpc_p = ctrl.NMPCParams()
nmpc_p.Np = 5; nmpc_p.Nu = 2; nmpc_p.rho_y = 1.0; nmpc_p.rho_u = 0.1
nmpc_p.uMin = -5.0; nmpc_p.uMax = 5.0; nmpc_p.Ts = Ts_s
nmpc_p.n_states = 1; nmpc_p.n_inputs = 1; nmpc_p.n_outputs = 1
def _f_nl(x, u):
    return np.array([0.9 * float(x[0]) + float(u[0])])
nmpc_obj = ctrl.NonlinearMPC(nmpc_p, _f_nl)
nmpc_obj.set_state(np.array([0.0]))
u_nmpc = nmpc_obj.compute_ref(np.array([0.0]), np.array([1.0]))
assert np.isfinite(u_nmpc[0]), "NonlinearMPC output not finite"
print('NonlinearMPC smoke test passed.')

# AdaptiveSmithPredictor smoke test
asp_p = ctrl.AdaptiveSPParams()
asp_p.max_delay_steps = 10; asp_p.estimate_interval = 100; asp_p.buffer_len = 150
sys_sp = ctrl.StateSpace(np.array([[0.8]]), np.array([[0.2]]),
                         np.array([[1.0]]), np.array([[0.0]]), Ts_s)
inner_pid = ctrl.DiscretePID(ctrl.PIDParams(), Ts_s)
asp_ctrl = ctrl.AdaptiveSmithPredictor(inner_pid, sys_sp, 3, Ts_s, asp_p)
asp_ctrl.set_plant_output(0.5)
u_asp = asp_ctrl.compute(0.5)
assert np.isfinite(u_asp), "AdaptiveSmithPredictor output not finite"
assert asp_ctrl.estimated_delay_steps() == 3
print('AdaptiveSmithPredictor smoke test passed.')

# AutoTuner smoke test
atp = ctrl.AutoTunerParams()
atp.n = 2; atp.sigma0 = 0.5; atp.maxIter = 20
tuner = ctrl.AutoTuner(atp, 42)
def _quad_cost(p):
    return float(p[0]**2 + p[1]**2)
res = tuner.tune(_quad_cost, np.array([1.0, 1.0]))
assert res.n_evals > 0, "AutoTuner produced 0 evaluations"
assert np.isfinite(res.cost), "AutoTuner cost not finite"
print('AutoTuner smoke test passed.')

# AntiWindupWrapper smoke test
pp_aw = ctrl.PIDParams()
pp_aw.Kp = 0.5; pp_aw.Ki = 1.0; pp_aw.Kb = 0.0
pp_aw.uMin = -1e9; pp_aw.uMax = 1e9
pid_aw_inner = ctrl.DiscretePID(pp_aw, 0.1)
aw = ctrl.AntiWindupWrapper(pid_aw_inner, -3.0, 1.0, 1.0)
u_aw = aw.compute(5.0)   # large error -> saturates
assert aw.is_saturated(), "AntiWindupWrapper should be saturated on large error"
assert aw.saturation_error() < 0.0, "saturation_error should be negative when clamped at uMax"
aw.reset()
assert not aw.is_saturated(), "AntiWindupWrapper should not be saturated after reset"
print('AntiWindupWrapper smoke test passed.')

# TubeMPC smoke test
tp_s = ctrl.TubeMPCParams()
tp_s.Np = 5; tp_s.Nu = 2
tp_s.Q = np.eye(1) * 2.0; tp_s.R = np.eye(1) * 0.3
tp_s.K = np.array([[-0.3]]); tp_s.wMax = np.array([0.05])
tp_s.uMin = np.array([-2.0]); tp_s.uMax = np.array([2.0]); tp_s.Ts = 0.1
sys_tube = ctrl.StateSpace(np.array([[0.8]]), np.array([[0.2]]),
                           np.eye(1), np.zeros((1,1)), 0.1)
tmpc_s = ctrl.TubeMPC(sys_tube, tp_s)
z = tmpc_s.tube_radius()
assert np.all(z > 0), "TubeMPC tube radius must be positive"
u_tube = tmpc_s.compute_ref(np.zeros(1), np.ones(1))
assert np.isfinite(u_tube[0]), "TubeMPC output not finite"
print('TubeMPC smoke test passed.')

# ParticleFilter smoke test
pfp_s = ctrl.ParticleFilterParams()
pfp_s.n_particles = 50; pfp_s.Q = np.eye(1)*0.1; pfp_s.R = np.eye(1)*0.5; pfp_s.seed = 1
pf_s = ctrl.ParticleFilter(pfp_s, 1, 1,
                            lambda x, u: np.array([0.9*x[0]+u[0]]),
                            lambda x, u: x.copy())
pf_s.initialise(np.zeros(1))
pf_s.step(np.array([0.3]), np.zeros(1))
assert np.isfinite(pf_s.state()[0]), "ParticleFilter state not finite"
assert pf_s.effective_sample_size() > 1.0, "ParticleFilter N_eff <= 1"
print('ParticleFilter smoke test passed.')


# ILC P-type
_ip = ctrl.ILCParams()
_ip.N = 20; _ip.Ts = 0.01; _ip.mode = ctrl.ILCMode.PType; _ip.Lp = 0.5
_ip.uMin = -2.0; _ip.uMax = 2.0
_ilc = ctrl.ILC(_ip)
assert _ilc.trial_index() == 0
for _k in range(20):
    _ilc.record_error(_k, 1.0 - float(_k) / 20.0)
_ilc.update_feedforward()
assert _ilc.trial_index() == 1
assert _ilc.last_rms_error() > 0.0
_ff0 = _ilc.feedforward(0)
assert _ip.uMin <= _ff0 <= _ip.uMax, "ILC feedforward out of bounds"
assert 'ilc' in ctrl.features(), "features() missing 'ilc'"
print('ILC smoke test passed.')

# SINDy: identify simple 1-state linear system  dx = -0.5*x + u
_sp = ctrl.SINDyParams()
_sp.n_state = 1; _sp.n_input = 1
_sp.library = ctrl.SINDyLibrary.PolyDeg2
_sp.threshold = 0.01; _sp.stls_iter = 10
_sindy = ctrl.SINDy(_sp)
for _k in range(200):
    _x = np.array([float(_k % 10) * 0.1 - 0.5])
    _u = np.array([float(_k % 5) * 0.2 - 0.4])
    _xdot = np.array([-0.5 * _x[0] + _u[0]])
    _sindy.add_snapshot(_x, _u, _xdot)
_sm = _sindy.fit()
assert _sm.n_state() == 1, "SINDy n_state mismatch"
_test_x = np.array([1.0]); _test_u = np.array([0.5])
_pred = _sm.predict(_test_x, _test_u)
assert np.isfinite(_pred[0]), "SINDy predict not finite"
assert abs(_pred[0] - (-0.5*1.0 + 0.5)) < 0.2, f"SINDy prediction far off: {_pred[0]:.3f}"
assert 'sindy' in ctrl.features(), "features() missing 'sindy'"
print('SINDy smoke test passed.')

# KoopmanEDMD
_kp = ctrl.KoopmanEDMDParams()
_kp.n_state = 1; _kp.n_input = 1; _kp.dict = ctrl.KoopmanDict.PolyDeg2
_edmd = ctrl.KoopmanEDMD(_kp)
for _k in range(30):
    _xk = np.array([float(_k) * 0.1 - 1.5])
    _uk = np.array([0.3])
    _xk1 = np.array([0.8 * _xk[0] + 0.2 * _uk[0]])
    _edmd.add_snapshot(_xk, _uk, _xk1)
_ss_proj = _edmd.fit_projected()
assert _ss_proj.state_size() >= 1, "KoopmanEDMD fit_projected state size mismatch"
assert 'koopman_edmd' in ctrl.features(), "features() missing 'koopman_edmd'"
print('KoopmanEDMD smoke test passed.')

# L1 Adaptive
_l1p = ctrl.L1AdaptiveParams()
_l1p.a_m = 0.8; _l1p.b_m = 0.2; _l1p.Gamma = 50.0; _l1p.omega_c = 2.0
_l1p.uMin = -2.0; _l1p.uMax = 2.0
_l1 = ctrl.L1AdaptiveController(_l1p, 0.01)
_l1.set_reference(1.0)
_u_l1 = _l1.compute(0.5)
assert np.isfinite(_u_l1), "L1Adaptive output not finite"
assert 'l1_adaptive' in ctrl.features()
print('L1AdaptiveController smoke test passed.')

# CBF Safety Filter
_pid_cbf = ctrl.DiscretePID(ctrl.PIDParams(), 0.01)
_h_fn  = lambda x: 1.5 - x
_dh_fn = lambda x: -1.0
_f0_fn = lambda x: 0.0
_g_fn  = lambda x: 0.01
_cbfp  = ctrl.CBFParams(); _cbfp.alpha = 1.0; _cbfp.uMin = -5.0; _cbfp.uMax = 5.0
_cbf   = ctrl.CBFSafetyFilter(_pid_cbf, _h_fn, _dh_fn, _f0_fn, _g_fn, _cbfp, 0.01)
_cbf.set_state(1.0)
_u_cbf = _cbf.compute(0.5)
assert np.isfinite(_u_cbf), "CBF output not finite"
assert 'cbf_safety_filter' in ctrl.features()
print('CBFSafetyFilter smoke test passed.')

# GaussianProcess
_gpp = ctrl.GPParams(); _gpp.length_scale = 1.0; _gpp.signal_var = 1.0; _gpp.noise_var = 0.01
_gpr = ctrl.GaussianProcess(1, _gpp)
for _k in range(10):
    _gpr.add_point(np.array([float(_k) * 0.5]), float(np.sin(_k * 0.5)))
_gpr.fit()
_pred = _gpr.predict(np.array([1.0]))
assert np.isfinite(_pred.mean), "GP mean not finite"
assert _pred.variance >= 0.0, "GP variance negative"
assert 'gaussian_process' in ctrl.features()
print('GaussianProcess smoke test passed.')

# EchoStateNetwork
_ep = ctrl.ESNParams(); _ep.n_res = 20; _ep.n_in = 1; _ep.n_out = 1
_ep.spectral_radius = 0.8; _ep.washout = 5
_esn = ctrl.EchoStateNetwork(_ep)
for _k in range(30):
    _u_esn = np.array([float(_k % 3) * 0.5 - 0.5])
    _esn.step_reservoir(_u_esn)
    _esn.add_training_target(np.array([np.tanh(0.5 * float(_k % 5))]))
_esn.fit_readout()
assert _esn.is_fitted(), "ESN not fitted"
_y_esn = _esn.predict(np.array([0.5]))
assert np.isfinite(_y_esn[0]), "ESN predict not finite"
assert 'echo_state_network' in ctrl.features()
print('EchoStateNetwork smoke test passed.')

# NeuralPID
_np_p = ctrl.NeuralPIDParams()
_np_p.n_hidden = 4; _np_p.lr = 1e-3; _np_p.Ts = 0.01; _np_p.plant_gain = 0.2
_np_p.uMin = -2.0; _np_p.uMax = 2.0
_npid = ctrl.NeuralPID(_np_p)
_u_npid = _npid.compute(0.5)
assert np.isfinite(_u_npid), "NeuralPID not finite"
assert np.isfinite(_npid.current_kp()), "NeuralPID Kp not finite"
assert 'neural_pid' in ctrl.features()
print('NeuralPID smoke test passed.')

# CEM-MPC
_cp = ctrl.CEMParams(); _cp.Np = 5; _cp.N_samples = 20; _cp.n_iter = 2
_cp.Q = 1.0; _cp.R = 0.1; _cp.uMin = -1.0; _cp.uMax = 1.0
import numpy as _npcem
_A_cem = _npcem.array([[0.9]])
_B_cem = _npcem.array([[0.2]])
_C_cem = _npcem.array([[1.0]])
_D_cem = _npcem.array([[0.0]])
_ss_cem = ctrl.StateSpace(_A_cem, _B_cem, _C_cem, _D_cem, 0.01)
def _f_cem(x, u): return _ss_cem.A @ x + _ss_cem.B @ u
_cem = ctrl.CEMController(_cp, _f_cem, _C_cem, 0.01)
_cem.set_state(_npcem.array([0.0]))
_cem.set_reference(_npcem.array([1.0]))
_u_cem = _cem.compute_ref(_npcem.array([0.0]), _npcem.array([1.0]))
assert _npcem.isfinite(_u_cem[0]), "CEM output not finite"
assert 'cem_mpc' in ctrl.features()
print('CEMController smoke test passed.')

# ---- DynaController --------------------------------------------------------
import numpy as _npdyna
_pid_for_dyna_p = ctrl.PIDParams()
_pid_for_dyna_p.Kp = 0.8; _pid_for_dyna_p.Ki = 0.2; _pid_for_dyna_p.Kd = 0.0
_pid_for_dyna = ctrl.DiscretePID(_pid_for_dyna_p, 0.01)
_dp = ctrl.DynaParams()
_dp.Ts = 0.01; _dp.n_collect = 5; _dp.n_refit_every = 50
_dyna = ctrl.DynaController(_dp, _pid_for_dyna)
for _k in range(10):
    _u_dyna = _dyna.compute(1.0 - 0.1 * _k)
assert _npdyna.isfinite(_u_dyna), "DynaController output not finite"
assert _dyna.buffer_size() == 9, f"Expected 9 transitions, got {_dyna.buffer_size()}"
assert _dyna.model_fitted(), "Model should be fitted after 9 >= n_collect=5 transitions"
_e_pred = _dyna.model_rollout(0.5, _npdyna.ones(5) * 0.1)
assert len(_e_pred) == 5, "Rollout should return 5-element vector"
assert 'dyna' in ctrl.features()
print('DynaController smoke test passed.')

# ---- ScenarioMPC -----------------------------------------------------------
import numpy as _npsmpc
_A_sm = _npsmpc.array([[0.9]])
_B_sm = _npsmpc.array([[0.1]])
_C_sm = _npsmpc.array([[1.0]])
_D_sm = _npsmpc.array([[0.0]])
_ss_sm = ctrl.StateSpace(_A_sm, _B_sm, _C_sm, _D_sm, 0.1)
_smp = ctrl.ScenarioMPCParams()
_smp.Np = 5; _smp.Nu = 2; _smp.Ts = 0.1; _smp.N_samples = 10; _smp.seed = 7
_smp.Q = _npsmpc.eye(1); _smp.R = _npsmpc.eye(1) * 0.1
_smp.Sigma_w = _npsmpc.eye(1) * 0.01
_smp.uMin = _npsmpc.array([-2.0]); _smp.uMax = _npsmpc.array([2.0])
_smpc = ctrl.ScenarioMPC(_ss_sm, _smp)
_smpc.set_state(_npsmpc.array([0.5]))
_smpc.set_reference(_npsmpc.array([1.0]))
_u_sm = _smpc.compute_control()
assert _npsmpc.isfinite(_u_sm[0]), "ScenarioMPC output not finite"
_u_siso = _smpc.compute(0.3)
assert _npsmpc.isfinite(_u_siso), "ScenarioMPC SISO output not finite"
assert 'scenario_mpc' in ctrl.features()
print('ScenarioMPC smoke test passed.')

# ---- BayesianOptimizer -----------------------------------------------------
import numpy as _npbo
_bp = ctrl.BayesOptParams()
_bp.n = 2; _bp.n_init = 4; _bp.maxIter = 6; _bp.seed = 7
_bp.lower = _npbo.array([0.0, 0.0]); _bp.upper = _npbo.array([5.0, 5.0])
_bo = ctrl.BayesianOptimizer(_bp)
_bo_x0 = _npbo.array([1.0, 1.0])
# Sphere: min at (2,2) -> should return cost near 0
_bo_result = _bo.tune(lambda p: float((p[0]-2.0)**2 + (p[1]-2.0)**2), _bo_x0)
assert _npbo.isfinite(_bo_result.cost), "BO result cost not finite"
assert _bo_result.n_evals == 10, f"Expected 10 evals, got {_bo_result.n_evals}"
assert _bo_result.cost < 8.0, f"BO should improve: cost={_bo_result.cost}"
assert 'bayesian_optimizer' in ctrl.features()
print('BayesianOptimizer smoke test passed.')

# ---- GeneticAlgorithm -------------------------------------------------------
import numpy as _npga
_gap = ctrl.GAParams()
_gap.n_dim = 2; _gap.population = 20; _gap.max_gen = 30; _gap.seed = 1
_gap.lower = _npga.array([0.0, 0.0]); _gap.upper = _npga.array([5.0, 5.0])
_ga = ctrl.GeneticAlgorithm(_gap)
_ga_result = _ga.optimize(lambda x: float((x[0]-2.0)**2 + (x[1]-3.0)**2))
assert _npga.isfinite(_ga_result.cost), f"GA result cost not finite"
assert _ga_result.cost < 2.0, f"GA should converge near min: cost={_ga_result.cost}"
assert 'genetic_algorithm' in ctrl.features()
print('GeneticAlgorithm smoke test passed.')

# ---- NelderMead (Phase 3 MO2) -----------------------------------------------
_nmp = ctrl.NelderMeadParams()
_nmp.n_dim = 2
_nm = ctrl.NelderMead(_nmp)
_nm_result = _nm.optimize(lambda x: float((x[0] - 2.0) ** 2 + (x[1] - 3.0) ** 2),
                           np.array([0.0, 0.0]))
assert np.isfinite(_nm_result.cost), "NelderMead result cost not finite"
assert _nm_result.cost < 1e-3, f"NelderMead should converge near min: cost={_nm_result.cost}"
assert 'nelder_mead' in ctrl.features()
print('NelderMead smoke test passed.')

# ---- ParticleSwarmOptimizer -------------------------------------------------
import numpy as _nppso
_psop = ctrl.PSOParams()
_psop.n_dim = 2; _psop.n_particles = 15; _psop.max_iter = 30; _psop.seed = 2
_psop.lower = _nppso.array([0.0, 0.0]); _psop.upper = _nppso.array([5.0, 5.0])
_pso = ctrl.ParticleSwarmOptimizer(_psop)
_pso_result = _pso.optimize(lambda x: float((x[0]-2.0)**2 + (x[1]-3.0)**2))
assert _nppso.isfinite(_pso_result.cost), f"PSO result cost not finite"
assert _pso_result.cost < 2.0, f"PSO should converge near min: cost={_pso_result.cost}"
assert 'particle_swarm' in ctrl.features()
print('ParticleSwarmOptimizer smoke test passed.')

# ---- DifferentialEvolution --------------------------------------------------
import numpy as _npde
_dep = ctrl.DEParams()
_dep.n_dim = 2; _dep.population = 15; _dep.max_gen = 30; _dep.seed = 3
_dep.lower = _npde.array([0.0, 0.0]); _dep.upper = _npde.array([5.0, 5.0])
_de = ctrl.DifferentialEvolution(_dep)
_de_result = _de.optimize(lambda x: float((x[0]-2.0)**2 + (x[1]-3.0)**2))
assert _npde.isfinite(_de_result.cost), f"DE result cost not finite"
assert _de_result.cost < 2.0, f"DE should converge near min: cost={_de_result.cost}"
assert 'differential_evolution' in ctrl.features()
print('DifferentialEvolution smoke test passed.')

# ---- ControllerRegistry (M2) -----------------------------------------------
assert ctrl.registry_count() > 0, "Registry should have entries after umbrella include"
assert ctrl.registry_has('pid'),  "pid should be registered"
assert ctrl.registry_has('dyna'), "dyna should be self-registered"
assert ctrl.registry_has('scenario_mpc'), "scenario_mpc should be self-registered"
assert ctrl.registry_has('bayesian_optimizer'), "bayesian_optimizer should be self-registered"
assert ctrl.registry_has('controller_monitor'), "controller_monitor should be self-registered"
assert 'pid' in ctrl.features(), "features() should include pid"
print('ControllerRegistry smoke test passed.')

# ---- ControllerMonitor (M3/SPC) --------------------------------------------
import numpy as _npmon
_mon = ctrl.ControllerMonitor()
_mon.set_target(0.0); _mon.set_sigma(0.1)
_alarms = []
_mon.set_alarm_callback(lambda chart, stat: _alarms.append((chart, stat)))
# Feed 10 normal samples then a large shift
for _v in [0.0]*10:
    _mon.on_compute(_v, 0.0)
_mon.on_compute(5.0, 0.0)  # large excursion should trigger alarm
assert _mon.n_samples() == 11
assert _mon.n_alarms() > 0, "Large shift should trigger at least one alarm"
print('ControllerMonitor smoke test passed.')

# ---- DAESystem (P1/P2): Index-1 semi-explicit DAE ---------------------------
import numpy as _npdae

# Build a simple RC circuit DAE:
#   x1' = f(x1, x2, u) = -(x1 + x2) / (R*C) + u/(R*C)   [capacitor voltage]
#   0   = g(x1, x2, u) = x1 - x2 - R*i                    [resistor voltage drop]
#   Where x2 = R*i and the algebraic constraint means x2 = x1 - R*i.
# Simplest Index-1 DAE: x1' = -x1 + u,  0 = x1 - x2  => x2 = x1
_dae = ctrl.DAESystem()
_dae.n_diff = 1
_dae.n_alg  = 1
_dae.Ts     = 0.05

_dae.set_f(lambda x1, x2, u: _npdae.array([-x1[0] + u]))
_dae.set_g(lambda x1, x2, u: _npdae.array([x1[0] - x2[0]]))  # x2 = x1

# consistent_init: starting from x2_guess=0, should find x2=x1=1
_x1_test = _npdae.array([1.0])
_x2_test = _npdae.array([0.0])  # wrong initial guess
_x2_found = ctrl.consistent_init(_dae, _x1_test, 0.0, _x2_test)
assert abs(_x2_found[0] - 1.0) < 1e-8, f"consistent_init failed: x2={_x2_found[0]:.6f}"
print(f'DAE consistent_init: x2_guess=0 -> x2_solved={_x2_found[0]:.6f} (expected 1.0)')

# dae2ode: step from x_aug=[0.5, 0.5] with u=1; need ~5tau = 100 steps at Ts=0.05 to reach ~0.997
_step_fn = ctrl.dae2ode(_dae)
_x_aug = _npdae.array([0.5, 0.5])
for _ in range(100):
    _x_aug = _step_fn(_x_aug, 1.0)
assert abs(_x_aug[0] - 1.0) < 0.05, f"dae2ode step fn diverged: x1={_x_aug[0]:.4f}"
assert abs(_x_aug[1] - _x_aug[0]) < 1e-6, f"dae2ode x2 != x1 (constraint violated)"
print(f'DAE dae2ode: x1 after 100 steps = {_x_aug[0]:.4f} (expected ~1.0)')

# dae_c2d: linearise and discretise; reduced model is x1' = -x1 + u => Ad = exp(-Ts)
_dae.set_h(lambda x1, x2, u: _npdae.array([x1[0]]))  # output = x1
_ss_dae = ctrl.dae_c2d(_dae, _npdae.array([1.0]), _npdae.array([1.0]), 1.0, _dae.Ts)
assert _ss_dae.state_size() == 1, f"dae_c2d: expected n=1, got {_ss_dae.state_size()}"
assert abs(_ss_dae.A[0, 0] - _npdae.exp(-_dae.Ts)) < 0.02, \
    f"dae_c2d A not near exp(-Ts): A={_ss_dae.A[0,0]:.4f}"
print(f'DAE dae_c2d: A={_ss_dae.A[0,0]:.4f} (expected ~{_npdae.exp(-_dae.Ts):.4f})')

assert ctrl.registry_has('dae_system'), "dae_system should be registered"
print('DAESystem smoke tests passed.')

# ---- GreyBoxEstimator (E1) --------------------------------------------------
import numpy as _npgb
# First-order linear ODE: xdot = -a*x + b*u, y = x.  True: a=0.5, b=1.0.
def _f_gb(x, u, p): return _npgb.array([-p[0]*x[0] + p[1]*u[0]])
def _h_gb(x, p):    return _npgb.array([x[0]])

_Ts_gb = 0.05
_N_gb  = 40
_x_gb  = _npgb.zeros(1)
_U_gb  = _npgb.zeros((1, _N_gb))
_Y_gb  = _npgb.zeros((1, _N_gb))
_true_a, _true_b = 0.5, 1.0
for _k in range(_N_gb):
    _U_gb[0, _k] = 1.0
    _Y_gb[0, _k] = _x_gb[0]
    _x_gb[0]    += _Ts_gb * (-_true_a*_x_gb[0] + _true_b*_U_gb[0, _k])

_gbpar = ctrl.GreyBoxParams()
_gbpar.p0     = _npgb.array([0.3, 0.7])  # wrong initial guess
_gbpar.lower  = _npgb.array([0.01, 0.1])
_gbpar.upper  = _npgb.array([5.0, 5.0])
_gbpar.Ts     = _Ts_gb
_gbpar.max_iter = 30

_gbe    = ctrl.GreyBoxEstimator(_f_gb, _h_gb, _gbpar)
_gb_res = _gbe.fit(_npgb.zeros(1), _U_gb, _Y_gb)
assert _gb_res.iterations >= 0, "GreyBoxEstimator iterations should be >= 0"
assert _npgb.isfinite(_gb_res.cost), "GreyBoxEstimator cost not finite"
_p_hat = _gbe.params()
assert _p_hat.shape == (2,), "GreyBoxEstimator params wrong shape"
# Parameters should move toward ground truth
assert abs(_p_hat[0] - _true_a) < 0.3, f"GreyBoxEstimator a estimate off: {_p_hat[0]:.3f}"
assert abs(_p_hat[1] - _true_b) < 0.3, f"GreyBoxEstimator b estimate off: {_p_hat[1]:.3f}"
_Y_hat = _gbe.predict(_npgb.zeros(1), _U_gb)
assert _Y_hat.shape[0] == 1 and _Y_hat.shape[1] == _N_gb, \
    f"GreyBoxEstimator predict() shape mismatch: got {_Y_hat.shape}, expected (1, {_N_gb})"
assert _npgb.all(_npgb.isfinite(_Y_hat)), "GreyBoxEstimator predict contains non-finite values"
assert ctrl.registry_has('grey_box_estimator'), "grey_box_estimator not registered"
print('GreyBoxEstimator (E1) smoke test passed.')

# ---- RecursiveGreyBoxEstimator (E2) ----------------------------------------
if feats.get('advanced_kalman', True):
    import numpy as _nprgb
    # Same first-order ODE, track parameter online
    def _f_rgb(x, u, p): return _nprgb.array([-p[0]*x[0] + u[0]])
    def _h_rgb(x, p):    return _nprgb.array([x[0]])
    _Ts_rgb = 0.05
    _rgbpar = ctrl.RecursiveGreyBoxParams()
    _rgbpar.p0      = _nprgb.array([0.3])
    _rgbpar.Q_state = 1e-4 * _nprgb.eye(1)
    _rgbpar.Q_param = 1e-6 * _nprgb.eye(1)
    _rgbpar.R_meas  = 0.01 * _nprgb.eye(1)
    _rgbpar.Ts      = _Ts_rgb
    _rge = ctrl.RecursiveGreyBoxEstimator(_f_rgb, _h_rgb,
                                           n_state=1, n_meas=1, params=_rgbpar)
    _rge.initialize(_nprgb.zeros(1), 0.1 * _nprgb.eye(1))
    assert _rge.is_initialized(), "RecursiveGreyBoxEstimator not initialized"
    _x_rgb = 0.0
    for _k in range(30):
        _u_rgb = _nprgb.array([1.0])
        _y_rgb = _nprgb.array([_x_rgb])
        _x_hat_rgb = _rge.step(_y_rgb, _u_rgb)
        _x_rgb += _Ts_rgb * (-_true_a*_x_rgb + _u_rgb[0])
    assert _x_hat_rgb.shape == (1,), "RecursiveGreyBoxEstimator state shape wrong"
    assert _nprgb.isfinite(_x_hat_rgb[0]), "RecursiveGreyBoxEstimator state not finite"
    _p_rgb = _rge.param_estimate()
    assert _p_rgb.shape == (1,), "RecursiveGreyBoxEstimator param shape wrong"
    assert _nprgb.isfinite(_p_rgb[0]), "RecursiveGreyBoxEstimator param not finite"
    assert ctrl.registry_has('recursive_grey_box'), "recursive_grey_box not registered"
    print('RecursiveGreyBoxEstimator (E2) smoke test passed.')

# ---- GPResidualModel (E3) --------------------------------------------------
import numpy as _npgrm
_grmp = ctrl.GPResidualParams()
_grmp.gp.length_scale = 1.0; _grmp.gp.signal_var = 0.5; _grmp.gp.noise_var = 0.01
_grm = ctrl.GPResidualModel(x_dim=1, params=_grmp)
assert _grm.size() == 0, "GPResidualModel should start empty"
for _k in range(12):
    _xf = _npgrm.array([float(_k) * 0.5])
    _grm.add_residual_point(_xf, float(2*_k)*0.1 + 0.3, float(2*_k)*0.1)
_grm.fit()
assert _grm.is_fitted(), "GPResidualModel should be fitted"
_grm_pred = _grm.predict_with_uncertainty(_npgrm.array([1.0]), model_pred=1.0)
assert _npgrm.isfinite(_grm_pred.mean_total),  "GPResidualModel mean_total not finite"
assert _npgrm.isfinite(_grm_pred.variance),    "GPResidualModel variance not finite"
assert _grm_pred.variance >= 0.0,              "GPResidualModel variance must be >= 0"
# Batch residualFit
_grm2 = ctrl.GPResidualModel(x_dim=1, params=_grmp)
_Xf   = _npgrm.array([[float(_k)*0.3 for _k in range(15)]])
_Yt   = _npgrm.array([float(_k)*0.2 + 0.1 for _k in range(15)])
_grm2.residual_fit(_Xf, _Yt, model_fn=lambda xf: float(xf[0])*0.2)
assert _grm2.is_fitted(), "GPResidualModel batch fit failed"
assert ctrl.registry_has('gp_residual_model'), "gp_residual_model not registered"
print('GPResidualModel (E3) smoke test passed.')

# ---- HybridModel (H1) -------------------------------------------------------
import numpy as _npmhm
def _f_phys_hm(x, u, p):
    return _npmhm.array([x[1], -p[0]*x[0] - p[1]*x[1] + u[0]])
_hmp = ctrl.HybridModelParams()
_hmp.n_states = 2; _hmp.n_inputs = 1; _hmp.n_outputs = 1; _hmp.Ts = 0.01; _hmp.rk4_steps = 4
_p_phys = _npmhm.array([4.0, 0.8])
_hmodel = ctrl.HybridModel(_f_phys_hm, _hmp, p_phys=_p_phys)
assert _hmodel.state_size()  == 2,    "HybridModel state_size wrong"
assert _hmodel.input_size()  == 1,    "HybridModel input_size wrong"
assert not _hmodel.has_data_model(),   "HybridModel should start without data model"
_x0_hm = _npmhm.zeros(2)
_u0_hm = _npmhm.array([1.0])
_xn_hm = _hmodel.predict(_x0_hm, _u0_hm)
assert _xn_hm.shape == (2,), "HybridModel predict shape wrong"
assert _npmhm.all(_npmhm.isfinite(_xn_hm)), "HybridModel predict not finite"
_xn_phys = _hmodel.predict_phys(_x0_hm, _u0_hm)
assert _npmhm.allclose(_xn_hm, _xn_phys), "predict and predict_phys should match without data model"
_hmodel.set_data_model(lambda x, u: _npmhm.zeros(len(x)))
assert _hmodel.has_data_model(), "HybridModel should have data model after set"
_hmodel.clear_data_model()
assert not _hmodel.has_data_model(), "HybridModel should not have data model after clear"
assert ctrl.registry_has('hybrid_model'), "hybrid_model not registered"
print('HybridModel (H1) smoke test passed.')

# ---- HybridMPC (H2) ---------------------------------------------------------
import numpy as _npmhc
def _f_phys_hc(x, u, p):
    return _npmhc.array([x[1], -p[0]*x[0] - p[1]*x[1] + u[0]])
_hmp2 = ctrl.HybridModelParams()
_hmp2.n_states = 2; _hmp2.n_inputs = 1; _hmp2.Ts = 0.01
_hmodel2 = ctrl.HybridModel(_f_phys_hc, _hmp2, _npmhc.array([4.0, 0.8]))
_hpars = ctrl.HybridMPCParams()
_hpars.nmpc.n_states = 2; _hpars.nmpc.n_inputs = 1; _hpars.nmpc.n_outputs = 2
_hpars.nmpc.Np = 5; _hpars.nmpc.Nu = 2; _hpars.nmpc.Ts = 0.01
_hpars.nmpc.rho_y = 1.0; _hpars.nmpc.rho_u = 0.1
_hpars.data_update_interval = 0   # manual only for smoke test
_hpars.min_observations = 5
_hmpc = ctrl.HybridMPC(_hpars, _hmodel2)
_x0_hc = _npmhc.zeros(2)
_hmpc.set_state(_x0_hc)
_u_hc = _hmpc.compute(0.5)
assert _npmhc.isfinite(_u_hc), "HybridMPC compute not finite"
assert _hmpc.observation_count() == 0, "Observation count should start at 0"
# add observations and trigger manual refit
for _ki in range(8):
    _xi = _npmhc.array([float(_ki)*0.01, 0.0])
    _ui = _npmhc.array([0.5])
    _xn = _hmodel2.predict(_xi, _ui) + _npmhc.random.randn(2)*0.001
    _hmpc.add_state_observation(_xi, _ui, _xn)
assert _hmpc.observation_count() == 8, "Observation count wrong"
ok = _hmpc.refit_data_model()
assert ok, "refit_data_model should succeed with 8 >= 5 observations"
assert _hmpc.is_data_model_fitted(), "HybridMPC should be fitted after refit"
assert ctrl.registry_has('hybrid_mpc'), "hybrid_mpc not registered"
print('HybridMPC (H2) smoke test passed.')

# ---- HybridModelTrainer (H4) ------------------------------------------------
import numpy as _npmht
def _f_phys_ht(x, u, p):
    return _npmht.array([x[1], -p[0]*x[0] - p[1]*x[1] + u[0]])
_hmp3 = ctrl.HybridModelParams()
_hmp3.n_states = 2; _hmp3.n_inputs = 1; _hmp3.Ts = 0.01
_hmodel3 = ctrl.HybridModel(_f_phys_ht, _hmp3, _npmht.array([4.0, 0.8]))
_Ts3 = 0.01
N3   = 40
_xs3 = _npmht.zeros(2)
_X3_obs = _npmht.zeros((2, N3)); _U3_obs = _npmht.zeros((1, N3)); _Xn3_obs = _npmht.zeros((2, N3))
for _ki3 in range(N3):
    _u3 = _npmht.array([1.0 if _ki3 < N3//2 else -0.5])
    _x3_next = _hmodel3.predict(_xs3, _u3)
    _X3_obs[:,_ki3] = _xs3; _U3_obs[:,_ki3] = _u3; _Xn3_obs[:,_ki3] = _x3_next
    _xs3 = _x3_next
_htp = ctrl.HybridTrainerParams()
_htp.method = ctrl.HybridTrainerMethod.Ridge
_ht = ctrl.HybridModelTrainer(_htp)
_ht_res = _ht.train_hybrid_model(_hmodel3, _X3_obs, _U3_obs, _Xn3_obs)
assert _ht_res.success, "HybridModelTrainer training failed"
assert _npmht.isfinite(_ht_res.train_rmse), "train_rmse not finite"
assert _ht_res.method == 'Ridge', "method name wrong"
_rmse_val = _ht.validate(_hmodel3, _X3_obs, _U3_obs, _Xn3_obs)
assert _npmht.isfinite(_rmse_val), "validate RMSE not finite"
assert ctrl.registry_has('hybrid_model_trainer'), "hybrid_model_trainer not registered"
print('HybridModelTrainer (H4) smoke test passed.')

# -- T3: VectorFitting --------------------------------------------------------
import numpy as _np_vf
import math as _math_vf
_Ts_vf = 0.01
_N_vf  = 30
_w_nyq = _math_vf.pi / _Ts_vf
_omega_vf = [_w_nyq * (k + 1) / _N_vf * 0.95 for k in range(_N_vf)]
_mag_vf   = [1.0 / (1.0 + (w / 80.0) ** 2) for w in _omega_vf]
_vfp = ctrl.VectorFittingParams()
_vfp.max_iter = 8
_vfr, _vf_ss = ctrl.VectorFitting.fit_magnitude(_omega_vf, _mag_vf, 3, _Ts_vf, _vfp)
assert _vf_ss.state_size() == 3,           "VectorFitting: wrong state size"
assert _math_vf.isfinite(_vfr.rms_error),  "VectorFitting: rms_error not finite"
_m_check = ctrl.VectorFitting.eval_magnitude(_vf_ss, _omega_vf[_N_vf // 2])
assert _math_vf.isfinite(_m_check) and _m_check >= 0.0, "VectorFitting: eval_magnitude not finite/positive"
assert ctrl.registry_has('vector_fitting'), "vector_fitting not registered"
print('VectorFitting (T3) smoke test passed.')

# M4: BasicPID / BasicSMC feature flags (header-only templates, no Python class)
assert ctrl.registry_has('basic_pid'), "basic_pid not registered"
assert ctrl.registry_has('basic_smc'), "basic_smc not registered"
print('BasicPID/BasicSMC (M4) feature flags smoke test passed.')

# D1: MismatchDetector on KalmanFilter
_d1_A = np.array([[0.9]]); _d1_B = np.array([[0.1]]); _d1_C = np.array([[1.0]]); _d1_D = np.array([[0.0]])
_d1_sys = ctrl.StateSpace(_d1_A, _d1_B, _d1_C, _d1_D, 0.1)
_d1_kf  = ctrl.KalmanFilter(_d1_sys, 0.01 * np.eye(1), 0.1 * np.eye(1))
_d1_kf.enable_mismatch_detection(sigma=1.0, k_cusum=0.5, h_threshold=5.0)
assert not _d1_kf.mismatch_detected(), "KF mismatch should not fire on zero steps"
assert isinstance(_d1_kf.mismatch_score(), float), "mismatch_score should be float"
assert ctrl.registry_has('mismatch_detector'), "mismatch_detector not registered"
print('MismatchDetector (D1) smoke test passed.')

# Binding presence checks for classes tested in integration but not constructed here
assert hasattr(ctrl, 'ComputationalDelayWrapper'), "ComputationalDelayWrapper not bound"
assert hasattr(ctrl, 'CUSUMChart'),  "CUSUMChart not bound"
assert hasattr(ctrl, 'EWMAChart'),   "EWMAChart not bound"
assert hasattr(ctrl, 'SmithPredictor'), "SmithPredictor not bound"
assert hasattr(ctrl, 'ExtremumSeeker'), "ExtremumSeeker not bound"
assert hasattr(ctrl, 'make_lqr_controller'), "make_lqr_controller not bound"
print('Binding presence checks passed.')

# DeePC smoke test
assert hasattr(ctrl, 'DeePCParams'), "DeePCParams not bound"
assert hasattr(ctrl, 'DeePC'), "DeePC not bound"
assert ctrl.registry_has('deepc'), "deepc not registered"
_dp = ctrl.DeePCParams()
_dp.T_ini = 5; _dp.N = 5; _dp.Q = 1.0; _dp.R = 0.1
_dp.lambda_g = 1.0; _dp.lambda_y = 10.0; _dp.lambda_u = 5.0
_dp.uMin = -2.0; _dp.uMax = 2.0
_deepc = ctrl.DeePC(_dp, 0.1)
assert not _deepc.is_data_collected(), "should be False before collect_data"
import numpy as np
_N_d = _dp.T_ini + _dp.N + 20  # 30 samples
_u_d = np.random.uniform(-1.0, 1.0, _N_d)
_y_d = np.zeros(_N_d)
for _k in range(1, _N_d):
    _y_d[_k] = 0.7 * _y_d[_k-1] + 0.3 * _u_d[_k-1]
_deepc.collect_data(_u_d, _y_d)
assert _deepc.is_data_collected(), "collect_data should set flag"
assert _deepc.hankel_columns() == _N_d - _dp.T_ini - _dp.N + 1
_deepc.set_reference(1.0)
_u0 = _deepc.compute(0.0)
assert isinstance(_u0, float), "compute should return float"
assert _dp.uMin <= _u0 <= _dp.uMax, "output must respect uMin/uMax bounds"
_deepc.reset()
print('DeePC smoke test passed.')

# RobustnessAnalysis (Monte-Carlo, Phase 1) smoke test
assert hasattr(ctrl, 'MonteCarloResult'), "MonteCarloResult not bound"
assert hasattr(ctrl, 'RobustnessSample'), "RobustnessSample not bound"
assert hasattr(ctrl, 'monte_carlo_analysis'), "monte_carlo_analysis not bound"
assert ctrl.registry_has('robustness_analysis'), "robustness_analysis not registered"
import numpy as _npra
# Stable first-order plant x[k+1]=0.6 x + 0.4 u, y = x.
_ra_plant = ctrl.StateSpace(_npra.array([[0.6]]), _npra.array([[0.4]]),
                            _npra.array([[1.0]]), _npra.array([[0.0]]), 0.1)
# Static proportional controller u = 0.5 e.
_ra_ctrl = ctrl.StateSpace(_npra.zeros((1, 1)), _npra.zeros((1, 1)),
                           _npra.zeros((1, 1)), _npra.array([[0.5]]), 0.1)
_ra_ens = ctrl.spawn_ss_samples(_ra_plant, 8, 0.05, seed=7)
assert len(_ra_ens) == 8, "spawn_ss_samples returned wrong count"
_ra_res = ctrl.monte_carlo_analysis(_ra_plant, _ra_ctrl, 50, 0.05, seed=7)
assert _ra_res.n_samples == 50, "monte_carlo_analysis n_samples wrong"
assert 0.0 <= _ra_res.instability_probability <= 1.0, "instability_probability out of range"
assert _npra.isfinite(_ra_res.sensitivity_peak_stats.mean), "sensitivity stats not finite"
assert len(_ra_res.samples) == 50, "samples record wrong length"
print('RobustnessAnalysis (Phase 1) smoke test passed.')

# SystemAnalysis extensions (Gang of Four + Disk Margin, Phase 2) smoke test
assert hasattr(ctrl, 'DiskMargin'), "DiskMargin not bound"
assert hasattr(ctrl, 'GangOfFour'), "GangOfFour not bound"
assert hasattr(ctrl, 'GangOfFourNorms'), "GangOfFourNorms not bound"
_g4_G = ctrl.StateSpace(_npra.array([[0.6]]), _npra.array([[0.4]]),
                        _npra.array([[1.0]]), _npra.array([[0.0]]), 0.1)
_g4_K = ctrl.StateSpace(_npra.zeros((1, 1)), _npra.zeros((1, 1)),
                        _npra.zeros((1, 1)), _npra.array([[0.5]]), 0.1)
_g4 = ctrl.SystemAnalysis.gang_of_four(_g4_G, _g4_K)
assert isinstance(_g4.S, ctrl.StateSpace), "GangOfFour.S not a StateSpace"
_g4n = ctrl.SystemAnalysis.gang_of_four_norms(_g4)
# Closed loop: x+ = 0.4x + 0.2r (e=r-y, u=0.5e, y=x) -> pole at 0.4, T(DC)=0.2/0.6=1/3.
_g4_freqs = list(_npra.linspace(0.01, _npra.pi / 0.1 - 0.01, 200))
_g4_S_resp = _npra.array(ctrl.SystemAnalysis.get_frequency_response(_g4.S, _g4_freqs))
_g4_T_resp = _npra.array(ctrl.SystemAnalysis.get_frequency_response(_g4.T, _g4_freqs))
assert _npra.max(_npra.abs(_g4_S_resp + _g4_T_resp - 1.0)) < 1e-9, "S + T != I"
assert abs(_g4n.norm_T - 1.0 / 3.0) < 1e-3, "norm_T should equal T(DC)=1/3 (monotonic 1st-order loop)"
assert _g4n.norm_S > 1.0, "norm_S should exceed 1 (waterbed effect for this loop)"
_L = ctrl.SystemAnalysis.series(_g4_K, _g4_G)
_dm = ctrl.SystemAnalysis.calculate_disk_margin(_L)
assert _dm.alpha > 0.0, "disk margin alpha must be positive for a stable loop"
assert abs(_dm.alpha - 1.0 / _g4n.norm_S) < 1e-9, "alpha must equal 1/||S||_inf"
assert hasattr(ctrl.SystemAnalysis, 'get_singular_values'), "get_singular_values not bound"
_sv_freqs = [1.0, 5.0, 10.0]
_sv = ctrl.SystemAnalysis.get_singular_values(_g4_G, _sv_freqs)
assert len(_sv) == len(_sv_freqs), "get_singular_values: wrong number of frequency points"
_sv_resp = _npra.array(ctrl.SystemAnalysis.get_frequency_response(_g4_G, _sv_freqs))
for _i, _s in enumerate(_sv):
    assert len(_s) == 1, "SISO get_singular_values should return one value per frequency"
    assert abs(_s[0] - abs(_sv_resp[_i])) < 1e-9, "SISO singular value must equal |frequency response|"
print('SystemAnalysis extensions (Phase 2) smoke test passed.')

# FreqDomainIdentifier - Levy's method (Phase 4 Iteration 2) smoke test
assert hasattr(ctrl, 'FreqDomainIdentifier'), "FreqDomainIdentifier not bound"
assert hasattr(ctrl, 'FreqDomainFitResult'), "FreqDomainFitResult not bound"
_fdi_tf = ctrl.TransferFunction([0.0, 0.2], [1.0, -0.8], 0.1)
_fdi_sys = ctrl.tf2ss(_fdi_tf)
_fdi_freqs = list(_npra.linspace(0.5, 20.0, 50))
_fdi_response = ctrl.SystemAnalysis.get_frequency_response(_fdi_sys, _fdi_freqs)
_fdi_result = ctrl.FreqDomainIdentifier.fit_levy(_fdi_freqs, _fdi_response,
                                                  num_order=1, den_order=1, Ts=0.1)
assert _fdi_result.full_rank, "fit_levy: expected a full-rank system for this fixture"
assert abs(_fdi_result.tf.num[0] - 0.0) < 1e-9, "fit_levy: num[0] should recover 0.0"
assert abs(_fdi_result.tf.num[1] - 0.2) < 1e-9, "fit_levy: num[1] should recover 0.2"
assert abs(_fdi_result.tf.den[1] - (-0.8)) < 1e-9, "fit_levy: den[1] should recover -0.8"
assert _fdi_result.rmse < 1e-9, "fit_levy: rmse should be ~0 for noiseless exact-order data"
print('FreqDomainIdentifier smoke test passed.')

# CorrelationID - cross-correlation impulse-response identification (Phase 3 SI2) smoke test
assert hasattr(ctrl, 'CorrelationID'), "CorrelationID not bound"
assert hasattr(ctrl, 'CorrelationIDParams'), "CorrelationIDParams not bound"
assert hasattr(ctrl, 'CorrelationIDResult'), "CorrelationIDResult not bound"
assert ctrl.registry_has('correlation_id'), "correlation_id not registered"

_ci_u = ctrl.CorrelationID.generate_prbs(100, 5, seed=99)
assert len(_ci_u) == 100, "generate_prbs: wrong length"
assert np.all((_ci_u == 1.0) | (_ci_u == -1.0)), "generate_prbs: values must be +-1"
_ci_y = _ci_u * 0.5
_ci_params = ctrl.CorrelationIDParams()
_ci_params.max_lag = 20
_ci_result = ctrl.CorrelationID.identify(_ci_u, _ci_y, Ts=0.01, params=_ci_params)
assert len(_ci_result.impulse_response) == 21, "identify: impulse_response length mismatch"
assert len(_ci_result.autocorr_u) == 21, "identify: autocorr_u length mismatch"
assert len(_ci_result.crosscorr_uy) == 21, "identify: crosscorr_uy length mismatch"
assert abs(_ci_result.impulse_response[0] - 0.5) < 1e-9, \
    "identify: g_hat(0) should recover the static gain 0.5"
print('CorrelationID smoke test passed.')

# SKFit - Sanathanan-Koerner-reweighted complex-response fitting (Phase 3 FD1) smoke test
assert hasattr(ctrl, 'SKFit'), "SKFit not bound"
assert hasattr(ctrl, 'SKFitResult'), "SKFitResult not bound"
assert ctrl.registry_has('sk_fit'), "sk_fit not registered"

_sk_tf = ctrl.TransferFunction([0.0, 0.2], [1.0, -0.8], 0.1)
_sk_sys = ctrl.tf2ss(_sk_tf)
_sk_freqs = list(np.linspace(0.5, 20.0, 50))
_sk_response = ctrl.SystemAnalysis.get_frequency_response(_sk_sys, _sk_freqs)
_sk_result = ctrl.SKFit.fit_sk(_sk_freqs, _sk_response, num_order=1, den_order=1, Ts=0.1)
assert len(_sk_result.iter_cost) >= 1, "fit_sk: expected at least one iteration"
assert abs(_sk_result.model.den[1] - (-0.8)) < 1e-6, "fit_sk: den[1] should recover -0.8"
print('SKFit smoke test passed.')

# HammersteinWienerIdentifier - Hammerstein/Wiener structured nonlinear ID (Phase 3 SI5)
# smoke test
assert hasattr(ctrl, 'HammersteinWienerIdentifier'), "HammersteinWienerIdentifier not bound"
assert hasattr(ctrl, 'HammersteinWienerParams'), "HammersteinWienerParams not bound"
assert hasattr(ctrl, 'HammersteinWienerResult'), "HammersteinWienerResult not bound"
assert ctrl.registry_has('hammerstein_wiener'), "hammerstein_wiener not registered"

_hw_rng = np.random.default_rng(3)
_hw_u = _hw_rng.uniform(-1.0, 1.0, 300)
_hw_v = _hw_u + 0.3 * _hw_u ** 3  # known cubic static nonlinearity, linear term = 1
_hw_y = np.zeros(300)
for _k in range(1, 300):
    _hw_y[_k] = 0.8 * _hw_y[_k - 1] + 0.5 * _hw_v[_k - 1]
_hw_params = ctrl.HammersteinWienerParams()
_hw_params.na = 1
_hw_params.nb = 1
_hw_result = ctrl.HammersteinWienerIdentifier.fit_hammerstein(_hw_u, _hw_y, Ts=0.1, params=_hw_params)
assert len(_hw_result.nl_input_coeffs) == _hw_params.nl_degree + 1, "nl_input_coeffs wrong length"
assert abs(_hw_result.nl_input_coeffs[1] - 1.0) < 1e-9, \
    "nl_input_coeffs[1] should be normalized to 1.0"
print('HammersteinWienerIdentifier smoke test passed.')

# MuAnalysis (Structured Singular Value, Phase 3) smoke test
assert hasattr(ctrl, 'UncertaintyStructure'), "UncertaintyStructure not bound"
assert hasattr(ctrl, 'UncertaintyBlock'), "UncertaintyBlock not bound"
assert hasattr(ctrl, 'MuBound'), "MuBound not bound"
assert hasattr(ctrl, 'PeakMuResult'), "PeakMuResult not bound"
assert hasattr(ctrl, 'peak_mu'), "peak_mu not bound"
assert hasattr(ctrl, 'compute_mu'), "compute_mu not bound"
assert hasattr(ctrl, 'robust_stability_radius'), "robust_stability_radius not bound"
assert ctrl.registry_has('mu_analysis'), "mu_analysis not registered"

_mu_block = ctrl.UncertaintyBlock()
_mu_block.type = ctrl.UncertaintyBlock.Type.ComplexFull
_mu_block.r_out = 1
_mu_block.r_in = 1
_mu_struc = ctrl.UncertaintyStructure()
_mu_struc.blocks = [_mu_block]
assert _mu_struc.total_outputs() == 1, "total_outputs() wrong"
assert _mu_struc.total_inputs() == 1, "total_inputs() wrong"

# Reuse the gang-of-four fixture: norm_T = 1/3 (derived above), single ComplexFull
# block spanning the SISO output -> peak.upper should equal sigma_rel * norm_T exactly.
_mu_peak = ctrl.peak_mu(_g4_G, _g4_K, _mu_struc, sigma_rel=1.0, freq_points=50, omega_min=1e-4)
assert abs(_mu_peak.peak.upper - 1.0 / 3.0) < 1e-3, "peak_mu should match norm_T=1/3"
assert len(_mu_peak.mu_curve) == 50, "mu_curve wrong length"

_mu_radius = ctrl.robust_stability_radius(_g4_G, _g4_K, _mu_struc, sigma_max=5.0)
assert abs(_mu_radius - 3.0) < 1e-2, "robust_stability_radius should match 1/norm_T=3.0"
print('MuAnalysis (Phase 3) smoke test passed.')

# LFTSystem - general multi-block LFT/Delta channel-gather (Phase 3 RC1) smoke test
assert hasattr(ctrl, 'LFTSystem'), "LFTSystem not bound"
assert hasattr(ctrl, 'LFTChannelMap'), "LFTChannelMap not bound"
assert ctrl.registry_has('lft_system'), "lft_system not registered"

# Degenerate single-block case: M0 = sigma_rel * T should reproduce peak_mu()'s result.
_lft_T = _g4.T
_lft_M0 = ctrl.StateSpace(_lft_T.A, _lft_T.B, _lft_T.C, _lft_T.D, _lft_T.Ts)
_lft_map = ctrl.LFTChannelMap()
_lft_map.row_start = [0]
_lft_map.col_start = [0]
_lft_system = ctrl.LFTSystem(_lft_M0, _mu_struc, _lft_map)
_lft_peak = _lft_system.peak_mu(freq_points=50, omega_min=1e-4)
assert abs(_lft_peak.peak.upper - _mu_peak.peak.upper) < 1e-9, \
    "LFTSystem degenerate single-block case should reproduce peak_mu() exactly"
print('LFTSystem smoke test passed.')

# BacksteppingController (Phase 3 NC1) smoke test
assert hasattr(ctrl, 'BacksteppingController'), "BacksteppingController not bound"
assert ctrl.registry_has('backstepping_controller'), "backstepping_controller not registered"
_bc_params = ctrl.BacksteppingParams()
_bc_params.k_gains = [2.0, 2.0]
_bc = ctrl.BacksteppingController(
    [lambda x, s: 0.0, lambda x, s: 0.0],
    [lambda x, s: 1.0, lambda x, s: 1.0],
    _bc_params, 0.01)
_bc.set_state(np.array([0.0, 0.0]))
_u_bc = _bc.compute(1.0)
assert np.isfinite(_u_bc), "BacksteppingController output not finite"
print('BacksteppingController smoke test passed.')

# PassivityBasedController (Phase 3 NC2) smoke test
assert hasattr(ctrl, 'PassivityBasedController'), "PassivityBasedController not bound"
assert ctrl.registry_has('passivity_based_controller'), "passivity_based_controller not registered"
_pbc_params = ctrl.PBCParams()
_pbc_params.Kp = np.array([[5.0]])
_pbc_params.Kd = np.array([[1.0]])
_pbc = ctrl.PassivityBasedController(
    lambda q: np.array([[1.0]]),
    lambda q: np.array([9.8 * np.sin(q[0])]),
    lambda q, qdot: np.array([[0.0]]),
    _pbc_params, 0.01)
_pbc.set_desired(np.array([0.5]))
_u_pbc = _pbc.compute_vec(np.array([0.0, 0.0]))
assert np.all(np.isfinite(_u_pbc)), "PassivityBasedController output not finite"
assert np.isfinite(_pbc.storage_energy()), "storage_energy not finite"
try:
    _pbc.compute(0.0)
    assert False, "PassivityBasedController.compute() should raise"
except RuntimeError:
    pass
print('PassivityBasedController smoke test passed.')

# CLFController (Phase 3 NC4) smoke test
assert hasattr(ctrl, 'CLFController'), "CLFController not bound"
assert ctrl.registry_has('clf_controller'), "clf_controller not registered"
_clf_params = ctrl.CLFParams()
_clf = ctrl.CLFController(
    lambda x: float(x[0] ** 2),
    lambda x: 0.0,
    lambda x: 2.0 * float(x[0]),
    _clf_params, 0.01)
_clf.set_state(np.array([1.0]))
_u_clf = _clf.compute(0.0)
assert np.isfinite(_u_clf), "CLFController output not finite"
assert _clf.is_healthy()
print('CLFController smoke test passed.')

# WorstCaseSearch (CMA-ES worst-case parameter search, Phase 4) smoke test
assert hasattr(ctrl, 'WorstCaseResult'), "WorstCaseResult not bound"
assert hasattr(ctrl, 'WorstCaseSearchParams'), "WorstCaseSearchParams not bound"
assert hasattr(ctrl, 'find_worst_case_sensitivity'), "find_worst_case_sensitivity not bound"
assert hasattr(ctrl, 'find_worst_case_iae'), "find_worst_case_iae not bound"
assert hasattr(ctrl, 'find_worst_case'), "find_worst_case not bound"
assert ctrl.registry_has('worst_case_search'), "worst_case_search not registered"


def _wc_plant_factory(params):
    a = float(params[0])
    return ctrl.StateSpace(_npra.array([[a]]), _npra.array([[0.4]]),
                            _npra.array([[1.0]]), _npra.array([[0.0]]), 0.1)


_wc_ctrl = ctrl.StateSpace(_npra.zeros((1, 1)), _npra.zeros((1, 1)),
                           _npra.zeros((1, 1)), _npra.array([[0.5]]), 0.1)
_wc_p = ctrl.WorstCaseSearchParams()
_wc_p.max_evals = 80
_wc_res = ctrl.find_worst_case_sensitivity(_wc_plant_factory, _wc_ctrl,
                                            _npra.array([0.6]), _npra.array([0.3]),
                                            params=_wc_p)
assert isinstance(_wc_res, ctrl.WorstCaseResult), "find_worst_case_sensitivity wrong return type"
assert _npra.isfinite(_wc_res.worst_cost), "worst_cost should be finite for a stable search box"
assert _wc_res.n_evals > 0, "n_evals should be positive"

_wc_res_generic = ctrl.find_worst_case(_wc_plant_factory,
                                        lambda ss: abs(ss.A[0, 0]),
                                        _npra.array([0.6]), _npra.array([0.3]),
                                        params=_wc_p)
assert _wc_res_generic.worst_cost >= abs(0.6) - 1e-9, "generic find_worst_case should find |a| >= |nominal a|"
print('WorstCaseSearch (Phase 4) smoke test passed.')

# LyapunovRobustness (common quadratic Lyapunov function, Phase 5) smoke test
assert hasattr(ctrl, 'LyapunovResult'), "LyapunovResult not bound"
assert hasattr(ctrl, 'LyapunovSearchParams'), "LyapunovSearchParams not bound"
assert hasattr(ctrl, 'find_common_lyapunov'), "find_common_lyapunov not bound"
assert hasattr(ctrl, 'is_quadratically_stable'), "is_quadratically_stable not bound"
assert hasattr(ctrl, 'build_box_vertices'), "build_box_vertices not bound"
assert ctrl.registry_has('lyapunov_robustness'), "lyapunov_robustness not registered"

_lyap_A = _npra.array([[0.5]])
_lyap_res = ctrl.find_common_lyapunov([_lyap_A])
assert _lyap_res.found, "single stable vertex should always admit a common Lyapunov function"
assert _lyap_res.P[0, 0] > 0.0, "P must be positive definite"

_lyap_delta = _npra.array([[0.05]])  # n*n x m = 1x1, single direction
_lyap_vertices = ctrl.build_box_vertices(_lyap_A, _lyap_delta)
assert len(_lyap_vertices) == 2, "build_box_vertices should return 2^1 = 2 vertices"
assert _npra.isclose(_lyap_vertices[0][0, 0], 0.45) or _npra.isclose(_lyap_vertices[0][0, 0], 0.55)

assert ctrl.is_quadratically_stable(_lyap_vertices), "small box around a stable vertex should be quadratically stable"
assert not ctrl.is_quadratically_stable([_npra.array([[1.5]])]), "unstable vertex must fail"
print('LyapunovRobustness (Phase 5) smoke test passed.')

# ---------------------------------------------------------------------------
# Additional Controller Types: ResonantController
# ---------------------------------------------------------------------------
rp = ctrl.ResonantParams()
rp.targetFreqHz = 50.0
rp.dampingRadPerSec = 5.0
rp.Kr = 2.0
rc = ctrl.ResonantController(rp, 1e-4)
u_rc = rc.compute(1.0)
assert math.isfinite(u_rc)
print(f'ResonantController compute(1.0) = {u_rc:.4f}')

# ---------------------------------------------------------------------------
# Additional Controller Types: NotchFilter
# ---------------------------------------------------------------------------
nfp = ctrl.NotchFilterParams()
nfp.centerFreqHz = 50.0
nfp.Q = 10.0
nf = ctrl.NotchFilter(nfp, 1e-4)
y_nf = nf.apply(1.0)
assert math.isfinite(y_nf)
print(f'NotchFilter apply(1.0) = {y_nf:.4f}')

# ---------------------------------------------------------------------------
# Additional Controller Types: PhaseLockedLoop
# ---------------------------------------------------------------------------
pllp = ctrl.PLLParams()
pllp.nominalFreqHz = 50.0
pllp.Kp = 90.0
pllp.Ki = 4000.0
pll = ctrl.PhaseLockedLoop(pllp, 1e-4)
pll.step(1.0)
assert math.isfinite(pll.frequency_hz())
print(f'PhaseLockedLoop frequency_hz() after one step = {pll.frequency_hz():.4f}')

print('\nAll smoke tests passed.')
