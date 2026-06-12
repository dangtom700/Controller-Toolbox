"""Quick smoke test for ctrl_toolbox Python bindings."""
import sys
import os

# On Windows, the MinGW runtime DLLs (libstdc++, libgcc, libwinpthread) must be
# reachable before importing the .pyd.  Add the MinGW bin dir explicitly.
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

# Load the .pyd from the build output directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build', 'bindings'))

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
else:
    print('Skipping Hinf: not compiled in.')

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

print('\nAll smoke tests passed.')
