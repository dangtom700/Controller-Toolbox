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

print('\nAll smoke tests passed.')
