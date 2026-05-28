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

print('\nAll smoke tests passed.')
