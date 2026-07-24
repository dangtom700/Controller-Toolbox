% verify_ex01_tf_pid.m
%
% Phase-2 exit-criterion check: a MATLAB +ctrl step-loop reproduces the C++
% example examples/ex01_tf_pid.cpp (compiled: build/examples/ex01_tf_pid.exe).
%
% Two parts:
%   (A) Exact plant port  -- tf2ss A/B/C/D and DC gain match the C++ output to full
%       printed precision (these are determined by the TF coefficients alone).
%   (B) Closed-loop PID   -- y/e/u at k = 0,150,...,1500 match the C++ trajectory.
%       The limiting precision here is the 3-decimal IMC-PID gains the C++ demo prints
%       (Kp=1.423, Ki=1.013, Kd=0.306); we feed those rounded gains, so the tolerance
%       below reflects that, not a port defect.
%
% Run:  matlab -batch "run('MATLAB/examples/verify_ex01_tf_pid.m')"

addpath(fullfile(fileparts(mfilename('fullpath')), '..'));   % put MATLAB/ (the +ctrl pkg) on path

fprintf('== verify_ex01_tf_pid ==\n\n');
allPass = true;

% ---- Plant: same z^-1 transfer function as ex01_tf_pid.cpp ----
Ts = 0.01;
plant_tf = ctrl.TransferFunction([0.0, 4.9625e-5, 4.9125e-5], ...
                                 [1.0, -1.98511, 0.98522], Ts);
plant = ctrl.tf2ss(plant_tf);

% (A) Exact plant checks against the C++ printed matrices / DC gain.
A_exp  = [1.98511, -0.98522; 1, 0];
B_exp  = [1; 0];
C_exp  = [4.9625e-5, 4.9125e-5];
dc_exp = 0.897727;

plantTol = 1e-4;
checkA  = max(abs(plant.A(:) - A_exp(:)));
checkB  = max(abs(plant.B(:) - B_exp(:)));
checkC  = max(abs(plant.C(:) - C_exp(:)));
checkD  = abs(plant.D - 0.0);
checkDC = abs(plant.dcgain() - dc_exp);

fprintf('(A) plant port (tol %.0e):\n', plantTol);
fprintf('    max|A-A_exp|=%.2e  max|B-B_exp|=%.2e  max|C-C_exp|=%.2e  |D|=%.2e  |dcgain-%.6f|=%.2e\n', ...
        checkA, checkB, checkC, checkD, dc_exp, checkDC);
if max([checkA, checkB, checkC, checkD, checkDC]) > plantTol
  allPass = false; fprintf('    [FAIL] plant matrices/DC gain off\n');
else
  fprintf('    [PASS]\n');
end

% (B) Closed-loop unit-step, identical loop structure to ex01.
pp = ctrl.DiscretePID.defaultParams();
pp.Kp = 1.423; pp.Ki = 1.013; pp.Kd = 0.306;   % IMC-PID gains printed by ex01
pp.N = 20.0; pp.uMin = -5.0; pp.uMax = 5.0;
% StepResponseTuner sets Kb = sqrt(|Ki*Kd|) (ControllerTuner.cpp); ex01 overrides
% N/uMin/uMax but NOT Kb, so it keeps the tuner's value (not the PIDParams default 1.0).
pp.Kb = sqrt(abs(pp.Ki * pp.Kd));
pid = ctrl.DiscretePID(pp, Ts);

x = zeros(plant.stateSize(), 1);
y = 0.0; ref = 1.0;
N = 1500;
yhist = zeros(1, N + 1); ehist = yhist; uhist = yhist;
for k = 0:N
  e = ref - y;
  u = pid.compute(e);
  [y, x] = plant.step(x, u);
  yhist(k + 1) = y; ehist(k + 1) = e; uhist(k + 1) = u;
end

% Reference trajectory from build/examples/ex01_tf_pid.exe (printed every 150 steps).
kref = 0:150:1500;
yref = [0.000 0.500 0.828 0.923 0.959 0.984 0.994 0.997 0.999 0.999 1.000];
eref = [1.000 0.503 0.173 0.077 0.041 0.016 0.006 0.003 0.001 0.001 0.000];
uref = [5.000 0.926 0.991 1.056 1.095 1.105 1.109 1.112 1.113 1.114 1.114];

trajTol = 5e-3;   % limited by the 3-decimal printed gains, not the port
fprintf('\n(B) closed-loop trajectory (tol %.0e):\n', trajTol);
fprintf('    %6s %10s %10s %8s | %10s %10s %8s\n', 'k', 'y', 'y_ref', 'dy', 'u', 'u_ref', 'du');
maxdev = 0;
for i = 1:numel(kref)
  idx = kref(i) + 1;
  dy = abs(yhist(idx) - yref(i));
  du = abs(uhist(idx) - uref(i));
  de = abs(ehist(idx) - eref(i));
  maxdev = max([maxdev, dy, du, de]);
  fprintf('    %6d %10.4f %10.3f %8.1e | %10.4f %10.3f %8.1e\n', ...
          kref(i), yhist(idx), yref(i), dy, uhist(idx), uref(i), du);
end
fprintf('    max deviation (y,e,u) = %.2e\n', maxdev);
if maxdev > trajTol
  allPass = false; fprintf('    [FAIL] trajectory deviates beyond tol\n');
else
  fprintf('    [PASS]\n');
end

fprintf('\n');
assert(allPass, 'verify_ex01_tf_pid: FAIL');
fprintf('verify_ex01_tf_pid: PASS - +ctrl reproduces ex01_tf_pid.cpp\n');
