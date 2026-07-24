% verify_lqr_kalman.m
%
% Internal-consistency checks for the +ctrl DiscreteLQR / LQRAdapter / KalmanFilter port.
% These need no external reference: they assert the mathematical properties the C++
% classes guarantee (DARE residual ~ 0, closed-loop stability, observer convergence),
% plus an optional cross-check against MATLAB's dlqr() if the Control System Toolbox
% is present.
%
% Run:  matlab -batch "run('MATLAB/examples/verify_lqr_kalman.m')"

addpath(fullfile(fileparts(mfilename('fullpath')), '..'));

fprintf('== verify_lqr_kalman ==\n\n');
allPass = true;

% ---- Plant: ZOH double integrator, Ts = 0.1 s ----
Ts = 0.1;
A = [1, Ts; 0, 1];
B = [0.5 * Ts^2; Ts];
C = [1, 0];
D = 0;
plant = ctrl.StateSpace(A, B, C, D, Ts);

% ================= DiscreteLQR =================
Q = eye(2); R = 1;
lqr = ctrl.DiscreteLQR(plant, struct('Q', Q, 'R', R));
P = lqr.P; K = lqr.K;

fprintf('DiscreteLQR:\n');
fprintf('    dareConverged=%d  iterations=%d\n', lqr.dareConverged, lqr.dareIterations);
if ~lqr.dareConverged, allPass = false; fprintf('    [FAIL] DARE did not converge\n'); end

% (1) DARE residual: A'PA - A'PB (R+B'PB)^-1 B'PA + Q - P ~ 0
S   = R + B.' * P * B;
res = A.' * P * A - (A.' * P * B) * (S \ (B.' * P * A)) + Q - P;
dareRes = norm(res, 'fro');
fprintf('    DARE residual ||.||_F = %.2e (tol 1e-6)\n', dareRes);
if dareRes > 1e-6, allPass = false; fprintf('    [FAIL] DARE residual too large\n'); end

% (2) Gain identity: K = (R+B'PB)^-1 B'PA
Kexp = S \ (B.' * P * A);
dK = norm(K - Kexp, 'fro');
fprintf('    ||K - (R+B''PB)^-1 B''PA||_F = %.2e (tol 1e-10)\n', dK);
if dK > 1e-10, allPass = false; fprintf('    [FAIL] gain identity\n'); end

% (3) Closed-loop stability: spectral radius of A - B K < 1
rho = max(abs(eig(A - B * K)));
fprintf('    spectral radius of (A - B K) = %.4f (must be < 1)\n', rho);
if rho >= 1.0, allPass = false; fprintf('    [FAIL] closed loop not stable\n'); end

% (4) Regulation to origin from a nonzero state
x = [1; 0.5];
for k = 1:300
  u = lqr.compute(x);
  x = A * x + B * u;
end
regErr = norm(x);
fprintf('    ||x|| after 300 steps of regulation = %.2e (tol 1e-4)\n', regErr);
if regErr > 1e-4, allPass = false; fprintf('    [FAIL] regulation did not converge\n'); end

% (5) Optional cross-check against MATLAB dlqr() (Control System Toolbox).
if exist('dlqr', 'file') > 0
  Kml = dlqr(A, B, Q, R);
  dKml = norm(K - Kml, 'fro');
  fprintf('    cross-check vs dlqr(): ||K - K_dlqr||_F = %.2e (tol 1e-6)\n', dKml);
  if dKml > 1e-6, allPass = false; fprintf('    [FAIL] disagrees with dlqr()\n'); end
else
  fprintf('    (dlqr() unavailable - skipping Control System Toolbox cross-check)\n');
end

% ================= LQRAdapter =================
fprintf('\nLQRAdapter:\n');
xfix = [2; -1];
adapter = ctrl.LQRAdapter(lqr, @() xfix);
ua = adapter.compute([]);
ud = lqr.compute(xfix);
dAdapt = abs(ua - ud(1));
fprintf('    adapter.compute vs lqr.compute u(1): diff = %.2e (tol 1e-12)\n', dAdapt);
if dAdapt > 1e-12, allPass = false; fprintf('    [FAIL] adapter mismatch\n'); end
if ~adapter.isHealthy(), allPass = false; fprintf('    [FAIL] adapter reports unhealthy\n'); end

% ================= KalmanFilter =================
fprintf('\nKalmanFilter (noiseless observer convergence):\n');
kf = ctrl.KalmanFilter(plant, 1e-4 * eye(2), 1e-2);
x_true = [1; 0];    % u = 0 keeps x_true fixed (A*[1;0] = [1;0]); estimate must converge to it
u_prev = 0;
errHist = zeros(1, 200);
for k = 1:200
  x_true = A * x_true + B * u_prev;   % x[k]
  y = C * x_true;                     % noiseless measurement y[k]
  kf.step(y, u_prev);                 % predict(u_prev) + update(y)
  errHist(k) = norm(x_true - kf.state());
end
% covariance must stay symmetric PSD
Psym = norm(kf.P - kf.P.', 'fro');
Pmineig = min(eig((kf.P + kf.P.') / 2));
fprintf('    final ||x_true - xhat|| = %.2e (tol 1e-2)\n', errHist(end));
fprintf('    covariance symmetry ||P-P''||_F = %.2e ; min eig = %.2e\n', Psym, Pmineig);
if errHist(end) > 1e-2, allPass = false; fprintf('    [FAIL] estimate did not converge\n'); end
if Psym > 1e-9 || Pmineig < -1e-12, allPass = false; fprintf('    [FAIL] covariance not symmetric PSD\n'); end

fprintf('\n');
assert(allPass, 'verify_lqr_kalman: FAIL');
fprintf('verify_lqr_kalman: PASS - DiscreteLQR / LQRAdapter / KalmanFilter port verified\n');
