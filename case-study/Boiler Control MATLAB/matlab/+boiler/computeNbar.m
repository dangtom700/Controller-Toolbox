function N = computeNbar(sys)
% COMPUTENBAR  Steady-state feedforward gain for setpoint tracking.
%   N = -(C (A-I)^{-1} B)^{-1}.  Returns identity if either inverse is singular.
AmI = sys.A - eye(size(sys.A));
if abs(det(AmI)) < 1e-12, N = eye(size(sys.C,1)); return; end
M = sys.C * (AmI \ sys.B);
if abs(det(M)) < 1e-12, N = eye(size(sys.C,1)); return; end
N = -inv(M);
end
