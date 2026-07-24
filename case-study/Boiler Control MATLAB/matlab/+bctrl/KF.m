classdef KF < handle
% KF  Steady-state linear Kalman observer for (A,B,C) with process cov Q and
%   measurement cov R.  Gain from dlqe (Control System Toolbox), with an idare
%   fallback.  Current-estimator form: xhat = xpred + L*(y - C*xpred).
    properties
        A; B; C; L; x
        n
    end
    methods
        function obj = KF(A, B, C, Q, R)
            obj.A = A; obj.B = B; obj.C = C; obj.n = size(A,1);
            obj.L = local_gain(A, C, Q, R);
            obj.x = zeros(obj.n, 1);
        end
        function xhat = step(obj, y, u)
            xpred = obj.A * obj.x + obj.B * u(:);
            obj.x = xpred + obj.L * (y(:) - obj.C * xpred);
            xhat  = obj.x;
        end
        function setState(obj, x0), obj.x = x0(:); end
        function reset(obj),        obj.x = zeros(obj.n, 1); end
    end
end

function L = local_gain(A, C, Q, R)
try
    L = dlqe(A, eye(size(A,1)), C, Q, R);
catch
    % Filter ARE via duality; current-estimator gain.
    P = idare(A', C', Q, R, [], []);
    L = (P * C') / (C * P * C' + R);
end
end
