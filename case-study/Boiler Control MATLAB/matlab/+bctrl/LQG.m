classdef LQG < bctrl.Controller
% LQG  LQR state feedback on a Kalman-observed state estimate.
    properties, K; Nbar; kf; du_prev; end
    methods
        function obj = LQG(sys, ~)
            [Q,R] = bctrl.brysonWeights([5;10;1], [0.3;0.3;0.1]);
            obj.K    = dlqr(sys.A, sys.B, Q, R);
            obj.Nbar = boiler.computeNbar(sys);
            obj.kf   = bctrl.KF(sys.A, sys.B, sys.C, 1e-4*eye(3), diag([0.25 1.0 25.0]));
            obj.du_prev = [0;0;0];
        end
        function du = compute(obj, ref_dy, dy)
            xhat = obj.kf.step(dy(:), obj.du_prev);
            du   = -obj.K*xhat + obj.Nbar*ref_dy(:);
            du   = min(max(du, -0.5), 0.5);
            obj.du_prev = du;
        end
        function reset(obj), obj.kf.reset(); obj.du_prev = [0;0;0]; end
        function n = name(~), n = 'LQG'; end
    end
end
