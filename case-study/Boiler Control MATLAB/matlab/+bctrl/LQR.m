classdef LQR < bctrl.Controller
% LQR  Bryson-weighted discrete LQR (dlqr) with steady-state feedforward Nbar.
%   du = -K*dy + Nbar*ref_dy (dy used as the state-deviation proxy).
    properties, K; Nbar; end
    methods
        function obj = LQR(sys, ~)
            [Q,R] = bctrl.brysonWeights([5;10;1], [0.3;0.3;0.1]);
            obj.K    = dlqr(sys.A, sys.B, Q, R);
            obj.Nbar = boiler.computeNbar(sys);
        end
        function du = compute(obj, ref_dy, dy)
            du = -obj.K*dy(:) + obj.Nbar*ref_dy(:);
            du = min(max(du, -0.5), 0.5);
        end
        function reset(~), end
        function n = name(~), n = 'LQR'; end
    end
end
