classdef NMPC < bctrl.Controller
% NMPC  Nonlinear MPC by successive online linearisation: re-linearise the
%   Bell-Astrom deviation dynamics about the current state every 10 steps and
%   resolve the condensed QP (Np=10, Nu=3).
    properties, mpc; op; Ts; k = 0; relin = 10; end
    methods
        function obj = NMPC(sys, op)
            obj.mpc = bctrl.CondensedMPC(sys, 10, 3, 1.0, 0.1, -0.5, 0.5, -0.02, 0.02);
            obj.op = op; obj.Ts = sys.Ts;
        end
        function du = compute(obj, ref_dy, dy)
            if obj.k > 0 && mod(obj.k, obj.relin) == 0
                oc = obj.op; oc.x1 = obj.op.x1 + dy(1); oc.x2 = obj.op.x2 + dy(2);
                try
                    obj.mpc.setPlant(boiler.linearize(oc, obj.Ts));
                catch
                end
            end
            du = obj.mpc.solve(dy(:), ref_dy(:));
            obj.k = obj.k + 1;
        end
        function reset(obj), obj.mpc.reset(); obj.k = 0; end
        function n = name(~), n = 'NMPC'; end
    end
end
