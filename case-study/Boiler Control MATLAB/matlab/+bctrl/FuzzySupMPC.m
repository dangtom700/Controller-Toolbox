classdef FuzzySupMPC < bctrl.Controller
% FuzzySup-MPC  MIMO condensed-QP MPC with a supervisor that re-linearises the
%   internal model (setPlant) when the tracking error stays large, subject to a
%   cooldown -- a fuzzy-supervised successive-linearisation MPC.
    properties, mpc; op; Ts; cooldown = 0; end
    methods
        function obj = FuzzySupMPC(sys, op)
            obj.mpc = bctrl.CondensedMPC(sys, 20, 5, 1.0, 0.1, -0.5, 0.5, -0.02, 0.02);
            obj.op = op; obj.Ts = sys.Ts;
        end
        function du = compute(obj, ref_dy, dy)
            emax = max(abs(ref_dy(:) - dy(:)));
            if obj.cooldown > 0, obj.cooldown = obj.cooldown - 1; end
            if emax > 5.0 && obj.cooldown == 0
                oc = obj.op; oc.x1 = obj.op.x1 + dy(1); oc.x2 = obj.op.x2 + dy(2);
                try
                    obj.mpc.setPlant(boiler.linearize(oc, obj.Ts));
                catch
                end
                obj.cooldown = 120;
            end
            du = obj.mpc.solve(dy(:), ref_dy(:));
        end
        function reset(obj), obj.mpc.reset(); obj.cooldown = 0; end
        function n = name(~), n = 'FuzzySup-MPC'; end
    end
end
