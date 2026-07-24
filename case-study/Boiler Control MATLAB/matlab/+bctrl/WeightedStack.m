classdef WeightedStack < bctrl.Controller
% WeightedStack  Per-axis blend of two PIDs, weighted by drum pressure:
%   w_pid = clamp((x1-75)/65, 0, 1), w_lqr = 1-w_pid.
    properties, pA; pB; op; end
    methods
        function obj = WeightedStack(sys, op)
            pp = bctrl.pidParams();
            obj.pA = {}; obj.pB = {};
            for i = 1:3
                obj.pA{i} = bctrl.DPID(pp, sys.Ts);
                obj.pB{i} = bctrl.DPID(pp, sys.Ts);
            end
            obj.op = op;
        end
        function du = compute(obj, ref_dy, dy)
            wp = min(max((obj.op.x1 + dy(1) - 75.0)/65.0, 0), 1);
            wl = 1 - wp;
            e = ref_dy(:) - dy(:);
            du = zeros(3,1);
            for i = 1:3
                du(i) = wp*obj.pA{i}.compute(e(i)) + wl*obj.pB{i}.compute(e(i));
            end
        end
        function reset(obj)
            for i=1:3, obj.pA{i}.reset(); obj.pB{i}.reset(); end
        end
        function n = name(~), n = 'WeightedStack'; end
    end
end
