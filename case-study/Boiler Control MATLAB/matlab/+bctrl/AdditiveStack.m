classdef AdditiveStack < bctrl.Controller
% AdditiveStack  u = PID + w*LeadLag with the lead-lag weight faded 1->0 over
%   the first 300 steps (hand off from transient shaping to steady PID).
    properties, pids; ll; k = 0; fade = 300; end
    methods
        function obj = AdditiveStack(sys, ~)
            pp = bctrl.pidParams();
            obj.pids = {}; obj.ll = {};
            for i = 1:3
                obj.pids{i} = bctrl.DPID(pp, sys.Ts);
                obj.ll{i}   = bctrl.LeadLag(0.01, 0.05, 1.0, sys.Ts);
            end
        end
        function du = compute(obj, ref_dy, dy)
            w = max(0, 1 - obj.k/obj.fade);
            e = ref_dy(:) - dy(:);
            du = zeros(3,1);
            for i = 1:3
                du(i) = obj.pids{i}.compute(e(i)) + w*obj.ll{i}.compute(e(i));
            end
            du = min(max(du, -0.5), 0.5);
            obj.k = obj.k + 1;
        end
        function reset(obj)
            for i=1:3, obj.pids{i}.reset(); obj.ll{i}.reset(); end
            obj.k = 0;
        end
        function n = name(~), n = 'AdditiveStack'; end
    end
end
