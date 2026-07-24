classdef LeadLagPID < bctrl.Controller
% LeadLag-PID  Per-axis lead-lag pre-filter (z=0.01, p=0.05) into a discrete PID.
    properties, ll; pids; end
    methods
        function obj = LeadLagPID(sys, ~)
            p = bctrl.pidParams();
            obj.ll = cell(1,3); obj.pids = cell(1,3);
            for i = 1:3
                obj.ll{i}   = bctrl.LeadLag(0.01, 0.05, 1.0, sys.Ts);
                obj.pids{i} = bctrl.DPID(p, sys.Ts);
            end
        end
        function du = compute(obj, ref_dy, dy)
            e = ref_dy(:) - dy(:);
            du = zeros(3,1);
            for i = 1:3
                du(i) = obj.pids{i}.compute(obj.ll{i}.compute(e(i)));
            end
        end
        function reset(obj)
            for i=1:3, obj.ll{i}.reset(); obj.pids{i}.reset(); end
        end
        function n = name(~), n = 'LeadLag-PID'; end
    end
end
