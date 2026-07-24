classdef PID < bctrl.Controller
% PID  Three diagonal discrete PIDs, e = ref_dy - dy.
    properties, pids; end
    methods
        function obj = PID(sys, ~)
            p = bctrl.pidParams();
            obj.pids = {bctrl.DPID(p,sys.Ts), bctrl.DPID(p,sys.Ts), bctrl.DPID(p,sys.Ts)};
        end
        function du = compute(obj, ref_dy, dy)
            e = ref_dy(:) - dy(:);
            du = [obj.pids{1}.compute(e(1)); obj.pids{2}.compute(e(2)); obj.pids{3}.compute(e(3))];
        end
        function reset(obj), for i=1:3, obj.pids{i}.reset(); end, end
        function n = name(~), n = 'PID'; end
    end
end
