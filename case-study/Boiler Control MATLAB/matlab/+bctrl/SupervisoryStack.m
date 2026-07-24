classdef SupervisoryStack < bctrl.Controller
% SupervisoryStack  Per-axis supervisory switch: sliding-mode control when the
%   error is large (|e|>5), PID otherwise.  Both run every step (kept warm) for
%   bumpless transfer.
    properties, pids; smcs; end
    methods
        function obj = SupervisoryStack(sys, ~)
            pp = bctrl.pidParams();
            phis = [0.10, 0.20, 0.05];
            obj.pids = {}; obj.smcs = {};
            for i = 1:3
                obj.pids{i} = bctrl.DPID(pp, sys.Ts);
                sp = struct('c_e',1.0,'c_de',0.2,'K',0.10,'phi',phis(i),'uMin',-0.5,'uMax',0.5);
                obj.smcs{i} = bctrl.SMCax(sp, sys.Ts);
            end
        end
        function du = compute(obj, ref_dy, dy)
            e = ref_dy(:) - dy(:);
            du = zeros(3,1);
            for i = 1:3
                uPID = obj.pids{i}.compute(e(i));
                uSMC = obj.smcs{i}.compute(dy(i) - ref_dy(i));
                if abs(e(i)) > 5.0, du(i) = uSMC; else, du(i) = uPID; end
            end
        end
        function reset(obj)
            for i=1:3, obj.pids{i}.reset(); obj.smcs{i}.reset(); end
        end
        function n = name(~), n = 'SupervisoryStack'; end
    end
end
