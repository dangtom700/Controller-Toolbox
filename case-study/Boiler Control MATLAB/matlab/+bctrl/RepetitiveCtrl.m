classdef RepetitiveCtrl < bctrl.Controller
% RepetitiveCtrl  Per-axis PID plus a repetitive internal model (period 200
%   steps matching the s08 disturbance, Q=0.98, Krc=0.5) that learns and cancels
%   periodic setpoint/load waveforms after one period.
    properties, pids; mem; P = 200; Q = 0.98; Krc = 0.5; k = 0; end
    methods
        function obj = RepetitiveCtrl(sys, ~)
            pp = bctrl.pidParams();
            obj.pids = {}; obj.mem = {};
            for i = 1:3
                obj.pids{i} = bctrl.DPID(pp, sys.Ts);
                obj.mem{i}  = zeros(1, obj.P);
            end
        end
        function du = compute(obj, ref_dy, dy)
            e = ref_dy(:) - dy(:);
            pos = mod(obj.k, obj.P) + 1;
            du = zeros(3,1);
            for i = 1:3
                uPID = obj.pids{i}.compute(e(i));
                rc   = obj.Q*obj.mem{i}(pos) + obj.Krc*e(i);
                obj.mem{i}(pos) = rc;
                du(i) = min(max(uPID + rc, -0.5), 0.5);
            end
            obj.k = obj.k + 1;
        end
        function reset(obj)
            for i=1:3, obj.pids{i}.reset(); obj.mem{i} = zeros(1, obj.P); end
            obj.k = 0;
        end
        function n = name(~), n = 'RepetitiveCtrl'; end
    end
end
