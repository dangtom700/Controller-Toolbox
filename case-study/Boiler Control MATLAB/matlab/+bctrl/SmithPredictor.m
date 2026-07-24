classdef SmithPredictor < bctrl.Controller
% SmithPredictor  Per-channel Smith dead-time compensation (delay d=2) around a
%   diagonal-channel model, inner discrete PID.
    properties, pids; Am; Bm; Cm; xm; buf; d = 2; end
    methods
        function obj = SmithPredictor(sys, ~)
            p = bctrl.pidParams();
            obj.pids={}; obj.Am={}; obj.Bm={}; obj.Cm={}; obj.xm={}; obj.buf={};
            for i = 1:3
                ch = boiler.diagonalChannel(sys, i);
                obj.Am{i}=ch.A; obj.Bm{i}=ch.B; obj.Cm{i}=ch.C;
                obj.pids{i}=bctrl.DPID(p, sys.Ts);
                obj.xm{i}=zeros(size(ch.A,1),1);
                obj.buf{i}=zeros(1, obj.d);
            end
        end
        function du = compute(obj, ref_dy, dy)
            du = zeros(3,1);
            for i = 1:3
                yhat     = obj.Cm{i}*obj.xm{i};      % delay-free model output
                yhat_del = obj.buf{i}(1);            % delayed by d
                ec = ref_dy(i) - dy(i) - (yhat - yhat_del);
                u  = obj.pids{i}.compute(ec);
                obj.xm{i} = obj.Am{i}*obj.xm{i} + obj.Bm{i}*u;
                obj.buf{i} = [obj.buf{i}(2:end), obj.Cm{i}*obj.xm{i}];
                du(i) = u;
            end
        end
        function reset(obj)
            for i=1:3
                obj.pids{i}.reset();
                obj.xm{i}(:) = 0;
                obj.buf{i} = zeros(1, obj.d);
            end
        end
        function n = name(~), n = 'SmithPredictor'; end
    end
end
