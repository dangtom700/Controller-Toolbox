classdef AdaptiveSP < bctrl.Controller
% AdaptiveSP  Per-channel Smith predictor whose dead-time estimate adapts online
%   via input/output cross-correlation (search range 0..8 steps, refreshed every
%   100 steps).
    properties, pids; Am; Bm; Cm; xm; buf; d; maxd = 8; ubuf; ybuf; bl = 200; cnt = 0; end
    methods
        function obj = AdaptiveSP(sys, ~)
            p = bctrl.pidParams();
            obj.pids={}; obj.Am={}; obj.Bm={}; obj.Cm={}; obj.xm={}; obj.buf={};
            obj.d = [2 2 2];
            obj.ubuf = zeros(3, obj.bl); obj.ybuf = zeros(3, obj.bl);
            for i = 1:3
                ch = boiler.diagonalChannel(sys, i);
                obj.Am{i}=ch.A; obj.Bm{i}=ch.B; obj.Cm{i}=ch.C;
                obj.pids{i}=bctrl.DPID(p, sys.Ts);
                obj.xm{i}=zeros(size(ch.A,1),1);
                obj.buf{i}=zeros(1, obj.maxd+1);
            end
        end
        function du = compute(obj, ref_dy, dy)
            du = zeros(3,1);
            for i = 1:3
                yhat     = obj.Cm{i}*obj.xm{i};
                yhat_del = obj.buf{i}(end - obj.d(i));      % delayed by d(i)
                ec = ref_dy(i) - dy(i) - (yhat - yhat_del);
                u  = obj.pids{i}.compute(ec);
                obj.xm{i} = obj.Am{i}*obj.xm{i} + obj.Bm{i}*u;
                obj.buf{i} = [obj.buf{i}(2:end), obj.Cm{i}*obj.xm{i}];
                obj.ubuf(i,:) = [obj.ubuf(i,2:end), u];
                obj.ybuf(i,:) = [obj.ybuf(i,2:end), dy(i)];
                du(i) = u;
            end
            obj.cnt = obj.cnt + 1;
            if mod(obj.cnt, 100) == 0
                for i = 1:3, obj.d(i) = obj.estimateDelay(i); end
            end
        end
        function dEst = estimateDelay(obj, i)
            u = obj.ubuf(i,:); y = obj.ybuf(i,:);
            u = u - mean(u); y = y - mean(y);
            best = 2; bestc = -inf;
            for lag = 0:obj.maxd
                a = u(1:end-lag); b = y(1+lag:end);
                c = abs(sum(a.*b));
                if c > bestc, bestc = c; best = lag; end
            end
            dEst = best;
        end
        function reset(obj)
            for i=1:3
                obj.pids{i}.reset(); obj.xm{i}(:) = 0;
                obj.buf{i} = zeros(1, obj.maxd+1);
            end
            obj.d = [2 2 2]; obj.cnt = 0;
            obj.ubuf = zeros(3, obj.bl); obj.ybuf = zeros(3, obj.bl);
        end
        function n = name(~), n = 'AdaptiveSP'; end
    end
end
