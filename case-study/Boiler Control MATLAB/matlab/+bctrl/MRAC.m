classdef MRAC < bctrl.Controller
% MRAC  Per-channel model-reference adaptive control (normalised MIT rule).
%   Reference model y_m[k+1]=0.8 y_m[k]+0.2 r[k] (DC gain 1); adaptive gains
%   thr, thy give du = thr*ref_dy - thy*dy.  Implemented in DEVIATION space
%   (well-scaled) rather than the C++ absolute-valve form; see study README.
    properties, op; am = 0.8; bm = 0.2; gamma = 2.0; sigma = 0.01; thmax = 50;
              ym; thr; thy; end
    methods
        function obj = MRAC(sys, op) %#ok<INUSD>
            obj.op = op;
            obj.ym = [0;0;0]; obj.thr = [0;0;0]; obj.thy = [0;0;0];
        end
        function du = compute(obj, ref_dy, dy)
            du = zeros(3,1);
            for i = 1:3
                r = ref_dy(i); y = dy(i);
                u = obj.thr(i)*r - obj.thy(i)*y;
                u = min(max(u, -0.5), 0.5);
                etr = y - obj.ym(i);
                nrm = 1 + r^2 + y^2;
                obj.thr(i) = obj.thr(i) - obj.gamma*etr*r/nrm - obj.sigma*obj.thr(i);
                obj.thy(i) = obj.thy(i) + obj.gamma*etr*y/nrm - obj.sigma*obj.thy(i);
                obj.thr(i) = min(max(obj.thr(i), -obj.thmax), obj.thmax);
                obj.thy(i) = min(max(obj.thy(i), -obj.thmax), obj.thmax);
                obj.ym(i)  = obj.am*obj.ym(i) + obj.bm*r;
                du(i) = u;
            end
        end
        function reset(obj)
            obj.ym = [0;0;0]; obj.thr = [0;0;0]; obj.thy = [0;0;0];
        end
        function n = name(~), n = 'MRAC'; end
    end
end
