classdef SMCax < handle
% SMCAX  Single-axis boundary-layer sliding-mode control.
%   Input is e = y - r (C++ DiscreteSMC sign convention).  Sliding surface
%   s = c_e*e + c_de*de/Ts; control u = -K*sat(s/phi), clamped.
    properties
        c_e; c_de; K; phi; uMin; uMax; Ts
        ep = 0; last = 0
    end
    methods
        function obj = SMCax(p, Ts)
            obj.c_e = p.c_e; obj.c_de = p.c_de; obj.K = p.K; obj.phi = p.phi;
            obj.uMin = p.uMin; obj.uMax = p.uMax; obj.Ts = Ts;
        end
        function u = compute(obj, e)
            if ~isfinite(e), u = obj.last; return; end
            de = (e - obj.ep) / obj.Ts;
            s  = obj.c_e * e + obj.c_de * de;
            sat = max(min(s / obj.phi, 1), -1);
            u  = min(max(-obj.K * sat, obj.uMin), obj.uMax);
            obj.ep = e; obj.last = u;
        end
        function reset(obj)
            obj.ep = 0; obj.last = 0;
        end
    end
end
