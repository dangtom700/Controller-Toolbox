classdef LeadLag < handle
% LEADLAG  First-order lead-lag C(s)=g*(s+z)/(s+p), bilinear (Tustin) discretised.
    properties
        b0; b1; a0; a1
        up = 0; yp = 0; last = 0
    end
    methods
        function obj = LeadLag(z, p, g, Ts)
            K = 2.0 / Ts;
            obj.b0 = g * (K + z);
            obj.b1 = g * (z - K);
            obj.a0 = (K + p);
            obj.a1 = (p - K);
        end
        function y = compute(obj, u)
            if ~isfinite(u), y = obj.last; return; end
            y = (obj.b0 * u + obj.b1 * obj.up - obj.a1 * obj.yp) / obj.a0;
            obj.up = u; obj.yp = y; obj.last = y;
        end
        function reset(obj)
            obj.up = 0; obj.yp = 0; obj.last = 0;
        end
    end
end
