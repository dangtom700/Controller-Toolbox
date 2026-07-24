classdef DPID < handle
% DPID  Discrete PID building block: filtered derivative (coefficient N),
%   conditional-integration anti-windup, output clamp.  Holds last output on a
%   non-finite input (NaN contract).  Used standalone and inside composite
%   controllers.
    properties
        Kp; Ki; Kd; N; uMin; uMax; Ts
        alpha           % derivative low-pass coefficient
        I = 0; dstate = 0; ep = 0; last = 0
    end
    methods
        function obj = DPID(p, Ts)
            obj.Kp = p.Kp; obj.Ki = p.Ki; obj.Kd = p.Kd; obj.N = p.N;
            obj.uMin = p.uMin; obj.uMax = p.uMax; obj.Ts = Ts;
            if obj.N > 0
                obj.alpha = (obj.N * Ts) / (1 + obj.N * Ts);
            else
                obj.alpha = 1.0;
            end
        end
        function u = compute(obj, e)
            if ~isfinite(e), u = obj.last; return; end
            deriv = obj.Kd * (e - obj.ep) / obj.Ts;
            obj.dstate = (1 - obj.alpha) * obj.dstate + obj.alpha * deriv;
            u_pre = obj.Kp * e + obj.I + obj.dstate;
            u = min(max(u_pre, obj.uMin), obj.uMax);
            % Conditional integration: pause when clamped and pushing further out.
            saturated = (u ~= u_pre);
            if ~(saturated && (sign(e) == sign(u_pre)))
                obj.I = obj.I + obj.Ki * obj.Ts * e;
            end
            obj.ep = e; obj.last = u;
        end
        function reset(obj)
            obj.I = 0; obj.dstate = 0; obj.ep = 0; obj.last = 0;
        end
    end
end
