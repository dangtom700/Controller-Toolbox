classdef FuzzyPID < bctrl.Controller
% FuzzyPID  Per-axis fuzzy self-tuning PID.  A triangular membership on the
%   normalised error magnitude (small/large) scales the effective proportional
%   action, boosting gain on large excursions and easing it near the setpoint.
%   (Toolbox-free fuzzy inference; a mamfis/evalfis object is a drop-in swap.)
    properties, pids; escale; end
    methods
        function obj = FuzzyPID(sys, ~)
            p = bctrl.pidParams();
            obj.pids   = {bctrl.DPID(p,sys.Ts), bctrl.DPID(p,sys.Ts), bctrl.DPID(p,sys.Ts)};
            obj.escale = [10.0, 20.0, 0.05];
        end
        function du = compute(obj, ref_dy, dy)
            e = ref_dy(:) - dy(:);
            du = zeros(3,1);
            for i = 1:3
                big   = min(abs(e(i))/obj.escale(i), 1);   % "error is large" membership
                scale = 1 + 0.8*big;                       % fuzzy gain boost
                du(i) = obj.pids{i}.compute(e(i)*scale);
            end
        end
        function reset(obj), for i=1:3, obj.pids{i}.reset(); end, end
        function n = name(~), n = 'FuzzyPID'; end
    end
end
