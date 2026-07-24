classdef AutoGS < bctrl.Controller
% AutoGS  Automated gain-scheduled LQR on the drum-pressure channel: design an
%   LQR at six pressure-deviation grid points (op.x1-30 .. op.x1+40) and switch
%   on the current pressure deviation; power/level channels use proportional aid.
    properties, Ks; grid; op; end
    methods
        function obj = AutoGS(sys, op)
            [Q,R] = bctrl.brysonWeights([5;10;1],[0.3;0.3;0.1]);
            pg = op.x1 + linspace(-30, 40, 6);
            obj.Ks = {}; obj.grid = [];
            for k = 1:numel(pg)
                opk = op; opk.x1 = pg(k);
                try
                    ssk = boiler.linearize(opk, sys.Ts);
                    obj.Ks{end+1} = dlqr(ssk.A, ssk.B, Q, R);
                    obj.grid(end+1) = pg(k) - op.x1;      % schedule on deviation
                catch
                end
            end
            if isempty(obj.grid)
                obj.Ks{1} = dlqr(sys.A, sys.B, Q, R); obj.grid = 0;
            end
            obj.op = op;
        end
        function du = compute(obj, ref_dy, dy)
            [~, idx] = min(abs(obj.grid - dy(1)));
            duFull = -obj.Ks{idx}*dy(:);
            du = [duFull(1);
                  0.05*(ref_dy(2) - dy(2));
                  0.05*(ref_dy(3) - dy(3))];
            du = min(max(du, -0.5), 0.5);
        end
        function reset(~), end
        function n = name(~), n = 'AutoGS'; end
    end
end
