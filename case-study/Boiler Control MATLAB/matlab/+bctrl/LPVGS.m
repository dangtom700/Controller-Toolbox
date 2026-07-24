classdef LPVGS < bctrl.Controller
% LPV-GS  Linear-parameter-varying gain scheduling: design an LQR at each of six
%   drum-pressure grid points (60..150 bar) and switch to the nearest-neighbour
%   gain on the current pressure.
    properties, Ks; Ns; grid; op; end
    methods
        function obj = LPVGS(sys, op)
            [Q,R] = bctrl.brysonWeights([5;10;1],[0.3;0.3;0.1]);
            pg = linspace(60, 150, 6);
            obj.Ks = {}; obj.Ns = {}; obj.grid = [];
            for k = 1:numel(pg)
                opk = op; opk.x1 = pg(k);
                try
                    ssk = boiler.linearize(opk, sys.Ts);
                    obj.Ks{end+1} = dlqr(ssk.A, ssk.B, Q, R);
                    obj.Ns{end+1} = boiler.computeNbar(ssk);
                    obj.grid(end+1) = pg(k);
                catch
                end
            end
            if isempty(obj.grid)
                obj.Ks{1} = dlqr(sys.A, sys.B, Q, R);
                obj.Ns{1} = boiler.computeNbar(sys);
                obj.grid = op.x1;
            end
            obj.op = op;
        end
        function du = compute(obj, ref_dy, dy)
            x1cur = obj.op.x1 + dy(1);
            [~, idx] = min(abs(obj.grid - x1cur));
            du = -obj.Ks{idx}*dy(:) + obj.Ns{idx}*ref_dy(:);
            du = min(max(du, -0.5), 0.5);
        end
        function reset(~), end
        function n = name(~), n = 'LPV-GS'; end
    end
end
