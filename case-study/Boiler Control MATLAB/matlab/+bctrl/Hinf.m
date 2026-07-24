classdef Hinf < bctrl.Controller
% H-inf  Per-channel mixed-sensitivity H-infinity synthesis (mixsyn, Robust
%   Control Toolbox) on the continuous-equivalent diagonal channel, discretised
%   back for online use.  Falls back to a discrete PID on any channel whose
%   synthesis is infeasible -- exactly the C++ behaviour.
    properties, ctrls; fb; useH; end
    methods
        function obj = Hinf(sys, ~)
            p = bctrl.pidParams();
            obj.ctrls = cell(1,3); obj.fb = cell(1,3); obj.useH = false(1,3);
            for i = 1:3
                obj.fb{i} = bctrl.DPID(p, sys.Ts);
                try
                    ch = boiler.diagonalChannel(sys, i);
                    Gd = ss(ch.A, ch.B, ch.C, ch.D, sys.Ts);
                    Gc = d2c(Gd, 'tustin');
                    W1 = makeweight(100, 0.5, 0.1);   % performance / low S at LF
                    W3 = makeweight(0.1, 1.5, 10);    % robustness / low T at HF
                    [Kc, ~, gam] = mixsyn(Gc, W1, [], W3);
                    if isfinite(gam)
                        Kd = c2d(ss(Kc), sys.Ts, 'tustin');
                        if all(isfinite(Kd.A(:))) && all(isfinite(Kd.B(:))) && ...
                           all(isfinite(Kd.C(:))) && all(isfinite(Kd.D(:)))
                            obj.ctrls{i} = struct('A',Kd.A,'B',Kd.B,'C',Kd.C,'D',Kd.D, ...
                                                  'x',zeros(size(Kd.A,1),1));
                            obj.useH(i) = true;
                        end
                    end
                catch
                end
            end
        end
        function du = compute(obj, ref_dy, dy)
            e = ref_dy(:) - dy(:);
            du = zeros(3,1);
            for i = 1:3
                if obj.useH(i)
                    c = obj.ctrls{i};
                    u = c.C*c.x + c.D*e(i);
                    c.x = c.A*c.x + c.B*e(i);
                    obj.ctrls{i} = c;
                    du(i) = min(max(u, -0.5), 0.5);
                else
                    du(i) = obj.fb{i}.compute(e(i));
                end
            end
        end
        function reset(obj)
            for i = 1:3
                obj.fb{i}.reset();
                if obj.useH(i), obj.ctrls{i}.x(:) = 0; end
            end
        end
        function n = name(~), n = 'H-inf'; end
    end
end
