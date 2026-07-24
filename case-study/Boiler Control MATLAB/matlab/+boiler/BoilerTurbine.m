classdef BoilerTurbine < handle
% BOILERTURBINE  Bell-Astrom (1987) nonlinear boiler-turbine plant.
%   Faithful port of sim/src/boiler_plant.cpp (forward-Euler, Ts=1 s).
%
%   States  x = [x1 drum pressure; x2 electric power; x3 fluid density]
%   Inputs  u = [u1 fuel valve; u2 steam valve; u3 feedwater valve] in [0,1]
%   Outputs y = [y1=x1; y2=x2; y3 drum water-level deviation]

    properties
        Ts  = 1.0
        u1  = 0.5;  u2 = 0.5;  u3 = 0.5
        x1  = 100;  x2 = 50;   x3 = 20
        y1  = 0;    y2 = 0;    y3 = 0
        du1 = 0;    du2 = 0;   du3 = 0
    end
    properties (Access = private)
        u1p = 0.5;  u2p = 0.5;  u3p = 0.5
    end

    methods
        function obj = BoilerTurbine()
            obj.recompute();
        end

        function initAt(obj, op)
            obj.x1 = op.x1; obj.x2 = op.x2; obj.x3 = op.x3;
            obj.u1 = op.u1; obj.u2 = op.u2; obj.u3 = op.u3;
            obj.u1p = op.u1; obj.u2p = op.u2; obj.u3p = op.u3;
            obj.du1 = 0; obj.du2 = 0; obj.du3 = 0;
            obj.recompute();
        end

        function setState(obj, x1_, x2_, x3_)
            obj.x1 = x1_; obj.x2 = x2_; obj.x3 = x3_;
            obj.recompute();
        end

        function setControls(obj, u1_, u2_, u3_)
            obj.u1 = u1_; obj.u2 = u2_; obj.u3 = u3_;
        end

        function constrainValve(obj)
            obj.u1 = min(max(obj.u1, 0), 1);
            obj.u2 = min(max(obj.u2, 0), 1);
            obj.u3 = min(max(obj.u3, 0), 1);
        end

        function constrainValveRate(obj)
            obj.du1 = min(max(obj.u1 - obj.u1p, -0.007), 0.007);
            obj.du2 = min(max(obj.u2 - obj.u2p, -0.020), 0.020);
            obj.du3 = min(max(obj.u3 - obj.u3p, -0.050), 0.050);
            obj.u1 = obj.u1p + obj.du1;
            obj.u2 = obj.u2p + obj.du2;
            obj.u3 = obj.u3p + obj.du3;
            obj.u1p = obj.u1; obj.u2p = obj.u2; obj.u3p = obj.u3;
        end

        function update(obj)
            x1_98 = obj.x1 ^ (9.0 / 8.0);
            dx1 = -0.0018 * obj.u2 * x1_98 + 0.9 * obj.u1 - 0.15 * obj.u3;
            dx2 = (0.073 * obj.u2 - 0.016) * x1_98 - 0.1 * obj.x2;
            dx3 = (141.0 * obj.u3 - (1.1 * obj.u2 - 0.19) * obj.x1) / 85.0;
            obj.x1 = obj.x1 + obj.Ts * dx1;
            obj.x2 = obj.x2 + obj.Ts * dx2;
            obj.x3 = obj.x3 + obj.Ts * dx3;
            obj.recompute();
        end

        function y = measureOutputs(obj), y = [obj.y1; obj.y2; obj.y3]; end
        function u = controls(obj),       u = [obj.u1; obj.u2; obj.u3]; end
        function x = state(obj),          x = [obj.x1; obj.x2; obj.x3]; end
    end

    methods (Access = private)
        function recompute(obj)
            obj.y1 = obj.x1;
            obj.y2 = obj.x2;
            obj.y3 = boiler.computeY3(obj.x1, obj.x2, obj.x3, obj.u1, obj.u2, obj.u3);
        end
    end
end
