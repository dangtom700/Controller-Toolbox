classdef ESC < bctrl.Controller
% ESC  Extremum-seeking on the feedwater valve u3 to maximise drum level y3.
%   Perturb / washout / demodulate / integrate.  u1,u2 stay at the operating
%   point (du=0).
    properties
        a = 0.005; w = 0.02; kesc = 0.5; lpf = 0.005; y3op
        ylp = 0; grad = 0; theta = 0; t = 0
    end
    methods
        function obj = ESC(~, op), obj.y3op = op.y3; end
        function du = compute(obj, ~, dy)
            y     = obj.y3op + dy(3);
            probe = obj.a * sin(obj.w * obj.t);
            obj.ylp  = obj.ylp + obj.lpf*(y - obj.ylp);       % washout low-pass
            yhp   = y - obj.ylp;
            demod = yhp * sin(obj.w * obj.t);
            obj.grad  = obj.grad + obj.lpf*(demod - obj.grad); % demod low-pass
            obj.theta = obj.theta + obj.kesc*obj.grad;         % gradient ascent
            u3 = min(max(obj.theta + probe, -0.5), 0.5);
            obj.t = obj.t + 1;
            du = [0; 0; u3];
        end
        function reset(obj), obj.ylp=0; obj.grad=0; obj.theta=0; obj.t=0; end
        function n = name(~), n = 'ESC'; end
    end
end
