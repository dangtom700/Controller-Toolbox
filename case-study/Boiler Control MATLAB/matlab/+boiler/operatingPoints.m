function ops = operatingPoints()
% OPERATINGPOINTS  The three Bell-Astrom (1987) equilibria A/B/C.
%   Returns a struct with fields .A .B .C, each a struct with label, states
%   (x1..x3), controls (u1..u3) and outputs (y1=x1, y2=x2, y3=computeY3).
ops.A = local_mk('Low Load',    75.6,  15.3,  508.97, 0.11926, 0.38063, 0.12262);
ops.B = local_mk('Medium Load', 97.2,  50.5,  469.51, 0.27049, 0.62082, 0.33979);
ops.C = local_mk('High Load',  140.0, 128.0,  323.68, 0.59589, 0.89447, 0.78829);
end

function op = local_mk(label, x1, x2, x3, u1, u2, u3)
op.label = label;
op.x1 = x1; op.x2 = x2; op.x3 = x3;
op.u1 = u1; op.u2 = u2; op.u3 = u3;
op.y1 = x1; op.y2 = x2;
op.y3 = boiler.computeY3(x1, x2, x3, u1, u2, u3);
end
