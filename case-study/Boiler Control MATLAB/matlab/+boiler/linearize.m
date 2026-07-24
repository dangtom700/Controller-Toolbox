function sys = linearize(op, Ts)
% LINEARIZE  Analytic Jacobian linearisation of the Bell-Astrom plant about op,
%   discretised via ZOH (Control System Toolbox c2d).  Faithful port of
%   sim/src/linearizer.cpp.  Returns a struct with fields A,B,C,D,Ts giving the
%   discrete deviation model:  dx[k+1]=A dx[k]+B du[k],  dy[k]=C dx[k]+D du[k].
if nargin < 2, Ts = 1.0; end
x1 = op.x1; x3 = op.x3; u2 = op.u2;
x1_18 = x1 ^ (1.0/8.0);
x1_98 = x1 ^ (9.0/8.0);

% Continuous Ac = df/dx
Ac = zeros(3);
Ac(1,1) = -0.0018 * u2 * (9.0/8.0) * x1_18;
Ac(2,1) = (0.073 * u2 - 0.016) * (9.0/8.0) * x1_18;
Ac(2,2) = -0.1;
Ac(3,1) = -(1.1 * u2 - 0.19) / 85.0;

% Continuous Bc = df/du
Bc = zeros(3);
Bc(1,1) = 0.9;  Bc(1,2) = -0.0018 * x1_98;  Bc(1,3) = -0.15;
Bc(2,2) = 0.073 * x1_98;
Bc(3,2) = -1.1 * x1 / 85.0;  Bc(3,3) = 141.0 / 85.0;

% Continuous Cc = dy/dx (analytic Jacobian of y3)
numer  = (1.0 - 0.001538 * x3) * 0.8 * x1 - 25.6;
denom  = x3 * (1.0394 - 0.0012304 * x1);
denom2 = denom * denom;
dn_dx1 = 0.8 * (1.0 - 0.001538 * x3);
dn_dx3 = -0.001538 * 0.8 * x1;
dd_dx1 = -0.0012304 * x3;
dd_dx3 =  1.0394 - 0.0012304 * x1;
dacs_dx1 = (dn_dx1 * denom - numer * dd_dx1) / denom2;
dacs_dx3 = (dn_dx3 * denom - numer * dd_dx3) / denom2;

Cc = zeros(3);
Cc(1,1) = 1.0;
Cc(2,2) = 1.0;
Cc(3,1) = 0.05 * (100.0 * dacs_dx1 + (0.854 * u2 - 0.147) / 9.0);
Cc(3,3) = 0.05 * (0.13073 + 100.0 * dacs_dx3);

% Continuous Dc = dy/du
Dc = zeros(3);
Dc(3,1) = 0.05 * 45.59 / 9.0;
Dc(3,2) = 0.05 * 0.854 * x1 / 9.0;
Dc(3,3) = -0.05 * 2.514 / 9.0;

% ZOH discretisation (Control System Toolbox)
sysd = c2d(ss(Ac, Bc, Cc, Dc), Ts, 'zoh');
sys.A = sysd.A;  sys.B = sysd.B;  sys.C = sysd.C;  sys.D = sysd.D;  sys.Ts = Ts;
end
