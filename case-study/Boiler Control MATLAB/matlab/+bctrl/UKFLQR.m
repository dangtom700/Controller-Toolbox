classdef UKFLQR < bctrl.Controller
% UKF-LQR  Unscented Kalman filter (alpha=1, beta=2, kappa=0) on the nonlinear
%   boiler feeding an LQR state-feedback law.
    properties, K; Nbar; op; Ts; x; P; Qn; Rn; du_prev; end
    methods
        function obj = UKFLQR(sys, op)
            [Q,R] = bctrl.brysonWeights([5;10;1],[0.3;0.3;0.1]);
            obj.K = dlqr(sys.A, sys.B, Q, R);
            obj.Nbar = boiler.computeNbar(sys);
            obj.op = op; obj.Ts = sys.Ts;
            obj.Qn = 1e-4*eye(3); obj.Rn = diag([0.25 1.0 25.0]);
            obj.x = [op.x1;op.x2;op.x3]; obj.P = eye(3); obj.du_prev = [0;0;0];
        end
        function du = compute(obj, ref_dy, dy)
            y_abs = [obj.op.y1+dy(1); obj.op.y2+dy(2); obj.op.y3+dy(3)];
            u_abs = [obj.op.u1;obj.op.u2;obj.op.u3] + obj.du_prev;
            L = 3; lam = 0; c = L + lam;          % alpha=1, kappa=0 => lambda=0
            Wm = [lam/c; repmat(1/(2*c), 2*L, 1)];
            Wc = Wm; Wc(1) = Wc(1) + (1 - 1 + 2); % + (1-alpha^2+beta)
            % sigma points
            Sm = ukf_chol(c*obj.P);
            X  = [obj.x, obj.x + Sm, obj.x - Sm];
            np = 2*L + 1;
            % predict
            Xp = zeros(3, np);
            for j=1:np, Xp(:,j) = ukf_f(X(:,j), u_abs, obj.Ts); end
            xp = Xp*Wm;
            Pp = obj.Qn;
            for j=1:np, d = Xp(:,j)-xp; Pp = Pp + Wc(j)*(d*d.'); end
            % measurement
            Yp = zeros(3, np);
            for j=1:np, Yp(:,j) = ukf_hy(Xp(:,j), u_abs); end
            yp  = Yp*Wm;
            Pyy = obj.Rn; Pxy = zeros(3);
            for j=1:np
                dyj = Yp(:,j)-yp; dxj = Xp(:,j)-xp;
                Pyy = Pyy + Wc(j)*(dyj*dyj.');
                Pxy = Pxy + Wc(j)*(dxj*dyj.');
            end
            Kk = Pxy / Pyy;
            obj.x = xp + Kk*(y_abs - yp);
            obj.P = Pp - Kk*Pyy*Kk.';
            x_dev = obj.x - [obj.op.x1;obj.op.x2;obj.op.x3];
            du = min(max(-obj.K*x_dev + obj.Nbar*ref_dy(:), -0.5), 0.5);
            obj.du_prev = du;
        end
        function reset(obj)
            obj.x = [obj.op.x1;obj.op.x2;obj.op.x3];
            obj.P = eye(3); obj.du_prev = [0;0;0];
        end
        function n = name(~), n = 'UKF-LQR'; end
    end
end

function S = ukf_chol(M)
M = (M + M.')/2;
[R, pd] = chol(M);
if pd > 0
    R = chol(M + (abs(min(real(eig(M)))) + 1e-9)*eye(size(M)));
end
S = R.';   % lower factor, columns are sigma offsets
end

function xn = ukf_f(x, u, Ts)
x1=x(1); x2=x(2); x3=x(3); u1=u(1); u2=u(2); u3=u(3);
x1_98 = max(x1,1)^(9/8);
xn = [x1 + Ts*(-0.0018*u2*x1_98 + 0.9*u1 - 0.15*u3);
      x2 + Ts*((0.073*u2 - 0.016)*x1_98 - 0.1*x2);
      x3 + Ts*((141*u3 - (1.1*u2 - 0.19)*x1)/85)];
end

function y = ukf_hy(x, u)
y = [x(1); x(2); boiler.computeY3(x(1),x(2),x(3),u(1),u(2),u(3))];
end
