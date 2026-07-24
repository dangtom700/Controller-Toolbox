classdef EKFLQR < bctrl.Controller
% EKF-LQR  Extended Kalman filter on the nonlinear boiler (analytic Jacobians
%   via boiler.linearize) feeding an LQR state-feedback law.
    properties, K; Nbar; op; Ts; x; P; Qn; Rn; du_prev; end
    methods
        function obj = EKFLQR(sys, op)
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
            % predict
            xp = ekf_f(obj.x, u_abs, obj.Ts);
            F  = ekf_jacA(obj.x, u_abs, obj.op, obj.Ts);
            Pp = F*obj.P*F.' + obj.Qn;
            % update
            [hh, H] = ekf_h(xp, u_abs, obj.op, obj.Ts);
            S  = H*Pp*H.' + obj.Rn;
            Kk = (Pp*H.') / S;
            obj.x = xp + Kk*(y_abs - hh);
            obj.P = (eye(3) - Kk*H)*Pp;
            % LQR on estimated deviation
            x_dev = obj.x - [obj.op.x1;obj.op.x2;obj.op.x3];
            du = min(max(-obj.K*x_dev + obj.Nbar*ref_dy(:), -0.5), 0.5);
            obj.du_prev = du;
        end
        function reset(obj)
            obj.x = [obj.op.x1;obj.op.x2;obj.op.x3];
            obj.P = eye(3); obj.du_prev = [0;0;0];
        end
        function n = name(~), n = 'EKF-LQR'; end
    end
end

function xn = ekf_f(x, u, Ts)
x1=x(1); x2=x(2); x3=x(3); u1=u(1); u2=u(2); u3=u(3);
x1_98 = max(x1,1)^(9/8);
xn = [x1 + Ts*(-0.0018*u2*x1_98 + 0.9*u1 - 0.15*u3);
      x2 + Ts*((0.073*u2 - 0.016)*x1_98 - 0.1*x2);
      x3 + Ts*((141*u3 - (1.1*u2 - 0.19)*x1)/85)];
end

function F = ekf_jacA(x, u, op, Ts)
oc = op; oc.x1=x(1); oc.x2=x(2); oc.x3=x(3); oc.u1=u(1); oc.u2=u(2); oc.u3=u(3);
s = boiler.linearize(oc, Ts); F = s.A;
end

function [h, H] = ekf_h(x, u, op, Ts)
h = [x(1); x(2); boiler.computeY3(x(1),x(2),x(3),u(1),u(2),u(3))];
oc = op; oc.x1=x(1); oc.x2=x(2); oc.x3=x(3); oc.u1=u(1); oc.u2=u(2); oc.u3=u(3);
s = boiler.linearize(oc, Ts); H = s.C;
end
