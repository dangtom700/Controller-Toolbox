classdef SubspaceIDLQG < bctrl.Controller
% SubspaceID-LQG  Identify a 3rd-order model from simulated multisine I/O with
%   n4sid (System Identification Toolbox), then design LQG (LQR + Kalman) on the
%   identified model.  Falls back to the analytic linearisation if ID fails.
    properties, K; Nbar; kf; du_prev; end
    methods
        function obj = SubspaceIDLQG(sys, ~)
            Nid = 300; U = zeros(Nid,3); Y = zeros(Nid,3);
            x = zeros(3,1);
            for k = 0:Nid-1
                t = k*sys.Ts;
                u = [0.05*sin(0.10*t); 0.05*sin(0.13*t+0.5); 0.02*sin(0.07*t+1.0)];
                U(k+1,:) = u.';
                Y(k+1,:) = (sys.C*x + sys.D*u).';
                x = sys.A*x + sys.B*u;
            end
            Ai = sys.A; Bi = sys.B; Ci = sys.C;   % default: analytic model
            try
                m  = n4sid(iddata(Y, U, sys.Ts), 3);
                Ai = m.A; Bi = m.B; Ci = m.C;
            catch
            end
            R  = diag(1 ./ ([0.3;0.3;0.1].^2));
            Qs = Ci.'*Ci + 1e-3*eye(size(Ai,1));
            obj.K    = dlqr(Ai, Bi, Qs, R);
            obj.Nbar = boiler.computeNbar(struct('A',Ai,'B',Bi,'C',Ci,'Ts',sys.Ts));
            obj.kf   = bctrl.KF(Ai, Bi, Ci, 1e-3*eye(size(Ai,1)), diag([0.25 1.0 25.0]));
            obj.du_prev = [0;0;0];
        end
        function du = compute(obj, ref_dy, dy)
            xhat = obj.kf.step(dy(:), obj.du_prev);
            du   = min(max(-obj.K*xhat + obj.Nbar*ref_dy(:), -0.5), 0.5);
            obj.du_prev = du;
        end
        function reset(obj), obj.kf.reset(); obj.du_prev = [0;0;0]; end
        function n = name(~), n = 'SubspaceID-LQG'; end
    end
end
