classdef MHELQR < bctrl.Controller
% MHE-LQR  Finite-memory (moving-horizon) least-squares state estimator over the
%   last N=8 (dy,du) samples, feeding an LQR state-feedback law.  Before the
%   window fills it falls back to LQR on the raw measurement.
    properties, K; Nbar; A; B; C; N = 8; ubuf; ybuf; cnt = 0; du_prev; end
    methods
        function obj = MHELQR(sys, ~)
            [Q,R] = bctrl.brysonWeights([5;10;1],[0.3;0.3;0.1]);
            obj.K = dlqr(sys.A, sys.B, Q, R);
            obj.Nbar = boiler.computeNbar(sys);
            obj.A = sys.A; obj.B = sys.B; obj.C = sys.C;
            obj.ubuf = zeros(3, obj.N); obj.ybuf = zeros(3, obj.N);
            obj.du_prev = [0;0;0];
        end
        function du = compute(obj, ref_dy, dy)
            obj.ubuf = [obj.ubuf(:,2:end), obj.du_prev];
            obj.ybuf = [obj.ybuf(:,2:end), dy(:)];
            obj.cnt = obj.cnt + 1;
            if obj.cnt >= obj.N
                xk = obj.estimateState();
            else
                xk = dy(:);
            end
            du = min(max(-obj.K*xk + obj.Nbar*ref_dy(:), -0.5), 0.5);
            obj.du_prev = du;
        end
        function xk = estimateState(obj)
            % Batch LS for x0 at window start, then propagate to current time.
            n = size(obj.A,1); p = size(obj.C,1); Nn = obj.N;
            Obs = zeros(Nn*p, n);
            rhs = zeros(Nn*p, 1);
            Aj = eye(n);                     % A^j
            forced = zeros(n,1);             % forced state at window index j
            for j = 0:Nn-1
                r = j*p + (1:p);
                Obs(r,:) = obj.C*Aj;
                rhs(r)   = obj.ybuf(:,j+1) - obj.C*forced;
                forced   = obj.A*forced + obj.B*obj.ubuf(:,j+1);
                Aj = obj.A*Aj;
            end
            x0 = Obs \ rhs;
            % propagate x0 -> current time (index N-1)
            xk = x0;
            for j = 0:Nn-2
                xk = obj.A*xk + obj.B*obj.ubuf(:,j+1);
            end
        end
        function reset(obj)
            obj.ubuf = zeros(3, obj.N); obj.ybuf = zeros(3, obj.N);
            obj.cnt = 0; obj.du_prev = [0;0;0];
        end
        function n = name(~), n = 'MHE-LQR'; end
    end
end
