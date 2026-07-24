classdef CondensedMPC < handle
% CONDENSEDMPC  Dense/condensed linear MPC solved online with quadprog
%   (Optimization Toolbox).  Decision variables are the control DEVIATIONS
%   u_0..u_{Nu-1} (held constant after Nu); prediction y_i = C*x_i over Np.
%   Cost sum||C x_i - ref||_Qy + sum||u_j||_Ru with box (uMin,uMax) and
%   move-rate (duMin,duMax) constraints relative to the previous applied move.
%
%   Works for the MIMO plant (m=p=3) and for SISO diagonal channels (m=p=1).
%   Robust: on a solver failure it falls back to a clamped unconstrained LQR
%   move, so a batch run never aborts.

    properties
        A; B; C; n; m; p; Np; Nu
        Phi; Theta; Qbar; Rbar; H
        Ddiff; lb; ub; duMaxVec; duMinVec
        Kfb; opts
        prevU; warm
    end
    methods
        function obj = CondensedMPC(model, Np, Nu, qy, ru, uMin, uMax, duMin, duMax)
            obj.A = model.A; obj.B = model.B; obj.C = model.C;
            obj.n = size(obj.A,1); obj.m = size(obj.B,2); obj.p = size(obj.C,1);
            obj.Np = Np; obj.Nu = Nu;

            qyv = local_vec(qy, obj.p);
            ruv = local_vec(ru, obj.m);

            % Prediction matrices with move-blocking (u held after Nu-1).
            Apow = cell(Np+1,1); Apow{1} = eye(obj.n);
            for i = 1:Np, Apow{i+1} = obj.A * Apow{i}; end
            obj.Phi   = zeros(Np*obj.p, obj.n);
            obj.Theta = zeros(Np*obj.p, Nu*obj.m);
            for i = 1:Np
                ri = (i-1)*obj.p + (1:obj.p);
                obj.Phi(ri,:) = obj.C * Apow{i+1};
                for g = 0:i-1
                    mj = min(g, Nu-1);
                    cj = mj*obj.m + (1:obj.m);
                    obj.Theta(ri, cj) = obj.Theta(ri, cj) + obj.C * Apow{i-g} * obj.B;
                end
            end
            obj.Qbar = kron(eye(Np), diag(qyv));
            obj.Rbar = kron(eye(Nu), diag(ruv));
            H0 = 2*(obj.Theta.'*obj.Qbar*obj.Theta + obj.Rbar);
            obj.H = (H0 + H0.')/2;

            % Box + rate constraint scaffolding.
            obj.lb = repmat(uMin, Nu*obj.m, 1);
            obj.ub = repmat(uMax, Nu*obj.m, 1);
            duMaxv = local_vec(duMax, obj.m);
            duMinv = local_vec(duMin, obj.m);
            obj.duMaxVec = repmat(duMaxv, Nu, 1);
            obj.duMinVec = repmat(duMinv, Nu, 1);
            obj.Ddiff = zeros(Nu*obj.m);
            for j = 1:Nu
                rj = (j-1)*obj.m + (1:obj.m);
                obj.Ddiff(rj, rj) = eye(obj.m);
                if j > 1
                    obj.Ddiff(rj, (j-2)*obj.m + (1:obj.m)) = -eye(obj.m);
                end
            end

            % Unconstrained LQR fallback gain.
            try
                Qs = obj.C.'*diag(qyv)*obj.C + 1e-6*eye(obj.n);
                obj.Kfb = dlqr(obj.A, obj.B, Qs, diag(ruv));
            catch
                obj.Kfb = zeros(obj.m, obj.n);
            end

            obj.opts = optimoptions('quadprog', 'Display', 'off');
            obj.prevU = zeros(obj.m, 1);
            obj.warm  = zeros(Nu*obj.m, 1);
        end

        function du0 = solve(obj, x0, ref)
            Rref = repmat(ref(:), obj.Np, 1);
            f = 2 * obj.Theta.' * obj.Qbar * (obj.Phi * x0(:) - Rref);
            s = [obj.prevU; zeros((obj.Nu-1)*obj.m, 1)];
            Aineq = [obj.Ddiff; -obj.Ddiff];
            bineq = [obj.duMaxVec + s; -obj.duMinVec - s];
            U = [];
            try
                [U, ~, flag] = quadprog(obj.H, f, Aineq, bineq, [], [], ...
                                        obj.lb, obj.ub, obj.warm, obj.opts);
                if flag <= 0, U = []; end
            catch
                U = [];
            end
            if isempty(U)
                du0 = min(max(-obj.Kfb * x0(:), obj.lb(1:obj.m)), obj.ub(1:obj.m));
            else
                obj.warm = U;
                du0 = U(1:obj.m);
            end
            du0 = min(max(du0, obj.lb(1:obj.m)), obj.ub(1:obj.m));
            obj.prevU = du0;
        end

        function setPlant(obj, model)
            % Rebuild prediction/fallback for a new (A,B,C) of identical size.
            keepNp = obj.Np; keepNu = obj.Nu;
            qyv = diag(obj.Qbar(1:obj.p, 1:obj.p));
            ruv = diag(obj.Rbar(1:obj.m, 1:obj.m));
            uMin = obj.lb(1); uMax = obj.ub(1);
            duMax = obj.duMaxVec(1:obj.m); duMin = obj.duMinVec(1:obj.m);
            rebuilt = bctrl.CondensedMPC(model, keepNp, keepNu, qyv, ruv, ...
                                         uMin, uMax, duMin, duMax);
            obj.A = rebuilt.A; obj.B = rebuilt.B; obj.C = rebuilt.C;
            obj.Phi = rebuilt.Phi; obj.Theta = rebuilt.Theta; obj.H = rebuilt.H;
            obj.Kfb = rebuilt.Kfb;
        end

        function reset(obj)
            obj.prevU = zeros(obj.m, 1);
            obj.warm  = zeros(obj.Nu*obj.m, 1);
        end
    end
end

function v = local_vec(x, k)
if isscalar(x), v = repmat(x, k, 1); else, v = x(:); end
end
