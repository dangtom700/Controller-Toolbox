classdef GPCRLS < bctrl.Controller
% GPC-RLS  Per-channel generalised predictive control (SISO condensed QP on the
%   diagonal channel, reference-trajectory filter alpha=0.8) with an online
%   recursive-least-squares ARX(2,2) identifier running in parallel (forgetting
%   0.995).  The identifier is kept guarded/decoupled from the predictor so a
%   transiently unstable identified model can never destabilise the loop.
    properties, mpcs; theta; Prls; rfilt; alpha = 0.8; yh; uh; end
    methods
        function obj = GPCRLS(sys, ~)
            obj.mpcs={}; obj.theta={}; obj.Prls={};
            for i = 1:3
                ch = boiler.diagonalChannel(sys, i);
                obj.mpcs{i}  = bctrl.CondensedMPC(ch, 20, 3, 1.0, 0.5, -0.5, 0.5, -0.02, 0.02);
                obj.theta{i} = zeros(4,1);
                obj.Prls{i}  = 1e3*eye(4);
            end
            obj.rfilt = [0;0;0];
            obj.yh = zeros(3,2);
            obj.uh = zeros(3,2);
        end
        function du = compute(obj, ref_dy, dy)
            du = zeros(3,1);
            obj.rfilt = obj.alpha*obj.rfilt + (1-obj.alpha)*ref_dy(:);
            for i = 1:3
                % --- RLS ARX(2,2) identification (parallel, guarded) ---
                phi = [-obj.yh(i,1); -obj.yh(i,2); obj.uh(i,1); obj.uh(i,2)];
                lam = 0.995;
                Pk = obj.Prls{i}; th = obj.theta{i};
                g  = (Pk*phi) / (lam + phi.'*Pk*phi);
                th = th + g*(dy(i) - phi.'*th);
                Pk = (Pk - g*phi.'*Pk)/lam;
                obj.theta{i} = th; obj.Prls{i} = Pk;
                % --- predictive control on the diagonal channel ---
                du(i) = obj.mpcs{i}.solve(dy(:), obj.rfilt(i));
                obj.yh(i,:) = [dy(i), obj.yh(i,1)];
                obj.uh(i,:) = [du(i), obj.uh(i,1)];
            end
        end
        function reset(obj)
            for i=1:3
                obj.mpcs{i}.reset();
                obj.theta{i}=zeros(4,1); obj.Prls{i}=1e3*eye(4);
            end
            obj.rfilt=[0;0;0]; obj.yh=zeros(3,2); obj.uh=zeros(3,2);
        end
        function n = name(~), n = 'GPC-RLS'; end
    end
end
