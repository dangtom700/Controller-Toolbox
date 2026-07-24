classdef MPC < bctrl.Controller
% MPC  MIMO condensed-QP MPC (quadprog) on the linearised model.
%   Np=20, Nu=5, output weight 1, move weight 0.1, valve box +/-0.5,
%   rate limit +/-0.02 (steam-valve limit).  dy is the state-deviation proxy.
    properties, mpc; end
    methods
        function obj = MPC(sys, ~)
            obj.mpc = bctrl.CondensedMPC(sys, 20, 5, 1.0, 0.1, -0.5, 0.5, -0.02, 0.02);
        end
        function du = compute(obj, ref_dy, dy)
            du = obj.mpc.solve(dy(:), ref_dy(:));
        end
        function reset(obj), obj.mpc.reset(); end
        function n = name(~), n = 'MPC'; end
    end
end
