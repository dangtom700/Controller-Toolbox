classdef SMC < bctrl.Controller
% SMC  Three per-axis boundary-layer sliding-mode controllers (e = y - r).
    properties, smcs; end
    methods
        function obj = SMC(sys, ~)
            phis = [0.10, 0.20, 0.05];
            obj.smcs = cell(1,3);
            for i = 1:3
                p = struct('c_e',1.0,'c_de',0.2,'K',0.10,'phi',phis(i),'uMin',-0.5,'uMax',0.5);
                obj.smcs{i} = bctrl.SMCax(p, sys.Ts);
            end
        end
        function du = compute(obj, ref_dy, dy)
            e = dy(:) - ref_dy(:);   % SMC sign convention
            du = [obj.smcs{1}.compute(e(1)); obj.smcs{2}.compute(e(2)); obj.smcs{3}.compute(e(3))];
        end
        function reset(obj), for i=1:3, obj.smcs{i}.reset(); end, end
        function n = name(~), n = 'SMC'; end
    end
end
