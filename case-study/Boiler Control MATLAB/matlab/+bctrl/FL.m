classdef FL < bctrl.Controller
% FL  Per-channel feedback linearisation.  Cancels the channel drift f_i and
%   inverts the input gain g_i so an inner PID sees a virtual integrator:
%   du_i = (v_i - f_i)/g_i,  v_i = PID(ref_dy_i - dy_i).
    properties, op; pids; end
    methods
        function obj = FL(sys, op)
            pp = struct('Kp',0.5,'Ki',0.005,'Kd',0.0,'N',5.0,'uMin',-2.0,'uMax',2.0);
            obj.pids = {bctrl.DPID(pp,sys.Ts), bctrl.DPID(pp,sys.Ts), bctrl.DPID(pp,sys.Ts)};
            obj.op = op;
        end
        function du = compute(obj, ref_dy, dy)
            x1 = obj.op.x1 + dy(1);
            x2 = obj.op.x2 + dy(2);
            x1_98 = max(x1,1)^(9/8);
            u2o = obj.op.u2;
            f1 = -0.0018*u2o*x1_98 + 0.9*obj.op.u1 - 0.15*obj.op.u3;  g1 = 0.9;
            f2 = (0.073*u2o - 0.016)*x1_98 - 0.1*x2;                  g2 = 0.073*x1_98;
            f3 = (141*obj.op.u3 - (1.1*u2o - 0.19)*x1)/85;            g3 = 141/85;
            v1 = obj.pids{1}.compute(ref_dy(1) - dy(1));
            v2 = obj.pids{2}.compute(ref_dy(2) - dy(2));
            v3 = obj.pids{3}.compute(ref_dy(3) - dy(3));
            du = [(v1 - f1)/g1; (v2 - f2)/g2; (v3 - f3)/g3];
            du = min(max(du, -0.5), 0.5);
        end
        function reset(obj), for i=1:3, obj.pids{i}.reset(); end, end
        function n = name(~), n = 'FL'; end
    end
end
