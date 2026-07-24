classdef ADRC < bctrl.Controller
% ADRC  Three per-axis linear active-disturbance-rejection controllers
%   (2-state extended state observer + proportional law), deviation space.
    properties, z1=[0;0;0]; z2=[0;0;0]; b0; beta1; beta2; kp; Ts; end
    methods
        function obj = ADRC(sys, op)
            obj.Ts = sys.Ts;
            x1_98 = op.x1 ^ (9.0/8.0);
            wo = [0.45; 0.40; 0.40];
            wc = [0.09; 0.08; 0.08];
            obj.b0    = [0.9; 0.073*x1_98; 141.0/85.0];
            obj.beta1 = 2*wo;
            obj.beta2 = wo.^2;
            obj.kp    = wc;
        end
        function du = compute(obj, ref_dy, dy)
            du = zeros(3,1);
            for i = 1:3
                y  = dy(i); r = ref_dy(i);
                eo = obj.z1(i) - y;
                u0 = obj.kp(i)*(r - obj.z1(i));
                u  = min(max((u0 - obj.z2(i))/obj.b0(i), -0.5), 0.5);
                obj.z1(i) = obj.z1(i) + obj.Ts*(obj.z2(i) + obj.b0(i)*u - obj.beta1(i)*eo);
                obj.z2(i) = obj.z2(i) + obj.Ts*(-obj.beta2(i)*eo);
                du(i) = u;
            end
        end
        function reset(obj), obj.z1=[0;0;0]; obj.z2=[0;0;0]; end
        function n = name(~), n = 'ADRC'; end
    end
end
