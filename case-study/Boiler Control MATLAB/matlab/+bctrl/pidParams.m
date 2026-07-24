function p = pidParams()
% PIDPARAMS  Standard per-channel discrete-PID tuning (matches controllers.cpp).
p.Kp = 0.05;  p.Ki = 0.002;  p.Kd = 0.02;  p.N = 5.0;
p.uMin = -0.5;  p.uMax = 0.5;
end
