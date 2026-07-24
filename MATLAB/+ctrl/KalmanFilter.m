classdef KalmanFilter < handle
%KALMANFILTER  Discrete-time linear Kalman filter.  Port of ctrl::KalmanFilter.
%
%       x[k+1] = A x[k] + B u[k] + w,   w ~ N(0, Q)
%       y[k]   = C x[k] + D u[k] + v,   v ~ N(0, R)
%
%   Construction:
%       kf = ctrl.KalmanFilter(plant, Q_noise, R_noise)
%       kf = ctrl.KalmanFilter(plant, Q_noise, R_noise, P0)   % P0 defaults to I
%   where plant is a ctrl.StateSpace.
%
%   Predict/update use the Joseph form for the covariance (numerically PSD-preserving),
%   and skip the update if the innovation covariance is not positive definite.
%   Equivalent MATLAB (Control System Toolbox): kalman()/kalmd().

  properties
    A; B; C; D
    Q; R
    xhat   % state estimate x^[k|k]
    P      % error covariance P[k|k]
    Ts
    n; p
  end

  methods
    function obj = KalmanFilter(plant, Q_noise, R_noise, P0)
      if nargin < 4, P0 = []; end
      obj.A = plant.A; obj.B = plant.B; obj.C = plant.C; obj.D = plant.D;
      obj.Q = Q_noise; obj.R = R_noise; obj.Ts = plant.Ts;
      obj.n = size(plant.A, 1);
      obj.p = size(plant.C, 1);
      obj.xhat = zeros(obj.n, 1);
      if ~isempty(P0) && size(P0, 1) == obj.n
        obj.P = P0;
      else
        obj.P = eye(obj.n);
      end
    end

    function predict(obj, u)
      %PREDICT  Advance the estimate with the previous control input u[k-1].
      obj.xhat = obj.A * obj.xhat + obj.B * u(:);
      obj.P    = obj.A * obj.P * obj.A.' + obj.Q;
    end

    function update(obj, y, u_current)
      %UPDATE  Incorporate measurement y[k] (u_current used for the D u feedthrough).
      Rsafe = obj.R;
      for i = 1:size(Rsafe, 1)           % minimum noise floor on the diagonal
        Rsafe(i, i) = max(Rsafe(i, i), 1e-12);
      end

      S = obj.C * obj.P * obj.C.' + Rsafe;

      [~, pflag] = chol(S);              % skip update if S is not positive definite
      if pflag ~= 0
        return;
      end

      Kf    = (obj.P * obj.C.') / S;
      innov = y(:) - obj.C * obj.xhat - obj.D * u_current(:);
      obj.xhat = obj.xhat + Kf * innov;

      % Joseph form: P = (I-KC) P (I-KC)' + K R K'
      IKC   = eye(obj.n) - Kf * obj.C;
      obj.P = IKC * obj.P * IKC.' + Kf * Rsafe * Kf.';
    end

    function step(obj, y, u_prev, u_current)
      %STEP  predict(u_prev) then update(y, u_current).  u_current defaults to u_prev.
      if nargin < 4, u_current = u_prev; end
      obj.predict(u_prev);
      obj.update(y, u_current);
    end

    function reset(obj)
      obj.xhat = zeros(obj.n, 1);
      obj.P    = eye(obj.n);
    end

    function x = state(obj),      x = obj.xhat; end
    function P = covariance(obj), P = obj.P;    end
    function t = sampleTime(obj), t = obj.Ts;   end
  end
end
