classdef DiscretePID < handle
%DISCRETEPID  Discrete-time PID with filtered derivative and back-calculation anti-windup.
%   Faithful port of ctrl::DiscretePID (lib/DiscretePID.{h,cpp}).
%
%   Construction:
%       pid = ctrl.DiscretePID(params, Ts)
%   where params is a struct with any subset of the fields returned by
%   ctrl.DiscretePID.defaultParams():
%       Kp, Ki, Kd, N, uMin, uMax, Kb, b_weight
%   (missing fields take their defaults, matching the C++ PIDParams struct).
%
%   Sign convention (compute): tracking error e = r - y.
%   Discretisation: backward Euler for both the integral and the derivative filter.
%   NaN contract: holds the last output on a non-finite input.

  properties
    p        % struct of tuning parameters
    Ts       % sample time [s]
    integral % accumulated integral I[k]
    deriv    % filtered derivative state d[k-1]
    e_prev   % previous error e[k-1]
    y_prev   % previous measurement y[k-1] (derivative-on-measurement)
    u_prev   % previous saturated output u[k-1] (anti-windup)
  end

  methods
    function obj = DiscretePID(params, sampleTime)
      obj.p = ctrl.DiscretePID.defaultParams();
      if nargin >= 1 && ~isempty(params)
        fn = fieldnames(params);
        for i = 1:numel(fn)
          obj.p.(fn{i}) = params.(fn{i});
        end
      end
      obj.Ts = sampleTime;
      obj.reset();
    end

    function u = compute(obj, error)
      %COMPUTE  u[k] from tracking error e[k] = r[k] - y[k].
      if ~isfinite(error)
        u = obj.u_prev;   % hold last output on bad measurement
        return;
      end

      % Filtered derivative (backward Euler): alpha = 1/(1+N Ts)
      alpha = 1.0 / (1.0 + obj.p.N * obj.Ts);
      d_new = alpha * obj.deriv + obj.p.Kd * obj.p.N * alpha * (error - obj.e_prev);

      % Integral: backward-Euler increment for this step
      ki_update = obj.p.Ki * obj.Ts * error;

      % Unsaturated output uses I[k] = I[k-1] + ki_update
      u_unsat = obj.p.Kp * error + (obj.integral + ki_update) + d_new;

      % Output saturation
      u_sat = max(obj.p.uMin, min(obj.p.uMax, u_unsat));

      % Integral state update with anti-windup back-calculation
      obj.integral = obj.integral + ki_update + obj.p.Kb * (u_sat - u_unsat);

      obj.deriv  = d_new;
      obj.e_prev = error;
      obj.u_prev = u_sat;
      u = u_sat;
    end

    function u = computeDoM(obj, y, r)
      %COMPUTEDOM  Derivative-on-measurement variant (no derivative kick on setpoint steps).
      error = r - y;
      if ~isfinite(error) || ~isfinite(y)
        u = obj.u_prev;
        return;
      end

      alpha = 1.0 / (1.0 + obj.p.N * obj.Ts);
      % Derivative filter on -y (not on error)
      d_new = alpha * obj.deriv - obj.p.Kd * obj.p.N * alpha * (y - obj.y_prev);

      ki_update   = obj.p.Ki * obj.Ts * error;
      prop_signal = obj.p.b_weight * r - y;   % 2DOF setpoint weight
      u_unsat     = obj.p.Kp * prop_signal + (obj.integral + ki_update) + d_new;
      u_sat       = max(obj.p.uMin, min(obj.p.uMax, u_unsat));

      obj.integral = obj.integral + ki_update + obj.p.Kb * (u_sat - u_unsat);

      obj.deriv  = d_new;
      obj.e_prev = error;
      obj.y_prev = y;
      obj.u_prev = u_sat;
      u = u_sat;
    end

    function reset(obj)
      obj.integral = 0.0;
      obj.deriv    = 0.0;
      obj.e_prev   = 0.0;
      obj.y_prev   = 0.0;
      obj.u_prev   = 0.0;
    end

    function bumplessInit(obj, u_target, error)
      %BUMPLESSINIT  Set integral so the next compute(error) returns ~u_target.
      obj.deriv    = 0.0;
      obj.e_prev   = error;
      obj.u_prev   = u_target;
      obj.integral = u_target - obj.p.Kp * error;
    end

    function t = sampleTime(obj), t = obj.Ts; end
    function u = lastOutput(obj), u = obj.u_prev; end
    function tf = hasInternalAntiWindup(obj), tf = (obj.p.Kb ~= 0.0); end
  end

  methods (Static)
    function p = defaultParams()
      %DEFAULTPARAMS  PIDParams defaults matching lib/DiscretePID.h.
      p = struct('Kp', 1.0, 'Ki', 0.0, 'Kd', 0.0, 'N', 100.0, ...
                 'uMin', -1e9, 'uMax', 1e9, 'Kb', 1.0, 'b_weight', 1.0);
    end
  end
end
