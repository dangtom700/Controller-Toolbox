classdef StateSpace
%STATESPACE  Discrete-time state-space model.  Port of ctrl::StateSpace (lib/PlantModel.h).
%
%       x[k+1] = A x[k] + B u[k]
%       y[k]   = C x[k] + D u[k]
%
%   step() is the non-mutating one-step update (matches ctrl::ssStepCopy, the
%   Python-preferred variant): [y, x_next] = sys.step(x, u).
%   Equivalent MATLAB (Control System Toolbox): ss(A, B, C, D, Ts).

  properties
    A; B; C; D; Ts
  end

  methods
    function obj = StateSpace(A, B, C, D, Ts)
      obj.A = A; obj.B = B; obj.C = C; obj.D = D; obj.Ts = Ts;
      obj.validate();
    end

    function validate(obj)
      %VALIDATE  Check mutual consistency of matrix dimensions (throws on mismatch).
      [ar, ac] = size(obj.A);
      if ar ~= ac
        error('ctrl:StateSpace:A', 'StateSpace: A must be square (n x n).');
      end
      n = ar;
      if size(obj.B, 1) ~= n
        error('ctrl:StateSpace:B', 'StateSpace: B must have n rows.');
      end
      if size(obj.C, 2) ~= n
        error('ctrl:StateSpace:C', 'StateSpace: C must have n columns.');
      end
      p = size(obj.C, 1);
      m = size(obj.B, 2);
      if size(obj.D, 1) ~= p
        error('ctrl:StateSpace:D', 'StateSpace: D row count must match C row count (p).');
      end
      if size(obj.D, 2) ~= m
        error('ctrl:StateSpace:D', 'StateSpace: D column count must match B column count (m).');
      end
      if obj.Ts < 0
        error('ctrl:StateSpace:Ts', 'StateSpace: Ts must be >= 0 (0 = continuous-time).');
      end
    end

    function [y, x_next] = step(obj, x, u)
      %STEP  One discrete step. y[k] uses x[k] (computed BEFORE the state advance).
      x = x(:); u = u(:);
      y      = obj.C * x + obj.D * u;   % y[k] = C x[k] + D u[k]
      x_next = obj.A * x + obj.B * u;   % x[k+1] = A x[k] + B u[k]
    end

    function g = dcgain(obj)
      %DCGAIN  Steady-state gain C (I - A)^-1 B + D.
      n = size(obj.A, 1);
      g = obj.C * ((eye(n) - obj.A) \ obj.B) + obj.D;
    end

    function n = stateSize(obj),  n = size(obj.A, 1); end
    function m = inputSize(obj),  m = size(obj.B, 2); end
    function p = outputSize(obj), p = size(obj.C, 1); end
  end
end
