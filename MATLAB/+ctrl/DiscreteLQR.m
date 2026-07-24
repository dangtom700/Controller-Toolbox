classdef DiscreteLQR
%DISCRETELQR  Discrete-time Linear Quadratic Regulator.  Port of ctrl::DiscreteLQR.
%   Solves the DARE offline via the doubling algorithm (Pappas-Laub-Sandell), then
%   applies u[k] = -K (x - x_ref) + u_ff.
%
%   Construction:
%       lqr = ctrl.DiscreteLQR(plant, params)
%   where plant is a ctrl.StateSpace and params is a struct with fields Q, R.
%
%   Stateless at runtime (value class): pass the current state to compute() each step.
%   Equivalent MATLAB (Control System Toolbox): K = dlqr(A, B, Q, R).

  properties
    K               % optimal feedback gain (m x n)
    P               % DARE stabilising solution (n x n)
    Ts
    dareConverged   % true if the DARE converged to tolerance
    dareIterations  % doubling iterations taken
    nStates
    nInputs
  end

  methods
    function obj = DiscreteLQR(plant, params)
      obj.Ts      = plant.Ts;
      obj.nStates = size(plant.A, 1);
      obj.nInputs = size(plant.B, 2);

      res = ctrl.DiscreteLQR.solveDARE(plant.A, plant.B, params.Q, params.R);

      % Fallback: Tikhonov-regularised state cost Q + rho*I if the DARE didn't converge.
      if ~res.converged
        rho     = 1e-9 * (1.0 + norm(params.Q, 'fro'));
        Qreg    = params.Q + rho * eye(obj.nStates);
        res_reg = ctrl.DiscreteLQR.solveDARE(plant.A, plant.B, Qreg, params.R);
        if res_reg.converged
          res = res_reg;
        end
      end

      obj.dareConverged  = res.converged;
      obj.dareIterations = res.iterations;
      obj.P = res.P;

      S     = params.R + plant.B.' * obj.P * plant.B;
      obj.K = S \ (plant.B.' * obj.P * plant.A);
    end

    function u = compute(obj, x, x_ref, u_ff)
      %COMPUTE  u[k] = -K (x - x_ref) + u_ff.  x_ref/u_ff default to empty (zero).
      if nargin < 3, x_ref = []; end
      if nargin < 4, u_ff  = []; end
      x  = x(:);
      xe = x;
      if ~isempty(x_ref), xe = x - x_ref(:); end
      u = -obj.K * xe;
      if ~isempty(u_ff), u = u + u_ff(:); end
    end

    function K = gainMatrix(obj),      K = obj.K;             end
    function P = riccatiSolution(obj), P = obj.P;             end
    function t = sampleTime(obj),      t = obj.Ts;            end
  end

  methods (Static)
    function res = solveDARE(A, B, Q, R)
      %SOLVEDARE  Doubling-algorithm DARE solver; never throws (see res.converged).
      %   Port of ctrl::DiscreteLQR::solveDARE.  Returns struct(P, converged, iterations).
      n       = size(A, 1);
      maxIter = 100;
      tol     = 1e-12;
      In      = eye(n);

      G0 = B * (R \ B.');   % B R^-1 B'
      X  = Q;               % X_0 = Q
      Lk = A;               % L_0 = A
      Gk = G0;              % G_0 = B R^-1 B'

      res = struct('P', X, 'converged', false, 'iterations', maxIter);

      for iter = 1:maxIter
        W  = In + Gk * X;
        Z1 = W \ Lk;
        Z2 = W \ Gk;

        X_new = X  + Lk.' * X * Z1;
        G_new = Gk + Lk * Z2 * Lk.';
        L_new = Lk * Z1;

        err = norm(X_new - X, 'fro') / (1.0 + norm(X, 'fro'));

        % Enforce symmetry each iteration (matches the C++ implementation).
        X  = 0.5 * (X_new + X_new.');
        Gk = G_new;
        Lk = L_new;

        if err < tol
          res.P = X;
          res.converged = true;
          res.iterations = iter;
          return;
        end
      end

      res.P = X;
    end
  end
end
