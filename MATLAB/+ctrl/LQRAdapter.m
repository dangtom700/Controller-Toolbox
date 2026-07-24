classdef LQRAdapter < handle
%LQRADAPTER  Wrap a ctrl.DiscreteLQR behind a controller-style interface.
%   Port of ctrl::LQRAdapter (lib/DiscreteLQR.h).  State and reference are supplied
%   via function handles, decoupling the LQR from any specific sensor/trajectory.
%
%       adapter = ctrl.LQRAdapter(lqr, stateProvider)
%       adapter = ctrl.LQRAdapter(lqr, stateProvider, refProvider)
%
%   stateProvider() must return the current state x[k] (n x 1); refProvider() (optional)
%   returns x_ref[k] (n x 1) or [] for regulation to the origin.  For time-varying state
%   in a loop, have stateProvider close over a handle object (see MATLAB/examples).
%
%   The scalar compute() returns u(1); use computeVec() for the full m x 1 control vector.

  properties
    lqr        % ctrl.DiscreteLQR (value)
    stateFn    % function handle -> x[k]
    refFn      % function handle -> x_ref[k], or []
  end

  methods
    function obj = LQRAdapter(lqr, stateProvider, refProvider)
      if nargin < 3, refProvider = []; end
      obj.lqr     = lqr;
      obj.stateFn = stateProvider;
      obj.refFn   = refProvider;
    end

    function u = compute(obj, ~)
      %COMPUTE  Scalar interface: returns u(1).  The signal argument is ignored.
      uv = obj.computeVec([]);
      u  = uv(1);
    end

    function uv = computeVec(obj, ~)
      %COMPUTEVEC  Full control vector u[k] = -K (x - x_ref).
      x_ref = [];
      if ~isempty(obj.refFn), x_ref = obj.refFn(); end
      uv = obj.lqr.compute(obj.stateFn(), x_ref);
    end

    function reset(~), end
    function t = sampleTime(obj), t = obj.lqr.Ts;            end
    function h = isHealthy(obj),  h = obj.lqr.dareConverged; end
  end
end
