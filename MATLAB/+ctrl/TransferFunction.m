classdef TransferFunction
%TRANSFERFUNCTION  Discrete-time transfer function in z^-1 polynomial form.
%   Port of ctrl::TransferFunction (lib/PlantModel.h).
%
%       H(z^-1) = (b0 + b1 z^-1 + ... + bm z^-m) / (1 + a1 z^-1 + ... + an z^-n)
%
%   The denominator must be monic (den(1) == 1), matching the C++ contract.
%   Equivalent MATLAB (Control System Toolbox): tf(num, den, Ts, 'Variable', 'z^-1').

  properties
    num   % numerator coefficients [b0, b1, ..., bm] (row)
    den   % monic denominator [1, a1, ..., an] (row)
    Ts    % sample time [s]
  end

  methods
    function obj = TransferFunction(numerator, denominator, sampleTime)
      if isempty(denominator) || abs(denominator(1) - 1.0) > 1e-9
        error('ctrl:TransferFunction:monic', ...
              'TransferFunction: denominator must be monic (den(1) = 1).');
      end
      obj.num = numerator(:).';
      obj.den = denominator(:).';
      obj.Ts  = sampleTime;
    end

    function n = order(obj)
      %ORDER  Denominator order n.
      n = numel(obj.den) - 1;
    end
  end
end
