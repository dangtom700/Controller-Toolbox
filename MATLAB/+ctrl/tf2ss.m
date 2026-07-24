function sys = tf2ss(tf)
%TF2SS  Convert a ctrl.TransferFunction (z^-1 form) to controllable canonical StateSpace.
%   sys = ctrl.tf2ss(tf).  Port of ctrl::tf2ss (lib/PlantModel.cpp).
%
%   For H(z^-1) = (b0 + b1 z^-1 + ... + bn z^-n) / (1 + a1 z^-1 + ... + an z^-n):
%
%       A = [ -a1  -a2  ... -an ;      B = [1;        C = [b1-a1 b0, ..., bn-an b0]
%              1    0   ...  0  ;            0;        D = b0
%              0    1   ...  0  ;            ...;
%              ...                           0]
%              0   ...  1    0  ]
%
%   Equivalent MATLAB: [A,B,C,D] = tf2ss(num, den) in the z^-1 convention.

  n   = tf.order();
  num = tf.num(:).';

  % Pad numerator to exactly n+1 coefficients (prepend zeros if shorter).
  if numel(num) < n + 1
    num = [zeros(1, n + 1 - numel(num)), num];
  end

  d0 = num(1);   % feed-through / direct term = b0

  % A: companion matrix, first row = -a1..-an, sub-diagonal = 1.
  A = zeros(n, n);
  for j = 1:n
    A(1, j) = -tf.den(j + 1);
  end
  for i = 2:n
    A(i, i - 1) = 1.0;
  end

  % B = [1, 0, ..., 0]'
  B = zeros(n, 1);
  B(1) = 1.0;

  % C = [b1-a1 b0, ..., bn-an b0]  (long-division remainder numerator)
  C = zeros(1, n);
  for j = 1:n
    C(1, j) = num(j + 1) - d0 * tf.den(j + 1);
  end

  D = d0;

  sys = ctrl.StateSpace(A, B, C, D, tf.Ts);
end
