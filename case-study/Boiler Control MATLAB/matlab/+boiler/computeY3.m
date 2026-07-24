function y3 = computeY3(x1, x2, x3, u1, u2, u3)
% COMPUTEY3  Bell-Astrom drum water-level output map (deviation [m]).
%   Faithful port of boiler_plant.cpp::computeY3.  x2 is unused (kept for
%   signature parity with the C++ source).
x3s = max(x3, 1.0);                      % guard: x3<1cm => drum essentially dry
acs = ((1.0 - 0.001538 * x3s) * 0.8 * x1 - 25.6) / ...
      (x3s * (1.0394 - 0.0012304 * x1));
qe  = (0.854 * u2 - 0.147) * x1 + 45.59 * u1 - 2.514 * u3 - 2.096;
y3  = 0.05 * (0.13073 * x3s + 100.0 * acs + qe / 9.0 - 67.975);
end
