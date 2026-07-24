function [Q, R] = brysonWeights(xmax, umax)
% BRYSONWEIGHTS  Bryson's rule LQR weights: Q=diag(1/xmax^2), R=diag(1/umax^2).
Q = diag(1 ./ (xmax(:).^2));
R = diag(1 ./ (umax(:).^2));
end
