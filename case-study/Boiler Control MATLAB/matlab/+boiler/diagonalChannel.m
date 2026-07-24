function ch = diagonalChannel(sys, axis)
% DIAGONALCHANNEL  Extract the SISO (axis,axis) channel from the MIMO model.
%   Full 3-state, single input u[axis], single output y[axis].  axis is 1-based.
ch.A  = sys.A;
ch.B  = sys.B(:, axis);
ch.C  = sys.C(axis, :);
ch.D  = sys.D(axis, axis);
ch.Ts = sys.Ts;
end
