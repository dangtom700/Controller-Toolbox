function c = make(clsname, sys, op)
% MAKE  Construct a boiler controller by class name: c = bctrl.make('PID', sys, op).
c = feval(['bctrl.' clsname], sys, op);
end
