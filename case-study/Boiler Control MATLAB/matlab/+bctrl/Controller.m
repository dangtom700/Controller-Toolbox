classdef (Abstract) Controller < handle
% CONTROLLER  Common interface for every boiler controller.
%   du = compute(ref_dy, dy)   ref_dy,dy,du are 3x1 output/valve deviations
%   reset()                    restore initial state
%   name()                     char row used in the log filename
    methods (Abstract)
        du = compute(obj, ref_dy, dy)
        reset(obj)
        n  = name(obj)
    end
end
