function op = getOperatingPoint(label)
% GETOPERATINGPOINT  Look up operating point A/B/C by label.
ops = boiler.operatingPoints();
switch upper(strtrim(label))
    case 'A', op = ops.A;
    case 'B', op = ops.B;
    case 'C', op = ops.C;
    otherwise, error('boiler:badOp', 'Unknown operating point label: %s', label);
end
end
