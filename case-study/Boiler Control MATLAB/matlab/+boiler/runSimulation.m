function [iae, ise, ev] = runSimulation(scenario, controller, logDir)
% RUNSIMULATION  Run one (scenario, controller) pair on the nonlinear plant and
%   write a conformant logs/run_<scenario>_<controller>.csv.
%
%   Faithful port of the C++ runSimulation loop with ONE deliberate deviation,
%   documented in the study README: at an operating-point transition (s07) the
%   MATLAB twin regulates to the NEW operating point (ref_dy stays the scenario
%   setpoint relative to the current op), rather than reproducing the C++ log's
%   doubled-reference quirk.  Every other step matches the C++ contract exactly.

Ts = scenario.Ts;
N  = round(scenario.duration_s / Ts);

opStart = boiler.getOperatingPoint(scenario.operating_point);
opEnd   = [];
if scenario.has_transition
    opEnd = boiler.getOperatingPoint(scenario.transition_to_op);
end

bt = boiler.BoilerTurbine();
bt.initAt(opStart);
bt.setState(opStart.x1 + scenario.dx0(1), ...
            opStart.x2 + scenario.dx0(2), ...
            opStart.x3 + scenario.dx0(3));
controller.reset();

logPath = fullfile(logDir, sprintf('run_%s_%s.csv', scenario.id, controller.name()));
logger  = boiler.TelemetryLogger(logPath);
cleaner = onCleanup(@() logger.close()); %#ok<NASGU>

for k = 0:N-1
    t = k * Ts;

    curOp  = opStart;
    ref_dy = scenario.setpoint_dy;
    if scenario.has_transition && ~isempty(opEnd) && t >= scenario.transition_time_s
        curOp = opEnd;                     % track/regulate to the new operating point
    end
    if scenario.has_periodic
        ref_dy = ref_dy + scenario.periodic_amp * sin(2*pi*scenario.periodic_freq_hz*t);
    end

    u0 = [curOp.u1; curOp.u2; curOp.u3];
    y0 = [curOp.y1; curOp.y2; curOp.y3];

    y  = bt.measureOutputs();
    dy = y - y0;

    du = controller.compute(ref_dy, dy);
    du = du(:);
    if any(~isfinite(du)), du = [0;0;0]; end

    u_abs = min(max(u0 + du, 0), 1);
    bt.setControls(u_abs(1), u_abs(2), u_abs(3));
    bt.constrainValveRate();
    bt.update();

    ref_abs = y0 + ref_dy;
    logger.log(t, y, bt.controls(), [bt.du1; bt.du2; bt.du3], ref_abs);
end

[iae, ise, ev] = logger.finals();
logger.close();
end
