function s = loadScenario(path)
% LOADSCENARIO  Parse a config/scenarios/*.json file into a scenario struct.
%   Mirrors ScenarioConfig::fromJson (simulation_runner.cpp).
cfg = jsondecode(fileread(path));

s.id              = local_get(cfg, 'id', 'unknown');
s.description     = local_get(cfg, 'description', '');
s.operating_point = local_get(cfg, 'operating_point', 'B');
s.mode            = local_get(cfg, 'mode', 'regulation');
s.duration_s      = local_get(cfg, 'duration_s', 3600);
s.Ts              = local_get(cfg, 'Ts', 1.0);
s.dx0             = local_col3(local_get(cfg, 'dx0', [0;0;0]));
s.setpoint_dy     = local_col3(local_get(cfg, 'setpoint_dy', [0;0;0]));

s.has_transition   = false;
s.transition_time_s = 0.0;
s.transition_to_op  = 'B';
if isfield(cfg, 'transition_time_s')
    s.has_transition    = true;
    s.transition_time_s = cfg.transition_time_s;
    s.transition_to_op  = local_get(cfg, 'transition_to_op', 'B');
end

s.has_periodic    = false;
s.periodic_amp    = [0;0;0];
s.periodic_freq_hz = 0.0;
if strcmp(s.mode, 'periodic_tracking')
    s.has_periodic     = true;
    s.periodic_amp     = local_col3(local_get(cfg, 'periodic_amp', [0;0;0]));
    s.periodic_freq_hz = local_get(cfg, 'periodic_freq_hz', 0.005);
end
end

function v = local_get(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end

function c = local_col3(v)
c = v(:);
if numel(c) < 3, c = [c; zeros(3 - numel(c), 1)]; end
c = c(1:3);
end
