% analyze.m -- Tier-1 MATLAB analysis driver for the Boiler Control case study.
%
% Loads the existing logs/run_*.csv (produced by the C++ boiler_sim), computes the
% standard metrics with ctrlanalysis.compute_metrics, and cross-checks a recomputed
% cumulative IAE against the logged IAE_y* columns. Prints per-run metrics + PASS/FAIL.
%
% This is the worked example referenced by MATLAB/HANDOFF.md (Tier 1 / Phase 0).
% Run from anywhere in MATLAB or Octave; it adds MATLAB/ to the path itself:
%   matlab  -batch "run('case-study/Boiler Control/matlab/analyze.m')"
%   octave  --no-gui --eval "run('case-study/Boiler Control/matlab/analyze.m')"

thisFile     = mfilename('fullpath');
scriptDir    = fileparts(thisFile);
logsDir      = fullfile(scriptDir, '..', 'logs');
matlabPkgDir = fullfile(scriptDir, '..', '..', '..', 'MATLAB');
addpath(matlabPkgDir);

scenario    = 's01_lowload_regulation';
controllers = {'PID', 'LQR', 'MPC'};
tol         = 1e-3;    % relative tolerance for the IAE cross-check
allPass     = true;

fprintf('Boiler Control -- Tier-1 MATLAB analysis (scenario %s)\n\n', scenario);
fprintf('%-8s %12s %12s %12s %10s\n', 'ctrl', 'IAE_y1', 'RMS_y1', 'settle_s', 'over_pct');

for c = 1:numel(controllers)
  name    = controllers{c};
  csvpath = fullfile(logsDir, sprintf('run_%s_%s.csv', scenario, name));
  if exist(csvpath, 'file') ~= 2
    fprintf('  SKIP %s (missing %s)\n', name, csvpath);
    continue;
  end

  d = ctrlanalysis.load_run(csvpath);

  % Standard metrics on the primary output y1 (ref_y1) and first actuator u1.
  m = ctrlanalysis.compute_metrics(d.t, d.y1, d.u1, d.ref_y1);
  fprintf('%-8s %12.4g %12.4g %12.4g %10.4g\n', ...
          name, m.iae, m.rms_error, m.settle_time_s, m.overshoot_pct);

  % Cross-check: recompute the C++-style cumulative IAE (sum |e|*Ts over ALL
  % samples, matching telemetry_logger) against the logged IAE_y* final value.
  Ts   = median(diff(d.t));
  outs = {'y1', 'y2', 'y3'};
  for k = 1:numel(outs)
    ycol   = outs{k};
    rcol   = sprintf('ref_%s', ycol);
    iaecol = sprintf('IAE_%s', ycol);
    if ~isfield(d, iaecol) || ~isfield(d, rcol), continue; end

    e          = d.(rcol) - d.(ycol);
    recomputed = sum(abs(e) .* Ts);
    logged     = d.(iaecol)(end);
    relerr     = abs(recomputed - logged) / max(abs(logged), 1e-9);

    if relerr > tol
      status  = 'FAIL';
      allPass = false;
    else
      status = 'PASS';
    end
    fprintf('    %s cross-check: recomputed=%.6g logged=%.6g relerr=%.2e [%s]\n', ...
            iaecol, recomputed, logged, relerr, status);
  end
end

fprintf('\n');
if allPass
  fprintf('PASS - MATLAB metrics reproduce the logged cumulative IAE\n');
else
  fprintf('FAIL - see cross-check mismatches above\n');
end
