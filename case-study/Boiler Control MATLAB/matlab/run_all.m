function run_all(varargin)
% RUN_ALL  MATLAB-native Boiler Control simulation driver.
%   Runs every scenario x controller on the nonlinear Bell-Astrom plant using
%   MATLAB-toolbox controllers, writes conformant logs/run_*.csv, and emits a
%   root mc_summary.csv so the study registers in the Python report pipeline.
%
%   Usage:
%     run_all                                          % full 8 x 27 sweep
%     run_all('scenarios', {'s01_lowload_regulation'}) % subset of scenarios
%     run_all('controllers', {'PID','LQR','MPC'})      % subset of controllers
%     run_all('duration', 120)                         % override duration (smoke)
%
%   From a shell:
%     matlab -batch "run('case-study/Boiler Control MATLAB/matlab/run_all.m')"

    p = inputParser;
    p.addParameter('scenarios', 'all');
    p.addParameter('controllers', 'all');
    p.addParameter('duration', []);
    p.parse(varargin{:});
    opt = p.Results;

    scriptDir = fileparts(mfilename('fullpath'));
    studyDir  = fileparts(scriptDir);
    repoRoot  = fileparts(fileparts(studyDir));       % .../Controller-Toolbox
    repoMatlab = fullfile(repoRoot, 'MATLAB');
    addpath(scriptDir);
    if isfolder(repoMatlab), addpath(repoMatlab); end  % reuse +ctrlanalysis metrics

    cfgDir  = fullfile(studyDir, 'config', 'scenarios');
    logsDir = fullfile(studyDir, 'logs');
    if ~isfolder(logsDir), mkdir(logsDir); end
    mcPath  = fullfile(studyDir, 'mc_summary.csv');
    studyName = 'Boiler Control MATLAB';

    files = dir(fullfile(cfgDir, '*.json'));
    [~, order] = sort({files.name});
    files = files(order);

    ctrlList = bctrl.controllerNames();
    if ~ischar(opt.controllers) || ~strcmp(opt.controllers, 'all')
        ctrlList = cellstr(opt.controllers);
    end

    fprintf('Boiler Control MATLAB - native simulation\n');
    fprintf('==========================================\n');

    rows = {};   % accumulate mc_summary rows
    nOK = 0; nFail = 0;
    sIdx = -1;   % sample_id: one per selected scenario

    for f = 1:numel(files)
        sc = boiler.loadScenario(fullfile(cfgDir, files(f).name));
        if ~scenarioSelected(sc.id, opt.scenarios), continue; end
        if ~isempty(opt.duration), sc.duration_s = opt.duration; end
        sIdx = sIdx + 1;

        op  = boiler.getOperatingPoint(sc.operating_point);
        sys = boiler.linearize(op, sc.Ts);
        fprintf('\n=== %s (%s) | op %s | %d steps ===\n', ...
                sc.id, sc.mode, sc.operating_point, round(sc.duration_s/sc.Ts));

        for c = 1:numel(ctrlList)
            cls = ctrlList{c};
            dispName = cls; stable = 0;
            m = struct('iae',NaN,'rms_error',NaN,'settle_time_s',NaN, ...
                       'overshoot_pct',NaN,'max_u',NaN,'energy_var',NaN);
            try
                ctrl = bctrl.make(cls, sys, op);
                dispName = ctrl.name();
                boiler.runSimulation(sc, ctrl, logsDir);
                logPath = fullfile(logsDir, sprintf('run_%s_%s.csv', sc.id, dispName));
                d = ctrlanalysis.load_run(logPath);
                m = ctrlanalysis.compute_metrics(d.t, d.y1, d.u1, d.ref_y1);
                stable = double(all(isfinite(d.y1)) && all(isfinite(d.y2)) && ...
                                all(isfinite(d.y3)) && all(d.u1 >= -1e-6) && ...
                                all(d.u1 <= 1+1e-6));
                nOK = nOK + 1;
                fprintf('  %-18s IAE=%10.4g  RMS=%9.4g  stable=%d\n', ...
                        dispName, m.iae, m.rms_error, stable);
            catch err
                nFail = nFail + 1;
                fprintf(2, '  %-18s FAILED: %s\n', dispName, err.message);
            end
            rows(end+1,:) = {studyName, dispName, sIdx, m.iae, m.rms_error, ...
                             m.settle_time_s, m.overshoot_pct, m.max_u, ...
                             m.energy_var, stable}; %#ok<AGROW>
        end
    end

    writeMcSummary(mcPath, rows);
    fprintf('\nDone. %d runs OK, %d failed. Logs in %s\n', nOK, nFail, logsDir);
    fprintf('Wrote %s\n', mcPath);
end

function tf = scenarioSelected(id, sel)
    if ischar(sel) && strcmp(sel, 'all'), tf = true; return; end
    sel = cellstr(sel);
    tf = any(strcmp(id, sel)) || any(cellfun(@(s) startsWith(id, s), sel));
end

function writeMcSummary(mcPath, rows)
    fid = fopen(mcPath, 'w');
    if fid < 0, error('Cannot write %s', mcPath); end
    fprintf(fid, ['study,controller,sample_id,iae,rms_error,settle_time_s,', ...
                  'overshoot_pct,max_u,energy_var,stable\n']);
    for r = 1:size(rows,1)
        fprintf(fid, '%s,%s,%d,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d\n', ...
                rows{r,1}, rows{r,2}, rows{r,3}, rows{r,4}, rows{r,5}, ...
                rows{r,6}, rows{r,7}, rows{r,8}, rows{r,9}, rows{r,10});
    end
    fclose(fid);
end
