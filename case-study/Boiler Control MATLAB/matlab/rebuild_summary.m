function rebuild_summary()
% REBUILD_SUMMARY  Regenerate mc_summary.csv from the existing logs/ WITHOUT
%   re-simulating.  Useful after a full run_all to (re)compute metrics, assigning
%   sample_id per scenario (each operating scenario = one Monte-Carlo sample).

    scriptDir = fileparts(mfilename('fullpath'));
    studyDir  = fileparts(scriptDir);
    repoMatlab = fullfile(fileparts(fileparts(studyDir)), 'MATLAB');
    addpath(scriptDir);
    if isfolder(repoMatlab), addpath(repoMatlab); end

    logsDir = fullfile(studyDir, 'logs');
    mcPath  = fullfile(studyDir, 'mc_summary.csv');
    studyName = 'Boiler Control MATLAB';

    files = dir(fullfile(logsDir, 'run_*.csv'));
    scen = {}; ctrl = {};
    for i = 1:numel(files)
        base = regexprep(files(i).name, '^run_|\.csv$', '');
        idx  = find(base == '_', 1, 'last');   % controller has no underscore
        scen{i}  = base(1:idx-1);   %#ok<AGROW>
        ctrl{i}  = base(idx+1:end); %#ok<AGROW>
    end
    uscen = unique(scen, 'stable');

    fid = fopen(mcPath, 'w');
    fprintf(fid, ['study,controller,sample_id,iae,rms_error,settle_time_s,', ...
                  'overshoot_pct,max_u,energy_var,stable\n']);
    for i = 1:numel(files)
        sid = find(strcmp(uscen, scen{i})) - 1;   % 0-based sample id
        try
            d = ctrlanalysis.load_run(fullfile(logsDir, files(i).name));
            m = ctrlanalysis.compute_metrics(d.t, d.y1, d.u1, d.ref_y1);
            stable = double(all(isfinite(d.y1)) && all(isfinite(d.y2)) && ...
                            all(isfinite(d.y3)) && all(d.u1 >= -1e-6) && all(d.u1 <= 1+1e-6));
            fprintf(fid, '%s,%s,%d,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d\n', ...
                    studyName, ctrl{i}, sid, m.iae, m.rms_error, m.settle_time_s, ...
                    m.overshoot_pct, m.max_u, m.energy_var, stable);
        catch err
            fprintf(2, 'skip %s: %s\n', files(i).name, err.message);
        end
    end
    fclose(fid);
    fprintf('Rebuilt %s from %d logs (%d scenarios)\n', mcPath, numel(files), numel(uscen));
end
