% analyze.m  Rank the MATLAB-native Boiler controllers per scenario by total
%   cumulative IAE (sum of the logged IAE_y1..IAE_y3 final values).  Text output
%   (runs headless); set MAKE_PLOTS=true for tracking plots of the top/bottom
%   controllers.  Run after run_all has populated logs/.
%
%   matlab -batch "run('case-study/Boiler Control MATLAB/matlab/analyze.m')"

scriptDir = fileparts(mfilename('fullpath'));
studyDir  = fileparts(scriptDir);
repoMatlab = fullfile(fileparts(fileparts(studyDir)), 'MATLAB');
addpath(scriptDir);
if isfolder(repoMatlab), addpath(repoMatlab); end

MAKE_PLOTS = false;   % set true in interactive MATLAB for figures

logsDir = fullfile(studyDir, 'logs');
files = dir(fullfile(logsDir, 'run_*.csv'));
if isempty(files)
    error('No logs in %s. Run run_all first.', logsDir);
end

scen = {}; ctrl = {};
for i = 1:numel(files)
    base = regexprep(files(i).name, '^run_|\.csv$', '');
    idx  = find(base == '_', 1, 'last');
    scen{i} = base(1:idx-1); ctrl{i} = base(idx+1:end); %#ok<SAGROW>
end
uscen = unique(scen, 'stable');

fprintf('Boiler Control MATLAB - controller ranking by total cumulative IAE\n');
for s = 1:numel(uscen)
    sel = find(strcmp(scen, uscen{s}));
    names = ctrl(sel); iae = zeros(1, numel(sel));
    for j = 1:numel(sel)
        d = ctrlanalysis.load_run(fullfile(logsDir, files(sel(j)).name));
        iae(j) = d.IAE_y1(end) + d.IAE_y2(end) + d.IAE_y3(end);
    end
    [iae, ord] = sort(iae); names = names(ord);
    fprintf('\n=== %s ===\n', uscen{s});
    for j = 1:numel(names)
        fprintf('  %2d. %-18s  total IAE = %12.4g\n', j, names{j}, iae(j));
    end

    if MAKE_PLOTS
        figure('Name', uscen{s});
        for j = [1, numel(names)]                     % best and worst
            d = ctrlanalysis.load_run(fullfile(logsDir, ...
                sprintf('run_%s_%s.csv', uscen{s}, names{j})));
            plot(d.t, d.y1, 'DisplayName', names{j}); hold on;
        end
        plot(d.t, d.ref_y1, 'k--', 'DisplayName', 'ref');
        xlabel('t [s]'); ylabel('y1 (drum pressure)'); legend show;
        title(sprintf('%s: best vs worst y1 tracking', uscen{s}));
    end
end
fprintf('\n');
