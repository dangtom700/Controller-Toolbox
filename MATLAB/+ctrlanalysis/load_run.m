function data = load_run(csvpath)
%LOAD_RUN  Read a case-study logs/run_*.csv into a struct of named columns.
%   data = LOAD_RUN(csvpath) returns a struct whose fields are the CSV header
%   names (sanitised to valid identifiers) and whose values are column vectors.
%   Tolerant of the non-uniform per-study schemas; assumes a numeric body.
%
%   Works in MATLAB (readmatrix) and GNU Octave (dlmread fallback).
%   See MATLAB/HANDOFF.md section 4 for the log-file contract.

  fid = fopen(csvpath, 'r');
  if fid < 0
    error('ctrlanalysis:load_run:open', 'Cannot open file: %s', csvpath);
  end
  headerLine = fgetl(fid);
  fclose(fid);
  if ~ischar(headerLine)
    error('ctrlanalysis:load_run:empty', 'Empty file: %s', csvpath);
  end

  names = strsplit(strtrim(headerLine), ',');

  if exist('readmatrix', 'file') > 0
    M = readmatrix(csvpath, 'NumHeaderLines', 1);
  else
    M = dlmread(csvpath, ',', 1, 0);   % Octave: skip 1 header row
  end

  if size(M, 2) < numel(names)
    error('ctrlanalysis:load_run:mismatch', ...
          'Header has %d columns but data has %d in %s', ...
          numel(names), size(M, 2), csvpath);
  end

  data = struct();
  for i = 1:numel(names)
    key = regexprep(strtrim(names{i}), '[^A-Za-z0-9_]', '_');
    if isempty(key) || isempty(regexp(key, '^[A-Za-z]', 'once'))
      key = ['c_' key];   % ensure a valid leading character
    end
    data.(key) = M(:, i);
  end
end
