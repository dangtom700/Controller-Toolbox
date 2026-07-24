function m = compute_metrics(t, y, u, ref, settle_band, settle_hysteresis)
%COMPUTE_METRICS  Faithful MATLAB port of tools/metrics.py::compute_metrics.
%   m = COMPUTE_METRICS(t, y, u, ref) returns a struct with the same six fields
%   as the Python reference:
%       iae, rms_error, settle_time_s, overshoot_pct, max_u, energy_var
%   using the error convention e = ref - y.
%
%   Optional args (defaults match Python):
%       settle_band        fraction of initial error for the settling band (0.02)
%       settle_hysteresis  consecutive in-band samples required            (10)
%
%   Keep this in sync with tools/metrics.py. See MATLAB/HANDOFF.md section 4.
%   Runs in MATLAB and GNU Octave (pure array math, no toolboxes).

  if nargin < 5 || isempty(settle_band),       settle_band = 0.02; end
  if nargin < 6 || isempty(settle_hysteresis), settle_hysteresis = 10; end

  t   = double(t(:));
  y   = double(y(:));
  u   = double(u(:));
  ref = double(ref(:));

  e = ref - y;
  N = numel(t);

  % --- IAE: left Riemann sum, drops the last sample (matches numpy version) ---
  if N > 1
    dt  = diff(t);
    iae = sum(abs(e(1:end-1)) .* dt);
  else
    iae = 0.0;
  end

  % --- RMS error ---
  rms_error = sqrt(mean(e .^ 2));

  % --- Settling time ---
  if N > 0, final_ref = ref(end); else, final_ref = 0.0; end
  if N > 0, e0_abs = abs(e(1)); else, e0_abs = 0.0; end
  if e0_abs > 1e-12
    band_abs = settle_band * e0_abs;
  else
    band_abs = settle_band * max(abs(final_ref), 1e-9);
  end

  settle_time_s = -1.0;
  if N >= settle_hysteresis
    in_band = abs(e) <= band_abs;
    for i = 1:(N - settle_hysteresis + 1)
      if all(in_band(i : i + settle_hysteresis - 1))
        settle_time_s = t(i);
        break;
      end
    end
  end

  % --- Overshoot (signed peak beyond final reference, as % of |final_ref|) ---
  overshoot_pct = 0.0;
  if abs(final_ref) > 1e-9
    if final_ref > 0
      peak = max(y);
      if peak > final_ref
        overshoot_pct = (peak - final_ref) / abs(final_ref) * 100.0;
      end
    else
      peak = min(y);
      if peak < final_ref
        overshoot_pct = (final_ref - peak) / abs(final_ref) * 100.0;
      end
    end
  end

  % --- Actuator metrics ---
  max_u      = max(abs(u));
  energy_var = var(u, 1);   % normalise by N (matches numpy np.var default)

  m = struct('iae', iae, ...
             'rms_error', rms_error, ...
             'settle_time_s', settle_time_s, ...
             'overshoot_pct', overshoot_pct, ...
             'max_u', max_u, ...
             'energy_var', energy_var);
end
