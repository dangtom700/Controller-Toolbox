"""
tools/generate_report.py  -- RPT-1: static HTML report for Controller Toolbox results.

Reads each case study's own analysis artifacts and writes a self-contained HTML
report with 8 sections. Expected per-study layout (see tools/study_protocol.py):

    case-study/<Study>/
      config/
      logs/                 run_*.csv (required), wcet_*.csv (optional, raw)
      sim/
      mc_summary.csv        (optional - from tools/monte_carlo.py)
      fault_sweep.csv       (optional - from tools/fault_sweep.py)
      wcet_summary.csv      (optional - from tools/wcet_report.py)
      mu_analysis.csv       (optional - from tools/mu_analysis.py)
      report.html           (written by this script)
      README.md
      <PDF>                 (optional)

Sections:
  1. Summary table: best controller per study (by IAE)
  2. Controller comparison: interactive IAE bar charts per study
  3. Scenario breakdown: per-scenario performance heatmap
  4. Monte Carlo: robustness violin plots (if mc_summary.csv present)
  5. Fault analysis: degradation curves (if fault_sweep.csv present)
  6. ANOVA: significant pairs table (if mc_summary.csv present)
  7. WCET: timing bar chart (if wcet_summary.csv present)
  8. Mu analysis: robustness margin bar chart (if mu_analysis.csv present)

Usage:
    python tools/generate_report.py [options]

Options:
    --study   NAME   filter to one study (partial match); also picks the default --out
    --out     FILE   output HTML path (default: case-study/<study>/report.html if
                      --study matches exactly one study, else report.html in cwd)
    --open           open the report in the default browser after generation

Dependencies: pandas, numpy, plotly (pip install plotly)
Plotly is bundled inline (no CDN required for offline use).
"""
from __future__ import annotations
import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT))

try:
    import numpy as np
    import pandas as pd
except ImportError:
    print("ERROR: numpy and pandas required.")
    sys.exit(1)

try:
    import plotly.graph_objects as go
    import plotly.io as pio
    _HAS_PLOTLY = True
except ImportError:
    _HAS_PLOTLY = False
    print("WARN: plotly not found.  Charts will be replaced by plain tables.")
    print("      pip install plotly")

from tools.compare_controllers import _discover_csvs, _parse_filename, _extract_row, METRICS_ORDER
from tools.metrics import extract_final_iae


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------

def _resolve_study_dirs(root: Path, study_filter: str) -> list[Path]:
    case_root = root / "case-study"
    dirs = sorted(d for d in case_root.iterdir() if d.is_dir())
    if study_filter:
        dirs = [d for d in dirs if study_filter.lower() in d.name.lower()]
    return dirs


def _load_run_data(root: Path, study_filter: str) -> pd.DataFrame:
    csvs = _discover_csvs(root)
    if study_filter:
        csvs = [p for p in csvs if study_filter.lower() in p.parent.parent.name.lower()]
    rows = [_extract_row(p) for p in csvs]
    return pd.DataFrame(rows) if rows else pd.DataFrame()


def _load_extra(root: Path, filename: str, study_filter: str = "") -> pd.DataFrame:
    """Load <filename> from every case-study/<Study>/ folder (e.g. mc_summary.csv)."""
    found = sorted((root / "case-study").glob(f"*/{filename}"))
    if study_filter:
        found = [p for p in found if study_filter.lower() in p.parent.name.lower()]
    if not found:
        return pd.DataFrame()
    frames = []
    for p in found:
        try:
            df = pd.read_csv(p)
            if "study" not in df.columns:
                df["study"] = p.parent.name
            frames.append(df)
        except Exception:
            pass
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


# ---------------------------------------------------------------------------
# HTML / Plotly helpers
# ---------------------------------------------------------------------------

def _fig_to_html(fig, div_id: str) -> str:
    if not _HAS_PLOTLY or fig is None:
        return f'<div id="{div_id}"><p><em>(plotly not available)</em></p></div>'
    return pio.to_html(fig, full_html=False, include_plotlyjs="cdn", div_id=div_id)


def _df_to_html_table(df: pd.DataFrame, float_fmt: str = "{:.4f}") -> str:
    if df is None or df.empty:
        return "<p><em>No data available.</em></p>"
    styled = df.copy()
    for c in styled.select_dtypes(include=[float]).columns:
        styled[c] = styled[c].apply(lambda v: float_fmt.format(v) if not math.isnan(v) else "N/A")
    return styled.to_html(index=False, classes="data-table", border=0)


# ---------------------------------------------------------------------------
# Section builders
# ---------------------------------------------------------------------------

def _section_summary(df: pd.DataFrame) -> str:
    if df.empty:
        return "<p>No run data found.</p>"
    # Drop rows with no finite IAE so idxmin never sees an all-NA group
    # (raises "idxmin encountered all NA values in a group" otherwise).
    df = df[df["iae"].notna()]
    if df.empty:
        return "<p>No finite IAE values in run data.</p>"
    best = df.loc[df.groupby(["study", "scenario"])["iae"].idxmin()]
    best = best[["study", "scenario", "controller", "iae", "rms_error", "settle_time_s"]].sort_values(["study", "scenario"])
    return _df_to_html_table(best)


def _section_comparison(df: pd.DataFrame) -> str:
    if df.empty:
        return _df_to_html_table(df)
    if not _HAS_PLOTLY:
        return _df_to_html_table(df[["study", "scenario", "controller", "iae"]].sort_values(["study", "iae"]))

    figs_html = []
    for study, sdf in df.groupby("study"):
        fig = go.Figure()
        for scen, ssdf in sdf.groupby("scenario"):
            ssdf_s = ssdf.sort_values("iae")
            fig.add_trace(go.Bar(
                name=scen,
                x=ssdf_s["controller"],
                y=ssdf_s["iae"],
            ))
        fig.update_layout(
            title=study,
            barmode="group",
            xaxis_title="Controller",
            yaxis_title="IAE",
            height=400,
            legend_title="Scenario",
        )
        figs_html.append(_fig_to_html(fig, f"fig_cmp_{study[:20].replace(' ', '_')}"))
    return "\n".join(figs_html)


def _section_heatmap(df: pd.DataFrame) -> str:
    if not _HAS_PLOTLY or df.empty:
        return "<p>No data.</p>"
    figs_html = []
    for study, sdf in df.groupby("study"):
        pivot = sdf.pivot_table(index="controller", columns="scenario", values="iae", aggfunc="mean")
        if pivot.empty:
            continue
        fig = go.Figure(data=go.Heatmap(
            z=pivot.values,
            x=list(pivot.columns),
            y=list(pivot.index),
            colorscale="RdYlGn_r",
            colorbar=dict(title="IAE"),
        ))
        fig.update_layout(title=f"{study} - IAE heatmap", height=max(300, len(pivot.index) * 25 + 100))
        figs_html.append(_fig_to_html(fig, f"fig_heat_{study[:20].replace(' ', '_')}"))
    return "\n".join(figs_html)


def _section_mc(mc_df: pd.DataFrame) -> str:
    if not _HAS_PLOTLY or mc_df.empty or "iae" not in mc_df.columns:
        return _df_to_html_table(mc_df.head(20)) if not mc_df.empty else "<p>No MC data found.</p>"
    figs_html = []
    study = mc_df["study"].iloc[0] if "study" in mc_df.columns else "MC"
    fig = go.Figure()
    for ctrl in sorted(mc_df["controller"].unique()):
        vals = mc_df[mc_df["controller"] == ctrl]["iae"].dropna()
        fig.add_trace(go.Violin(y=vals, name=ctrl, box_visible=True, meanline_visible=True))
    fig.update_layout(title=f"{study} - MC IAE distribution", yaxis_title="IAE", height=450)
    figs_html.append(_fig_to_html(fig, "fig_mc_violin"))
    return "\n".join(figs_html)


def _section_fault(fault_df: pd.DataFrame) -> str:
    if not _HAS_PLOTLY or fault_df.empty:
        return "<p>No fault sweep data found.</p>"
    figs_html = []
    for fk in fault_df.get("fault_kind", pd.Series(dtype=str)).unique():
        sub = fault_df[fault_df["fault_kind"] == fk]
        fig = go.Figure()
        for ctrl in sorted(sub["controller"].unique()):
            csub = sub[sub["controller"] == ctrl].sort_values("magnitude")
            fig.add_trace(go.Scatter(
                x=csub["magnitude"], y=csub.get("iae", csub.iloc[:, 4]),
                mode="lines+markers", name=ctrl))
        fig.update_layout(title=f"Fault: {fk}", xaxis_title="Magnitude", yaxis_title="IAE")
        figs_html.append(_fig_to_html(fig, f"fig_fault_{fk}"))
    return "\n".join(figs_html)


def _section_anova(mc_df: pd.DataFrame) -> str:
    if mc_df.empty or "controller" not in mc_df.columns or "iae" not in mc_df.columns:
        return "<p>No MC data available for ANOVA.</p>"
    try:
        from scipy import stats as sp_stats
        controllers = sorted(mc_df["controller"].unique())
        groups = [mc_df[mc_df["controller"] == c]["iae"].dropna().values for c in controllers]
        groups = [g for g in groups if len(g) > 1]
        if len(groups) < 2:
            return "<p>Not enough groups for ANOVA.</p>"
        F, p = sp_stats.f_oneway(*groups)
        sig = "YES" if p < 0.05 else "NO"
        return f"<p><strong>One-way ANOVA:</strong> F = {F:.4f}, p = {p:.6f}, significant = {sig}</p>"
    except ImportError:
        return "<p>scipy not available for ANOVA.</p>"


def _section_wcet(wcet_df: pd.DataFrame) -> str:
    if wcet_df.empty:
        return "<p>No WCET data found.</p>"
    if not _HAS_PLOTLY:
        return _df_to_html_table(wcet_df)
    figs_html = []
    for study, sdf in wcet_df.groupby("study"):
        fig = go.Figure()
        fig.add_trace(go.Bar(x=sdf["controller"], y=sdf["wcet_us"], name="WCET"))
        fig.add_trace(go.Bar(x=sdf["controller"], y=sdf["mean_us"], name="Mean", opacity=0.7))
        fig.update_layout(barmode="overlay", title=f"{study} - WCET per controller", yaxis_title="Time [us]")
        figs_html.append(_fig_to_html(fig, f"fig_wcet_{study[:20].replace(' ', '_')}"))
    return "\n".join(figs_html)


def _section_mu(mu_df: pd.DataFrame) -> str:
    if not _HAS_PLOTLY or mu_df.empty:
        return _df_to_html_table(mu_df) if not mu_df.empty else "<p>No mu analysis data found.</p>"
    by_study = mu_df.groupby("study")["peak_T"].mean().sort_values(ascending=False).reset_index()
    colors = ["red" if v > 1.0 else "steelblue" for v in by_study["peak_T"]]
    fig = go.Figure(go.Bar(x=by_study["study"], y=by_study["peak_T"], marker_color=colors))
    fig.add_hline(y=1.0, line_dash="dash", annotation_text="Stability margin")
    fig.update_layout(title="Mu upper bound (peak |T(z)|) per study", yaxis_title="peak |T(z)|")
    return _fig_to_html(fig, "fig_mu")


# ---------------------------------------------------------------------------
# HTML template
# ---------------------------------------------------------------------------

_HTML_TEMPLATE = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Controller Toolbox Report - {date}</title>
<style>
body {{ font-family: Arial, sans-serif; margin: 2em; color: #222; }}
h1   {{ color: #003366; }}
h2   {{ color: #005599; border-bottom: 2px solid #005599; padding-bottom: 4px; }}
.data-table {{ border-collapse: collapse; width: 100%; font-size: 0.85em; }}
.data-table th {{ background: #003366; color: white; padding: 6px 10px; }}
.data-table td {{ border: 1px solid #ddd; padding: 4px 8px; }}
.data-table tr:nth-child(even) {{ background: #f5f5f5; }}
.section {{ margin-bottom: 3em; }}
nav {{ background: #f0f0f0; padding: 1em; border-radius: 4px; margin-bottom: 2em; }}
nav a {{ margin-right: 1em; color: #005599; text-decoration: none; }}
.timestamp {{ color: #777; font-size: 0.85em; }}
</style>
</head>
<body>
<h1>Controller Toolbox - Performance Report</h1>
<p class="timestamp">Generated: {date}  &nbsp;|&nbsp;  {n_runs} runs across {n_studies} studies</p>

<nav>
<a href="#summary">1. Summary</a>
<a href="#comparison">2. Comparison</a>
<a href="#heatmap">3. Heatmap</a>
<a href="#mc">4. Monte Carlo</a>
<a href="#fault">5. Fault Analysis</a>
<a href="#anova">6. ANOVA</a>
<a href="#wcet">7. WCET</a>
<a href="#mu">8. Mu Analysis</a>
</nav>

<div class="section" id="summary">
<h2>1. Summary - Best Controller per Study/Scenario</h2>
{section_summary}
</div>

<div class="section" id="comparison">
<h2>2. Controller Comparison (IAE by Study)</h2>
{section_comparison}
</div>

<div class="section" id="heatmap">
<h2>3. Scenario Breakdown Heatmap</h2>
{section_heatmap}
</div>

<div class="section" id="mc">
<h2>4. Monte Carlo Robustness</h2>
{section_mc}
</div>

<div class="section" id="fault">
<h2>5. Fault Injection Analysis</h2>
{section_fault}
</div>

<div class="section" id="anova">
<h2>6. ANOVA - Statistical Significance</h2>
{section_anova}
</div>

<div class="section" id="wcet">
<h2>7. WCET Profiling</h2>
{section_wcet}
</div>

<div class="section" id="mu">
<h2>8. Mu / Robustness Margins</h2>
{section_mu}
</div>

</body>
</html>
"""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out",   default="", help="default: case-study/<study>/report.html if --study "
                                                 "matches exactly one study, else report.html in cwd")
    ap.add_argument("--study", default="")
    ap.add_argument("--open",  action="store_true")
    args = ap.parse_args(argv)

    matched_dirs = _resolve_study_dirs(_ROOT, args.study)
    if args.out:
        out_path = Path(args.out)
    elif len(matched_dirs) == 1:
        out_path = matched_dirs[0] / "report.html"
    else:
        out_path = Path("report.html")

    print("Loading run data ...")
    run_df   = _load_run_data(_ROOT, args.study)
    mc_df    = _load_extra(_ROOT, "mc_summary.csv", args.study)
    fault_df = _load_extra(_ROOT, "fault_sweep.csv", args.study)
    wcet_df  = _load_extra(_ROOT, "wcet_summary.csv", args.study)
    mu_df    = _load_extra(_ROOT, "mu_analysis.csv", args.study)

    n_runs    = len(run_df)
    n_studies = run_df["study"].nunique() if not run_df.empty else 0
    print(f"  {n_runs} runs, {n_studies} studies")

    html = _HTML_TEMPLATE.format(
        date             = datetime.now().strftime("%Y-%m-%d %H:%M"),
        n_runs           = n_runs,
        n_studies        = n_studies,
        section_summary  = _section_summary(run_df),
        section_comparison = _section_comparison(run_df),
        section_heatmap  = _section_heatmap(run_df),
        section_mc       = _section_mc(mc_df),
        section_fault    = _section_fault(fault_df),
        section_anova    = _section_anova(mc_df),
        section_wcet     = _section_wcet(wcet_df),
        section_mu       = _section_mu(mu_df),
    )

    out_path.write_text(html, encoding="utf-8")
    print(f"Report written to: {out_path}  ({out_path.stat().st_size // 1024} KB)")

    if args.open:
        import webbrowser
        webbrowser.open(out_path.resolve().as_uri())


if __name__ == "__main__":
    main()
