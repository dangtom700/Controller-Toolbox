"""
Detect and report the status of every case study under case-study/.

Language detection (in priority order):
  1. sim/main.py exists -> Python
     sim/src/main.cpp exists -> C++
  2. sim/src/ directory exists -> C++ (depth heuristic)
     sim/ directory exists -> Python
  3. Count source file extensions across the whole study tree.

Status detection (4-tier):
  Complete         - sim/ has real plant+controller code (placeholder signature
                     gone), AND logs/*.csv AND config/ AND a completed report.html report exist
                     -- plant model, controller, and performance analysis done.
  On-going         - sim/ has real plant+controller code (placeholder signature
                     gone) -- plant model and controller implemented, but no
                     performance-analysis artifact yet.
  Open placeholder - sim/ exists but the plant or controller file still matches
                     the literal body tools/new_case_study.py writes (untouched
                     scaffold). NOTE: a fresh scaffold's placeholder dynamics and
                     OpenLoop controller actually run, so logs/*.csv may already
                     exist here -- that is placeholder output, not real progress.
  Not started      - no sim/ framework at all (just a PDF, or PDF + README spec).

Output: docs/case_study_status.md  (Markdown table)
"""
from __future__ import annotations

import csv
import os
import pathlib

ROOT = pathlib.Path("case-study")
DOCS_DIR = pathlib.Path("docs")

CPP_EXTENSION = {'.cpp', '.cc', '.cxx', '.c', '.h', '.hpp', '.hh', '.hxx', '.ixx', '.cppm', '.tpp', '.ipp'}
PYTHON_EXTENSION = {'.py', '.pyc', '.ipynb', '.toml', '.so', '.pyd'}


# -- Language detection --------------------------------------------------------

def detect_language(case_study_path: pathlib.Path) -> str:
    # Step 1: check for main files
    if (case_study_path / "sim" / "main.py").exists():
        return "Python"
    if (case_study_path / "sim" / "src" / "main.cpp").exists():
        return "C++"

    # Step 2: depth heuristic on sim/
    sim = case_study_path / "sim"
    if sim.exists():
        if (sim / "src").exists():
            return "C++"
        return "Python"

    # Step 3: count source file extensions across the whole study
    cpp_count = 0
    python_count = 0
    for f in case_study_path.rglob("*"):
        if f.is_file():
            ext = f.suffix.lower()
            if ext in CPP_EXTENSION:
                cpp_count += 1
            elif ext in PYTHON_EXTENSION:
                python_count += 1

    if cpp_count > 0 and python_count == 0:
        return "C++"
    if python_count > 0 and cpp_count == 0:
        return "Python"
    if cpp_count > 0 and python_count > 0:
        return "mixed"
    return "undetermined"


# -- Status detection ----------------------------------------------------------

def _has_config(path: pathlib.Path) -> bool:
    for candidate in ("config", "configs", "sim/config", "sim/configs"):
        d = path / candidate
        if d.exists() and d.is_dir():
            return True
    return False


def _has_logs(path: pathlib.Path) -> bool:
    logs = path / "logs"
    if not logs.exists():
        return False
    return any(f.suffix == ".csv" for f in logs.iterdir() if f.is_file())


def _has_sim(path: pathlib.Path) -> bool:
    sim = path / "sim"
    if not sim.exists():
        return False
    return any(True for f in sim.rglob("main.*") if f.is_file())


def _read_text(path: pathlib.Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


# Literal fingerprints copied from the templates in tools/new_case_study.py
# (CPP_PLANT_CPP, CPP_CONTROLLERS_CPP, PY_PLANT, PY_CONTROLLERS). An untouched
# scaffold still contains these exact lines verbatim.
_PLACEHOLDER_PLANT_CPP = "-p_.param_a * x_(0) + p_.param_b * u"
_PLACEHOLDER_PLANT_PY = "-self.a * self.x[0] + self.b * u"
_PLACEHOLDER_CTRL_CPP = 'out.push_back({"OpenLoop", nullptr})'
_PLACEHOLDER_CTRL_PY = 'roster = [("OpenLoop", OpenLoop())]'


def _is_untouched_scaffold(case_study_path: pathlib.Path, language: str) -> bool:
    """True if the plant or controller file still matches the literal
    new_case_study.py placeholder body, i.e. nobody has implemented real
    physics/controllers yet -- regardless of how many log files exist."""
    sim = case_study_path / "sim"
    if language == "C++":
        plant_files = list(sim.rglob("*_plant.cpp"))
        ctrl_files = list(sim.rglob("controllers.cpp"))
        plant_sig, ctrl_sig = _PLACEHOLDER_PLANT_CPP, _PLACEHOLDER_CTRL_CPP
    elif language == "Python":
        plant_files = list(sim.glob("*_plant.py"))
        ctrl_files = list(sim.glob("controllers.py"))
        plant_sig, ctrl_sig = _PLACEHOLDER_PLANT_PY, _PLACEHOLDER_CTRL_PY
    else:
        return False
    plant_untouched = any(plant_sig in _read_text(f) for f in plant_files)
    ctrl_untouched = any(ctrl_sig in _read_text(f) for f in ctrl_files)
    return plant_untouched or ctrl_untouched

def _has_complete_report(case_study_path: pathlib.Path) -> bool:
    report_path = case_study_path / "report.html"
    if not report_path.exists():
        return False
    report_text = _read_text(report_path)
    missing_markers = [
        "No MC data found.",
        "No fault sweep data found.",
        "No MC data available for ANOVA.",
        "No WCET data found.",
    ]
    return not any(marker in report_text for marker in missing_markers)
    
def detect_status(case_study_path: pathlib.Path, language: str) -> str:
    if not _has_sim(case_study_path):
        return "Not started"

    if _is_untouched_scaffold(case_study_path, language):
        return "Open placeholder"

    has_logs = _has_logs(case_study_path)
    has_config = _has_config(case_study_path)
    has_report = _has_complete_report(case_study_path)

    if has_logs and has_config and has_report:
        return "Complete"
    return "On-going"


# -- Link helpers --------------------------------------------------------------

def _rel_from_docs(path: pathlib.Path) -> str:
    return os.path.relpath(path, DOCS_DIR).replace(os.sep, "/")

response = {True: "Yes",
            False: "_"}
def has_pdf_link(case_study_path: pathlib.Path) -> str:
    # for pdf in sorted(case_study_path.glob("*.pdf")):
    #     return f"[{pdf.name}]({_rel_from_docs(pdf)})"
    # return "-"
    return response[any(case_study_path.glob("*.pdf"))]

def find_readme_link(case_study_path: pathlib.Path) -> str:
    readme = case_study_path / "README.md"
    if readme.exists():
        return f"[README]({_rel_from_docs(readme)})"
    return "-"

check_list_stat = [
    "fault_sweep.csv",
    "mc_summary.csv",
    "mu_analysis.csv",
    "wcet_summary.csv"
]
def _has_useful_rows(path: pathlib.Path) -> bool:
    """True if path exists, parses as CSV, and has >=1 row of actual analysis output.

    These are small per-study summary CSVs (one row per scenario/controller/sample,
    not per timestep), so reading them in full is fine -- unlike logs/run_*.csv.
    mu_analysis.csv rows carry a "status" column ("ok" vs "no_columns"/"too_short");
    a file can exist with every row skipped (e.g. a study with no continuous y/u
    pair to fit), which is not the same as analysis having actually run. Files
    without a "status" column (fault_sweep/mc_summary/wcet_summary) just need
    at least one data row beyond the header.
    """
    if not path.exists():
        return False
    try:
        with path.open("r", newline="", encoding="utf-8", errors="ignore") as fh:
            reader = csv.DictReader(fh)
            if reader.fieldnames is None:
                return False
            if "status" in reader.fieldnames:
                return any(row.get("status") == "ok" for row in reader)
            return any(True for _ in reader)
    except OSError:
        return False


def get_check_list(case_study_path: pathlib.Path) -> list[str]:
    checklist = []
    for file in check_list_stat:
        checklist.append(
            response[_has_useful_rows(case_study_path / file)]
        )

    return checklist


# -- IAE semaphore ---------------------------------------------------------------
#
# logs/run_*.csv in this repo can exceed 1 MB each (2000+ files, ~2.7 GB
# combined), so the checks below read only a file's header line and, when
# needed, its final line -- never the whole file -- to keep this script fast.
# This script must also stay pandas/numpy-free: run.py Phase 7 runs it with
# the bare interpreter, not the soft_robotics conda env, so it duplicates
# (rather than imports) tools/metrics.py's extract_final_iae() column logic.

IAE_FINE, IAE_NO_DATA, IAE_NO_FINITE = 0, 1, 2
IAE_LABEL = {
    IAE_FINE: "Fine",
    IAE_NO_DATA: "No data",
    IAE_NO_FINITE: "No finite IAE",
}


def _csv_header(path: pathlib.Path) -> list[str] | None:
    try:
        with path.open("r", newline="", encoding="utf-8", errors="ignore") as fh:
            return next(csv.reader(fh), None)
    except OSError:
        return None


def _csv_last_line(path: pathlib.Path, chunk_size: int = 8192) -> str | None:
    """Tail-read the final non-blank line of a CSV without loading the rest."""
    try:
        with path.open("rb") as fh:
            fh.seek(0, os.SEEK_END)
            pos = fh.tell()
            if pos == 0:
                return None
            chunk = b""
            while True:
                read_size = min(chunk_size, pos)
                pos -= read_size
                fh.seek(pos)
                chunk = fh.read(read_size) + chunk
                trimmed = chunk.rstrip(b"\r\n")
                newline_idx = trimmed.rfind(b"\n")
                if newline_idx != -1 or pos == 0:
                    return trimmed[newline_idx + 1:].decode("utf-8", errors="ignore")
    except OSError:
        return None


def _extract_final_iae(path: pathlib.Path) -> float | None:
    """Stdlib mirror of tools/metrics.py::extract_final_iae: last-row value
    of a recognised cumulative IAE column (scalar or summed MIMO IAE_y*), or
    a trapezoidal integral of an error column. None where that function
    would return NaN (unrecognised schema or unparsable values)."""
    header = _csv_header(path)
    if not header:
        return None

    for col in ("iae_cumulative", "iae", "IAE_cumulative"):
        if col in header:
            last_line = _csv_last_line(path)
            last_row = next(csv.reader([last_line]), None) if last_line else None
            if not last_row or len(last_row) != len(header):
                return None
            try:
                return float(dict(zip(header, last_row))[col])
            except ValueError:
                return None

    mimo_cols = [c for c in header if c.startswith("IAE_y") or c.startswith("iae_y")]
    if mimo_cols:
        last_line = _csv_last_line(path)
        last_row = next(csv.reader([last_line]), None) if last_line else None
        if not last_row or len(last_row) != len(header):
            return None
        row = dict(zip(header, last_row))
        try:
            return sum(float(row[c]) for c in mimo_cols)
        except ValueError:
            return None

    e_col = next((c for c in ("error", "e1", "e") if c in header), None)
    if e_col is None:
        return None
    t_col = "t" if "t" in header else ("time" if "time" in header else None)
    try:
        with path.open("r", newline="", encoding="utf-8", errors="ignore") as fh:
            reader = csv.DictReader(fh)
            if t_col is None:
                return sum(abs(float(r[e_col])) for r in reader)
            total = 0.0
            prev: tuple[float, float] | None = None
            for r in reader:
                t_val, e_val = float(r[t_col]), float(r[e_col])
                if prev is not None:
                    prev_t, prev_e = prev
                    total += abs(prev_e) * (t_val - prev_t)
                prev = (t_val, e_val)
            return total
    except (OSError, ValueError, KeyError):
        return None


def get_iae_status(case_study_path: pathlib.Path) -> int:
    """Semaphore mirroring generate_report.py::_section_summary scoped to one
    study: IAE_FINE once any run has a finite IAE, IAE_NO_DATA when no
    logs/run_*.csv exist at all ("No run data found"), else IAE_NO_FINITE
    ("No finite IAE values in run data")."""
    logs = case_study_path / "logs"
    csvs = sorted(logs.glob("run_*.csv")) if logs.exists() else []
    if not csvs:
        return IAE_NO_DATA
    for f in csvs:
        if _extract_final_iae(f) is not None:
            return IAE_FINE
    return IAE_NO_FINITE


_RECOGNISED_IAE_COLS = {"iae_cumulative", "iae", "IAE_cumulative", "error", "e1", "e"}


def get_iae_notes(case_study_path: pathlib.Path, status: int, sample_n: int = 6) -> str:
    """Diagnostic for IAE_NO_FINITE studies: which recognised IAE/error column
    was missing (or present-but-unparsable), plus a sample of the columns the
    study's CSVs actually have. "_" for IAE_FINE/IAE_NO_DATA -- those are
    already explained by the Status/IAE columns."""
    if status != IAE_NO_FINITE:
        return "_"
    logs = case_study_path / "logs"
    csvs = sorted(logs.glob("run_*.csv")) if logs.exists() else []
    if not csvs:
        return "_"
    header = _csv_header(csvs[0])
    if not header:
        return "no readable header"
    sample = ", ".join(header[:sample_n]) + (", ..." if len(header) > sample_n else "")
    recognised = [c for c in header if c in _RECOGNISED_IAE_COLS or c.startswith("IAE_y") or c.startswith("iae_y")]
    if recognised:
        return f"found {', '.join(recognised)} but value unparsable/non-finite; has: {sample}"
    return f"no iae/error column; has: {sample}"

# -- Main ----------------------------------------------------------------------

def main() -> None:
    entries = []
    for name in sorted(ROOT.iterdir(), key=lambda p: p.name.lower()):
        # common/ is the C++ header
        if not name.is_dir() or name.name == "__pycache__" or name.name == "common":
            continue
        language  = detect_language(name)
        status    = detect_status(name, language)
        has_explicit_reference = has_pdf_link(name)
        link      = find_readme_link(name)
        iae_status = get_iae_status(name)
        iae_label = IAE_LABEL[iae_status]
        iae_notes = get_iae_notes(name, iae_status)
        [check1, check2, check3, check4] = get_check_list(name)
        result = (name.name, language, has_explicit_reference, status, iae_label, iae_notes, link, check1, check2, check3, check4)
        entries.append(result)

    auto_message = (
        "# Case Study Status\n\n"
        "<!-- Auto-generated by tools/case_study_tracker.py - do not edit manually -->\n\n"
    )
    headers = ["Case Study Name", "Language", "Reference?", "Status", "IAE", "IAE Notes", "Link", "fault_sweep", "mc_summary", "mu_analysis", "wcet_summary"]
    # Build markdown table
    line1 = "| " + " | ".join(headers) + " |\n"
    line2 = "".join(["|---"]*len(headers)) + " |\n"
    header = line1 + line2
    rows = "\n".join(
        f"| {name} | {lang} | {ref} | {status} | {iae} | {iae_notes} | {lnk} | {fault_sweep} | {mc_summary} | {mu_analysis} | {wcet_summary} |"
        for name, lang, ref, status, iae, iae_notes, lnk, fault_sweep, mc_summary, mu_analysis, wcet_summary in entries
    )

    out_path = DOCS_DIR / "case_study_status.md"
    out_path.write_text(auto_message + header + rows + "\n", encoding="utf-8")
    print(f"Written {len(entries)} entries -> {out_path}")

    # Summary counts
    from collections import Counter
    status_counts = Counter(e[3] for e in entries)
    lang_counts   = Counter(e[1] for e in entries)
    iae_counts    = Counter(e[4] for e in entries)
    print(f"\nStatus:   {dict(status_counts)}")
    print(f"Language: {dict(lang_counts)}")
    print(f"IAE:      {dict(iae_counts)}")


if __name__ == "__main__":
    main()
