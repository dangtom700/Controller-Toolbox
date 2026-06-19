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


# -- Main ----------------------------------------------------------------------

def main() -> None:
    entries = []
    for name in sorted(ROOT.iterdir(), key=lambda p: p.name.lower()):
        if not name.is_dir() or name.name == "__pycache__":
            continue
        language  = detect_language(name)
        status    = detect_status(name, language)
        has_explicit_reference = has_pdf_link(name)
        link      = find_readme_link(name)
        entries.append((name.name, language, has_explicit_reference, status, link))

    # Build markdown table
    header = (
        "# Case Study Status\n\n"
        "<!-- Auto-generated by tools/case_study_tracker.py - do not edit manually -->\n\n"
        "| Case Study Name | Language | Reference? | Status | Link |\n"
        "|---|---|---|---|---|\n"
    )
    rows = "\n".join(
        f"| {name} | {lang} | {ref} | {status} | {lnk} |"
        for name, lang, ref, status, lnk in entries
    )

    out_path = DOCS_DIR / "case_study_status.md"
    out_path.write_text(header + rows + "\n", encoding="utf-8")
    print(f"Written {len(entries)} entries -> {out_path}")

    # Summary counts
    from collections import Counter
    status_counts = Counter(e[3] for e in entries)
    lang_counts   = Counter(e[1] for e in entries)
    print(f"\nStatus:   {dict(status_counts)}")
    print(f"Language: {dict(lang_counts)}")


if __name__ == "__main__":
    main()
