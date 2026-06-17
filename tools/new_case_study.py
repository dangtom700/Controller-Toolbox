#!/usr/bin/env python3
"""
new_case_study.py - scaffold a case-study framework from a source PDF.

Given a paper at ``case-study/<pdf_name>.pdf`` this generates a complete
case-study skeleton (folders + files + extracted PDF text) for EITHER a C++
study (headers + sources + CMakeLists) OR a Python-only study (modules), matching
the conventions used by the existing studies in this repo.

Usage (from repo root):
  python tools/new_case_study.py case-study/MyPaper.pdf --lang cpp
  python tools/new_case_study.py case-study/MyPaper.pdf --lang python --name "My Study"

Flags:
  --lang {cpp,python}   target template family (required)
  --name  NAME          human-readable study name (default: derived from PDF stem)
  --slug  SLUG          lowercase identifier for namespace/target (default: derived)
  --controllers N       number of controller stubs to emit (default: 3)
  --scenarios   N       number of scenario JSON files to emit (default: 5)
  --force               overwrite an existing study directory

After C++ scaffolding, register the new target in:
  case-study/CMakeLists.txt   (add_subdirectory line)
  compile.bat / compile.sh    (add the <slug>_sim target - missing targets run stale .exe)
Python studies need no registration; run.py Phase 6 auto-discovers sim/main.py.

PDF text extraction uses PyMuPDF (`pip install pymupdf`, imported as `fitz`).
If it is not installed the script still scaffolds everything and writes a
placeholder note into extracted_text.md.
"""

import argparse
import datetime
import os
import re
import sys

# --------------------------------------------------------------------------- #
# PDF text extraction (best-effort across available libraries)
# --------------------------------------------------------------------------- #

def extract_pdf_text(pdf_path):
    """Return (text, backend_name) using PyMuPDF. ('', 'none') if unavailable."""
    # PyMuPDF imports as `fitz` (classic) or `pymupdf` (newer alias).
    try:
        import fitz  # PyMuPDF
    except Exception:
        try:
            import pymupdf as fitz
        except Exception:
            return "", "none"
    try:
        out = []
        with fitz.open(pdf_path) as doc:
            for page in doc:
                out.append(page.get_text())
        return "\n\n".join(out), "pymupdf"
    except Exception:
        return "", "none"


# --------------------------------------------------------------------------- #
# Naming helpers
# --------------------------------------------------------------------------- #

def derive_slug(text):
    """lowercase snake_case identifier from arbitrary text."""
    s = re.sub(r"[^0-9a-zA-Z]+", "_", text).strip("_").lower()
    s = re.sub(r"_+", "_", s)
    if not s or s[0].isdigit():
        s = "study_" + s
    return s[:40]


def derive_name(stem):
    """human-readable title from a PDF stem."""
    s = re.sub(r"[_\-]+", " ", stem).strip()
    return s if s else "New Case Study"


def fill(template, tokens):
    out = template
    for k, v in tokens.items():
        out = out.replace("@@%s@@" % k, v)
    return out


def write_file(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(content)
    print("  + %s" % os.path.relpath(path))


# --------------------------------------------------------------------------- #
# Shared templates (config + readme + extracted text)
# --------------------------------------------------------------------------- #

README_TMPL = """# @@NAME@@

> Scaffolded @@DATE@@ by `tools/new_case_study.py` from `@@PDF@@`.
> This is a TEMPLATE - fill every TODO before counting it as implemented.

## Source

- Paper PDF: `@@PDF@@`
- Extracted text: [`extracted_text.md`](extracted_text.md) (verify; PDF extraction is lossy)

## Plant model

TODO: state vector, inputs, outputs, governing equations, parameters
(mirror `config/plant_params.json`), integrator + sample time `Ts`.

## Control objective

TODO: reference/regulation goal, constraints, primary metric (IAE / RMS / CEP / ...).

## Controllers (@@NCTRL@@)

TODO: list the controllers. The paper's proposed method must be one of them.

## Scenarios (@@NSCEN@@)

TODO: describe each scenario in `config/scenarios/`.

## Run

@@RUN_HINT@@
"""

EXTRACTED_TMPL = """# Extracted text - @@PDF@@

> Auto-extracted by `tools/new_case_study.py` (backend: **@@BACKEND@@**) on @@DATE@@.
> PDF text extraction is lossy: equations, tables, and figures are usually mangled.
> Treat this as a searchable transcript, not ground truth - always cross-check the PDF.

---

@@BODY@@
"""

PLANT_PARAMS_TMPL = """{
  "_comment": "TODO: real plant parameters for @@NAME@@",
  "Ts":        0.01,
  "param_a":   1.0,
  "param_b":   1.0,
  "u_max":     1.0,
  "u_min":    -1.0
}
"""

SCENARIO_TMPL = """{
  "id":          "@@SID@@",
  "description": "TODO describe scenario @@SNUM@@",
  "T_sim":       30.0,
  "ref_type":    "step",
  "ref_init":    0.0,
  "ref_final":   1.0,
  "step_time":   1.0
}
"""


def scaffold_common(study_dir, tokens, n_scen, pdf_text, backend):
    # extracted text
    body = pdf_text.strip() if pdf_text.strip() else (
        "*(No PDF text extracted. Run `pip install pymupdf` then re-run, "
        "or paste the paper text here manually.)*")
    t = dict(tokens, BACKEND=backend, BODY=body)
    write_file(os.path.join(study_dir, "extracted_text.md"), fill(EXTRACTED_TMPL, t))
    write_file(os.path.join(study_dir, "README.md"), fill(README_TMPL, tokens))
    write_file(os.path.join(study_dir, "config", "plant_params.json"),
               fill(PLANT_PARAMS_TMPL, tokens))
    for i in range(1, n_scen + 1):
        sid = "s%02d_scenario" % i
        st = dict(tokens, SID=sid, SNUM=str(i))
        write_file(os.path.join(study_dir, "config", "scenarios", sid + ".json"),
                   fill(SCENARIO_TMPL, st))
    write_file(os.path.join(study_dir, "logs", ".gitkeep"), "")


# --------------------------------------------------------------------------- #
# C++ templates
# --------------------------------------------------------------------------- #

CPP_PLANT_H = """#pragma once
// @@SLUG@@_plant.h - plant model for @@NAME@@ (TEMPLATE)
#include <Eigen/Dense>
#include <string>

namespace @@NS@@ {

// Plant parameters loaded from config/plant_params.json (nlohmann/json).
struct PlantParams {
    double Ts     = 0.01;
    double param_a = 1.0;
    double param_b = 1.0;
    double u_max  =  1.0;
    double u_min  = -1.0;
    static PlantParams fromJson(const std::string& path);
};

// Discrete-time plant: x[k+1] = f(x[k], u[k]); y = h(x).
class Plant {
public:
    explicit Plant(const PlantParams& p);
    void reset(const Eigen::VectorXd& x0);
    void step(double u);                 // advance one Ts (RK4/Euler internally)
    double output() const;               // measured output y
    const Eigen::VectorXd& state() const { return x_; }
    int stateSize() const { return static_cast<int>(x_.size()); }
private:
    PlantParams p_;
    Eigen::VectorXd x_;
};

}  // namespace @@NS@@
"""

CPP_PLANT_CPP = """#include "@@SLUG@@_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace @@NS@@ {

PlantParams PlantParams::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j;
    PlantParams p;
    p.Ts      = j.value("Ts", p.Ts);
    p.param_a = j.value("param_a", p.param_a);
    p.param_b = j.value("param_b", p.param_b);
    p.u_max   = j.value("u_max", p.u_max);
    p.u_min   = j.value("u_min", p.u_min);
    return p;
}

Plant::Plant(const PlantParams& p) : p_(p) {
    x_ = Eigen::VectorXd::Zero(1);       // TODO: real state dimension
}

void Plant::reset(const Eigen::VectorXd& x0) { x_ = x0; }

void Plant::step(double u) {
    if (u > p_.u_max) u = p_.u_max;
    if (u < p_.u_min) u = p_.u_min;
    // TODO: real dynamics. Placeholder first-order: x' = -a*x + b*u
    const double dx = -p_.param_a * x_(0) + p_.param_b * u;
    x_(0) += p_.Ts * dx;
}

double Plant::output() const { return x_(0); }

}  // namespace @@NS@@
"""

CPP_CONTROLLERS_H = """#pragma once
// controllers.h - controller roster for @@NAME@@ (TEMPLATE)
#include "IController.h"          // from lib/ (umbrella: ControllerToolbox.h)
#include <memory>
#include <vector>
#include <string>

namespace @@NS@@ {

struct NamedController {
    std::string name;
    std::shared_ptr<ctrl::IController> ctrl;
};

// Build all controllers to be benchmarked (@@NCTRL@@ entries).
std::vector<NamedController> makeControllers(double Ts);

}  // namespace @@NS@@
"""

CPP_CONTROLLERS_CPP = """#include "controllers.h"
#include "ControllerToolbox.h"   // umbrella include from lib/

namespace @@NS@@ {

std::vector<NamedController> makeControllers(double Ts) {
    std::vector<NamedController> out;

    // TODO: replace stubs with the real roster (paper's method must appear).
@@CTRL_STUBS@@
    return out;
}

}  // namespace @@NS@@
"""

CPP_RUNNER_H = """#pragma once
// simulation_runner.h - one (controller, scenario) run for @@NAME@@ (TEMPLATE)
#include "@@SLUG@@_plant.h"
#include "controllers.h"
#include <string>

namespace @@NS@@ {

struct Scenario {
    std::string id;
    double T_sim   = 30.0;
    double ref_init  = 0.0;
    double ref_final = 1.0;
    double step_time = 1.0;
    static Scenario fromJson(const std::string& path);
};

// Run one simulation; writes CSV telemetry to <log_dir>/run_<scenario>_<ctrl>.csv.
// Returns the final cumulative IAE.
double runSimulation(const PlantParams& plant,
                     const Scenario& scen,
                     const NamedController& nc,
                     const std::string& log_dir);

}  // namespace @@NS@@
"""

CPP_RUNNER_CPP = """#include "simulation_runner.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <stdexcept>

namespace @@NS@@ {

Scenario Scenario::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j;
    Scenario s;
    s.id        = j.value("id", std::string("scenario"));
    s.T_sim     = j.value("T_sim", s.T_sim);
    s.ref_init  = j.value("ref_init", s.ref_init);
    s.ref_final = j.value("ref_final", s.ref_final);
    s.step_time = j.value("step_time", s.step_time);
    return s;
}

double runSimulation(const PlantParams& plant_p,
                     const Scenario& scen,
                     const NamedController& nc,
                     const std::string& log_dir) {
    Plant plant(plant_p);
    plant.reset(Eigen::VectorXd::Zero(plant.stateSize()));
    if (nc.ctrl) nc.ctrl->reset();

    const std::string csv = log_dir + "/run_" + scen.id + "_" + nc.name + ".csv";
    std::ofstream out(csv);
    out << "t,ref,y,u,iae_cumulative\\n";

    const int N = static_cast<int>(scen.T_sim / plant_p.Ts);
    double iae = 0.0;
    for (int k = 0; k < N; ++k) {
        const double t   = k * plant_p.Ts;
        const double ref = (t >= scen.step_time) ? scen.ref_final : scen.ref_init;
        const double y   = plant.output();
        const double e   = ref - y;
        const double u   = nc.ctrl ? nc.ctrl->compute(e) : 0.0;  // TODO: sign convention
        plant.step(u);
        iae += std::abs(e) * plant_p.Ts;
        out << t << ',' << ref << ',' << y << ',' << u << ',' << iae << '\\n';
    }
    return iae;
}

}  // namespace @@NS@@
"""

CPP_MAIN_CPP = """// main.cpp - entry point for @@NAME@@ (TEMPLATE). Target: @@TARGET@@
#include "@@SLUG@@_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>

int main(int argc, char* argv[]) {
#ifndef @@GUARD@@_SIM_SOURCE_DIR
#define @@GUARD@@_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : @@GUARD@@_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    @@NS@@::PlantParams plant;
    try {
        plant = @@NS@@::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\\n';
        return 1;
    }

    std::vector<@@NS@@::Scenario> scenarios;
    for (const auto& entry : std::filesystem::directory_iterator(scen_dir)) {
        if (entry.path().extension() == ".json")
            scenarios.push_back(@@NS@@::Scenario::fromJson(entry.path().string()));
    }

    auto controllers = @@NS@@::makeControllers(plant.Ts);
    std::cout << "@@NAME@@: " << controllers.size() << " controllers x "
              << scenarios.size() << " scenarios\\n";

    for (const auto& scen : scenarios)
        for (const auto& nc : controllers) {
            double iae = @@NS@@::runSimulation(plant, scen, nc, log_dir);
            std::cout << "  " << scen.id << " / " << nc.name
                      << "  IAE=" << iae << '\\n';
        }
    return 0;
}
"""

CPP_CMAKE = """# CMakeLists.txt - @@NAME@@ (TEMPLATE)
# Register in case-study/CMakeLists.txt:  add_subdirectory("@@DIRNAME@@")
# Add target @@TARGET@@ to compile.bat AND compile.sh.

add_executable(@@TARGET@@
    sim/src/main.cpp
    sim/src/@@SLUG@@_plant.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(@@TARGET@@ PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(@@TARGET@@ PRIVATE controller_toolbox Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_definitions(@@TARGET@@ PRIVATE
    @@GUARD@@_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(@@TARGET@@ PRIVATE cxx_std_20)
"""


def cpp_ctrl_stubs(n):
    lines = []
    for i in range(1, n + 1):
        lines.append('    // out.push_back({"Ctrl%d", std::make_shared<ctrl::DiscretePID>(/*...*/)});'
                     % i)
    lines.append('    out.push_back({"OpenLoop", nullptr});  // TODO: real controllers above')
    return "\n".join(lines)


def scaffold_cpp(study_dir, tokens, n_ctrl):
    inc = os.path.join(study_dir, "sim", "include")
    src = os.path.join(study_dir, "sim", "src")
    t = dict(tokens, CTRL_STUBS=cpp_ctrl_stubs(n_ctrl))
    write_file(os.path.join(inc, "%s_plant.h" % tokens["SLUG"]), fill(CPP_PLANT_H, t))
    write_file(os.path.join(inc, "controllers.h"), fill(CPP_CONTROLLERS_H, t))
    write_file(os.path.join(inc, "simulation_runner.h"), fill(CPP_RUNNER_H, t))
    write_file(os.path.join(src, "%s_plant.cpp" % tokens["SLUG"]), fill(CPP_PLANT_CPP, t))
    write_file(os.path.join(src, "controllers.cpp"), fill(CPP_CONTROLLERS_CPP, t))
    write_file(os.path.join(src, "simulation_runner.cpp"), fill(CPP_RUNNER_CPP, t))
    write_file(os.path.join(src, "main.cpp"), fill(CPP_MAIN_CPP, t))
    write_file(os.path.join(study_dir, "CMakeLists.txt"), fill(CPP_CMAKE, t))


# --------------------------------------------------------------------------- #
# Python templates
# --------------------------------------------------------------------------- #

PY_PLANT = '''"""@@SLUG@@_plant.py - plant model for @@NAME@@ (TEMPLATE)."""
import numpy as np


class Plant:
    """Discrete-time plant: x[k+1] = f(x[k], u[k]); y = h(x)."""

    def __init__(self, params: dict):
        self.Ts = params.get("Ts", 0.01)
        self.a = params.get("param_a", 1.0)
        self.b = params.get("param_b", 1.0)
        self.u_max = params.get("u_max", 1.0)
        self.u_min = params.get("u_min", -1.0)
        self.x = np.zeros(1)          # TODO: real state dimension

    def reset(self, x0=None):
        self.x = np.zeros_like(self.x) if x0 is None else np.asarray(x0, float)

    def step(self, u: float):
        u = float(np.clip(u, self.u_min, self.u_max))
        # TODO: real dynamics. Placeholder first-order: x' = -a*x + b*u
        dx = -self.a * self.x[0] + self.b * u
        self.x[0] += self.Ts * dx

    def output(self) -> float:
        return float(self.x[0])
'''

PY_CONTROLLERS = '''"""controllers.py - controller roster for @@NAME@@ (TEMPLATE).

ctrl_toolbox is imported with a graceful fallback; if unavailable, only the
OpenLoop stub runs.
"""
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
# sim/ -> StudyName/ -> case-study/ -> repo root
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_THIS)))
sys.path.insert(0, _ROOT)
try:
    import _setup_bindings  # noqa: F401  (sets ctrl_toolbox DLL/.so path)
    import ctrl_toolbox as ctrl
    _HAVE_CTRL = True
except Exception as _e:  # pragma: no cover
    print("WARNING: ctrl_toolbox not available (%s) - only OpenLoop will run." % _e)
    _HAVE_CTRL = False


class OpenLoop:
    """Zero-input baseline."""
    def reset(self):
        pass
    def compute(self, error: float) -> float:
        return 0.0


def make_controllers(Ts: float):
    """Return list of (name, controller) tuples (@@NCTRL@@ entries)."""
    roster = [("OpenLoop", OpenLoop())]
    if _HAVE_CTRL:
        # TODO: add the real roster; the paper's proposed method must be present.
        # roster.append(("PID", ctrl.DiscretePID(Kp, Ki, Kd, Ts)))
        pass
    return roster
'''

PY_RUNNER = '''"""simulation_runner.py - one (controller, scenario) run for @@NAME@@ (TEMPLATE)."""
import csv
import os


def run_simulation(plant, scenario: dict, name: str, controller, log_dir: str) -> float:
    """Run one simulation, write CSV telemetry, return final cumulative IAE."""
    Ts = plant.Ts
    T_sim = scenario.get("T_sim", 30.0)
    step_time = scenario.get("step_time", 1.0)
    ref_init = scenario.get("ref_init", 0.0)
    ref_final = scenario.get("ref_final", 1.0)

    plant.reset()
    if hasattr(controller, "reset"):
        controller.reset()

    os.makedirs(log_dir, exist_ok=True)
    csv_path = os.path.join(log_dir, "run_%s_%s.csv" % (scenario["id"], name))
    n = int(T_sim / Ts)
    iae = 0.0
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "ref", "y", "u", "iae_cumulative"])
        for k in range(n):
            t = k * Ts
            ref = ref_final if t >= step_time else ref_init
            y = plant.output()
            e = ref - y
            u = controller.compute(e)        # TODO: sign convention
            plant.step(u)
            iae += abs(e) * Ts
            w.writerow([t, ref, y, u, iae])
    return iae
'''

PY_MAIN = '''"""main.py - entry point for @@NAME@@ (TEMPLATE).

Runs every controller x scenario, writes CSV to ../logs/, prints a summary.
Auto-discovered by run.py Phase 6 (no CMake/compile registration needed).

Usage (from repo root):
  conda run -n soft_robotics -- python "@@MAINREL@@"
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controllers import make_controllers          # noqa: E402
from simulation_runner import run_simulation       # noqa: E402


def load_json(path):
    with open(path, "r") as fh:
        return json.load(fh)


def main():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = os.path.dirname(this_dir)                  # StudyName/
    config_dir = os.path.join(base_dir, "config")
    scen_dir = os.path.join(config_dir, "scenarios")
    log_dir = os.path.join(base_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)

    from @@SLUG@@_plant import Plant                      # noqa: E402
    plant_params = load_json(os.path.join(config_dir, "plant_params.json"))

    scenarios = []
    for fn in sorted(os.listdir(scen_dir)):
        if fn.endswith(".json"):
            scenarios.append(load_json(os.path.join(scen_dir, fn)))

    controllers = make_controllers(plant_params.get("Ts", 0.01))
    print("@@NAME@@: %d controllers x %d scenarios" % (len(controllers), len(scenarios)))

    for scen in scenarios:
        for name, c in controllers:
            plant = Plant(plant_params)
            iae = run_simulation(plant, scen, name, c, log_dir)
            print("  %s / %s  IAE=%.4f" % (scen["id"], name, iae))


if __name__ == "__main__":
    main()
'''


def scaffold_python(study_dir, tokens):
    sim = os.path.join(study_dir, "sim")
    write_file(os.path.join(sim, "%s_plant.py" % tokens["SLUG"]), fill(PY_PLANT, tokens))
    write_file(os.path.join(sim, "controllers.py"), fill(PY_CONTROLLERS, tokens))
    write_file(os.path.join(sim, "simulation_runner.py"), fill(PY_RUNNER, tokens))
    write_file(os.path.join(sim, "main.py"), fill(PY_MAIN, tokens))


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Scaffold a case-study framework from a PDF.")
    ap.add_argument("pdf", help="path to case-study/<pdf_name>.pdf")
    ap.add_argument("--lang", choices=["cpp", "python"], required=True)
    ap.add_argument("--name", help="human-readable study name")
    ap.add_argument("--slug", help="lowercase identifier (namespace/target)")
    ap.add_argument("--controllers", type=int, default=3)
    ap.add_argument("--scenarios", type=int, default=5)
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing study directory")
    args = ap.parse_args(argv)

    pdf_path = args.pdf
    if not os.path.isfile(pdf_path):
        ap.error("PDF not found: %s" % pdf_path)
    if not pdf_path.lower().endswith(".pdf"):
        ap.error("input must be a .pdf file")

    stem = os.path.splitext(os.path.basename(pdf_path))[0]
    name = args.name or derive_name(stem)
    slug = args.slug or derive_slug(args.name or stem)
    dirname = name                          # study folder lives under case-study/
    study_dir = os.path.join("case-study", dirname)

    if os.path.exists(study_dir) and not args.force:
        ap.error("study dir already exists (use --force): %s" % study_dir)

    guard = slug.upper()
    ns = slug.replace("_", "")
    target = "%s_sim" % slug
    main_rel = os.path.join(study_dir, "sim", "main.py").replace("\\", "/")
    run_hint = ('conda run -n soft_robotics -- python "%s"' % main_rel) if args.lang == "python" \
        else ('Build target `%s` (registered in compile.bat); runs in run.py Phase 4.' % target)

    tokens = {
        "NAME": name, "SLUG": slug, "NS": ns, "GUARD": guard, "TARGET": target,
        "DIRNAME": dirname, "PDF": os.path.relpath(pdf_path).replace("\\", "/"),
        "DATE": datetime.date.today().isoformat(),
        "NCTRL": str(args.controllers), "NSCEN": str(args.scenarios),
        "MAINREL": main_rel, "RUN_HINT": run_hint,
    }

    print("Extracting text from %s ..." % pdf_path)
    pdf_text, backend = extract_pdf_text(pdf_path)
    if backend == "none":
        print("  (PyMuPDF not available - writing placeholder; "
              "pip install pymupdf to enable extraction)")
    else:
        print("  extracted %d chars via %s" % (len(pdf_text), backend))

    print("Scaffolding %s study at %s" % (args.lang, study_dir))
    scaffold_common(study_dir, tokens, args.scenarios, pdf_text, backend)
    if args.lang == "cpp":
        scaffold_cpp(study_dir, tokens, args.controllers)
    else:
        scaffold_python(study_dir, tokens)

    print("\nDone. Next steps:")
    if args.lang == "cpp":
        print('  1. add_subdirectory("%s") in case-study/CMakeLists.txt' % dirname)
        print("  2. add target %s to compile.bat AND compile.sh" % target)
        print("  3. implement plant dynamics + controller roster (TODO markers)")
        print("  4. add tests/test_%s_regression.cpp" % slug)
    else:
        print("  1. implement plant dynamics + controller roster (TODO markers)")
        print("  2. run.py Phase 6 auto-discovers sim/main.py - no registration needed")
    print("  Review extracted_text.md against the PDF before trusting it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
