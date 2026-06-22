#!/usr/bin/env python3
"""
new_controller.py - scaffold a new IController subclass.

Generates lib/ClassName.h, lib/ClassName.cpp, an examples/exNN_classname.cpp stub,
and an examples/python/exNN_classname.py stub, matching CONTRIBUTING.md's "Adding a
New Controller" checklist (Step 1 + Step 3). Like tools/new_case_study.py, this is
templates only: it does not edit any shared file (lib/CMakeLists.txt,
ControllerToolbox.h, Features.h, compile.bat, examples/CMakeLists.txt,
bindings/*_bindings.cpp, smoke_test.py, tests/test_catch2_advanced.cpp). Those are
exactly the "forgot to register it somewhere" class of bug CONTRIBUTING.md's PR
checklist exists to catch, so the script prints them as explicit next steps instead
of guessing at insertion points in files it doesn't own.

Usage (from repo root):
  python tools/new_controller.py MyController
  python tools/new_controller.py MyController --tag myctrl --stateless

Flags:
  name            PascalCase class name (required)
  --tag   TAG     Catch2 [tag] for tests/test_catch2_advanced.cpp (default: lowercase name)
  --stateless     Use the DiscreteLQR/LQRAdapter split (pure-math class + thin
                  IController adapter) instead of the single-class pattern. Only do
                  this if the core computation is genuinely stateless - see
                  CONTRIBUTING.md#architecture-pattern before choosing this.
  --force         overwrite existing files
"""
import argparse
import glob
import os
import re
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def write_file(path, content, force):
    if os.path.exists(path) and not force:
        print("  SKIP (exists, use --force): %s" % os.path.relpath(path, _ROOT))
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(content)
    print("  + %s" % os.path.relpath(path, _ROOT))


def next_example_number(pattern):
    nums = []
    for path in glob.glob(pattern):
        m = re.search(r"ex(\d+)_", os.path.basename(path))
        if m:
            nums.append(int(m.group(1)))
    return (max(nums) + 1) if nums else 1


HEADER_TMPL = '''#pragma once
#include "IController.h"

namespace ctrl
{

/// @brief TODO: one-line description of @@NAME@@.
struct @@NAME@@Params
{
    // TODO: tuning parameters. Keep this a POD struct (no std::string/vector/pointer
    // members) if you want @@NAME@@ to work with AtomicParamBuffer for runtime updates
    // (see docs/deployment.md "AtomicParamBuffer").
    double Kp = 1.0;
};

/// @brief TODO: one-line description of @@NAME@@.
///
/// @@SIGN_TODO@@
class @@NAME@@ : public IController
{
public:
    @@NAME@@(const @@NAME@@Params &params, double Ts);

    double compute(double signal) override;
    void reset() override;
    double sampleTime() const override { return Ts_; }

    // TODO: set once the sign convention is known - see CONTRIBUTING.md#sign-conventions
    // and lib/IController.h's SignConvention doc comment before picking a value.
    SignConvention signConvention() const override { return SignConvention::Unspecified; }

    std::string name() const override { return "@@NAME@@"; }

private:
    @@NAME@@Params p_;
    double Ts_;
    double u_prev_ = 0.0;
};

} // namespace ctrl
'''

SOURCE_TMPL = '''#include "@@NAME@@.h"
#include <cmath>

namespace ctrl
{

@@NAME@@::@@NAME@@(const @@NAME@@Params &params, double Ts)
    : p_(params), Ts_(Ts)
{
}

double @@NAME@@::compute(double signal)
{
    // NaN guard - first statement of every compute() override (CONTRIBUTING.md
    // "Numerical Safety Rules" #7). Hold the last output rather than propagate NaN.
    if (!std::isfinite(signal))
        return u_prev_;

    // TODO: real control law. Placeholder: proportional-only passthrough.
    const double u = p_.Kp * signal;

    u_prev_ = u;
    notifyObserver(u, signal);
    return u;
}

void @@NAME@@::reset()
{
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
'''

EXAMPLE_CPP_TMPL = '''// ============================================================
//  ex@@EXNUM@@_@@SLUG@@.cpp
//  TODO: one-line description of what this example demonstrates.
// ============================================================
#include "ControllerToolbox.h"
#include <cstdio>

int main()
{
    const double Ts = 0.01;
    ctrl::@@NAME@@Params p;
    ctrl::@@NAME@@ ctrl_obj(p, Ts);

    double y = 0.0;
    for (int k = 0; k < 500; ++k)
    {
        const double error = 1.0 - y; // TODO: confirm against signConvention()
        const double u = ctrl_obj.compute(error);
        y += Ts * u; // TODO: replace with a real plant step
    }

    if (!std::isfinite(y))
    {
        std::printf("FAIL\\n");
        return 1;
    }
    std::printf("PASS\\n");
    return 0;
}
'''

EXAMPLE_PY_TMPL = '''"""ex@@EXNUM@@_@@SLUG@@.py - TODO: one-line description."""
import _setup_bindings  # noqa: F401  (sets ctrl_toolbox DLL/.so path)
import ctrl_toolbox as ctrl

p = ctrl.@@NAME@@Params()
p.Kp = 1.0
c = ctrl.@@NAME@@(p, 0.01)

y = 0.0
for _ in range(500):
    error = 1.0 - y  # TODO: confirm against the real sign convention
    u = c.compute(error)
    y += 0.01 * u  # TODO: replace with a real plant step

assert y == y, "NaN encountered"  # TODO: replace with a real assertion
print("PASS")
'''


def derive_slug(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Scaffold a new IController subclass.")
    ap.add_argument("name", help="PascalCase class name, e.g. MySmcVariant")
    ap.add_argument("--tag", help="Catch2 [tag] (default: derived from name)")
    ap.add_argument("--stateless", action="store_true",
                     help="DiscreteLQR-style split (pure math + thin adapter) instead "
                          "of the single-class pattern - see CONTRIBUTING.md#architecture-pattern")
    ap.add_argument("--force", action="store_true", help="overwrite existing files")
    args = ap.parse_args(argv)

    name = args.name
    if not re.match(r"^[A-Z][A-Za-z0-9]*$", name):
        ap.error("name must be PascalCase, e.g. MySmcVariant")
    slug = derive_slug(name)
    tag = args.tag or slug

    if args.stateless:
        print("--stateless requested: this scaffolds the single-class template anyway.")
        print("Read CONTRIBUTING.md#architecture-pattern and model the DiscreteLQR / "
              "LQRAdapter split by hand - it's a deliberate, narrow exception, not a "
              "second template path worth generalizing for one flag.")

    sign_todo = ("TODO: document the compute() signal convention here once known "
                 "(e.g. \"@param signal Tracking error e[k] = r[k] - y[k].\").")

    tokens = {"NAME": name, "SLUG": slug, "SIGN_TODO": sign_todo}

    def fill(tmpl, extra=None):
        t = dict(tokens, **(extra or {}))
        out = tmpl
        for k, v in t.items():
            out = out.replace("@@%s@@" % k, str(v))
        return out

    print("Scaffolding controller %s (tag [%s])" % (name, tag))
    write_file(os.path.join(_ROOT, "lib", "%s.h" % name), fill(HEADER_TMPL), args.force)
    write_file(os.path.join(_ROOT, "lib", "%s.cpp" % name), fill(SOURCE_TMPL), args.force)

    ex_num_cpp = next_example_number(os.path.join(_ROOT, "examples", "ex*.cpp"))
    ex_num_py = next_example_number(os.path.join(_ROOT, "examples", "python", "ex*.py"))
    write_file(
        os.path.join(_ROOT, "examples", "ex%02d_%s.cpp" % (ex_num_cpp, slug)),
        fill(EXAMPLE_CPP_TMPL, {"EXNUM": "%02d" % ex_num_cpp}), args.force)
    write_file(
        os.path.join(_ROOT, "examples", "python", "ex%02d_%s.py" % (ex_num_py, slug)),
        fill(EXAMPLE_PY_TMPL, {"EXNUM": "%02d" % ex_num_py}), args.force)

    print("\nDone. Files generated above still need the real control law (TODOs).")
    print("Remaining steps from CONTRIBUTING.md#adding-a-new-controller (not automated -")
    print("these touch shared files this script doesn't own):")
    print("  1. lib/CMakeLists.txt       - add %s.cpp to CTRL_CORE_SOURCES" % name)
    print("  2. lib/ControllerToolbox.h  - add #include \"%s.h\" with a one-line docstring" % name)
    print("  3. lib/Features.h           - add {\"%s\", true}" % slug)
    print("  4. compile.bat              - add ex%02d_%s to the sequential target loop" % (ex_num_cpp, slug))
    print("     (check compile.sh too - tools/check_build_target_drift.py catches drift)")
    print("  5. examples/CMakeLists.txt  - add_example(ex%02d_%s)" % (ex_num_cpp, slug))
    print("  6. tests/test_catch2_advanced.cpp - add a [%s]-tagged test with a real numeric assertion" % tag)
    print("  7. bindings/*_bindings.cpp  - bind with std::shared_ptr<ctrl::%s> as the 3rd" % name)
    print("     py::class_ template arg, then add an assertion to bindings/smoke_test.py")
    print("  8. Fill in signConvention() once the real convention is known, and the")
    print("     sign-convention table in CONTRIBUTING.md#sign-conventions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
