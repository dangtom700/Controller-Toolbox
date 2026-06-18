#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "ControllerToolbox.h"
#include "trampoline.h"

namespace py = pybind11;

// Forward declarations - each binding file defines one of these.
void bind_plantmodel(py::module_ &m);
void bind_controllers(py::module_ &m);
void bind_estimation(py::module_ &m);
void bind_advanced(py::module_ &m);   // RepetitiveController, Fuzzy, SubspaceID, DiscreteHinf
void bind_analysis(py::module_ &m);

PYBIND11_MODULE(ctrl_toolbox, m)
{
    m.doc() = R"doc(
Controller Toolbox - discrete-time control library Python bindings.

Quick start
-----------
>>> import ctrl_toolbox as ctrl
>>> import numpy as np
>>>
>>> # Plant: first-order TF z-domain
>>> tf = ctrl.TransferFunction([0.2], [1.0, -0.8], 0.01)
>>> sys = ctrl.tf2ss(tf)
>>>
>>> # PID
>>> p = ctrl.PIDParams(); p.Kp = 2.0; p.Ki = 0.5; p.Kd = 0.0
>>> pid = ctrl.DiscretePID(p, 0.01)
>>>
>>> # Simulation loop
>>> x = np.zeros(sys.state_size())
>>> for k in range(500):
...     u = pid.compute(1.0 - y)
...     y, x = ctrl.ss_step_copy(sys, x, np.array([u]))
...     y = float(y[0])
>>>
>>> # Available optional modules
>>> ctrl.features()
{'hinf': True, 'subspace': True, 'fuzzy': True, 'function_approx': True, 'advanced_kalman': True}
)doc";

    // Runtime feature flags - exposes CTRL_HAS_* compile-time flags to Python.
    m.def("features", &ctrl::features,
          "Return a dict of optional-module names to availability booleans.");

    bind_plantmodel(m);
    bind_controllers(m);
    bind_estimation(m);
    bind_advanced(m);
    bind_analysis(m);
}
