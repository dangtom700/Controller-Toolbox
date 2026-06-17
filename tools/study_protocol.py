"""
tools/study_protocol.py  -- Analysis hook interface for Python case studies.

Each Python case study's sim/main.py exports the following so the analysis
tools in tools/ can call them programmatically.

This module is documentation-as-code: no ABCs, no runtime enforcement.
The names and signatures below are the contract every sim/main.py must honour.

REQUIRED ATTRIBUTES
-------------------
CONTROLLER_NAMES : list[str]
    Ordered list of controller (or planner) names as returned by .name().
    Evaluated at module-load time so monte_carlo.py can read it without
    constructing controllers. Must not trigger long computations.

REQUIRED FUNCTIONS
------------------
run_single(ctrl_name, params=None, scenario_id=None) -> dict
    Run one controller for one scenario and return the metrics dict.

    Parameters
    ----------
    ctrl_name   : str   must be an element of CONTROLLER_NAMES
    params      : dict  merged on top of nominal plant_params (None = nominal)
    scenario_id : str   scenario 'id' field (None = first scenario in dir)

    Returns
    -------
    dict - same shape as simulation_runner.run_simulation; at minimum:
        { 'IAE': float, 'name': str, 'scenario_id': str }

run_with_fault(ctrl_name, fault, scenario_id=None) -> dict
    Run one controller with a fault injected via tools.fault_injector.

    Parameters
    ----------
    ctrl_name   : str        must be in CONTROLLER_NAMES
    fault       : FaultSpec  a tools.fault_injector.FaultSpec instance
    scenario_id : str        (None = first scenario)

    Returns
    -------
    Same shape as run_single.

OPTIONAL FUNCTIONS
------------------
grey_box_model() -> tuple
    Return (ode_fn, h_fn, x0, param_names, lb, ub) for GreyBoxEstimator.
        ode_fn(x, u, p) -> xdot  [np.ndarray]
        h_fn(x, p)       -> y    [np.ndarray]
        x0               -> initial state [np.ndarray]
        param_names      -> list[str]
        lb, ub           -> parameter bounds [np.ndarray]

FAULT INJECTION CONTRACT
------------------------
Each sim/simulation_runner.py must accept an optional keyword argument:

    run_simulation(..., fault_injector=None)

When fault_injector is not None, the runner applies:

    y_obs = fault_injector.inject_sensor(t, y_true)   # before controller.compute
    u_act = fault_injector.inject_actuator(t, u_cmd)  # after controller.compute

The metrics (IAE, errors) must use the true plant state, not the corrupted
observation, so Monte Carlo / fault-sweep results remain comparable.

CSV FILE NAMING CONTRACT
------------------------
All output CSVs must use the prefix "run_":

    logs/run_{scenario_id}_{controller_name}.csv

This prefix is required by tools/compare_controllers.py and
tools/generate_report.py which discover data with:

    (root / "case-study").rglob("logs/run_*.csv")

Studies added before this convention was introduced (SurfaceShip, EV6x6)
must be patched to add the "run_" prefix.

ANALYSIS CONFIG
---------------
Each study should have config/analysis.json with:

    {
        "mc_n_samples"    : int,       # Monte Carlo samples per controller
        "mc_sigma"        : float,     # relative std of parameter perturbation
        "mc_perturb_keys" : [str, ...],# which plant_params keys to perturb
        "nominal_scenario_id": str,    # scenario used for MC and fault sweep
        "fault_specs": [
            { "kind": str, "magnitudes": [float, ...] },
            ...
        ],
        "notes": str
    }

tools/run_analysis.py reads this file to configure its sweep.
"""
# (This module exports no symbols at runtime; it exists as documentation.)
