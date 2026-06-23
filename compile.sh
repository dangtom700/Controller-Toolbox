#!/usr/bin/env bash
# compile.sh  -- Sequential full build for Linux / macOS (mirrors compile.bat).
#
# Runs cmake --build one target at a time in dependency order.
# Avoid --parallel here -- ~140 targets built in parallel can peg every core on a dev
# machine for the whole run. Not a correctness requirement (CI builds the same targets
# with --parallel deliberately and that's fine there).
#
# Usage:
#   ./compile.sh              # configure (Release) + build all targets
#   ./compile.sh --no-config  # skip re-configure, build only

set -euo pipefail

NO_CONFIG=0
for arg in "$@"; do
    case "$arg" in
        --no-config) NO_CONFIG=1 ;;
        -h|--help)
            head -12 "$0" | grep '^#' | sed 's/^# \?//'
            exit 0
            ;;
        *)
            echo "Unknown flag: $arg  (use --no-config)" >&2
            exit 1
            ;;
    esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$ROOT/build"

printf '\033[0;36m============================================================\033[0m\n'
printf '\033[0;36m  Controller Toolbox  -  Sequential Build  (Linux/macOS)\033[0m\n'
printf '\033[0;36m============================================================\033[0m\n'
echo

# ── Python binding helper (non-fatal) ─────────────────────────────────────────
_build_python_binding() {
    echo
    echo "============================================================"
    echo "  Python Binding - ctrl_toolbox (Release)"
    echo "============================================================"
    echo

    if ! command -v conda >/dev/null 2>&1; then
        echo "[WARN] conda not found - Python binding skipped."
        echo "       Install Miniconda and create the soft_robotics env."
        return 0
    fi

    echo "[BINDING] Configuring with CTRL_BUILD_PYTHON_BINDINGS=ON..."
    if ! conda run -n soft_robotics -- cmake -B "$BUILD" -S "$ROOT" \
            -DCTRL_BUILD_PYTHON_BINDINGS=ON \
            -DCMAKE_BUILD_TYPE=Release \
            -G Ninja; then
        printf '    \033[0;33m[WARN]\033[0m Binding configure failed - ctrl_toolbox skipped.\n'
        return 0
    fi

    echo "[BINDING] Building ctrl_toolbox target..."
    if ! conda run -n soft_robotics -- cmake --build "$BUILD" --target ctrl_toolbox; then
        printf '    \033[0;33m[WARN]\033[0m ctrl_toolbox build failed - Python binding not updated.\n'
        return 0
    fi

    printf '    \033[0;32m[OK]\033[0m ctrl_toolbox (Python binding - Release)\n'
}

# ── Configure ─────────────────────────────────────────────────────────────────
if [[ $NO_CONFIG -eq 0 ]]; then
    echo "[CONFIG] Configuring CMake..."
    cmake -B "$BUILD" -S "$ROOT" -DCMAKE_BUILD_TYPE=Release -G Ninja
    echo
fi

# ── Build targets ─────────────────────────────────────────────────────────────
TARGETS=(
    controller_toolbox
    test_controllers
    test_tuners_extended
    test_integration
    ex01_tf_pid
    ex02_ss_lqr
    ex03_tf_to_ss_mpc
    ex04_esc_minimum
    ex05_smith_predictor
    ex06_lead_lag
    ex07_lqg_kalman
    ex08_smc
    ex09_adrc
    ex10_supervisory_stack
    ex11_additive_stack
    ex12_weighted_stack
    ex13_pid_antiwindup
    ex14_smc_chattering
    ex15_esc_moving_minimum
    ex16_adrc_time_varying
    ex17_kalman_filter_standalone
    ex18_mpc_disturbances
    ex19_lqr_pole_placement
    ex20_system_identification_data
    ex21_boiler_turbine_case_study
    ex22_full_pipeline_robustness
    ex23_fuzzy_pd_temperature
    ex24_fuzzy_pid_dc_motor
    ex25_fuzzy_supervisor_mpc
    ex26_fuzzy_ts_gain_scheduler
    ex27_function_approximator
    ex28_gpc_adaptive
    ex29_repetitive_controller
    ex30_ekf_nonlinear
    ex31_subspace_id
    ex32_sopdt_identification
    ex33_mhe_estimation
    ex34_mu_synthesis_full_dk
    ex35_cascade_pid
    ex36_feedforward_mpc
    ex37_smith_predictor_fractional
    ex38_sopdt_pid_tuning
    ex39_repetitive_with_ekf
    ex40_ukf_quadrotor
    ex41_lpv_gain_scheduling
    ex42_pid_inner_mpc_outer
    ex43_smc_inner_lqr_outer
    ex44_fuzzy_pd_inner_pid_outer
    ex45_adrc_inner_outer_pid
    ex46_leadlag_inner_repetitive_outer
    ex47_esc_additive_pid
    ex48_fuzzy_chattering_smc
    ex49_leadlag_integrator
    ex50_ekf_mpc
    ex51_ukf_smc
    ex52_dob_pi
    ex53_mhe_mpc_dual
    ex54_bumpless_transfer
    ex55_linearisation_helper
    ex56_feedback_linearisation
    ex57_mrac
    ex58_balanced_truncation
    ex59_zpetc
    ex60_gap_clustering
    ex61_lpv_identification
    ex62_auto_gain_scheduler
    ex63_nonlinear_mpc
    ex64_adaptive_smith_predictor
    ex65_autotuner_pid
    ex66_antwindup_wrapper
    ex67_tube_mpc
    ex68_particle_filter
    ex69_deepc
    ex70_ilc
    ex71_sindy
    ex72_koopman_edmd
    ex73_l1_adaptive
    ex74_cbf_safety
    ex75_gp_esn_neural
    ex76_dyna_mbrl
    ex77_scenario_mpc
    ex78_bayesian_tuner
    ex79_registry_monitor
    ex80_grey_box_estimator
    ex81_hybrid_model_mpc
    ex82_metaheuristics
    ex83_robustness_mc
    ex84_gang_of_four
    ex85_mu_analysis
    ex86_worst_case
    ex87_lyapunov_robust
    ex88_h2_synthesis
    test_autoscheduling
    test_stability_margins
    example_pid_feedback
    mimo_known
    mimo_unknown
    siso_coupled
    siso_unknown
    tune_all
    simulate_all
    realtime_all
    boiler_sim
    boiler_robustness
    tug_sim
    tug_robustness
    solar_cooling_sim
    solar_cooling_robustness
    humidification_sim
    humidification_robustness
    susp_sim
    susp_robustness
    buck_boost_sim
    buck_boost_robustness
    solar_cooker_sim
    solar_cooker_robustness
    sotec_sim
    sotec_robustness
    smismo_sim
    smismo_robustness
    stewart_sim
    stewart_robustness
    bouyancy_driven_airship_in_vertical_plan_sim
    toolbox_examples
    test_catch2_pilot
    test_catch2_advanced
    test_tugsim_regression
    test_humidification
    test_boiler_regression
    test_solar_regression
    test_humid_regression
    test_susp_regression
    test_buck_boost_regression
    test_solar_cooker_regression
    test_sotec_regression
    test_smismo_regression
    test_stewart_regression
    test_bouyancy_driven_airship_regression
)

FAILED=()

for TARGET in "${TARGETS[@]}"; do
    echo
    echo "----------------------------------------------------------"
    echo "[BUILD] $TARGET"
    echo "----------------------------------------------------------"
    if cmake --build "$BUILD" --target "$TARGET" --config Release; then
        printf '    \033[0;32m[OK]\033[0m %s\n' "$TARGET"
    else
        printf '    \033[0;31m[FAIL]\033[0m %s\n' "$TARGET"
        FAILED+=("$TARGET")
        echo
        echo "ERROR: Failed to build [$TARGET]"
        echo "       Fix the error above and re-run compile.sh."
        exit 1
    fi
done

_build_python_binding

echo
echo "============================================================"
echo "  All targets built successfully."
echo "============================================================"
exit 0
