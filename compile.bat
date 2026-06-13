@echo off
setlocal EnableDelayedExpansion

cls

echo ============================================================
echo   Controller Toolbox  -  Sequential Build  (no benchmarks)
echo ============================================================
echo.

REM ---- locate cmake -------------------------------------------------------
where cmake >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: cmake not found in PATH.
    echo        Install CMake ^(https://cmake.org/download/^) and add it to PATH.
    exit /b 1
)

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
set "BUILD=%ROOT%\build"

REM =========================================================================
REM  CONFIGURE
REM =========================================================================
echo [CONFIG] Configuring CMake...
cmake -B "%BUILD%" -S "%ROOT%" -DCMAKE_BUILD_TYPE=Release
if %ERRORLEVEL% neq 0 (
    echo ERROR: CMake configuration failed.
    exit /b 1
)
echo.

REM =========================================================================
REM  BUILD — one target at a time in dependency order
REM =========================================================================
for %%T in (
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
    tug_sim
    solar_cooling_sim
    humidification_sim
    susp_sim
    buck_boost_sim
    solar_cooker_sim
    sotec_sim
    smismo_sim
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
) do (
    echo.
    echo ----------------------------------------------------------
    echo [BUILD] %%T
    echo ----------------------------------------------------------
    cmake --build "%BUILD%" --target %%T --config Release
    if !ERRORLEVEL! neq 0 (
        echo.
        echo ERROR: Failed to build [%%T]
        echo        Fix the error above and re-run compile.bat.
        exit /b 1
    )
    echo [OK] %%T
)

echo.
echo ============================================================
echo   All targets built successfully.
echo ============================================================
exit /b 0
