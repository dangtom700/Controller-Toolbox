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
    example_pid_feedback
    mimo_known
    mimo_unknown
    siso_coupled
    siso_unknown
    tune_all
    simulate_all
    realtime_all
    boiler_turbine_case_study
) do (
    echo.
    echo ----------------------------------------------------------
    echo [BUILD] %%T
    echo ----------------------------------------------------------
    cmake --build "%BUILD%" --target %%T --parallel 1 --config Release
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
