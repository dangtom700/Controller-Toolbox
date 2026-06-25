cmake_minimum_required(VERSION 3.16...3.31)

function(add_example NAME)
    add_executable(${NAME} ${NAME}.cpp)
    target_link_libraries(${NAME} PRIVATE controller_toolbox)
    target_compile_features(${NAME} PRIVATE cxx_std_20)
endfunction()

add_example(ex01_tf_pid)
add_example(ex02_ss_lqr)
add_example(ex03_tf_to_ss_mpc)
add_example(ex04_esc_minimum)
add_example(ex05_smith_predictor)
add_example(ex06_lead_lag)
add_example(ex07_lqg_kalman)
add_example(ex08_smc)
add_example(ex09_adrc)
add_example(ex10_supervisory_stack)
add_example(ex11_additive_stack)
add_example(ex12_weighted_stack)
add_example(ex13_pid_antiwindup)
add_example(ex14_smc_chattering)
add_example(ex15_esc_moving_minimum)
add_example(ex16_adrc_time_varying)
add_example(ex17_kalman_filter_standalone)
add_example(ex18_mpc_disturbances)
add_example(ex19_lqr_pole_placement)
add_example(ex20_system_identification_data)
add_example(ex21_boiler_turbine_case_study)
add_example(ex22_full_pipeline_robustness)
add_example(example_pid_feedback)

# Fuzzy logic examples (ex23-ex26)
add_example(ex23_fuzzy_pd_temperature)
add_example(ex24_fuzzy_pid_dc_motor)
add_example(ex25_fuzzy_supervisor_mpc)
add_example(ex26_fuzzy_ts_gain_scheduler)

# Advanced controller examples (ex27-ex31)
add_example(ex27_function_approximator)
add_example(ex28_gpc_adaptive)
add_example(ex29_repetitive_controller)
add_example(ex30_ekf_nonlinear)
add_example(ex31_subspace_id)

# Part 18 examples (ex32-ex41)
add_example(ex32_sopdt_identification)
add_example(ex33_mhe_estimation)
add_example(ex34_mu_synthesis_full_dk)
add_example(ex35_cascade_pid)
add_example(ex36_feedforward_mpc)
add_example(ex37_smith_predictor_fractional)
add_example(ex38_sopdt_pid_tuning)
add_example(ex39_repetitive_with_ekf)
add_example(ex40_ukf_quadrotor)
add_example(ex41_lpv_gain_scheduling)

# Corrector pattern examples (ex42-ex54)
add_example(ex42_pid_inner_mpc_outer)
add_example(ex43_smc_inner_lqr_outer)
add_example(ex44_fuzzy_pd_inner_pid_outer)
add_example(ex45_adrc_inner_outer_pid)
add_example(ex46_leadlag_inner_repetitive_outer)
add_example(ex47_esc_additive_pid)
add_example(ex48_fuzzy_chattering_smc)
add_example(ex49_leadlag_integrator)
add_example(ex50_ekf_mpc)
add_example(ex51_ukf_smc)
add_example(ex52_dob_pi)
add_example(ex53_mhe_mpc_dual)
add_example(ex54_bumpless_transfer)

# Linearisation + new algorithm examples (E2/E4/E3/E1/E5)
add_example(ex55_linearisation_helper)
add_example(ex56_feedback_linearisation)
add_example(ex57_mrac)
add_example(ex58_balanced_truncation)
add_example(ex59_zpetc)

# Part 20: gap-metric clustering + LPV identification + auto gain scheduling
add_example(ex60_gap_clustering)
add_example(ex61_lpv_identification)
add_example(ex62_auto_gain_scheduler)

# Part 22: NonlinearMPC, AdaptiveSmithPredictor, AutoTuner
add_example(ex63_nonlinear_mpc)
add_example(ex64_adaptive_smith_predictor)
add_example(ex65_autotuner_pid)

# Part 24: AntiWindupWrapper
add_example(ex66_antwindup_wrapper)

# Part 25: TubeMPC, ParticleFilter
add_example(ex67_tube_mpc)
add_example(ex68_particle_filter)

# Part 30: Data-driven and ML algorithms
add_example(ex69_deepc)
add_example(ex70_ilc)
add_example(ex71_sindy)
add_example(ex72_koopman_edmd)
add_example(ex73_l1_adaptive)
add_example(ex74_cbf_safety)
add_example(ex75_gp_esn_neural)
add_example(ex76_dyna_mbrl)
add_example(ex77_scenario_mpc)
add_example(ex78_bayesian_tuner)
add_example(ex79_registry_monitor)

# Phase 2: Model Estimation (E1/E2/E3)
add_example(ex80_grey_box_estimator)

# Phase 2: Hybrid Models (H1/H2/H4)
add_example(ex81_hybrid_model_mpc)

# Metaheuristic optimisers (GA / PSO / DE)
add_example(ex82_metaheuristics)

# Robustness analysis Phase 1: Monte-Carlo closed-loop robustness
add_example(ex83_robustness_mc)

# Robustness analysis Phase 2: Gang of Four + Disk Margin
add_example(ex84_gang_of_four)

# Robustness analysis Phase 3: Structured Singular Value (mu) analysis
add_example(ex85_mu_analysis)

# Robustness analysis Phase 4: CMA-ES worst-case parameter search
add_example(ex86_worst_case)

# Robustness analysis Phase 5: Common Lyapunov function for polytopic uncertainty
add_example(ex87_lyapunov_robust)

# Robust Control Phase 4 Iteration 3: discrete H2/LQG synthesis
add_example(ex88_h2_synthesis)

# Additional Controller Types: ResonantController multi-harmonic disturbance rejection
add_example(ex89_resonant_controller)

# Additional Controller Types: NotchFilter removing a known resonance
add_example(ex90_notch_filter)

# Additional Controller Types: PhaseLockedLoop tracking a synthetic AC signal
add_example(ex91_phase_locked_loop)

# Phase 3 (Algorithm Roadmap Phase 1): SI2 correlation-based identification
add_example(ex92_correlation_id)
# Phase 3 (Algorithm Roadmap Phase 1): MO2 Nelder-Mead simplex
add_example(ex93_nelder_mead)
# Phase 3 (Algorithm Roadmap Phase 1): FD1 generalized SK complex-response fitting
add_example(ex94_sk_complex_fit)
# Phase 3 (Algorithm Roadmap Phase 1): EF1 H-infinity filter (guards itself on CTRL_HAS_HINF)
add_example(ex95_hinf_filter)
# Phase 3 (Algorithm Roadmap Phase 1): RC1 general LFT uncertainty representation
add_example(ex96_lft_system)
# Phase 3 (Algorithm Roadmap Phase 1): NC1/NC2/NC4 nonlinear control trio
add_example(ex97_backstepping)
add_example(ex98_passivity_based)
add_example(ex99_clf_controller)
# Phase 3 (Algorithm Roadmap Phase 1): SI5 Hammerstein-Wiener identification
add_example(ex100_hammerstein_wiener)

# MIMO / advanced examples (cpp/ subdirectory)
function(add_cpp_example NAME)
    add_executable(${NAME} cpp/${NAME}.cpp)
    target_link_libraries(${NAME} PRIVATE controller_toolbox)
    target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cpp)
    target_compile_features(${NAME} PRIVATE cxx_std_20)
endfunction()

add_cpp_example(mimo_known)
add_cpp_example(mimo_unknown)
add_cpp_example(siso_coupled)
add_cpp_example(siso_unknown)
