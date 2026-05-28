cmake_minimum_required(VERSION 3.16)

find_package(Eigen3 REQUIRED NO_MODULE)

# --- Optional module selection -----------------------------------------------
# All default ON so existing builds are unaffected.
# Turn OFF to reduce build time or binary size on constrained targets.
#
#   cmake .. -DCTRL_ENABLE_HINF=OFF          # exclude Hinf synthesis (heavy Riccati)
#   cmake .. -DCTRL_ENABLE_SUBSPACE=OFF      # exclude N4SID batch subspace ID
#   cmake .. -DCTRL_ENABLE_FUZZY=OFF         # exclude FuzzyLogic inference engine
#   cmake .. -DCTRL_ENABLE_FUNCTION_APPROX=OFF  # exclude polynomial/Pade approximation
#   cmake .. -DCTRL_ENABLE_ADVANCED_KALMAN=OFF  # exclude EKF / UKF (no nonlinear estimation)
option(CTRL_ENABLE_HINF            "Include discrete Hinf synthesis (DiscreteHinf, MixedSensitivity)" ON)
option(CTRL_ENABLE_SUBSPACE        "Include N4SID subspace identification (SubspaceID)"              ON)
option(CTRL_ENABLE_FUZZY           "Include Mamdani/TS fuzzy inference (FuzzyLogic)"                ON)
option(CTRL_ENABLE_FUNCTION_APPROX "Include Taylor/Pade function approximation"                     ON)
option(CTRL_ENABLE_ADVANCED_KALMAN "Include EKF and UKF nonlinear estimators"                        ON)

# --- Core sources (always compiled) ------------------------------------------
set(CTRL_CORE_SOURCES
    PlantModel.cpp
    DiscretePID.cpp
    DiscreteLQR.cpp
    DiscreteMPC.cpp
    ExtremumSeeker.cpp
    SmithPredictor.cpp
    DiscreteLeadLag.cpp
    DiscreteSMC.cpp
    DiscreteADRC.cpp
    KalmanFilter.cpp
    DiscreteLQG.cpp
    ControllerTuner.cpp
    ControllerStack.cpp
    TunerSuite.cpp
    MetricsAnalyzer.cpp
    SystemAnalysis.cpp
    hal/SimPlant.cpp
    RecursiveLeastSquares.cpp
    RepetitiveController.cpp
    GeneralizedPredictiveControl.cpp
)

# --- Optional sources --------------------------------------------------------
if(CTRL_ENABLE_ADVANCED_KALMAN)
    list(APPEND CTRL_CORE_SOURCES
        ExtendedKalmanFilter.cpp
        UnscentedKalmanFilter.cpp
    )
endif()

if(CTRL_ENABLE_SUBSPACE)
    list(APPEND CTRL_CORE_SOURCES SubspaceID.cpp)
endif()

if(CTRL_ENABLE_FUZZY)
    list(APPEND CTRL_CORE_SOURCES FuzzyLogic.cpp)
endif()

if(CTRL_ENABLE_HINF)
    list(APPEND CTRL_CORE_SOURCES DiscreteHinf.cpp)
endif()

if(CTRL_ENABLE_FUNCTION_APPROX)
    list(APPEND CTRL_CORE_SOURCES FunctionApproximator.cpp)
endif()

add_library(controller_toolbox STATIC ${CTRL_CORE_SOURCES})

# Required so the static library can be linked into a pybind11 shared module on Linux/macOS.
set_target_properties(controller_toolbox PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- Compile definitions reflecting enabled modules --------------------------
# Headers use these to conditionally include optional sections.
if(CTRL_ENABLE_HINF)
    target_compile_definitions(controller_toolbox PUBLIC CTRL_HAS_HINF)
endif()
if(CTRL_ENABLE_SUBSPACE)
    target_compile_definitions(controller_toolbox PUBLIC CTRL_HAS_SUBSPACE)
endif()
if(CTRL_ENABLE_FUZZY)
    target_compile_definitions(controller_toolbox PUBLIC CTRL_HAS_FUZZY)
endif()
if(CTRL_ENABLE_FUNCTION_APPROX)
    target_compile_definitions(controller_toolbox PUBLIC CTRL_HAS_FUNCTION_APPROX)
endif()
if(CTRL_ENABLE_ADVANCED_KALMAN)
    target_compile_definitions(controller_toolbox PUBLIC CTRL_HAS_ADVANCED_KALMAN)
endif()

# lib/ is the public include root: consumers write  #include "ControllerToolbox.h"
target_include_directories(controller_toolbox PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(controller_toolbox PUBLIC Eigen3::Eigen)
target_compile_features(controller_toolbox PUBLIC cxx_std_20)

# Enable warnings for the library's own translation units (PRIVATE: not forced on consumers).
target_compile_options(controller_toolbox PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# --- Install rules (consumed by pip/scikit-build and system packaging) -------
include(GNUInstallDirs)
install(TARGETS controller_toolbox
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/controller_toolbox
    FILES_MATCHING PATTERN "*.h"
)