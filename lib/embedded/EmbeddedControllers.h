#pragma once
/**
 * @file EmbeddedControllers.h
 * @brief Umbrella header for the embedded (no-Eigen, no-heap) controller subset.
 *
 * Include this single header to get all embedded components:
 *
 * @code
 *   #include "EmbeddedControllers.h"
 *
 *   ctrl_embedded::BasicPID<float>            pid({1.f, 0.1f, 0.f, 0.f, 0.f, -100.f, 100.f, 1.f, 10.f}, 0.01f);
 *   ctrl_embedded::BasicSMC<float>            smc({1.f, 0.1f, 0.1f, -100.f, 100.f}, 0.01f);
 *   ctrl_embedded::DiscreteIntegrator<float>  integrator(0.01f);
 *   ctrl_embedded::FixedRateFilter<float, 2>  lpf(50.f, 0.01f);
 *   ctrl_embedded::RingBuffer<float, 32>      buf;
 * @endcode
 *
 * Build without Eigen and without RTTI/exceptions:
 * @code
 *   cmake .. -DCTRL_BUILD_EMBEDDED_ONLY=ON
 * @endcode
 *
 * @note BasicPID and BasicSMC are re-exported here from the parent lib/ headers.
 *       They live in the global ctrl_embedded namespace via the include-guard alias trick;
 *       see BasicPID.h / BasicSMC.h for the original ctrl namespace definitions.
 *
 * @see lib/BasicPID.h  - Template PID (float/double, no Eigen, no IController)
 * @see lib/BasicSMC.h  - Template SMC (float/double, no Eigen, no IController)
 */

// Parent-library embedded-friendly templates (include via relative path)
#include "../BasicPID.h"
#include "../BasicSMC.h"

// Embedded-only components (this directory)
#include "DiscreteIntegrator.h"
#include "FixedRateFilter.h"
#include "RingBuffer.h"
