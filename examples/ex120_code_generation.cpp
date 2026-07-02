/**
 * @file ex120_code_generation.cpp
 * @brief Phase 4 (DT1): generate flat, dependency-free C99 for a tuned step-based controller of
 *        each supported type, plus one corrector-fused example, and write the .h/.c pairs to
 *        disk.
 *
 * @see docs/superpowers/specs/2026-06-30-code-generation-design.md
 */

#include "ControllerToolbox.h"
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

bool writePair(const std::string &tag, const ctrl::GeneratedCode &code)
{
    namespace fs = std::filesystem;
    const fs::path dir = fs::path("generated") / tag;
    fs::create_directories(dir);
    std::ofstream(dir / "controller_gen.h") << code.header;
    std::ofstream(dir / "controller_gen.c") << code.source;
    const bool ok = !code.header.empty() && !code.source.empty() &&
                     code.header.find("controller_step") != std::string::npos;
    std::cout << "  [" << tag << "] " << (ok ? "generated" : "FAILED") << " -> "
              << dir.string() << "\n";
    return ok;
}

} // namespace

int main()
{
    bool ok = true;

    ctrl::PIDParams pid;
    pid.Kp = 2.0; pid.Ki = 0.5; pid.Kd = 0.1; pid.uMin = -5.0; pid.uMax = 5.0;
    ok &= writePair("pid", ctrl::generateControllerC(pid, 0.01));

    ctrl::SMCParams smc;
    smc.c_e = 1.0; smc.c_de = 0.05; smc.K = 3.0; smc.phi = 0.2; smc.uMin = -10.0; smc.uMax = 10.0;
    ok &= writePair("smc", ctrl::generateControllerC(smc, 0.01));

    ctrl::LeadLagParams ll;
    ll.continuousZero = 1.0; ll.continuousPole = 10.0; ll.gain = 2.0;
    ok &= writePair("leadlag", ctrl::generateControllerC(ll, 0.01));

    // One corrector-fused example: SMC + AntiWindupWrapper-equivalent, fused inline.
    ctrl::CodeGenParams cfg;
    cfg.corrector = ctrl::AntiWindupConfig{-4.0, 4.0, 0.8};
    ok &= writePair("smc_corrector", ctrl::generateControllerC(smc, 0.01, cfg));

    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
