#include "CodeGenC.h"
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ctrl {
namespace {

// Formats a double as a C99 floating-point literal that round-trips exactly
// (max_digits10 = 17 for IEEE-754 double).
std::string fmtD(double v)
{
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10) << v;
    std::string s = oss.str();
    if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
        s.find('E') == std::string::npos && s.find("inf") == std::string::npos &&
        s.find("nan") == std::string::npos)
        s += ".0";
    return s;
}

std::string headerFor(const std::string &function_name)
{
    std::ostringstream h;
    h << "#ifndef CONTROLLER_GEN_H\n"
      << "#define CONTROLLER_GEN_H\n\n"
      << "double " << function_name << "(double input);\n"
      << "void controller_reset(void);\n\n"
      << "#endif\n";
    return h.str();
}

} // namespace

GeneratedCode generateControllerC(const PIDParams &p, double Ts, const CodeGenParams &cfg)
{
    if (cfg.corrector && p.Kb != 0.0)
        throw std::invalid_argument(
            "generateControllerC(PIDParams): a corrector was requested but PIDParams::Kb != 0 "
            "(DiscretePID already has built-in anti-windup). Set Kb = 0 before requesting a "
            "corrector.");

    std::ostringstream c;
    c << "#include \"controller_gen.h\"\n"
      << "#include <math.h>\n\n"
      << "static const double KP = " << fmtD(p.Kp) << ";\n"
      << "static const double KI = " << fmtD(p.Ki) << ";\n"
      << "static const double KD = " << fmtD(p.Kd) << ";\n"
      << "static const double ND = " << fmtD(p.N) << ";\n"
      << "static const double KB = " << fmtD(p.Kb) << ";\n"
      << "static const double U_MIN = " << fmtD(p.uMin) << ";\n"
      << "static const double U_MAX = " << fmtD(p.uMax) << ";\n"
      << "static const double TS = " << fmtD(Ts) << ";\n\n";

    if (cfg.corrector)
        c << "static const double CORR_U_MIN = " << fmtD(cfg.corrector->uMin) << ";\n"
          << "static const double CORR_U_MAX = " << fmtD(cfg.corrector->uMax) << ";\n"
          << "static const double CORR_KB = " << fmtD(cfg.corrector->Kb) << ";\n"
          << "static double s_corr = 0.0;\n\n";

    c << "static double s_integral = 0.0;\n"
      << "static double s_deriv = 0.0;\n"
      << "static double s_e_prev = 0.0;\n"
      << "static double s_u_prev = 0.0;\n\n"
      << "static double clampd(double v, double lo, double hi) {\n"
      << "    return v < lo ? lo : (v > hi ? hi : v);\n"
      << "}\n\n"
      << "double " << cfg.function_name << "(double error) {\n"
      << "    if (!isfinite(error)) return s_u_prev;\n";

    const std::string input = cfg.corrector ? "e_in" : "error";
    if (cfg.corrector)
        c << "    const double e_in = error + s_corr;\n";

    c << "    const double alpha = 1.0 / (1.0 + ND * TS);\n"
      << "    const double d_new = alpha * s_deriv + KD * ND * alpha * (" << input
      << " - s_e_prev);\n"
      << "    const double ki_update = KI * TS * " << input << ";\n"
      << "    const double u_unsat = KP * " << input << " + (s_integral + ki_update) + d_new;\n"
      << "    const double u_raw = clampd(u_unsat, U_MIN, U_MAX);\n"
      << "    s_integral += ki_update + KB * (u_raw - u_unsat);\n"
      << "    s_deriv = d_new;\n"
      << "    s_e_prev = " << input << ";\n";

    if (cfg.corrector)
        c << "    const double u_sat = clampd(u_raw, CORR_U_MIN, CORR_U_MAX);\n"
          << "    s_corr = CORR_KB * (u_sat - u_raw);\n"
          << "    s_u_prev = u_sat;\n"
          << "    return u_sat;\n";
    else
        c << "    s_u_prev = u_raw;\n"
          << "    return u_raw;\n";

    c << "}\n\n"
      << "void controller_reset(void) {\n"
      << "    s_integral = 0.0;\n"
      << "    s_deriv = 0.0;\n"
      << "    s_e_prev = 0.0;\n"
      << "    s_u_prev = 0.0;\n";
    if (cfg.corrector)
        c << "    s_corr = 0.0;\n";
    c << "}\n";

    return GeneratedCode{headerFor(cfg.function_name), c.str()};
}

GeneratedCode generateControllerC(const SMCParams &p, double /*Ts*/, const CodeGenParams &cfg)
{
    std::ostringstream c;
    c << "#include \"controller_gen.h\"\n"
      << "#include <math.h>\n\n"
      << "static const double C_E = " << fmtD(p.c_e) << ";\n"
      << "static const double C_DE = " << fmtD(p.c_de) << ";\n"
      << "static const double K = " << fmtD(p.K) << ";\n"
      << "static const double PHI = " << fmtD(p.phi) << ";\n"
      << "static const double U_MIN = " << fmtD(p.uMin) << ";\n"
      << "static const double U_MAX = " << fmtD(p.uMax) << ";\n\n";

    if (cfg.corrector)
        c << "static const double CORR_U_MIN = " << fmtD(cfg.corrector->uMin) << ";\n"
          << "static const double CORR_U_MAX = " << fmtD(cfg.corrector->uMax) << ";\n"
          << "static const double CORR_KB = " << fmtD(cfg.corrector->Kb) << ";\n"
          << "static double s_corr = 0.0;\n\n";

    c << "static double s_e_prev = 0.0;\n"
      << "static double s_u_prev = 0.0;\n\n"
      << "static double clampd(double v, double lo, double hi) {\n"
      << "    return v < lo ? lo : (v > hi ? hi : v);\n"
      << "}\n\n"
      << "double " << cfg.function_name << "(double error) {\n"
      << "    if (!isfinite(error)) return s_u_prev;\n";

    const std::string input = cfg.corrector ? "e_in" : "error";
    if (cfg.corrector)
        c << "    const double e_in = error + s_corr;\n";

    c << "    const double s = C_E * " << input << " + C_DE * (" << input << " - s_e_prev);\n"
      << "    double sat_val;\n"
      << "    if (PHI > 1e-12) {\n"
      << "        sat_val = s / PHI;\n"
      << "        if (sat_val < -1.0) sat_val = -1.0;\n"
      << "        if (sat_val > 1.0) sat_val = 1.0;\n"
      << "    } else {\n"
      << "        sat_val = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);\n"
      << "    }\n"
      << "    const double u_raw = clampd(-K * sat_val, U_MIN, U_MAX);\n"
      << "    s_e_prev = " << input << ";\n";

    if (cfg.corrector)
        c << "    const double u_sat = clampd(u_raw, CORR_U_MIN, CORR_U_MAX);\n"
          << "    s_corr = CORR_KB * (u_sat - u_raw);\n"
          << "    s_u_prev = u_sat;\n"
          << "    return u_sat;\n";
    else
        c << "    s_u_prev = u_raw;\n"
          << "    return u_raw;\n";

    c << "}\n\n"
      << "void controller_reset(void) {\n"
      << "    s_e_prev = 0.0;\n"
      << "    s_u_prev = 0.0;\n";
    if (cfg.corrector)
        c << "    s_corr = 0.0;\n";
    c << "}\n";

    return GeneratedCode{headerFor(cfg.function_name), c.str()};
}

GeneratedCode generateControllerC(const LeadLagParams &p, double Ts, const CodeGenParams &cfg)
{
    const double two_over_Ts = 2.0 / Ts;
    const double denom = two_over_Ts + p.continuousPole;
    const double b0 = p.gain * (two_over_Ts + p.continuousZero) / denom;
    const double b1 = p.gain * (p.continuousZero - two_over_Ts) / denom;
    const double a1 = (p.continuousPole - two_over_Ts) / denom;

    std::ostringstream c;
    c << "#include \"controller_gen.h\"\n"
      << "#include <math.h>\n\n"
      << "static const double B0 = " << fmtD(b0) << ";\n"
      << "static const double B1 = " << fmtD(b1) << ";\n"
      << "static const double A1 = " << fmtD(a1) << ";\n\n";

    if (cfg.corrector)
        c << "static const double CORR_U_MIN = " << fmtD(cfg.corrector->uMin) << ";\n"
          << "static const double CORR_U_MAX = " << fmtD(cfg.corrector->uMax) << ";\n"
          << "static const double CORR_KB = " << fmtD(cfg.corrector->Kb) << ";\n"
          << "static double s_corr = 0.0;\n\n";

    c << "static double s_u_prev = 0.0;\n"
      << "static double s_y_prev = 0.0;\n\n"
      << "static double clampd(double v, double lo, double hi) {\n"
      << "    return v < lo ? lo : (v > hi ? hi : v);\n"
      << "}\n\n"
      << "double " << cfg.function_name << "(double u_in) {\n"
      << "    if (!isfinite(u_in)) return s_y_prev;\n";

    const std::string input = cfg.corrector ? "e_in" : "u_in";
    if (cfg.corrector)
        c << "    const double e_in = u_in + s_corr;\n";

    c << "    const double y_raw = B0 * " << input << " + B1 * s_u_prev - A1 * s_y_prev;\n"
      << "    s_u_prev = " << input << ";\n";

    if (cfg.corrector)
        c << "    const double y_sat = clampd(y_raw, CORR_U_MIN, CORR_U_MAX);\n"
          << "    s_corr = CORR_KB * (y_sat - y_raw);\n"
          << "    s_y_prev = y_sat;\n"
          << "    return y_sat;\n";
    else
        c << "    s_y_prev = y_raw;\n"
          << "    return y_raw;\n";

    c << "}\n\n"
      << "void controller_reset(void) {\n"
      << "    s_u_prev = 0.0;\n"
      << "    s_y_prev = 0.0;\n";
    if (cfg.corrector)
        c << "    s_corr = 0.0;\n";
    c << "}\n";

    return GeneratedCode{headerFor(cfg.function_name), c.str()};
}

} // namespace ctrl
