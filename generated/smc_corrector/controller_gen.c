#include "controller_gen.h"
#include <math.h>

static const double C_E = 1.0;
static const double C_DE = 0.050000000000000003;
static const double K = 3.0;
static const double PHI = 0.20000000000000001;
static const double U_MIN = -10.0;
static const double U_MAX = 10.0;

static const double CORR_U_MIN = -4.0;
static const double CORR_U_MAX = 4.0;
static const double CORR_KB = 0.80000000000000004;
static double s_corr = 0.0;

static double s_e_prev = 0.0;
static double s_u_prev = 0.0;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

double controller_step(double error) {
    if (!isfinite(error)) return s_u_prev;
    const double e_in = error + s_corr;
    const double s = C_E * e_in + C_DE * (e_in - s_e_prev);
    double sat_val;
    if (PHI > 1e-12) {
        sat_val = s / PHI;
        if (sat_val < -1.0) sat_val = -1.0;
        if (sat_val > 1.0) sat_val = 1.0;
    } else {
        sat_val = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);
    }
    const double u_raw = clampd(-K * sat_val, U_MIN, U_MAX);
    s_e_prev = e_in;
    const double u_sat = clampd(u_raw, CORR_U_MIN, CORR_U_MAX);
    s_corr = CORR_KB * (u_sat - u_raw);
    s_u_prev = u_sat;
    return u_sat;
}

void controller_reset(void) {
    s_e_prev = 0.0;
    s_u_prev = 0.0;
    s_corr = 0.0;
}
