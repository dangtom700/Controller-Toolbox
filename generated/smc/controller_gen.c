#include "controller_gen.h"
#include <math.h>

static const double C_E = 1.0;
static const double C_DE = 0.050000000000000003;
static const double K = 3.0;
static const double PHI = 0.20000000000000001;
static const double U_MIN = -10.0;
static const double U_MAX = 10.0;

static double s_e_prev = 0.0;
static double s_u_prev = 0.0;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

double controller_step(double error) {
    if (!isfinite(error)) return s_u_prev;
    const double s = C_E * error + C_DE * (error - s_e_prev);
    double sat_val;
    if (PHI > 1e-12) {
        sat_val = s / PHI;
        if (sat_val < -1.0) sat_val = -1.0;
        if (sat_val > 1.0) sat_val = 1.0;
    } else {
        sat_val = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);
    }
    const double u_raw = clampd(-K * sat_val, U_MIN, U_MAX);
    s_e_prev = error;
    s_u_prev = u_raw;
    return u_raw;
}

void controller_reset(void) {
    s_e_prev = 0.0;
    s_u_prev = 0.0;
}
