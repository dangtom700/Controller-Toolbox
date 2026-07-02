#include "controller_gen.h"
#include <math.h>

static const double KP = 2.0;
static const double KI = 0.5;
static const double KD = 0.10000000000000001;
static const double ND = 100.0;
static const double KB = 1.0;
static const double U_MIN = -5.0;
static const double U_MAX = 5.0;
static const double TS = 0.01;

static double s_integral = 0.0;
static double s_deriv = 0.0;
static double s_e_prev = 0.0;
static double s_u_prev = 0.0;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

double controller_step(double error) {
    if (!isfinite(error)) return s_u_prev;
    const double alpha = 1.0 / (1.0 + ND * TS);
    const double d_new = alpha * s_deriv + KD * ND * alpha * (error - s_e_prev);
    const double ki_update = KI * TS * error;
    const double u_unsat = KP * error + (s_integral + ki_update) + d_new;
    const double u_raw = clampd(u_unsat, U_MIN, U_MAX);
    s_integral += ki_update + KB * (u_raw - u_unsat);
    s_deriv = d_new;
    s_e_prev = error;
    s_u_prev = u_raw;
    return u_raw;
}

void controller_reset(void) {
    s_integral = 0.0;
    s_deriv = 0.0;
    s_e_prev = 0.0;
    s_u_prev = 0.0;
}
