#include "controller_gen.h"
#include <math.h>

static const double B0 = 1.9142857142857144;
static const double B1 = -1.8952380952380952;
static const double A1 = -0.90476190476190477;

static double s_u_prev = 0.0;
static double s_y_prev = 0.0;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

double controller_step(double u_in) {
    if (!isfinite(u_in)) return s_y_prev;
    const double y_raw = B0 * u_in + B1 * s_u_prev - A1 * s_y_prev;
    s_u_prev = u_in;
    s_y_prev = y_raw;
    return y_raw;
}

void controller_reset(void) {
    s_u_prev = 0.0;
    s_y_prev = 0.0;
}
