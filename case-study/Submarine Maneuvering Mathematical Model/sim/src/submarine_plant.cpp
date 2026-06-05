#include "submarine_plant.h"
#include <algorithm>
#include <cmath>

namespace sub {

static inline double sat(double x, double lim) {
    return std::max(-lim, std::min(lim, x));
}

SubState step(const SubState& s, double delta_r, double delta_s,
              const PlantParams& p, double dt)
{
    // Saturate control surfaces (+/-35^\circ = +/-0.611 rad)
    const double dr = sat(delta_r, 0.611);
    const double ds = sat(delta_s, 0.611);

    const double A22 = p.A22(), A33 = p.A33();
    const double A55 = p.A55(), A66 = p.A66();
    const double mU  = p.m * p.U;

    // -- Horizontal plane: v, r, ψ ------------------------------------------
    // A22.v. = Yv.v + Yr.r + Yvv.v|v| + Yrr.r|r| + Ydr.deltar + dist_Y - m.U.r
    //          Centripetal correction: -m.U.r  (from Y=m(v.+ru) body-frame kinematics)
    const double Y = p.Yv * s.v
                   + p.Yr * s.r
                   + p.Yvv * s.v * std::abs(s.v)
                   + p.Yrr * s.r * std::abs(s.r)
                   + p.Ydr * dr
                   + p.dist_Y
                   - mU * s.r;

    // A66.ṙ = Nv.v + Nr.r + Nvv.v|v| + Nrr.r|r| + Ndr.deltar
    const double N = p.Nv * s.v
                   + p.Nr * s.r
                   + p.Nvv * s.v * std::abs(s.v)
                   + p.Nrr * s.r * std::abs(s.r)
                   + p.Ndr * dr;

    const double v_dot   = Y / A22;
    const double r_dot   = N / A66;
    const double psi_dot = s.r;

    // -- Vertical plane: w, q, theta, z -----------------------------------------
    // A33.ẇ = Zw.w + Zq.q + Zww.w|w| + Zds.deltas + m.U.q
    //          Centripetal correction: +m.U.q  (from Z=m(ẇ-qu) body-frame)
    const double Z = p.Zw * s.w
                   + p.Zq * s.q
                   + p.Zww * s.w * std::abs(s.w)
                   + p.Zds * ds
                   + mU * s.q;

    // A55.q. = Mw.w + Mq.q + Mqq.q|q| + Mds.deltas - m.g.BG.sin(theta)
    const double M = p.Mw * s.w
                   + p.Mq * s.q
                   + p.Mqq * s.q * std::abs(s.q)
                   + p.Mds * ds
                   - p.m * p.g * p.BG * std::sin(s.tht);

    const double w_dot   = Z / A33;
    const double q_dot   = M / A55;
    const double tht_dot = s.q;
    const double z_dot   = p.U * std::sin(s.tht) - s.w * std::cos(s.tht);

    // -- Earth-frame position (2-D: ignoring heave component in x,y) ---------
    const double x_dot = p.U * std::cos(s.psi) - s.v * std::sin(s.psi);
    const double y_dot = p.U * std::sin(s.psi) + s.v * std::cos(s.psi);

    // -- Euler forward integration -------------------------------------------
    SubState ns = s;
    ns.v   += v_dot   * dt;
    ns.r   += r_dot   * dt;
    ns.psi += psi_dot * dt;
    ns.w   += w_dot   * dt;
    ns.q   += q_dot   * dt;
    ns.tht += tht_dot * dt;
    ns.z   += z_dot   * dt;
    ns.x   += x_dot   * dt;
    ns.y   += y_dot   * dt;

    return ns;
}

} // namespace sub
