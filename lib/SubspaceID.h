#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>
#include <optional>

// Subspace State-Space System Identification (N4SID / MOESP).
//
// Identifies a linear discrete-time state-space model from batch I/O data:
//   x[k+1] = A.x[k] + B.u[k] + w[k]
//   y[k]   = C.x[k] + D.u[k] + v[k]
//
// Algorithm — MOESP oblique-projection variant (Verhaegen & Dewilde 1992):
//
//  1. Build 2i-block Hankel matrices from (Y, U):
//       Z = [U_f; W_p; Y_f]   where W_p = [U_p; Y_p]
//
//  2. Thin LQ decomposition of Z (QR of Z') isolates the (3,2) block L32
//     which contains the oblique projection of Y_f onto W_p ⊥ U_f.
//
//  3. SVD(L32): singular values reveal system order; left singular vectors
//     form the extended observability matrix Γ = U[:, :n].sqrt(S[:n]).
//
//  4. Extract  C = Γ[:p, :]
//              A = Γ_up† . Γ_down   (shift-invariance, least squares)
//
//  5. Least-squares regression for B and D from the state sequence and data.
//
// Data requirements:
//   - Persistent excitation: the input must excite at least n_order frequencies.
//   - N >> 2*i*(m+p): more samples → better estimates.
//   - i >= n_order/p (i_horizon must be large enough to capture all modes).
//
// Output singularValues can be plotted to choose n_order: look for an elbow
// where singular values drop sharply (signal vs. noise subspace boundary).
//
// Ref: Van Overschee & De Moor "Subspace Identification for Linear Systems"
//      (Kluwer 1996); Verhaegen "Identification of the Deterministic Part of
//      MIMO State Space Models" Automatica (1994).
namespace ctrl
{

struct SubspaceIDResult
{
    std::optional<StateSpace> model;  // identified A,B,C,D,Ts (empty on failure)
    Eigen::VectorXd  singularValues;  // all singular values of L32 (for order selection)
    bool             success = false;
    std::string      message;         // error description on failure
};

// n4sid: batch subspace identification.
//
// Y:          output matrix (p × N)  — rows = outputs, cols = time samples
// U:          input  matrix (m × N)  — rows = inputs,  cols = time samples
// n_order:    desired state order n  (choose by inspecting result.singularValues,
//             or use suggestOrder() below)
// i_horizon:  number of block rows in Hankel matrices (recommend i >= 2*n_order/p,
//             minimum i = n_order + 1)
// Ts:         sample time [s]
//
// Returns a SubspaceIDResult. On failure, result.success = false and result.message
// describes the cause.
SubspaceIDResult n4sid(const Eigen::MatrixXd &Y,
                       const Eigen::MatrixXd &U,
                       int n_order,
                       int i_horizon,
                       double Ts);

// suggestOrder: automated system-order selection from the singular-value spectrum.
//
// Uses the elbow (maximum consecutive ratio) heuristic: finds the index i* at which
// sv(i*) / sv(i*+1) is maximised (the sharpest drop in the spectrum) and returns
// i* + 1 as the suggested order.  A secondary threshold guard caps the result at
// the first index where sv(i)/sv(0) falls below `threshold` (default 1 %).
//
// sv:        SubspaceIDResult::singularValues from a previous n4sid call.
// threshold: relative floor; singular values below threshold*sv(0) are treated as
//            noise.  Set to 0 to disable the threshold guard.
// maxOrder:  hard cap on the returned order (-1 = no cap).
//
// Returns at least 1.  Call after n4sid() to eliminate the manual "elbow inspection"
// step when building adaptive loops (e.g., RLS → n4sid → GPC).
int suggestOrder(const Eigen::VectorXd &sv,
                 double threshold = 0.01,
                 int maxOrder     = -1);

} // namespace ctrl
