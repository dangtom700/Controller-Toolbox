"""
ex101_rl_mpc_stitching.py -- H3: RL-MPC Stitching.

A lightweight DQN-style reinforcement learning policy (< 10k parameters,
numpy only) that adjusts the MPC cost weight rho_y in real time to
improve tracking on a spring-mass-damper plant.

The RL agent:
  - State:  [error, derivative of error]  (2-D)
  - Action: one of {0.5, 2.0, 10.0, 50.0} for rho_y  (4 discrete choices)
  - Network: 2 -> 16 -> 4  (2-layer MLP, tanh hidden, linear output)
  - Training: epsilon-greedy Q-learning (replay buffer, TD-0 targets)

The MPC underneath is DiscreteMPC on a linearised SMD model at the current
operating point, re-tuned each step with the RL-selected rho_y.

Plant (true): SMD  m=1, k=4, c=0.8 (no friction for this example)
"""

import sys
import os
import numpy as np

# -- ctrl_toolbox import guard -------------------------------------------------
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'DiscreteMPC'):
        raise AttributeError("DiscreteMPC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

# =============================================================================
# Plant: linear SMD (no friction, for clean RL training signal)
# =============================================================================
Ts   = 0.1
m, k, c = 1.0, 4.0, 0.8

# Continuous-time matrices  Ac, Bc
Ac = np.array([[0.0, 1.0], [-k/m, -c/m]])
Bc = np.array([[0.0], [1.0/m]])
Cc = np.array([[1.0, 0.0]])
Dc = np.zeros((1, 1))

# ZOH discretisation via ctrl
plant_c = ctrl.StateSpace(Ac, Bc, Cc, Dc, 0.0)
plant   = ctrl.c2d(plant_c, Ts, ctrl.C2dMethod.ZOH)
A, B    = plant.A, plant.B  # numpy matrices after binding

def step_plant(x, u_scalar):
    """One discrete plant step, returns x_next (2-D)."""
    u_vec = np.array([[u_scalar]])
    return (A @ x.reshape(-1, 1) + B * u_scalar).ravel()

# =============================================================================
# RL agent: small 2-layer MLP Q-network  (< 10k params)
# =============================================================================
class QNetwork:
    """Two-layer MLP: 2 -> n_hidden -> n_actions."""

    def __init__(self, n_in=2, n_hidden=16, n_actions=4, seed=0):
        rng = np.random.default_rng(seed)
        scale = 0.3
        self.W1 = rng.normal(0, scale, (n_hidden, n_in))
        self.b1 = np.zeros(n_hidden)
        self.W2 = rng.normal(0, scale, (n_actions, n_hidden))
        self.b2 = np.zeros(n_actions)
        self.n_params = (self.W1.size + self.b1.size +
                         self.W2.size + self.b2.size)

    def forward(self, x):
        h = np.tanh(self.W1 @ x + self.b1)
        return self.W2 @ h + self.b2

    def copy(self):
        q = QNetwork()
        q.W1 = self.W1.copy(); q.b1 = self.b1.copy()
        q.W2 = self.W2.copy(); q.b2 = self.b2.copy()
        return q

    def update_from(self, other, tau=0.05):
        """Soft target update."""
        self.W1 = tau*other.W1 + (1-tau)*self.W1
        self.b1 = tau*other.b1 + (1-tau)*self.b1
        self.W2 = tau*other.W2 + (1-tau)*self.W2
        self.b2 = tau*other.b2 + (1-tau)*self.b2

    def grad_update(self, x, a_idx, target, lr=1e-3):
        """Single-sample TD gradient step (semi-gradient Q-learning)."""
        h     = np.tanh(self.W1 @ x + self.b1)
        q_all = self.W2 @ h + self.b2
        err   = q_all[a_idx] - target   # TD error

        # Backprop through output layer
        dW2 = np.outer(np.eye(len(q_all))[a_idx] * err, h)
        db2 = np.eye(len(q_all))[a_idx] * err

        # Backprop through hidden layer
        dh  = self.W2[a_idx] * err
        dz  = dh * (1.0 - h**2)
        dW1 = np.outer(dz, x)
        db1 = dz

        self.W1 -= lr * dW1
        self.b1 -= lr * db1
        self.W2 -= lr * dW2
        self.b2 -= lr * db2

# =============================================================================
# MPC builder: DiscreteMPC with given rho_y
# =============================================================================
RHO_Y_CHOICES = np.array([0.5, 2.0, 10.0, 50.0])

def make_mpc(rho_y):
    mp       = ctrl.MPCParams()
    mp.Np    = 8; mp.Nc = 3
    mp.rho_y = float(rho_y)
    mp.rho_u = 0.1
    mp.uMin  = -10.0; mp.uMax = 10.0
    return ctrl.DiscreteMPC(plant, mp)

# =============================================================================
# RL-MPC Training
# =============================================================================
print("\n=== H3  RL-MPC Stitching ===")
print(f"  QNetwork params: ", end="")
qnet    = QNetwork(n_in=2, n_hidden=16, n_actions=4, seed=0)
qtarget = qnet.copy()
print(f"{qnet.n_params}  (<10k: {'yes' if qnet.n_params < 10000 else 'no'})")

# Replay buffer (simple list of tuples)
ReplayBuffer = []
BUF_MAX      = 5_000
BATCH        = 32
GAMMA        = 0.90
LR           = 3e-3
TAU          = 0.05

def rl_state(x, x_ref):
    """RL observation: [pos_error, vel]."""
    return np.array([x[0] - x_ref, x[1]])

rng_env  = np.random.default_rng(1)
eps      = 1.0          # exploration rate
eps_min  = 0.05
eps_dec  = 0.995

x_ref    = 1.0          # step reference
episode_len = 50        # steps per episode
n_episodes  = 60        # training episodes

iae_hist     = []
action_hist  = []

for ep in range(n_episodes):
    xp        = np.zeros(2)          # start from rest
    ep_iae    = 0.0
    ep_actions = []
    mpc       = make_mpc(RHO_Y_CHOICES[2])   # start with rho_y=10

    for t in range(episode_len):
        s = rl_state(xp, x_ref)

        # Epsilon-greedy action
        if rng_env.random() < eps:
            a_idx = rng_env.integers(0, 4)
        else:
            q_vals = qnet.forward(s)
            a_idx  = int(np.argmax(q_vals))

        # Rebuild MPC with selected rho_y (lightweight: only MPCParams changes)
        mpc = make_mpc(RHO_Y_CHOICES[a_idx])

        # MPC compute
        x_ref_vec = np.ones(plant.output_size())
        u_vec     = mpc.compute_ref(xp, x_ref_vec)
        u_scalar  = float(np.squeeze(u_vec))

        xp_next   = step_plant(xp, u_scalar)
        ep_iae   += abs(xp[0] - x_ref) * Ts
        ep_actions.append(int(a_idx))

        # Reward: negative absolute error
        reward    = -abs(xp_next[0] - x_ref)
        s_next    = rl_state(xp_next, x_ref)

        # Store transition
        ReplayBuffer.append((s, a_idx, reward, s_next))
        if len(ReplayBuffer) > BUF_MAX:
            ReplayBuffer.pop(0)

        # Mini-batch update
        if len(ReplayBuffer) >= BATCH:
            idx   = rng_env.integers(0, len(ReplayBuffer), size=BATCH)
            for i in idx:
                sb, ab, rb, snb = ReplayBuffer[i]
                q_next = qtarget.forward(snb)
                td_target = rb + GAMMA * np.max(q_next)
                qnet.grad_update(sb, ab, td_target, lr=LR)
            qtarget.update_from(qnet, tau=TAU)

        xp = xp_next

    eps = max(eps_min, eps * eps_dec)
    iae_hist.append(ep_iae)
    action_hist.append(ep_actions)

    if (ep + 1) % 10 == 0:
        print(f"  Episode {ep+1:3d}  IAE={ep_iae:.4f}  eps={eps:.3f}  "
              f"dominant_rho_y={RHO_Y_CHOICES[max(set(ep_actions), key=ep_actions.count)]:.1f}")

# =============================================================================
# Evaluation: greedy RL-MPC vs fixed rho_y
# =============================================================================
print("\n--- Evaluation (greedy RL policy) ---")

def run_fixed(rho_y_val, steps=100):
    mpc = make_mpc(rho_y_val)
    xp  = np.zeros(2)
    iae = 0.0
    for _ in range(steps):
        u_vec = mpc.compute_ref(xp, np.ones(plant.output_size()))
        xp    = step_plant(xp, float(np.squeeze(u_vec)))
        iae  += abs(xp[0] - x_ref) * Ts
    return iae

def run_rl(steps=100):
    xp  = np.zeros(2)
    iae = 0.0
    for _ in range(steps):
        s    = rl_state(xp, x_ref)
        a    = int(np.argmax(qnet.forward(s)))
        mpc_ = make_mpc(RHO_Y_CHOICES[a])
        u_vec= mpc_.compute_ref(xp, np.ones(plant.output_size()))
        xp   = step_plant(xp, float(np.squeeze(u_vec)))
        iae += abs(xp[0] - x_ref) * Ts
    return iae

eval_steps   = 100
iae_rl       = run_rl(eval_steps)
iae_fixed_lo = run_fixed(0.5,  eval_steps)
iae_fixed_hi = run_fixed(50.0, eval_steps)
iae_fixed_med= run_fixed(10.0, eval_steps)

print(f"  RL-MPC IAE:           {iae_rl:.4f}")
print(f"  Fixed rho_y=0.5 IAE:  {iae_fixed_lo:.4f}")
print(f"  Fixed rho_y=10  IAE:  {iae_fixed_med:.4f}")
print(f"  Fixed rho_y=50  IAE:  {iae_fixed_hi:.4f}")
print(f"  Training IAE (last episode): {iae_hist[-1]:.4f}")
print(f"  Training IAE (mean last 10): {np.mean(iae_hist[-10:]):.4f}")

# Sanity checks
assert qnet.n_params < 10_000, "RL policy exceeds 10k param limit"
assert np.isfinite(iae_rl),    "RL-MPC evaluation IAE not finite"
assert iae_hist[-1] < iae_hist[0] * 1.5 or iae_hist[-1] < 0.5, \
    "RL training did not maintain or improve performance"

print("\n[PASS] H3 RL-MPC stitching checks passed.")
