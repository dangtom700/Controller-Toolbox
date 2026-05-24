%% PV_Evaporative_Chimney_Model.m
% Implementation of the PV evaporative chimney cooling system model.
% Reference: Ruiz et al., Appl. Therm. Eng. 245 (2024) 122878
%
% Three control strategies are compared over a simulated summer day (08:00-20:00):
%   1. Baseline  — fixed Q = 1.05 m³/h (design-point optimal, Table 4)
%   2. PID       — tracks instantaneous optimal Q (rate-limited, table-assisted)
%   3. MPC       — minimises W_grid over 30-min horizon via fmincon + lookup table
%
% NOTE: pump peak efficiency η_pump0 = 3.4 in Table 1 of the paper is
%       physically impossible (η > 1). Value corrected to 0.34 here.
%
% Usage: open in MATLAB and press F5, or type PV_Evaporative_Chimney_Model

clear; clc; close all;

%% ═══════════════════════════════════════════════════════════════════════════
%% 1.  SYSTEM PARAMETERS
%% ═══════════════════════════════════════════════════════════════════════════
p = struct();

% PV electrical — Eq. (8)
p.eta_ref  = 0.157;
p.beta_ref = 0.0044;        % °C⁻¹
p.T_ref    = 25;            % °C  (STC)
p.G_ref    = 1000;          % W/m²
p.tau      = 0.9;           % glass transmissivity

% PV layer thermal properties for Eqs. (1)–(4)  [typical crystalline-Si]
p.k_g = 1.8;   p.x_g = 0.003;    % glass  [W/mK, m]
p.k_c = 148;   p.x_c = 3.0e-4;   % Si cell
p.k_t = 0.2;   p.x_t = 5.0e-4;   % tedlar backing

% Total convective (dry) PV area — calibration parameter
p.A_PV = 8.0;               % m²

% Chiller EER quadratic — Eq. (20), Table 2
%  EER = a + b·T_evap + c·T_evap² + d·T_cond + e·T_cond² + f·T_evap·T_cond
p.EER_coef = [12.15, -1.826, 0.2108, 0.1166, 0.004213, -0.05686];

% Chiller design point
p.Q_evap = 3800;            % W
p.T_evap = 7.0;             % °C  (chilled-water supply)

% Hydraulic loop — Eqs. (22)–(27)
p.H0     = 45.05;           % pump shutoff head [m]
p.Q0_ls  = 1.308;           % pump nominal flow [l/s]
p.eta_p0 = 0.34;            % pump peak efficiency [-]  (paper typo: 3.4 → 0.34)
p.rho_w  = 1000;            % kg/m³
p.g_acc  = 9.81;            % m/s²

% Psychrometric constants
p.p_atm  = 101325;          % Pa
p.Le     = 0.865;           % Lewis factor (air–water)
p.c_pa   = 1006;            % J/(kg·K)  dry air
p.c_pv   = 1805;            % J/(kg·K)  water vapour
p.c_pw   = 4186;            % J/(kg·K)  liquid water
p.h_fg0  = 2.501e6;         % J/kg  latent heat at 0 °C

% Flow limits for optimisation
p.Q_min_ls = 0.20;
p.Q_max_ls = p.Q0_ls;

%% ═══════════════════════════════════════════════════════════════════════════
%% 2.  SUMMER-DAY AMBIENT PROFILES  (08:00–20:00, Δt = 1 min)
%% ═══════════════════════════════════════════════════════════════════════════
t_start = 8;  t_end = 20;  dt_h = 1/60;
t_h = (t_start : dt_h : t_end)';
N   = length(t_h);

f = sin(pi*(t_h - t_start)/(t_end - t_start));   % 0→1→0

T_amb_v = 28 + 12*f;              % °C   dawn 28 → noon 40
phi_v   = 0.55 - 0.25*f;         % [-]  55% → 30%  (anti-correlated)
G_v     = 950*f;                  % W/m² sinusoidal solar
vw_v    = 3.0*ones(N,1);          % m/s  constant

%% ═══════════════════════════════════════════════════════════════════════════
%% 3.  DESIGN-POINT VERIFICATION (Table 4)
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('=== Design-Point Verification (Table 4) ===\n');
amb_dp = mk_amb(35.6, 0.35, 920.17, 3.028);
Q_dp   = 1.05/3.6;               % m³/h → l/s

r_dp = plant_model(Q_dp, amb_dp, p);
fprintf('  Q_total    = %.4f l/s  (%.3f m³/h)\n', Q_dp, Q_dp*3.6);
fprintf('  T_w1 (hot) = %.2f °C\n', r_dp.T_w1);
fprintf('  T_w2 (cool)= %.2f °C   [condenser inlet]\n', r_dp.T_w2);
fprintf('  T_ai       = %.2f °C   [air leaving evap area]\n', r_dp.T_ai);
fprintf('  EER_ch     = %.3f\n', r_dp.EER_ch);
fprintf('  W_comp     = %.1f W\n', r_dp.W_comp);
fprintf('  W_pump     = %.1f W\n', r_dp.W_pump);
fprintf('  W_PV       = %.1f W\n', r_dp.W_PV);
fprintf('  W_grid     = %.1f W     [Eq. 28]\n', r_dp.W_grid);
fprintf('  EER_grid   = %.3f\n\n', r_dp.EER_grid);

%% ═══════════════════════════════════════════════════════════════════════════
%% 4.  LOOKUP TABLE  (T_amb × φ × G × Q)
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('Building 4-D lookup table ...\n');

LT.Tamb = [25, 30, 35, 40];
LT.phi  = [0.25, 0.40, 0.55, 0.70];
LT.G    = [50, 300, 600, 900];
LT.Q    = linspace(p.Q_min_ls, p.Q_max_ls, 12);

[nT,np,nG,nQ] = deal(numel(LT.Tamb),numel(LT.phi),numel(LT.G),numel(LT.Q));
LT.Wg = NaN(nT,np,nG,nQ);

for iT = 1:nT
  for ip = 1:np
    for iG = 1:nG
      al = mk_amb(LT.Tamb(iT),LT.phi(ip),LT.G(iG),3.0);
      for iQ = 1:nQ
        try
          rt = plant_model(LT.Q(iQ), al, p);
          LT.Wg(iT,ip,iG,iQ) = rt.W_grid;
        catch
        end
      end
    end
  end
end

% Replace NaN entries with column-wise max (large penalty, never optimal)
mx = max(LT.Wg(:),[],'omitnan');
LT.Wg(isnan(LT.Wg)) = mx;
fprintf('  Done (%dx%dx%dx%d).\n\n', nT,np,nG,nQ);

%% ═══════════════════════════════════════════════════════════════════════════
%% 5.  BASELINE SIMULATION
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('Running Baseline ...\n');
BL = sim_baseline(1.05/3.6, T_amb_v, phi_v, G_v, vw_v, p);
fprintf('  E_grid=%.1f Wh  meanEER_grid=%.2f\n\n', ...
    sum(BL.W_grid)*dt_h, mean(BL.EER_grid(G_v>50)));

%% ═══════════════════════════════════════════════════════════════════════════
%% 6.  PID SIMULATION
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('Running PID ...\n');
pid = struct('Kp',0.8,'Ki',0.015,'Kd',0,'dt_s',dt_h*3600, ...
             'dQ_max',0.04,'Q_min',p.Q_min_ls,'Q_max',p.Q_max_ls);
PD = sim_pid(T_amb_v, phi_v, G_v, vw_v, p, LT, pid);
fprintf('  E_grid=%.1f Wh  meanEER_grid=%.2f\n\n', ...
    sum(PD.W_grid)*dt_h, mean(PD.EER_grid(G_v>50)));

%% ═══════════════════════════════════════════════════════════════════════════
%% 7.  MPC SIMULATION
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('Running MPC ...\n');
mpc = struct('Np',30,'Nc',10,'dQ_max',0.04, ...
             'Q_min',p.Q_min_ls,'Q_max',p.Q_max_ls);
MP = sim_mpc(T_amb_v, phi_v, G_v, vw_v, p, LT, mpc);
fprintf('  E_grid=%.1f Wh  meanEER_grid=%.2f\n\n', ...
    sum(MP.W_grid)*dt_h, mean(MP.EER_grid(G_v>50)));

%% ═══════════════════════════════════════════════════════════════════════════
%% 8.  CONSOLE SUMMARY
%% ═══════════════════════════════════════════════════════════════════════════
E_comp = [sum(BL.W_comp), sum(PD.W_comp), sum(MP.W_comp)]*dt_h;
E_pump = [sum(BL.W_pump), sum(PD.W_pump), sum(MP.W_pump)]*dt_h;
E_PV   = [sum(BL.W_PV),  sum(PD.W_PV),  sum(MP.W_PV) ]*dt_h;
E_grid = [sum(BL.W_grid), sum(PD.W_grid), sum(MP.W_grid)]*dt_h;
EER_av = [mean(BL.EER_grid(G_v>50)), mean(PD.EER_grid(G_v>50)), ...
          mean(MP.EER_grid(G_v>50))];

fprintf('┌──────────────────────────────────────────────────────┐\n');
fprintf('│             DAILY PERFORMANCE SUMMARY                │\n');
fprintf('├──────────────────────┬─────────┬─────────┬──────────┤\n');
fprintf('│ Metric               │Baseline │   PID   │   MPC    │\n');
fprintf('├──────────────────────┼─────────┼─────────┼──────────┤\n');
fprintf('│ E_comp   [Wh]        │%8.1f │%8.1f │%9.1f │\n', E_comp);
fprintf('│ E_pump   [Wh]        │%8.1f │%8.1f │%9.1f │\n', E_pump);
fprintf('│ E_PV gen [Wh]        │%8.1f │%8.1f │%9.1f │\n', E_PV);
fprintf('│ E_grid   [Wh]        │%8.1f │%8.1f │%9.1f │\n', E_grid);
fprintf('│ Mean EER_grid        │%8.2f │%8.2f │%9.2f │\n', EER_av);
fprintf('├──────────────────────┴─────────┴─────────┴──────────┤\n');
fprintf('│ PID vs Baseline: %+7.1f Wh  (%+5.1f%%)              │\n', ...
    E_grid(2)-E_grid(1), 100*(E_grid(2)-E_grid(1))/E_grid(1));
fprintf('│ MPC vs Baseline: %+7.1f Wh  (%+5.1f%%)              │\n', ...
    E_grid(3)-E_grid(1), 100*(E_grid(3)-E_grid(1))/E_grid(1));
fprintf('│ MPC vs PID:      %+7.1f Wh  (%+5.1f%%)              │\n', ...
    E_grid(3)-E_grid(2), 100*(E_grid(3)-E_grid(2))/E_grid(2));
fprintf('└──────────────────────────────────────────────────────┘\n');

%% ═══════════════════════════════════════════════════════════════════════════
%% 9.  FIGURES
%% ═══════════════════════════════════════════════════════════════════════════
strats = {BL, PD, MP};  names = {'Baseline','PID','MPC'};
cols   = {'b','r',[0 0.6 0]};  lw = 1.5;

% ── Figure 1: time-series ────────────────────────────────────────────────────
figure(1); clf;
set(gcf,'Position',[30 30 1380 820]);

sp = @(r,c) subplot(3,4,(r-1)*4+c);

% Row 1: ambient
sp(1,1); plot(t_h,T_amb_v,'k','LineWidth',lw); grid on;
xlabel('h'); ylabel('°C'); title('T_{amb}');

sp(1,2); plot(t_h,G_v,'Color',[0.8 0.4 0],'LineWidth',lw); grid on;
xlabel('h'); ylabel('W/m²'); title('Irradiance G');

sp(1,3); plot(t_h,phi_v*100,'Color',[0 0.5 0.8],'LineWidth',lw); grid on;
xlabel('h'); ylabel('%'); title('Rel. humidity φ');

sp(1,4); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.Q_ls*3.6,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('m³/h'); title('Pump flow Q');
legend(names,'Location','best');

% Row 2: temperatures
sp(2,1); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.T_w1,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('°C'); title('T_{w1} chimney in (hot)');
legend(names,'Location','best');

sp(2,2); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.T_w2,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('°C'); title('T_{w2} cond in (cool)');
legend(names,'Location','best');

sp(2,3); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.T_ai,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('°C'); title('T_{ai} air exit');
legend(names,'Location','best');

sp(2,4); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.EER_ch,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('-'); title('EER_{chiller}');
legend(names,'Location','best');

% Row 3: power
sp(3,1); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.W_comp,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('W'); title('W_{comp}');
legend(names,'Location','best');

sp(3,2); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.W_PV,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('W'); title('W_{PV} (generated)');
legend(names,'Location','best');

sp(3,3); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.W_grid,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('W'); title('W_{grid} [Eq. 28]');
legend(names,'Location','best');

sp(3,4); hold on; grid on;
for k=1:3; plot(t_h,strats{k}.EER_grid,'Color',cols{k},'LineWidth',lw); end
xlabel('h'); ylabel('-'); title('EER_{grid}');
legend(names,'Location','best');

sgtitle('PV Evaporative Chimney — Control Strategy Comparison','FontSize',14);

% ── Figure 2: bar chart ──────────────────────────────────────────────────────
figure(2); clf;
set(gcf,'Position',[100 100 760 420]);

subplot(1,2,1);
bm = [E_comp; E_pump; E_PV]';
b = bar(bm,'stacked');
b(1).FaceColor = [0.85 0.2 0.2];
b(2).FaceColor = [0.2 0.4 0.85];
b(3).FaceColor = [0.2 0.75 0.3];
hold on;
plot(1:3, E_grid,'k^--','LineWidth',2,'MarkerSize',8,'DisplayName','Net W_{grid}');
set(gca,'XTickLabel',names); ylabel('Wh'); title('Daily Energy Breakdown');
legend({'W_{comp}','W_{pump}','W_{PV}','Net W_{grid}'},'Location','best');
grid on;

subplot(1,2,2);
bh = bar(EER_av,0.5);
bh.FaceColor = 'flat';
bh.CData = [0 0 1; 1 0 0; 0 0.6 0];
set(gca,'XTickLabel',names);
ylabel('Mean EER_{grid}  (G>50 W/m²)');
title('Mean Grid EER');  grid on;
for k=1:3
    text(k, EER_av(k)+0.02*max(EER_av), sprintf('%.2f',EER_av(k)), ...
        'HorizontalAlignment','center','FontSize',11,'FontWeight','bold');
end

sgtitle('PV Evaporative Chimney — Daily Summary','FontSize',13);
fprintf('\nDone.  See Figures 1 and 2.\n');


%% ═══════════════════════════════════════════════════════════════════════════
%%                          LOCAL  FUNCTIONS
%% ═══════════════════════════════════════════════════════════════════════════

% ────────────────────────────────────────────────────────────────────────────
%  SIMULATION WRAPPERS
% ────────────────────────────────────────────────────────────────────────────

function S = sim_baseline(Q_ls, Ta, phi, G, vw, p)
    N = numel(Ta);  S = init_S(N);
    for k = 1:N
        S = eval_step(S, k, Q_ls, mk_amb(Ta(k),phi(k),G(k),vw(k)), p);
    end
end

function S = sim_pid(Ta, phi, G, vw, p, LT, cfg)
    N = numel(Ta);  S = init_S(N);
    Q  = (cfg.Q_min + cfg.Q_max)/2;
    eI = 0;  ePrev = 0;

    for k = 1:N
        Q_opt = opt_Q_LT(Ta(k), phi(k), G(k), LT);
        err   = Q_opt - Q;
        eI    = eI + err*cfg.dt_s;
        eD    = (err - ePrev)/cfg.dt_s;  ePrev = err;
        dQ    = cfg.Kp*err + cfg.Ki*eI + cfg.Kd*eD;
        dQ    = max(-cfg.dQ_max, min(cfg.dQ_max, dQ));
        Q     = max(cfg.Q_min,   min(cfg.Q_max,  Q+dQ));
        S = eval_step(S, k, Q, mk_amb(Ta(k),phi(k),G(k),vw(k)), p);
    end
end

function S = sim_mpc(Ta, phi, G, vw, p, LT, cfg)
    N = numel(Ta);  S = init_S(N);
    Q  = (cfg.Q_min + cfg.Q_max)/2;

    fo = optimoptions('fmincon','Display','off','MaxIterations',60, ...
                      'Algorithm','sqp','OptimalityTolerance',1e-4, ...
                      'StepTolerance',1e-5);

    for k = 1:N
        Np = min(cfg.Np, N-k+1);
        Nc = min(cfg.Nc, Np);

        Ta_h = Ta(k:k+Np-1);  ph_h = phi(k:k+Np-1);  G_h = G(k:k+Np-1);

        lb = -cfg.dQ_max*ones(Nc,1);
        ub =  cfg.dQ_max*ones(Nc,1);
        x0 = zeros(Nc,1);

        cost = @(dQ) mpc_cost(dQ, Q, Ta_h, ph_h, G_h, LT, ...
                               cfg.Q_min, cfg.Q_max, Np, Nc);
        dQ_opt = fmincon(cost, x0,[],[],[],[],lb,ub,[],fo);

        Q = max(cfg.Q_min, min(cfg.Q_max, Q + dQ_opt(1)));
        S = eval_step(S, k, Q, mk_amb(Ta(k),phi(k),G(k),vw(k)), p);
    end
end

% ────────────────────────────────────────────────────────────────────────────
%  MPC COST
% ────────────────────────────────────────────────────────────────────────────

function J = mpc_cost(dQ, Q0, Ta_h, phi_h, G_h, LT, Qmin, Qmax, Np, Nc)
    J = 0;  Q = Q0;
    for i = 1:Np
        if i <= Nc
            Q = max(Qmin, min(Qmax, Q + dQ(i)));
        end
        J = J + interp_LT(LT, Ta_h(i), phi_h(i), G_h(i), Q);
    end
end

% ────────────────────────────────────────────────────────────────────────────
%  LOOKUP TABLE HELPERS
% ────────────────────────────────────────────────────────────────────────────

function Q_opt = opt_Q_LT(T_amb, phi, G, LT)
    Wg = zeros(size(LT.Q));
    for i = 1:numel(LT.Q)
        Wg(i) = interp_LT(LT, T_amb, phi, G, LT.Q(i));
    end
    [~,idx] = min(Wg);
    Q_opt = LT.Q(idx);
end

function Wg = interp_LT(LT, T_amb, phi, G, Q_ls)
    Tc = max(LT.Tamb(1), min(LT.Tamb(end), T_amb));
    pc = max(LT.phi(1),  min(LT.phi(end),  phi));
    Gc = max(LT.G(1),    min(LT.G(end),    G));
    Qc = max(LT.Q(1),    min(LT.Q(end),    Q_ls));
    Wg = interpn(LT.Tamb, LT.phi, LT.G, LT.Q, LT.Wg, Tc, pc, Gc, Qc, 'linear');
    if ~isfinite(Wg), Wg = 1e6; end
end

% ────────────────────────────────────────────────────────────────────────────
%  PLANT MODEL  (Steps 1-2-3 from paper Fig. 3)
% ────────────────────────────────────────────────────────────────────────────

function r = plant_model(Q_ls, amb, p)
    % Step 1: evaporative chimney + chiller coupling
    [T_w1, T_w2, T_ai, m_dot_a] = solve_chimney(Q_ls, amb, p);

    % Chiller
    EER_ch = eval_EER(p.T_evap, T_w2, p.EER_coef);
    W_comp = p.Q_evap / EER_ch;

    % Step 2: PV convective panel
    m_dot_w = Q_ls * p.rho_w / 1000;   % kg/s
    W_PV = pv_panel(amb, T_ai, m_dot_w, p);

    % Step 3: hydraulic loop
    W_pump = hydraulic(Q_ls, p);

    % Eq. (28)
    W_grid   = W_comp + W_pump - W_PV;
    EER_grid = p.Q_evap / max(W_grid, 1);

    r = struct('T_w1',T_w1,'T_w2',T_w2,'T_ai',T_ai,'m_dot_a',m_dot_a, ...
               'EER_ch',EER_ch,'W_comp',W_comp,'W_PV',W_PV, ...
               'W_pump',W_pump,'W_grid',W_grid,'EER_grid',EER_grid);
end

% ────────────────────────────────────────────────────────────────────────────
%  CHIMNEY SOLVER  (1-D root-find on T_w2)
% ────────────────────────────────────────────────────────────────────────────

function [T_w1, T_w2, T_ai, m_dot_a] = solve_chimney(Q_ls, amb, p)
    m_dot_w = Q_ls * p.rho_w / 1000;

    % Air mass flow — Eq. (18)
    m_dot_a = max(-6.1751*m_dot_w^2 + 2.2469*m_dot_w - 0.07653, 0.005);

    % Merkel number from prototype correlation — Eq. (17)
    Me_corr = 0.7099 * (m_dot_w / m_dot_a)^(-0.3254);

    % Bracket: T_wb (thermodynamic floor) to T_amb+15 (no cooling limit)
    T_wb = wet_bulb_approx(amb.T_amb, amb.phi_amb);
    T_lo = max(T_wb - 2, 5);
    T_hi = amb.T_amb + 20;

    res = @(Tw2) chim_res(Tw2, m_dot_w, m_dot_a, Me_corr, amb, p);
    rlo = res(T_lo);  rhi = res(T_hi);

    % Widen bracket if needed
    if rlo*rhi > 0
        for ext = [5 10 15]
            T_lo2 = max(5, T_lo-ext);  T_hi2 = T_hi+ext;
            if res(T_lo2)*res(T_hi2) < 0
                T_lo = T_lo2;  T_hi = T_hi2;  break;
            end
        end
    end

    try
        if res(T_lo)*res(T_hi) < 0
            T_w2 = fzero(res, [T_lo, T_hi], optimset('TolX',1e-3,'Display','off'));
        else
            T_w2 = (T_lo + T_hi)/2;
        end
    catch
        T_w2 = (T_lo + T_hi)/2;
    end

    % Recover T_w1 and air exit temperature
    EER_ch = eval_EER(p.T_evap, T_w2, p.EER_coef);
    Q_cond = p.Q_evap * (1 + 1/EER_ch);
    T_w1   = T_w2 + Q_cond / (m_dot_w * p.c_pw);
    [~, ~, T_ai] = integrate_poppe(T_w1, T_w2, m_dot_w, m_dot_a, amb, p);
end

function r = chim_res(T_w2, m_dot_w, m_dot_a, Me_corr, amb, p)
    EER_ch = eval_EER(p.T_evap, T_w2, p.EER_coef);
    Q_cond = p.Q_evap * (1 + 1/EER_ch);
    T_w1   = T_w2 + Q_cond / (m_dot_w * p.c_pw);
    Me_pp  = integrate_poppe(T_w1, T_w2, m_dot_w, m_dot_a, amb, p);
    r = Me_pp - Me_corr;
end

% ────────────────────────────────────────────────────────────────────────────
%  POPPE ODE INTEGRATOR — Eqs. (14)–(16)
% ────────────────────────────────────────────────────────────────────────────

function [Me, omega_out, T_ai] = integrate_poppe(T_w1, T_w2, m_dot_w, m_dot_a, amb, p)
    if abs(T_w1 - T_w2) < 0.05
        Me = 0;  omega_out = omega_from_phi(amb.T_amb, amb.phi_amb, p);
        T_ai = amb.T_amb;  return;
    end

    omega0 = omega_from_phi(amb.T_amb, amb.phi_amb, p);
    h0     = h_moist(amb.T_amb, omega0, p);
    y0     = [omega0; h0; 0];

    mra = m_dot_w/m_dot_a;
    Le  = p.Le;   pat = p.p_atm;
    cpw = p.c_pw; cpv = p.c_pv;
    cpa = p.c_pa; hfg = p.h_fg0;

    rhs = @(Tw,y) poppe_rhs(Tw,y,mra,Le,pat,cpw,cpv,cpa,hfg);

    opts = odeset('RelTol',1e-5,'AbsTol',1e-7,'MaxStep',0.5);
    try
        [~,Y] = ode45(rhs, [T_w1, T_w2], y0, opts);
        Me        = abs(Y(end,3));
        omega_out = max(Y(end,1), 0);
        h_out     = Y(end,2);
        T_ai      = (h_out - omega_out*hfg) / (cpa + omega_out*cpv);
    catch
        Me = 1e6;  omega_out = omega0;  T_ai = amb.T_amb;
    end
end

function dydt = poppe_rhs(Tw, y, mra, Le, pat, cpw, cpv, cpa, hfg)
    omega = y(1);  h_a = y(2);

    % Saturated conditions at water temperature
    ps    = sat_p(Tw);
    om_sw = 0.622*ps/(pat - ps);
    h_sw  = cpa*Tw + om_sw*(hfg + cpv*Tw);

    h_v = hfg + cpv*Tw;     % vapour enthalpy
    h_w = cpw*Tw;            % liquid water enthalpy

    % Common denominator — Eqs. (14)–(16)
    Dom = om_sw - omega;
    Dh  = h_sw - h_a;
    D   = Dh + (Le-1)*(Dh - Dom*h_v) - Dom*h_w;
    D   = sign(D+1e-9)*max(abs(D), 5);   % prevent singularity

    dydt = [cpw*mra*Dom/D;
            cpw*mra*(1 + Dom*cpw*Tw/D);
            cpw/D];
end

% ────────────────────────────────────────────────────────────────────────────
%  PV CONVECTIVE PANEL — Eqs. (1)–(4), (8)
% ────────────────────────────────────────────────────────────────────────────

function W_PV = pv_panel(amb, T_i, m_dot_w, p)
    if amb.G < 1, W_PV = 0; return; end

    h_e = 0.841*amb.v_w + 4.61;                               % Eq. (5)
    v_i = max(-207.325*m_dot_w^2 + 67.978*m_dot_w - 2.38, 0.05); % Eq. (7)
    h_i = 1.97*v_i + 10;                                      % Eq. (6)

    Ug = p.k_g/p.x_g;  Uc = p.k_c/p.x_c;  Ut = p.k_t/p.x_t;
    et = p.eta_ref;     br = p.beta_ref;    Tr = p.T_ref;

    % 4×4 linear system for [T_g, T_c, T_t, T_r] — Eqs. (1)–(4)+(8)
    A = [h_e+Ug,  -Ug,            0,          0;
         -Ug,      Ug+Uc-amb.G*et*br, -Uc,    0;
          0,        Uc,          -(Uc+Ut),    Ut;
          0,        0,            -Ut,         h_i+Ut];

    b = [h_e*amb.T_amb;
         amb.G*p.tau - amb.G*et*(1+br*Tr);
         0;
         h_i*T_i];

    T_vec = A \ b;
    T_c   = T_vec(2);
    eta   = et*(1 - br*(T_c - Tr));          % Eq. (8)
    W_PV  = max(eta*amb.G*p.A_PV, 0);        % Eq. (12)
end

% ────────────────────────────────────────────────────────────────────────────
%  HYDRAULIC LOOP — Eqs. (22)–(27)
% ────────────────────────────────────────────────────────────────────────────

function W_pump = hydraulic(Q_ls, p)
    H_sys = 1 + 89.649*Q_ls^2;                      % Eq. (22)

    % Speed ratio from VFD affinity: H0·kr² - H0·(Q/Q0)² = H_sys  →  Eq. (26)
    kr_sq = (H_sys + p.H0*(Q_ls/p.Q0_ls)^2) / p.H0;
    kr    = sqrt(max(kr_sq, 1e-6));

    Q_rel   = Q_ls / (kr*p.Q0_ls);
    eta_p   = max(p.eta_p0*Q_rel*(1-Q_rel), 0.01);  % Eq. (27)
    W_pump  = p.rho_w*p.g_acc*(Q_ls/1000)*H_sys / eta_p;  % Eq. (23)
end

% ────────────────────────────────────────────────────────────────────────────
%  EER MODEL — Eq. (20)
% ────────────────────────────────────────────────────────────────────────────

function EER = eval_EER(T_evap, T_cond, coef)
    a=coef(1); b=coef(2); c=coef(3); d=coef(4); e=coef(5); f=coef(6);
    EER = a + b*T_evap + c*T_evap^2 + d*T_cond + e*T_cond^2 + f*T_evap*T_cond;
    EER = max(EER, 0.5);
end

% ────────────────────────────────────────────────────────────────────────────
%  PSYCHROMETRICS
% ────────────────────────────────────────────────────────────────────────────

function ps = sat_p(T_C)
    % Buck equation [Pa], T in °C
    ps = 611.21*exp((18.678 - T_C/234.5)*T_C/(257.14 + T_C));
end

function omega = omega_from_phi(T_C, phi, p)
    ps    = sat_p(T_C);
    omega = 0.622*phi*ps/(p.p_atm - phi*ps);
end

function h = h_moist(T_C, omega, p)
    h = p.c_pa*T_C + omega*(p.h_fg0 + p.c_pv*T_C);
end

function T_wb = wet_bulb_approx(T_C, phi)
    % Stull (2011) approximation — avoids nested fzero; accurate to ±0.5 °C
    rh = phi*100;
    T_wb = T_C*atan(0.151977*(rh+8.313659)^0.5) + atan(T_C+rh) ...
         - atan(rh-1.676331) ...
         + 0.00391838*rh^1.5*atan(0.023101*rh) - 4.686035;
end

% ────────────────────────────────────────────────────────────────────────────
%  UTILITIES
% ────────────────────────────────────────────────────────────────────────────

function amb = mk_amb(Ta, phi, G, vw)
    amb = struct('T_amb',Ta,'phi_amb',phi,'G',G,'v_w',vw);
end

function S = init_S(N)
    z = NaN(N,1);
    S = struct('T_w1',z,'T_w2',z,'T_ai',z,'EER_ch',z,'EER_grid',z, ...
               'W_comp',z,'W_pump',z,'W_PV',z,'W_grid',z,'Q_ls',z);
end

function S = eval_step(S, k, Q_ls, amb, p)
    S.Q_ls(k) = Q_ls;
    try
        r = plant_model(Q_ls, amb, p);
        S.T_w1(k)     = r.T_w1;       S.T_w2(k)     = r.T_w2;
        S.T_ai(k)     = r.T_ai;       S.EER_ch(k)   = r.EER_ch;
        S.EER_grid(k) = r.EER_grid;   S.W_comp(k)   = r.W_comp;
        S.W_pump(k)   = r.W_pump;     S.W_PV(k)     = r.W_PV;
        S.W_grid(k)   = r.W_grid;
    catch
        % NaN persists; logged silently
    end
end
