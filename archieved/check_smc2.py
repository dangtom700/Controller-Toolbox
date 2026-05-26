import csv
path = r'case-study\Tug Boat Numerical Simulation\logs\run_S2_SMC.csv'
with open(path) as f:
    rows = list(csv.DictReader(f))
# Print 20 consecutive rows around t=600
start_idx = next(i for i,r in enumerate(rows) if float(r['t']) >= 600)
for row in rows[start_idx:start_idx+20]:
    t = float(row['t'])
    y = float(row['y'])
    v = float(row['v'])
    ty = float(row['tau_y'])
    T1 = float(row['T1'])
    T2 = float(row['T2'])
    T3 = float(row['T3'])
    T4 = float(row['T4'])
    print(f"t={t:.1f} y={y:.3f} v={v:.5f} tau_y={ty:.0f} T1={T1:.0f} T2={T2:.0f} T3={T3:.0f} T4={T4:.0f}")
