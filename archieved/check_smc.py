import csv
path = r'case-study\Tug Boat Numerical Simulation\logs\run_S2_SMC.csv'
with open(path) as f:
    rows = list(csv.DictReader(f))
for t_target in [100, 500, 1000, 2000, 4000]:
    row = min(rows, key=lambda r: abs(float(r['t']) - t_target))
    print(f"t={float(row['t']):.0f} y={float(row['y']):.2f} v={float(row['v']):.5f} tau_y={float(row['tau_y']):.0f} T1={float(row['T1']):.0f} T2={float(row['T2']):.0f}")
