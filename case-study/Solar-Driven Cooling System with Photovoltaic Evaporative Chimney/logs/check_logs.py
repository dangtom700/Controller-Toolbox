import csv, os
folder = os.path.dirname(os.path.abspath(__file__))
for fn in sorted(os.listdir(folder)):
    if fn.endswith('.csv') and 's01' in fn:
        rows = list(csv.DictReader(open(os.path.join(folder, fn))))
        last = rows[-1]
        print(f"{fn[:48]:48s}  Tw1={float(last['Tw1_C']):6.2f}  EER={float(last['EER']):5.3f}  EER_grid={float(last['EER_grid']):5.3f}  W_PV={float(last['W_PV_W']):6.1f}W")
