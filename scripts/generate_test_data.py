import numpy as np
import os
import csv

def generate_step_response_data(filename):
    print(f"Generating step response data for {filename}...")
    
    # Discrete plant G(z) = (b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
    # y[k] = -a1 y[k-1] - a2 y[k-2] + b1 u[k-1] + b2 u[k-2]
    
    a1, a2 = -1.98511, 0.98522
    b1, b2 = 4.9625e-5, 4.9125e-5
    Ts = 0.01
    
    steps = 1500
    t = np.arange(steps) * Ts
    u = np.ones(steps)
    y = np.zeros(steps)
    
    for k in range(2, steps):
        y[k] = -a1 * y[k-1] - a2 * y[k-2] + b1 * u[k-1] + b2 * u[k-2]
        
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'output'])
        for i in range(steps):
            writer.writerow([f"{t[i]:.4f}", f"{y[i]:.6f}"])
            
    # Calculate analytical FOPDT using tangent method for cross-check
    K = y[-1]
    y283 = 0.283 * K
    y632 = 0.632 * K
    
    t283 = t[np.argmax(y >= y283)]
    t632 = t[np.argmax(y >= y632)]
    
    tau = 1.5 * (t632 - t283)
    theta = max(0, t632 - tau)
    
    print(f"Python Analytical FOPDT: K={K:.4f}, tau={tau:.4f}, theta={theta:.4f}")

if __name__ == "__main__":
    os.makedirs("tests/data", exist_ok=True)
    generate_step_response_data("tests/data/step_response.csv")
