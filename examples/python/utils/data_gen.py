"""
Excitation signal generators for system identification experiments.
All functions return (t, signal) numpy arrays of equal length.
"""

from __future__ import annotations
import numpy as np
from typing import Optional


def step_signal(steps: int, Ts: float,
                amplitude: float = 1.0,
                start_step: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Unit (or scaled) step starting at start_step."""
    t = np.arange(steps) * Ts
    u = np.zeros(steps)
    u[start_step:] = amplitude
    return t, u


def prbs(steps: int, Ts: float,
         amplitude: float = 1.0,
         seed: int = 42) -> tuple[np.ndarray, np.ndarray]:
    """
    Maximum-length pseudo-random binary sequence (PRBS) using a 10-bit LFSR.
    Output values are +/-amplitude.
    """
    rng = np.random.default_rng(seed)
    state = int(rng.integers(1, 1023))  # non-zero 10-bit seed
    bits = np.zeros(steps, dtype=int)
    for k in range(steps):
        bits[k] = state & 1
        feedback = ((state >> 9) ^ (state >> 6)) & 1
        state = ((state << 1) | feedback) & 0x3FF
    t = np.arange(steps) * Ts
    u = np.where(bits == 1, amplitude, -amplitude).astype(float)
    return t, u


def chirp_signal(steps: int, Ts: float,
                 f_start: float, f_end: float,
                 amplitude: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """
    Linear-frequency-sweep (chirp) from f_start to f_end Hz over the signal duration.
    Useful for frequency-domain plant identification.
    """
    t = np.arange(steps) * Ts
    T_total = steps * Ts
    phase = 2.0 * np.pi * (f_start * t + 0.5 * (f_end - f_start) / T_total * t**2)
    u = amplitude * np.sin(phase)
    return t, u


def multi_sine(steps: int, Ts: float,
               frequencies: list[float],
               amplitudes: Optional[list] = None,
               phases: Optional[list] = None) -> tuple[np.ndarray, np.ndarray]:
    """
    Sum of sinusoids at specified frequencies.
    amplitudes and phases default to 1 and 0 for each frequency.
    Amplitudes are normalised so peak amplitude approx = 1.
    """
    if amplitudes is None:
        amplitudes = [1.0] * len(frequencies)
    if phases is None:
        phases = [0.0] * len(frequencies)
    t = np.arange(steps) * Ts
    u = np.zeros(steps)
    for f, a, p in zip(frequencies, amplitudes, phases):
        u += a * np.sin(2.0 * np.pi * f * t + p)
    peak = np.max(np.abs(u))
    if peak > 0:
        u /= peak
    return t, u


def add_noise(signal: np.ndarray,
              snr_db: float = 30.0,
              seed: int = 0) -> np.ndarray:
    """
    Add white Gaussian noise to achieve a target SNR (dB).
    Returns a new array; does not modify signal in-place.
    """
    rng = np.random.default_rng(seed)
    signal_power = np.mean(signal**2)
    if signal_power == 0.0:
        return signal.copy()
    noise_power = signal_power / (10.0 ** (snr_db / 10.0))
    noise = rng.normal(0.0, np.sqrt(noise_power), size=signal.shape)
    return signal + noise


