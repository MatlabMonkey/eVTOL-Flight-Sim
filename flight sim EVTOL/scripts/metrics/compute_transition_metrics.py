#!/usr/bin/env python3
"""Transition metrics computation (Python helper for non-MATLAB environments)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


def _first_window(mask: np.ndarray, win: int, start_idx: int = 0):
    for i in range(start_idx, len(mask) - win + 1):
        if np.all(mask[i : i + win]):
            return i
    return None


def compute_transition_metrics(log, target_speed=None, settle_band_frac=0.02, settle_window_s=2.0):
    t = np.asarray(log["t"]).reshape(-1)
    speed = np.asarray(log["speed"]).reshape(-1)
    tilt = np.asarray(log["tilt_deg"]).reshape(-1)
    accel = np.asarray(log.get("accel_xyz", np.zeros((len(t), 3))))
    control = np.asarray(log.get("control", np.zeros((len(t), 1))))

    if target_speed is None:
        target_speed = float(speed[-1])

    dt = np.diff(t)

    # Derive jerk if not provided.
    if "jerk_xyz" in log:
        jerk = np.asarray(log["jerk_xyz"])
    else:
        dacc = np.diff(accel, axis=0)
        jerk = dacc / np.maximum(dt[:, None], 1e-9)
        jerk = np.vstack([jerk, jerk[-1, :]])

    energy_proxy = float(np.trapz(np.sum(np.abs(control), axis=1), t))
    accel_norm = np.linalg.norm(accel, axis=1)
    jerk_norm = np.linalg.norm(jerk, axis=1)
    accel_rms = float(np.sqrt(np.mean(accel_norm**2)))
    jerk_rms = float(np.sqrt(np.mean(jerk_norm**2)))

    if len(t) > 1:
        du = np.diff(control, axis=0)
        urate = du / np.maximum(dt[:, None], 1e-9)
        control_smoothness = float(np.trapz(np.sum(urate**2, axis=1), t[:-1]))
    else:
        control_smoothness = 0.0

    start_threshold = float(np.min(tilt) + 0.05 * (np.max(tilt) - np.min(tilt)))
    starts = np.where(tilt > start_threshold)[0]
    start_idx = int(starts[0]) if len(starts) else 0

    band = settle_band_frac * max(abs(target_speed), 1e-9)
    settled_mask = np.abs(speed - target_speed) <= band

    sample_time = float(np.mean(dt)) if len(dt) else 0.01
    win = max(1, int(round(settle_window_s / max(sample_time, 1e-9))))
    end_idx = _first_window(settled_mask, win, start_idx)

    if end_idx is None:
        transition_time = float("nan")
        settling_time = float("nan")
    else:
        transition_time = float(t[end_idx] - t[start_idx])
        settling_time = transition_time

    overshoot = float(max(0.0, np.max(speed[start_idx:]) - target_speed))

    return {
        "energy_proxy": energy_proxy,
        "accel_rms": accel_rms,
        "jerk_rms": jerk_rms,
        "control_smoothness": control_smoothness,
        "transition_start_s": float(t[start_idx]),
        "transition_time_s": transition_time,
        "overshoot_mps": overshoot,
        "settling_time_s": settling_time,
        "target_speed_mps": float(target_speed),
    }
