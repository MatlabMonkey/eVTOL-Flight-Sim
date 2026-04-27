#!/usr/bin/env python3
"""Generate an example transition metrics table (Python helper)."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.metrics.compute_transition_metrics import compute_transition_metrics


def main():
    dt = 0.05
    t = np.arange(0.0, 30.0 + 1e-12, dt)
    sig = lambda x: 1.0 / (1.0 + np.exp(-x))
    core = sig((t - 10.0) / 1.5)

    tilt = 90.0 * core
    speed = 55.0 * core + 2.5 * np.exp(-0.15 * (t - 12.0) ** 2)

    accel_x = np.gradient(speed, dt)
    accel = np.column_stack([accel_x, np.zeros_like(t), 0.05 * np.sin(0.8 * t)])
    jerk = np.gradient(accel, dt, axis=0)

    u_front = np.tile(0.3 + 0.5 * core, (6, 1)).T
    u_rear = np.tile(0.7 - 0.4 * core, (6, 1)).T
    control = np.hstack([u_front, u_rear])

    log = {
        "t": t,
        "speed": speed,
        "tilt_deg": tilt,
        "accel_xyz": accel,
        "jerk_xyz": jerk,
        "control": control,
    }

    metrics = compute_transition_metrics(log, target_speed=55.0)

    out = {
        "generated_at": datetime.now().isoformat(),
        "metrics": metrics,
    }

    repo = Path(__file__).resolve().parents[2]
    out_json = repo / "docs" / "evidence" / "metrics_example_latest.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(out, indent=2))

    print(f"Wrote {out_json}")
    for k, v in metrics.items():
        print(f"{k:24s}: {v:.6f}" if isinstance(v, (float, int)) else f"{k:24s}: {v}")


if __name__ == "__main__":
    main()
