#!/usr/bin/env python3
"""Fallback cruise-trim estimator using extracted model equations.

This is a non-Simulink, quasi-steady estimator intended for environments
without MATLAB. It uses the same aero/prop equations visible in the library
Stateflow code and parameters from Full_Sim_Init.m.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.common.evtol_model_parser import parse_full_sim_init, compute_mass_properties


def aero_surface_force_moment(v_body: np.ndarray, omega: np.ndarray, surf: dict, cg: np.ndarray):
    if np.linalg.norm(v_body) < 0.1:
        return np.zeros(3), np.zeros(3)

    r_arm = surf["pos"] - cg
    v_local = v_body + np.cross(omega, r_arm)
    V2 = float(np.dot(v_local, v_local))
    v_mag = np.sqrt(V2)
    v_dir = v_local / v_mag

    alpha = surf["i"] - np.arcsin(np.dot(v_dir, surf["n"]))
    qS = surf["half_rho_S"] * V2

    CL = surf["CL0"] + surf["CLa"] * alpha
    CD = surf["CD0"] + surf["CDa"] * (alpha - surf["a0"]) ** 2
    CM = surf["CM0"] + surf["CMa"] * alpha

    L = qS * CL
    D = qS * CD

    dir_D = -v_dir
    dir_L = surf["n"] - np.dot(surf["n"], v_dir) * v_dir
    nL = np.linalg.norm(dir_L)
    dir_L = dir_L / nL if nL > 0 else surf["n"]

    F_surf = L * dir_L + D * dir_D

    m_axis = np.cross(surf["n"], v_dir)
    nM = np.linalg.norm(m_axis)
    if nM > 0:
        m_axis = m_axis / nM
    M_surf = (qS * surf["c"] * CM) * m_axis

    F_cg = F_surf
    M_cg = M_surf + np.cross(r_arm, F_surf)
    return F_cg, M_cg


def _force_moment_coeff_for_group(hub_positions, n_vec, cg, kT):
    """Per-rpm^2 force/moment coefficient for a rotor group."""
    F = np.zeros(3)
    M = np.zeros(3)
    for p in hub_positions:
        f_i = kT * n_vec
        r = p - cg
        m_i = np.cross(r, f_i)
        F += f_i
        M += m_i
    return F, M


def _solve_nonnegative_ls(A: np.ndarray, b: np.ndarray):
    """Solve min ||A u - b|| with u>=0 for 2-variable case without SciPy."""

    def eval_candidate(u):
        r = A @ u - b
        return float(np.linalg.norm(r)), u, r

    cands = []

    # Unconstrained LS.
    u_ls = np.linalg.lstsq(A, b, rcond=None)[0]
    if np.all(u_ls >= 0):
        cands.append(eval_candidate(u_ls))

    # Front-only (u2=0)
    a1 = A[:, 0]
    u1 = max(float(np.dot(a1, b) / max(np.dot(a1, a1), 1e-12)), 0.0)
    cands.append(eval_candidate(np.array([u1, 0.0])))

    # Rear-only (u1=0)
    a2 = A[:, 1]
    u2 = max(float(np.dot(a2, b) / max(np.dot(a2, a2), 1e-12)), 0.0)
    cands.append(eval_candidate(np.array([0.0, u2])))

    # Zero actuation
    cands.append(eval_candidate(np.array([0.0, 0.0])))

    cands.sort(key=lambda x: x[0])
    _, u_best, r_best = cands[0]
    return u_best, r_best


def solve_trim(parsed, speed_mps: float, tilt_front_deg: float = 90.0):
    props = compute_mass_properties(parsed["components"])
    m = props["mass"]
    cg = props["cg"]
    kT = parsed["prop"]["k_Thrust"]
    kQ = parsed["prop"]["k_Torque"]
    hub_offset = parsed["prop"]["hub_offset"]

    g_mag = 9.81
    weight = np.array([0.0, 0.0, m * g_mag])  # +z is down in this convention

    n_front = np.array([
        np.sin(np.deg2rad(tilt_front_deg)),
        0.0,
        -np.cos(np.deg2rad(tilt_front_deg)),
    ])
    n_rear = np.array([0.0, 0.0, -1.0])

    # Rotor hub locations from component list.
    comps = parsed["components"]
    front_pivots = np.array([c.pos for c in comps if c.name.startswith("F-Rotor")])
    rear_hubs = np.array([c.pos for c in comps if c.name.startswith("R-Rotor")])
    front_hubs = front_pivots + hub_offset * n_front

    F_front_per_r2, M_front_per_r2 = _force_moment_coeff_for_group(front_hubs, n_front, cg, kT)
    F_rear_per_r2, M_rear_per_r2 = _force_moment_coeff_for_group(rear_hubs, n_rear, cg, kT)

    alpha_grid_deg = np.linspace(-8.0, 16.0, 481)

    best = None
    candidates = []

    for alpha_deg in alpha_grid_deg:
        alpha = np.deg2rad(alpha_deg)
        v_body = np.array([speed_mps * np.cos(alpha), 0.0, speed_mps * np.sin(alpha)])
        omega = np.zeros(3)

        F_aero = np.zeros(3)
        M_aero = np.zeros(3)
        for surf in parsed["surfaces"]:
            Fi, Mi = aero_surface_force_moment(v_body, omega, surf, cg)
            F_aero += Fi
            M_aero += Mi

        # Solve for front/rear rpm^2 from Fx, Fz, My (least-squares with non-negativity).
        A = np.array(
            [
                [F_front_per_r2[0], F_rear_per_r2[0]],
                [F_front_per_r2[2], F_rear_per_r2[2]],
                [M_front_per_r2[1], M_rear_per_r2[1]],
            ],
            dtype=float,
        )
        b = -np.array(
            [
                F_aero[0] + weight[0],
                F_aero[2] + weight[2],
                M_aero[1],
            ],
            dtype=float,
        )

        u, resid_vec = _solve_nonnegative_ls(A, b)
        rf2, rr2 = float(u[0]), float(u[1])

        rpm_front = np.sqrt(max(rf2, 0.0))
        rpm_rear = np.sqrt(max(rr2, 0.0))

        F_front = F_front_per_r2 * rf2
        F_rear = F_rear_per_r2 * rr2
        M_front = M_front_per_r2 * rf2
        M_rear = M_rear_per_r2 * rr2

        F_total = F_aero + F_front + F_rear + weight
        M_total = M_aero + M_front + M_rear

        residual_norm = float(np.linalg.norm(np.array([F_total[0], F_total[2], M_total[1]])))
        power_proxy = float(6.0 * kQ * (rpm_front**3 + rpm_rear**3))

        candidate = {
            "alpha_deg": float(alpha_deg),
            "rpm_front": float(rpm_front),
            "rpm_rear": float(rpm_rear),
            "F_aero_N": F_aero.tolist(),
            "F_front_N": F_front.tolist(),
            "F_rear_N": F_rear.tolist(),
            "F_total_N": F_total.tolist(),
            "M_aero_Nm": M_aero.tolist(),
            "M_front_Nm": M_front.tolist(),
            "M_rear_Nm": M_rear.tolist(),
            "M_total_Nm": M_total.tolist(),
            "residual_vector_[Fx,Fz,My]": [float(F_total[0]), float(F_total[2]), float(M_total[1])],
            "residual_norm": residual_norm,
            "power_proxy": power_proxy,
        }
        candidates.append(candidate)

        # Selection criterion: low residual first, then lower power proxy.
        key = (residual_norm, power_proxy)
        if best is None or key < (best["residual_norm"], best["power_proxy"]):
            best = candidate

    feasible = [
        c
        for c in candidates
        if c["rpm_front"] <= 8000 and c["rpm_rear"] <= 8000 and c["residual_norm"] < 500.0
    ]
    feasible.sort(key=lambda c: (c["power_proxy"], c["residual_norm"]))

    return {
        "generated_at": datetime.now().isoformat(),
        "speed_mps": speed_mps,
        "tilt_front_deg": tilt_front_deg,
        "mass_kg": m,
        "weight_N": float(np.linalg.norm(weight)),
        "front_group_coeff_per_rpm2": {
            "F": F_front_per_r2.tolist(),
            "M": M_front_per_r2.tolist(),
        },
        "rear_group_coeff_per_rpm2": {
            "F": F_rear_per_r2.tolist(),
            "M": M_rear_per_r2.tolist(),
        },
        "best_candidate": best,
        "best_feasible_candidate": feasible[0] if feasible else None,
        "num_candidates": len(candidates),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", default=".")
    ap.add_argument("--speed", type=float, default=70.0, help="Cruise speed in m/s")
    ap.add_argument("--tilt-front-deg", type=float, default=90.0)
    ap.add_argument("--out-json", default="docs/evidence/trim_fallback_latest.json")
    args = ap.parse_args()

    root = Path(args.repo_root).resolve()
    parsed = parse_full_sim_init(root / "Full_Sim_Init.m")
    result = solve_trim(parsed, speed_mps=args.speed, tilt_front_deg=args.tilt_front_deg)

    out = root / args.out_json
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))

    print(f"Wrote {out}")
    bc = result["best_candidate"]
    print("Best candidate:")
    print(
        "  alpha_deg={alpha_deg:.3f}, rpm_front={rpm_front:.1f}, rpm_rear={rpm_rear:.1f}, residual={residual_norm:.3f}".format(
            **bc
        )
    )
    print(
        "  residual [Fx,Fz,My] = {}".format(
            [round(v, 3) for v in bc["residual_vector_[Fx,Fz,My]"]]
        )
    )
    if result["best_feasible_candidate"] is None:
        print("No feasible candidate met residual<500 and RPM<=8000 constraints.")
    else:
        fc = result["best_feasible_candidate"]
        print(
            "Best feasible candidate: alpha_deg={alpha_deg:.3f}, rpm_front={rpm_front:.1f}, rpm_rear={rpm_rear:.1f}, residual={residual_norm:.3f}".format(
                **fc
            )
        )


if __name__ == "__main__":
    main()
