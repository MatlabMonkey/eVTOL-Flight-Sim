#!/usr/bin/env python3
"""Run static physical-consistency checks for eVTOL-Flight-Sim.

This script does not require MATLAB. It inspects Full_Sim_Init.m and selected
model XML content inside .slx archives.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import sys
import zipfile

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.common.evtol_model_parser import parse_full_sim_init, compute_mass_properties


def pass_fail(name: str, ok: bool, details: str):
    return {"name": name, "status": "PASS" if ok else "FAIL", "details": details}


def warn(name: str, details: str):
    return {"name": name, "status": "WARN", "details": details}


def _format_matrix(m: np.ndarray) -> str:
    return "[" + "; ".join(", ".join(f"{v:,.3f}" for v in row) for row in m) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", default=".")
    ap.add_argument("--out-json", default="docs/evidence/validation_latest.json")
    ap.add_argument("--out-md", default="docs/evidence/validation_latest.md")
    args = ap.parse_args()

    root = Path(args.repo_root).resolve()
    parsed = parse_full_sim_init(root / "Full_Sim_Init.m")
    props = compute_mass_properties(parsed["components"])

    checks = []

    # Basic scalar checks
    mass = props["mass"]
    cg = props["cg"]
    J = props["inertia"]
    eig = props["principal_moments"]

    checks.append(pass_fail("Mass positive", mass > 0, f"mass={mass:.3f} kg"))
    checks.append(
        pass_fail(
            "Mass plausible range",
            500 <= mass <= 10000,
            "Expected 500..10000 kg for this class (heuristic).",
        )
    )

    all_pos = np.array([c.pos for c in parsed["components"]])
    pmin = all_pos.min(axis=0)
    pmax = all_pos.max(axis=0)
    in_bbox = np.all(cg >= pmin - 1e-9) and np.all(cg <= pmax + 1e-9)
    checks.append(pass_fail("CG within component bounding box", bool(in_bbox), f"CG={cg.tolist()}, bbox_min={pmin.tolist()}, bbox_max={pmax.tolist()}"))

    checks.append(pass_fail("Inertia matrix symmetry", np.allclose(J, J.T, atol=1e-9), f"J-J^T max abs={np.max(np.abs(J-J.T)):.3e}"))
    checks.append(pass_fail("Inertia positive definite", np.all(eig > 0), f"principal moments={eig.tolist()}"))

    tri_ok = (eig[0] <= eig[1] + eig[2]) and (eig[1] <= eig[0] + eig[2]) and (eig[2] <= eig[0] + eig[1])
    checks.append(pass_fail("Principal moments satisfy triangle inequality", bool(tri_ok), f"I={eig.tolist()}"))

    # Rotor sanity checks
    names = [c.name for c in parsed["components"]]
    n_front = sum(1 for n in names if n.startswith("F-Rotor"))
    n_rear = sum(1 for n in names if n.startswith("R-Rotor"))
    checks.append(pass_fail("Front rotor count == 6", n_front == 6, f"count={n_front}"))
    checks.append(pass_fail("Rear rotor count == 6", n_rear == 6, f"count={n_rear}"))

    kT = parsed["prop"]["k_Thrust"]
    kQ = parsed["prop"]["k_Torque"]
    checks.append(pass_fail("Thrust coefficient positive", kT > 0, f"kT={kT}"))
    checks.append(pass_fail("Torque coefficient positive", kQ > 0, f"kQ={kQ}"))

    # Derived thrust-to-weight check at current hardcoded command (1500 RPM)
    rpm_ref = 1500.0
    thrust_per_motor = kT * rpm_ref**2
    total_thrust_12 = 12.0 * thrust_per_motor
    total_thrust_rear = 6.0 * thrust_per_motor
    weight = mass * 9.81
    checks.append(
        warn(
            "Static thrust margin at 1500 RPM",
            (
                f"T_motor={thrust_per_motor:.1f} N, T_12={total_thrust_12:.1f} N, "
                f"T_rear6={total_thrust_rear:.1f} N, Weight={weight:.1f} N"
            ),
        )
    )

    # Cross-check model expects variables that init script defines.
    init_text = parsed["raw_text"]
    model_expectations = []
    with zipfile.ZipFile(root / "Brown_Full_Sim.slx") as zf:
        prop_xml = zf.read("simulink/systems/system_90.xml").decode("utf-8", "ignore")
        model_expectations.extend(re.findall(r"<P Name=\"spin_dir\">([^<]+)</P>", prop_xml))

    for expr in sorted(set(model_expectations)):
        # Expect expressions like prop.Lspin_dir, prop.Rspin_dir
        var_name = expr.strip()
        ok = re.search(rf"\b{re.escape(var_name)}\b", init_text) is not None
        checks.append(pass_fail(f"Init defines {var_name}", ok, "Required by Propellers subsystem mask."))

    # Optional check for saturation blocks (none currently at top-level prop subsystem)
    with zipfile.ZipFile(root / "Brown_Full_Sim.slx") as zf:
        prop_xml = zf.read("simulink/systems/system_90.xml").decode("utf-8", "ignore")
    has_sat = "BlockType=\"Saturation\"" in prop_xml
    if not has_sat:
        checks.append(warn("Propeller command saturation blocks present", "No explicit Saturation block found in Propellers subsystem (system_90.xml)."))
    else:
        checks.append(pass_fail("Propeller command saturation blocks present", True, "Found at least one Saturation block in Propellers subsystem."))

    status_counts = {
        "PASS": sum(1 for c in checks if c["status"] == "PASS"),
        "FAIL": sum(1 for c in checks if c["status"] == "FAIL"),
        "WARN": sum(1 for c in checks if c["status"] == "WARN"),
    }

    out = {
        "generated_at": datetime.now().isoformat(),
        "mass_properties": {
            "mass_kg": mass,
            "cg_m": cg.tolist(),
            "inertia_kgm2": J.tolist(),
            "principal_moments_kgm2": eig.tolist(),
        },
        "derived": {
            "weight_N": weight,
            "thrust_per_motor_at_1500rpm_N": thrust_per_motor,
            "total_thrust_12_at_1500rpm_N": total_thrust_12,
            "total_rear_thrust_6_at_1500rpm_N": total_thrust_rear,
            "T12_to_weight_ratio": total_thrust_12 / weight,
            "Trear6_to_weight_ratio": total_thrust_rear / weight,
        },
        "check_counts": status_counts,
        "checks": checks,
    }

    out_json = root / args.out_json
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(out, indent=2))

    lines = []
    lines.append("# Validation check output")
    lines.append("")
    lines.append(f"Generated: {out['generated_at']}")
    lines.append("")
    lines.append("## Mass properties")
    lines.append(f"- Mass: **{mass:.3f} kg**")
    lines.append(f"- CG: **{cg.tolist()} m**")
    lines.append(f"- Inertia J (kg·m^2): `{_format_matrix(J)}`")
    lines.append(f"- Principal moments: **{eig.tolist()}**")
    lines.append("")
    lines.append("## Derived thrust/weight")
    lines.append(f"- Weight: **{weight:.1f} N**")
    lines.append(f"- Thrust per motor at 1500 RPM: **{thrust_per_motor:.1f} N**")
    lines.append(f"- Total thrust (12 motors): **{total_thrust_12:.1f} N** (T/W={total_thrust_12/weight:.3f})")
    lines.append(f"- Rear-only thrust (6 motors): **{total_thrust_rear:.1f} N** (T/W={total_thrust_rear/weight:.3f})")
    lines.append("")
    lines.append("## Checks")
    for c in checks:
        lines.append(f"- [{c['status']}] **{c['name']}** — {c['details']}")

    out_md = root / args.out_md
    out_md.write_text("\n".join(lines) + "\n")

    print(f"Wrote {out_json}")
    print(f"Wrote {out_md}")
    print(f"Check counts: {status_counts}")


if __name__ == "__main__":
    main()
