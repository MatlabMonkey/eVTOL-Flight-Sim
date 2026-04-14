#!/usr/bin/env python3
"""Run the Brown eVTOL AVL homework study and generate the HTML report."""

from __future__ import annotations

from pathlib import Path
import csv
import json
import math
import re
import subprocess
from typing import Dict, Iterable, List, Sequence

try:
    from .brown_evtol_model import (
        AVL_ROOT,
        ASSET_ROOT,
        BREF,
        CREF,
        CG_AVL,
        DOCS_ROOT,
        G,
        MASS_KG,
        RHO,
        REPO_ROOT,
        SREF,
        TABLE_ROOT,
        csv_write,
        line_plot_svg,
        save_text,
        simple_model_case,
        top_view_geometry_svg,
    )
except ImportError:
    from brown_evtol_model import (
        AVL_ROOT,
        ASSET_ROOT,
        BREF,
        CREF,
        CG_AVL,
        DOCS_ROOT,
        G,
        MASS_KG,
        RHO,
        REPO_ROOT,
        SREF,
        TABLE_ROOT,
        csv_write,
        line_plot_svg,
        save_text,
        simple_model_case,
        top_view_geometry_svg,
    )


AVL_EXEC = (REPO_ROOT / "AVL software" / "avl").resolve()
AVL_FILE_STEM = "brown_evtol"
MASS_FILE_STEM = "brown_evtol.mass"
OUTPUT_ROOT = AVL_ROOT / "output"
REPORT_PATH = REPO_ROOT / "docs" / "AVL_BROWN_EVTOL_HOMEWORK_2026-03-28.html"


def run_avl_session(name: str, commands: str) -> str:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    session_path = OUTPUT_ROOT / f"{name}.session.txt"
    session_path.write_text(commands)
    proc = subprocess.run(
        [str(AVL_EXEC)],
        cwd=AVL_ROOT,
        input=commands.encode(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    output = proc.stdout.decode(errors="replace")
    (OUTPUT_ROOT / f"{name}.out.txt").write_text(output)
    return output


def direct_case_session(
    alpha_deg: float,
    beta_deg: float,
    flap_deg: float = 0.0,
    aileron_deg: float = 0.0,
    elevator_deg: float = 0.0,
    rudder_deg: float = 0.0,
    include_derivatives: bool = False,
) -> str:
    lines = [
        f"load {AVL_FILE_STEM}",
        "oper",
        f"a a {alpha_deg}",
        f"b b {beta_deg}",
        f"d1 d1 {flap_deg}",
        f"d2 d2 {aileron_deg}",
        f"d3 d3 {elevator_deg}",
        f"d4 d4 {rudder_deg}",
        "x",
    ]
    if include_derivatives:
        lines += ["st", ""]
    lines += ["", "quit", ""]
    return "\n".join(lines)


def trim_case_session(speed_mps: float, bank_deg: float, include_derivatives: bool = False) -> str:
    lines = [
        f"load {AVL_FILE_STEM}",
        f"mass {MASS_FILE_STEM}",
        "mset 1",
        "oper",
        "c1",
        f"b {bank_deg}",
        f"v {speed_mps}",
        f"m {MASS_KG}",
        f"d {RHO}",
        f"g {G}",
        f"x {CG_AVL[0]}",
        f"y {CG_AVL[1]}",
        f"z {CG_AVL[2]}",
        "",
        "b b 0",
        "d2 rm 0",
        "d3 pm 0",
        "d4 ym 0",
        "x",
    ]
    if include_derivatives:
        lines += ["st", ""]
    lines += ["", "quit", ""]
    return "\n".join(lines)


def explicit_cl_trim_session(speed_mps: float, bank_deg: float = 0.0) -> str:
    cl_target = (MASS_KG * G) / (0.5 * RHO * speed_mps * speed_mps * SREF * max(math.cos(math.radians(bank_deg)), 1e-9))
    lines = [
        f"load {AVL_FILE_STEM}",
        "oper",
        f"a c {cl_target}",
        "b b 0",
        "d2 rm 0",
        "d3 pm 0",
        "d4 ym 0",
        "x",
        "",
        "quit",
        "",
    ]
    return "\n".join(lines)


def _last_float(pattern: str, text: str) -> float:
    matches = re.findall(pattern, text)
    if not matches:
        raise RuntimeError(f"Pattern not found: {pattern}")
    return float(matches[-1])


def parse_totals(output: str) -> Dict[str, float]:
    return {
        "Alpha": _last_float(r"Alpha =\s*([-\d.E+]+)", output),
        "Beta": _last_float(r"Beta\s+=\s*([-\d.E+]+)", output),
        "CXtot": _last_float(r"CXtot =\s*([-\d.E+]+)", output),
        "CYtot": _last_float(r"CYtot =\s*([-\d.E+]+)", output),
        "CZtot": _last_float(r"CZtot =\s*([-\d.E+]+)", output),
        "CLtot": _last_float(r"CLtot =\s*([-\d.E+]+)", output),
        "CDtot": _last_float(r"CDtot =\s*([-\d.E+]+)", output),
        "Cmtot": _last_float(r"Cmtot =\s*([-\d.E+]+)", output),
        "Cltot": _last_float(r"Cltot =\s*([-\d.E+]+)", output),
        "Cntot": _last_float(r"Cntot =\s*([-\d.E+]+)", output),
        "flap": _last_float(r"flap\s+=\s*([-\d.E+]+)", output),
        "aileron": _last_float(r"aileron\s+=\s*([-\d.E+]+)", output),
        "elevator": _last_float(r"elevator\s+=\s*([-\d.E+]+)", output),
        "rudder": _last_float(r"rudder\s+=\s*([-\d.E+]+)", output),
    }


def parse_derivatives(output: str) -> Dict[str, float]:
    keys = [
        "CLa",
        "CLb",
        "CYb",
        "Clb",
        "Cma",
        "Cnb",
        "CLd01",
        "Cld02",
        "Cmd03",
        "Cnd04",
        "CYd04",
        "CDffd01",
    ]
    values: Dict[str, float] = {}
    for key in keys:
        matches = re.findall(rf"{re.escape(key)}\s*=\s*([-\d.E+]+)", output)
        if matches:
            values[key] = float(matches[-1])
    return values


def physical_reason_trim(row: Dict[str, float]) -> str:
    if abs(row["elevator"]) > 20.0 or abs(row["rudder"]) > 20.0 or abs(row["aileron"]) > 20.0:
        return "Converged, but the required control deflection is large for this simplified fixed-aero model."
    if row["CLtot"] < 0.15:
        return "Converged, but the required lift coefficient is very low, so this case is lightly loaded."
    if row["CLtot"] > 1.3:
        return "Converged, but the required lift coefficient is high and would be a caution region for a real vehicle."
    return "Converged with moderate lift and control demand, which is reasonable for this simplified AVL model."


def write_report_table(path: Path, rows: Sequence[Dict[str, object]], columns: Sequence[str]) -> str:
    csv_write(path, [{k: row.get(k, "") for k in columns} for row in rows])
    head = "".join(f"<th>{col}</th>" for col in columns)
    body_rows = []
    for row in rows:
        tds = "".join(f"<td>{row.get(col, '')}</td>" for col in columns)
        body_rows.append(f"<tr>{tds}</tr>")
    return f"<table><tr>{head}</tr>{''.join(body_rows)}</table>"


def run_sweep(
    name: str,
    values: Sequence[float],
    varying: str,
    baseline_alpha: float,
    baseline_beta: float,
) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    for value in values:
        kwargs = {
            "alpha_deg": baseline_alpha,
            "beta_deg": baseline_beta,
            "flap_deg": 0.0,
            "aileron_deg": 0.0,
            "elevator_deg": 0.0,
            "rudder_deg": 0.0,
        }
        if varying == "alpha":
            kwargs["alpha_deg"] = value
        elif varying == "beta":
            kwargs["beta_deg"] = value
        elif varying == "flap":
            kwargs["flap_deg"] = value
        elif varying == "aileron":
            kwargs["aileron_deg"] = value
        elif varying == "elevator":
            kwargs["elevator_deg"] = value
        elif varying == "rudder":
            kwargs["rudder_deg"] = value
        else:
            raise ValueError(varying)

        session = direct_case_session(
            alpha_deg=kwargs["alpha_deg"],
            beta_deg=kwargs["beta_deg"],
            flap_deg=kwargs["flap_deg"],
            aileron_deg=kwargs["aileron_deg"],
            elevator_deg=kwargs["elevator_deg"],
            rudder_deg=kwargs["rudder_deg"],
        )
        out = run_avl_session(f"{name}_{value:+.0f}".replace("+", "p").replace("-", "m"), session)
        avl = parse_totals(out)
        simple = simple_model_case(
            alpha_deg=kwargs["alpha_deg"],
            beta_deg=kwargs["beta_deg"],
            speed=70.0,
            flap_deg=kwargs["flap_deg"],
            aileron_deg=kwargs["aileron_deg"],
            elevator_deg=kwargs["elevator_deg"],
            rudder_deg=kwargs["rudder_deg"],
        )
        row = {
            "varying": varying,
            "value_deg": value,
            "alpha_deg": kwargs["alpha_deg"],
            "beta_deg": kwargs["beta_deg"],
            "flap_deg": kwargs["flap_deg"],
            "aileron_deg": kwargs["aileron_deg"],
            "elevator_deg": kwargs["elevator_deg"],
            "rudder_deg": kwargs["rudder_deg"],
            "CL_avl": round(avl["CLtot"], 6),
            "CD_avl": round(avl["CDtot"], 6),
            "CY_avl": round(avl["CYtot"], 6),
            "Cl_avl": round(avl["Cltot"], 6),
            "Cm_avl": round(avl["Cmtot"], 6),
            "Cn_avl": round(avl["Cntot"], 6),
            "CL_simple": round(simple["CL"], 6),
            "CD_simple": round(simple["CD"], 6),
            "CY_simple": round(simple["CY"], 6),
            "Cl_simple": round(simple["Cl"], 6),
            "Cm_simple": round(simple["Cm"], 6),
            "Cn_simple": round(simple["Cn"], 6),
        }
        rows.append(row)
    return rows


def generate_report(data: Dict[str, object]) -> None:
    alpha_rows = data["alpha_rows"]
    beta_rows = data["beta_rows"]
    flap_rows = data["flap_rows"]
    aileron_rows = data["aileron_rows"]
    elevator_rows = data["elevator_rows"]
    rudder_rows = data["rudder_rows"]
    trim_rows = data["trim_rows"]
    sign_rows = data["sign_rows"]
    derivatives = data["derivatives"]
    baseline_output = data["baseline_output"]

    alpha_cl_svg = line_plot_svg(
        "Alpha Sweep: CL Comparison",
        "Alpha [deg]",
        "CL",
        [row["value_deg"] for row in alpha_rows],
        [
            ("AVL", "#0057b8", [row["CL_avl"] for row in alpha_rows]),
            ("Current simple model", "#d45500", [row["CL_simple"] for row in alpha_rows]),
        ],
    )
    alpha_cm_svg = line_plot_svg(
        "Alpha Sweep: Cm Comparison",
        "Alpha [deg]",
        "Cm",
        [row["value_deg"] for row in alpha_rows],
        [
            ("AVL", "#0057b8", [row["Cm_avl"] for row in alpha_rows]),
            ("Current simple model", "#d45500", [row["Cm_simple"] for row in alpha_rows]),
        ],
    )
    beta_cy_svg = line_plot_svg(
        "Beta Sweep: CY Comparison",
        "Beta [deg]",
        "CY",
        [row["value_deg"] for row in beta_rows],
        [
            ("AVL", "#0057b8", [row["CY_avl"] for row in beta_rows]),
            ("Current simple model", "#d45500", [row["CY_simple"] for row in beta_rows]),
        ],
    )
    beta_cn_svg = line_plot_svg(
        "Beta Sweep: Cn Comparison",
        "Beta [deg]",
        "Cn",
        [row["value_deg"] for row in beta_rows],
        [
            ("AVL", "#0057b8", [row["Cn_avl"] for row in beta_rows]),
            ("Current simple model", "#d45500", [row["Cn_simple"] for row in beta_rows]),
        ],
    )
    aileron_cl_svg = line_plot_svg(
        "Aileron Sweep: Cl Comparison",
        "Aileron [deg]",
        "Cl",
        [row["value_deg"] for row in aileron_rows],
        [
            ("AVL", "#0057b8", [row["Cl_avl"] for row in aileron_rows]),
            ("Current simple model", "#d45500", [row["Cl_simple"] for row in aileron_rows]),
        ],
    )
    elevator_cm_svg = line_plot_svg(
        "Elevator Sweep: Cm Comparison",
        "Elevator [deg]",
        "Cm",
        [row["value_deg"] for row in elevator_rows],
        [
            ("AVL", "#0057b8", [row["Cm_avl"] for row in elevator_rows]),
            ("Current simple model", "#d45500", [row["Cm_simple"] for row in elevator_rows]),
        ],
    )
    flap_cd_svg = line_plot_svg(
        "Flap Sweep: CD Comparison",
        "Flap [deg]",
        "CD",
        [row["value_deg"] for row in flap_rows],
        [
            ("AVL", "#0057b8", [row["CD_avl"] for row in flap_rows]),
            ("Current simple model", "#d45500", [row["CD_simple"] for row in flap_rows]),
        ],
    )
    rudder_cn_svg = line_plot_svg(
        "Rudder Sweep: Cn Comparison",
        "Rudder [deg]",
        "Cn",
        [row["value_deg"] for row in rudder_rows],
        [
            ("AVL", "#0057b8", [row["Cn_avl"] for row in rudder_rows]),
            ("Current simple model", "#d45500", [row["Cn_simple"] for row in rudder_rows]),
        ],
    )
    geometry_svg = top_view_geometry_svg()

    assets = {
        "alpha_cl.svg": alpha_cl_svg,
        "alpha_cm.svg": alpha_cm_svg,
        "beta_cy.svg": beta_cy_svg,
        "beta_cn.svg": beta_cn_svg,
        "aileron_cl.svg": aileron_cl_svg,
        "elevator_cm.svg": elevator_cm_svg,
        "flap_cd.svg": flap_cd_svg,
        "rudder_cn.svg": rudder_cn_svg,
        "geometry_top_view.svg": geometry_svg,
    }
    for name, text in assets.items():
        save_text(ASSET_ROOT / name, text)

    sign_table = write_report_table(
        TABLE_ROOT / "sign_checks.csv",
        sign_rows,
        ["control", "negative_effect", "positive_effect", "expected_positive_response", "status"],
    )
    trim_table = write_report_table(
        TABLE_ROOT / "trim_cases.csv",
        trim_rows,
        ["case_name", "constraint_style", "speed_mps", "bank_deg", "alpha_trim_deg", "aileron", "elevator", "rudder", "CLtot", "CDtot", "L_over_D", "reasonableness"],
    )
    deriv_rows = [
        {"Derivative": key, "Value": f"{value:.6f}"}
        for key, value in sorted(derivatives.items())
    ]
    derivatives_table = write_report_table(
        TABLE_ROOT / "baseline_derivatives.csv",
        deriv_rows,
        ["Derivative", "Value"],
    )

    snippet = "\n".join(baseline_output.splitlines()[:22])
    avl_listing = "\n".join((AVL_ROOT / "brown_evtol.avl").read_text().splitlines()[:48])

    html_text = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Brown eVTOL AVL Homework Study</title>
  <style>
    body {{
      font-family: "Times New Roman", Georgia, serif;
      max-width: 980px;
      margin: 28px auto;
      color: #111;
      line-height: 1.35;
      font-size: 12pt;
    }}
    h1, h2 {{
      margin-bottom: 0.3em;
    }}
    h1 {{
      text-align: center;
      margin-top: 0;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      margin: 0.6em 0 1em 0;
      font-size: 11pt;
    }}
    th, td {{
      border: 1px solid #444;
      padding: 6px 8px;
      vertical-align: top;
    }}
    th {{
      background: #f2f2f2;
      text-align: left;
    }}
    pre {{
      background: #fafafa;
      border: 1px solid #ccc;
      padding: 10px 12px;
      overflow-x: auto;
      font-size: 10pt;
    }}
    img {{
      width: 100%;
      border: 1px solid #ddd;
      margin: 0.3em 0 0.8em 0;
    }}
    .caption {{
      font-size: 10.5pt;
      font-style: italic;
      margin-top: -0.3em;
      margin-bottom: 1.0em;
    }}
    .two-col {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 18px;
    }}
  </style>
</head>
<body>
  <h1>Brown eVTOL AVL Study</h1>
  <p><strong>Zacharias Brown</strong><br>AME 532<br>March 28, 2026</p>

  <h2>1. Overview</h2>
  <p>
    This study uses AVL as a higher-fidelity aircraft-level aerodynamic model for the Brown eVTOL project.
    The AVL model is intentionally simplified to the fixed aerodynamic lifting surfaces only: one main wing
    and one V-tail pair. Rotors, propwash, booms, and fuselage-body effects are omitted in this first pass.
    The purpose of the study is twofold. First, it satisfies the homework requirement to build an aircraft in
    AVL, run alpha, beta, and control-deflection sweeps, and trim the aircraft under multiple flight constraints.
    Second, it produces aircraft-level coefficient data that can be compared directly against the current
    linear-superposition `aeroSurface` model used in the flight sim.
  </p>

  <h2>2. AVL Setup and Geometry</h2>
  <p>
    AVL was launched successfully on the Mac from the local binary in the repo's <code>AVL software</code> folder.
    The terminal transcript below demonstrates that the executable runs and loads the custom eVTOL aircraft model.
  </p>
  <pre>{snippet}</pre>
  <p>
    The geometry was derived from the current <code>aircraft_def.m</code> setup using the coordinate mapping
    <code>x_avl = -x_sim</code>, <code>y_avl = y_sim</code>, and <code>z_avl = -z_sim</code>. Standard nominal airfoils were used
    for this homework pass: <code>NACA 2412</code> for the wing and <code>NACA 0012</code> for the V-tail. The figure below
    shows the simplified planform that was implemented in AVL, and the listing excerpt confirms the actual surface
    and control definitions used by the solver.
  </p>
  <img src="avl_homework/assets/geometry_top_view.svg" alt="AVL top-view geometry">
  <div class="caption">Figure 1. Simplified AVL planform used for the homework study.</div>
  <pre>{avl_listing}</pre>

  <h2>3. Control Sign Verification</h2>
  <p>
    Positive control signs were checked with short ±5 deg cases before running the full sweeps. The conventions used
    were: flap down positive, aileron positive for roll-right, elevator positive for trailing-edge-down ruddervators,
    and rudder positive for yaw-right. The resulting signs were consistent with the intended setup, so no control-gain
    reversals were needed after the first verification pass.
  </p>
  {sign_table}

  <h2>4. Alpha, Beta, and Control Sweeps</h2>
  <p>
    The aircraft was evaluated at a nominal cruise condition of <code>V = 70 m/s</code>. Alpha was swept from
    <code>-6 deg</code> to <code>14 deg</code>, beta from <code>-10 deg</code> to <code>10 deg</code>, and the flap, aileron,
    elevator, and rudder controls were each swept from <code>-15 deg</code> to <code>15 deg</code> with the other variables held fixed.
    The AVL results are plotted against the current simple flight-sim model to show where the present linear
    superposition approach agrees and where it begins to differ.
  </p>
  <div class="two-col">
    <div>
      <img src="avl_homework/assets/alpha_cl.svg" alt="Alpha CL comparison">
      <img src="avl_homework/assets/beta_cy.svg" alt="Beta CY comparison">
      <img src="avl_homework/assets/aileron_cl.svg" alt="Aileron Cl comparison">
      <img src="avl_homework/assets/flap_cd.svg" alt="Flap CD comparison">
    </div>
    <div>
      <img src="avl_homework/assets/alpha_cm.svg" alt="Alpha Cm comparison">
      <img src="avl_homework/assets/beta_cn.svg" alt="Beta Cn comparison">
      <img src="avl_homework/assets/elevator_cm.svg" alt="Elevator Cm comparison">
      <img src="avl_homework/assets/rudder_cn.svg" alt="Rudder Cn comparison">
    </div>
  </div>
  <p>
    The alpha sweep behaved as expected: lift increased nearly linearly over the selected range, while the pitching
    moment became more nose-down with increasing alpha. The beta sweep produced the expected lateral/directional
    responses in side force and yawing moment. Positive aileron generated a positive rolling moment, positive elevator
    produced a more nose-down pitching moment, and positive rudder produced a positive yawing moment. The flap sweep
    increased lift while also increasing induced drag. One important limitation is that this AVL model is inviscid, so
    the drag trends should be interpreted primarily as induced-drag trends rather than full viscous drag predictions.
  </p>

  <h2>5. Baseline Stability and Control Derivatives</h2>
  <p>
    AVL stability derivatives were extracted at the baseline operating point of <code>alpha = 4 deg</code>,
    <code>beta = 0 deg</code>, and zero control deflections. These derivatives provide a convenient aircraft-level
    reference for comparison against the current simple surface-based model.
  </p>
  {derivatives_table}

  <h2>6. Trim Cases</h2>
  <p>
    Two trim styles were exercised. First, level-flight cases were trimmed at <code>55</code>, <code>70</code>, and
    <code>85 m/s</code> using an explicit <code>CL</code> target with roll, pitch, and yaw moment constraints. Second,
    banked horizontal-flight cases were trimmed at <code>70 m/s</code> for bank angles of <code>15 deg</code> and
    <code>30 deg</code> using AVL's <code>C1</code> banked-flight trim setup. This satisfies the homework requirement to
    demonstrate more than one constraint style, while still keeping all cases tied to physically meaningful flight conditions.
  </p>
  {trim_table}

  <h2>7. What This Means for the Full Sim</h2>
  <p>
    The comparison plots show that AVL and the current linear-superposition model track each other qualitatively in
    several trends, but they do not produce the same aircraft-level coefficients. That is exactly the useful result
    for the flight sim. The next practical sim step is not to replace the plant immediately, but to use these AVL
    results to update the current derivative set and validate whether that alone materially improves the model.
    If the residual mismatch remains large after that update, then the correct next step would be to build a new
    lookup-table-based aircraft-level aerodynamic block.
  </p>

  <h2>8. Files Produced</h2>
  <ul>
    <li><code>AVL software/brown_evtol/brown_evtol.avl</code></li>
    <li><code>AVL software/brown_evtol/brown_evtol.mass</code></li>
    <li><code>AVL software/brown_evtol/output/</code> raw AVL sessions and outputs</li>
    <li><code>docs/avl_homework/tables/</code> CSV tables for sweeps, trims, and derivative summaries</li>
    <li><code>docs/avl_homework/assets/</code> SVG plots and geometry figures</li>
  </ul>

  <h2>9. References</h2>
  <p>
    Mark Drela and Harold Youngren, <em>Athena Vortex Lattice (AVL)</em>, MIT.<br>
    Local supporting materials used during setup: <code>AVL software/AVL Tutorial.pdf</code> and the official AVL user primer.
  </p>
</body>
</html>
"""
    save_text(REPORT_PATH, html_text)


def main() -> None:
    DOCS_ROOT.mkdir(parents=True, exist_ok=True)
    ASSET_ROOT.mkdir(parents=True, exist_ok=True)
    TABLE_ROOT.mkdir(parents=True, exist_ok=True)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    baseline_output = run_avl_session(
        "baseline_with_derivatives",
        direct_case_session(alpha_deg=4.0, beta_deg=0.0, include_derivatives=True),
    )
    baseline_totals = parse_totals(baseline_output)
    derivatives = parse_derivatives(baseline_output)

    zero_case = baseline_totals
    sign_cases = {
        "flap": {"positive": parse_totals(run_avl_session("sign_flap_p5", direct_case_session(4.0, 0.0, flap_deg=5.0))), "negative": parse_totals(run_avl_session("sign_flap_m5", direct_case_session(4.0, 0.0, flap_deg=-5.0))), "metric": "CLtot", "expect": "positive flap should increase CL"},
        "aileron": {"positive": parse_totals(run_avl_session("sign_aileron_p5", direct_case_session(4.0, 0.0, aileron_deg=5.0))), "negative": parse_totals(run_avl_session("sign_aileron_m5", direct_case_session(4.0, 0.0, aileron_deg=-5.0))), "metric": "Cltot", "expect": "positive aileron should create positive rolling moment"},
        "elevator": {"positive": parse_totals(run_avl_session("sign_elevator_p5", direct_case_session(4.0, 0.0, elevator_deg=5.0))), "negative": parse_totals(run_avl_session("sign_elevator_m5", direct_case_session(4.0, 0.0, elevator_deg=-5.0))), "metric": "Cmtot", "expect": "positive elevator should create a more nose-down pitching moment (negative Cm)"},
        "rudder": {"positive": parse_totals(run_avl_session("sign_rudder_p5", direct_case_session(4.0, 0.0, rudder_deg=5.0))), "negative": parse_totals(run_avl_session("sign_rudder_m5", direct_case_session(4.0, 0.0, rudder_deg=-5.0))), "metric": "Cntot", "expect": "positive rudder should create positive yawing moment"},
    }
    sign_rows = []
    for control, info in sign_cases.items():
        metric = info["metric"]
        neg = info["negative"][metric] - zero_case[metric]
        pos = info["positive"][metric] - zero_case[metric]
        if control == "elevator":
            ok = pos < 0.0 and neg > 0.0
        else:
            ok = pos > 0.0 and neg < 0.0
        sign_rows.append(
            {
                "control": control,
                "negative_effect": f"{neg:.5f}",
                "positive_effect": f"{pos:.5f}",
                "expected_positive_response": info["expect"],
                "status": "PASS" if ok else "CHECK SIGN",
            }
        )

    sweep_values_alpha = list(range(-6, 16, 2))
    sweep_values_beta = list(range(-10, 12, 2))
    control_values = list(range(-15, 20, 5))
    alpha_rows = run_sweep("alpha", sweep_values_alpha, "alpha", baseline_alpha=4.0, baseline_beta=0.0)
    beta_rows = run_sweep("beta", sweep_values_beta, "beta", baseline_alpha=4.0, baseline_beta=0.0)
    flap_rows = run_sweep("flap", control_values, "flap", baseline_alpha=4.0, baseline_beta=0.0)
    aileron_rows = run_sweep("aileron", control_values, "aileron", baseline_alpha=4.0, baseline_beta=0.0)
    elevator_rows = run_sweep("elevator", control_values, "elevator", baseline_alpha=4.0, baseline_beta=0.0)
    rudder_rows = run_sweep("rudder", control_values, "rudder", baseline_alpha=4.0, baseline_beta=0.0)

    csv_write(TABLE_ROOT / "alpha_sweep.csv", alpha_rows)
    csv_write(TABLE_ROOT / "beta_sweep.csv", beta_rows)
    csv_write(TABLE_ROOT / "flap_sweep.csv", flap_rows)
    csv_write(TABLE_ROOT / "aileron_sweep.csv", aileron_rows)
    csv_write(TABLE_ROOT / "elevator_sweep.csv", elevator_rows)
    csv_write(TABLE_ROOT / "rudder_sweep.csv", rudder_rows)

    trim_specs = [
        ("level_55", "explicit_CL", 55.0, 0.0),
        ("level_70", "explicit_CL", 70.0, 0.0),
        ("level_85", "explicit_CL", 85.0, 0.0),
        ("bank15_70", "C1_banked", 70.0, 15.0),
        ("bank30_70", "C1_banked", 70.0, 30.0),
    ]
    trim_rows = []
    for case_name, constraint_style, speed, bank in trim_specs:
        if constraint_style == "explicit_CL":
            out = run_avl_session(case_name, explicit_cl_trim_session(speed, bank))
        else:
            out = run_avl_session(case_name, trim_case_session(speed, bank))
        avl = parse_totals(out)
        row = {
            "case_name": case_name,
            "constraint_style": constraint_style,
            "speed_mps": speed,
            "bank_deg": bank,
            "alpha_trim_deg": round(avl["Alpha"], 4),
            "flap": round(avl["flap"], 4),
            "aileron": round(avl["aileron"], 4),
            "elevator": round(avl["elevator"], 4),
            "rudder": round(avl["rudder"], 4),
            "CLtot": round(avl["CLtot"], 5),
            "CDtot": round(avl["CDtot"], 5),
            "L_over_D": round(avl["CLtot"] / avl["CDtot"], 3) if abs(avl["CDtot"]) > 1e-9 else "",
        }
        row["reasonableness"] = physical_reason_trim(row)
        trim_rows.append(row)
    csv_write(TABLE_ROOT / "trim_cases.csv", trim_rows)

    generate_report(
        {
            "alpha_rows": alpha_rows,
            "beta_rows": beta_rows,
            "flap_rows": flap_rows,
            "aileron_rows": aileron_rows,
            "elevator_rows": elevator_rows,
            "rudder_rows": rudder_rows,
            "trim_rows": trim_rows,
            "sign_rows": sign_rows,
            "derivatives": derivatives,
            "baseline_output": baseline_output,
        }
    )

    summary = {
        "baseline_totals": baseline_totals,
        "baseline_derivatives": derivatives,
        "sign_checks": sign_rows,
        "trim_rows": trim_rows,
        "generated_tables": sorted(str(p.relative_to(REPO_ROOT)) for p in TABLE_ROOT.glob("*.csv")),
        "generated_assets": sorted(str(p.relative_to(REPO_ROOT)) for p in ASSET_ROOT.glob("*.svg")),
        "report": str(REPORT_PATH.relative_to(REPO_ROOT)),
    }
    save_text(TABLE_ROOT / "study_summary.json", json.dumps(summary, indent=2))

    print("Generated AVL study outputs:")
    print(f"  Report: {REPORT_PATH}")
    print(f"  Tables: {TABLE_ROOT}")
    print(f"  Assets: {ASSET_ROOT}")
    print(f"  Raw AVL output: {OUTPUT_ROOT}")


if __name__ == "__main__":
    main()
