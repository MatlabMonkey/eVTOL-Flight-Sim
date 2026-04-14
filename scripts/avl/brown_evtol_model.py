"""Brown eVTOL AVL/simple-model helper functions.

This module intentionally uses only the Python standard library so it can run
on the current machine without extra package installs.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple
import csv
import html
import math


REPO_ROOT = Path(__file__).resolve().parents[2]
AVL_ROOT = REPO_ROOT / "AVL software" / "brown_evtol"
DOCS_ROOT = REPO_ROOT / "docs" / "avl_homework"
ASSET_ROOT = DOCS_ROOT / "assets"
TABLE_ROOT = DOCS_ROOT / "tables"

RHO = 1.225
MASS_KG = 3040.0
G = 9.81
SREF = 18.75
CREF = 1.50
BREF = 12.50
CG_SIM = (0.0002, 0.0, -0.2089)
CG_AVL = (-CG_SIM[0], CG_SIM[1], -CG_SIM[2])
INERTIA_SIM = {
    "Ixx": 2.184e4,
    "Iyy": 1.418e4,
    "Izz": 3.474e4,
    "Ixz": -5.03e2,
}


@dataclass(frozen=True)
class Surface:
    name: str
    c: float
    b: float
    pos: Tuple[float, float, float]
    n: Tuple[float, float, float]
    cl0: float
    cd0: float
    cda: float
    a0: float
    cm0: float
    cma: float
    cla: float
    ctrl_tau: float
    cm_delta: float
    cd_delta2: float

    @property
    def area(self) -> float:
        return self.c * self.b


WING_L = Surface(
    name="L Main Wing",
    c=1.50,
    b=6.25,
    pos=(-0.50, -3.75, -0.60),
    n=(0.0, 0.0, -1.0),
    cl0=0.35,
    cd0=0.0085,
    cda=0.0424,
    a0=-0.0733,
    cm0=-0.10,
    cma=0.0,
    cla=4.7671,
    ctrl_tau=0.55,
    cm_delta=-0.35,
    cd_delta2=0.30,
)
WING_R = Surface(
    name="R Main Wing",
    c=1.50,
    b=6.25,
    pos=(-0.50, 3.75, -0.60),
    n=(0.0, 0.0, -1.0),
    cl0=0.35,
    cd0=0.0085,
    cda=0.0424,
    a0=-0.0733,
    cm0=-0.10,
    cma=0.0,
    cla=4.7671,
    ctrl_tau=0.55,
    cm_delta=-0.35,
    cd_delta2=0.30,
)
TAIL_L = Surface(
    name="L V-Tail",
    c=1.40,
    b=2.00,
    pos=(-3.50, -0.71, -0.91),
    n=(0.0, math.sin(math.radians(45.0)), -math.cos(math.radians(45.0))),
    cl0=0.0,
    cd0=0.0200,
    cda=0.3183,
    a0=0.0,
    cm0=0.0,
    cma=0.0,
    cla=2.2028,
    ctrl_tau=0.45,
    cm_delta=0.0,
    cd_delta2=0.10,
)
TAIL_R = Surface(
    name="R V-Tail",
    c=1.40,
    b=2.00,
    pos=(-3.50, 0.71, -0.91),
    n=(0.0, -math.sin(math.radians(45.0)), -math.cos(math.radians(45.0))),
    cl0=0.0,
    cd0=0.0200,
    cda=0.3183,
    a0=0.0,
    cm0=0.0,
    cma=0.0,
    cla=2.2028,
    ctrl_tau=0.45,
    cm_delta=0.0,
    cd_delta2=0.10,
)

SURFACES = (WING_L, WING_R, TAIL_L, TAIL_R)


def body_velocity(alpha_deg: float, beta_deg: float, speed: float) -> Tuple[float, float, float]:
    alpha = math.radians(alpha_deg)
    beta = math.radians(beta_deg)
    u = speed * math.cos(alpha) * math.cos(beta)
    v = speed * math.sin(beta)
    w = speed * math.sin(alpha) * math.cos(beta)
    return (u, v, w)


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def add(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return tuple(x + y for x, y in zip(a, b))


def sub(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return tuple(x - y for x, y in zip(a, b))


def scale(v: Sequence[float], s: float) -> Tuple[float, float, float]:
    return tuple(s * x for x in v)


def norm(v: Sequence[float]) -> float:
    return math.sqrt(max(dot(v, v), 0.0))


def normalize(v: Sequence[float]) -> Tuple[float, float, float]:
    n = norm(v)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return tuple(x / n for x in v)


def q_times_sref(speed: float) -> float:
    return 0.5 * RHO * speed * speed * SREF


def simple_surface_forces(
    surface: Surface,
    speed: float,
    alpha_deg: float,
    beta_deg: float,
    delta_local_deg: float,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    v_body = body_velocity(alpha_deg, beta_deg, speed)
    v_dir = normalize(v_body)

    normal_proj = max(-1.0, min(1.0, dot(v_dir, surface.n)))
    alpha_geom = -math.asin(normal_proj)
    alpha_eff = alpha_geom + surface.ctrl_tau * math.radians(delta_local_deg)

    q_s = 0.5 * RHO * speed * speed * surface.area
    cl = surface.cl0 + surface.cla * alpha_eff
    cd = surface.cd0 + surface.cda * (alpha_eff - surface.a0) ** 2 + surface.cd_delta2 * math.radians(delta_local_deg) ** 2
    cm = surface.cm0 + surface.cma * alpha_eff + surface.cm_delta * math.radians(delta_local_deg)

    lift_dir = sub(surface.n, scale(v_dir, dot(surface.n, v_dir)))
    if norm(lift_dir) < 1e-12:
        lift_dir = surface.n
    lift_dir = normalize(lift_dir)
    drag_dir = scale(v_dir, -1.0)

    force = add(scale(lift_dir, q_s * cl), scale(drag_dir, q_s * cd))
    m_axis = normalize(cross(surface.n, v_dir))
    moment_local = scale(m_axis, q_s * surface.c * cm)
    arm = sub(surface.pos, CG_SIM)
    moment = add(moment_local, cross(arm, force))
    return force, moment


def simple_model_case(
    alpha_deg: float,
    beta_deg: float,
    speed: float,
    flap_deg: float = 0.0,
    aileron_deg: float = 0.0,
    elevator_deg: float = 0.0,
    rudder_deg: float = 0.0,
) -> Dict[str, float]:
    deltas = {
        "L Main Wing": flap_deg + aileron_deg,
        "R Main Wing": flap_deg - aileron_deg,
        "L V-Tail": elevator_deg - rudder_deg,
        "R V-Tail": elevator_deg + rudder_deg,
    }

    total_force = (0.0, 0.0, 0.0)
    total_moment = (0.0, 0.0, 0.0)
    for surface in SURFACES:
        force, moment = simple_surface_forces(
            surface,
            speed=speed,
            alpha_deg=alpha_deg,
            beta_deg=beta_deg,
            delta_local_deg=deltas[surface.name],
        )
        total_force = add(total_force, force)
        total_moment = add(total_moment, moment)

    q_ref = q_times_sref(speed)
    v_body = body_velocity(alpha_deg, beta_deg, speed)
    xw = normalize(v_body)
    z_body_down = (0.0, 0.0, 1.0)
    yw = normalize(cross(z_body_down, xw))
    if norm(yw) < 1e-12:
        yw = (0.0, 1.0, 0.0)
    zw = normalize(cross(xw, yw))
    fx_w = dot(total_force, xw)
    fy_w = dot(total_force, yw)
    fz_w = dot(total_force, zw)

    return {
        "CX": total_force[0] / q_ref,
        "CY_body": total_force[1] / q_ref,
        "CZ": total_force[2] / q_ref,
        "CL": -fz_w / q_ref,
        "CD": -fx_w / q_ref,
        "CY": fy_w / q_ref,
        "Cl": total_moment[0] / (q_ref * BREF),
        "Cm": total_moment[1] / (q_ref * CREF),
        "Cn": total_moment[2] / (q_ref * BREF),
    }


def csv_write(path: Path, rows: Iterable[Dict[str, object]]) -> None:
    rows = list(rows)
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _nice_ticks(vmin: float, vmax: float, steps: int = 5) -> List[float]:
    if abs(vmax - vmin) < 1e-9:
        return [vmin]
    return [vmin + i * (vmax - vmin) / steps for i in range(steps + 1)]


def line_plot_svg(
    title: str,
    x_label: str,
    y_label: str,
    x_values: Sequence[float],
    series: Sequence[Tuple[str, str, Sequence[float]]],
    width: int = 760,
    height: int = 360,
) -> str:
    margin_left = 72
    margin_right = 20
    margin_top = 48
    margin_bottom = 54
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    xs = list(x_values)
    ys = [y for _, _, vals in series for y in vals]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if abs(xmax - xmin) < 1e-9:
        xmax = xmin + 1.0
    if abs(ymax - ymin) < 1e-9:
        ymax = ymin + 1.0
    ypad = 0.08 * (ymax - ymin)
    ymin -= ypad
    ymax += ypad

    def px(x: float) -> float:
        return margin_left + (x - xmin) / (xmax - xmin) * plot_w

    def py(y: float) -> float:
        return margin_top + (ymax - y) / (ymax - ymin) * plot_h

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2:.1f}" y="24" text-anchor="middle" font-size="18" font-family="Times New Roman">{html.escape(title)}</text>',
        f'<line x1="{margin_left}" y1="{margin_top+plot_h}" x2="{margin_left+plot_w}" y2="{margin_top+plot_h}" stroke="#111"/>',
        f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top+plot_h}" stroke="#111"/>',
        f'<text x="{width/2:.1f}" y="{height-12}" text-anchor="middle" font-size="14" font-family="Times New Roman">{html.escape(x_label)}</text>',
        f'<text x="18" y="{height/2:.1f}" text-anchor="middle" transform="rotate(-90 18 {height/2:.1f})" font-size="14" font-family="Times New Roman">{html.escape(y_label)}</text>',
    ]

    for tick in _nice_ticks(xmin, xmax):
        x = px(tick)
        parts.append(f'<line x1="{x:.1f}" y1="{margin_top+plot_h}" x2="{x:.1f}" y2="{margin_top+plot_h+5}" stroke="#111"/>')
        parts.append(f'<text x="{x:.1f}" y="{margin_top+plot_h+20}" text-anchor="middle" font-size="12" font-family="Times New Roman">{tick:.1f}</text>')

    for tick in _nice_ticks(ymin, ymax):
        y = py(tick)
        parts.append(f'<line x1="{margin_left-5}" y1="{y:.1f}" x2="{margin_left}" y2="{y:.1f}" stroke="#111"/>')
        parts.append(f'<line x1="{margin_left}" y1="{y:.1f}" x2="{margin_left+plot_w}" y2="{y:.1f}" stroke="#ddd"/>')
        parts.append(f'<text x="{margin_left-10}" y="{y+4:.1f}" text-anchor="end" font-size="12" font-family="Times New Roman">{tick:.2f}</text>')

    legend_x = margin_left + plot_w - 150
    legend_y = margin_top + 18
    for idx, (name, color, vals) in enumerate(series):
        path = " ".join(
            ("M" if i == 0 else "L") + f" {px(x):.1f} {py(y):.1f}"
            for i, (x, y) in enumerate(zip(xs, vals))
        )
        parts.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="2.2"/>')
        ly = legend_y + idx * 18
        parts.append(f'<line x1="{legend_x}" y1="{ly}" x2="{legend_x+18}" y2="{ly}" stroke="{color}" stroke-width="2.2"/>')
        parts.append(f'<text x="{legend_x+24}" y="{ly+4}" font-size="12" font-family="Times New Roman">{html.escape(name)}</text>')

    parts.append("</svg>")
    return "\n".join(parts)


def top_view_geometry_svg(width: int = 760, height: int = 320) -> str:
    # Simple top-view planform sketch from the AVL assumptions.
    margin = 24
    x_min, x_max = -0.5, 4.5
    y_min, y_max = -7.5, 7.5
    plot_w = width - 2 * margin
    plot_h = height - 2 * margin

    def px(x: float) -> float:
        return margin + (x - x_min) / (x_max - x_min) * plot_w

    def py(y: float) -> float:
        return margin + (y_max - y) / (y_max - y_min) * plot_h

    def rect_points(x_le: float, chord: float, y0: float, y1: float) -> str:
        x_te = x_le + chord
        pts = [(x_le, y0), (x_le, y1), (x_te, y1), (x_te, y0)]
        return " ".join(f"{px(x):.1f},{py(y):.1f}" for x, y in pts)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2:.1f}" y="22" text-anchor="middle" font-size="18" font-family="Times New Roman">AVL Simplified Geometry (Top View)</text>',
        f'<line x1="{px(x_min)}" y1="{py(0)}" x2="{px(x_max)}" y2="{py(0)}" stroke="#bbb" stroke-dasharray="4 4"/>',
        f'<polygon points="{rect_points(-0.25, 1.5, 0.625, 6.875)}" fill="#cc33cc" stroke="#111"/>',
        f'<polygon points="{rect_points(-0.25, 1.5, -6.875, -0.625)}" fill="#cc33cc" stroke="#111"/>',
        f'<polygon points="{px(2.8):.1f},{py(0.0):.1f} {px(2.8):.1f},{py(1.4171):.1f} {px(4.2):.1f},{py(1.4171):.1f} {px(4.2):.1f},{py(0.0):.1f}" fill="#ffcc66" stroke="#111" opacity="0.8"/>',
        f'<polygon points="{px(2.8):.1f},{py(0.0):.1f} {px(2.8):.1f},{py(-1.4171):.1f} {px(4.2):.1f},{py(-1.4171):.1f} {px(4.2):.1f},{py(0.0):.1f}" fill="#ffcc66" stroke="#111" opacity="0.8"/>',
        f'<text x="{px(0.4):.1f}" y="{py(7.05):.1f}" font-size="12" font-family="Times New Roman">Wing</text>',
        f'<text x="{px(3.0):.1f}" y="{py(1.9):.1f}" font-size="12" font-family="Times New Roman">V-tail</text>',
        f'<text x="{px(x_min)+4:.1f}" y="{py(y_max)-4:.1f}" font-size="12" font-family="Times New Roman">x aft, y right (AVL)</text>',
        "</svg>",
    ]
    return "\n".join(parts)


def save_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
