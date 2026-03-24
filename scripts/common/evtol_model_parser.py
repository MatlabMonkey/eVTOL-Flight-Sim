#!/usr/bin/env python3
"""Utilities to parse the Brown eVTOL MATLAB definition files.

This parser is intentionally lightweight and tailored to this repository's
current script format. It is used by validation / trim helper scripts to keep
assumptions explicit and reproducible outside MATLAB.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import re
from typing import Dict, List, Any

import numpy as np


@dataclass
class Component:
    name: str
    kind: str
    mass: float
    dims: np.ndarray
    pos: np.ndarray
    eul_deg: np.ndarray


@dataclass
class AeroSurfaceDef:
    name: str
    n: np.ndarray
    CL0: float
    e: float
    i: float
    CD0: float
    CDa: float
    a0: float
    CM0: float
    CMa: float
    CLa: float


def _extract_block(text: str, var_name: str) -> str:
    m = re.search(rf"{re.escape(var_name)}\s*=\s*\{{(.*?)\n\}};", text, re.S)
    if not m:
        raise ValueError(f"Could not locate block for {var_name}")
    return m.group(1)


def _eval_token(token: str, ctx: Dict[str, float]) -> float:
    t = token.strip()
    if not t:
        raise ValueError("Empty token")
    if t in ctx:
        return float(ctx[t])

    # Handle optional unary sign.
    sign = 1.0
    if t.startswith("+"):
        t = t[1:].strip()
    elif t.startswith("-"):
        sign = -1.0
        t = t[1:].strip()

    fn_match = re.fullmatch(r"(sin|cos|tan)d\(([^\)]+)\)", t)
    if fn_match:
        fn, arg = fn_match.groups()
        arg_val = _eval_token(arg, ctx)
        arg_rad = math.radians(arg_val)
        if fn == "sin":
            return sign * math.sin(arg_rad)
        if fn == "cos":
            return sign * math.cos(arg_rad)
        return sign * math.tan(arg_rad)

    try:
        return sign * float(t)
    except ValueError as exc:
        raise ValueError(f"Unsupported token: {token}") from exc


def _parse_vector(vec_text: str, ctx: Dict[str, float]) -> np.ndarray:
    # Supports MATLAB row [a, b, c] and column [a; b; c] forms.
    toks = [t for t in re.split(r"[,;]", vec_text) if t.strip()]
    vals = [_eval_token(t, ctx) for t in toks]
    return np.array(vals, dtype=float)


def _definition_source(path: Path) -> tuple[Path, str, str]:
    text = path.read_text()
    combined_text = text

    try:
        _extract_block(text, "compData")
        _extract_block(text, "aeroData")
        return path, text, combined_text
    except ValueError:
        pass

    aircraft_path = path.with_name("aircraft_def.m")
    if not aircraft_path.exists():
        raise ValueError(
            f"Could not locate compData/aeroData in {path} "
            "and no sibling aircraft_def.m was found."
        )

    aircraft_text = aircraft_path.read_text()
    combined_text = aircraft_text + "\n" + combined_text

    scenario_path = path.with_name("scenario_def.m")
    if scenario_path.exists():
        combined_text = combined_text + "\n" + scenario_path.read_text()

    return aircraft_path, aircraft_text, combined_text


def parse_full_sim_init(path: str | Path = "Full_Sim_Init.m") -> Dict[str, Any]:
    p = Path(path)
    source_path, text, combined_text = _definition_source(p)

    flight_mode = float(re.search(r"\bflight_mode\s*=\s*([\-\d\.eE\+]+)\s*;", text).group(1))
    tilt_angle = 90.0 * flight_mode
    rho = float(re.search(r"\brho\s*=\s*([\-\d\.eE\+]+)\s*;", text).group(1))
    g = float(re.search(r"\bg\s*=\s*([\-\d\.eE\+]+)\s*;", text).group(1))

    prop = {
        "hub_offset": float(re.search(r"\bprop\.hub_offset\s*=\s*([\-\d\.eE\+]+)\s*;", text).group(1)),
        "k_Thrust": float(re.search(r"\bprop\.k_Thrust\s*=\s*([\-\d\.eE\+]+)\s*;", text).group(1)),
        "k_Torque": float(re.search(r"\bprop\.k_Torque\s*=\s*([\-\d\.eE\+]+)\s*;", text).group(1)),
    }

    ctx = {"tilt_angle": tilt_angle}

    comp_block = _extract_block(text, "compData")
    row_re = re.compile(
        r"'([^']+)'\s*,\s*'([^']+)'\s*,\s*([\-\d\.eE\+]+)\s*,\s*\[([^\]]+)\]\s*,\s*\[([^\]]+)\]\s*,\s*\[([^\]]+)\]\s*;"
    )

    components: List[Component] = []
    for m in row_re.finditer(comp_block):
        name, kind, mass_s, dims_s, pos_s, eul_s = m.groups()
        components.append(
            Component(
                name=name,
                kind=kind,
                mass=float(mass_s),
                dims=_parse_vector(dims_s, ctx),
                pos=_parse_vector(pos_s, ctx),
                eul_deg=_parse_vector(eul_s, ctx),
            )
        )

    if not components:
        raise RuntimeError("Failed to parse any compData rows.")

    aero_block = _extract_block(text, "aeroData")
    aero_row_re = re.compile(r"'([^']+)'\s*,\s*\[([^\]]+)\]\s*,\s*([^;]+);")
    aero_defs: List[AeroSurfaceDef] = []
    for m in aero_row_re.finditer(aero_block):
        name, n_vec_s, coeff_s = m.groups()
        coeff_tokens = [t.strip() for t in coeff_s.split(",") if t.strip()]
        coeff = [_eval_token(t, ctx) for t in coeff_tokens]
        if len(coeff) != 9:
            raise RuntimeError(f"Unexpected aero coeff count for {name}: {len(coeff)}")
        aero_defs.append(
            AeroSurfaceDef(
                name=name,
                n=_parse_vector(n_vec_s, ctx),
                CL0=coeff[0],
                e=coeff[1],
                i=coeff[2],
                CD0=coeff[3],
                CDa=coeff[4],
                a0=coeff[5],
                CM0=coeff[6],
                CMa=coeff[7],
                CLa=coeff[8],
            )
        )

    # Build combined surface structs analogous to Full_Sim_Init.
    comp_by_name = {c.name: c for c in components}
    surfaces = []
    for surf in aero_defs:
        c = comp_by_name[surf.name]
        chord = float(c.dims[0])
        span = float(c.dims[1])
        area = chord * span
        surfaces.append(
            {
                "name": surf.name,
                "c": chord,
                "b": span,
                "S": area,
                "half_rho_S": 0.5 * rho * area,
                "pos": c.pos.astype(float),
                "n": surf.n.astype(float),
                "CL0": surf.CL0,
                "e": surf.e,
                "i": surf.i,
                "CD0": surf.CD0,
                "CDa": surf.CDa,
                "a0": surf.a0,
                "CM0": surf.CM0,
                "CMa": surf.CMa,
                "CLa": surf.CLa,
                "AR": span / chord,
            }
        )

    return {
        "path": str(source_path),
        "flight_mode": flight_mode,
        "tilt_angle": tilt_angle,
        "rho": rho,
        "g": g,
        "prop": prop,
        "components": components,
        "surfaces": surfaces,
        "raw_text": combined_text,
    }


def compute_mass_properties(components: List[Component]) -> Dict[str, Any]:
    m_tot = sum(c.mass for c in components)
    cg = sum(c.mass * c.pos for c in components) / m_tot

    I_total = np.zeros((3, 3), dtype=float)
    for c in components:
        m = c.mass
        xL, yL, zL = float(c.dims[0]), abs(float(c.dims[1])), float(c.dims[2])

        if c.kind == "crossprop":
            D, W, T = xL, yL, zL
            Ixx1 = (1 / 12) * (m / 2) * (W**2 + T**2)
            Iyy1 = (1 / 12) * (m / 2) * (D**2 + T**2)
            Izz1 = (1 / 12) * (m / 2) * (D**2 + W**2)
            Ixx2 = (1 / 12) * (m / 2) * (D**2 + T**2)
            Iyy2 = (1 / 12) * (m / 2) * (W**2 + T**2)
            Izz2 = (1 / 12) * (m / 2) * (D**2 + W**2)
            I_local = np.diag([Ixx1 + Ixx2, Iyy1 + Iyy2, Izz1 + Izz2])
        else:
            I_local = np.diag(
                [
                    (1 / 12) * m * (yL**2 + zL**2),
                    (1 / 12) * m * (xL**2 + zL**2),
                    (1 / 12) * m * (xL**2 + yL**2),
                ]
            )

        phi, theta, psi = np.radians(c.eul_deg)
        Rx = np.array(
            [
                [1, 0, 0],
                [0, math.cos(phi), -math.sin(phi)],
                [0, math.sin(phi), math.cos(phi)],
            ]
        )
        Ry = np.array(
            [
                [math.cos(theta), 0, math.sin(theta)],
                [0, 1, 0],
                [-math.sin(theta), 0, math.cos(theta)],
            ]
        )
        Rz = np.array(
            [
                [math.cos(psi), -math.sin(psi), 0],
                [math.sin(psi), math.cos(psi), 0],
                [0, 0, 1],
            ]
        )
        R = Rz @ Ry @ Rx
        I_rot = R @ I_local @ R.T

        d = (c.pos - cg).reshape(3, 1)
        I_total = I_total + I_rot + m * (((d.T @ d).item()) * np.eye(3) - (d @ d.T))

    return {
        "mass": float(m_tot),
        "cg": cg,
        "inertia": I_total,
        "principal_moments": np.linalg.eigvalsh(I_total),
    }
