"""Steady-state MF13 cornering-balance model.

This is a numerical port of ``VehicleBalance_MF13.m``.  The default setup,
Pacejka coefficients, calculation order, and 1--10 degree slip-angle search
bounds intentionally match the MATLAB script so that the two implementations
can be compared directly.

The model uses the same mixed US customary/SI units as the source script:
weights and forces are lbf, dimensions are inches, acceleration is ft/s^2,
roll stiffness is N*m/deg, and corner speed is m/s.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]

FY_COEFFICIENTS: tuple[float, ...] = (
    250,
    1.4,
    2.4,
    -0.25,
    3,
    -0.1,
    -1.5,
    0,
    0,
    -30.5,
    1.15,
    1,
    0,
    0,
    -0.128,
    0,
    0,
    0,
    1.43,
)
FY_SCALING: tuple[float, ...] = (0.62, 1, 1, 1, 1, 1, 1, 1)

MZ_COEFFICIENTS: tuple[float, ...] = (
    250,
    2.5,
    0.16,
    0.12,
    0,
    -0.065,
    -0.8,
    0,
    0,
    4.8,
    1.8,
    0,
    0,
    0,
    0.2,
    -0.01,
    0,
    0.4,
    0,
    -0.045,
    250,
)
MZ_SCALING: tuple[float, ...] = (0.62, 1, 1, 1, 1, 1, 1, 1)


@dataclass(frozen=True)
class VehicleConfig:
    """Scalar setup inputs from the MATLAB ``VEHICLE SETUP`` block."""

    weight_dist_front: float = 0.49
    weight_dist_left: float = 0.51
    downforce_dist_front: float = 0.46
    total_weight_lbf: float = 580.0
    front_unsprung_weight_lbf: float = 37.5
    rear_unsprung_weight_lbf: float = 40.5
    wheelbase_in: float = 60.5
    front_track_in: float = 48.0
    rear_track_in: float = 48.0
    front_toe_deg: float = 0.0
    rear_toe_deg: float = 0.0
    front_camber_deg: float = -1.25
    rear_camber_deg: float = -1.25
    caster_deg: float = 4.0
    kpi_deg: float = 7.6
    loaded_tire_radius_in: float = 7.875
    sprung_cg_height_in: float = 12.35
    front_roll_center_height_in: float = 2.329
    rear_roll_center_height_in: float = 2.644
    front_arb_roll_stiffness_nm_per_deg: float = 0.0
    rear_arb_roll_stiffness_nm_per_deg: float = 317.6
    front_wheel_rate_lbf_per_in: float = 307.5
    rear_wheel_rate_lbf_per_in: float = 272.5
    corner_radius_m: float = 10.0
    lift_coefficient: float = 3.05
    lift_to_drag_ratio: float = 2.0
    lateral_g_min: float = 1.0
    lateral_g_max: float = 1.66
    sample_count: int = 1000

    def validate(self) -> None:
        if self.sample_count < 2:
            raise ValueError("sample_count must be at least 2")
        if self.lateral_g_max <= self.lateral_g_min:
            raise ValueError("lateral_g_max must be greater than lateral_g_min")
        if self.total_weight_lbf <= 0:
            raise ValueError("total_weight_lbf must be positive")
        if self.wheelbase_in <= 0 or self.front_track_in <= 0 or self.rear_track_in <= 0:
            raise ValueError("wheelbase and track widths must be positive")
        if self.corner_radius_m <= 0:
            raise ValueError("corner_radius_m must be positive")
        if self.lift_to_drag_ratio == 0:
            raise ValueError("lift_to_drag_ratio must not be zero")


@dataclass(frozen=True)
class SteeringGeometry:
    scrub_radius_in: float
    kpi_deg: float
    caster_deg: float
    mechanical_trail_in: float


@dataclass(frozen=True)
class VehicleBalanceResults:
    """Arrays and scalar geometry produced by :func:`run_vehicle_balance`."""

    lateral_g: FloatArray
    lateral_accel_ft_s2: FloatArray
    speed_m_s: FloatArray
    downforce_lbf: FloatArray
    drag_lbf: FloatArray
    front_outside_camber_deg: FloatArray
    front_inside_camber_deg: FloatArray
    rear_outside_camber_deg: FloatArray
    rear_inside_camber_deg: FloatArray
    front_outside_load_lbf: FloatArray
    front_inside_load_lbf: FloatArray
    rear_outside_load_lbf: FloatArray
    rear_inside_load_lbf: FloatArray
    front_load_lbf: FloatArray
    rear_load_lbf: FloatArray
    front_load_transfer_lbf: FloatArray
    rear_load_transfer_lbf: FloatArray
    front_lateral_force_lbf: FloatArray
    rear_lateral_force_lbf: FloatArray
    front_slip_angle_deg: FloatArray
    rear_slip_angle_deg: FloatArray
    steering_angle_deg: FloatArray
    front_outside_aligning_moment_lbf_in: FloatArray
    front_inside_aligning_moment_lbf_in: FloatArray
    rear_outside_aligning_moment_lbf_in: FloatArray
    rear_inside_aligning_moment_lbf_in: FloatArray
    mf13_steering_wheel_force_lbf: FloatArray
    mf12_steering_wheel_force_lbf: FloatArray
    mf11_steering_wheel_force_lbf: FloatArray
    mf13_column_torque_lbf_in: FloatArray
    mf12_column_torque_lbf_in: FloatArray
    mf11_column_torque_lbf_in: FloatArray
    outside_steer_angle_deg: float
    inside_steer_angle_deg: float
    ackermann_inside_steer_angle_deg: float
    effective_front_toe_deg: float


def pacejka_lateral_force(
    coefficients: Sequence[float],
    scaling: Sequence[float],
    normal_load_lbf: float | FloatArray,
    inclination_rad: float | FloatArray,
    slip_angle_rad: float | FloatArray,
) -> float | FloatArray:
    """Return pure-cornering lateral force using ``pacejka.m`` conventions."""

    p = coefficients
    l = scaling
    fz = np.asarray(normal_load_lbf, dtype=float)
    ia = np.asarray(inclination_rad, dtype=float)
    alpha = np.asarray(slip_angle_rad, dtype=float)

    dfz = (fz - p[0]) / p[0]
    svy = fz * (p[15] + p[16] * dfz + (p[17] + p[18] * dfz) * ia) * l[7] * l[4]
    ey = (p[5] + p[6] * dfz) * (1 - (p[7] + p[8] * ia) * np.sign(alpha)) * l[6]
    shy = (p[12] + p[13] * dfz + p[14] * ia) * l[5]
    alpha_y = alpha + shy
    cy = p[1] * l[1]
    dy = fz * (p[2] + p[3] * dfz) * (1 - p[4] * ia**2) * l[0]
    x1 = 2 * np.arctan(fz / (p[10] * p[0] * l[2]))
    x2 = p[9] * p[0] * np.sin(x1) * (1 - p[11] * np.abs(ia)) * l[3] * l[4]
    by = x2 / (cy * dy)
    x3 = by * alpha_y
    force = dy * np.sin(cy * np.arctan(x3 - ey * (x3 - np.arctan(x3)))) + svy
    return float(force) if force.ndim == 0 else force


def pacejka_aligning_moment(
    coefficients: Sequence[float],
    scaling: Sequence[float],
    normal_load_lbf: float | FloatArray,
    inclination_rad: float | FloatArray,
    slip_angle_rad: float | FloatArray,
) -> float | FloatArray:
    """Return pure-cornering aligning moment using ``pacejkaMZ.m`` conventions."""

    p = coefficients
    l = scaling
    fz = np.asarray(normal_load_lbf, dtype=float)
    ia = np.asarray(inclination_rad, dtype=float)
    alpha = np.asarray(slip_angle_rad, dtype=float)

    dfz = (fz - p[0]) / p[0]
    svy = fz * (p[15] + p[16] * dfz + (p[17] + p[18] * dfz) * ia) * l[7] * l[4]
    ey = (p[5] + p[6] * dfz) * (1 - (p[7] + p[8] * ia) * np.sign(alpha)) * l[6]
    shy = (p[12] + p[13] * dfz + p[14] * ia) * l[5]
    alpha_y = alpha + shy
    cy = (p[1] + p[19] * np.sqrt(np.abs(250 - fz)) * np.sign(250 - fz)) * l[1]
    dy = fz * (p[2] + p[3] * dfz) * (1 - p[4] * ia**2) * l[0]
    x1 = 2 * np.arctan(fz / (p[10] * p[0] * l[2]))
    x2 = p[9] * p[0] * np.sin(x1) * (1 - p[11] * np.abs(ia)) * l[3] * l[4]
    by = x2 / (cy * dy)
    x3 = by * alpha_y
    moment = dy * np.sin(cy * np.arctan(x3 - ey * (x3 - np.arctan(x3)))) + svy + p[20] * ia
    return float(moment) if moment.ndim == 0 else moment


def find_slip_angle_deg(
    outside_load_lbf: float,
    axle_load_lbf: float,
    required_lateral_force_lbf: float,
    outside_camber_deg: float,
    inside_camber_deg: float,
    effective_toe_deg: float,
    coefficients: Sequence[float] = FY_COEFFICIENTS,
    scaling: Sequence[float] = FY_SCALING,
) -> float:
    """Match ``findSlip.m`` using its bounded bisection procedure."""

    outside_ia = math.radians(outside_camber_deg)
    inside_ia = -math.radians(inside_camber_deg)
    toe = math.radians(effective_toe_deg)
    target_force = -required_lateral_force_lbf
    inside_load_lbf = axle_load_lbf - outside_load_lbf
    lower = math.radians(1.0)
    upper = math.radians(10.0)
    outside_guess = (lower + upper) / 2 + toe
    inside_guess = (lower + upper) / 2 - toe

    for _ in range(1000):
        axle_force = pacejka_lateral_force(
            coefficients, scaling, outside_load_lbf, outside_ia, outside_guess
        ) + pacejka_lateral_force(
            coefficients, scaling, inside_load_lbf, inside_ia, inside_guess
        )
        if abs(axle_force - target_force) <= 0.01:
            break
        if axle_force > target_force:
            lower = (outside_guess + inside_guess) / 2
        else:
            upper = (outside_guess + inside_guess) / 2
        outside_guess = (lower + upper) / 2 + toe
        inside_guess = (lower + upper) / 2 - toe

    return math.degrees((outside_guess + inside_guess) / 2)


def _solve_axle_slip_angles(
    outside_load: FloatArray,
    axle_load: FloatArray,
    lateral_force: FloatArray,
    outside_camber: FloatArray,
    inside_camber: FloatArray,
    effective_toe_deg: float,
) -> FloatArray:
    return np.fromiter(
        (
            find_slip_angle_deg(fz1, fzt, fy, ia1, ia2, effective_toe_deg)
            for fz1, fzt, fy, ia1, ia2 in zip(
                outside_load,
                axle_load,
                lateral_force,
                outside_camber,
                inside_camber,
                strict=True,
            )
        ),
        dtype=float,
        count=len(lateral_force),
    )


def _steering_effort(
    geometry: SteeringGeometry,
    front_outside_load: FloatArray,
    front_inside_load: FloatArray,
    front_load_transfer: FloatArray,
    front_lateral_force: FloatArray,
    front_aligning_moment: FloatArray,
) -> tuple[FloatArray, FloatArray]:
    steer_rad = math.radians(10.0)
    kpi_rad = math.radians(geometry.kpi_deg)
    caster_rad = math.radians(geometry.caster_deg)
    vertical_moment = -(
        front_outside_load + front_inside_load
    ) * geometry.scrub_radius_in * math.sin(kpi_rad) * math.sin(steer_rad)
    vertical_moment += (
        front_load_transfer
        * geometry.scrub_radius_in
        * math.sin(caster_rad)
        * math.sin(steer_rad)
    )
    lateral_moment = -front_lateral_force * geometry.mechanical_trail_in
    aligning_moment = -front_aligning_moment * math.cos(math.hypot(kpi_rad, caster_rad))
    total_moment = vertical_moment + lateral_moment + aligning_moment
    rack_force = total_moment / (2.95 * math.cos(steer_rad))
    column_torque = rack_force * (1.25 / 2)
    steering_wheel_force = -column_torque / 8.5
    return steering_wheel_force, column_torque


def run_vehicle_balance(config: VehicleConfig | None = None) -> VehicleBalanceResults:
    """Evaluate the MF13 setup over its configured lateral-acceleration range."""

    cfg = config or VehicleConfig()
    cfg.validate()
    g_ft_s2 = 32.2
    total_mass_slugs = cfg.total_weight_lbf / g_ft_s2
    front_unsprung_mass = cfg.front_unsprung_weight_lbf / g_ft_s2
    rear_unsprung_mass = cfg.rear_unsprung_weight_lbf / g_ft_s2
    sprung_mass = total_mass_slugs - front_unsprung_mass - rear_unsprung_mass
    unsprung_cg_height = cfg.loaded_tire_radius_in
    cg_height = (
        cfg.sprung_cg_height_in * sprung_mass
        + unsprung_cg_height * (front_unsprung_mass + rear_unsprung_mass)
    ) / total_mass_slugs

    lateral_g = np.linspace(cfg.lateral_g_min, cfg.lateral_g_max, cfg.sample_count)
    lateral_accel = lateral_g * g_ft_s2
    longitudinal_accel = np.zeros(cfg.sample_count)
    outside_steer = math.degrees(
        math.atan(cfg.wheelbase_in / (cfg.corner_radius_m / 0.0254 + cfg.front_track_in / 2))
    )
    inside_steer = -outside_steer * (1 + 0.002079275 * outside_steer) + 2 * cfg.front_toe_deg
    ackermann_inside_steer = -math.degrees(
        math.atan(cfg.wheelbase_in / (cfg.corner_radius_m / 0.0254 - cfg.front_track_in / 2))
    )
    effective_front_toe = (inside_steer - ackermann_inside_steer) / 2

    speed_m_s = np.sqrt(cfg.corner_radius_m * lateral_g * 9.81)
    downforce = 0.5 * 1.225 * speed_m_s**2 * cfg.lift_coefficient * 1.08 * 0.224809
    drag = downforce / cfg.lift_to_drag_ratio

    front_wheel_roll_stiffness = (
        cfg.front_wheel_rate_lbf_per_in
        * cfg.front_track_in**2
        * math.tan(math.radians(1))
        / 2
        * 0.113
    )
    rear_wheel_roll_stiffness = (
        cfg.rear_wheel_rate_lbf_per_in
        * cfg.rear_track_in**2
        * math.tan(math.radians(1))
        / 2
        * 0.113
    )
    front_roll_stiffness = front_wheel_roll_stiffness + cfg.front_arb_roll_stiffness_nm_per_deg
    rear_roll_stiffness = rear_wheel_roll_stiffness + cfg.rear_arb_roll_stiffness_nm_per_deg

    front_outside_camber = (
        cfg.front_camber_deg
        - cfg.caster_deg * math.sin(math.radians(outside_steer))
        + cfg.kpi_deg * (1 - math.cos(math.radians(outside_steer)))
    )
    front_inside_camber = (
        cfg.front_camber_deg
        - cfg.caster_deg * math.sin(math.radians(inside_steer))
        + cfg.kpi_deg * (1 - math.cos(math.radians(inside_steer)))
    )
    roll_couple = lateral_accel * sprung_mass * (
        cfg.sprung_cg_height_in
        - (cfg.rear_roll_center_height_in + cfg.front_roll_center_height_in) / 2
    )
    body_roll_deg = roll_couple * 0.113 / (front_roll_stiffness + rear_roll_stiffness)
    heave_in = downforce / (cfg.front_wheel_rate_lbf_per_in + cfg.rear_wheel_rate_lbf_per_in)
    front_outside_camber = front_outside_camber - 1.12 * heave_in + 0.531 * body_roll_deg
    front_inside_camber = front_inside_camber - 1.12 * heave_in - 0.531 * body_roll_deg
    rear_outside_camber = cfg.rear_camber_deg - 0.972 * heave_in + 0.593 * body_roll_deg
    rear_inside_camber = cfg.rear_camber_deg - 0.972 * heave_in - 0.593 * body_roll_deg

    static_rear_right = (
        cfg.total_weight_lbf * (1 - cfg.weight_dist_front)
        + downforce * (1 - cfg.downforce_dist_front)
    ) * (1 - cfg.weight_dist_left)
    static_rear_left = (
        cfg.total_weight_lbf * (1 - cfg.weight_dist_front)
        + downforce * (1 - cfg.downforce_dist_front)
    ) * cfg.weight_dist_left
    static_front_right = (
        cfg.total_weight_lbf * cfg.weight_dist_front + downforce * cfg.downforce_dist_front
    ) * (1 - cfg.weight_dist_left)
    static_front_left = (
        cfg.total_weight_lbf * cfg.weight_dist_front + downforce * cfg.downforce_dist_front
    ) * cfg.weight_dist_left

    # These denominators intentionally match MF13.m. They are equivalent for
    # the baseline because front and rear tracks are both 48 inches.
    front_unsprung_transfer = (
        lateral_accel * front_unsprung_mass * cfg.loaded_tire_radius_in / cfg.rear_track_in
    )
    rear_unsprung_transfer = (
        lateral_accel * rear_unsprung_mass * cfg.loaded_tire_radius_in / cfg.front_track_in
    )
    front_link_transfer = (
        lateral_accel
        * sprung_mass
        * cfg.weight_dist_front
        * cfg.front_roll_center_height_in
        / cfg.front_track_in
    )
    rear_link_transfer = (
        lateral_accel
        * sprung_mass
        * (1 - cfg.weight_dist_front)
        * cfg.rear_roll_center_height_in
        / cfg.rear_track_in
    )
    front_spring_transfer = (
        front_roll_stiffness
        / (rear_roll_stiffness + front_roll_stiffness)
        * roll_couple
        / cfg.front_track_in
    )
    rear_spring_transfer = (
        rear_roll_stiffness
        / (rear_roll_stiffness + front_roll_stiffness)
        * roll_couple
        / cfg.rear_track_in
    )
    steering_jacking = 7 / 20 * outside_steer / 2
    front_jacking_transfer = -steering_jacking
    rear_jacking_transfer = steering_jacking
    longitudinal_transfer = longitudinal_accel * total_mass_slugs * cg_height / cfg.wheelbase_in

    front_outside_load = (
        static_front_left
        + front_spring_transfer
        + front_link_transfer
        + front_unsprung_transfer
        + front_jacking_transfer
        - longitudinal_transfer / 2
    )
    front_inside_load = (
        static_front_right
        - front_spring_transfer
        - front_link_transfer
        - front_unsprung_transfer
        - front_jacking_transfer
        - longitudinal_transfer / 2
    )
    rear_outside_load = (
        static_rear_left
        + rear_spring_transfer
        + rear_link_transfer
        + rear_unsprung_transfer
        + rear_jacking_transfer
        + longitudinal_transfer / 2
    )
    rear_inside_load = (
        static_rear_right
        - rear_spring_transfer
        - rear_link_transfer
        - rear_unsprung_transfer
        - rear_jacking_transfer
        + longitudinal_transfer / 2
    )
    front_load = front_outside_load + front_inside_load
    rear_load = rear_outside_load + rear_inside_load
    total_load = front_load + rear_load
    left_load = front_outside_load + rear_outside_load
    right_load = total_load - left_load
    front_load_transfer = front_outside_load - front_inside_load
    rear_load_transfer = rear_outside_load - rear_inside_load

    total_lateral_force = total_mass_slugs * lateral_accel
    front_lateral_force = total_lateral_force * cfg.weight_dist_front
    rear_lateral_force = total_lateral_force * (1 - cfg.weight_dist_front)

    front_slip_deg = _solve_axle_slip_angles(
        front_outside_load,
        front_load,
        front_lateral_force,
        front_outside_camber,
        front_inside_camber,
        effective_front_toe,
    )
    rear_slip_deg = _solve_axle_slip_angles(
        rear_outside_load,
        rear_load,
        rear_lateral_force,
        rear_outside_camber,
        rear_inside_camber,
        cfg.rear_toe_deg,
    )
    front_slip_rad = np.radians(front_slip_deg)
    rear_slip_rad = np.radians(rear_slip_deg)

    parasitic_drag = (
        lateral_accel * total_mass_slugs * np.sin(rear_slip_rad)
        + 0.5
        * lateral_accel
        * total_mass_slugs
        * np.sin(front_slip_rad - rear_slip_rad)
        + 0.02 * cfg.total_weight_lbf
    )
    # MF13 evaluates only zero longitudinal acceleration, so the RWD branch
    # applies at every sample. Keep the vector form ready for future extension.
    front_longitudinal_force = np.zeros(cfg.sample_count)
    rear_longitudinal_force = longitudinal_accel * total_mass_slugs + parasitic_drag + drag
    front_lateral_force = np.hypot(front_lateral_force, front_longitudinal_force)
    rear_lateral_force = np.hypot(rear_lateral_force, rear_longitudinal_force)

    rolling_resistance_moment = 0.03 * (left_load - right_load) * cfg.front_track_in / 2
    rolling_resistance_force = rolling_resistance_moment / cfg.wheelbase_in

    front_slip_deg = _solve_axle_slip_angles(
        front_outside_load,
        front_load,
        front_lateral_force,
        front_outside_camber,
        front_inside_camber,
        effective_front_toe,
    )
    rear_slip_deg = _solve_axle_slip_angles(
        rear_outside_load,
        rear_load,
        rear_lateral_force,
        rear_outside_camber,
        rear_inside_camber,
        cfg.rear_toe_deg,
    )
    front_slip_rad = np.radians(front_slip_deg)
    rear_slip_rad = np.radians(rear_slip_deg)

    front_outside_mz = 12 * pacejka_aligning_moment(
        MZ_COEFFICIENTS,
        MZ_SCALING,
        front_outside_load,
        np.radians(front_outside_camber),
        front_slip_rad,
    )
    front_inside_mz = 12 * pacejka_aligning_moment(
        MZ_COEFFICIENTS,
        MZ_SCALING,
        front_inside_load,
        -np.radians(front_inside_camber),
        front_slip_rad,
    )
    rear_outside_mz = 12 * pacejka_aligning_moment(
        MZ_COEFFICIENTS,
        MZ_SCALING,
        rear_outside_load,
        np.radians(rear_outside_camber),
        rear_slip_rad,
    )
    rear_inside_mz = 12 * pacejka_aligning_moment(
        MZ_COEFFICIENTS,
        MZ_SCALING,
        rear_inside_load,
        -np.radians(rear_inside_camber),
        rear_slip_rad,
    )
    self_aligning_force = (
        front_outside_mz + front_inside_mz + rear_outside_mz + rear_inside_mz
    ) / cfg.wheelbase_in

    induced_drag_moment = (
        front_load_transfer
        * lateral_g
        * np.sin(front_slip_rad - rear_slip_rad)
        * cfg.front_track_in
        / 2
    )
    induced_drag_force = induced_drag_moment / cfg.front_track_in
    front_lateral_force = (
        front_lateral_force
        + rolling_resistance_force
        + self_aligning_force
        + induced_drag_force
    )
    rear_lateral_force = (
        rear_lateral_force
        - rolling_resistance_force
        - self_aligning_force
        - induced_drag_force
    )

    front_slip_deg = _solve_axle_slip_angles(
        front_outside_load,
        front_load,
        front_lateral_force,
        front_outside_camber,
        front_inside_camber,
        effective_front_toe,
    )
    rear_slip_deg = _solve_axle_slip_angles(
        rear_outside_load,
        rear_load,
        rear_lateral_force,
        rear_outside_camber,
        rear_inside_camber,
        cfg.rear_toe_deg,
    )
    steering_angle_deg = (
        front_slip_deg
        - rear_slip_deg
        + (abs(outside_steer) + abs(inside_steer)) / 2
    )

    front_aligning_moment = front_inside_mz + front_outside_mz
    steering_geometries = {
        "mf13": SteeringGeometry(0.579, 7.6, 3.94, 0.552),
        "mf12": SteeringGeometry(0.539, 8.0, 4.0, 0.752),
        "mf11": SteeringGeometry(0.77, 5.34, 2.0, 0.475),
    }
    efforts = {
        name: _steering_effort(
            geometry,
            front_outside_load,
            front_inside_load,
            front_load_transfer,
            front_lateral_force,
            front_aligning_moment,
        )
        for name, geometry in steering_geometries.items()
    }

    return VehicleBalanceResults(
        lateral_g=lateral_g,
        lateral_accel_ft_s2=lateral_accel,
        speed_m_s=speed_m_s,
        downforce_lbf=downforce,
        drag_lbf=drag,
        front_outside_camber_deg=front_outside_camber,
        front_inside_camber_deg=front_inside_camber,
        rear_outside_camber_deg=rear_outside_camber,
        rear_inside_camber_deg=rear_inside_camber,
        front_outside_load_lbf=front_outside_load,
        front_inside_load_lbf=front_inside_load,
        rear_outside_load_lbf=rear_outside_load,
        rear_inside_load_lbf=rear_inside_load,
        front_load_lbf=front_load,
        rear_load_lbf=rear_load,
        front_load_transfer_lbf=front_load_transfer,
        rear_load_transfer_lbf=rear_load_transfer,
        front_lateral_force_lbf=front_lateral_force,
        rear_lateral_force_lbf=rear_lateral_force,
        front_slip_angle_deg=front_slip_deg,
        rear_slip_angle_deg=rear_slip_deg,
        steering_angle_deg=steering_angle_deg,
        front_outside_aligning_moment_lbf_in=front_outside_mz,
        front_inside_aligning_moment_lbf_in=front_inside_mz,
        rear_outside_aligning_moment_lbf_in=rear_outside_mz,
        rear_inside_aligning_moment_lbf_in=rear_inside_mz,
        mf13_steering_wheel_force_lbf=efforts["mf13"][0],
        mf12_steering_wheel_force_lbf=efforts["mf12"][0],
        mf11_steering_wheel_force_lbf=efforts["mf11"][0],
        mf13_column_torque_lbf_in=efforts["mf13"][1],
        mf12_column_torque_lbf_in=efforts["mf12"][1],
        mf11_column_torque_lbf_in=efforts["mf11"][1],
        outside_steer_angle_deg=outside_steer,
        inside_steer_angle_deg=inside_steer,
        ackermann_inside_steer_angle_deg=ackermann_inside_steer,
        effective_front_toe_deg=effective_front_toe,
    )


def create_plots(
    results: VehicleBalanceResults,
    output_dir: str | Path = "VehicleBalance_MF13_outputs",
    *,
    show: bool = False,
) -> list[Path]:
    """Create and save the seven figures produced by the MATLAB script."""

    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    def save_plot(filename: str) -> None:
        path = destination / filename
        plt.tight_layout()
        plt.savefig(path, dpi=160)
        written.append(path)

    plt.figure("Front vs Rear Slip Angle")
    plt.plot(results.lateral_g, results.front_slip_angle_deg, label="Front")
    plt.plot(results.lateral_g, results.rear_slip_angle_deg, label="Rear")
    plt.xlabel("Cornering g-force")
    plt.ylabel("Slip angle [deg]")
    plt.title("Comparison of Front vs Rear Slip Angle")
    plt.grid(True)
    plt.legend()
    save_plot("front_vs_rear_slip_angle.png")

    plt.figure("Required Lateral Grip")
    plt.plot(results.lateral_g, results.front_lateral_force_lbf, label="Front")
    plt.plot(results.lateral_g, results.rear_lateral_force_lbf, label="Rear")
    plt.xlabel("Cornering g-force")
    plt.ylabel("Required lateral grip [lbf]")
    plt.title("Required Front and Rear Lateral Grip")
    plt.grid(True)
    plt.legend()
    save_plot("required_lateral_grip.png")

    plt.figure("Front vs Rear Load Transfer")
    plt.plot(results.lateral_g, results.front_load_transfer_lbf, label="Front")
    plt.plot(results.lateral_g, results.rear_load_transfer_lbf, label="Rear")
    plt.xlabel("Cornering g-force")
    plt.ylabel("Load transfer [lbf]")
    plt.title("Front and Rear Load Transfer")
    plt.grid(True)
    plt.legend()
    save_plot("front_vs_rear_load_transfer.png")

    plt.figure("Inside Wheel Loads")
    plt.plot(results.lateral_g, results.front_inside_load_lbf, label="Front")
    plt.plot(results.lateral_g, results.rear_inside_load_lbf, label="Rear")
    plt.xlabel("Cornering g-force")
    plt.ylabel("Inside load [lbf]")
    plt.title("Inside Wheel Loads")
    plt.grid(True)
    plt.legend()
    save_plot("inside_wheel_loads.png")

    plt.figure("Understeer Gradient")
    plt.plot(results.lateral_g, results.steering_angle_deg)
    plt.xlabel("Lateral acceleration [g]")
    plt.ylabel("Steering angle [deg]")
    plt.title("Understeer Gradient for 10 m Radius Corner")
    plt.grid(True)
    save_plot("understeer_gradient.png")

    plt.figure("Steering Wheel Force Comparison")
    plt.plot(results.lateral_g, results.mf13_steering_wheel_force_lbf, label="MF13")
    plt.plot(results.lateral_g, results.mf12_steering_wheel_force_lbf, label="MF12")
    plt.plot(results.lateral_g, results.mf11_steering_wheel_force_lbf, label="MF11")
    plt.xlim(0.8, 1.9)
    plt.xlabel("Cornering g-force")
    plt.ylabel("Steering force [lbf]")
    plt.title("Steering Force at 10 deg Steering Angle, 8.5 in Wheel")
    plt.grid(True)
    plt.legend()
    save_plot("steering_wheel_force_comparison.png")

    plt.figure("Steering Column Torque Comparison")
    plt.plot(results.lateral_g, -results.mf13_column_torque_lbf_in, label="MF13")
    plt.plot(results.lateral_g, -results.mf12_column_torque_lbf_in, label="MF12")
    plt.plot(results.lateral_g, -results.mf11_column_torque_lbf_in, label="MF11")
    plt.xlim(0.8, 1.9)
    plt.xlabel("Cornering g-force")
    plt.ylabel("Column torque [lbf-in]")
    plt.title("Steering Column Torque at 10 deg Steering Angle")
    plt.grid(True)
    plt.legend()
    save_plot("steering_column_torque_comparison.png")

    if show:
        plt.show()
    else:
        plt.close("all")
    return written


def result_summary(config: VehicleConfig, results: VehicleBalanceResults) -> dict[str, Any]:
    """Return compact baseline/end-point metrics suitable for JSON output."""

    return {
        "config": asdict(config),
        "geometry": {
            "outside_steer_angle_deg": results.outside_steer_angle_deg,
            "inside_steer_angle_deg": results.inside_steer_angle_deg,
            "ackermann_inside_steer_angle_deg": results.ackermann_inside_steer_angle_deg,
            "effective_front_toe_deg": results.effective_front_toe_deg,
        },
        "at_min_lateral_g": _sample_summary(results, 0),
        "at_max_lateral_g": _sample_summary(results, -1),
    }


def _sample_summary(results: VehicleBalanceResults, index: int) -> dict[str, float]:
    return {
        "lateral_g": float(results.lateral_g[index]),
        "speed_m_s": float(results.speed_m_s[index]),
        "downforce_lbf": float(results.downforce_lbf[index]),
        "front_outside_load_lbf": float(results.front_outside_load_lbf[index]),
        "front_inside_load_lbf": float(results.front_inside_load_lbf[index]),
        "rear_outside_load_lbf": float(results.rear_outside_load_lbf[index]),
        "rear_inside_load_lbf": float(results.rear_inside_load_lbf[index]),
        "front_lateral_force_lbf": float(results.front_lateral_force_lbf[index]),
        "rear_lateral_force_lbf": float(results.rear_lateral_force_lbf[index]),
        "front_slip_angle_deg": float(results.front_slip_angle_deg[index]),
        "rear_slip_angle_deg": float(results.rear_slip_angle_deg[index]),
        "steering_angle_deg": float(results.steering_angle_deg[index]),
        "mf13_steering_wheel_force_lbf": float(results.mf13_steering_wheel_force_lbf[index]),
        "mf13_column_torque_lbf_in": float(results.mf13_column_torque_lbf_in[index]),
    }


def load_config(path: str | Path) -> VehicleConfig:
    """Load scalar :class:`VehicleConfig` overrides from a JSON object."""

    source = Path(path)
    raw = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{source} must contain a JSON object")
    valid_names = {field.name for field in fields(VehicleConfig)}
    unknown = sorted(set(raw) - valid_names)
    if unknown:
        raise ValueError(f"Unknown configuration field(s): {', '.join(unknown)}")
    return replace(VehicleConfig(), **raw)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        help="JSON file containing VehicleConfig field overrides",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("VehicleBalance_MF13_outputs"),
        help="plot and summary destination (default: %(default)s)",
    )
    parser.add_argument("--show", action="store_true", help="leave interactive plot windows open")
    parser.add_argument("--no-plots", action="store_true", help="calculate results without creating plots")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    config = load_config(args.config) if args.config else VehicleConfig()
    results = run_vehicle_balance(config)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(result_summary(config, results), indent=2) + "\n",
        encoding="utf-8",
    )
    plots = [] if args.no_plots else create_plots(results, args.output_dir, show=args.show)
    print(f"Calculated {config.sample_count} MF13 operating points.")
    print(f"Wrote {summary_path}")
    for path in plots:
        print(f"Wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
