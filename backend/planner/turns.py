"""Время и траектория разворота между соседними галсами.

Самолёт: путь Дубинса с минимальным радиусом R = V²/(g·tan φ) (при шаге галсов < 2R получается
петля — «разворот с выходом на ЛЗП» из руководства Геоскан 201). Мультиротор: остановка,
поворот на месте, разгон — потеря времени ≈ V/a плюс перелёт на шаг галсов.
"""
from __future__ import annotations

import math

from .dubins import shortest_path
from .fleet import DroneModel

MULTIROTOR_ACCEL = 2.5  # м/с², допущение


def turn_time(drone: DroneModel, speed: float, spacing: float) -> float:
    if drone.type == "fixed_wing":
        r = drone.turn_radius_m
        path = shortest_path((0.0, 0.0, 0.0), (0.0, spacing, math.pi), r)
        return path.length / speed
    return spacing / speed + speed / MULTIROTOR_ACCEL


def turn_points(
    drone: DroneModel,
    p_end: tuple[float, float],
    heading_end: float,
    p_next: tuple[float, float],
    heading_next: float,
    step: float = 25.0,
) -> tuple[list[tuple[float, float]], float]:
    """Промежуточные точки разворота (без концов) и длина пути."""
    if drone.type != "fixed_wing":
        return [], math.dist(p_end, p_next)
    path = shortest_path((*p_end, heading_end), (*p_next, heading_next), drone.turn_radius_m)
    pts = [(x, y) for x, y, _ in path.sample(step)[1:-1]]
    return pts, path.length
