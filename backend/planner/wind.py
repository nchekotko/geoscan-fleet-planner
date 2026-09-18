"""Треугольник скоростей: путевая скорость вдоль заданного путевого угла при постоянном ветре.

Ветер задаётся метеорологически: `from_deg` — откуда дует (0° = с севера), скорость в м/с.
Углы пути — математические (0 рад = восток, против часовой), как в локальной системе x/y.
"""
from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field


class Wind(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)

    speed_ms: float = Field(0.0, ge=0.0, le=40.0)
    from_deg: float = Field(0.0, ge=0.0, le=360.0)

    def vector(self) -> tuple[float, float]:
        """Вектор движения воздуха (куда дует) в локальных x (восток), y (север)."""
        to_rad = math.radians(self.from_deg + 180.0)
        # азимут → x = sin, y = cos
        return self.speed_ms * math.sin(to_rad), self.speed_ms * math.cos(to_rad)


def ground_speed(airspeed: float, track_rad: float, wind: Wind) -> float | None:
    """Путевая скорость вдоль направления track_rad. None, если ветер не даёт держать линию пути."""
    wx, wy = wind.vector()
    ux, uy = math.cos(track_rad), math.sin(track_rad)
    along = wx * ux + wy * uy
    cross = -wx * uy + wy * ux
    if abs(cross) >= airspeed:
        return None
    gs = along + math.sqrt(airspeed**2 - cross**2)
    return gs if gs > 0.5 else None


def segment_time(length: float, airspeed: float, track_rad: float, wind: Wind) -> float | None:
    gs = ground_speed(airspeed, track_rad, wind)
    return None if gs is None else length / gs
