"""Параметры съёмки по типу сенсора.

Камера: GSD = Sw·H / (F·imW), кадр = im·GSD (Pix4D), шаг галсов = ширина кадра·(1 − поперечное
перекрытие), базис фотографирования = длина кадра·(1 − продольное перекрытие).
LiDAR: полоса = 2H·tan(FOV/2), плотность ≈ PRF / (V·полоса) (USGS Lidar Base Specification).
Геофизика: фиксированная малая высота и шаг галсов, секущие маршруты через tie_line_factor
(UAV Magnetic Survey Guidelines, 2022).
"""
from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field

from .fleet import DroneModel, Payload


class SurveyRequirements(BaseModel):
    """Требования к съёмке, которые задаёт пользователь. Пустые поля — значения по умолчанию."""

    model_config = ConfigDict(allow_inf_nan=False)

    gsd_cm: float | None = Field(3.0, gt=0.0)           # камеры
    front_overlap: float = Field(0.75, ge=0.0, le=0.95)
    side_overlap: float = Field(0.65, ge=0.0, le=0.95)
    altitude_m: float | None = Field(None, ge=0.0)      # явная высота AGL, перекрывает расчёт из GSD
    lidar_density_pts_m2: float = Field(50.0, gt=0.0)
    # перекрытие полос LiDAR: по руководству Геоскан 401 — 10–20 % (у камер — side_overlap)
    lidar_side_overlap: float = Field(0.2, ge=0.0, le=0.9)
    line_spacing_m: float | None = Field(None, gt=0.0)  # геофизика
    altitude_ceiling_m: float | None = Field(None, ge=0.0)  # потолок разрешённого объёма (AGL)


class SurveyParams(BaseModel):
    altitude_agl_m: float
    line_spacing_m: float
    speed_ms: float
    swath_m: float
    gsd_cm: float | None = None
    photo_base_m: float | None = None
    trigger_interval_s: float | None = None
    lidar_density_pts_m2: float | None = None
    tie_line_spacing_m: float | None = None
    notes: list[str] = []


class SurveyInfeasible(ValueError):
    pass


def _alt_bounds(drone: DroneModel, req: SurveyRequirements) -> tuple[float, float]:
    lo = drone.min_alt_agl_m
    hi = drone.max_alt_agl_m or drone.max_alt_amsl_m
    if req.altitude_ceiling_m is not None:
        hi = min(hi, req.altitude_ceiling_m)
    if lo > hi:
        raise SurveyInfeasible(
            f"{drone.name}: минимальная высота {lo:.0f} м выше потолка {hi:.0f} м"
        )
    return lo, hi


def _cruise(drone: DroneModel, min_speed: float) -> float:
    """Скорость съёмки: крейсерская, но не ниже min_speed (ветер + запас), в пределах макс. скорости."""
    return min(max(drone.cruise_speed_ms, min_speed), drone.airspeed_max_ms)


def camera_params(drone: DroneModel, cam: Payload, req: SurveyRequirements, min_speed: float = 0.0) -> SurveyParams:
    notes: list[str] = []
    lo, hi = _alt_bounds(drone, req)
    k = cam.sensor_w_mm / (cam.focal_mm * cam.image_w_px)  # GSD[м] = k·H
    if req.altitude_m is not None:
        h = req.altitude_m
    else:
        h = (req.gsd_cm or 3.0) / 100 / k
    h_clamped = min(max(h, lo), hi)
    if abs(h_clamped - h) > 0.01:
        notes.append(f"высота {h:.0f} м вне допустимых {lo:.0f}–{hi:.0f} м, взята {h_clamped:.0f} м")
    h = h_clamped
    gsd = k * h
    # Длинная сторона кадра — поперёк маршрута.
    swath = cam.image_w_px * gsd
    along = cam.image_h_px * gsd
    spacing = swath * (1 - req.side_overlap)
    base = along * (1 - req.front_overlap)
    speed = _cruise(drone, min_speed)
    if speed > drone.cruise_speed_ms:
        notes.append(f"скорость поднята до {speed:.1f} м/с из-за ветра")
    interval = base / speed
    min_int = cam.min_trigger_interval_s or 0.0
    if interval < min_int:
        if drone.type == "multirotor":
            speed = base / min_int
            interval = min_int
            notes.append(f"скорость снижена до {speed:.1f} м/с из-за интервала срабатывания камеры")
        else:
            notes.append(
                f"интервал съёмки {interval:.2f} с меньше минимального {min_int:.1f} с — "
                "продольное перекрытие будет ниже заданного"
            )
    return SurveyParams(
        altitude_agl_m=h,
        line_spacing_m=spacing,
        speed_ms=speed,
        swath_m=swath,
        gsd_cm=gsd * 100,
        photo_base_m=base,
        trigger_interval_s=interval,
        notes=notes,
    )


def lidar_params(drone: DroneModel, lidar: Payload, req: SurveyRequirements, min_speed: float = 0.0) -> SurveyParams:
    notes: list[str] = []
    lo, hi = _alt_bounds(drone, req)
    hi = min(hi, lidar.max_range_m or hi)
    h = req.altitude_m or lidar.recommended_alt_m or 100.0
    h = min(max(h, lo), hi)
    swath = 2 * h * math.tan(math.radians(lidar.effective_fov_deg or 70.0) / 2)
    spacing = swath * (1 - req.lidar_side_overlap)
    # Скорость, при которой обеспечивается плотность в одной полосе.
    v_max = lidar.pulse_rate_hz / (req.lidar_density_pts_m2 * swath)
    speed = min(_cruise(drone, min_speed), v_max)
    if speed < 2.0:
        raise SurveyInfeasible(
            f"{lidar.name}: плотность {req.lidar_density_pts_m2} т/м² недостижима на высоте {h:.0f} м"
        )
    if speed < drone.cruise_speed_ms:
        notes.append(f"скорость снижена до {speed:.1f} м/с ради плотности точек")
    elif speed > drone.cruise_speed_ms:
        notes.append(f"скорость поднята до {speed:.1f} м/с из-за ветра")
    density = lidar.pulse_rate_hz / (speed * swath)
    return SurveyParams(
        altitude_agl_m=h,
        line_spacing_m=spacing,
        speed_ms=speed,
        swath_m=swath,
        lidar_density_pts_m2=density,
        notes=notes,
    )


def geophysics_params(drone: DroneModel, sensor: Payload, req: SurveyRequirements, min_speed: float = 0.0) -> SurveyParams:
    lo, hi = _alt_bounds(drone, req)
    h = min(max(req.altitude_m or sensor.recommended_alt_m or 30.0, lo), hi)
    spacing = req.line_spacing_m or sensor.recommended_line_spacing_m or 50.0
    # Для магнитной съёмки скорость ограничиваем, чтобы датчик на тросе не раскачивался.
    speed = max(min(drone.cruise_speed_ms, 8.0), min(min_speed, drone.airspeed_max_ms))
    tie = spacing * (sensor.tie_line_factor or 10)
    return SurveyParams(
        altitude_agl_m=h,
        line_spacing_m=spacing,
        speed_ms=speed,
        swath_m=spacing,
        tie_line_spacing_m=tie,
        notes=["полёт с огибанием рельефа"],
    )


def survey_params(
    drone: DroneModel, payload: Payload, req: SurveyRequirements, min_speed: float = 0.0
) -> SurveyParams:
    """min_speed — нижняя граница скорости съёмки (для удержания линии пути против ветра)."""
    if payload.kind == "camera":
        return camera_params(drone, payload, req, min_speed)
    if payload.kind == "lidar":
        return lidar_params(drone, payload, req, min_speed)
    return geophysics_params(drone, payload, req, min_speed)
