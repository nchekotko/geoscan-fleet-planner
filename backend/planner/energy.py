"""Бюджет полёта одного вылета.

Бюджет считаем во времени: паспортная продолжительность × (1 − резерв) × поправка на ветер.
Для мультиротора потребная мощность растёт с воздушной скоростью и ветром (Dorling et al., 2017;
Thibbotuwawa et al., 2019). Используем квадратичную поправку, откалиброванную так, чтобы при
ветре, равном паспортному пределу, продолжительность снижалась на 30 % (допущение). У самолёта
крейсерский режим задан по воздушной скорости, и ветер влияет на путевую скорость, а не на мощность.
"""
from __future__ import annotations

from .fleet import DroneModel
from .wind import Wind

WIND_PENALTY_AT_LIMIT = 0.30


def usable_flight_time_s(drone: DroneModel, wind: Wind, reserve: float = 0.2) -> float:
    t = drone.endurance_min * 60 * (1 - reserve)
    if drone.type == "multirotor" and drone.max_wind_ms > 0:
        k = min(wind.speed_ms / drone.max_wind_ms, 1.0)
        t *= 1 - WIND_PENALTY_AT_LIMIT * k * k
    return t


def check_wind(drone: DroneModel, wind: Wind) -> str | None:
    if wind.speed_ms > drone.max_wind_ms:
        return f"{drone.name}: ветер {wind.speed_ms:.0f} м/с выше допустимого {drone.max_wind_ms:.0f} м/с"
    return None
