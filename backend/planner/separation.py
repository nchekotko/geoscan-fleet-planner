"""Разведение бортов: проверка сближений по времени (4D).

Положение каждого борта восстанавливается по участкам вылетов: время начала вылета, длительность
участка, равномерное движение вдоль ломаной. Сближение считается конфликтом, если по горизонтали
борта ближе H_MIN_M и по высоте ближе V_MIN_M. Участки взлёта и посадки и окрестность ВПП
(TERMINAL_R_M) не проверяются: там очерёдность задают интервалы стартов.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

H_MIN_M = 50.0      # минимальный интервал по горизонтали (допущение для демо)
V_MIN_M = 15.0      # минимальный интервал по высоте
TERMINAL_R_M = 200.0
DT_S = 5.0


@dataclass
class Track:
    drone_id: str
    t: np.ndarray   # моменты, с от начала работ
    xyz: np.ndarray  # положения (x, y, высота AGL), м


@dataclass
class Approach:
    min_h_m: float        # наименьшее горизонтальное расстояние при конфликте по высоте
    pair: tuple[str, str]
    t_s: float
    conflicts: int        # число моментов (шаг DT_S) с конфликтом


def track(drone_id: str, sorties, alt_of) -> Track:
    """Трек борта. alt_of(leg) — высота участка AGL (с учётом эшелона перелёта)."""
    ts: list[float] = []
    pts: list[tuple[float, float, float]] = []
    for s in sorties:
        t = s.start_s
        for leg in s.legs:
            if leg.kind in ("takeoff", "landing") or len(leg.points) < 2 or leg.duration_s <= 0:
                t += leg.duration_s
                continue
            seg = [math.dist(a, b) for a, b in zip(leg.points, leg.points[1:])]
            total = sum(seg) or 1.0
            z = alt_of(leg)
            acc = 0.0
            for (x, y), d in zip(leg.points, [0.0] + seg):
                acc += d
                ts.append(t + leg.duration_s * acc / total)
                pts.append((x, y, z))
            t += leg.duration_s
        # разрыв между вылетами: борт на земле — отмечаем NaN, чтобы не интерполировать
        ts.append(t + 1e-3)
        pts.append((math.nan, math.nan, math.nan))
    return Track(drone_id, np.array(ts), np.array(pts, dtype=float))


def _at(tr: Track, times: np.ndarray) -> np.ndarray:
    out = np.full((len(times), 3), np.nan)
    if len(tr.t) < 2:
        return out
    order = np.argsort(tr.t, kind="stable")
    t, p = tr.t[order], tr.xyz[order]
    idx = np.searchsorted(t, times, side="right")
    ok = (idx > 0) & (idx < len(t))
    i1 = idx[ok]
    i0 = i1 - 1
    w = ((times[ok] - t[i0]) / np.maximum(t[i1] - t[i0], 1e-9))[:, None]
    seg = p[i0] * (1 - w) + p[i1] * w  # NaN на концах переходит в NaN — борт на земле
    out[ok] = seg
    return out


def closest_approach(tracks: list[Track], bases: list[tuple[float, float]], horizon_s: float) -> Approach | None:
    """Наименьшее горизонтальное расстояние между бортами в моменты, когда они на близких высотах."""
    if len(tracks) < 2 or horizon_s <= 0:
        return None
    times = np.arange(0.0, horizon_s, DT_S)
    pos = [_at(tr, times) for tr in tracks]
    b = np.array(bases, dtype=float) if bases else np.zeros((0, 2))

    def terminal(p: np.ndarray) -> np.ndarray:
        if not len(b):
            return np.zeros(len(p), dtype=bool)
        d = np.min(np.hypot(p[:, None, 0] - b[None, :, 0], p[:, None, 1] - b[None, :, 1]), axis=1)
        return d < TERMINAL_R_M

    best: Approach | None = None
    for i in range(len(tracks)):
        for j in range(i + 1, len(tracks)):
            a, c = pos[i], pos[j]
            both = ~np.isnan(a[:, 0]) & ~np.isnan(c[:, 0]) & ~terminal(a) & ~terminal(c)
            if not both.any():
                continue
            h = np.hypot(a[both, 0] - c[both, 0], a[both, 1] - c[both, 1])
            v = np.abs(a[both, 2] - c[both, 2])
            level = v < V_MIN_M
            if not level.any():
                continue
            k = int(np.argmin(np.where(level, h, np.inf)))
            n_conf = int(np.sum(level & (h < H_MIN_M)))
            cand = Approach(float(h[k]), (tracks[i].drone_id, tracks[j].drone_id), float(times[both][k]), n_conf)
            if best is None or cand.min_h_m < best.min_h_m:
                best = Approach(cand.min_h_m, cand.pair, cand.t_s, cand.conflicts + (best.conflicts if best else 0))
            else:
                best.conflicts += n_conf
    return best
