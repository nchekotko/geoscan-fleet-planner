"""Кратчайшие пути Дубинса (Dubins, 1957) для разворотов самолётного БВС.

Формулы шести типов путей — по Shkel & Lumelsky (2001). Поза: (x, y, θ), θ — курс в радианах
(0 = восток, против часовой). Путь описывается тремя сегментами (L — влево, R — вправо, S — прямо).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

TWO_PI = 2 * math.pi


def _mod(a: float) -> float:
    return a % TWO_PI


@dataclass(frozen=True)
class DubinsPath:
    start: tuple[float, float, float]
    radius: float
    word: str                  # например "LSL"
    params: tuple[float, float, float]  # нормированные длины сегментов (углы / длина в радиусах)

    @property
    def length(self) -> float:
        return sum(self.params) * self.radius

    def sample(self, step: float) -> list[tuple[float, float, float]]:
        """Точки пути с шагом step метров (включая конечную)."""
        pts = [self.start]
        x, y, th = self.start
        r = self.radius
        for seg, p in zip(self.word, self.params):
            seg_len = p * r
            n = max(1, math.ceil(seg_len / step))
            ds = seg_len / n
            for _ in range(n):
                if seg == "S":
                    x += ds * math.cos(th)
                    y += ds * math.sin(th)
                else:
                    sgn = 1.0 if seg == "L" else -1.0
                    dth = sgn * ds / r
                    # движение по дуге окружности
                    x += r * sgn * (math.sin(th + dth) - math.sin(th))
                    y += r * sgn * (-math.cos(th + dth) + math.cos(th))
                    th += dth
                pts.append((x, y, th))
        return pts


def _candidates(alpha: float, beta: float, d: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    # LSL
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        yield "LSL", (_mod(-alpha + tmp), math.sqrt(p2), _mod(beta - tmp))
    # RSR
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        yield "RSR", (_mod(alpha - tmp), math.sqrt(p2), _mod(-beta + tmp))
    # LSR
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        yield "LSR", (_mod(-alpha + tmp), p, _mod(-beta + tmp))
    # RSL
    p2 = d * d - 2 + 2 * cab - 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        yield "RSL", (_mod(alpha - tmp), p, _mod(beta - tmp))
    # RLR
    tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8
    if abs(tmp) <= 1:
        p = _mod(TWO_PI - math.acos(tmp))
        t = _mod(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2)
        yield "RLR", (t, p, _mod(alpha - beta - t + p))
    # LRL
    tmp = (6 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8
    if abs(tmp) <= 1:
        p = _mod(TWO_PI - math.acos(tmp))
        t = _mod(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2)
        yield "LRL", (t, p, _mod(_mod(beta) - alpha - t + p))


def shortest_path(
    start: tuple[float, float, float], end: tuple[float, float, float], radius: float
) -> DubinsPath:
    dx, dy = end[0] - start[0], end[1] - start[1]
    d = math.hypot(dx, dy) / radius
    phi = math.atan2(dy, dx)
    alpha = _mod(start[2] - phi)
    beta = _mod(end[2] - phi)
    best = min(_candidates(alpha, beta, d), key=lambda c: sum(c[1]))
    return DubinsPath(start=start, radius=radius, word=best[0], params=best[1])
