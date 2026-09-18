"""Случайные сценарии для статистической оценки планировщика.

Область — случайный звёздный (невыпуклый) или выпуклый многоугольник 2–30 км² в Московском
регионе (долгота 37–38°, широта 55–56°), иногда вытянутый; 0–2 запретные зоны внутри области;
1–2 базы в 0,3–2 км от границы; парк из 2–6 бортов моделей geoscan_201 / geoscan_gemini /
geoscan_801 / geoscan_401 (часть бортов привязана к базе, часть — к ближайшей); съёмка RGB с GSD
3–5 см; ветер 0–8 м/с с любого направления. Сценарий целиком определяется своим seed.
"""
from __future__ import annotations

import math
import random

from shapely import affinity
from shapely.geometry import LineString, Point, Polygon

MODELS = ("geoscan_201", "geoscan_gemini", "geoscan_801", "geoscan_401")
SHORT = {"geoscan_201": "201", "geoscan_gemini": "gemini", "geoscan_801": "801", "geoscan_401": "401"}
M_PER_DEG = 111_320.0


def _shape(rng: random.Random, area_m2: float) -> tuple[Polygon, str]:
    n = rng.randint(5, 12)
    step = 2 * math.pi / n
    convex = rng.random() < 0.35
    pts = []
    for k in range(n):
        a = k * step + rng.uniform(-0.35, 0.35) * step
        r = rng.uniform(0.85, 1.0) if convex else rng.uniform(0.45, 1.0)
        pts.append((r * math.cos(a), r * math.sin(a)))
    poly = Polygon(pts)
    if convex:
        poly = poly.convex_hull
    if not poly.is_valid:
        poly = poly.buffer(0)
    stretch = rng.uniform(1.0, 2.5)
    poly = affinity.scale(poly, xfact=stretch, yfact=1.0, origin=(0, 0))
    poly = affinity.rotate(poly, rng.uniform(0, 180), origin=(0, 0))
    k = math.sqrt(area_m2 / poly.area)
    poly = affinity.scale(poly, xfact=k, yfact=k, origin=(0, 0))
    return poly, "выпуклая" if convex else "невыпуклая"


def _nfz(rng: random.Random, area: Polygon, count: int) -> list[Polygon]:
    out: list[Polygon] = []
    minx, miny, maxx, maxy = area.bounds
    for _ in range(count):
        size = min(rng.uniform(0.01, 0.05) * area.area, 1.5e6)
        for _ in range(200):
            c = Point(rng.uniform(minx, maxx), rng.uniform(miny, maxy))
            if rng.random() < 0.5:
                w = math.sqrt(size * rng.uniform(0.5, 2.0))
                z = affinity.rotate(Polygon([(-w / 2, -size / w / 2), (w / 2, -size / w / 2),
                                             (w / 2, size / w / 2), (-w / 2, size / w / 2)]),
                                    rng.uniform(0, 90), origin=(0, 0))
            else:
                z = Point(0, 0).buffer(math.sqrt(size / math.pi), 4)  # восьмиугольник
            z = affinity.translate(z, c.x, c.y)
            # зона с буфером планировщика и запасом — целиком внутри области, не касается других
            if area.buffer(-150).contains(z) and all(z.distance(o) > 300 for o in out):
                out.append(z)
                break
    return out


def _bases(rng: random.Random, area: Polygon, count: int) -> list[tuple[float, float]]:
    out = []
    c = area.centroid
    far = math.sqrt(area.area) * 10
    for _ in range(count):
        phi = rng.uniform(0, 2 * math.pi)
        ray = LineString([(c.x, c.y), (c.x + far * math.cos(phi), c.y + far * math.sin(phi))])
        hit = ray.intersection(area.exterior)
        pts = [hit] if hit.geom_type == "Point" else list(getattr(hit, "geoms", []))
        d_edge = max((math.dist((c.x, c.y), (q.x, q.y)) for q in pts), default=0.0)
        d = d_edge + rng.uniform(300, 2000)
        out.append((c.x + d * math.cos(phi), c.y + d * math.sin(phi)))
    return out


def random_request(seed: int) -> dict:
    """Запрос планирования (JSON-совместимый dict) для заданного seed."""
    rng = random.Random(seed)
    lon0, lat0 = rng.uniform(37.1, 37.9), rng.uniform(55.1, 55.9)
    kx, ky = M_PER_DEG * math.cos(math.radians(lat0)), M_PER_DEG

    def ll(x: float, y: float) -> list[float]:
        return [round(lon0 + x / kx, 6), round(lat0 + y / ky, 6)]

    def ring(p: Polygon) -> list[list[float]]:
        return [ll(x, y) for x, y in p.exterior.coords]

    area_km2 = rng.uniform(2.0, 30.0)
    area, kind = _shape(rng, area_km2 * 1e6)
    zones = _nfz(rng, area, rng.choice((0, 1, 2)))
    bases = _bases(rng, area, rng.choice((1, 2)))
    base_ids = ["A", "B"][: len(bases)]
    n = rng.randint(2, 6)
    drones = []
    for k in range(n):
        m = rng.choice(MODELS)
        d = {"id": f"{SHORT[m]}-{k + 1}", "model": m}
        if rng.random() < 0.7:
            d["base_id"] = rng.choice(base_ids)
        drones.append(d)
    return {
        "_comment": f"случайный сценарий seed={seed}: {kind} область {area_km2:.1f} км², "
                    f"NFZ {len(zones)}, баз {len(bases)}, бортов {n}",
        "survey_area": {"type": "Polygon", "coordinates": [ring(area)]},
        "no_fly_zones": [{"type": "Polygon", "coordinates": [ring(z)]} for z in zones],
        "bases": [{"id": i, "lon": ll(x, y)[0], "lat": ll(x, y)[1]} for i, (x, y) in zip(base_ids, bases)],
        "drones": drones,
        "survey_type": "rgb",
        "requirements": {"gsd_cm": rng.choice((3.0, 3.5, 4.0, 4.5, 5.0))},
        "wind": {"speed_ms": round(rng.uniform(0.0, 8.0), 1), "from_deg": round(rng.uniform(0.0, 359.0))},
        "time_weight": 1.0,
        "use_terrain": False,
    }


def case_seeds(seed: int, n: int) -> list[int]:
    """Seed'ы сценариев набора: из общего seed, чтобы набор N+1 начинался с набора N."""
    rng = random.Random(seed)
    return [rng.randrange(1, 2**31) for _ in range(n)]
