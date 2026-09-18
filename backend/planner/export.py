"""Экспорт индивидуальных полётных заданий.

GeoJSON — RFC 7946: WGS84, [lon, lat, высота AGL]. Участки маршрута — LineString с этапом полёта
и скоростью в properties, ключевые точки — Point с порядковым номером, действием, высотами,
скоростью и временем от начала вылета; ограничения (NFZ, разрешённая зона) — Polygon.
KML — OGC KML 2.2: Document → папки «Площадки», «Ограничения», «Область борта», папка на вылет
с участками (LineString) и вложенной папкой «Ключевые точки» (Point).
"""
from __future__ import annotations

import io
import json
import math
import zipfile
from typing import Any

import simplekml
from shapely.geometry import LineString, shape
from shapely.ops import unary_union

from .schemas import DronePlanOut, LegOut, PlanResponse

PHASE_COLORS = {  # aabbggrr
    "takeoff": "ff00a5ff",
    "transit": "ff9e9e9e",
    "turn": "ffcfcfcf",
    "survey": "ff00c800",
    "tie": "ff00ffff",
    "return": "ffff8c00",
    "landing": "ff0000ff",
}
PHASE_RU = {
    "takeoff": "взлёт и набор высоты",
    "transit": "перелёт к области",
    "turn": "разворот",
    "survey": "съёмка (галс)",
    "tie": "секущий маршрут",
    "return": "возврат",
    "landing": "посадка",
}
ACTION_RU = {
    "takeoff": "взлёт",
    "climb": "набор высоты завершён",
    "waypoint": "поворотная точка",
    "turn": "разворот",
    "survey_start": "начало галса",
    "survey_end": "конец галса",
    "approach": "заход на посадку",
    "land": "посадка",
}
# действия, которые уступают более конкретному при слиянии совпадающих точек соседних участков
GENERIC_ACTIONS = ("waypoint", "turn")
# допуски прореживания вершин разворотов и галсов с огибанием рельефа
THIN_TOL_H_M = 5.0
THIN_TOL_V_M = 2.0


def _xy(lon: float, lat: float, lat0: float) -> tuple[float, float]:
    """Локальные метры (равнопромежуточная проекция; для допусков в несколько метров достаточно)."""
    return math.radians(lon) * 6371000.0 * math.cos(math.radians(lat0)), math.radians(lat) * 6371000.0


def _thin(xy: list[tuple[float, float]], z: list[float] | None) -> list[int]:
    """Дуглас — Пейкер по плану и высоте: убираем вершину, если ломаная без неё отклоняется
    меньше чем на THIN_TOL_H_M по горизонтали и THIN_TOL_V_M по высоте. Концы сохраняются."""
    n = len(xy)
    if n <= 2:
        return list(range(n))
    keep = {0, n - 1}
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        (ax, ay), (bx, by) = xy[a], xy[b]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        worst, wi = 1.0, -1
        for i in range(a + 1, b):
            px, py = xy[i]
            t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            err = math.hypot(px - ax - t * dx, py - ay - t * dy) / THIN_TOL_H_M
            if z is not None:
                err = max(err, abs(z[i] - (z[a] + t * (z[b] - z[a]))) / THIN_TOL_V_M)
            if err > worst:
                worst, wi = err, i
        if wi >= 0:
            keep.add(wi)
            stack += [(a, wi), (wi, b)]
    return sorted(keep)


def _action(kind: str, k: int, n: int) -> str:
    first, last = k == 0, k == n - 1
    if kind == "takeoff":
        return "takeoff" if first else "climb"
    if kind == "landing":
        # самолёт при ветре: заход → раскрытие парашюта → посадка
        return "land" if last else "parachute" if (n >= 3 and k == n - 2) else "approach"
    if kind in ("survey", "tie"):
        return "survey_start" if first else "survey_end" if last else "waypoint"
    return "turn" if kind == "turn" else "waypoint"


def _nfz_union(plan: PlanResponse | None):
    if plan is None or not plan.no_fly_zones:
        return None
    return unary_union([shape(z) for z in plan.no_fly_zones])


def _leg_indices(leg: LegOut, lat0: float, nfz) -> list[int]:
    """Индексы вершин участка, попадающих в ключевые точки. Перелёты, возврат, взлёт и посадка —
    все вершины (в том числе обходы NFZ); развороты и галсы с огибанием рельефа — с прореживанием."""
    n = len(leg.coordinates)
    if leg.kind not in ("turn", "survey", "tie") or n <= 2:
        return list(range(n))
    xy = [_xy(c[0], c[1], lat0) for c in leg.coordinates]
    idx = _thin(xy, leg.alt_amsl)
    if nfz is not None and len(idx) < n:
        full = LineString([c[:2] for c in leg.coordinates])
        thin = LineString([leg.coordinates[i][:2] for i in idx])
        if thin.intersects(nfz) and not full.intersects(nfz):
            return list(range(n))  # срезанная дуга задела бы запретную зону
    return idx


def _leg_times(leg: LegOut, lat0: float) -> list[float]:
    """Время прохождения вершин от начала участка: длительность делится пропорционально пути."""
    n = len(leg.coordinates)
    if n == 1:
        return [0.0]
    xy = [_xy(c[0], c[1], lat0) for c in leg.coordinates]
    cum = [0.0]
    for a, b in zip(xy, xy[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    if cum[-1] < 1e-6:
        return [leg.duration_s * k / (n - 1) for k in range(n)]
    return [leg.duration_s * c / cum[-1] for c in cum]


def waypoints(d: DronePlanOut, plan: PlanResponse | None = None) -> list[dict[str, Any]]:
    """Ключевые точки заданий по вылетам: все вершины участков по порядку (взлёт на земле, конец
    набора высоты, вершины перелётов и обходов NFZ, развороты, начала и концы галсов, заход и
    посадка на земле). Совпадающие точки соседних участков сливаются в одну.

    phase и speed_ms относятся к отрезку, который начинается в точке (у точки посадки
    speed_ms = 0), eta_s — время от начала вылета, с."""
    wps: list[dict[str, Any]] = []
    nfz = _nfz_union(plan)
    for s in d.sorties:
        lat0 = s.legs[0].coordinates[0][1] if s.legs and s.legs[0].coordinates else 0.0
        t0 = 0.0
        prev: dict[str, Any] | None = None
        for leg in s.legs:
            n = len(leg.coordinates)
            times = _leg_times(leg, lat0)
            for k in _leg_indices(leg, lat0, nfz):
                c = leg.coordinates[k]
                amsl = leg.alt_amsl[k] if leg.alt_amsl else None
                action = _action(leg.kind, k, n)
                speed = 0.0 if (leg.kind == "landing" and k == n - 1) else leg.speed_ms
                if prev is not None and prev["coord"] == c and prev["altitude_amsl_m"] == amsl:
                    prev["phase"], prev["speed_ms"] = leg.kind, speed
                    if prev["action"] in GENERIC_ACTIONS:
                        prev["action"] = action
                    continue
                prev = {
                    "sortie": s.index + 1,
                    "phase": leg.kind,
                    "action": action,
                    "coord": c,
                    "altitude_agl_m": round(c[2], 1) if len(c) > 2 else None,
                    "altitude_amsl_m": amsl,
                    "speed_ms": speed,
                    "eta_s": round(t0 + times[k], 1),
                }
                wps.append(prev)
            t0 += leg.duration_s
    for i, w in enumerate(wps, 1):
        w["seq"] = i
    return wps


def _constraints(plan: PlanResponse | None) -> list[tuple[str, int, dict[str, Any]]]:
    if plan is None:
        return []
    out = [("no_fly_zone", i + 1, z) for i, z in enumerate(plan.no_fly_zones)]
    if plan.allowed_area:
        out.append(("allowed_area", 1, plan.allowed_area))
    return out


def drone_geojson(d: DronePlanOut, plan: PlanResponse | None = None) -> dict[str, Any]:
    feats: list[dict[str, Any]] = [
        {
            "type": "Feature",
            "geometry": d.region,
            "properties": {"kind": "region", "drone_id": d.drone_id, "area_km2": d.area_km2},
        }
    ]
    for s in d.sorties:
        for i, leg in enumerate(s.legs):
            # взлёт и посадка — вертикальные участки: LineString из двух точек над базой
            feats.append(
                {
                    "type": "Feature",
                    "geometry": {"type": "LineString", "coordinates": leg.coordinates},
                    "properties": {
                        "kind": "leg",
                        "drone_id": d.drone_id,
                        "sortie": s.index + 1,
                        "step": i + 1,
                        "phase": leg.kind,
                        "phase_ru": PHASE_RU.get(leg.kind, leg.kind),
                        "altitude_agl_m": round(d.params.altitude_agl_m, 1),
                        "speed_ms": leg.speed_ms,
                        "duration_s": leg.duration_s,
                        "distance_m": leg.distance_m,
                        "altitude_amsl_m": leg.alt_amsl,
                    },
                }
            )
    for w in waypoints(d, plan):
        feats.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": w["coord"]},
                "properties": {
                    "kind": "waypoint",
                    "drone_id": d.drone_id,
                    "seq": w["seq"],
                    "sortie": w["sortie"],
                    "phase": w["phase"],
                    "action": w["action"],
                    "altitude_agl_m": w["altitude_agl_m"],
                    "altitude_amsl_m": w["altitude_amsl_m"],
                    "speed_ms": w["speed_ms"],
                    "eta_s": w["eta_s"],
                },
            }
        )
    if plan is not None:
        for site, kind in [(b, "base") for b in plan.bases] + [(r, "reserve_site") for r in plan.reserve_sites]:
            feats.append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [site.lon, site.lat]},
                "properties": {"kind": kind, "id": site.id, "name": site.name},
            })
    for kind, i, geom in _constraints(plan):
        feats.append({"type": "Feature", "geometry": geom, "properties": {"kind": kind, "index": i}})
    return {
        "type": "FeatureCollection",
        # foreign member (RFC 7946, п. 6.1): параметры задания
        "mission": {
            "drone_id": d.drone_id,
            "model": d.model_name,
            "payload": d.payload,
            "altitude_agl_m": round(d.params.altitude_agl_m, 1),
            "speed_ms": round(d.params.speed_ms, 2),
            "line_spacing_m": round(d.params.line_spacing_m, 1),
            "gsd_cm": d.params.gsd_cm and round(d.params.gsd_cm, 2),
            "trigger_interval_s": d.params.trigger_interval_s and round(d.params.trigger_interval_s, 2),
            "sorties": [
                {"index": s.index + 1, "base_id": s.base_id, "start_s": s.start_s, "duration_s": s.duration_s,
                 "max_divert_s": s.max_divert_s, "divert_site": s.divert_site}
                for s in d.sorties
            ],
            "flight_time_s": d.flight_time_s,
            "finish_s": d.finish_s,
        },
        "features": feats,
    }


def drone_kml(d: DronePlanOut, plan: PlanResponse | None = None) -> str:
    kml = simplekml.Kml(name=f"{d.drone_id} — {d.model_name}")
    doc = kml.document
    doc.description = (
        f"Нагрузка: {d.payload}; высота {d.params.altitude_agl_m:.0f} м AGL; "
        f"скорость съёмки {d.params.speed_ms:.1f} м/с; шаг галсов {d.params.line_spacing_m:.0f} м"
    )
    if plan is not None and (plan.bases or plan.reserve_sites):
        sites = doc.newfolder(name="Площадки")
        for b in plan.bases:
            sites.newpoint(name=f"ВПП {b.id} {b.name}".strip(), coords=[(b.lon, b.lat)])
        for r in plan.reserve_sites:
            sites.newpoint(name=f"Резервная площадка {r.id}", coords=[(r.lon, r.lat)])
    constraints = _constraints(plan)
    if constraints:
        lim = doc.newfolder(name="Ограничения")
        for kind, i, geom in constraints:
            if kind == "no_fly_zone":
                _kml_polygons(lim, geom, f"Запретная зона {i}", "4d0000ff", "ff0000ff")
            else:
                _kml_polygons(lim, geom, "Разрешённая зона", "00000000", "ffff8c00")
    region = doc.newfolder(name="Область борта")
    _kml_polygons(region, d.region, "область", "3300c800", "ff00c800")
    by_sortie: dict[int, list[dict[str, Any]]] = {}
    for w in waypoints(d, plan):
        by_sortie.setdefault(w["sortie"], []).append(w)
    for s in d.sorties:
        f = doc.newfolder(name=f"Вылет {s.index + 1} (база {s.base_id})")
        f.description = (
            f"Начало через {s.start_s / 60:.1f} мин, длительность {s.duration_s / 60:.1f} мин; "
            f"худший уход на площадку {s.max_divert_s / 60:.1f} мин ({s.divert_site})"
        )
        for i, leg in enumerate(s.legs):
            if leg.kind in ("takeoff", "landing"):
                continue  # вертикальные участки — в ключевых точках
            if leg.alt_amsl:
                # есть рельеф: абсолютные высоты (над геоидом EGM2008)
                coords = [(c[0], c[1], a) for c, a in zip(leg.coordinates, leg.alt_amsl)]
                mode = simplekml.AltitudeMode.absolute
            else:
                coords = [tuple(c) for c in leg.coordinates]
                mode = simplekml.AltitudeMode.relativetoground
            ls = f.newlinestring(name=f"{i + 1}. {PHASE_RU.get(leg.kind, leg.kind)}", coords=coords)
            ls.altitudemode = mode
            ls.style.linestyle.color = PHASE_COLORS.get(leg.kind, "ffffffff")
            ls.style.linestyle.width = 3 if leg.kind == "survey" else 2
            ls.extendeddata.newdata("phase", leg.kind)
            ls.extendeddata.newdata("speed_ms", leg.speed_ms)
            ls.extendeddata.newdata("duration_s", leg.duration_s)
            ls.extendeddata.newdata("distance_m", leg.distance_m)
        kp = f.newfolder(name="Ключевые точки")
        for w in by_sortie.get(s.index + 1, []):
            lon, lat = w["coord"][:2]
            if w["altitude_amsl_m"] is not None:
                p = kp.newpoint(coords=[(lon, lat, w["altitude_amsl_m"])])
                p.altitudemode = simplekml.AltitudeMode.absolute
            else:
                p = kp.newpoint(coords=[(lon, lat, w["altitude_agl_m"] or 0.0)])
                p.altitudemode = simplekml.AltitudeMode.relativetoground
            p.name = f"{w['seq']}. {ACTION_RU.get(w['action'], w['action'])}"
            for key in ("phase", "action", "speed_ms", "altitude_agl_m", "altitude_amsl_m", "eta_s"):
                p.extendeddata.newdata(key, "" if w[key] is None else w[key])
    return kml.kml()


def _kml_polygons(folder: simplekml.Folder, geom: dict[str, Any], name: str, fill: str, line: str) -> None:
    polys = [geom["coordinates"]] if geom["type"] == "Polygon" else geom.get("coordinates", [])
    for rings in polys:
        if not rings:
            continue
        pol = folder.newpolygon(name=name, outerboundaryis=[tuple(c[:2]) for c in rings[0]])
        pol.innerboundaryis = [[tuple(c[:2]) for c in r] for r in rings[1:]]
        pol.style.polystyle.color = fill
        pol.style.linestyle.color = line


def plan_zip(plan: PlanResponse) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for d in plan.drones:
            z.writestr(f"{d.drone_id}.geojson", json.dumps(drone_geojson(d, plan), ensure_ascii=False, indent=1))
            z.writestr(f"{d.drone_id}.kml", drone_kml(d, plan))
        z.writestr("summary.json", plan.summary.model_dump_json(indent=1))
    return buf.getvalue()
