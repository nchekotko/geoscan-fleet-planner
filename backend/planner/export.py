"""Экспорт индивидуальных полётных заданий.

GeoJSON — RFC 7946: WGS84, [lon, lat, высота AGL]. Участки маршрута — LineString с этапом полёта
в properties, ключевые точки — Point с порядковым номером и действием.
KML — OGC KML 2.2: Document → Folder на вылет → Placemark на участок, altitudeMode relativeToGround.
"""
from __future__ import annotations

import io
import json
import zipfile
from typing import Any

import simplekml

from .schemas import DronePlanOut, PlanResponse

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


def waypoints(d: DronePlanOut) -> list[dict[str, Any]]:
    """Ключевые точки: взлёт, начало/конец каждого галса, посадка."""
    wps: list[dict[str, Any]] = []

    def amsl(leg, k):
        return leg.alt_amsl[k] if leg.alt_amsl else None

    for s in d.sorties:
        for leg in s.legs:
            if leg.kind == "takeoff":
                wps.append({"sortie": s.index + 1, "action": "takeoff", "coord": leg.coordinates[0], "amsl": amsl(leg, 0)})
            elif leg.kind in ("survey", "tie"):
                wps.append({"sortie": s.index + 1, "action": "survey_start", "coord": leg.coordinates[0], "amsl": amsl(leg, 0)})
                wps.append({"sortie": s.index + 1, "action": "survey_end", "coord": leg.coordinates[-1], "amsl": amsl(leg, -1)})
            elif leg.kind == "landing":
                wps.append({"sortie": s.index + 1, "action": "land", "coord": leg.coordinates[-1], "amsl": amsl(leg, -1)})
    for i, w in enumerate(wps, 1):
        w["seq"] = i
    return wps


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
            if leg.kind in ("takeoff", "landing"):
                continue
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
                        "speed_ms": round(d.params.speed_ms, 2),
                        "duration_s": leg.duration_s,
                        "distance_m": leg.distance_m,
                        "altitude_amsl_m": leg.alt_amsl,
                    },
                }
            )
    for w in waypoints(d):
        feats.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": w["coord"]},
                "properties": {
                    "kind": "waypoint",
                    "drone_id": d.drone_id,
                    "seq": w["seq"],
                    "sortie": w["sortie"],
                    "action": w["action"],
                    "altitude_agl_m": w["coord"][2] if len(w["coord"]) > 2 else None,
                    "altitude_amsl_m": w["amsl"],
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
        f"скорость {d.params.speed_ms:.1f} м/с; шаг галсов {d.params.line_spacing_m:.0f} м"
    )
    if plan is not None and (plan.bases or plan.reserve_sites):
        sites = doc.newfolder(name="Площадки")
        for b in plan.bases:
            sites.newpoint(name=f"ВПП {b.id} {b.name}".strip(), coords=[(b.lon, b.lat)])
        for r in plan.reserve_sites:
            sites.newpoint(name=f"Резервная площадка {r.id}", coords=[(r.lon, r.lat)])
    region = doc.newfolder(name="Область борта")
    _kml_region(region, d.region)
    for s in d.sorties:
        f = doc.newfolder(name=f"Вылет {s.index + 1} (база {s.base_id})")
        f.description = (
            f"Начало через {s.start_s / 60:.1f} мин, длительность {s.duration_s / 60:.1f} мин; "
            f"худший уход на площадку {s.max_divert_s / 60:.1f} мин ({s.divert_site})"
        )
        for i, leg in enumerate(s.legs):
            if leg.kind in ("takeoff", "landing"):
                p = f.newpoint(name=f"{i + 1}. {PHASE_RU[leg.kind]}", coords=[tuple(leg.coordinates[0][:2])])
                p.extendeddata.newdata("phase", leg.kind)
                p.extendeddata.newdata("duration_s", leg.duration_s)
                continue
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
            ls.extendeddata.newdata("duration_s", leg.duration_s)
            ls.extendeddata.newdata("distance_m", leg.distance_m)
    return kml.kml()


def _kml_region(folder: simplekml.Folder, geom: dict[str, Any]) -> None:
    polys = [geom["coordinates"]] if geom["type"] == "Polygon" else geom.get("coordinates", [])
    for rings in polys:
        if not rings:
            continue
        pol = folder.newpolygon(name="область", outerboundaryis=[tuple(c[:2]) for c in rings[0]])
        pol.innerboundaryis = [[tuple(c[:2]) for c in r] for r in rings[1:]]
        pol.style.polystyle.color = "3300c800"
        pol.style.linestyle.color = "ff00c800"


def plan_zip(plan: PlanResponse) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for d in plan.drones:
            z.writestr(f"{d.drone_id}.geojson", json.dumps(drone_geojson(d, plan), ensure_ascii=False, indent=1))
            z.writestr(f"{d.drone_id}.kml", drone_kml(d, plan))
        z.writestr("summary.json", plan.summary.model_dump_json(indent=1))
    return buf.getvalue()
