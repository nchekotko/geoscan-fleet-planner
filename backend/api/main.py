"""HTTP API сервиса планирования. Документация OpenAPI: /docs."""
from __future__ import annotations

import json
import uuid
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response

from planner.export import drone_geojson, drone_kml, plan_zip
from planner.fleet import load_fleet
from planner.planner import PlanningError, plan
from planner.schemas import PlanRequest, PlanResponse

SCENARIOS = Path(__file__).resolve().parent.parent / "data" / "scenarios"

app = FastAPI(title="Geoscan Fleet Planner", version="0.1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# Рассчитанные планы держим в памяти процесса: для демо этого достаточно.
_plans: dict[str, PlanResponse] = {}


@app.get("/api/health")
def health() -> dict[str, str]:
    return {"status": "ok"}


@app.get("/api/fleet")
def fleet() -> dict:
    return load_fleet().model_dump()


@app.get("/api/scenarios")
def scenarios() -> list[str]:
    return sorted(p.stem for p in SCENARIOS.glob("*.json"))


@app.get("/api/scenarios/{name}")
def scenario(name: str) -> dict:
    path = SCENARIOS / f"{name}.json"
    if not path.is_file() or path.parent != SCENARIOS:
        raise HTTPException(404, "сценарий не найден")
    return json.loads(path.read_text(encoding="utf-8"))


@app.post("/api/plan")
def make_plan(req: PlanRequest) -> dict:
    try:
        result = plan(req)
    except PlanningError as e:
        raise HTTPException(422, str(e)) from e
    except ValueError as e:
        raise HTTPException(422, f"план не построен: {e}") from e
    plan_id = uuid.uuid4().hex[:12]
    _plans[plan_id] = result
    return {"plan_id": plan_id, **result.model_dump()}


def _get(plan_id: str) -> PlanResponse:
    if plan_id not in _plans:
        raise HTTPException(404, "план не найден, пересчитайте")
    return _plans[plan_id]


@app.get("/api/plan/{plan_id}/export/{drone_id}.{fmt}")
def export_drone(plan_id: str, drone_id: str, fmt: str) -> Response:
    p = _get(plan_id)
    d = next((x for x in p.drones if x.drone_id == drone_id), None)
    if d is None:
        raise HTTPException(404, "борт не найден в плане")
    headers = {"Content-Disposition": f'attachment; filename="{drone_id}.{fmt}"'}
    if fmt == "geojson":
        return JSONResponse(drone_geojson(d, p), media_type="application/geo+json", headers=headers)
    if fmt == "kml":
        return Response(drone_kml(d, p), media_type="application/vnd.google-earth.kml+xml", headers=headers)
    raise HTTPException(400, "формат: geojson или kml")


@app.get("/api/plan/{plan_id}/export.zip")
def export_all(plan_id: str) -> Response:
    data = plan_zip(_get(plan_id))
    return Response(
        data,
        media_type="application/zip",
        headers={"Content-Disposition": f'attachment; filename="plan_{plan_id}.zip"'},
    )
