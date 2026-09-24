"""HTTP API сервиса планирования. Документация OpenAPI: /docs."""
from __future__ import annotations

import json
import logging
import os
import re
import threading
import uuid
from pathlib import Path
from urllib.parse import quote

from fastapi import FastAPI, HTTPException, Request
from fastapi.encoders import jsonable_encoder
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import Field

from planner.advisor import advise
from planner.export import drone_geojson, drone_kml, plan_zip
from planner.fleet import load_fleet
from planner.pareto import pareto_front
from planner.planner import PlanningError, plan
from planner.schemas import PlanRequest, PlanResponse

SCENARIOS = Path(__file__).resolve().parent.parent / "data" / "scenarios"
GEODATA = Path(__file__).resolve().parent.parent / "data" / "geodata"
log = logging.getLogger("geoscan.api")

app = FastAPI(title="Geoscan Fleet Planner", version="0.1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


@app.middleware("http")
async def _unexpected_errors(request: Request, call_next):
    """Непредвиденная ошибка → JSON 500 с коротким сообщением для интерфейса, трассировка — в лог."""
    try:
        return await call_next(request)
    except Exception:  # noqa: BLE001
        log.exception("ошибка при обработке %s %s", request.method, request.url.path)
        return JSONResponse(
            {"detail": "внутренняя ошибка планировщика: план не построен, проверьте входные данные"},
            status_code=500,
        )


# Частые ошибки pydantic по-русски; остальные — исходным текстом.
_VALIDATION_RU = {
    "missing": "обязательное поле",
    "less_than_equal": "должно быть не больше {le}",
    "less_than": "должно быть меньше {lt}",
    "greater_than_equal": "должно быть не меньше {ge}",
    "greater_than": "должно быть больше {gt}",
    "finite_number": "должно быть конечным числом",
    "float_parsing": "нужно число",
    "float_type": "нужно число",
    "too_short": "нужно хотя бы {min_length} знач.",
    "string_too_short": "не может быть пустым",
    "literal_error": "допустимые значения: {expected}",
}


@app.exception_handler(RequestValidationError)
async def _validation_error(request: Request, exc: RequestValidationError) -> JSONResponse:
    """422 с читаемой строкой в detail (её показывает интерфейс) и полным списком в errors."""
    parts = []
    for e in exc.errors():
        loc = ".".join(str(x) for x in e.get("loc", ()) if x != "body")
        tpl = _VALIDATION_RU.get(e.get("type", ""))
        try:
            msg = tpl.format(**e.get("ctx", {})) if tpl else str(e.get("msg", "")).removeprefix("Value error, ")
        except (KeyError, IndexError):
            msg = str(e.get("msg", ""))
        parts.append(f"{loc}: {msg}" if loc else msg)
    return JSONResponse(
        {"detail": "некорректные входные данные — " + "; ".join(parts),
         # без input: в нём может быть NaN, который не сериализуется в JSON
         "errors": jsonable_encoder([{k: e[k] for k in ("type", "loc", "msg") if k in e} for e in exc.errors()])},
        status_code=422,
    )


def _attachment(filename: str) -> dict[str, str]:
    """Content-Disposition по RFC 6266: ASCII-запасное имя и filename* в UTF-8."""
    fallback = re.sub(r'[^A-Za-z0-9._-]', "_", filename)
    return {"Content-Disposition": f"attachment; filename=\"{fallback}\"; filename*=UTF-8''{quote(filename, safe='')}"}

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


# Фронт Парето считается в пуле процессов на все ядра — одновременно только один расчёт
_pareto_lock = threading.Semaphore(1)


@app.post("/api/pareto")
def make_pareto(req: PlanRequest) -> list[dict]:
    """Фронт Парето: недоминируемые планы по времени работ и суммарному налёту."""
    if not _pareto_lock.acquire(timeout=120):
        raise HTTPException(503, "сервис занят расчётом другого фронта Парето — повторите через минуту")
    try:
        front = pareto_front(req)
    except PlanningError as e:
        raise HTTPException(422, str(e)) from e
    except ValueError as e:
        raise HTTPException(422, f"фронт Парето не построен: {e}") from e
    finally:
        _pareto_lock.release()
    if not front:
        raise HTTPException(422, "не удалось построить ни одного плана")
    out = []
    for p in front:
        plan_id = uuid.uuid4().hex[:12]
        _plans[plan_id] = p
        out.append({"plan_id": plan_id, **p.model_dump()})
    return out


class AdviceRequest(PlanRequest):
    """Запрос к советнику: тот же план плюс ограничения пользователя."""

    deadline_s: float | None = Field(None, gt=0.0)   # срок выполнения работ, с
    max_drones: int | None = Field(None, ge=1)       # сколько бортов разрешено занять


@app.post("/api/advise")
def make_advice(req: AdviceRequest) -> dict:
    """Подсказки по ограничениям: выполнимо ли задание, сколько бортов нужно к сроку,
    сколько времени займут работы заданным числом бортов."""
    if not _pareto_lock.acquire(timeout=120):
        raise HTTPException(503, "сервис занят другим расчётом — повторите через минуту")
    try:
        plan_req = PlanRequest(**req.model_dump(exclude={"deadline_s", "max_drones"}))
        return advise(plan_req, deadline_s=req.deadline_s, max_drones=req.max_drones).model_dump()
    except PlanningError as e:
        raise HTTPException(422, str(e)) from e
    except ValueError as e:
        raise HTTPException(422, f"подсказка не построена: {e}") from e
    finally:
        _pareto_lock.release()


@app.get("/api/geodata")
def list_geodata() -> list[dict]:
    """Выгрузки данных о воздушном пространстве, лежащие рядом с сервисом."""
    out = []
    for p in sorted(GEODATA.glob("*.geojson")):
        try:
            head = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        out.append({"name": p.stem, "title": head.get("name", p.stem),
                    "features": len(head.get("features", [])), "size_kb": round(p.stat().st_size / 1024)})
    return out


@app.get("/api/geodata/{name}")
def get_geodata(name: str) -> Response:
    """Выгрузка целиком (GeoJSON нашего формата): зоны ограничений, препятствия, задание."""
    if not re.fullmatch(r"[a-z0-9_]+", name):
        raise HTTPException(404, "нет такой выгрузки")
    path = GEODATA / f"{name}.geojson"
    if not path.exists():
        raise HTTPException(404, "нет такой выгрузки")
    return Response(path.read_bytes(), media_type="application/geo+json")


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
    headers = _attachment(f"{drone_id}.{fmt}")
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
        headers=_attachment(f"plan_{plan_id}.zip"),
    )


# Собранный интерфейс (в Docker — /app/static). Монтируется последним, чтобы не перекрывать /api.
_static = Path(os.environ.get("STATIC_DIR", Path(__file__).resolve().parent.parent.parent / "frontend" / "dist"))
if _static.is_dir():
    app.mount("/", StaticFiles(directory=_static, html=True), name="web")
