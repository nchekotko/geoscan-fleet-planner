# Сервис планирования и распределения беспилотных авиационных работ

Хакатон «Лидеры цифровой трансформации 2026», задача Geoscan. План и первоисточники: [PLAN.md](PLAN.md).

## Запуск для разработки

Нужны Python 3.12+ и Node.js 20+. Внешние ключи не требуются: подложка OpenStreetMap.

```bash
# backend
cd backend
python -m venv .venv
.venv/Scripts/python -m pip install fastapi "uvicorn[standard]" pydantic shapely pyproj numpy networkx ortools simplekml pyyaml pytest httpx
.venv/Scripts/python -m uvicorn api.main:app --host 127.0.0.1 --port 8000 --reload
# тесты
.venv/Scripts/python -m pytest -q

# frontend (в другом терминале)
cd frontend
npm install
npm run dev -- --host 127.0.0.1
```

Интерфейс: http://127.0.0.1:5173. Документация API (OpenAPI): http://127.0.0.1:8000/docs.

## Структура

- `backend/data/fleet.yaml` — датасет бортов и нагрузок со ссылками на паспорта и пометками допущений.
- `backend/data/scenarios/` — тестовые сценарии.
- `backend/planner/` — ядро: `sensors` (параметры съёмки), `coverage` (галсы), `dubins`/`turns` (развороты),
  `wind`, `energy`, `avoid` (обход NFZ), `partition` (разбиение между бортами), `mission` (вылеты),
  `planner` (оркестрация и критерии), `export` (GeoJSON/KML).
- `backend/api/` — FastAPI.
- `frontend/` — React + TypeScript + Leaflet.
