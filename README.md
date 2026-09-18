# Сервис планирования и распределения беспилотных авиационных работ

Хакатон «Лидеры цифровой трансформации 2026», задача Geoscan.

Веб-сервис рассчитывает оптимальные полётные задания для парка БВС Геоскан (201, Gemini, 801, 401): делит область съёмки между бортами с учётом их ТТХ, заряда, ветра, запретных зон, разрешённого воздушного пространства и рельефа, строит маршруты с галсами, разворотами и вылетами, оптимизирует по времени выполнения работ или суммарному налёту (включая фронт Парето) и экспортирует индивидуальные задания в KML и GeoJSON.

## Быстрый запуск (Docker)

```bash
docker compose up --build
```

Откройте http://127.0.0.1:8000. Внешние ключи и доступы не нужны: подложка — OpenStreetMap, рельеф демо-районов скачивается при сборке из открытого бакета Copernicus DEM (без ключа). Первая сборка — 3–5 минут.

## Запуск для разработки

Нужны Python 3.12+ и Node.js 20+.

```bash
# backend
cd backend
python -m venv .venv
.venv/Scripts/python -m pip install -r requirements-dev.txt     # Linux/macOS: .venv/bin/python
.venv/Scripts/python -m uvicorn api.main:app --host 127.0.0.1 --port 8000
.venv/Scripts/python -m pytest -q                                # тесты
.venv/Scripts/python bench/benchmark.py                          # бенчмарк → docs/benchmark.md

# frontend (в другом терминале)
cd frontend
npm install
npm run dev -- --host 127.0.0.1
```

Интерфейс: http://127.0.0.1:5173. API и OpenAPI: http://127.0.0.1:8000/docs.

## Документация

- [Архитектура](docs/architecture.md)
- [Алгоритм оптимизации](docs/algorithm.md)
- [API](docs/api.md)
- [Руководство пользователя](docs/user_guide.md)
- [Известные ограничения](docs/limitations.md)
- [Бенчмарк](docs/benchmark.md)
- [План работ и первоисточники](PLAN.md)

## Структура

```
backend/
  data/fleet.yaml        датасет бортов и нагрузок (ТТХ со ссылками на паспорта, допущения помечены)
  data/scenarios/        тестовые сценарии (генератор — data/make_scenarios.py)
  planner/               ядро: съёмка, галсы, развороты, ветер, заряд, обход NFZ,
                         распределение по парку, фронт Парето, рельеф, экспорт
  api/main.py            FastAPI
  bench/benchmark.py     сравнение с базовыми подходами, масштабируемость
  tests/                 тесты (pytest)
frontend/src/            React + TypeScript + Leaflet
docs/                    документация
sources/                 первоисточники ТТХ (PDF из ТЗ и руководство Геоскан 401)
```
