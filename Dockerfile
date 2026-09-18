# Сборка интерфейса
FROM node:20-alpine AS web
WORKDIR /web
COPY frontend/package.json frontend/package-lock.json ./
RUN npm ci
COPY frontend/ ./
RUN npm run build

# Сервис: API + собранный интерфейс
FROM python:3.12-slim
WORKDIR /app
# libexpat нужен колесу rasterio (чтение рельефа)
RUN apt-get update && apt-get install -y --no-install-recommends libexpat1 && rm -rf /var/lib/apt/lists/*
COPY backend/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt
COPY backend/ ./
COPY --from=web /web/dist ./static
# Тайлы рельефа для демо-районов скачиваются при сборке, чтобы демо работало без сети.
# Формат: "lat:lon lat:lon"; пустое значение — не скачивать.
ARG DEM_TILES="55:37 55:38"
RUN python -c "import sys; from planner.terrain import _tile_path; \
[print(t, _tile_path(int(t.split(':')[0]), int(t.split(':')[1]), True)) for t in sys.argv[1:]]" ${DEM_TILES}
ENV STATIC_DIR=/app/static
EXPOSE 8000
CMD ["uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "8000"]
