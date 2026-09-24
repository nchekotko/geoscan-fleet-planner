"""Конвертация данных заказчика (KML) в наш формат — GeoJSON с разобранными свойствами.

Исходные файлы (Яндекс.Диск заказчика, ссылка в docs/data_formats.md):
    Московская зона.kml                 — временные и постоянные запретные зоны
    obstacles_Московская область.kml    — высотные препятствия (3D-примитивы)
    высотные препятствия Приморский край.kml
    Границы полетов.kml                 — пример задания на съёмку (696 полигонов)

Запуск (из каталога backend):
    python data/convert_geodata.py ../sources/geodata data/geodata
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planner.geodata import (  # noqa: E402
    features_to_collection,
    parse_obstacles_kml,
    parse_task_kml,
    parse_zones_kml,
)

FILES = [
    ("moscow_zone.kml", "zones_moscow.geojson", parse_zones_kml, "Московская зона.kml"),
    ("obstacles_moscow.kml", "obstacles_moscow.geojson", parse_obstacles_kml, "obstacles_Московская область.kml"),
    ("obstacles_primorsky.kml", "obstacles_primorsky.geojson", parse_obstacles_kml,
     "высотные препятствия Приморский край.kml"),
    ("flight_boundaries.kml", "task_moscow.geojson", parse_task_kml, "Границы полетов.kml"),
]


def main(src_dir: str, out_dir: str) -> None:
    src, out = Path(src_dir), Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for name, target, parse, source in FILES:
        path = src / name
        if not path.exists():
            print(f"нет файла {path} — пропущен")
            continue
        features = parse(str(path), source) if parse is not parse_task_kml else parse(str(path))
        fc = features_to_collection(features)
        fc["name"] = source
        (out / target).write_text(json.dumps(fc, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
        print(f"{source} → {target}: {len(features)} объектов, {(out / target).stat().st_size / 1e6:.1f} МБ")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "../sources/geodata",
         sys.argv[2] if len(sys.argv) > 2 else "data/geodata")
