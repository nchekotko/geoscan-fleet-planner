"""Рельеф: Copernicus DEM GLO-30 (DSM, 30 м, высоты над геоидом EGM2008).

Тайлы 1°×1° берутся из открытого бакета AWS без ключа (registry.opendata.aws/copernicus-dem)
и кэшируются в data/dem/. Если сети нет и тайла нет в кэше — рельеф не используется,
а план строится относительно точки старта с предупреждением.
"""
from __future__ import annotations

import math
import os
import urllib.request
from functools import lru_cache
from pathlib import Path

import numpy as np

# Атрибуция по условиям лицензии Copernicus DEM
TERRAIN_ATTRIBUTION = (
    "Copernicus DEM GLO-30 (DSM, EGM2008): © DLR e.V. 2010–2014 и © Airbus Defence and Space GmbH "
    "2014–2018, предоставлено в рамках COPERNICUS Европейским союзом и ESA"
)
DEM_DIR = Path(os.environ.get("DEM_DIR", Path(__file__).resolve().parent.parent / "data" / "dem"))
URL = "https://copernicus-dem-30m.s3.amazonaws.com/{name}/{name}.tif"


def tile_name(lat: int, lon: int) -> str:
    ns = "N" if lat >= 0 else "S"
    ew = "E" if lon >= 0 else "W"
    return f"Copernicus_DSM_COG_10_{ns}{abs(lat):02d}_00_{ew}{abs(lon):03d}_00_DEM"


def _tile_path(lat: int, lon: int, download: bool) -> Path | None:
    name = tile_name(lat, lon)
    path = DEM_DIR / f"{name}.tif"
    if path.exists():
        return path
    if not download or os.environ.get("DEM_OFFLINE") == "1":
        return None
    DEM_DIR.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".part")
    try:
        with urllib.request.urlopen(URL.format(name=name), timeout=60) as r, open(tmp, "wb") as f:
            while chunk := r.read(1 << 20):
                f.write(chunk)
        tmp.replace(path)
        return path
    except Exception:
        tmp.unlink(missing_ok=True)
        return None


@lru_cache(maxsize=8)
def _open(path: str):
    import rasterio

    ds = rasterio.open(path)
    return ds, ds.read(1)


class Terrain:
    """Высоты рельефа в точках (lon, lat). available=False — данных нет."""

    def __init__(self, bounds: tuple[float, float, float, float], download: bool = True):
        minx, miny, maxx, maxy = bounds
        self.tiles: dict[tuple[int, int], Path] = {}
        self.missing: list[str] = []
        for la in range(math.floor(miny), math.floor(maxy) + 1):
            for lo in range(math.floor(minx), math.floor(maxx) + 1):
                p = _tile_path(la, lo, download)
                if p is None:
                    self.missing.append(tile_name(la, lo))
                else:
                    self.tiles[(la, lo)] = p
        self.available = bool(self.tiles) and not self.missing
        if self.available:
            try:
                for p in self.tiles.values():
                    _open(str(p))
            except Exception:  # нет rasterio/GDAL или битый файл — работаем без рельефа
                self.available = False

    def heights(self, lonlat: list[tuple[float, float]]) -> np.ndarray:
        out = np.full(len(lonlat), np.nan)
        for i, (lon, lat) in enumerate(lonlat):
            p = self.tiles.get((math.floor(lat), math.floor(lon)))
            if p is None:
                continue
            ds, arr = _open(str(p))
            r, c = ds.index(lon, lat)
            r = min(max(r, 0), arr.shape[0] - 1)
            c = min(max(c, 0), arr.shape[1] - 1)
            out[i] = float(arr[r, c])
        return out
