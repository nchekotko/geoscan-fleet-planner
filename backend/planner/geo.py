"""Перевод WGS84 ↔ локальная метрическая проекция (азимутальная равнопромежуточная с центром
в районе работ). Вся геометрия ядра считается в метрах."""
from __future__ import annotations

from dataclasses import dataclass

from pyproj import CRS, Transformer
from shapely.geometry.base import BaseGeometry
from shapely.ops import transform


@dataclass(frozen=True)
class LocalFrame:
    lon0: float
    lat0: float

    def __post_init__(self) -> None:
        crs = CRS.from_proj4(f"+proj=aeqd +lat_0={self.lat0} +lon_0={self.lon0} +datum=WGS84 +units=m")
        object.__setattr__(self, "_fwd", Transformer.from_crs("EPSG:4326", crs, always_xy=True))
        object.__setattr__(self, "_inv", Transformer.from_crs(crs, "EPSG:4326", always_xy=True))

    @classmethod
    def around(cls, geom: BaseGeometry) -> "LocalFrame":
        c = geom.centroid
        return cls(lon0=c.x, lat0=c.y)

    def to_local(self, geom: BaseGeometry) -> BaseGeometry:
        return transform(self._fwd.transform, geom)

    def to_wgs(self, geom: BaseGeometry) -> BaseGeometry:
        return transform(self._inv.transform, geom)

    def xy_to_lonlat(self, x: float, y: float) -> tuple[float, float]:
        return self._inv.transform(x, y)

    def lonlat_to_xy(self, lon: float, lat: float) -> tuple[float, float]:
        return self._fwd.transform(lon, lat)
