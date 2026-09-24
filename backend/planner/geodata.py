"""Данные о воздушном пространстве: зоны ограничений и высотные препятствия.

Заказчик выдал примеры в KML (`Московская зона.kml`, `obstacles_Московская область.kml`,
`Границы полетов.kml`). В них диапазон высот и время действия ограничения записаны свободным
текстом («От земли до 500 м (1700 фут) AMSL», «От FL280 до FL400»), стандартного формата нет —
заказчик предложил придумать свой. Здесь такой формат и разбор исходных файлов в него:
GeoJSON-Feature с разобранными полями в properties (описание — в docs/data_formats.md).

Зачем разбирать высоты: зона «от 800 м AMSL до FL90» не мешает съёмке на 150 м над землёй,
а «от земли до 500 м AMSL» — мешает. Без разбора пришлось бы считать запретными все зоны
подряд и терять площадь съёмки.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Any, Iterable, Literal

KML_NS = {"k": "http://www.opengis.net/kml/2.2", "gx": "http://www.google.com/kml/ext/2.2"}

FT_M = 0.3048
FL_FT = 100.0  # FL090 — 9000 футов по стандартному давлению

# отсчёт высоты: земля, над рельефом, над уровнем моря
AltRef = Literal["GND", "AGL", "AMSL"]

# типы зон в примере заказчика (styleUrl / поле Type) → наши коды
ZONE_TYPES: dict[str, str] = {
    "запретная_зона": "prohibited",
    "пост_ограничение": "permanent",
    "врем_ограничение": "temporary",
    "svo": "special",
}
ZONE_TYPE_NAMES: dict[str, str] = {
    "prohibited": "запретная зона",
    "permanent": "постоянное ограничение",
    "temporary": "временное ограничение",
    "special": "особый режим",
    "unknown": "ограничение",
}


@dataclass
class AltBand:
    """Диапазон высот ограничения: нижняя и верхняя границы со своим отсчётом."""

    lower_m: float = 0.0
    lower_ref: AltRef = "GND"
    upper_m: float = math.inf
    upper_ref: AltRef = "AMSL"
    raw: str = ""
    notes: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {
            "lower_m": self.lower_m,
            "lower_ref": self.lower_ref,
            "upper_m": None if math.isinf(self.upper_m) else round(self.upper_m, 1),
            "upper_ref": self.upper_ref,
            "raw": self.raw,
        }

    def agl_span(self, ground_min_m: float, ground_max_m: float) -> tuple[float, float]:
        """Диапазон в метрах над землёй. Отсчёт AMSL переводится по рельефу, причём
        консервативно: нижняя граница — по самой высокой земле участка, верхняя — по самой
        низкой. Так зона скорее попадёт в расчёт, чем будет пропущена."""
        lo = _to_agl(self.lower_m, self.lower_ref, ground_max_m)
        hi = _to_agl(self.upper_m, self.upper_ref, ground_min_m)
        return lo, hi

    def blocks(self, alt_agl_m: float, ground_min_m: float = 0.0, ground_max_m: float = 0.0) -> bool:
        """Мешает ли зона полёту на высоте alt_agl_m над землёй."""
        lo, hi = self.agl_span(ground_min_m, ground_max_m)
        return hi > 0.0 and lo <= alt_agl_m


def _to_agl(value: float, ref: AltRef, ground_m: float) -> float:
    if math.isinf(value):
        return value
    if ref == "GND":
        return 0.0
    if ref == "AGL":
        return value
    return value - ground_m


@dataclass
class TimeWindow:
    """Время действия ограничения. В примере заказчика его нет — поля появляются в нашем
    формате, чтобы временные зоны (`врем_ограничение`) можно было задать по-настоящему."""

    start: datetime | None = None
    end: datetime | None = None
    daily_from_min: int | None = None  # минуты от полуночи местного времени
    daily_to_min: int | None = None
    raw: str = ""

    def as_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        if self.start is not None:
            out["from"] = self.start.isoformat()
        if self.end is not None:
            out["to"] = self.end.isoformat()
        if self.daily_from_min is not None and self.daily_to_min is not None:
            out["daily"] = [_hhmm(self.daily_from_min), _hhmm(self.daily_to_min)]
        if self.raw:
            out["raw"] = self.raw
        return out

    def active(self, t0: datetime | None, t1: datetime | None) -> bool:
        """Действует ли ограничение хотя бы в один момент окна работ [t0, t1].
        Без времени работ считаем, что действует (консервативно)."""
        if t0 is None or t1 is None:
            return True
        if self.end is not None and self.end <= t0:
            return False
        if self.start is not None and self.start >= t1:
            return False
        if self.daily_from_min is not None and self.daily_to_min is not None:
            return _daily_overlap(t0, t1, self.daily_from_min, self.daily_to_min)
        return True


def _hhmm(minutes: int) -> str:
    return f"{minutes // 60:02d}:{minutes % 60:02d}"


def _daily_overlap(t0: datetime, t1: datetime, from_min: int, to_min: int) -> bool:
    """Пересекается ли окно работ с суточным интервалом действия (в том числе через полночь)."""
    day = t0.replace(hour=0, minute=0, second=0, microsecond=0)
    while day <= t1:
        a = day + timedelta(minutes=from_min)
        b = day + timedelta(minutes=to_min if to_min > from_min else to_min + 24 * 60)
        if a < t1 and b > t0:
            return True
        day += timedelta(days=1)
    return False


# ------------------------------------------------------------------ разбор высот
_NUM = r"(\d[\d\s ]*(?:[.,]\d+)?)"
_RE_FL = re.compile(r"FL\s*0*(\d+)", re.I)
_RE_M = re.compile(_NUM + r"\s*м(?:етр\w*)?\b", re.I)
_RE_FT_ONLY = re.compile(_NUM + r"\s*(?:фут\w*|ft)\b", re.I)
_RE_GND = re.compile(r"от\s+(земли|земной поверхности|поверхности|GND)", re.I)
_RE_ALL = re.compile(r"на\s+всех\s+высотах|все\s+высоты", re.I)


def parse_alt_band(text: str) -> AltBand:
    """Разбор строки высот из данных заказчика.

    Понимает «От земли до 500 м (1700 фут) AMSL», «От FL280 до FL400»,
    «От 800 м (2700 фут) AMSL до FL90», «На всех высотах», пустую строку. Всё, что не
    относится к высотам (например, «Некорректная геометрия!» или оговорки про воздушные
    суда), уходит в notes; сама строка сохраняется в raw.
    """
    raw = (text or "").replace(" ", " ").strip()
    head = raw.split("\n")[0].strip()
    band = AltBand(raw=raw)
    band.notes = "\n".join(p.strip() for p in raw.split("\n")[1:] if p.strip())
    if not head or _RE_ALL.search(head):
        # высоты не заданы или «на всех высотах» — считаем, что ограничение от земли и выше
        return band
    if m := re.match(r"^([^.!?]*[!?])\s*(.*)$", head):
        # ведущая оговорка вроде «Некорректная геометрия!» — в notes
        prefix, rest = m.group(1).strip(), m.group(2).strip()
        if not re.search(r"\d|FL", prefix, re.I):
            band.notes = "\n".join(x for x in (prefix, band.notes) if x)
            head = rest
    parts = re.split(r"\bдо\b", head, maxsplit=1, flags=re.I)
    lower_txt = parts[0]
    upper_txt = parts[1] if len(parts) > 1 else ""
    if not upper_txt:
        # одна граница: «от земли» — снизу, всё остальное считаем верхней границей
        if _RE_GND.search(lower_txt):
            return band
        upper_txt, lower_txt = lower_txt, ""
    lo = _parse_one(lower_txt)
    hi = _parse_one(upper_txt)
    if lo is not None:
        band.lower_m, band.lower_ref = lo
    if hi is not None:
        band.upper_m, band.upper_ref = hi
    return band


def _parse_one(text: str) -> tuple[float, AltRef] | None:
    """Одна граница диапазона: значение в метрах и отсчёт."""
    t = (text or "").strip()
    if not t:
        return None
    if _RE_GND.search(t) or re.fullmatch(r"земли|земля|GND", t, re.I):
        return 0.0, "GND"
    if m := _RE_FL.search(t):
        return int(m.group(1)) * FL_FT * FT_M, "AMSL"
    ref: AltRef = "AGL" if re.search(r"\bAGL\b|над\s+земл", t, re.I) else "AMSL"
    if m := _RE_M.search(t):
        return float(m.group(1).replace(" ", "").replace(",", ".")), ref
    if m := _RE_FT_ONLY.search(t):
        return float(m.group(1).replace(" ", "").replace(",", ".")) * FT_M, ref
    return None


# ------------------------------------------------------------------ разбор KML
def _coords(text: str) -> list[tuple[float, float, float]]:
    """Координаты KML: «lon,lat[,alt]» через пробелы. В файлах заказчика встречаются пробелы
    после запятых и лишние разделители, поэтому текст сначала нормализуется."""
    norm = re.sub(r"\s*,\s*", ",", (text or "").strip())
    out = []
    for tok in norm.split():
        parts = tok.split(",")
        if len(parts) < 2:
            continue
        try:
            lon, lat = float(parts[0]), float(parts[1])
            alt = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
        except ValueError:
            continue
        out.append((lon, lat, alt))
    return out


def _ring(el: ET.Element) -> list[list[float]]:
    return [[round(lon, 7), round(lat, 7)] for lon, lat, _ in _coords(el.findtext("k:coordinates", "", KML_NS))]


def _placemark_polygons(pm: ET.Element) -> list[list[list[list[float]]]]:
    """Полигоны метки: список полигонов, каждый — внешнее кольцо и дырки."""
    polys = []
    for poly in pm.findall(".//k:Polygon", KML_NS):
        rings = []
        for outer in poly.findall("k:outerBoundaryIs/k:LinearRing", KML_NS):
            rings.append(_ring(outer))
        for inner in poly.findall("k:innerBoundaryIs/k:LinearRing", KML_NS):
            rings.append(_ring(inner))
        rings = [r for r in rings if len(r) >= 4]
        if rings:
            polys.append(rings)
    return polys


def _placemark_lines(pm: ET.Element) -> list[list[list[float]]]:
    lines = []
    for ls in pm.findall(".//k:LineString", KML_NS):
        pts = [[round(lon, 7), round(lat, 7)] for lon, lat, _ in _coords(ls.findtext("k:coordinates", "", KML_NS))]
        if len(pts) >= 2:
            lines.append(pts)
    return lines


def _placemark_top_m(pm: ET.Element) -> tuple[float, AltRef]:
    """Верх препятствия: максимальная высота в координатах и её отсчёт (altitudeMode)."""
    zs = [z for el in pm.findall(".//k:coordinates", KML_NS) for _, _, z in _coords(el.text or "")]
    mode = (pm.findtext(".//k:altitudeMode", "", KML_NS) or "").strip().lower()
    ref: AltRef = "AGL" if mode in ("relativetoground", "clamptoground") else "AMSL"
    return (max(zs) if zs else 0.0), ref


def _multi_polygon(polys: list[list[list[list[float]]]]) -> dict[str, Any] | None:
    if not polys:
        return None
    if len(polys) == 1:
        return {"type": "Polygon", "coordinates": polys[0]}
    return {"type": "MultiPolygon", "coordinates": polys}


def _data_fields(pm: ET.Element) -> dict[str, str]:
    out = {}
    for d in pm.findall(".//k:Data", KML_NS):
        name = d.get("name") or ""
        out[name] = (d.findtext("k:value", "", KML_NS) or "").strip()
    for sd in pm.findall(".//k:SimpleData", KML_NS):
        out[sd.get("name") or ""] = (sd.text or "").strip()
    return out


def _zone_type(pm: ET.Element, fields: dict[str, str]) -> str:
    raw = fields.get("Type") or pm.findtext("k:styleUrl", "", KML_NS) or ""
    for key, code in ZONE_TYPES.items():
        if key in raw:
            return code
    return "unknown"


def parse_zones_kml(path: str, source: str = "") -> list[dict[str, Any]]:
    """Зоны ограничений из KML заказчика → Feature нашего формата (kind = "restriction")."""
    root = ET.parse(path).getroot()
    src = source or path.replace("\\", "/").rsplit("/", 1)[-1]
    out = []
    for i, pm in enumerate(root.findall(".//k:Placemark", KML_NS)):
        geom = _multi_polygon(_placemark_polygons(pm))
        if geom is None:
            continue
        fields = _data_fields(pm)
        name = (fields.get("Name") or pm.findtext("k:name", "", KML_NS) or "").strip()
        alt_txt = fields.get("Altitudes") or pm.findtext("k:description", "", KML_NS) or ""
        band = parse_alt_band(alt_txt)
        props: dict[str, Any] = {
            "kind": "restriction",
            "id": name or f"zone-{i + 1}",
            "name": name,
            "zone_type": _zone_type(pm, fields),
            "alt": band.as_dict(),
            "source": src,
        }
        if band.notes:
            props["notes"] = band.notes
        out.append({"type": "Feature", "geometry": geom, "properties": props})
    return out


def parse_obstacles_kml(path: str, source: str = "") -> list[dict[str, Any]]:
    """Высотные препятствия из KML заказчика (3D-примитивы: полигон или линия с высотой
    в координатах) → Feature нашего формата (kind = "obstacle")."""
    root = ET.parse(path).getroot()
    src = source or path.replace("\\", "/").rsplit("/", 1)[-1]
    out = []
    for i, pm in enumerate(root.findall(".//k:Placemark", KML_NS)):
        name = (pm.findtext("k:name", "", KML_NS) or "").strip()
        ident, _, kind = name.partition(" ")
        top, ref = _placemark_top_m(pm)
        geom = _multi_polygon(_placemark_polygons(pm))
        if geom is None:
            lines = _placemark_lines(pm)
            if not lines:
                continue
            geom = ({"type": "LineString", "coordinates": lines[0]} if len(lines) == 1
                    else {"type": "MultiLineString", "coordinates": lines})
        props = {
            "kind": "obstacle",
            "id": ident or f"obstacle-{i + 1}",
            "obstacle_type": _obstacle_type(kind),
            "top_m": round(top, 1),
            "top_ref": ref,
            "source": src,
        }
        out.append({"type": "Feature", "geometry": geom, "properties": props})
    return out


def _obstacle_type(raw: str) -> str:
    """Тип препятствия: «OTHER:COMMUNICATION_TOWER» → COMMUNICATION_TOWER."""
    t = (raw or "").strip().split(":")[-1].strip()
    t = re.sub(r"\s+", "_", t)
    return t or "UNKNOWN"


def parse_task_kml(path: str) -> list[dict[str, Any]]:
    """Полигоны задания на съёмку (файл «Границы полетов.kml»): по одному Feature на метку."""
    root = ET.parse(path).getroot()
    out = []
    for i, pm in enumerate(root.findall(".//k:Placemark", KML_NS)):
        geom = _multi_polygon(_placemark_polygons(pm))
        if geom is None:
            continue
        fields = _data_fields(pm)
        props = {"kind": "survey_area", "id": fields.get("FID") or f"area-{i + 1}"}
        if name := (pm.findtext("k:name", "", KML_NS) or "").strip():
            props["name"] = name
        out.append({"type": "Feature", "geometry": geom, "properties": props})
    return out


# ------------------------------------------------------------------ наш формат
@dataclass
class Restriction:
    """Зона ограничения в нашем формате (разобранная из Feature)."""

    geometry: dict[str, Any]
    id: str = ""
    name: str = ""
    zone_type: str = "unknown"
    band: AltBand = field(default_factory=AltBand)
    window: TimeWindow = field(default_factory=TimeWindow)
    notes: str = ""

    @property
    def title(self) -> str:
        what = ZONE_TYPE_NAMES.get(self.zone_type, ZONE_TYPE_NAMES["unknown"])
        return f"{what} {self.name or self.id}".strip()


def restriction_from_feature(f: dict[str, Any]) -> Restriction:
    """Feature нашего формата → Restriction. Полигон без properties — зона на всех высотах,
    действующая всегда (так работают простые `no_fly_zones` из прежних запросов)."""
    geom = f.get("geometry") if f.get("type") == "Feature" else f
    props = (f.get("properties") or {}) if isinstance(f, dict) else {}
    alt = props.get("alt") or {}
    band = AltBand(
        lower_m=float(alt.get("lower_m") or 0.0),
        lower_ref=alt.get("lower_ref") or ("GND" if alt.get("lower_m") in (None, 0) else "AMSL"),
        upper_m=math.inf if alt.get("upper_m") is None else float(alt["upper_m"]),
        upper_ref=alt.get("upper_ref") or "AMSL",
        raw=alt.get("raw") or "",
    )
    act = props.get("active") or {}
    window = TimeWindow(
        start=_dt(act.get("from")),
        end=_dt(act.get("to")),
        daily_from_min=_minutes((act.get("daily") or [None, None])[0]),
        daily_to_min=_minutes((act.get("daily") or [None, None])[1]),
        raw=act.get("raw") or "",
    )
    return Restriction(
        geometry=geom or {},
        id=str(props.get("id") or ""),
        name=str(props.get("name") or ""),
        zone_type=str(props.get("zone_type") or "unknown"),
        band=band,
        window=window,
        notes=str(props.get("notes") or ""),
    )


@dataclass
class Obstacle:
    """Высотное препятствие в нашем формате."""

    geometry: dict[str, Any]
    id: str = ""
    obstacle_type: str = "UNKNOWN"
    top_m: float = 0.0
    top_ref: AltRef = "AGL"

    def top_agl(self, ground_m: float = 0.0, obstacle_ground_m: float | None = None) -> float:
        """Верх препятствия в метрах над землёй в точке съёмки. Для препятствий, заданных
        над уровнем моря, вычитаем высоту земли под ними (если известна) или под областью."""
        if self.top_ref == "AGL":
            return self.top_m
        return self.top_m - (obstacle_ground_m if obstacle_ground_m is not None else ground_m)


def obstacle_from_feature(f: dict[str, Any]) -> Obstacle:
    geom = f.get("geometry") if f.get("type") == "Feature" else f
    props = (f.get("properties") or {}) if isinstance(f, dict) else {}
    return Obstacle(
        geometry=geom or {},
        id=str(props.get("id") or ""),
        obstacle_type=str(props.get("obstacle_type") or "UNKNOWN"),
        top_m=float(props.get("top_m") or 0.0),
        top_ref=props.get("top_ref") or "AGL",
    )


def _dt(v: Any) -> datetime | None:
    if not v:
        return None
    if isinstance(v, datetime):
        return v
    try:
        return datetime.fromisoformat(str(v).replace("Z", "+00:00"))
    except ValueError:
        return None


def _minutes(v: Any) -> int | None:
    if not v:
        return None
    m = re.match(r"(\d{1,2}):(\d{2})", str(v))
    return int(m.group(1)) * 60 + int(m.group(2)) if m else None


def features_to_collection(features: Iterable[dict[str, Any]]) -> dict[str, Any]:
    return {"type": "FeatureCollection", "features": list(features)}
