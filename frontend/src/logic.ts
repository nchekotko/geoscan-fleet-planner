// Чистые функции состояния интерфейса (без React) — покрыты тестами в tests/logic.test.ts.
import type { MultiPolygon, Polygon } from 'geojson'
import type {
  AirspaceInfo,
  AreaFeature,
  DroneInstance,
  Maintenance,
  ObstacleFeature,
  PlanRequest,
  PlanResponse,
  RestrictionFeature,
  Site,
  SurveyType,
  ZoneType,
} from './api'

/** Пустой запрос: с него начинается интерфейс, им же дополняются загруженные сценарии. */
export const EMPTY_REQUEST: PlanRequest = {
  survey_area: null,
  allowed_area: null,
  no_fly_zones: [],
  bases: [],
  reserve_sites: [],
  drones: [],
  survey_type: 'rgb',
  requirements: { gsd_cm: 3, front_overlap: 0.75, side_overlap: 0.65, lidar_density_pts_m2: 50 },
  wind: { speed_ms: 0, from_deg: 0 },
  time_weight: 1,
  reserve: 0.2,
  nfz_buffer_m: 30,
  restrictions: [],
  obstacles: [],
  obstacle_clearance_m: 30,
  obstacle_buffer_m: 50,
  mission_start: null,
  mission_window_h: 12,
}

/** Ключ входных данных для сравнения «план посчитан по этим данным или нет».
 *  Вес критерия не входит: фронт Парето от него не зависит, для одиночного плана он сравнивается отдельно. */
export function requestKey(req: PlanRequest): string {
  const { time_weight: _w, ...rest } = req
  return JSON.stringify(rest)
}

/** Первый свободный id вида `${prefix}${n}`, n = 1, 2, … */
export function nextFreeId(prefix: string, existing: Iterable<string>): string {
  const used = new Set(existing)
  for (let n = 1; ; n++) if (!used.has(`${prefix}${n}`)) return `${prefix}${n}`
}

/** Первая свободная буква для ВПП (A…Z), дальше — A1, A2, … */
export function nextBaseId(existing: Iterable<string>): string {
  const used = new Set(existing)
  for (let i = 0; i < 26; i++) {
    const id = String.fromCharCode(65 + i)
    if (!used.has(id)) return id
  }
  return nextFreeId('A', used)
}

/** id борта для модели: geoscan_gemini → gemini-1, gemini-2, … (первый свободный). */
export function nextDroneId(model: string, drones: DroneInstance[]): string {
  return nextFreeId(`${model.replace('geoscan_', '')}-`, drones.map((d) => d.id))
}

/** Ошибки id бортов по индексу строки: пустой или повторяющийся id. */
export function droneIdErrors(drones: DroneInstance[]): Record<number, string> {
  const count = new Map<string, number>()
  drones.forEach((d) => count.set(d.id.trim(), (count.get(d.id.trim()) ?? 0) + 1))
  const out: Record<number, string> = {}
  drones.forEach((d, i) => {
    const id = d.id.trim()
    if (!id) out[i] = 'id борта не может быть пустым'
    else if ((count.get(id) ?? 0) > 1) out[i] = `id «${id}» уже занят другим бортом`
  })
  return out
}

/** План фронта получен методом ε-ограничения (а не взвешенной суммой)?
 *  Если бэкенд отдаёт makespan_cap_s — верим ему; иначе ε-планы узнаём по w = 1 при времени работ
 *  больше минимального на фронте (взвешенный план с w = 1 — самый быстрый). */
export function isEpsilonPlan(p: PlanResponse, front: PlanResponse[]): boolean {
  if (p.makespan_cap_s !== undefined) return p.makespan_cap_s !== null
  if (p.time_weight < 1) return false
  const fastest = Math.min(...front.map((q) => q.summary.makespan_s))
  return p.summary.makespan_s > fastest + 1
}

/** Как был выбран показанный план. */
export type Criterion =
  | { kind: 'weighted'; w: number }
  | { kind: 'front'; w: number | null } // null — ε-ограничение

const fmtW = (w: number) => String(Math.round(w * 100) / 100)

export function criterionLabel(c: Criterion): string {
  if (c.kind === 'front')
    return c.w === null ? 'точка фронта Парето (ε-ограничение)' : `точка фронта Парето (w=${fmtW(c.w)})`
  if (c.w >= 1) return 'время работ'
  if (c.w <= 0) return 'налёт'
  return `компромисс w=${fmtW(c.w)}`
}

/** Покрытие с одним знаком без округления вверх: 99.96 % — это «99.9 %», а не «100.0 %». */
export function formatCoverage(pct: number): string {
  return (Math.floor(pct * 10 + 1e-9) / 10).toFixed(1)
}

export const COVERAGE_OK_PCT = 99.5

export interface ExcludedInfo {
  drone_id: string
  reason: string
}

/** Разбор поля detail ответа 4xx: строка, список ошибок pydantic или объект с message/excluded. */
export function parseErrorDetail(detail: unknown, status: number): { message: string; items: string[]; excluded: ExcludedInfo[] } {
  const res = { message: `Ошибка ${status}`, items: [] as string[], excluded: [] as ExcludedInfo[] }
  if (typeof detail === 'string') {
    res.message = detail
  } else if (Array.isArray(detail)) {
    res.message = 'Некорректные входные данные:'
    res.items = detail.map(formatValidationItem)
  } else if (detail && typeof detail === 'object') {
    const d = detail as { message?: unknown; msg?: unknown; excluded?: unknown; errors?: unknown }
    const msg = d.message ?? d.msg
    if (typeof msg === 'string') res.message = msg
    if (Array.isArray(d.errors)) res.items = d.errors.map(formatValidationItem)
    if (Array.isArray(d.excluded))
      res.excluded = d.excluded
        .filter((e): e is ExcludedInfo => !!e && typeof e === 'object' && 'drone_id' in e)
        .map((e) => ({ drone_id: String(e.drone_id), reason: String(e.reason ?? '') }))
    if (typeof msg !== 'string' && !res.items.length && !res.excluded.length) res.message = JSON.stringify(detail)
  }
  return res
}

/** Одна ошибка pydantic → «drones[0].id: сообщение». Префикс body отбрасываем. */
export function formatValidationItem(e: unknown): string {
  if (typeof e === 'string') return e
  if (!e || typeof e !== 'object') return String(e)
  const { loc, msg } = e as { loc?: unknown; msg?: unknown }
  let path = ''
  if (Array.isArray(loc)) {
    for (const part of loc[0] === 'body' ? loc.slice(1) : loc) {
      path += typeof part === 'number' ? `[${part}]` : path ? `.${part}` : String(part)
    }
  }
  const text = typeof msg === 'string' ? msg : JSON.stringify(e)
  return path ? `${path}: ${text}` : text
}

// ---------- Геометрия: проверка координат и контуров ----------

type LonLat = [number, number]
type Area = Polygon | MultiPolygon

const isObject = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v)
const stripBom = (s: string) => (s.charCodeAt(0) === 0xfeff ? s.slice(1) : s)

/** Координата [долгота, широта] в WGS84 (высота отбрасывается) или текст ошибки. */
export function toLonLat(c: unknown): LonLat | string {
  if (!Array.isArray(c) || c.length < 2) return 'координата должна быть парой [долгота, широта]'
  const [lon, lat] = c as unknown[]
  if (typeof lon !== 'number' || typeof lat !== 'number' || !Number.isFinite(lon) || !Number.isFinite(lat))
    return 'координаты должны быть конечными числами'
  if (Math.abs(lon) > 180 || Math.abs(lat) > 90)
    return `координата (${lon}, ${lat}) вне диапазона: долгота ±180°, широта ±90° (нужна WGS84)`
  return [lon, lat]
}

/** Удвоенная ориентированная площадь незамкнутого контура, град². */
function ringArea2(r: LonLat[]): number {
  let s = 0
  for (let i = 0; i < r.length; i++) {
    const [x1, y1] = r[i]
    const [x2, y2] = r[(i + 1) % r.length]
    s += x1 * y2 - x2 * y1
  }
  return s
}

/** Контур полигона: координаты проверены, повторы подряд убраны, не меньше трёх вершин, замкнут. */
function normalizeRing(ring: unknown): LonLat[] | string {
  if (!Array.isArray(ring)) return 'контур должен быть массивом координат'
  const out: LonLat[] = []
  for (const c of ring) {
    const p = toLonLat(c)
    if (typeof p === 'string') return p
    const last = out[out.length - 1]
    if (!last || last[0] !== p[0] || last[1] !== p[1]) out.push(p)
  }
  if (out.length < 3) return 'в контуре меньше трёх вершин'
  const first = out[0]
  const last = out[out.length - 1]
  if (first[0] === last[0] && first[1] === last[1]) out.pop()
  if (out.length < 3) return 'в контуре меньше трёх вершин'
  if (Math.abs(ringArea2(out)) < 1e-12) return 'контур нулевой площади'
  out.push([first[0], first[1]])
  return out
}

/** GeoJSON Polygon из массива контуров (первый — внешний, остальные — дыры) или текст ошибки. */
export function normalizePolygon(rings: unknown): Polygon | string {
  if (!Array.isArray(rings) || rings.length === 0) return 'нет внешнего контура'
  const coordinates: LonLat[][] = []
  for (const [i, r] of rings.entries()) {
    const ring = normalizeRing(r)
    if (typeof ring === 'string') return i === 0 ? ring : `внутренний контур ${i}: ${ring}`
    coordinates.push(ring)
  }
  return { type: 'Polygon', coordinates }
}

/** Полигон из контуров, но вырожденные дыры отбрасываются, а не делают весь полигон ошибкой:
 *  в данных заказчика такие дыры есть (бэкенд чистит их make_valid), и терять из-за одной
 *  лишней дыры всю область съёмки нельзя. */
export function normalizePolygonDropHoles(rings: unknown): Polygon | string {
  if (!Array.isArray(rings) || rings.length === 0) return 'нет внешнего контура'
  const outer = normalizeRing(rings[0])
  if (typeof outer === 'string') return outer
  const holes = rings.slice(1).map(normalizeRing).filter((r): r is LonLat[] => typeof r !== 'string')
  return { type: 'Polygon', coordinates: [outer, ...holes] }
}

/** Геометрия после правки на карте: вырожденные дыры отбрасываются, части без внешнего контура — тоже.
 *  null — не осталось ни одного корректного полигона (правку нужно откатить). */
export function cleanPolygonGeometry(g: Area): Area | null {
  const parts: Polygon[] = []
  for (const rings of g.type === 'Polygon' ? [g.coordinates] : g.coordinates) {
    const outer = normalizeRing(rings[0])
    if (typeof outer === 'string') continue
    const holes = rings.slice(1).map(normalizeRing).filter((r): r is LonLat[] => typeof r !== 'string')
    parts.push({ type: 'Polygon', coordinates: [outer, ...holes] })
  }
  if (!parts.length) return null
  return g.type === 'Polygon' ? parts[0] : { type: 'MultiPolygon', coordinates: parts.map((p) => p.coordinates) }
}

/** Охват [запад, юг, восток, север]. */
export type BBox = [number, number, number, number]

/** Охват полигонов и точек; null — охватывать нечего. */
export function bboxOf(areas: (Area | null | undefined)[], points: { lon: number; lat: number }[] = []): BBox | null {
  const b: BBox = [Infinity, Infinity, -Infinity, -Infinity]
  const add = (lon: number, lat: number) => {
    b[0] = Math.min(b[0], lon)
    b[1] = Math.min(b[1], lat)
    b[2] = Math.max(b[2], lon)
    b[3] = Math.max(b[3], lat)
  }
  for (const a of areas) {
    if (!a) continue
    for (const poly of a.type === 'Polygon' ? [a.coordinates] : a.coordinates)
      for (const ring of poly) for (const [lon, lat] of ring) add(lon, lat)
  }
  points.forEach((p) => add(p.lon, p.lat))
  return Number.isFinite(b[0]) ? b : null
}

/** Что показать после загрузки сценария: область съёмки, запретные зоны и площадки
 *  (разрешённая зона бывает огромной — берём её, только если больше ничего нет). */
export function requestBBox(req: PlanRequest): BBox | null {
  return bboxOf([req.survey_area, ...req.no_fly_zones], [...req.bases, ...req.reserve_sites]) ?? bboxOf([req.allowed_area])
}

// ---------- Импорт геометрии из файлов (GeoJSON, KML) ----------

/** Точка из файла — будущая ВПП или резервная площадка. */
export interface ImportedPoint {
  lon: number
  lat: number
  name?: string
}

/** Что удалось взять из файла; warnings — что пропущено и почему (показываем пользователю). */
export interface ImportedGeometry {
  polygons: Polygon[]
  points: ImportedPoint[]
  warnings: string[]
}

const MAX_INVALID_SHOWN = 5

/** Накопитель результата импорта: проверяет координаты и собирает понятные предупреждения. */
function importCollector() {
  const res: ImportedGeometry = { polygons: [], points: [], warnings: [] }
  const skipped = new Map<string, number>()
  const invalid: string[] = []
  let nPoly = 0
  let nPoint = 0
  const label = (name: string | undefined, n: number) => (name ? `«${name}»` : `№${n}`)
  return {
    polygon(rings: unknown, name?: string) {
      nPoly++
      const p = normalizePolygon(rings)
      if (typeof p === 'string') invalid.push(`Пропущен полигон ${label(name, nPoly)}: ${p}`)
      else res.polygons.push(p)
    },
    point(c: unknown, name?: string) {
      nPoint++
      const p = toLonLat(c)
      if (typeof p === 'string') invalid.push(`Пропущена точка ${label(name, nPoint)}: ${p}`)
      else res.points.push(name ? { lon: p[0], lat: p[1], name } : { lon: p[0], lat: p[1] })
    },
    skip(type: string) {
      skipped.set(type, (skipped.get(type) ?? 0) + 1)
    },
    done(): ImportedGeometry {
      if (skipped.size)
        res.warnings.push(
          'Пропущены объекты других типов (импортируются только полигоны и точки): ' +
            [...skipped].map(([t, n]) => (n > 1 ? `${t} ×${n}` : t)).join(', '),
        )
      res.warnings.push(...invalid.slice(0, MAX_INVALID_SHOWN))
      if (invalid.length > MAX_INVALID_SHOWN)
        res.warnings.push(`…и ещё ${invalid.length - MAX_INVALID_SHOWN} объект(ов) с некорректными координатами`)
      return res
    },
  }
}

const GEOJSON_TYPES = new Set([
  'FeatureCollection',
  'Feature',
  'GeometryCollection',
  'Point',
  'MultiPoint',
  'LineString',
  'MultiLineString',
  'Polygon',
  'MultiPolygon',
])

const featureName = (props: unknown): string | undefined => {
  if (!isObject(props)) return undefined
  const n = props.name ?? props.Name ?? props.title
  if (typeof n === 'number') return String(n)
  return typeof n === 'string' && n.trim() ? n.trim() : undefined
}

/** GeoJSON (FeatureCollection, Feature или голая геометрия) → полигоны и точки.
 *  MultiPolygon/MultiPoint раскладываются на части, прочие типы пропускаются с предупреждением.
 *  Бросает Error, если это не JSON или не GeoJSON. */
export function parseGeoJSONText(text: string): ImportedGeometry {
  let data: unknown
  try {
    data = JSON.parse(stripBom(text))
  } catch (e) {
    throw new Error(`Файл не является корректным JSON: ${(e as Error).message}`)
  }
  if (!isObject(data) || typeof data.type !== 'string' || !GEOJSON_TYPES.has(data.type)) {
    if (isObject(data) && ('survey_area' in data || 'drones' in data))
      throw new Error('Это файл сценария, а не геометрии — используйте «Загрузить сценарий»')
    throw new Error('Файл не похож на GeoJSON: ожидается FeatureCollection, Feature или геометрия с полем type')
  }
  const c = importCollector()
  const visit = (o: unknown, name?: string): void => {
    if (!isObject(o)) return c.skip('не объект')
    const coords = o.coordinates
    const parts = (f: (x: unknown) => void) => (Array.isArray(coords) ? coords.forEach(f) : f(undefined))
    switch (o.type) {
      case 'FeatureCollection':
        if (Array.isArray(o.features)) o.features.forEach((f) => visit(f))
        break
      case 'Feature':
        if (o.geometry == null) c.skip('Feature без геометрии')
        else visit(o.geometry, featureName(o.properties))
        break
      case 'GeometryCollection':
        if (Array.isArray(o.geometries)) o.geometries.forEach((g) => visit(g, name))
        break
      case 'Polygon':
        c.polygon(coords, name)
        break
      case 'MultiPolygon':
        parts((p) => c.polygon(p, name))
        break
      case 'Point':
        c.point(coords, name)
        break
      case 'MultiPoint':
        parts((p) => c.point(p, name))
        break
      default:
        c.skip(typeof o.type === 'string' ? o.type : 'без типа')
    }
  }
  visit(data)
  return c.done()
}

/** Строка <coordinates> KML: кортежи «lon,lat[,alt]» через пробелы/переводы строк → массив чисел. */
export function parseKmlCoordinates(s: string): number[][] {
  return s
    .trim()
    .replace(/\s*,\s*/g, ',')
    .split(/\s+/)
    .filter(Boolean)
    .map((t) => t.split(',').map((v) => (v === '' ? NaN : Number(v))))
}

/** Узел XML: имя без префикса пространства имён, дочерние элементы, собственный текст. */
interface XmlNode {
  name: string
  children: XmlNode[]
  text: string
}

// комментарий | CDATA | <?…?> | <!…> | закрывающий тег | открывающий (или пустой) тег | текст
const XML_TOKEN =
  /<!--[\s\S]*?-->|<!\[CDATA\[([\s\S]*?)\]\]>|<\?[\s\S]*?\?>|<![^>]*>|<\/\s*([^\s>]+)\s*>|<([^\s/>]+)(?:"[^"]*"|'[^']*'|[^'">])*?(\/?)>|([^<]+)/g

const XML_ENTITIES: Record<string, string> = { amp: '&', lt: '<', gt: '>', quot: '"', apos: "'" }

const decodeXmlEntities = (s: string) =>
  s.replace(/&(#x[0-9a-f]+|#\d+|[a-z]+);/gi, (all, e: string) => {
    if (e[0] !== '#') return XML_ENTITIES[e] ?? all
    const cp = e[1] === 'x' || e[1] === 'X' ? parseInt(e.slice(2), 16) : Number(e.slice(1))
    return cp >= 0 && cp <= 0x10ffff ? String.fromCodePoint(cp) : all
  })

const localName = (tag: string) => tag.slice(tag.indexOf(':') + 1)

/** Лёгкий разбор XML без DOM (нужен только для KML; работает и в node --test).
 *  Прощает незакрытые теги: закрывающий тег снимает стек до ближайшего одноимённого. */
function parseXml(src: string): XmlNode {
  const root: XmlNode = { name: '', children: [], text: '' }
  const stack = [root]
  for (const [, cdata, close, open, selfClose, text] of src.matchAll(XML_TOKEN)) {
    const top = stack[stack.length - 1]
    if (cdata !== undefined) top.text += cdata
    else if (close !== undefined) {
      const name = localName(close)
      const i = stack.findLastIndex((n) => n.name === name)
      if (i > 0) stack.length = i
    } else if (open !== undefined) {
      const node: XmlNode = { name: localName(open), children: [], text: '' }
      top.children.push(node)
      if (!selfClose) stack.push(node)
    } else if (text !== undefined) top.text += decodeXmlEntities(text)
  }
  return root
}

/** Все потомки с данным именем (внутрь найденных не спускаемся). */
const findAll = (n: XmlNode, name: string, out: XmlNode[] = []): XmlNode[] => {
  for (const c of n.children) {
    if (c.name === name) out.push(c)
    else findAll(c, name, out)
  }
  return out
}

/** Геометрии KML, которые не импортируем (линии, треки, 3D-модели). */
const KML_SKIPPED = new Set(['LineString', 'LinearRing', 'Track', 'MultiTrack', 'Model'])

/** KML → полигоны (outerBoundaryIs + innerBoundaryIs) и точки, в т.ч. внутри MultiGeometry и папок.
 *  Имя точки — <name> её Placemark. Бросает Error, если файл не похож на KML. */
export function parseKMLText(text: string): ImportedGeometry {
  const src = stripBom(text).trim()
  if (src.startsWith('PK')) throw new Error('Это KMZ (сжатый KML) — распакуйте архив и импортируйте doc.kml')
  const root = parseXml(src)
  if (!['kml', 'Document', 'Folder', 'Placemark'].some((t) => findAll(root, t).length))
    throw new Error('Файл не похож на KML: нет элементов kml, Document или Placemark')
  const c = importCollector()
  const firstCoords = (n: XmlNode) => parseKmlCoordinates(findAll(n, 'coordinates')[0]?.text ?? '')
  const walk = (node: XmlNode, name?: string): void => {
    for (const ch of node.children) {
      if (ch.name === 'Placemark') {
        walk(ch, ch.children.find((x) => x.name === 'name')?.text.trim() || undefined)
      } else if (ch.name === 'Polygon') {
        const outer = ch.children.find((x) => x.name === 'outerBoundaryIs')
        const inner = ch.children
          .filter((x) => x.name === 'innerBoundaryIs')
          .flatMap((b) => findAll(b, 'coordinates'))
          .map((x) => parseKmlCoordinates(x.text))
        c.polygon(outer ? [firstCoords(outer), ...inner] : [], name)
      } else if (ch.name === 'Point') {
        c.point(firstCoords(ch)[0], name)
      } else if (KML_SKIPPED.has(ch.name)) {
        c.skip(ch.name)
      } else walk(ch, name)
    }
  }
  walk(root)
  return c.done()
}

/** Разбор файла геометрии: .kml — KML, .geojson/.json — GeoJSON, иначе по содержимому. */
export function parseGeometryFile(fileName: string, text: string): ImportedGeometry {
  const ext = /\.([^.]+)$/.exec(fileName.toLowerCase())?.[1] ?? ''
  const src = stripBom(text).trimStart()
  if (ext === 'kmz' || src.startsWith('PK'))
    throw new Error('KMZ (сжатый KML) не поддерживается — распакуйте архив и импортируйте doc.kml')
  if (ext === 'kml' || ext === 'xml' || (ext !== 'json' && ext !== 'geojson' && src.startsWith('<')))
    return parseKMLText(src)
  return parseGeoJSONText(src)
}

/** Куда назначить импортированное: полигоны и точки — независимо; null — не импортировать. */
export type PolygonTarget = 'survey' | 'allowed' | 'nfz'
export type PointTarget = 'base' | 'reserve'
export interface ImportChoice {
  polygons: PolygonTarget | null
  points: PointTarget | null
}

/** Применить импорт к запросу. Область съёмки — один полигон или MultiPolygon из всех;
 *  разрешённая зона — первый полигон; запретные зоны, ВПП и резервные площадки добавляются к имеющимся. */
export function applyImport(req: PlanRequest, imp: ImportedGeometry, choice: ImportChoice): PlanRequest {
  let r = req
  const polys = imp.polygons
  if (choice.polygons && polys.length) {
    if (choice.polygons === 'survey')
      r = {
        ...r,
        survey_area: polys.length === 1 ? polys[0] : { type: 'MultiPolygon', coordinates: polys.map((p) => p.coordinates) },
      }
    else if (choice.polygons === 'allowed') r = { ...r, allowed_area: polys[0] }
    else r = { ...r, no_fly_zones: [...r.no_fly_zones, ...polys] }
  }
  if (choice.points && imp.points.length) {
    // id ВПП и резервных площадок не должны пересекаться (это проверяет и бэкенд)
    const used = [...r.bases, ...r.reserve_sites].map((s) => s.id)
    const take = (id: string) => {
      used.push(id)
      return id
    }
    if (choice.points === 'base')
      r = {
        ...r,
        bases: [
          ...r.bases,
          ...imp.points.map((p) => ({ id: take(nextBaseId(used)), name: p.name ?? '', lon: p.lon, lat: p.lat })),
        ],
      }
    else
      r = {
        ...r,
        reserve_sites: [
          ...r.reserve_sites,
          ...imp.points.map((p) => ({
            id: take(nextFreeId('R', used)),
            ...(p.name ? { name: p.name } : {}),
            lon: p.lon,
            lat: p.lat,
          })),
        ],
      }
  }
  return r
}

// ---------- Правка геометрии на карте ----------

/** Какой входной объект правили на карте. */
export type EditTarget = { kind: 'survey' } | { kind: 'allowed' } | { kind: 'nfz' | 'base' | 'reserve'; index: number }

export type EditValue = Area | { lon: number; lat: number }

/** Записать правку с карты в запрос (через него же работает признак «план устарел»). */
export function applyEdit(req: PlanRequest, target: EditTarget, value: EditValue): PlanRequest {
  if ('type' in value) {
    if (target.kind === 'survey') return { ...req, survey_area: value }
    if (target.kind === 'allowed') return { ...req, allowed_area: value }
    if (target.kind === 'nfz') {
      const parts: Polygon[] =
        value.type === 'Polygon' ? [value] : value.coordinates.map((coordinates) => ({ type: 'Polygon', coordinates }))
      const zones = [...req.no_fly_zones]
      zones.splice(target.index, 1, ...parts)
      return { ...req, no_fly_zones: zones }
    }
    return req
  }
  const move = (sites: Site[], i: number) => sites.map((s, j) => (j === i ? { ...s, lon: value.lon, lat: value.lat } : s))
  if (target.kind === 'base') return { ...req, bases: move(req.bases, target.index) }
  if (target.kind === 'reserve') return { ...req, reserve_sites: move(req.reserve_sites, target.index) }
  return req
}

// ---------- Сценарий: сохранение и загрузка файла ----------

/** Ошибка разбора файла сценария со списком замечаний. */
export class ScenarioError extends Error {
  items: string[]
  constructor(message: string, items: string[] = []) {
    super(message)
    this.items = items
  }
}

/** Текущие входные данные как файл сценария (формат backend/data/scenarios/*.json). */
export function scenarioJSON(req: PlanRequest): string {
  return JSON.stringify(req, null, 2) + '\n'
}

const SURVEY_TYPE_IDS: SurveyType[] = ['rgb', 'multispectral', 'thermal', 'lidar', 'geophysics']
const REQUIREMENT_NUMBERS = [
  'gsd_cm',
  'front_overlap',
  'side_overlap',
  'altitude_m',
  'lidar_density_pts_m2',
  'lidar_side_overlap',
  'line_spacing_m',
  'altitude_ceiling_m',
]

/** Сценарий (объект запроса на планирование) → PlanRequest с умолчаниями из EMPTY_REQUEST.
 *  Проверяются геометрия, площадки, борта и числовые поля; неизвестные поля (use_terrain и т. п.) сохраняются.
 *  Бросает ScenarioError со списком ошибок. */
export function normalizeScenario(data: unknown): PlanRequest {
  if (!isObject(data)) throw new ScenarioError('Сценарий должен быть JSON-объектом запроса на планирование')
  if (typeof data.type === 'string' && GEOJSON_TYPES.has(data.type))
    throw new ScenarioError('Это файл GeoJSON, а не сценарий — используйте «Импорт KML/GeoJSON»')
  const errors: string[] = []
  const area = (v: unknown, what: string): Area | null => {
    if (v == null) return null
    if (!isObject(v) || (v.type !== 'Polygon' && v.type !== 'MultiPolygon')) {
      errors.push(`${what}: нужен GeoJSON Polygon или MultiPolygon`)
      return null
    }
    const polys: Polygon[] = []
    for (const rings of v.type === 'Polygon' ? [v.coordinates] : Array.isArray(v.coordinates) ? v.coordinates : [null]) {
      const p = normalizePolygonDropHoles(rings)
      if (typeof p === 'string') {
        errors.push(`${what}: ${p}`)
        return null
      }
      polys.push(p)
    }
    return v.type === 'Polygon' ? polys[0] : { type: 'MultiPolygon', coordinates: polys.map((p) => p.coordinates) }
  }
  const list = (v: unknown, what: string): unknown[] => {
    if (v == null) return []
    if (Array.isArray(v)) return v
    errors.push(`${what}: ожидается массив`)
    return []
  }
  const features = (v: unknown, what: string): unknown[] =>
    list(v, what).filter((f, i) => {
      const ok = isObject(f) && (isObject(f.geometry) || typeof f.type === 'string')
      if (!ok) errors.push(`${what}[${i}]: ожидается GeoJSON Feature`)
      return ok
    })
  const idOf = (v: unknown) => (typeof v === 'string' || typeof v === 'number' ? String(v).trim() : '')
  const sites = (v: unknown, what: string, isBase: boolean): Site[] =>
    list(v, what).flatMap((s, i) => {
      if (!isObject(s)) {
        errors.push(`${what}[${i}]: ожидается объект`)
        return []
      }
      const id = idOf(s.id)
      const p = toLonLat([s.lon, s.lat])
      if (!id) errors.push(`${what}[${i}]: нет id`)
      if (typeof p === 'string') errors.push(`${what}[${i}]: ${p}`)
      if (!id || typeof p === 'string') return []
      const name = typeof s.name === 'string' ? s.name : ''
      return [isBase || name ? { id, name, lon: p[0], lat: p[1] } : { id, lon: p[0], lat: p[1] }]
    })
  const num = (v: unknown, what: string, def: number): number => {
    if (v == null) return def
    if (typeof v === 'number' && Number.isFinite(v)) return v
    errors.push(`${what}: ожидается число`)
    return def
  }
  const obj = (v: unknown, what: string): Record<string, unknown> => {
    if (v == null) return {}
    if (isObject(v)) return v
    errors.push(`${what}: ожидается объект`)
    return {}
  }

  const {
    survey_area,
    allowed_area,
    no_fly_zones,
    bases,
    reserve_sites,
    drones,
    survey_type,
    requirements,
    wind,
    time_weight,
    reserve,
    nfz_buffer_m,
    restrictions,
    obstacles,
    obstacle_clearance_m,
    obstacle_buffer_m,
    mission_start,
    mission_window_h,
    ...extra
  } = data
  const nfz: Polygon[] = []
  list(no_fly_zones, 'no_fly_zones').forEach((z, i) => {
    const a = area(z, `запретная зона ${i + 1}`)
    if (a?.type === 'Polygon') nfz.push(a)
    else if (a) a.coordinates.forEach((coordinates) => nfz.push({ type: 'Polygon', coordinates }))
  })
  const drs: DroneInstance[] = list(drones, 'drones').flatMap((d, i) => {
    if (!isObject(d) || typeof d.model !== 'string' || !d.model) {
      errors.push(`drones[${i}]: нужны поля id и model`)
      return []
    }
    return [
      {
        id: idOf(d.id),
        model: d.model,
        ...(typeof d.payload === 'string' ? { payload: d.payload } : {}),
        ...(typeof d.base_id === 'string' ? { base_id: d.base_id } : {}),
        ...(typeof d.flights_done === 'number' ? { flights_done: d.flights_done } : {}),
        ...(typeof d.hours_done === 'number' ? { hours_done: d.hours_done } : {}),
      },
    ]
  })
  if (survey_type != null && !SURVEY_TYPE_IDS.includes(survey_type as SurveyType))
    errors.push(`survey_type: неизвестный тип съёмки «${String(survey_type)}»`)
  const rq = obj(requirements, 'requirements')
  for (const k of REQUIREMENT_NUMBERS) {
    const v = rq[k]
    if (v != null && !(typeof v === 'number' && Number.isFinite(v))) errors.push(`requirements.${k}: ожидается число`)
  }
  const w = obj(wind, 'wind')

  const out: PlanRequest = {
    ...EMPTY_REQUEST,
    ...extra,
    survey_area: area(survey_area, 'область съёмки'),
    allowed_area: area(allowed_area, 'разрешённая зона'),
    no_fly_zones: nfz,
    bases: sites(bases, 'bases', true),
    reserve_sites: sites(reserve_sites, 'reserve_sites', false),
    drones: drs,
    survey_type: survey_type == null ? EMPTY_REQUEST.survey_type : (survey_type as SurveyType),
    requirements: { ...EMPTY_REQUEST.requirements, ...(rq as Partial<PlanRequest['requirements']>) },
    wind: {
      ...w,
      speed_ms: num(w.speed_ms, 'wind.speed_ms', EMPTY_REQUEST.wind.speed_ms),
      from_deg: num(w.from_deg, 'wind.from_deg', EMPTY_REQUEST.wind.from_deg),
    },
    time_weight: num(time_weight, 'time_weight', EMPTY_REQUEST.time_weight),
    reserve: num(reserve, 'reserve', EMPTY_REQUEST.reserve),
    nfz_buffer_m: num(nfz_buffer_m, 'nfz_buffer_m', EMPTY_REQUEST.nfz_buffer_m),
    // зоны ограничений и препятствия проверяет бэкенд: здесь только тип «массив Feature»
    restrictions: features(restrictions, 'restrictions') as RestrictionFeature[],
    obstacles: features(obstacles, 'obstacles') as ObstacleFeature[],
    obstacle_clearance_m: num(obstacle_clearance_m, 'obstacle_clearance_m', EMPTY_REQUEST.obstacle_clearance_m),
    obstacle_buffer_m: num(obstacle_buffer_m, 'obstacle_buffer_m', EMPTY_REQUEST.obstacle_buffer_m),
    mission_start: typeof mission_start === 'string' && mission_start.trim() ? mission_start : null,
    mission_window_h: num(mission_window_h, 'mission_window_h', EMPTY_REQUEST.mission_window_h),
  }
  if (errors.length) throw new ScenarioError('Некорректный файл сценария:', errors)
  return out
}

/** Текст файла сценария → PlanRequest (см. normalizeScenario). */
export function parseScenarioText(text: string): PlanRequest {
  let data: unknown
  try {
    data = JSON.parse(stripBom(text))
  } catch (e) {
    throw new ScenarioError(`Файл не является корректным JSON: ${(e as Error).message}`)
  }
  return normalizeScenario(data)
}

// ---------- Советник: срок работ и окно работ ----------

const MAX_DEADLINE_S = 168 * 3600 // неделя: больше бэкенд всё равно не планирует

/** Срок работ из строки: «2:30» — часы и минуты, «150» — минуты, «2ч30», «1,5 ч» — тоже понимаем.
 *  null — поле пустое (срок не задан), строка — текст ошибки для пользователя. */
export function parseDuration(text: string): number | null | string {
  const s = text.trim().toLowerCase().replace(',', '.').replace(/\s+/g, ' ')
  if (!s) return null
  const hm = /^(\d{1,3}):(\d{1,2})$/.exec(s)
  const hMin = /^(\d{1,3}) ?(?:ч|час|часа|часов|h) ?(?:(\d{1,2}) ?(?:мин|минут|минуты|м|min|m)?)?$/.exec(s)
  const plain = /^(\d+(?:\.\d+)?) ?(ч|час|часа|часов|h|мин|минут|минуты|минута|м|min|m)?$/.exec(s)
  let seconds: number
  if (hm) {
    if (+hm[2] > 59) return 'минут в сроке должно быть меньше 60'
    seconds = (+hm[1] * 60 + +hm[2]) * 60
  } else if (hMin) {
    if (hMin[2] !== undefined && +hMin[2] > 59) return 'минут в сроке должно быть меньше 60'
    seconds = (+hMin[1] * 60 + +(hMin[2] ?? 0)) * 60
  } else if (plain) {
    const hours = /^(?:ч|час|часа|часов|h)$/.test(plain[2] ?? '')
    seconds = +plain[1] * (hours ? 3600 : 60)
  } else return 'срок задаётся как «чч:мм» (2:30) или числом минут (150)'
  if (!(seconds > 0)) return 'срок должен быть больше нуля'
  if (seconds > MAX_DEADLINE_S) return 'срок больше недели — проверьте ввод'
  return Math.round(seconds)
}

/** Длительность в секундах → «2 ч 30 мин» или «45 мин» (как в сообщениях советника). */
export function formatDuration(seconds: number): string {
  const total = Math.max(0, Math.round(seconds / 60))
  const h = Math.floor(total / 60)
  const m = total % 60
  return h ? `${h} ч ${String(m).padStart(2, '0')} мин` : `${m} мин`
}

/** Значение для <input type="datetime-local">: «2026-09-29T06:00:00+03:00» → «2026-09-29T06:00». */
export function datetimeLocalValue(iso: string | null | undefined): string {
  const m = /^(\d{4}-\d{2}-\d{2})[T ](\d{2}:\d{2})/.exec((iso ?? '').trim())
  return m ? `${m[1]}T${m[2]}` : ''
}

function pluralFlights(n: number): string {
  const a = Math.abs(n) % 100
  if (a % 10 === 1 && a !== 11) return 'полёт'
  if ([2, 3, 4].includes(a % 10) && (a < 12 || a > 14)) return 'полёта'
  return 'полётов'
}

/** Остаток ресурса до ТО для карточки борта; null — норматив для этой модели не задан. */
export function formatMaintenance(m: Maintenance | null | undefined): string | null {
  if (!m) return null
  if (m.remaining_flights != null) return `до ТО: ${m.remaining_flights} ${pluralFlights(m.remaining_flights)}`
  if (m.remaining_hours != null) return `до ТО: ${m.remaining_hours.toFixed(1).replace('.', ',')} ч`
  return null
}

/** Сводка по воздушному пространству для результата; null — данных о нём не было. */
export function airspaceSummary(a: AirspaceInfo | null | undefined): string | null {
  if (!a || (!a.restrictions_total && !a.obstacles_total)) return null
  const parts: string[] = []
  if (a.restrictions_total) parts.push(`зоны ограничений: учтено ${a.restrictions_applied} из ${a.restrictions_total}`)
  if (a.obstacles_total) parts.push(`высотные препятствия: ${a.obstacles_blocking} из ${a.obstacles_total} мешают`)
  const band = a.alt_band_agl_m
  const tail = band && band.length === 2 ? `, полоса высот работ ${Math.round(band[0])}–${Math.round(band[1])} м` : ''
  return parts.join('; ') + tail
}

// ---------- Воздушное пространство: отбор объектов вблизи области ----------

/** Охват любой геометрии GeoJSON (Feature, геометрия, GeometryCollection); null — координат нет. */
export function geometryBBox(input: unknown): BBox | null {
  const b: BBox = [Infinity, Infinity, -Infinity, -Infinity]
  const walk = (c: unknown): void => {
    if (!Array.isArray(c)) return
    if (typeof c[0] === 'number' && typeof c[1] === 'number') {
      b[0] = Math.min(b[0], c[0])
      b[1] = Math.min(b[1], c[1])
      b[2] = Math.max(b[2], c[0])
      b[3] = Math.max(b[3], c[1])
      return
    }
    c.forEach(walk)
  }
  const visit = (g: unknown): void => {
    const o = isObject(g) && g.type === 'Feature' ? g.geometry : g
    if (!isObject(o)) return
    if (Array.isArray(o.geometries)) o.geometries.forEach(visit)
    else walk(o.coordinates)
  }
  visit(input)
  return Number.isFinite(b[0]) ? b : null
}

/** Охват, расширенный на km километров (по широте 111,32 км/°, по долготе — с поправкой). */
export function expandBBox(b: BBox, km: number): BBox {
  const dLat = km / 111.32
  const latMid = (b[1] + b[3]) / 2
  const dLon = km / Math.max(111.32 * Math.cos((latMid * Math.PI) / 180), 1e-6)
  return [
    Math.max(b[0] - dLon, -180),
    Math.max(b[1] - dLat, -90),
    Math.min(b[2] + dLon, 180),
    Math.min(b[3] + dLat, 90),
  ]
}

/** Объекты выгрузки вблизи области съёмки: охват области, расширенный на marginKm, пересекается с
 *  охватом объекта. Точная геометрия не проверяется — грубого отбора хватает, чтобы из 5000
 *  препятствий региона в запрос ушли десятки; лишнее отсеет планировщик. */
export function filterFeaturesNear<T>(features: T[], area: Area | null | undefined, marginKm: number): T[] {
  const box = bboxOf([area])
  if (!box) return []
  const [w, s, e, n] = expandBBox(box, Math.max(marginKm, 0))
  return features.filter((f) => {
    const b = geometryBBox(f)
    return !!b && b[0] <= e && b[2] >= w && b[1] <= n && b[3] >= s
  })
}

/** Разобранная выгрузка данных заказчика: объекты по видам. */
export interface AirspaceImport {
  restrictions: RestrictionFeature[]
  obstacles: ObstacleFeature[]
  /** полигоны задания на съёмку */
  areas: AreaFeature[]
  /** сколько объектов не удалось отнести ни к одному виду */
  skipped: number
}

const AREA_GEOMETRIES = new Set(['Polygon', 'MultiPolygon'])

/** FeatureCollection или список Feature → объекты по видам (properties.kind, а если его нет — по
 *  набору свойств: alt — зона, top_m — препятствие, полигон без свойств — задание на съёмку). */
export function splitAirspaceFeatures(data: unknown): AirspaceImport {
  const out: AirspaceImport = { restrictions: [], obstacles: [], areas: [], skipped: 0 }
  const list = Array.isArray(data) ? data : isObject(data) && Array.isArray(data.features) ? data.features : []
  for (const f of list) {
    if (!isObject(f)) {
      out.skipped++
      continue
    }
    const props = isObject(f.properties) ? f.properties : {}
    const geom = isObject(f.geometry) ? f.geometry : null
    const kind = typeof props.kind === 'string' ? props.kind : ''
    const type = typeof geom?.type === 'string' ? geom.type : ''
    if (kind === 'restriction' || (!kind && isObject(props.alt) && AREA_GEOMETRIES.has(type)))
      out.restrictions.push(f as unknown as RestrictionFeature)
    else if (kind === 'obstacle' || (!kind && props.top_m != null)) out.obstacles.push(f as unknown as ObstacleFeature)
    else if (AREA_GEOMETRIES.has(type)) out.areas.push(f as unknown as AreaFeature)
    else out.skipped++
  }
  return out
}

/** Нечего предлагать пользователю: ни зон, ни препятствий, ни полигонов задания. */
export const airspaceEmpty = (a: AirspaceImport): boolean =>
  !a.restrictions.length && !a.obstacles.length && !a.areas.length

/** Отбор вблизи области съёмки во всех видах сразу. */
export function filterAirspaceNear(a: AirspaceImport, area: Area | null | undefined, marginKm: number): AirspaceImport {
  return {
    restrictions: filterFeaturesNear(a.restrictions, area, marginKm),
    obstacles: filterFeaturesNear(a.obstacles, area, marginKm),
    areas: filterFeaturesNear(a.areas, area, marginKm),
    skipped: a.skipped,
  }
}

/** Лимит бэкенда на число вершин одного полигона (schemas.MAX_VERTICES). */
export const MAX_VERTICES = 20_000

/** Число вершин геометрии — чтобы предупредить о лимите бэкенда. */
export function countVertices(input: unknown): number {
  let n = 0
  const walk = (c: unknown): void => {
    if (!Array.isArray(c)) return
    if (typeof c[0] === 'number' && typeof c[1] === 'number') n++
    else c.forEach(walk)
  }
  const visit = (g: unknown): void => {
    const o = isObject(g) && g.type === 'Feature' ? g.geometry : g
    if (!isObject(o)) return
    if (Array.isArray(o.geometries)) o.geometries.forEach(visit)
    else walk(o.coordinates)
  }
  visit(input)
  return n
}

/** Область съёмки из полигонов задания: выбранный по номеру или объединение всех в MultiPolygon
 *  (объединение — сложение частей; пересечения полигонов задания планировщик снимет сам). */
export function areaFromFeatures(areas: AreaFeature[], pick: 'union' | number): Area | null {
  const chosen = pick === 'union' ? areas : areas[pick] ? [areas[pick]] : []
  const parts: Polygon['coordinates'][] = []
  for (const f of chosen) {
    const g = f?.geometry
    if (g?.type === 'Polygon') parts.push(g.coordinates)
    else if (g?.type === 'MultiPolygon') parts.push(...g.coordinates)
  }
  if (!parts.length) return null
  return parts.length === 1 ? { type: 'Polygon', coordinates: parts[0] } : { type: 'MultiPolygon', coordinates: parts }
}

/** Что взять из разобранной выгрузки: зоны, препятствия и область съёмки. */
export interface AirspaceChoice {
  restrictions: boolean
  obstacles: boolean
  /** 'none' — область не менять, 'union' — все полигоны задания, число — номер полигона */
  area: 'none' | 'union' | number
}

const featureId = (f: unknown): string => {
  if (!isObject(f)) return ''
  const props = isObject(f.properties) ? f.properties : {}
  const id = props.id ?? props.name
  return typeof id === 'string' || typeof id === 'number' ? String(id) : ''
}

/** Добавить к списку, пропуская объекты с уже занятым id (повторная загрузка той же выгрузки). */
function addFeatures<T>(have: T[], add: T[]): T[] {
  const ids = new Set(have.map(featureId).filter(Boolean))
  const out = [...have]
  for (const f of add) {
    const id = featureId(f)
    if (id && ids.has(id)) continue
    if (id) ids.add(id)
    out.push(f)
  }
  return out
}

/** Применить данные о воздушном пространстве к запросу: зоны и препятствия добавляются к
 *  имеющимся (повторы по id пропускаются), область съёмки — заменяется. */
export function applyAirspace(req: PlanRequest, imp: AirspaceImport, choice: AirspaceChoice): PlanRequest {
  let r = req
  if (choice.restrictions && imp.restrictions.length)
    r = { ...r, restrictions: addFeatures(r.restrictions, imp.restrictions) }
  if (choice.obstacles && imp.obstacles.length) r = { ...r, obstacles: addFeatures(r.obstacles, imp.obstacles) }
  if (choice.area !== 'none') {
    const area = areaFromFeatures(imp.areas, choice.area)
    if (area) r = { ...r, survey_area: area }
  }
  return r
}

/** Столько полигонов в одном файле — это задание на съёмку заказчика: область выбирается по
 *  одному полигону, «все как мультиполигон» в лимит вершин не влезет. */
export const MANY_POLYGONS = 50

/** Отдать KML на разбор бэкенду? Да, если клиентский парсер не справился, если в файле признаки
 *  данных заказчика или если это задание из многих полигонов (нужен выбор области). */
export function needsServerImport(text: string, parsed: ImportedGeometry | null): boolean {
  if (!parsed) return true
  if (looksLikeAirspaceKml(text)) return true
  return !parsed.points.length && parsed.polygons.length > MANY_POLYGONS
}

/** Признаки данных заказчика в KML: высоты зон ограничений или 3D-примитивы препятствий. Такой
 *  файл разбирает бэкенд (/api/import/kml) — клиентский парсер взял бы только контуры и потерял
 *  высоты, время действия и тип зоны. */
export function looksLikeAirspaceKml(text: string): boolean {
  const head = text.slice(0, 200_000)
  return /<extrude>|relativeToGround|Altitudes|запретная зона|запретная_зона|ограничение/i.test(head)
}

// ---------- Названия для попапов и легенды ----------

export const ZONE_TYPE_LABELS: Record<ZoneType, string> = {
  prohibited: 'запретная зона',
  permanent: 'постоянное ограничение',
  temporary: 'временное ограничение',
  special: 'особый режим',
  unknown: 'ограничение',
}

export const zoneTypeLabel = (t: string | undefined): string =>
  ZONE_TYPE_LABELS[(t ?? 'unknown') as ZoneType] ?? ZONE_TYPE_LABELS.unknown

const OBSTACLE_TYPE_LABELS: Record<string, string> = {
  COMMUNICATION_TOWER: 'вышка связи',
  ANTENNA: 'антенна',
  MAST: 'мачта',
  TOWER: 'башня',
  CONTROL_TOWER: 'диспетчерская вышка',
  WATER_TOWER: 'водонапорная башня',
  CHIMNEY: 'труба',
  STACK: 'труба',
  BUILDING: 'здание',
  URBAN: 'застройка',
  CONSTRUCTION: 'стройка',
  CRANE: 'кран',
  VEGETATION: 'растительность',
  TREE: 'дерево',
  POWER_TRANSMISSION_PYLON: 'опора ЛЭП',
  TRANSMISSION_LINE: 'ЛЭП',
  AERIAL_CABLE: 'воздушная линия',
  LIGHT_SUPPORT_STRUCTURE: 'опора освещения',
  LIGHTNING_ROD: 'молниеотвод',
  POLE: 'столб',
  FENCE: 'ограда',
  SIGN: 'вывеска',
  SPIRE: 'шпиль',
  CHURCH: 'храм',
  DOME: 'купол',
  NATURAL_HIGHPOINT: 'высота рельефа',
  RETRANSMITTER: 'ретранслятор',
  UNKNOWN: 'препятствие',
}

/** Тип препятствия по-русски; незнакомый код показываем как есть (их в выгрузках десятки). */
export const obstacleTypeLabel = (t: string | undefined): string => {
  const code = (t ?? '').trim()
  return OBSTACLE_TYPE_LABELS[code] ?? (code ? code.replace(/_/g, ' ').toLowerCase() : 'препятствие')
}

/** Верх препятствия для попапа: «77,1 м над землёй» или «212 м над уровнем моря». */
export function obstacleTopLabel(top: number | null | undefined, ref: string | undefined): string {
  if (top == null) return 'высота не задана'
  const v = (Math.round(top * 10) / 10).toFixed(1).replace('.', ',').replace(/,0$/, '')
  return `${v} м ${ref === 'AMSL' ? 'над уровнем моря' : 'над землёй'}`
}

/** Полоса высот зоны для попапа: исходная строка заказчика, а если её нет — разобранные поля. */
export function altBandLabel(alt: RestrictionFeature['properties']['alt']): string {
  if (!alt) return 'высоты не заданы'
  if (alt.raw) return alt.raw
  const ref = (r: string) => (r === 'AGL' ? 'над землёй' : 'AMSL')
  const lo = alt.lower_ref === 'GND' || !alt.lower_m ? 'от земли' : `от ${Math.round(alt.lower_m)} м ${ref(alt.lower_ref)}`
  const hi = alt.upper_m == null ? 'и выше' : `до ${Math.round(alt.upper_m)} м ${ref(alt.upper_ref)}`
  return `${lo} ${hi}`
}
