// Чистые функции состояния интерфейса (без React) — покрыты тестами в tests/logic.test.ts.
import type { MultiPolygon, Polygon } from 'geojson'
import type { DroneInstance, PlanRequest, PlanResponse, Site, SurveyType } from './api'

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
      const p = normalizePolygon(rings)
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
