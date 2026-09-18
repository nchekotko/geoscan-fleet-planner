// Тесты чистой логики интерфейса: node --test (Node ≥ 23.6 исполняет TypeScript без сборки).
import { test } from 'node:test'
import assert from 'node:assert/strict'
import { existsSync, readdirSync, readFileSync } from 'node:fs'
import { join } from 'node:path'
import type { Polygon } from 'geojson'
import type { PlanRequest, PlanResponse } from '../src/api.ts'
import {
  applyEdit,
  applyImport,
  bboxOf,
  cleanPolygonGeometry,
  criterionLabel,
  droneIdErrors,
  EMPTY_REQUEST,
  formatCoverage,
  formatValidationItem,
  isEpsilonPlan,
  nextBaseId,
  nextDroneId,
  normalizeScenario,
  parseErrorDetail,
  parseGeoJSONText,
  parseGeometryFile,
  parseKMLText,
  parseKmlCoordinates,
  parseScenarioText,
  requestBBox,
  requestKey,
  scenarioJSON,
  ScenarioError,
} from '../src/logic.ts'

const req = (over: Partial<PlanRequest> = {}): PlanRequest => ({
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
  ...over,
})

const plan = (makespan: number, w: number, extra: Partial<PlanResponse> = {}): PlanResponse =>
  ({
    plan_id: `${makespan}-${w}`,
    summary: { makespan_s: makespan, total_flight_s: 0, sorties: 1, area_km2: 1, covered_km2: 1, coverage_pct: 100, drones_used: 1 },
    drones: [],
    excluded: [],
    warnings: [],
    working_area: { type: 'Polygon', coordinates: [] },
    time_weight: w,
    ...extra,
  }) as PlanResponse

test('ключ запроса меняется от ветра, но не от веса критерия', () => {
  const a = req()
  assert.notEqual(requestKey(a), requestKey(req({ wind: { speed_ms: 8, from_deg: 0 } })))
  assert.equal(requestKey(a), requestKey(req({ time_weight: 0.3 })))
})

test('id борта — первый свободный номер после удаления', () => {
  const drones = [
    { id: 'gemini-1', model: 'geoscan_gemini' },
    { id: 'gemini-2', model: 'geoscan_gemini' },
  ]
  // удалили gemini-1, осталось gemini-2 → новый должен быть gemini-1, а не второй gemini-2
  assert.equal(nextDroneId('geoscan_gemini', drones.slice(1)), 'gemini-1')
  assert.equal(nextDroneId('geoscan_gemini', drones), 'gemini-3')
  assert.equal(nextDroneId('geoscan_201', drones), '201-1')
})

test('id ВПП — первая свободная буква', () => {
  assert.equal(nextBaseId(['B']), 'A')
  assert.equal(nextBaseId(['A', 'B']), 'C')
})

test('повторяющиеся и пустые id бортов подсвечиваются', () => {
  const e = droneIdErrors([
    { id: 'a', model: 'm' },
    { id: 'b', model: 'm' },
    { id: 'a ', model: 'm' },
    { id: ' ', model: 'm' },
  ])
  assert.deepEqual(Object.keys(e).map(Number), [0, 2, 3])
  assert.match(e[0], /занят/)
  assert.match(e[3], /пуст/)
})

test('покрытие не округляется вверх до 100 %', () => {
  assert.equal(formatCoverage(99.96), '99.9')
  assert.equal(formatCoverage(100), '100.0')
  assert.equal(formatCoverage(97.25), '97.2')
})

test('подпись критерия', () => {
  assert.equal(criterionLabel({ kind: 'weighted', w: 1 }), 'время работ')
  assert.equal(criterionLabel({ kind: 'weighted', w: 0 }), 'налёт')
  assert.equal(criterionLabel({ kind: 'weighted', w: 0.6 }), 'компромисс w=0.6')
  assert.match(criterionLabel({ kind: 'front', w: null }), /ε-ограничение/)
  assert.match(criterionLabel({ kind: 'front', w: 0.45 }), /Парето \(w=0\.45\)/)
})

test('ε-план фронта: по makespan_cap_s или по w=1 не на самой быстрой точке', () => {
  const fast = plan(1000, 1)
  const eps = plan(1500, 1)
  const mid = plan(1800, 0.5)
  const front = [fast, eps, mid]
  assert.equal(isEpsilonPlan(fast, front), false)
  assert.equal(isEpsilonPlan(eps, front), true)
  assert.equal(isEpsilonPlan(mid, front), false)
  assert.equal(isEpsilonPlan(plan(1500, 1, { makespan_cap_s: null }), front), false)
  assert.equal(isEpsilonPlan(plan(900, 1, { makespan_cap_s: 1200 }), front), true)
})

test('ошибки 422: строка, список pydantic, объект с исключёнными бортами', () => {
  assert.deepEqual(parseErrorDetail('нет баз', 422), { message: 'нет баз', items: [], excluded: [] })
  const list = parseErrorDetail(
    [
      { loc: ['body', 'drones', 0, 'id'], msg: 'String should have at least 1 character', type: 'x' },
      { loc: ['body', 'requirements', 'front_overlap'], msg: 'Input should be less than 1' },
    ],
    422,
  )
  assert.equal(list.message, 'Некорректные входные данные:')
  assert.deepEqual(list.items, [
    'drones[0].id: String should have at least 1 character',
    'requirements.front_overlap: Input should be less than 1',
  ])
  const obj = parseErrorDetail(
    { message: 'ни один борт не может…', excluded: [{ drone_id: '401-1', reason: 'ветер 14 м/с > 12' }] },
    422,
  )
  assert.equal(obj.message, 'ни один борт не может…')
  assert.deepEqual(obj.excluded, [{ drone_id: '401-1', reason: 'ветер 14 м/с > 12' }])
  assert.equal(formatValidationItem({ msg: 'bad' }), 'bad')
})

// ---------- импорт геометрии ----------

const square = (x: number, y: number, d = 0.01): Polygon => ({
  type: 'Polygon',
  coordinates: [[[x, y], [x + d, y], [x + d, y + d], [x, y + d], [x, y]]],
})

const KML = `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
<Document>
  <name>Участок</name>
  <!-- <Placemark><Point><coordinates>0,0</coordinates></Point></Placemark> — закомментировано -->
  <Folder>
    <Placemark id="p1">
      <name>Поле &amp; лес</name>
      <Polygon>
        <outerBoundaryIs><LinearRing><coordinates>
          37.60,55.60,120 37.64,55.60,120
          37.64,55.62,120 37.60,55.62,120 37.60,55.60,120
        </coordinates></LinearRing></outerBoundaryIs>
        <innerBoundaryIs><LinearRing><coordinates>37.61,55.605 37.62,55.605 37.62,55.61 37.61,55.605</coordinates></LinearRing></innerBoundaryIs>
      </Polygon>
    </Placemark>
    <Placemark>
      <name><![CDATA[ВПП <Север>]]></name>
      <Point><coordinates>37.65, 55.63, 0</coordinates></Point>
    </Placemark>
    <Placemark><name>Дорога</name><LineString><coordinates>37.6,55.6 37.7,55.7</coordinates></LineString></Placemark>
    <Placemark>
      <MultiGeometry>
        <kml:Polygon xmlns:kml="http://www.opengis.net/kml/2.2"><kml:outerBoundaryIs><kml:LinearRing>
          <kml:coordinates>37.70,55.60 37.71,55.60 37.71,55.61 37.70,55.60</kml:coordinates>
        </kml:LinearRing></kml:outerBoundaryIs></kml:Polygon>
        <Point><coordinates>37.705,55.605</coordinates></Point>
      </MultiGeometry>
    </Placemark>
    <Placemark><name>Кривой</name><Polygon><outerBoundaryIs><LinearRing><coordinates>37.6,95 37.7,55.6 37.7,55.7</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>
    <Placemark><name>Без координат</name><Point><coordinates>abc,55</coordinates></Point></Placemark>
  </Folder>
</Document>
</kml>`

test('координаты KML: кортежи lon,lat[,alt] через любые пробелы', () => {
  assert.deepEqual(parseKmlCoordinates(' 37.6,55.6,120\n\t37.61,55.6  37.62 , 55.61 '), [
    [37.6, 55.6, 120],
    [37.61, 55.6],
    [37.62, 55.61],
  ])
  assert.deepEqual(parseKmlCoordinates(''), [])
  assert.ok(parseKmlCoordinates('37.6,,55').flat().some(Number.isNaN))
})

test('KML: полигоны с дырами, точки с именами, MultiGeometry, префиксы, пропуски с сообщением', () => {
  const r = parseKMLText(KML)
  assert.equal(r.polygons.length, 2)
  const [field, small] = r.polygons
  // высота отброшена, контур замкнут, дыра сохранена
  assert.deepEqual(field.coordinates[0], [[37.6, 55.6], [37.64, 55.6], [37.64, 55.62], [37.6, 55.62], [37.6, 55.6]])
  assert.equal(field.coordinates.length, 2)
  assert.deepEqual(small.coordinates[0].at(0), small.coordinates[0].at(-1))
  assert.deepEqual(r.points, [
    { lon: 37.65, lat: 55.63, name: 'ВПП <Север>' },
    { lon: 37.705, lat: 55.605 },
  ])
  const w = r.warnings.join('\n')
  assert.match(w, /LineString/)
  assert.match(w, /полигон «Кривой».*вне диапазона/)
  assert.match(w, /точка «Без координат».*конечными/)
})

test('KML: незамкнутый контур замыкается, не-KML и KMZ — понятная ошибка', () => {
  const open = '<kml><Placemark><Polygon><outerBoundaryIs><LinearRing><coordinates>1,1 2,1 2,2</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark></kml>'
  assert.deepEqual(parseKMLText(open).polygons[0].coordinates[0], [[1, 1], [2, 1], [2, 2], [1, 1]])
  assert.throws(() => parseKMLText('<html><body>нет</body></html>'), /не похож на KML/)
  assert.throws(() => parseGeometryFile('area.kmz', 'PK...'), /KMZ/)
})

const GEOJSON = JSON.stringify({
  type: 'FeatureCollection',
  features: [
    { type: 'Feature', properties: { name: 'Участок' }, geometry: { type: 'Polygon', coordinates: [[[37.6, 55.6, 150], [37.64, 55.6], [37.64, 55.62], [37.6, 55.62]]] } },
    {
      type: 'Feature',
      properties: {},
      geometry: {
        type: 'MultiPolygon',
        coordinates: [square(37.7, 55.6).coordinates, square(37.8, 55.6).coordinates],
      },
    },
    { type: 'Feature', properties: { name: 'ВПП 1' }, geometry: { type: 'Point', coordinates: [37.59, 55.59] } },
    { type: 'Feature', properties: { title: 'Р' }, geometry: { type: 'MultiPoint', coordinates: [[37.5, 55.5], [37.51, 55.51]] } },
    { type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [[37.6, 55.6], [37.7, 55.7]] } },
    { type: 'Feature', properties: {}, geometry: null },
    { type: 'Feature', properties: { name: 'Меркатор' }, geometry: { type: 'Polygon', coordinates: [[[4187000, 7480000], [4190000, 7480000], [4190000, 7483000], [4187000, 7480000]]] } },
    { type: 'Feature', properties: {}, geometry: { type: 'Polygon', coordinates: [[[37.6, 55.6], [37.61, 55.6], [37.62, 55.6], [37.6, 55.6]]] } },
  ],
})

test('GeoJSON: FeatureCollection — полигоны, мультиполигоны, точки; остальное пропущено с сообщением', () => {
  const r = parseGeoJSONText(GEOJSON)
  assert.equal(r.polygons.length, 3)
  assert.deepEqual(r.polygons[0].coordinates[0], [[37.6, 55.6], [37.64, 55.6], [37.64, 55.62], [37.6, 55.62], [37.6, 55.6]])
  assert.deepEqual(r.polygons[1], square(37.7, 55.6))
  assert.deepEqual(r.points, [
    { lon: 37.59, lat: 55.59, name: 'ВПП 1' },
    { lon: 37.5, lat: 55.5, name: 'Р' },
    { lon: 37.51, lat: 55.51, name: 'Р' },
  ])
  const w = r.warnings.join('\n')
  assert.match(w, /LineString/)
  assert.match(w, /Feature без геометрии/)
  assert.match(w, /«Меркатор».*WGS84/)
  assert.match(w, /нулевой площади/)
})

test('GeoJSON: голая геометрия и Feature; не GeoJSON — ошибка', () => {
  assert.equal(parseGeoJSONText(JSON.stringify(square(10, 10))).polygons.length, 1)
  assert.deepEqual(parseGeoJSONText('{"type":"Feature","geometry":{"type":"Point","coordinates":[1,2]}}').points, [{ lon: 1, lat: 2 }])
  assert.deepEqual(parseGeoJSONText('﻿{"type":"Point","coordinates":[1,2,3]}').points, [{ lon: 1, lat: 2 }])
  assert.throws(() => parseGeoJSONText('{"type":'), /корректным JSON/)
  assert.throws(() => parseGeoJSONText('{"foo":1}'), /не похож на GeoJSON/)
  assert.throws(() => parseGeoJSONText('{"survey_area":null,"drones":[]}'), /Загрузить сценарий/)
  assert.throws(() => parseGeoJSONText('[1,2]'), /не похож на GeoJSON/)
  // бесконечность в JSON не записать, но null/строки вместо чисел — бывают
  const bad = parseGeoJSONText('{"type":"Point","coordinates":[null, 55]}')
  assert.equal(bad.points.length, 0)
  assert.match(bad.warnings[0], /конечными числами/)
})

test('выбор формата файла: по расширению, иначе по содержимому', () => {
  const kml = '<kml><Placemark><Point><coordinates>1,2</coordinates></Point></Placemark></kml>'
  assert.equal(parseGeometryFile('a.KML', kml).points.length, 1)
  assert.equal(parseGeometryFile('a.txt', kml).points.length, 1)
  assert.equal(parseGeometryFile('a.geojson', '{"type":"Point","coordinates":[1,2]}').points.length, 1)
  assert.throws(() => parseGeometryFile('a.json', kml), /JSON/)
})

test('назначение импорта: область съёмки, разрешённая зона, запретные зоны, ВПП, резерв', () => {
  const imp = { polygons: [square(1, 1), square(2, 2)], points: [{ lon: 1, lat: 1, name: 'Север' }, { lon: 2, lat: 2 }], warnings: [] }
  const base = req({ bases: [{ id: 'A', name: '', lon: 0, lat: 0 }], no_fly_zones: [square(5, 5)] })

  const one = applyImport(base, { ...imp, polygons: [square(1, 1)] }, { polygons: 'survey', points: null })
  assert.deepEqual(one.survey_area, square(1, 1))
  const multi = applyImport(base, imp, { polygons: 'survey', points: null })
  assert.equal(multi.survey_area?.type, 'MultiPolygon')
  assert.equal(multi.survey_area?.coordinates.length, 2)

  assert.deepEqual(applyImport(base, imp, { polygons: 'allowed', points: null }).allowed_area, square(1, 1))
  assert.equal(applyImport(base, imp, { polygons: 'nfz', points: null }).no_fly_zones.length, 3)

  const bases = applyImport(base, imp, { polygons: null, points: 'base' })
  assert.deepEqual(bases.bases.map((b) => [b.id, b.name]), [['A', ''], ['B', 'Север'], ['C', '']])
  assert.equal(bases.survey_area, null)

  const res = applyImport(req({ reserve_sites: [{ id: 'R1', lon: 0, lat: 0 }] }), imp, { polygons: null, points: 'reserve' })
  assert.deepEqual(res.reserve_sites.map((s) => [s.id, s.name]), [['R1', undefined], ['R2', 'Север'], ['R3', undefined]])

  // ничего не выбрано — запрос не меняется
  assert.equal(applyImport(base, imp, { polygons: null, points: null }), base)
})

test('охват для подгонки карты', () => {
  assert.deepEqual(bboxOf([square(1, 2, 1)], [{ lon: 0, lat: 5 }]), [0, 2, 2, 5])
  assert.equal(bboxOf([], []), null)
  assert.equal(requestBBox(req()), null)
  assert.deepEqual(requestBBox(req({ allowed_area: square(0, 0, 10) })), [0, 0, 10, 10])
  // при области съёмки разрешённая зона не учитывается
  assert.deepEqual(requestBBox(req({ allowed_area: square(0, 0, 10), survey_area: square(1, 1, 1) })), [1, 1, 2, 2])
})

// ---------- правка геометрии ----------

test('правка с карты пишется в запрос и делает план устаревшим', () => {
  const a = req({
    survey_area: square(1, 1),
    no_fly_zones: [square(3, 3), square(4, 4)],
    bases: [{ id: 'A', name: 'Юг', lon: 0, lat: 0 }],
    reserve_sites: [{ id: 'R1', lon: 9, lat: 9 }],
  })
  const moved = applyEdit(a, { kind: 'survey' }, square(1.5, 1))
  assert.deepEqual(moved.survey_area, square(1.5, 1))
  assert.notEqual(requestKey(moved), requestKey(a))

  const z = applyEdit(a, { kind: 'nfz', index: 1 }, square(7, 7))
  assert.deepEqual(z.no_fly_zones, [square(3, 3), square(7, 7)])

  const b = applyEdit(a, { kind: 'base', index: 0 }, { lon: 0.5, lat: 0.25 })
  assert.deepEqual(b.bases, [{ id: 'A', name: 'Юг', lon: 0.5, lat: 0.25 }])
  assert.deepEqual(applyEdit(a, { kind: 'reserve', index: 0 }, { lon: 8, lat: 8 }).reserve_sites, [{ id: 'R1', lon: 8, lat: 8 }])
  assert.deepEqual(applyEdit(a, { kind: 'allowed' }, square(0, 0, 5)).allowed_area, square(0, 0, 5))
})

test('правка с карты: повторы вершин убираются, вырожденный полигон — откат', () => {
  const dup: Polygon = { type: 'Polygon', coordinates: [[[0, 0], [1, 0], [1, 0], [1, 1], [0, 0]]] }
  assert.deepEqual(cleanPolygonGeometry(dup), { type: 'Polygon', coordinates: [[[0, 0], [1, 0], [1, 1], [0, 0]]] })
  assert.equal(cleanPolygonGeometry({ type: 'Polygon', coordinates: [[[0, 0], [1, 0], [0, 0]]] }), null)
  // вырожденная дыра отбрасывается, внешний контур остаётся
  const holed: Polygon = { type: 'Polygon', coordinates: [square(0, 0, 1).coordinates[0], [[0.2, 0.2], [0.3, 0.3], [0.2, 0.2]]] }
  assert.equal((cleanPolygonGeometry(holed) as Polygon).coordinates.length, 1)
  // вершина за пределами ±180° (карта прокручена через антимеридиан) — откат
  assert.equal(cleanPolygonGeometry({ type: 'Polygon', coordinates: [[[179, 0], [181, 0], [181, 1], [179, 0]]] }), null)
})

// ---------- файл сценария ----------

test('сценарий: сохранение и загрузка дают тот же запрос', () => {
  const a = req({
    survey_area: square(37.6, 55.6),
    allowed_area: square(37.5, 55.5, 0.3),
    no_fly_zones: [square(37.61, 55.605, 0.005)],
    bases: [{ id: 'A', name: 'ВПП Юг', lon: 37.605, lat: 55.595 }],
    reserve_sites: [{ id: 'R1', lon: 37.625, lat: 55.63 }],
    drones: [{ id: 'gemini-1', model: 'geoscan_gemini', base_id: 'A' }, { id: '401-1', model: 'geoscan_401', payload: 'agm_ms3' }],
    survey_type: 'lidar',
    wind: { speed_ms: 7, from_deg: 250 },
    time_weight: 0.4,
  })
  assert.deepEqual(parseScenarioText(scenarioJSON(a)), a)
})

test('сценарий: умолчания для пропущенных полей, неизвестные поля сохраняются', () => {
  const s = normalizeScenario({
    survey_area: square(1, 1),
    bases: [{ id: 'A', lon: 1, lat: 1 }],
    drones: [{ id: 'd1', model: 'geoscan_201' }],
    requirements: { gsd_cm: 4 },
    use_terrain: false,
  })
  assert.deepEqual(s.requirements, { ...EMPTY_REQUEST.requirements, gsd_cm: 4 })
  assert.deepEqual(s.wind, EMPTY_REQUEST.wind)
  assert.deepEqual(s.bases, [{ id: 'A', name: '', lon: 1, lat: 1 }])
  assert.equal(s.allowed_area, null)
  assert.equal((s as PlanRequest & { use_terrain?: boolean }).use_terrain, false)
})

test('сценарий: ошибки собираются списком, GeoJSON вместо сценария распознаётся', () => {
  const err = (f: () => unknown) => {
    try {
      f()
    } catch (e) {
      assert.ok(e instanceof ScenarioError)
      return e
    }
    assert.fail('ожидалась ошибка')
  }
  const e = err(() =>
    normalizeScenario({
      survey_area: { type: 'LineString', coordinates: [] },
      bases: [{ id: 'A', lon: 200, lat: 55 }],
      drones: [{ id: 'x' }],
      wind: { speed_ms: 'сильный' },
      survey_type: 'radar',
    }),
  )
  assert.equal(e.items.length, 5)
  assert.match(e.items.join('\n'), /область съёмки: нужен GeoJSON Polygon/)
  assert.match(e.items.join('\n'), /bases\[0\].*вне диапазона/)
  assert.match(err(() => normalizeScenario(square(1, 1))).message, /Импорт KML\/GeoJSON/)
  assert.match(err(() => parseScenarioText('не json')).message, /корректным JSON/)
  assert.match(err(() => parseScenarioText('[]')).message, /JSON-объектом/)
})

test('сценарии бэкенда (backend/data/scenarios) загружаются без ошибок', (t) => {
  const dir = join(import.meta.dirname, '..', '..', 'backend', 'data', 'scenarios')
  if (!existsSync(dir)) return t.skip('нет каталога сценариев бэкенда')
  const files = readdirSync(dir).filter((f) => f.endsWith('.json'))
  assert.ok(files.length > 0)
  for (const f of files) {
    const raw = JSON.parse(readFileSync(join(dir, f), 'utf8'))
    const s = normalizeScenario(raw)
    assert.equal(s.drones.length, raw.drones.length, f)
    assert.equal(s.bases.length, raw.bases.length, f)
    assert.equal(s.no_fly_zones.length, (raw.no_fly_zones ?? []).length, f)
    assert.ok(s.survey_area, f)
    assert.deepEqual(parseScenarioText(scenarioJSON(s)), s, f)
  }
})
