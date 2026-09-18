// Тесты чистой логики интерфейса: node --test (Node ≥ 23.6 исполняет TypeScript без сборки).
import { test } from 'node:test'
import assert from 'node:assert/strict'
import type { PlanRequest, PlanResponse } from '../src/api.ts'
import {
  criterionLabel,
  droneIdErrors,
  formatCoverage,
  formatValidationItem,
  isEpsilonPlan,
  nextBaseId,
  nextDroneId,
  parseErrorDetail,
  requestKey,
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
