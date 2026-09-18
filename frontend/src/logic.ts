// Чистые функции состояния интерфейса (без React) — покрыты тестами в tests/logic.test.ts.
import type { DroneInstance, PlanRequest, PlanResponse } from './api'

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
