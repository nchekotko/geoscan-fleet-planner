import type { Feature, FeatureCollection, Geometry, Polygon, MultiPolygon } from 'geojson'
import { parseErrorDetail, type ExcludedInfo } from './logic'

export type SurveyType = 'rgb' | 'multispectral' | 'thermal' | 'lidar' | 'geophysics'

export interface Site {
  id: string
  name?: string
  lon: number
  lat: number
}

export interface DroneInstance {
  id: string
  model: string
  payload?: string | null
  base_id?: string | null
  /** наработка с прошлого ТО: нормативы — 80 полётов (201, 401) или 160 часов (801, Gemini) */
  flights_done?: number
  hours_done?: number
}

// ---------- Воздушное пространство: зоны ограничений и высотные препятствия ----------

/** Отсчёт высоты: от земли, над рельефом, над уровнем моря. */
export type AltRef = 'GND' | 'AGL' | 'AMSL'

/** Полоса высот зоны; upper_m = null — «и выше». */
export interface AltBand {
  lower_m: number
  lower_ref: AltRef
  upper_m: number | null
  upper_ref: AltRef
  /** исходная строка заказчика («От земли до 500 м (1700 фут) AMSL») */
  raw?: string
}

/** Время действия зоны: срок и/или ежедневное окно ["08:00", "20:00"]. */
export interface ActiveWindow {
  from?: string
  to?: string
  daily?: string[]
  raw?: string
}

export type ZoneType = 'prohibited' | 'permanent' | 'temporary' | 'special' | 'unknown'

export interface RestrictionProps {
  kind?: 'restriction'
  id?: string
  name?: string
  zone_type?: ZoneType
  alt?: AltBand
  active?: ActiveWindow
  notes?: string
  source?: string
  /** в ответе: зона вошла в расчёт */
  applies?: boolean
  /** в ответе: почему зона не учтена (высоты выше работ, не действует в окно) */
  skip_reason?: string
}

export type RestrictionFeature = Feature<Polygon | MultiPolygon, RestrictionProps>

export interface ObstacleProps {
  kind?: 'obstacle'
  id?: string
  obstacle_type?: string
  top_m?: number
  top_ref?: AltRef
  source?: string
  /** в ответе: верх препятствия над землёй области */
  top_agl_m?: number
  blocks?: boolean
}

export type ObstacleFeature = Feature<Geometry, ObstacleProps>

/** Полигон задания на съёмку из данных заказчика. */
export type AreaFeature = Feature<Polygon | MultiPolygon, { kind?: string; id?: string; name?: string }>

export interface Requirements {
  gsd_cm?: number | null
  front_overlap: number
  side_overlap: number
  altitude_m?: number | null
  lidar_density_pts_m2: number
  /** перекрытие полос LiDAR (по руководству 401 — 10–20 %) */
  lidar_side_overlap?: number
  line_spacing_m?: number | null
  altitude_ceiling_m?: number | null
}

export interface PlanRequest {
  survey_area: Polygon | MultiPolygon | null
  allowed_area?: Polygon | MultiPolygon | null
  no_fly_zones: Polygon[]
  bases: Site[]
  reserve_sites: Site[]
  drones: DroneInstance[]
  survey_type: SurveyType
  requirements: Requirements
  /** ref_height_m — высота, на которой задан ветер (метеостанция); null — ветер на рабочей высоте */
  wind: { speed_ms: number; from_deg: number; ref_height_m?: number | null }
  time_weight: number
  reserve: number
  nfz_buffer_m: number
  /** зоны ограничений заказчика: с полосой высот и временем действия */
  restrictions: RestrictionFeature[]
  /** высотные препятствия: мачты, трубы, ЛЭП, лес */
  obstacles: ObstacleFeature[]
  /** вертикальный зазор над препятствием, м */
  obstacle_clearance_m: number
  /** горизонтальный обход препятствия, м */
  obstacle_buffer_m: number
  /** начало работ — по нему отбираются временные зоны; null — время не задано */
  mission_start?: string | null
  /** окно работ, ч */
  mission_window_h: number
}

export interface Leg {
  kind: 'takeoff' | 'transit' | 'survey' | 'tie' | 'turn' | 'return' | 'landing'
  coordinates: number[][]
  alt_amsl?: number[] | null
  duration_s: number
  distance_m: number
  speed_ms: number
}

export interface Sortie {
  index: number
  base_id: string
  start_s: number
  duration_s: number
  survey_length_m: number
  max_divert_s: number
  divert_site: string
  legs: Leg[]
}

export interface SurveyParams {
  altitude_agl_m: number
  line_spacing_m: number
  speed_ms: number
  swath_m: number
  gsd_cm?: number | null
  lidar_density_pts_m2?: number | null
  notes: string[]
}

/** Наработка борта до техобслуживания: норматив — в полётах (201, 401) или в часах (801, Gemini). */
export interface Maintenance {
  interval_flights: number | null
  interval_hours: number | null
  flights_before: number
  hours_before: number
  flights_after: number
  hours_after: number
  remaining_flights: number | null
  remaining_hours: number | null
  /** ТО потребуется до конца задания */
  due: boolean
}

export interface DronePlan {
  drone_id: string
  model: string
  model_name: string
  payload: string
  params: SurveyParams
  sweep_angle_deg: number
  region: Geometry
  area_km2: number
  sorties: Sortie[]
  flight_time_s: number
  finish_s: number
  /** эшелон перелётов, м AGL */
  transit_alt_agl_m?: number
  maintenance?: Maintenance | null
}

/** Что из данных о воздушном пространстве попало в расчёт. */
export interface AirspaceInfo {
  restrictions_total: number
  restrictions_applied: number
  obstacles_total: number
  obstacles_blocking: number
  /** полоса высот работ над землёй [низ, верх], м */
  alt_band_agl_m: number[]
  ground_min_m?: number | null
  ground_max_m?: number | null
}

export interface PlanResponse {
  plan_id: string
  summary: {
    makespan_s: number
    total_flight_s: number
    sorties: number
    area_km2: number
    covered_km2: number
    coverage_pct: number
    drones_used: number
  }
  drones: DronePlan[]
  excluded: { drone_id: string; reason: string }[]
  warnings: string[]
  working_area: Geometry
  time_weight: number
  no_fly_zones: Geometry[]
  allowed_area: Geometry | null
  terrain?: { source: string; ground_min_m: number; ground_max_m: number } | null
  /** ограничение на время работ (метод ε-ограничений); есть не во всех версиях API */
  makespan_cap_s?: number | null
  /** зоны ограничений, попавшие в коридор работ: с полями applies и skip_reason */
  restrictions?: RestrictionFeature[]
  /** только мешающие препятствия: с полем top_agl_m */
  obstacles?: ObstacleFeature[]
  airspace?: AirspaceInfo | null
}

/** Точка кривой «время работ от числа бортов». */
export interface AdviceOption {
  drones: number
  drone_ids: string[]
  makespan_s?: number | null
  total_flight_s?: number | null
  sorties?: number | null
  coverage_pct?: number | null
  /** лучшее время работ, достижимое этим числом бортов или меньшим (кривая не монотонна) */
  best_makespan_s?: number | null
  feasible: boolean
  reason: string
}

export interface AdviceResponse {
  options: AdviceOption[]
  deadline_s?: number | null
  feasible?: boolean | null
  drones_needed?: number | null
  drones_needed_ids: string[]
  best_makespan_s?: number | null
  best_drones?: number | null
  best_drone_ids: string[]
  min_total_flight_s?: number | null
  message: string
}

/** Ограничения пользователя для советника: срок работ и/или сколько бортов разрешено занять. */
export interface AdviceLimits {
  deadline_s?: number | null
  max_drones?: number | null
}

/** Выгрузка данных заказчика на диске сервиса. */
export interface GeodataSet {
  name: string
  title: string
  features: number
  size_kb: number
}

/** Что бэкенд распознал в KML заказчика. */
export interface KmlImport {
  kind: 'zones' | 'obstacles' | 'task'
  features: Feature[]
}

export interface DroneModel {
  id: string
  name: string
  type: 'fixed_wing' | 'multirotor'
  endurance_min: number
  cruise_speed_ms: number
  max_wind_ms: number
  payloads: string[]
}

export interface Payload {
  id: string
  name: string
  survey_types: SurveyType[]
}

export interface Fleet {
  drones: Record<string, DroneModel>
  payloads: Record<string, Payload>
}

/** Ошибка API с разобранными деталями: список ошибок валидации и исключённые борта. */
export class ApiError extends Error {
  items: string[]
  excluded: ExcludedInfo[]
  /** код ответа: 503 — сервис занят другим тяжёлым расчётом */
  status: number
  constructor(message: string, items: string[] = [], excluded: ExcludedInfo[] = [], status = 0) {
    super(message)
    this.items = items
    this.excluded = excluded
    this.status = status
  }
}

async function json<T>(r: Response): Promise<T> {
  if (!r.ok) {
    let body: { detail?: unknown } | null = null
    try {
      body = await r.json()
    } catch {
      /* тело не JSON */
    }
    const e = parseErrorDetail(body?.detail ?? `${r.status} ${r.statusText}`.trim(), r.status)
    throw new ApiError(e.message, e.items, e.excluded, r.status)
  }
  return r.json() as Promise<T>
}

const post = (url: string, body: unknown) =>
  fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })

export const api = {
  fleet: () => fetch('/api/fleet').then((r) => json<Fleet>(r)),
  scenarios: () => fetch('/api/scenarios').then((r) => json<string[]>(r)),
  scenario: (name: string) => fetch(`/api/scenarios/${name}`).then((r) => json<PlanRequest>(r)),
  plan: (req: PlanRequest) => post('/api/plan', req).then((r) => json<PlanResponse>(r)),
  pareto: (req: PlanRequest) => post('/api/pareto', req).then((r) => json<PlanResponse[]>(r)),
  /** Советник: кривая «время работ от числа бортов» и ответ по сроку (расчёт 5–60 с). */
  advise: (req: PlanRequest, limits: AdviceLimits) =>
    post('/api/advise', { ...req, ...limits }).then((r) => json<AdviceResponse>(r)),
  /** Выгрузки данных заказчика, лежащие рядом с сервисом. */
  geodata: () => fetch('/api/geodata').then((r) => json<GeodataSet[]>(r)),
  /** Выгрузка целиком (до 2,5 МБ). */
  geodataSet: (name: string) =>
    fetch(`/api/geodata/${encodeURIComponent(name)}`).then((r) => json<FeatureCollection>(r)),
  /** Разбор KML заказчика на сервере: зоны, препятствия или задание на съёмку. */
  importKml: (kml: string, kind: 'auto' | 'zones' | 'obstacles' | 'task' = 'auto') =>
    post('/api/import/kml', { kml, kind }).then((r) => json<KmlImport>(r)),
  exportUrl: (planId: string, droneId: string, fmt: 'geojson' | 'kml') =>
    `/api/plan/${planId}/export/${encodeURIComponent(droneId)}.${fmt}`,
  zipUrl: (planId: string) => `/api/plan/${planId}/export.zip`,
}
