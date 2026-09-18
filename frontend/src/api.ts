import type { Geometry, Polygon, MultiPolygon } from 'geojson'

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
}

export interface Requirements {
  gsd_cm?: number | null
  front_overlap: number
  side_overlap: number
  altitude_m?: number | null
  lidar_density_pts_m2: number
  line_spacing_m?: number | null
  altitude_ceiling_m?: number | null
}

export interface PlanRequest {
  survey_area: Polygon | MultiPolygon | null
  allowed_area?: Polygon | null
  no_fly_zones: Polygon[]
  bases: Site[]
  reserve_sites: Site[]
  drones: DroneInstance[]
  survey_type: SurveyType
  requirements: Requirements
  wind: { speed_ms: number; from_deg: number }
  time_weight: number
  reserve: number
  nfz_buffer_m: number
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

async function json<T>(r: Response): Promise<T> {
  if (!r.ok) {
    let msg = `${r.status}`
    try {
      const body = await r.json()
      msg = typeof body.detail === 'string' ? body.detail : JSON.stringify(body.detail)
    } catch {
      /* тело не JSON */
    }
    throw new Error(msg)
  }
  return r.json() as Promise<T>
}

export const api = {
  fleet: () => fetch('/api/fleet').then((r) => json<Fleet>(r)),
  scenarios: () => fetch('/api/scenarios').then((r) => json<string[]>(r)),
  scenario: (name: string) => fetch(`/api/scenarios/${name}`).then((r) => json<PlanRequest>(r)),
  plan: (req: PlanRequest) =>
    fetch('/api/plan', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(req),
    }).then((r) => json<PlanResponse>(r)),
  pareto: (req: PlanRequest) =>
    fetch('/api/pareto', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(req),
    }).then((r) => json<PlanResponse[]>(r)),
  exportUrl: (planId: string, droneId: string, fmt: 'geojson' | 'kml') =>
    `/api/plan/${planId}/export/${encodeURIComponent(droneId)}.${fmt}`,
  zipUrl: (planId: string) => `/api/plan/${planId}/export.zip`,
}
