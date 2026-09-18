import { useEffect, useRef } from 'react'
import L from 'leaflet'
import '@geoman-io/leaflet-geoman-free'
import 'leaflet/dist/leaflet.css'
import '@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css'
import type { Polygon } from 'geojson'
import type { PlanRequest, PlanResponse, Site } from './api'

export type DrawMode = 'survey' | 'allowed' | 'nfz' | 'base' | 'reserve' | null

export const DRONE_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4', '#f032e6', '#9a6324']

interface Props {
  req: PlanRequest
  plan: PlanResponse | null
  /** план посчитан по другим входным данным — приглушаем маршруты */
  stale: boolean
  drawMode: DrawMode
  onDrawn: (mode: Exclude<DrawMode, null>, geom: Polygon | { lon: number; lat: number }) => void
  fitKey: number
}

const RESULTS_PANE = 'results'

export default function MapView({ req, plan, stale, drawMode, onDrawn, fitKey }: Props) {
  const el = useRef<HTMLDivElement>(null)
  const map = useRef<L.Map | null>(null)
  const inputs = useRef<L.LayerGroup>(L.layerGroup())
  const results = useRef<L.LayerGroup>(L.layerGroup())
  const modeRef = useRef<DrawMode>(null)
  const onDrawnRef = useRef(onDrawn)
  onDrawnRef.current = onDrawn

  useEffect(() => {
    if (!el.current || map.current) return
    const m = L.map(el.current, { center: [55.61, 37.62], zoom: 13 })
    // OSM: атрибуция обязательна (Tile Usage Policy)
    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    }).addTo(m)
    // маршруты — в отдельной панели, чтобы приглушать их целиком
    m.createPane(RESULTS_PANE).style.zIndex = '400'
    results.current.addTo(m)
    inputs.current.addTo(m)
    m.pm.setLang('ru')
    m.on('pm:create', (e: { layer: L.Layer }) => {
      const mode = modeRef.current
      const layer = e.layer as L.Layer & { toGeoJSON: () => GeoJSON.Feature }
      const gj = layer.toGeoJSON()
      m.removeLayer(layer)
      if (!mode) return
      if (gj.geometry.type === 'Point') {
        const [lon, lat] = gj.geometry.coordinates
        onDrawnRef.current(mode, { lon, lat })
      } else if (gj.geometry.type === 'Polygon') {
        onDrawnRef.current(mode, gj.geometry)
      }
    })
    map.current = m
    return () => {
      m.remove()
      map.current = null
    }
  }, [])

  useEffect(() => {
    const m = map.current
    if (!m) return
    modeRef.current = drawMode
    m.pm.disableDraw()
    if (drawMode === 'base' || drawMode === 'reserve') {
      m.pm.enableDraw('CircleMarker', { continueDrawing: false })
    } else if (drawMode) {
      m.pm.enableDraw('Polygon', { continueDrawing: false })
    }
  }, [drawMode])

  // входные слои
  useEffect(() => {
    const g = inputs.current
    g.clearLayers()
    const poly = (geom: GeoJSON.Geometry, style: L.PathOptions, tip: string) =>
      L.geoJSON(geom, { style, pmIgnore: true } as L.GeoJSONOptions).bindTooltip(tip, { sticky: true }).addTo(g)
    if (req.allowed_area)
      poly(req.allowed_area, { color: '#2a9d8f', weight: 2, dashArray: '8 6', fillOpacity: 0.02 }, 'Разрешённая зона')
    if (req.survey_area) poly(req.survey_area, { color: '#1d3557', weight: 2, fillOpacity: 0.06 }, 'Область съёмки')
    req.no_fly_zones.forEach((z, i) =>
      poly(z, { color: '#d62828', weight: 2, fillColor: '#d62828', fillOpacity: 0.3 }, `Запретная зона ${i + 1}`),
    )
    const site = (s: Site, color: string, label: string) =>
      L.circleMarker([s.lat, s.lon], { radius: 8, color: '#fff', weight: 2, fillColor: color, fillOpacity: 1, pmIgnore: true } as L.CircleMarkerOptions)
        .bindTooltip(label, { permanent: true, direction: 'right', offset: [8, 0], className: 'site-label' })
        .addTo(g)
    req.bases.forEach((b) => site(b, '#000', `ВПП ${b.id}${b.name ? ' · ' + b.name : ''}`))
    req.reserve_sites.forEach((r) => site(r, '#f4a261', `Резерв ${r.id}`))
  }, [req])

  // результаты
  useEffect(() => {
    const g = results.current
    g.clearLayers()
    if (!plan) return
    plan.drones.forEach((d, i) => {
      const color = DRONE_COLORS[i % DRONE_COLORS.length]
      L.geoJSON(d.region, { pane: RESULTS_PANE, style: { color, weight: 1, fillOpacity: 0.08, dashArray: '2 4' } }).addTo(g)
      for (const s of d.sorties) {
        for (const leg of s.legs) {
          if (leg.kind === 'takeoff' || leg.kind === 'landing') continue
          const latlngs = leg.coordinates.map((c) => [c[1], c[0]] as [number, number])
          const style: L.PolylineOptions =
            leg.kind === 'survey'
              ? { color, weight: 3 }
              : leg.kind === 'tie'
                ? { color, weight: 3, dashArray: '10 4', opacity: 0.9 }
                : leg.kind === 'turn'
                ? { color, weight: 1.5, opacity: 0.7 }
                : { color, weight: 2, dashArray: '6 6', opacity: 0.8 }
          L.polyline(latlngs, { ...style, pane: RESULTS_PANE })
            .bindTooltip(
              `${d.drone_id} · вылет ${s.index + 1} · ${leg.kind} · ${(leg.duration_s / 60).toFixed(1)} мин`,
              { sticky: true },
            )
            .addTo(g)
        }
      }
    })
  }, [plan])

  useEffect(() => {
    map.current?.getPane(RESULTS_PANE)?.classList.toggle('stale', stale)
  }, [stale])

  useEffect(() => {
    const m = map.current
    if (!m || !req.survey_area) return
    const b = L.geoJSON(req.survey_area).getBounds()
    req.bases.forEach((s) => b.extend([s.lat, s.lon]))
    if (b.isValid()) m.fitBounds(b.pad(0.15))
    // перецентрируем только при загрузке сценария
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [fitKey])

  return (
    <>
      <div ref={el} className="map" />
      <div className="legend">
        <div><i className="l-survey" /> галс съёмки</div>
        <div><i className="l-tie" /> секущий маршрут</div>
        <div><i className="l-turn" /> разворот</div>
        <div><i className="l-transit" /> перелёт / возврат</div>
        <div><i className="l-nfz" /> запретная зона</div>
      </div>
    </>
  )
}
