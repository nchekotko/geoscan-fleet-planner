import { useEffect, useRef, useState } from 'react'
import L from 'leaflet'
import '@geoman-io/leaflet-geoman-free'
import 'leaflet/dist/leaflet.css'
import '@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css'
import type { MultiPolygon, Polygon } from 'geojson'
import type { PlanRequest, PlanResponse, Site } from './api'
import { cleanPolygonGeometry, type BBox, type EditTarget, type EditValue } from './logic'

export type DrawMode = 'survey' | 'allowed' | 'nfz' | 'base' | 'reserve' | null

export const DRONE_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4', '#f032e6', '#9a6324']

/** Запрос на подгонку карты: при смене key карта показывает bbox. */
export interface FitRequest {
  key: number
  bbox: BBox | null
}

interface Props {
  req: PlanRequest
  plan: PlanResponse | null
  /** план посчитан по другим входным данным — приглушаем маршруты */
  stale: boolean
  drawMode: DrawMode
  onDrawn: (mode: Exclude<DrawMode, null>, geom: Polygon | { lon: number; lat: number }) => void
  /** режим правки: вершины полигонов и площадки можно перетаскивать */
  editing: boolean
  onEdited: (target: EditTarget, value: EditValue) => void
  fit: FitRequest
}

const RESULTS_PANE = 'results'
// ВПП и резервные площадки — над маршрутами: перелёты начинаются в них и иначе перехватывают мышь
const SITES_PANE = 'sites'

// правка полигона: только вершины (сам полигон не таскаем — иначе легко сдвинуть его вместо карты),
// меньше трёх вершин не оставляем; у больших импортированных контуров показываем ближайшие вершины
const POLYGON_EDIT: L.PM.EditModeOptions = {
  draggable: false,
  snappable: true,
  removeLayerBelowMinVertexCount: false,
  limitMarkersToCount: 300,
}
// площадку можно только перетащить: удаление правым кликом и изменение радиуса выключены
const SITE_EDIT: L.PM.EditModeOptions = { draggable: true, snappable: false, preventMarkerRemoval: true }

export default function MapView({ req, plan, stale, drawMode, onDrawn, editing, onEdited, fit }: Props) {
  const el = useRef<HTMLDivElement>(null)
  const map = useRef<L.Map | null>(null)
  const inputs = useRef<L.LayerGroup>(L.layerGroup())
  const results = useRef<L.LayerGroup>(L.layerGroup())
  const modeRef = useRef<DrawMode>(null)
  const onDrawnRef = useRef(onDrawn)
  onDrawnRef.current = onDrawn
  const onEditedRef = useRef(onEdited)
  // перерисовка входных слоёв без изменения запроса — откат недопустимой правки
  const [restore, setRestore] = useState(0)

  useEffect(() => {
    onEditedRef.current = onEdited
  }, [onEdited])

  useEffect(() => {
    if (!el.current || map.current) return
    const m = L.map(el.current, { center: [55.61, 37.62], zoom: 13, zoomSnap: 0.25, zoomDelta: 0.5 })
    // OSM: атрибуция обязательна (Tile Usage Policy)
    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    }).addTo(m)
    // маршруты — в отдельной панели, чтобы приглушать их целиком
    m.createPane(RESULTS_PANE).style.zIndex = '400'
    m.createPane(SITES_PANE).style.zIndex = '450'
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

  // входные слои; в режиме правки — с включённым редактированием geoman.
  // Любая правка идёт через onEdited → новый req → слои строятся заново (правка geoman снимается
  // сама при удалении слоя с карты), поэтому на карте всегда ровно то, что уйдёт в запрос.
  useEffect(() => {
    const g = inputs.current
    g.clearLayers()
    const poly = (geom: Polygon | MultiPolygon, style: L.PathOptions, tip: string, target: EditTarget) => {
      const latlngs = L.GeoJSON.coordsToLatLngs(geom.coordinates, geom.type === 'Polygon' ? 1 : 2)
      const layer = L.polygon(latlngs, { ...style, pmIgnore: !editing }).bindTooltip(tip, { sticky: true }).addTo(g)
      if (!editing) return
      layer.pm.enable(POLYGON_EDIT)
      layer.on('pm:edit', () => {
        const edited = cleanPolygonGeometry(layer.toGeoJSON(false).geometry as Polygon | MultiPolygon)
        if (edited) onEditedRef.current(target, edited)
        else setRestore((k) => k + 1)
      })
    }
    if (req.allowed_area)
      poly(req.allowed_area, { color: '#2a9d8f', weight: 2, dashArray: '8 6', fillOpacity: 0.02 }, 'Разрешённая зона', {
        kind: 'allowed',
      })
    if (req.survey_area)
      poly(req.survey_area, { color: '#1d3557', weight: 2, fillOpacity: 0.06 }, 'Область съёмки', { kind: 'survey' })
    req.no_fly_zones.forEach((z, i) =>
      poly(z, { color: '#d62828', weight: 2, fillColor: '#d62828', fillOpacity: 0.3 }, `Запретная зона ${i + 1}`, {
        kind: 'nfz',
        index: i,
      }),
    )
    const site = (s: Site, color: string, label: string, target: EditTarget) => {
      const mk = L.circleMarker([s.lat, s.lon], {
        pane: SITES_PANE,
        radius: editing ? 9 : 8,
        color: editing ? '#ffd166' : '#fff',
        weight: editing ? 3 : 2,
        fillColor: color,
        fillOpacity: 1,
        pmIgnore: !editing,
      })
        .bindTooltip(label, { permanent: true, direction: 'right', offset: [8, 0], className: 'site-label' })
        .addTo(g)
      if (!editing) return
      mk.pm.enable(SITE_EDIT)
      mk.on('pm:edit', () => {
        const ll = mk.getLatLng().wrap()
        onEditedRef.current(target, { lon: ll.lng, lat: ll.lat })
      })
    }
    req.bases.forEach((b, i) => site(b, '#000', `ВПП ${b.id}${b.name ? ' · ' + b.name : ''}`, { kind: 'base', index: i }))
    req.reserve_sites.forEach((r, i) =>
      site(r, '#f4a261', `Резерв ${r.id}${r.name ? ' · ' + r.name : ''}`, { kind: 'reserve', index: i }),
    )
  }, [req, editing, restore])

  // результаты
  useEffect(() => {
    const g = results.current
    g.clearLayers()
    if (!plan) return
    // маршруты не редактируются и не служат точками привязки при рисовании и правке
    const out = { pane: RESULTS_PANE, pmIgnore: true }
    plan.drones.forEach((d, i) => {
      const color = DRONE_COLORS[i % DRONE_COLORS.length]
      L.geoJSON(d.region, { ...out, style: { color, weight: 1, fillOpacity: 0.08, dashArray: '2 4' } }).addTo(g)
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
          L.polyline(latlngs, { ...style, ...out })
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

  // перецентрирование — только по явному запросу (загрузка сценария, импорт)
  useEffect(() => {
    const m = map.current
    if (!m || !fit.bbox) return
    const [w, s, e, n] = fit.bbox
    m.fitBounds(L.latLngBounds([s, w], [n, e]).pad(0.08), { maxZoom: 16 })
  }, [fit])

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
