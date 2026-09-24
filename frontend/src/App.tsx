import { useEffect, useMemo, useRef, useState, type ChangeEvent } from 'react'
import './App.css'
import MapView, { DRONE_COLORS, type DrawMode, type FitRequest } from './MapView'
import ImportChooser, { AirspaceChooser } from './ImportChooser'
import {
  api,
  ApiError,
  type AdviceResponse,
  type Fleet,
  type GeodataSet,
  type KmlImport,
  type PlanRequest,
  type PlanResponse,
  type SurveyType,
} from './api'
import { AdviceChart, Gantt, ParetoChart } from './charts'
import {
  airspaceEmpty,
  airspaceSummary,
  applyAirspace,
  applyEdit,
  applyImport,
  areaFromFeatures,
  bboxOf,
  COVERAGE_OK_PCT,
  criterionLabel,
  datetimeLocalValue,
  droneIdErrors,
  EMPTY_REQUEST as EMPTY,
  filterAirspaceNear,
  formatCoverage,
  formatDuration,
  formatMaintenance,
  isEpsilonPlan,
  needsServerImport,
  nextBaseId,
  nextDroneId,
  nextFreeId,
  normalizeScenario,
  parseDuration,
  parseGeometryFile,
  parseScenarioText,
  requestBBox,
  requestKey,
  scenarioJSON,
  ScenarioError,
  splitAirspaceFeatures,
  type AirspaceChoice,
  type AirspaceImport,
  type BBox,
  type Criterion,
  type EditTarget,
  type EditValue,
  type ExcludedInfo,
  type ImportChoice,
  type ImportedGeometry,
} from './logic'

const SURVEY_TYPES: { id: SurveyType; label: string }[] = [
  { id: 'rgb', label: 'RGB' },
  { id: 'multispectral', label: 'Мультиспектральная' },
  { id: 'thermal', label: 'ИК (тепловизор)' },
  { id: 'lidar', label: 'LiDAR' },
  { id: 'geophysics', label: 'Геофизическая' },
]

const GEOMETRY_ACCEPT = '.kml,.geojson,.json,application/vnd.google-earth.kml+xml,application/geo+json,application/json'

const DRAW_BUTTONS: { mode: Exclude<DrawMode, null>; label: string }[] = [
  { mode: 'survey', label: 'Область съёмки' },
  { mode: 'allowed', label: 'Разрешённая зона' },
  { mode: 'nfz', label: '+ Запретная зона' },
  { mode: 'base', label: '+ ВПП' },
  { mode: 'reserve', label: '+ Резервная площадка' },
]

const min = (s: number) => `${(s / 60).toFixed(1)} мин`

/** По каким входным данным и с каким критерием построен показанный результат. */
interface Shown {
  key: string
  weight: number // положение ползунка на момент расчёта/выбора точки
  criterion: Criterion
}

interface UiError {
  message: string
  items: string[]
  excluded: ExcludedInfo[]
}

const toUiError = (e: unknown): UiError =>
  e instanceof ApiError
    ? { message: e.message, items: e.items, excluded: e.excluded }
    : e instanceof ScenarioError
      ? { message: e.message, items: e.items, excluded: [] }
      : { message: (e as Error).message, items: [], excluded: [] }

/** Блок ошибки: сообщение, список замечаний, исключённые борта. */
function ErrorBox({ error }: { error: UiError }) {
  return (
    <div className="error" role="alert">
      <p>{error.message}</p>
      {error.items.length > 0 && (
        <ul>
          {error.items.map((t, i) => <li key={i}>{t}</li>)}
        </ul>
      )}
      {error.excluded.length > 0 && (
        <>
          <p>Исключённые борта:</p>
          <ul>
            {error.excluded.map((e) => <li key={e.drone_id}><b>{e.drone_id}</b>: {e.reason}</li>)}
          </ul>
        </>
      )}
    </div>
  )
}

/** Отдать пользователю текст как файл (скачивание). */
function download(name: string, text: string, type: string) {
  const url = URL.createObjectURL(new Blob([text], { type }))
  const a = document.createElement('a')
  a.href = url
  a.download = name
  document.body.appendChild(a)
  a.click()
  a.remove()
  setTimeout(() => URL.revokeObjectURL(url), 1000)
}

// номер для окон выбора: React-ключ, по которому окно пересоздаётся на каждый новый файл
let pendingSeq = 0
const nextPendingId = () => ++pendingSeq

const pad2 = (n: number) => String(n).padStart(2, '0')
const fileStamp = (d: Date) =>
  `${d.getFullYear()}-${pad2(d.getMonth() + 1)}-${pad2(d.getDate())}_${pad2(d.getHours())}-${pad2(d.getMinutes())}`

/** Прочитанный, но ещё не назначенный файл геометрии. */
interface PendingImport {
  id: number
  file: string
  data: ImportedGeometry
}

/** Разобранные данные заказчика (выгрузка или KML), которые пользователь ещё не подтвердил. */
interface PendingAirspace {
  id: number
  source: string
  kind: KmlImport['kind'] | ''
  data: AirspaceImport
}

/** Ссылка экспорта; у устаревшего плана — неактивна. */
function ExportLink({ href, stale, className, children }: { href: string; stale: boolean; className?: string; children: string }) {
  if (stale)
    return (
      <span className={`${className ?? ''} disabled`} aria-disabled="true" title="План устарел — пересчитайте">
        {children}
      </span>
    )
  return <a className={className} href={href}>{children}</a>
}

export default function App() {
  const [req, setReq] = useState<PlanRequest>(EMPTY)
  const [fleet, setFleet] = useState<Fleet | null>(null)
  const [scenarios, setScenarios] = useState<string[]>([])
  const [plan, setPlan] = useState<PlanResponse | null>(null)
  const [drawMode, setDrawMode] = useState<DrawMode>(null)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<UiError | null>(null)
  const [fit, setFit] = useState<FitRequest>({ key: 0, bbox: null })
  const [editing, setEditing] = useState(false)
  const [scenarioName, setScenarioName] = useState('')
  const [scenarioError, setScenarioError] = useState<UiError | null>(null)
  const [pendingImport, setPendingImport] = useState<PendingImport | null>(null)
  const [importError, setImportError] = useState<UiError | null>(null)
  const scenarioFile = useRef<HTMLInputElement>(null)
  const geometryFile = useRef<HTMLInputElement>(null)
  const [front, setFront] = useState<PlanResponse[] | null>(null)
  const [paretoBusy, setParetoBusy] = useState(false)
  const [shown, setShown] = useState<Shown | null>(null)
  const [elapsed, setElapsed] = useState(0)
  // советник: срок работ («чч:мм» или минуты) и ограничение на число бортов
  const [deadlineText, setDeadlineText] = useState('')
  const [maxDronesText, setMaxDronesText] = useState('')
  const [advice, setAdvice] = useState<AdviceResponse | null>(null)
  const [adviceBusy, setAdviceBusy] = useState(false)
  const [adviceError, setAdviceError] = useState<UiError | null>(null)
  const [adviceShown, setAdviceShown] = useState<{ key: string; deadline: number | null } | null>(null)
  // данные о воздушном пространстве
  const [geodata, setGeodata] = useState<GeodataSet[] | null>(null)
  const [geodataBusy, setGeodataBusy] = useState(false)
  const [airspaceError, setAirspaceError] = useState<UiError | null>(null)
  const [airspaceNote, setAirspaceNote] = useState<string | null>(null)
  const [pendingAirspace, setPendingAirspace] = useState<PendingAirspace | null>(null)
  const [nearOnly, setNearOnly] = useState(true)
  const [marginKm, setMarginKm] = useState(10)
  const [showRestrictions, setShowRestrictions] = useState(true)
  const [showObstacles, setShowObstacles] = useState(true)
  const computing = busy || paretoBusy || adviceBusy

  // секундомер для долгих расчётов (фронт Парето — 10–15 с, советник — 5–60 с)
  useEffect(() => {
    if (!computing) return
    const t0 = Date.now()
    const t = setInterval(() => setElapsed(Math.floor((Date.now() - t0) / 1000)), 250)
    return () => clearInterval(t)
  }, [computing])

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== 'Escape') return
      setDrawMode(null)
      setEditing(false)
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [])

  useEffect(() => {
    api.fleet().then(setFleet).catch((e) => setError({ message: `API недоступен: ${e.message}`, items: [], excluded: [] }))
    api.scenarios().then(setScenarios).catch(() => {})
  }, [])

  const fitTo = (bbox: BBox | null) => bbox && setFit((f) => ({ key: f.key + 1, bbox }))

  // новый сценарий: прежний план и фронт к нему не относятся
  const applyScenario = (s: PlanRequest) => {
    setReq(s)
    setPlan(null)
    setFront(null)
    setShown(null)
    setError(null)
    setScenarioError(null)
    setAdvice(null)
    setAdviceShown(null)
    setAdviceError(null)
    setPendingAirspace(null)
    setAirspaceNote(null)
    setAirspaceError(null)
    fitTo(requestBBox(s))
  }

  const loadScenario = async (name: string) => {
    setScenarioName(name)
    if (!name) return
    try {
      applyScenario(normalizeScenario(await api.scenario(name)))
    } catch (e) {
      setScenarioError(toUiError(e))
    }
  }

  const saveScenario = () => download(`scenario_${fileStamp(new Date())}.json`, scenarioJSON(req), 'application/json')

  const onScenarioFile = async (e: ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0]
    e.target.value = '' // чтобы повторный выбор того же файла снова сработал
    if (!file) return
    try {
      applyScenario(parseScenarioText(await file.text()))
      setScenarioName('')
    } catch (err) {
      setScenarioError(toUiError(err))
    }
  }

  /** Показать окно выбора для разобранных данных заказчика. Файл с одними полигонами задания
   *  нужен, чтобы задать область съёмки, — отбор «вблизи области» для него выключаем. */
  const openAirspace = (p: PendingAirspace) => {
    setNearOnly(p.data.restrictions.length + p.data.obstacles.length > 0)
    setPendingAirspace(p)
  }

  /** KML заказчика разбирает бэкенд: у него есть разбор высот зон и 3D-примитивов препятствий. */
  const importOnServer = async (name: string, text: string, notes: string[] = []) => {
    setGeodataBusy(true)
    try {
      const res = await api.importKml(text)
      const data = splitAirspaceFeatures(res.features)
      if (airspaceEmpty(data)) throw new Error('в файле нет зон ограничений, препятствий и полигонов задания')
      openAirspace({ id: nextPendingId(), source: name, kind: res.kind, data })
      setAirspaceNote(null)
    } catch (e) {
      const ui = toUiError(e)
      setImportError({ ...ui, items: [...notes, ...ui.items] })
    } finally {
      setGeodataBusy(false)
    }
  }

  const onGeometryFile = async (e: ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0]
    e.target.value = ''
    if (!file) return
    setImportError(null)
    setPendingImport(null)
    setPendingAirspace(null)
    const text = await file.text()
    const isKml = /\.(kml|xml)$/i.test(file.name) || text.trimStart().startsWith('<')
    let data: ImportedGeometry | null = null
    let problem = ''
    let warnings: string[] = []
    try {
      data = parseGeometryFile(file.name, text)
      warnings = data.warnings
      if (!data.polygons.length && !data.points.length) {
        problem = `В файле ${file.name} нет полигонов и точек.`
        data = null
      }
    } catch (err) {
      problem = (err as Error).message
    }
    // данные заказчика (высоты зон, 3D-препятствия) и задание из многих полигонов разбирает
    // бэкенд: наш парсер свёл бы их к простым контурам без высот и без выбора области
    if (isKml && needsServerImport(text, data))
      return importOnServer(file.name, text, problem ? [problem, ...warnings] : warnings)
    if (!data) return setImportError({ message: problem, items: warnings, excluded: [] })
    setPendingImport({ id: nextPendingId(), file: file.name, data })
  }

  /** Список выгрузок данных заказчика (повторное нажатие — закрыть). */
  const openGeodata = async () => {
    setAirspaceError(null)
    if (geodata) {
      setGeodata(null)
      return
    }
    setGeodataBusy(true)
    try {
      setGeodata(await api.geodata())
    } catch (e) {
      setAirspaceError(toUiError(e))
    } finally {
      setGeodataBusy(false)
    }
  }

  const loadGeodataSet = async (s: GeodataSet) => {
    setAirspaceError(null)
    setAirspaceNote(null)
    if (!req.survey_area) {
      setAirspaceError({
        message: 'Сначала задайте область съёмки: по ней отбираются зоны и препятствия вблизи работ (в выгрузке их тысячи).',
        items: [],
        excluded: [],
      })
      return
    }
    setGeodataBusy(true)
    try {
      const data = splitAirspaceFeatures(await api.geodataSet(s.name))
      openAirspace({ id: nextPendingId(), source: `${s.name} · ${s.title}`, kind: '', data })
      setGeodata(null)
    } catch (e) {
      setAirspaceError(toUiError(e))
    } finally {
      setGeodataBusy(false)
    }
  }

  // что из разобранной выгрузки попадёт в запрос: отбор вблизи области съёмки
  const airspaceNear = useMemo(() => {
    if (!pendingAirspace) return null
    return nearOnly && req.survey_area
      ? filterAirspaceNear(pendingAirspace.data, req.survey_area, marginKm)
      : pendingAirspace.data
  }, [pendingAirspace, nearOnly, marginKm, req.survey_area])

  const applyAirspaceChoice = (choice: AirspaceChoice) => {
    if (!pendingAirspace || !airspaceNear) return
    const next = applyAirspace(req, airspaceNear, choice)
    setReq(next)
    // отчёт «сколько добавлено из скольких»: было в выгрузке → попало в запрос
    const added: string[] = []
    if (choice.restrictions && airspaceNear.restrictions.length)
      added.push(
        `зон ограничений +${next.restrictions.length - req.restrictions.length} из ${pendingAirspace.data.restrictions.length}`,
      )
    if (choice.obstacles && airspaceNear.obstacles.length)
      added.push(`препятствий +${next.obstacles.length - req.obstacles.length} из ${pendingAirspace.data.obstacles.length}`)
    if (choice.area !== 'none') {
      const area = areaFromFeatures(airspaceNear.areas, choice.area)
      if (area) {
        added.push('область съёмки заменена')
        fitTo(bboxOf([area]))
      }
    }
    setAirspaceNote(`${pendingAirspace.source}: ${added.join(', ') || 'ничего не добавлено'}.`)
    setPendingAirspace(null)
  }

  const applyImported = (choice: ImportChoice) => {
    if (!pendingImport) return
    const { data } = pendingImport
    setReq((r) => applyImport(r, data, choice))
    fitTo(bboxOf(choice.polygons ? data.polygons : [], choice.points ? data.points : []))
    setPendingImport(null)
  }

  // правка на карте идёт через тот же req — план от этого становится устаревшим
  const onEdited = (target: EditTarget, value: EditValue) => setReq((r) => applyEdit(r, target, value))

  const chooseDraw = (mode: Exclude<DrawMode, null>) => {
    setEditing(false)
    setDrawMode(drawMode === mode ? null : mode)
  }

  const toggleEditing = () => {
    setDrawMode(null)
    setEditing((v) => !v)
  }

  const onDrawn = (mode: Exclude<DrawMode, null>, g: GeoJSON.Polygon | { lon: number; lat: number }) => {
    setDrawMode(null)
    setReq((r) => {
      if ('type' in g) {
        if (mode === 'survey') return { ...r, survey_area: g }
        if (mode === 'allowed') return { ...r, allowed_area: g }
        return { ...r, no_fly_zones: [...r.no_fly_zones, g] }
      }
      if (mode === 'base') {
        const id = nextBaseId(r.bases.map((b) => b.id))
        return { ...r, bases: [...r.bases, { id, name: '', ...g }] }
      }
      return { ...r, reserve_sites: [...r.reserve_sites, { id: nextFreeId('R', r.reserve_sites.map((s) => s.id)), ...g }] }
    })
  }

  const addDrone = (model: string) => {
    if (!model) return
    setReq((r) => {
      const id = nextDroneId(model, r.drones)
      return { ...r, drones: [...r.drones, { id, model, base_id: r.bases[0]?.id ?? null }] }
    })
  }

  const run = async () => {
    const snapshot = req
    setElapsed(0)
    setBusy(true)
    setError(null)
    try {
      const p = await api.plan(snapshot)
      setPlan(p)
      setFront(null)
      setShown({ key: requestKey(snapshot), weight: snapshot.time_weight, criterion: { kind: 'weighted', w: p.time_weight } })
    } catch (e) {
      setError(toUiError(e))
      setPlan(null)
      setShown(null)
    } finally {
      setBusy(false)
    }
  }

  // выбор точки фронта: план не устаревает, ползунок встаёт на вес плана (для ε-плана — не трогаем)
  const selectFrontPlan = (p: PlanResponse, f: PlanResponse[], key: string, weightNow: number) => {
    const eps = isEpsilonPlan(p, f)
    const w = eps ? weightNow : p.time_weight
    setPlan(p)
    setShown({ key, weight: w, criterion: { kind: 'front', w: eps ? null : p.time_weight } })
    if (!eps) setReq((r) => ({ ...r, time_weight: p.time_weight }))
  }

  const runPareto = async () => {
    const snapshot = req
    setElapsed(0)
    setParetoBusy(true)
    setError(null)
    try {
      const f = await api.pareto(snapshot)
      setFront(f)
      // по умолчанию показываем самый быстрый план фронта
      if (f[0]) selectFrontPlan(f[0], f, requestKey(snapshot), snapshot.time_weight)
      else {
        setPlan(null)
        setShown(null)
      }
    } catch (e) {
      setError(toUiError(e))
    } finally {
      setParetoBusy(false)
    }
  }

  // срок работ: «2:30» или число минут; строка — текст ошибки, null — срок не задан
  const deadline = useMemo(() => parseDuration(deadlineText), [deadlineText])
  const deadlineS = typeof deadline === 'number' ? deadline : null
  const deadlineError = typeof deadline === 'string' ? deadline : null
  const maxDrones = maxDronesText.trim() ? Math.max(1, Math.floor(+maxDronesText)) : null

  const runAdvise = async () => {
    const snapshot = req
    setElapsed(0)
    setAdviceBusy(true)
    setAdviceError(null)
    try {
      const a = await api.advise(snapshot, { deadline_s: deadlineS, max_drones: maxDrones })
      setAdvice(a)
      setAdviceShown({ key: requestKey(snapshot), deadline: deadlineS })
    } catch (e) {
      const ui = toUiError(e)
      // бэкенд считает фронт Парето и подсказку по одной за раз
      if (e instanceof ApiError && e.status === 503)
        ui.items = ['Одновременно идёт только один тяжёлый расчёт (фронт Парето или советник) — подождите и повторите.']
      setAdviceError(ui)
      setAdvice(null)
      setAdviceShown(null)
    } finally {
      setAdviceBusy(false)
    }
  }

  const idErrors = useMemo(() => droneIdErrors(req.drones), [req.drones])
  const hasIdErrors = Object.keys(idErrors).length > 0
  const currentKey = useMemo(() => requestKey(req), [req])
  const stale = !!shown && (shown.key !== currentKey || shown.weight !== req.time_weight)
  const frontStale = !!shown && shown.key !== currentKey
  const epsSelected = shown?.criterion.kind === 'front' && shown.criterion.w === null && !stale
  const canRun = req.survey_area && req.bases.length > 0 && req.drones.length > 0 && !hasIdErrors && !computing
  const setReqField = <K extends keyof PlanRequest['requirements']>(k: K, v: PlanRequest['requirements'][K]) =>
    setReq((r) => ({ ...r, requirements: { ...r.requirements, [k]: v } }))

  const adviceStale = !!adviceShown && adviceShown.key !== currentKey
  const canAdvise = !!canRun && !deadlineError

  // на карте — воздушное пространство из ответа (там есть applies и skip_reason), иначе из запроса
  const mapRestrictions = useMemo(
    () => (plan && !stale && plan.restrictions?.length ? plan.restrictions : req.restrictions),
    [plan, stale, req.restrictions],
  )
  const mapObstacles = useMemo(
    () => (plan && !stale && plan.obstacles?.length ? plan.obstacles : req.obstacles),
    [plan, stale, req.obstacles],
  )

  const droneColor = useMemo(() => {
    const m: Record<string, string> = {}
    plan?.drones.forEach((d, i) => (m[d.drone_id] = DRONE_COLORS[i % DRONE_COLORS.length]))
    return m
  }, [plan])

  return (
    <div className="layout">
      <aside className="panel">
        <h1>Планировщик парка БВС</h1>

        <section>
          <h2>Сценарий</h2>
          <select value={scenarioName} onChange={(e) => loadScenario(e.target.value)}>
            <option value="">— загрузить тестовый сценарий —</option>
            {scenarios.map((s) => (
              <option key={s}>{s}</option>
            ))}
          </select>
          <div className="buttons file-buttons">
            <button onClick={saveScenario} title="Скачать текущие входные данные как JSON-файл сценария">
              Сохранить сценарий
            </button>
            <button onClick={() => scenarioFile.current?.click()} title="Открыть JSON-файл сценария (формат запроса /api/plan)">
              Загрузить сценарий
            </button>
          </div>
          <input ref={scenarioFile} type="file" accept=".json,application/json" hidden onChange={onScenarioFile} />
          {scenarioError && <ErrorBox error={scenarioError} />}
        </section>

        <section>
          <h2>Геометрия</h2>
          <div className="buttons">
            {DRAW_BUTTONS.map((b) => (
              <button key={b.mode} className={drawMode === b.mode ? 'active' : ''} onClick={() => chooseDraw(b.mode)}>
                {b.label}
              </button>
            ))}
          </div>
          <div className="buttons file-buttons">
            <button onClick={() => geometryFile.current?.click()} title="Полигоны и точки из файла KML или GeoJSON (WGS84)">
              Импорт KML/GeoJSON
            </button>
            <button
              className={editing ? 'active' : ''}
              aria-pressed={editing}
              onClick={toggleEditing}
              title="Перетаскивание вершин полигонов, ВПП и резервных площадок"
            >
              Редактировать геометрию
            </button>
          </div>
          <input ref={geometryFile} type="file" accept={GEOMETRY_ACCEPT} hidden onChange={onGeometryFile} />
          {importError && <ErrorBox error={importError} />}
          {pendingImport && (
            <ImportChooser
              key={pendingImport.id}
              file={pendingImport.file}
              data={pendingImport.data}
              initial={{
                polygons: req.survey_area ? 'nfz' : 'survey',
                points: req.bases.length ? 'reserve' : 'base',
              }}
              has={{ survey: !!req.survey_area, allowed: !!req.allowed_area }}
              onApply={applyImported}
              onCancel={() => setPendingImport(null)}
            />
          )}
          <div className="objects">
            {req.survey_area && (
              <span className="chip">
                область съёмки
                <button className="link" onClick={() => setReq({ ...req, survey_area: null })}>✕</button>
              </span>
            )}
            {req.allowed_area && (
              <span className="chip">
                разрешённая зона
                <button className="link" onClick={() => setReq({ ...req, allowed_area: null })}>✕</button>
              </span>
            )}
            {req.no_fly_zones.map((_, i) => (
              <span className="chip nfz" key={`z${i}`}>
                NFZ {i + 1}
                <button className="link" onClick={() => setReq({ ...req, no_fly_zones: req.no_fly_zones.filter((_, j) => j !== i) })}>✕</button>
              </span>
            ))}
            {req.bases.map((b, i) => (
              <span className="chip" key={`b${b.id}`}>
                ВПП {b.id}
                <button className="link" onClick={() => setReq({
                  ...req,
                  bases: req.bases.filter((_, j) => j !== i),
                  drones: req.drones.map((d) => (d.base_id === b.id ? { ...d, base_id: null } : d)),
                })}>✕</button>
              </span>
            ))}
            {req.reserve_sites.map((r, i) => (
              <span className="chip reserve" key={`r${r.id}`}>
                {r.id}
                <button className="link" onClick={() => setReq({ ...req, reserve_sites: req.reserve_sites.filter((_, j) => j !== i) })}>✕</button>
              </span>
            ))}
            <button className="link" onClick={() => { setReq(EMPTY); setPlan(null); setFront(null); setShown(null); setError(null); setScenarioName(''); setAdvice(null); setAdviceShown(null); setAirspaceNote(null); setPendingAirspace(null) }}>
              очистить всё
            </button>
          </div>
        </section>

        <section>
          <h2>Воздушное пространство</h2>
          <div className="buttons">
            <button
              className={geodata ? 'active' : ''}
              disabled={geodataBusy}
              onClick={openGeodata}
              title="Выгрузки заказчика: зоны ограничений, высотные препятствия, полигоны задания"
            >
              {geodataBusy ? (
                <>
                  <span className="spinner dark" /> Загрузка…
                </>
              ) : (
                'Данные о воздушном пространстве'
              )}
            </button>
          </div>
          {geodata && (
            <ul className="geodata">
              {geodata.map((s) => (
                <li key={s.name}>
                  <button className="link" onClick={() => loadGeodataSet(s)}>
                    {s.name}
                  </button>
                  <small>
                    {s.title} · объектов {s.features} · {s.size_kb} КБ
                  </small>
                </li>
              ))}
              {!geodata.length && <li>выгрузок рядом с сервисом нет</li>}
            </ul>
          )}
          {airspaceError && <ErrorBox error={airspaceError} />}
          {airspaceNote && <p className="hint">{airspaceNote}</p>}
          {pendingAirspace && airspaceNear && (
            <AirspaceChooser
              key={pendingAirspace.id}
              source={pendingAirspace.source}
              kind={pendingAirspace.kind}
              data={airspaceNear}
              total={pendingAirspace.data}
              hasArea={!!req.survey_area}
              nearOnly={nearOnly}
              marginKm={marginKm}
              onNearOnly={setNearOnly}
              onMarginKm={setMarginKm}
              onApply={applyAirspaceChoice}
              onCancel={() => setPendingAirspace(null)}
            />
          )}
          <div className="objects">
            {req.restrictions.length > 0 && (
              <span className="chip zone">
                зоны ограничений: {req.restrictions.length}
                <button className="link" onClick={() => setReq({ ...req, restrictions: [] })}>
                  ✕
                </button>
              </span>
            )}
            {req.obstacles.length > 0 && (
              <span className="chip obstacle">
                препятствия: {req.obstacles.length}
                <button className="link" onClick={() => setReq({ ...req, obstacles: [] })}>
                  ✕
                </button>
              </span>
            )}
            {!req.restrictions.length && !req.obstacles.length && (
              <span className="hint">Зоны и препятствия не заданы — планировщик учтёт только запретные зоны с карты.</span>
            )}
          </div>
          <label>
            <span>Показывать зоны ограничений</span>
            <input
              type="checkbox"
              checked={showRestrictions}
              onChange={(e) => setShowRestrictions(e.target.checked)}
              disabled={!mapRestrictions.length}
            />
          </label>
          <label>
            <span>Показывать препятствия</span>
            <input
              type="checkbox"
              checked={showObstacles}
              onChange={(e) => setShowObstacles(e.target.checked)}
              disabled={!mapObstacles.length}
            />
          </label>
        </section>

        <section>
          <h2>Парк</h2>
          {req.drones.map((d, i) => (
            <div className="drone" key={i}>
            <div className="row">
              <span className="swatch" style={{ background: droneColor[d.id] ?? '#ccc' }} />
              <input
                className={idErrors[i] ? 'id invalid' : 'id'}
                aria-invalid={!!idErrors[i]}
                value={d.id}
                onChange={(e) =>
                  setReq((r) => ({ ...r, drones: r.drones.map((x, j) => (j === i ? { ...x, id: e.target.value } : x)) }))
                }
              />
              <span className="model">{fleet?.drones[d.model]?.name ?? d.model}</span>
              <select
                value={d.base_id ?? ''}
                onChange={(e) =>
                  setReq((r) => ({
                    ...r,
                    drones: r.drones.map((x, j) => (j === i ? { ...x, base_id: e.target.value || null } : x)),
                  }))
                }
              >
                <option value="">авто</option>
                {req.bases.map((b) => (
                  <option key={b.id} value={b.id}>
                    ВПП {b.id}
                  </option>
                ))}
              </select>
              <button className="link" onClick={() => setReq((r) => ({ ...r, drones: r.drones.filter((_, j) => j !== i) }))}>
                ✕
              </button>
            </div>
            {idErrors[i] && <div className="field-error">{idErrors[i]}</div>}
            </div>
          ))}
          <select value="" onChange={(e) => addDrone(e.target.value)}>
            <option value="">+ добавить борт</option>
            {fleet &&
              Object.values(fleet.drones).map((d) => (
                <option key={d.id} value={d.id}>
                  {d.name} ({d.type === 'fixed_wing' ? 'самолёт' : 'мультиротор'}, {d.endurance_min} мин)
                </option>
              ))}
          </select>
        </section>

        <section>
          <h2>Съёмка</h2>
          <label>
            Тип
            <select value={req.survey_type} onChange={(e) => setReq({ ...req, survey_type: e.target.value as SurveyType })}>
              {SURVEY_TYPES.map((t) => (
                <option key={t.id} value={t.id}>
                  {t.label}
                </option>
              ))}
            </select>
          </label>
          {['rgb', 'multispectral', 'thermal'].includes(req.survey_type) && (
            <>
              <label>
                GSD, см/пикс
                <input type="number" step="0.5" min="0.5" value={req.requirements.gsd_cm ?? ''}
                  onChange={(e) => setReqField('gsd_cm', e.target.value ? +e.target.value : null)} />
              </label>
              <label>
                Продольное перекрытие, %
                <input type="number" min="0" max="95" value={Math.round(req.requirements.front_overlap * 100)}
                  onChange={(e) => setReqField('front_overlap', +e.target.value / 100)} />
              </label>
            </>
          )}
          {req.survey_type === 'lidar' && (
            <label>
              Плотность, точек/м²
              <input type="number" min="1" value={req.requirements.lidar_density_pts_m2}
                onChange={(e) => setReqField('lidar_density_pts_m2', +e.target.value)} />
            </label>
          )}
          {req.survey_type === 'geophysics' && (
            <label>
              Шаг галсов, м
              <input type="number" min="5" value={req.requirements.line_spacing_m ?? 50}
                onChange={(e) => setReqField('line_spacing_m', +e.target.value)} />
            </label>
          )}
          {req.survey_type === 'lidar' && (
            <label>
              Перекрытие полос LiDAR, %
              <input type="number" min="0" max="90" value={Math.round((req.requirements.lidar_side_overlap ?? 0.2) * 100)}
                onChange={(e) => setReqField('lidar_side_overlap', +e.target.value / 100)} />
            </label>
          )}
          {req.survey_type !== 'geophysics' && req.survey_type !== 'lidar' && (
            <label>
              Поперечное перекрытие, %
              <input type="number" min="0" max="95" value={Math.round(req.requirements.side_overlap * 100)}
                onChange={(e) => setReqField('side_overlap', +e.target.value / 100)} />
            </label>
          )}
          <label>
            Потолок высоты, м AGL
            <input type="number" min="0" placeholder="нет" value={req.requirements.altitude_ceiling_m ?? ''}
              onChange={(e) => setReqField('altitude_ceiling_m', e.target.value ? +e.target.value : null)} />
          </label>
        </section>

        <section>
          <h2>Условия</h2>
          <label>
            Ветер, м/с
            <input type="number" min="0" max="25" value={req.wind.speed_ms}
              onChange={(e) => setReq({ ...req, wind: { ...req.wind, speed_ms: +e.target.value } })} />
          </label>
          <label>
            Ветер задан
            <select value={req.wind.ref_height_m == null ? '' : String(req.wind.ref_height_m)}
              onChange={(e) => setReq({ ...req, wind: { ...req.wind, ref_height_m: e.target.value ? +e.target.value : null } })}>
              <option value="">на высоте полёта</option>
              <option value="10">у земли, 10 м (метеостанция)</option>
            </select>
          </label>
          <label>
            Откуда дует, °
            <input type="number" min="0" max="359" value={req.wind.from_deg}
              onChange={(e) => setReq({ ...req, wind: { ...req.wind, from_deg: +e.target.value } })} />
          </label>
          <label>
            Резерв заряда, %
            <input type="number" min="0" max="60" value={Math.round(req.reserve * 100)}
              onChange={(e) => setReq({ ...req, reserve: +e.target.value / 100 })} />
          </label>
          <label>
            Начало работ
            <input type="datetime-local" value={datetimeLocalValue(req.mission_start)}
              onChange={(e) => setReq({ ...req, mission_start: e.target.value || null })} />
          </label>
          <label>
            Окно работ, ч
            <input type="number" min="1" max="168" value={req.mission_window_h}
              onChange={(e) => setReq({ ...req, mission_window_h: +e.target.value })} />
          </label>
          <p className="hint">Окно работ нужно временным зонам: зона, не действующая в это время, в расчёт не идёт.</p>
          <label>
            Зазор над препятствием, м
            <input type="number" min="0" max="1000" value={req.obstacle_clearance_m}
              onChange={(e) => setReq({ ...req, obstacle_clearance_m: +e.target.value })} />
          </label>
          <label>
            Обход препятствия, м
            <input type="number" min="0" max="10000" value={req.obstacle_buffer_m}
              onChange={(e) => setReq({ ...req, obstacle_buffer_m: +e.target.value })} />
          </label>
        </section>

        <section>
          <h2>Критерий оптимизации</h2>
          <div className="criterion">
            <span>суммарный налёт</span>
            <input type="range" min="0" max="1" step="0.05" value={req.time_weight}
              onChange={(e) => setReq({ ...req, time_weight: +e.target.value })} />
            <span>время работ</span>
          </div>
          <p className="hint criterion-value">
            {epsSelected
              ? 'ε-ограничение: выбранный план фронта построен не взвешенной суммой, вес к нему не относится'
              : `Вес «время работ» w = ${req.time_weight}`}
          </p>
          <button className="primary" disabled={!canRun} onClick={run}>
            {busy ? <><span className="spinner" /> Расчёт… {elapsed} с</> : 'Рассчитать план'}
          </button>
          <button className="secondary" disabled={!canRun} onClick={runPareto}>
            {paretoBusy ? <><span className="spinner dark" /> Строим фронт Парето… {elapsed} с</> : 'Сравнить варианты (фронт Парето)'}
          </button>
          {hasIdErrors && <p className="field-error">Исправьте id бортов — они должны быть уникальными и непустыми.</p>}
          {front && front.length > 0 && (
            <div className={frontStale ? 'pareto stale' : 'pareto'}>
              <p className="hint">
                Каждая точка — недоминируемый план. Левее — быстрее, ниже — меньше суммарный налёт. Нажмите на точку, чтобы
                открыть план.
              </p>
              <ParetoChart
                front={front}
                selected={plan?.plan_id ?? null}
                onSelect={(p) => shown && !frontStale && selectFrontPlan(p, front, shown.key, req.time_weight)}
              />
            </div>
          )}
          {error && <ErrorBox error={error} />}
        </section>

        <section>
          <h2>Советник</h2>
          <label>
            Срок работ
            <input
              type="text"
              inputMode="text"
              placeholder="чч:мм или минуты"
              className={deadlineError ? 'invalid' : ''}
              aria-invalid={!!deadlineError}
              value={deadlineText}
              onChange={(e) => setDeadlineText(e.target.value)}
            />
          </label>
          {deadlineError ? (
            <p className="field-error">{deadlineError}</p>
          ) : deadlineS ? (
            <p className="hint">Срок — {formatDuration(deadlineS)}.</p>
          ) : (
            <p className="hint">Без срока подскажем, за сколько справится парк и сколько бортов для этого нужно.</p>
          )}
          <label>
            Не больше бортов
            <input
              type="number"
              min="1"
              max={Math.max(req.drones.length, 1)}
              placeholder="весь парк"
              value={maxDronesText}
              onChange={(e) => setMaxDronesText(e.target.value)}
            />
          </label>
          <button className="secondary" disabled={!canAdvise} onClick={runAdvise}>
            {adviceBusy ? (
              <>
                <span className="spinner dark" /> Считаем варианты… {elapsed} с
              </>
            ) : (
              'Подсказать'
            )}
          </button>
          <p className="hint">Расчёт 5–60 с: строится план для 1, 2, … бортов по убыванию производительности.</p>
          {adviceError && <ErrorBox error={adviceError} />}
          {advice && (
            <div className={adviceStale ? 'advice stale' : 'advice'}>
              {adviceStale && <p className="hint">Входные данные изменились — подсказка относится к прежним.</p>}
              <p className="advice-message">{advice.message}</p>
              <AdviceChart options={advice.options} deadlineS={adviceShown?.deadline ?? null} />
              <table>
                <thead>
                  <tr>
                    <th>Бортов</th>
                    <th>Борта</th>
                    <th>Работы</th>
                    <th>Налёт</th>
                    <th>Вылетов</th>
                    <th>Покрытие</th>
                  </tr>
                </thead>
                <tbody>
                  {advice.options.map((o, i) => (
                    <tr
                      key={i}
                      className={o.reason ? 'bad' : advice.drones_needed === o.drones ? 'pick' : ''}
                      title={o.reason || undefined}
                    >
                      <td>{o.drones}</td>
                      <td>
                        {o.drone_ids.join(', ') || '—'}
                        {o.reason && <small>не покрывает область</small>}
                      </td>
                      <td>{o.makespan_s != null ? min(o.makespan_s) : '—'}</td>
                      <td>{o.total_flight_s != null ? min(o.total_flight_s) : '—'}</td>
                      <td>{o.sorties ?? '—'}</td>
                      <td>{o.coverage_pct != null ? `${formatCoverage(o.coverage_pct)}%` : '—'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </section>

        {plan && (
          <section className="result">
            {stale && (
              <div className="stale-banner" role="status">
                Входные данные изменились — нажмите «Рассчитать план»
              </div>
            )}
            <div className={stale ? 'result-body stale' : 'result-body'}>
            <h2>Результат</h2>
            {shown && <div className="badge">Критерий: {criterionLabel(shown.criterion)}</div>}
            <div className="kpis">
              <div><b>{min(plan.summary.makespan_s)}</b><span>время работ</span></div>
              <div><b>{min(plan.summary.total_flight_s)}</b><span>суммарный налёт</span></div>
              <div><b>{plan.summary.sorties}</b><span>вылетов</span></div>
              <div
                className={plan.summary.coverage_pct < COVERAGE_OK_PCT ? 'warn' : ''}
                title={`снято ${plan.summary.covered_km2.toFixed(3)} из ${plan.summary.area_km2.toFixed(3)} км²`}
              >
                <b>{formatCoverage(plan.summary.coverage_pct)}%</b>
                <span>покрытие {plan.summary.area_km2.toFixed(2)} км²</span>
              </div>
            </div>
            <h2>Вылеты во времени, мин</h2>
            <Gantt plan={plan} colors={droneColor} />
            <table>
              <thead>
                <tr><th /><th>Борт</th><th>Выс., м</th><th>Вылеты</th><th>Налёт</th><th>Финиш</th><th>Уход*</th><th>Экспорт</th></tr>
              </thead>
              <tbody>
                {plan.drones.map((d) => (
                  <tr key={d.drone_id} title={d.params.notes.join('; ')}>
                    <td><span className="swatch" style={{ background: droneColor[d.drone_id] }} /></td>
                    <td>
                      {d.drone_id}
                      <small>{d.payload} · {d.area_km2.toFixed(2)} км²</small>
                      {formatMaintenance(d.maintenance) && (
                        <small
                          className={d.maintenance?.due ? 'due' : ''}
                          title={
                            d.maintenance?.interval_flights
                              ? `норматив: каждые ${d.maintenance.interval_flights} полётов, после задания — ${d.maintenance.flights_after}`
                              : `норматив: каждые ${d.maintenance?.interval_hours} ч, после задания — ${d.maintenance?.hours_after} ч`
                          }
                        >
                          {formatMaintenance(d.maintenance)}
                          {d.maintenance?.due ? ' — ТО в этом задании' : ''}
                        </small>
                      )}
                    </td>
                    <td>{d.params.altitude_agl_m.toFixed(0)}</td>
                    <td>{d.sorties.length}</td>
                    <td>{min(d.flight_time_s)}</td>
                    <td>{min(d.finish_s)}</td>
                    <td>{d.sorties.length ? min(Math.max(...d.sorties.map((s) => s.max_divert_s))) : '—'}</td>
                    <td>
                      <ExportLink stale={stale} href={api.exportUrl(plan.plan_id, d.drone_id, 'geojson')}>GeoJSON</ExportLink>{' '}
                      <ExportLink stale={stale} href={api.exportUrl(plan.plan_id, d.drone_id, 'kml')}>KML</ExportLink>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
            <p className="hint">* Уход — худшее время до ближайшей ВПП или резервной площадки с посадкой.</p>
            <ExportLink className="button" stale={stale} href={api.zipUrl(plan.plan_id)}>Скачать все задания (ZIP)</ExportLink>
            {plan.terrain && (
              <p className="hint">
                Рельеф {plan.terrain.ground_min_m.toFixed(0)}–{plan.terrain.ground_max_m.toFixed(0)} м. {plan.terrain.source}
              </p>
            )}
            {airspaceSummary(plan.airspace) && <p className="hint">{airspaceSummary(plan.airspace)}</p>}
            {plan.excluded.length > 0 && (
              <ul className="excluded">
                {plan.excluded.map((e) => (
                  <li key={e.drone_id}><b>{e.drone_id}</b>: {e.reason}</li>
                ))}
              </ul>
            )}
            {plan.warnings.length > 0 && (
              <ul className="warnings">
                {plan.warnings.map((w, i) => (
                  <li key={i}>{w}</li>
                ))}
              </ul>
            )}
            </div>
          </section>
        )}
      </aside>
      <main>
        <MapView
          req={req}
          plan={plan}
          stale={stale}
          drawMode={drawMode}
          onDrawn={onDrawn}
          editing={editing}
          onEdited={onEdited}
          fit={fit}
          restrictions={mapRestrictions}
          obstacles={mapObstacles}
          showRestrictions={showRestrictions}
          showObstacles={showObstacles}
          obstacleBufferM={req.obstacle_buffer_m}
        />
        <div className="map-notes">
          {drawMode && (
            <div className="draw-hint">
              Рисование: {DRAW_BUTTONS.find((b) => b.mode === drawMode)?.label}. Esc — отмена.
            </div>
          )}
          {editing && (
            <div className="draw-hint">
              Правка геометрии: тяните вершины, ВПП и резервные площадки; новая вершина — потяните за середину стороны,
              удалить вершину — правый клик. Esc — выход.
            </div>
          )}
          {plan && stale && <div className="map-stale">Маршруты на карте устарели — пересчитайте план</div>}
        </div>
      </main>
    </div>
  )
}
