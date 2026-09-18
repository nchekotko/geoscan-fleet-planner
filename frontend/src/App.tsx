import { useEffect, useMemo, useState } from 'react'
import './App.css'
import MapView, { DRONE_COLORS, type DrawMode } from './MapView'
import { api, ApiError, type Fleet, type PlanRequest, type PlanResponse, type SurveyType } from './api'
import { Gantt, ParetoChart } from './charts'
import {
  COVERAGE_OK_PCT,
  criterionLabel,
  droneIdErrors,
  formatCoverage,
  isEpsilonPlan,
  nextBaseId,
  nextDroneId,
  nextFreeId,
  requestKey,
  type Criterion,
  type ExcludedInfo,
} from './logic'

const SURVEY_TYPES: { id: SurveyType; label: string }[] = [
  { id: 'rgb', label: 'RGB' },
  { id: 'multispectral', label: 'Мультиспектральная' },
  { id: 'thermal', label: 'ИК (тепловизор)' },
  { id: 'lidar', label: 'LiDAR' },
  { id: 'geophysics', label: 'Геофизическая' },
]

const EMPTY: PlanRequest = {
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
    : { message: (e as Error).message, items: [], excluded: [] }

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
  const [fitKey, setFitKey] = useState(0)
  const [front, setFront] = useState<PlanResponse[] | null>(null)
  const [paretoBusy, setParetoBusy] = useState(false)
  const [shown, setShown] = useState<Shown | null>(null)
  const [elapsed, setElapsed] = useState(0)
  const computing = busy || paretoBusy

  // секундомер для долгих расчётов (фронт Парето — 10–15 с)
  useEffect(() => {
    if (!computing) return
    const t0 = Date.now()
    const t = setInterval(() => setElapsed(Math.floor((Date.now() - t0) / 1000)), 250)
    return () => clearInterval(t)
  }, [computing])

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => e.key === 'Escape' && setDrawMode(null)
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [])

  useEffect(() => {
    api.fleet().then(setFleet).catch((e) => setError({ message: `API недоступен: ${e.message}`, items: [], excluded: [] }))
    api.scenarios().then(setScenarios).catch(() => {})
  }, [])

  const loadScenario = async (name: string) => {
    if (!name) return
    const s = await api.scenario(name)
    setReq({ ...EMPTY, ...s, requirements: { ...EMPTY.requirements, ...s.requirements } })
    setPlan(null)
    setFront(null)
    setShown(null)
    setError(null)
    setFitKey((k) => k + 1)
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

  const idErrors = useMemo(() => droneIdErrors(req.drones), [req.drones])
  const hasIdErrors = Object.keys(idErrors).length > 0
  const currentKey = useMemo(() => requestKey(req), [req])
  const stale = !!shown && (shown.key !== currentKey || shown.weight !== req.time_weight)
  const frontStale = !!shown && shown.key !== currentKey
  const epsSelected = shown?.criterion.kind === 'front' && shown.criterion.w === null && !stale
  const canRun = req.survey_area && req.bases.length > 0 && req.drones.length > 0 && !hasIdErrors && !computing
  const setReqField = <K extends keyof PlanRequest['requirements']>(k: K, v: PlanRequest['requirements'][K]) =>
    setReq((r) => ({ ...r, requirements: { ...r.requirements, [k]: v } }))

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
          <select defaultValue="" onChange={(e) => loadScenario(e.target.value)}>
            <option value="">— загрузить тестовый сценарий —</option>
            {scenarios.map((s) => (
              <option key={s}>{s}</option>
            ))}
          </select>
        </section>

        <section>
          <h2>Геометрия</h2>
          <div className="buttons">
            {DRAW_BUTTONS.map((b) => (
              <button
                key={b.mode}
                className={drawMode === b.mode ? 'active' : ''}
                onClick={() => setDrawMode(drawMode === b.mode ? null : b.mode)}
              >
                {b.label}
              </button>
            ))}
          </div>
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
            <button className="link" onClick={() => { setReq(EMPTY); setPlan(null); setFront(null); setShown(null); setError(null) }}>
              очистить всё
            </button>
          </div>
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
          {req.survey_type !== 'geophysics' && (
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
            Откуда дует, °
            <input type="number" min="0" max="359" value={req.wind.from_deg}
              onChange={(e) => setReq({ ...req, wind: { ...req.wind, from_deg: +e.target.value } })} />
          </label>
          <label>
            Резерв заряда, %
            <input type="number" min="0" max="60" value={Math.round(req.reserve * 100)}
              onChange={(e) => setReq({ ...req, reserve: +e.target.value / 100 })} />
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
          {error && (
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
                    <td>{d.drone_id}<small>{d.payload} · {d.area_km2.toFixed(2)} км²</small></td>
                    <td>{d.params.altitude_agl_m.toFixed(0)}</td>
                    <td>{d.sorties.length}</td>
                    <td>{min(d.flight_time_s)}</td>
                    <td>{min(d.finish_s)}</td>
                    <td>{min(Math.max(...d.sorties.map((s) => s.max_divert_s)))}</td>
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
        <MapView req={req} plan={plan} stale={stale} drawMode={drawMode} onDrawn={onDrawn} fitKey={fitKey} />
        {plan && stale && <div className="map-stale">Маршруты на карте устарели — пересчитайте план</div>}
        {drawMode && (
          <div className="draw-hint">
            Рисование: {DRAW_BUTTONS.find((b) => b.mode === drawMode)?.label}. Esc — отмена.
          </div>
        )}
      </main>
    </div>
  )
}
