import type { PlanResponse } from './api'

const fmtMin = (s: number) => `${Math.round(s / 60)}`

/** Фронт Парето: время работ (x) против суммарного налёта (y). Клик по точке выбирает план. */
export function ParetoChart({
  front,
  selected,
  onSelect,
}: {
  front: PlanResponse[]
  selected: string | null
  onSelect: (p: PlanResponse) => void
}) {
  const W = 360
  const H = 200
  const P = { l: 44, r: 12, t: 12, b: 34 }
  const xs = front.map((p) => p.summary.makespan_s)
  const ys = front.map((p) => p.summary.total_flight_s)
  const pad = (a: number, b: number) => (b - a || a || 1) * 0.12
  const x0 = Math.min(...xs) - pad(Math.min(...xs), Math.max(...xs))
  const x1 = Math.max(...xs) + pad(Math.min(...xs), Math.max(...xs))
  const y0 = Math.min(...ys) - pad(Math.min(...ys), Math.max(...ys))
  const y1 = Math.max(...ys) + pad(Math.min(...ys), Math.max(...ys))
  const sx = (v: number) => P.l + ((v - x0) / (x1 - x0)) * (W - P.l - P.r)
  const sy = (v: number) => H - P.b - ((v - y0) / (y1 - y0)) * (H - P.t - P.b)
  const sorted = [...front].sort((a, b) => a.summary.makespan_s - b.summary.makespan_s)
  const ticks = (a: number, b: number) => [a, (a + b) / 2, b]
  return (
    <svg width={W} height={H} className="chart" role="img" aria-label="Фронт Парето">
      <line x1={P.l} y1={H - P.b} x2={W - P.r} y2={H - P.b} className="axis" />
      <line x1={P.l} y1={P.t} x2={P.l} y2={H - P.b} className="axis" />
      {ticks(x0, x1).map((v) => (
        <text key={`x${v}`} x={sx(v)} y={H - P.b + 14} textAnchor="middle" className="tick">
          {fmtMin(v)}
        </text>
      ))}
      {ticks(y0, y1).map((v) => (
        <text key={`y${v}`} x={P.l - 6} y={sy(v) + 4} textAnchor="end" className="tick">
          {fmtMin(v)}
        </text>
      ))}
      <text x={(W + P.l) / 2} y={H - 4} textAnchor="middle" className="label">
        время выполнения работ, мин
      </text>
      <text x={12} y={(H - P.b) / 2} textAnchor="middle" className="label" transform={`rotate(-90 12 ${(H - P.b) / 2})`}>
        налёт, мин
      </text>
      <polyline
        points={sorted.map((p) => `${sx(p.summary.makespan_s)},${sy(p.summary.total_flight_s)}`).join(' ')}
        className="front"
      />
      {sorted.map((p) => (
        <g key={p.plan_id} onClick={() => onSelect(p)} style={{ cursor: 'pointer' }}>
          <circle
            cx={sx(p.summary.makespan_s)}
            cy={sy(p.summary.total_flight_s)}
            r={p.plan_id === selected ? 7 : 5}
            className={p.plan_id === selected ? 'pt selected' : 'pt'}
          />
          <title>
            {`вес «время» ${p.time_weight}: работы ${fmtMin(p.summary.makespan_s)} мин, налёт ${fmtMin(p.summary.total_flight_s)} мин, бортов ${p.summary.drones_used}`}
          </title>
        </g>
      ))}
    </svg>
  )
}

/** Диаграмма Ганта: вылеты каждого борта во времени, между ними — смена АКБ. */
export function Gantt({ plan, colors }: { plan: PlanResponse; colors: Record<string, string> }) {
  const W = 360
  const row = 20
  const left = 70
  const H = plan.drones.length * row + 24
  const tmax = plan.summary.makespan_s || 1
  const sx = (t: number) => left + (t / tmax) * (W - left - 8)
  const step = tmax > 4 * 3600 ? 3600 : tmax > 3600 ? 1800 : 600
  const marks: number[] = []
  for (let t = 0; t <= tmax; t += step) marks.push(t)
  return (
    <svg width={W} height={H} className="chart" role="img" aria-label="График вылетов">
      {marks.map((t) => (
        <g key={t}>
          <line x1={sx(t)} y1={0} x2={sx(t)} y2={H - 16} className="grid" />
          <text x={sx(t)} y={H - 4} textAnchor="middle" className="tick">
            {Math.round(t / 60)}
          </text>
        </g>
      ))}
      {plan.drones.map((d, i) => (
        <g key={d.drone_id}>
          <text x={4} y={i * row + 14} className="tick">
            {d.drone_id}
          </text>
          {d.sorties.map((s) => (
            <rect
              key={s.index}
              x={sx(s.start_s)}
              y={i * row + 4}
              width={Math.max(1, sx(s.start_s + s.duration_s) - sx(s.start_s))}
              height={row - 8}
              rx={2}
              fill={colors[d.drone_id]}
            >
              <title>{`${d.drone_id}, вылет ${s.index + 1}: ${fmtMin(s.start_s)}–${fmtMin(s.start_s + s.duration_s)} мин, база ${s.base_id}`}</title>
            </rect>
          ))}
        </g>
      ))}
    </svg>
  )
}
