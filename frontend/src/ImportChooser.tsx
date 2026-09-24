import { useState } from 'react'
import type { KmlImport } from './api'
import {
  areaFromFeatures,
  countVertices,
  MAX_VERTICES,
  type AirspaceChoice,
  type AirspaceImport,
  type ImportChoice,
  type ImportedGeometry,
  type PointTarget,
  type PolygonTarget,
} from './logic'

/** Что бэкенд распознал в KML заказчика — для заголовка окна выбора. */
const KML_KINDS: Record<KmlImport['kind'], string> = {
  zones: 'зоны ограничений',
  obstacles: 'высотные препятствия',
  task: 'задание на съёмку',
}

interface Props {
  file: string
  data: ImportedGeometry
  initial: ImportChoice
  /** что уже задано — чтобы предупредить о замене */
  has: { survey: boolean; allowed: boolean }
  onApply: (choice: ImportChoice) => void
  onCancel: () => void
}

/** Выбор, куда назначить геометрию из импортированного файла: полигоны и точки — отдельно. */
export default function ImportChooser({ file, data, initial, has, onApply, onCancel }: Props) {
  const [polygons, setPolygons] = useState<PolygonTarget | ''>(initial.polygons ?? '')
  const [points, setPoints] = useState<PointTarget | ''>(initial.points ?? '')
  const nPoly = data.polygons.length
  const nPts = data.points.length
  const many = nPoly > 1
  const nothing = !(nPoly && polygons) && !(nPts && points)
  const replaces =
    nPoly && polygons === 'survey' && has.survey
      ? 'Текущая область съёмки будет заменена.'
      : nPoly && polygons === 'allowed' && has.allowed
        ? 'Текущая разрешённая зона будет заменена.'
        : null
  return (
    <div className="import-box" role="dialog" aria-label="Импорт геометрии">
      <p className="import-title">
        <b>{file}</b>: полигонов — {nPoly}, точек — {nPts}
      </p>
      {nPoly > 0 && (
        <label>
          {many ? `Полигоны (${nPoly})` : 'Полигон'}
          <select value={polygons} onChange={(e) => setPolygons(e.target.value as PolygonTarget | '')}>
            <option value="survey">{many ? 'Область съёмки (все как мультиполигон)' : 'Область съёмки'}</option>
            <option value="allowed">{many ? 'Разрешённая зона (первый полигон)' : 'Разрешённая зона'}</option>
            <option value="nfz">{many ? 'Запретные зоны (все)' : 'Запретная зона'}</option>
            <option value="">не импортировать</option>
          </select>
        </label>
      )}
      {nPts > 0 && (
        <label>
          {nPts > 1 ? `Точки (${nPts})` : 'Точка'}
          <select value={points} onChange={(e) => setPoints(e.target.value as PointTarget | '')}>
            <option value="base">ВПП</option>
            <option value="reserve">{nPts > 1 ? 'Резервные площадки' : 'Резервная площадка'}</option>
            <option value="">не импортировать</option>
          </select>
        </label>
      )}
      {data.warnings.length > 0 && (
        <ul className="warnings">
          {data.warnings.map((w, i) => (
            <li key={i}>{w}</li>
          ))}
        </ul>
      )}
      {replaces && <p className="hint">{replaces}</p>}
      <div className="buttons">
        <button
          className="active"
          disabled={nothing}
          onClick={() => onApply({ polygons: polygons || null, points: points || null })}
        >
          Добавить на карту
        </button>
        <button onClick={onCancel}>Отмена</button>
      </div>
    </div>
  )
}

/** Данные заказчика: что взять из выгрузки или из разобранного бэкендом KML.
 *  data — то, что попадёт в запрос (после отбора вблизи области), total — что было в файле. */
export function AirspaceChooser({
  source,
  kind,
  data,
  total,
  hasArea,
  nearOnly,
  marginKm,
  onNearOnly,
  onMarginKm,
  onApply,
  onCancel,
}: {
  source: string
  kind?: KmlImport['kind'] | ''
  data: AirspaceImport
  total: AirspaceImport
  hasArea: boolean
  nearOnly: boolean
  marginKm: number
  onNearOnly: (v: boolean) => void
  onMarginKm: (v: number) => void
  onApply: (choice: AirspaceChoice) => void
  onCancel: () => void
}) {
  const [restrictions, setRestrictions] = useState(true)
  const [obstacles, setObstacles] = useState(true)
  // по умолчанию берём все полигоны задания, но только если объединение проходит лимит вершин
  const [area, setArea] = useState<string>(() => {
    if (!data.areas.length) return 'none'
    if (data.areas.length === 1) return '0'
    return countVertices(areaFromFeatures(data.areas, 'union')) <= MAX_VERTICES ? 'union' : '0'
  })
  const nZones = data.restrictions.length
  const nObs = data.obstacles.length
  const nAreas = data.areas.length
  const pick: AirspaceChoice['area'] = area === 'none' || area === 'union' ? area : Number(area)
  const areaGeom = pick === 'none' ? null : areaFromFeatures(data.areas, pick)
  const vertices = areaGeom ? countVertices(areaGeom) : 0
  const nothing = !(nZones && restrictions) && !(nObs && obstacles) && !areaGeom
  const count = (n: number, all: number) => (n === all ? `${n}` : `${n} из ${all}`)
  return (
    <div className="import-box" role="dialog" aria-label="Данные о воздушном пространстве">
      <p className="import-title">
        <b>{source}</b>
        {kind ? ` · ${KML_KINDS[kind]}` : ''}: зон — {count(nZones, total.restrictions.length)}, препятствий —{' '}
        {count(nObs, total.obstacles.length)}, полигонов задания — {count(nAreas, total.areas.length)}
      </p>
      {hasArea && (
        <>
          <label>
            <span>
              <input type="checkbox" checked={nearOnly} onChange={(e) => onNearOnly(e.target.checked)} /> только вблизи
              области
            </span>
            <input
              type="number"
              min="0"
              max="200"
              step="1"
              disabled={!nearOnly}
              value={marginKm}
              onChange={(e) => onMarginKm(Math.max(0, +e.target.value))}
              title="запас вокруг области съёмки, км"
            />
          </label>
          <p className="hint">Отбор по охвату области съёмки, расширенному на указанное число километров.</p>
        </>
      )}
      {nZones > 0 && (
        <label>
          <span>Зоны ограничений ({nZones})</span>
          <input type="checkbox" checked={restrictions} onChange={(e) => setRestrictions(e.target.checked)} />
        </label>
      )}
      {nObs > 0 && (
        <label>
          <span>Высотные препятствия ({nObs})</span>
          <input type="checkbox" checked={obstacles} onChange={(e) => setObstacles(e.target.checked)} />
        </label>
      )}
      {nAreas > 0 && (
        <label>
          Полигоны задания
          <select value={area} onChange={(e) => setArea(e.target.value)}>
            <option value="none">не менять область съёмки</option>
            {nAreas > 1 && <option value="union">все {nAreas} как мультиполигон</option>}
            {data.areas.slice(0, 200).map((f, i) => (
              <option key={i} value={i}>
                {f.properties?.name || f.properties?.id || `полигон ${i + 1}`}
              </option>
            ))}
          </select>
        </label>
      )}
      {nAreas > 200 && <p className="hint">В списке показаны первые 200 полигонов задания.</p>}
      {areaGeom && (
        <p className={vertices > MAX_VERTICES ? 'field-error' : 'hint'}>
          Вершин в области съёмки: {vertices}
          {vertices > MAX_VERTICES
            ? ` — больше лимита ${MAX_VERTICES}: выберите один полигон или «не менять область съёмки»`
            : ''}
          {hasArea ? '. Текущая область съёмки будет заменена.' : ''}
        </p>
      )}
      {!nZones && !nObs && !nAreas && (
        <p className="hint">Вблизи области съёмки объектов не нашлось — увеличьте запас или снимите отбор.</p>
      )}
      <div className="buttons">
        <button
          className="active"
          disabled={nothing || vertices > MAX_VERTICES}
          onClick={() => onApply({ restrictions, obstacles, area: pick })}
        >
          Добавить в запрос
        </button>
        <button onClick={onCancel}>Отмена</button>
      </div>
    </div>
  )
}
