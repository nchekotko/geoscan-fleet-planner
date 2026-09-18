import { useState } from 'react'
import type { ImportChoice, ImportedGeometry, PointTarget, PolygonTarget } from './logic'

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
