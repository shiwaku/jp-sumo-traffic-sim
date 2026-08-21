import type { ExpressionSpecification, LayerSpecification } from 'maplibre-gl'

/**
 * データレイヤーの定義。パネルの並び順（先頭＝一番上）。
 * main.ts が配列順に addLayer するため、**配列末尾ほど地図で最前面**。
 * 面（コードン）を背面に、点（信号）を前面に置く。
 * データは scripts/05_export_viewer.py が viewer/public/data/ に書き出す GeoJSON。
 */
export interface LayerDef {
  key: string
  name: string
  /** public/data/ 内のファイル名（拡張子なし） */
  file: string
  on: boolean
  defaultOpacity: number
  opacity?: number
  desc: string
}

export const LAYERS: LayerDef[] = [
  {
    key: 'cordon',
    name: 'コードン（対象区域）',
    file: 'cordon',
    on: false,
    defaultOpacity: 1,
    desc:
      '北5条通・南7条通・創成川通・石山通に囲まれた対象区域(1520×1740m)。' +
      '札幌の碁盤目に合わせて10.906度回転した矩形。破線は解析用のクリップ範囲(+500m)。',
  },
  {
    key: 'lattice',
    name: '130m格子線',
    file: 'lattice',
    on: false,
    defaultOpacity: 0.6,
    desc:
      '条・丁目の理論格子線。実測で東西・南北とも130.0m等間隔であることを確認済み。' +
      '街路の同定はこの格子への直接割り当てで行っている。',
  },
  {
    key: 'network',
    name: '道路ネットワーク（KSJ）',
    file: 'network',
    on: false,
    defaultOpacity: 0.9,
    desc:
      '国土数値情報 道路データ(N13 2024年度版)を区域+バッファでクリップした1,034リンク。' +
      '青=格子街路 / 橙=街区中央の裏通り / 灰=斜行・格子外。太線は区域を横断する通し街路。',
  },
  {
    key: 'census',
    name: 'センサス区間',
    file: 'census',
    on: false,
    defaultOpacity: 0.5,
    desc:
      '道路交通センサス(令和3年度)の交通調査基本区間。区域内17区間。' +
      '車線数・車道部幅員・旅行速度・大型車混入率・12/24時間交通量を持つ。' +
      '国道・都道府県道・主要幹線市道のみで、区域内の延長被覆は18%。',
  },
  {
    key: 'reg-line',
    name: '規制（一方通行・最高速度）',
    file: 'reg_line',
    on: false,
    defaultOpacity: 0.9,
    desc:
      'JARTIC 交通規制情報の線規制。橙=一方通行(通行方向は頂点順の逆。OSM照合で確定) / ' +
      '紫=最高速度 / 茶=その他の線規制。',
  },
  {
    key: 'reg-point',
    name: '規制（一時停止・信号機）',
    file: 'reg_point',
    on: false,
    defaultOpacity: 0.85,
    desc:
      'JARTIC 交通規制情報の点規制。赤=一時停止 / 緑=信号機 / 灰=その他。' +
      '北海道は停止線・指定方向外進行禁止を提供していないため、それらは含まれない。',
  },
  {
    key: 'signals',
    name: '交差点制御情報（サイクル長）',
    file: 'signals',
    on: true,
    defaultOpacity: 1,
    desc:
      'JARTIC 交差点制御情報(情報源コード3001=札幌方面、231交差点)。' +
      '色は日中最大サイクル長。区域内は20交差点のみで、系統制御対象の幹線に限られる。' +
      '定義CSV由来の現示数・流入リンク数も持つ。',
  },
]

export function opacityOf(def: LayerDef): number {
  return def.opacity ?? def.defaultOpacity
}

// ---- 配色 ----

export const KIND_COLORS: Record<string, string> = {
  格子街路: '#3d7ab8',
  裏通り: '#d98c3f',
}

/** 平均サイクル長の段階色（signal-cycle-converter の配色を踏襲） */
export const CYCLE_STOPS = [
  { min: 0, color: 'rgba(0, 127, 255, 1)', label: '100秒未満' },
  { min: 100, color: 'rgba(0, 255, 255, 1)', label: '100–110秒' },
  { min: 110, color: 'rgba(0, 255, 127, 1)', label: '110–120秒' },
  { min: 120, color: 'rgba(0, 255, 0, 1)', label: '120–130秒' },
  { min: 130, color: 'rgba(255, 255, 0, 1)', label: '130–140秒' },
  { min: 140, color: 'rgba(255, 127, 0, 1)', label: '140–150秒' },
  { min: 150, color: 'rgba(255, 0, 0, 1)', label: '150秒以上' },
]

function cycleColor(): ExpressionSpecification {
  const rest: (number | string)[] = []
  for (const s of CYCLE_STOPS.slice(1)) rest.push(s.min, s.color)
  return ['step', ['to-number', ['get', 'cycle_max_s'], 0], CYCLE_STOPS[0].color, ...rest] as unknown as ExpressionSpecification
}

const zoomWidth = (thin: number, thick: number): ExpressionSpecification =>
  ['interpolate', ['linear'], ['zoom'], 12, thin, 16, thick, 18, thick * 2] as unknown as ExpressionSpecification

// ---- レイヤー仕様 ----

export function layerIdsOf(def: LayerDef): string[] {
  switch (def.key) {
    case 'cordon':
      return ['cordon-fill', 'cordon-line', 'clip-line']
    case 'network':
      return ['network-lyr', 'network-grade']
    case 'reg-line':
      return ['regline-oneway', 'regline-speed', 'regline-other']
    case 'reg-point':
      return ['regpoint-stop', 'regpoint-signal', 'regpoint-other']
    case 'signals':
      return ['signals-glow', 'signals-core']
    default:
      return [`${def.key}-lyr`]
  }
}

/** クリック・ホバー判定に使うレイヤー ID。 */
export function pickLayerIdsOf(def: LayerDef): string[] {
  switch (def.key) {
    case 'cordon':
      return []
    case 'network':
      return ['network-lyr']
    case 'reg-line':
      return ['regline-oneway', 'regline-speed', 'regline-other']
    case 'reg-point':
      return ['regpoint-stop', 'regpoint-signal', 'regpoint-other']
    case 'signals':
      return ['signals-glow']
    default:
      return [`${def.key}-lyr`]
  }
}

export function layerSpecs(def: LayerDef): LayerSpecification[] {
  const v = opacityOf(def)
  switch (def.key) {
    case 'cordon':
      return [
        {
          id: 'cordon-fill', type: 'fill', source: def.key,
          filter: ['==', ['get', 'kind'], 'cordon'],
          paint: { 'fill-color': '#2f8f6b', 'fill-opacity': 0.05 * v },
        },
        {
          id: 'cordon-line', type: 'line', source: def.key,
          filter: ['==', ['get', 'kind'], 'cordon'],
          paint: { 'line-color': '#2f8f6b', 'line-width': 2.2, 'line-opacity': v },
        },
        {
          id: 'clip-line', type: 'line', source: def.key,
          filter: ['==', ['get', 'kind'], 'clip'],
          paint: {
            'line-color': '#8b9199', 'line-width': 1,
            'line-dasharray': [3, 3], 'line-opacity': 0.7 * v,
          },
        },
      ]
    case 'lattice':
      return [
        {
          id: 'lattice-lyr', type: 'line', source: def.key,
          paint: {
            'line-color': '#3d7ab8', 'line-width': 0.8,
            'line-dasharray': [2, 4], 'line-opacity': v,
          },
        },
      ]
    case 'network':
      return [
        {
          id: 'network-lyr', type: 'line', source: def.key,
          layout: { 'line-cap': 'round' },
          paint: {
            'line-color': [
              'match', ['get', 'street_kind'],
              '格子街路', KIND_COLORS['格子街路'],
              '裏通り', KIND_COLORS['裏通り'],
              '#9aa2ab',
            ],
            'line-width': [
              'interpolate', ['linear'], ['zoom'],
              12, ['case', ['to-boolean', ['get', 'through']], 1.6, 0.7],
              16, ['case', ['to-boolean', ['get', 'through']], 4, 1.8],
              18, ['case', ['to-boolean', ['get', 'through']], 8, 3.6],
            ],
            'line-opacity': v,
          },
        },
        {
          // トンネル・橋を紫で強調（創成トンネルの坑口確認用）
          id: 'network-grade', type: 'line', source: def.key,
          filter: ['in', ['get', 'road_state'], ['literal', ['トンネル', '橋・高架']]],
          paint: { 'line-color': '#7b52c4', 'line-width': zoomWidth(2.4, 6), 'line-opacity': v },
        },
      ]
    case 'census':
      return [
        {
          id: 'census-lyr', type: 'line', source: def.key,
          paint: { 'line-color': '#b8860b', 'line-width': zoomWidth(3, 9), 'line-opacity': v },
        },
      ]
    case 'reg-line':
      return [
        {
          id: 'regline-other', type: 'line', source: def.key,
          filter: ['!', ['in', ['get', 'code'], ['literal', ['11', '112', '114']]]],
          paint: { 'line-color': '#8a6d3b', 'line-width': zoomWidth(1, 2.6), 'line-opacity': 0.6 * v },
        },
        {
          id: 'regline-speed', type: 'line', source: def.key,
          filter: ['in', ['get', 'code'], ['literal', ['112', '114']]],
          paint: { 'line-color': '#7b52c4', 'line-width': zoomWidth(1.4, 3.4), 'line-opacity': 0.8 * v },
        },
        {
          id: 'regline-oneway', type: 'line', source: def.key,
          filter: ['==', ['get', 'code'], '11'],
          layout: { 'line-cap': 'round' },
          paint: { 'line-color': '#c1571f', 'line-width': zoomWidth(1.8, 4.4), 'line-opacity': v },
        },
      ]
    case 'reg-point':
      return [
        {
          id: 'regpoint-other', type: 'circle', source: def.key,
          filter: ['!', ['in', ['get', 'code'], ['literal', ['63', '98']]]],
          paint: {
            'circle-color': '#9aa2ab', 'circle-radius': zoomWidth(1.6, 3.6),
            'circle-opacity': 0.55 * v,
          },
        },
        {
          id: 'regpoint-signal', type: 'circle', source: def.key,
          filter: ['==', ['get', 'code'], '98'],
          paint: {
            'circle-color': '#3d8b40', 'circle-radius': zoomWidth(2, 4.6),
            'circle-opacity': 0.85 * v,
            'circle-stroke-width': 0.6, 'circle-stroke-color': '#fff',
          },
        },
        {
          id: 'regpoint-stop', type: 'circle', source: def.key,
          filter: ['==', ['get', 'code'], '63'],
          paint: {
            'circle-color': '#d64545', 'circle-radius': zoomWidth(2, 4.6),
            'circle-opacity': 0.9 * v,
            'circle-stroke-width': 0.6, 'circle-stroke-color': '#fff',
          },
        },
      ]
    case 'signals':
      return [
        {
          id: 'signals-glow', type: 'circle', source: def.key,
          paint: {
            'circle-color': cycleColor(), 'circle-radius': zoomWidth(9, 15),
            'circle-blur': 1.2, 'circle-opacity': 0.65 * v,
          },
        },
        {
          id: 'signals-core', type: 'circle', source: def.key,
          paint: {
            'circle-color': cycleColor(), 'circle-radius': zoomWidth(3, 5.5),
            'circle-opacity': v,
            'circle-stroke-width': 1.2, 'circle-stroke-color': '#fff',
          },
        },
      ]
    default:
      return []
  }
}

/** 不透明度スライダーの反映。 */
export function opacityUpdates(def: LayerDef, v: number): { id: string; prop: string; value: number }[] {
  const propOf = (id: string): string =>
    id.includes('point') || id.startsWith('signals') ? 'circle-opacity'
    : id === 'cordon-fill' ? 'fill-opacity' : 'line-opacity'
  const factor: Record<string, number> = {
    'cordon-fill': 0.05, 'clip-line': 0.7,
    'regline-other': 0.6, 'regline-speed': 0.8,
    'regpoint-other': 0.55, 'regpoint-signal': 0.85, 'regpoint-stop': 0.9,
    'signals-glow': 0.65,
  }
  return layerIdsOf(def).map((id) => ({ id, prop: propOf(id), value: (factor[id] ?? 1) * v }))
}

// ---- 凡例 ----

export interface LegendItem {
  color: string
  label: string
  shape: 'circle' | 'square' | 'line'
}

export function legendFor(def: LayerDef): LegendItem[] {
  switch (def.key) {
    case 'network':
      return [
        { color: KIND_COLORS['格子街路'], label: '格子街路（条通・丁目通）', shape: 'line' },
        { color: KIND_COLORS['裏通り'], label: '街区中央の裏通り', shape: 'line' },
        { color: '#9aa2ab', label: '斜行・格子外', shape: 'line' },
        { color: '#7b52c4', label: 'トンネル・橋', shape: 'line' },
      ]
    case 'reg-line':
      return [
        { color: '#c1571f', label: '一方通行', shape: 'line' },
        { color: '#7b52c4', label: '最高速度', shape: 'line' },
        { color: '#8a6d3b', label: 'その他の線規制', shape: 'line' },
      ]
    case 'reg-point':
      return [
        { color: '#d64545', label: '一時停止', shape: 'circle' },
        { color: '#3d8b40', label: '信号機', shape: 'circle' },
        { color: '#9aa2ab', label: 'その他の点規制', shape: 'circle' },
      ]
    case 'signals':
      return CYCLE_STOPS.map((s) => ({ color: s.color, label: s.label, shape: 'circle' as const }))
    case 'census':
      return [{ color: '#b8860b', label: '交通調査基本区間', shape: 'line' }]
    case 'lattice':
      return [{ color: '#3d7ab8', label: '条・丁目の理論格子', shape: 'line' }]
    case 'cordon':
      return [
        { color: '#2f8f6b', label: 'コードン', shape: 'line' },
        { color: '#8b9199', label: 'クリップ範囲(+500m)', shape: 'line' },
      ]
    default:
      return []
  }
}

// ---- ポップアップ ----

function esc(s: string): string {
  return s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c] as string)
}

const S = (p: Record<string, unknown>, k: string): string => {
  const v = p[k]
  return v === undefined || v === null || v === '' ? '' : String(v)
}

function row(label: string, value: string, strong = false): string {
  if (!value) return ''
  return `<dt>${esc(label)}</dt><dd${strong ? ' class="pp-strong"' : ''}>${esc(value)}</dd>`
}

function footHtml(lng: number, lat: number): string {
  const q = `${lat},${lng}`
  return (
    `<div class="pp-foot">座標: ${lat.toFixed(7)}, ${lng.toFixed(7)}（クリック位置）<br />` +
    `<a href="https://www.google.com/maps?q=${q}&hl=ja" target="_blank" rel="noopener">🌎 Google Maps</a> ` +
    `<a href="https://www.google.com/maps/@?api=1&map_action=pano&viewpoint=${q}&hl=ja" target="_blank" rel="noopener">📷 Street View</a></div>`
  )
}

export function popupHtml(def: LayerDef, p: Record<string, unknown>, lng: number, lat: number): string {
  let title = def.name
  let rows = ''
  switch (def.key) {
    case 'network': {
      title = S(p, 'street') || '（街路未割り当て）'
      rows =
        row('link_id', S(p, 'link_id')) +
        row('種別', S(p, 'street_kind') + (p.through ? ' / 通し街路' : '')) +
        row('方向', S(p, 'axis')) +
        row('長さ', S(p, 'length_m') ? `${S(p, 'length_m')} m` : '') +
        row('道路分類', S(p, 'road_category')) +
        row('幅員区分', S(p, 'width_class')) +
        row('道路状態', S(p, 'road_state')) +
        row('格子誤差', S(p, 'assign_err_m') ? `${S(p, 'assign_err_m')} m` : '')
      break
    }
    case 'signals': {
      title = `信号交差点 ${S(p, 'jartic_id')}`
      rows =
        row('サイクル長', `${S(p, 'cycle_min_s')}–${S(p, 'cycle_max_s')} 秒`, true) +
        row('現示数', S(p, 'n_phases')) +
        row('流入リンク', S(p, 'n_in_links') ? `${S(p, 'n_in_links')} 本` : '') +
        row('流出リンク', S(p, 'n_out_links') ? `${S(p, 'n_out_links')} 本` : '') +
        row('時間帯区分', S(p, 'n_hours')) +
        row('区域内', p.in_cordon ? 'はい' : 'いいえ')
      break
    }
    case 'census': {
      title = S(p, 'route') || 'センサス区間'
      rows =
        row('区間番号', S(p, 'section_id')) +
        row('車線数', S(p, 'n_lanes')) +
        row('車道部幅員', S(p, 'w_carriageway') ? `${S(p, 'w_carriageway')} m` : '') +
        row('指定最高速度', S(p, 'speed_limit') ? `${S(p, 'speed_limit')} km/h` : '') +
        row('混雑時旅行速度', S(p, 'v_peak_up') ? `${S(p, 'v_peak_up')} km/h` : '', true) +
        row('非混雑時旅行速度', S(p, 'v_off_up') ? `${S(p, 'v_off_up')} km/h` : '') +
        row('大型車混入率', S(p, 'heavy_pct') ? `${S(p, 'heavy_pct')} %` : '') +
        row('混雑度', S(p, 'congestion')) +
        row('12時間交通量', S(p, 'v12h') ? `${S(p, 'v12h')} 台` : '')
      break
    }
    case 'reg-line':
    case 'reg-point': {
      title = S(p, 'kind') || '交通規制'
      rows =
        row('共通規制種別コード', S(p, 'code')) +
        row('交差点名称', S(p, 'crossing')) +
        row('路線名', S(p, 'route')) +
        row('速度', S(p, 'speed')) +
        row('通行帯数', S(p, 'n_lanes')) +
        row('延長', S(p, 'length') ? `${S(p, 'length')} m` : '') +
        (S(p, 'code') === '11' ? row('向き', '頂点順の逆（OSM照合で確定・15_oneway_check）') : '')
      break
    }
    case 'lattice': {
      title = S(p, 'name') || '格子線'
      break
    }
  }
  return (
    `<div class="pp-title">${esc(title)}</div>` +
    `<div class="pp-sub">${esc(def.name)}</div>` +
    (rows ? `<dl class="pp-dl">${rows}</dl>` : '') +
    footHtml(lng, lat)
  )
}
