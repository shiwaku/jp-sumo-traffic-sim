import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'

import { getBasemapStyle, type Basemap } from './basemap'
import {
  LAYERS,
  type LayerDef,
  layerIdsOf,
  layerSpecs,
  legendFor,
  opacityOf,
  opacityUpdates,
  pickLayerIdsOf,
  popupHtml,
} from './layers'
import { SimPlayer, initSimUI } from './sim'
import { applyThemeAttr, initialTheme, type Theme } from './theme'
import './style.css'

const DATA_BASE = 'data'
const SITE_ATTRIBUTION =
  '（<a href="https://github.com/shiwaku/sapporo-micro-traffic-sim" target="_blank" rel="noopener">GitHub</a>）'

interface Meta {
  center: [number, number]
  bearing: number
  n_links: number
  n_nodes: number
  n_junctions: number
  n_streets: number
  n_through: number
  cordon_size: [number, number]
  grid_bearing: number
  signals?: { n_in_cordon?: number; cycle?: { min?: number; median?: number; max?: number } }
  regulations?: { n_in_cordon?: number; target_month?: string; not_provided?: string[] }
  census?: { n_in_cordon?: number; coverage?: number }
}

let theme: Theme = initialTheme()
let base: Basemap = 'pale'
applyThemeAttr(theme)

const isMobile = window.matchMedia('(max-width: 640px)').matches

const map = new maplibregl.Map({
  container: 'map',
  style: getBasemapStyle(base, theme),
  center: [141.3501, 43.0591],
  zoom: 14.2,
  bearing: -10.906, // グリッドを画面の縦横に合わせる
  hash: true,
  attributionControl: false,
  maxTileCacheSize: isMobile ? 24 : undefined,
})
map.addControl(new maplibregl.NavigationControl({ showCompass: true, visualizePitch: true }), 'top-right')
map.addControl(new maplibregl.FullscreenControl(), 'top-right')
map.addControl(new maplibregl.ScaleControl({ maxWidth: 200, unit: 'metric' }), 'bottom-left')
map.addControl(new maplibregl.AttributionControl({ compact: true, customAttribution: SITE_ATTRIBUTION }))

// ---- クリックハイライト ----
const HL_SRC = 'click-highlight'
const HL_LINE = 'click-highlight-line'
const HL_CIRCLE = 'click-highlight-circle'
const EMPTY_FC: GeoJSON.FeatureCollection = { type: 'FeatureCollection', features: [] }

function ensureHighlightLayers(): void {
  if (!map.getSource(HL_SRC)) map.addSource(HL_SRC, { type: 'geojson', data: EMPTY_FC })
  if (!map.getLayer(HL_LINE)) {
    map.addLayer({
      id: HL_LINE, type: 'line', source: HL_SRC,
      filter: ['!=', ['geometry-type'], 'Point'],
      paint: { 'line-color': 'rgba(255,200,0,1)', 'line-width': 4 },
    })
  }
  if (!map.getLayer(HL_CIRCLE)) {
    map.addLayer({
      id: HL_CIRCLE, type: 'circle', source: HL_SRC,
      filter: ['==', ['geometry-type'], 'Point'],
      paint: {
        'circle-radius': 11, 'circle-color': 'rgba(0,0,0,0)',
        'circle-stroke-color': 'rgba(255,220,0,1)', 'circle-stroke-width': 3,
      },
    })
  }
}

function setHighlight(f: maplibregl.MapGeoJSONFeature | null): void {
  const src = map.getSource(HL_SRC) as maplibregl.GeoJSONSource | undefined
  if (!src) return
  src.setData(
    f ? { type: 'FeatureCollection', features: [{ type: 'Feature', geometry: f.geometry, properties: {} }] } : EMPTY_FC,
  )
}

// ---- データレイヤー ----
// GeoJSON はロード時に一括 fetch してキャッシュする（合計 ~1.5MB）。
const dataCache = new Map<string, GeoJSON.FeatureCollection>()

async function fetchData(def: LayerDef): Promise<GeoJSON.FeatureCollection> {
  const cached = dataCache.get(def.file)
  if (cached) return cached
  const r = await fetch(`${DATA_BASE}/${def.file}.geojson`)
  if (!r.ok) throw new Error(`${def.file}.geojson: ${r.status}`)
  const fc = (await r.json()) as GeoJSON.FeatureCollection
  dataCache.set(def.file, fc)
  return fc
}

function beforeIdFor(def: LayerDef): string | undefined {
  const i = LAYERS.findIndex((d) => d.key === def.key)
  for (let j = i + 1; j < LAYERS.length; j++) {
    for (const id of layerIdsOf(LAYERS[j])) {
      if (map.getLayer(id)) return id
    }
  }
  return map.getLayer(HL_LINE) ? HL_LINE : undefined
}

async function ensureLayer(def: LayerDef): Promise<void> {
  if (!map.getSource(def.key)) {
    const data = await fetchData(def)
    if (map.getSource(def.key)) return
    map.addSource(def.key, { type: 'geojson', data })
  }
  const before = beforeIdFor(def)
  for (const spec of layerSpecs(def)) {
    if (map.getLayer(spec.id)) continue
    map.addLayer(spec, before)
  }
}

function removeLayer(def: LayerDef): void {
  for (const id of layerIdsOf(def)) {
    if (map.getLayer(id)) map.removeLayer(id)
  }
  if (map.getSource(def.key)) map.removeSource(def.key)
}

function addDataLayers(): void {
  ensureHighlightLayers()
  for (const def of LAYERS) {
    if (def.on) void ensureLayer(def)
    else removeLayer(def)
  }
  // シミュレーションの点群は常に最前面
  if (simPlayer.loaded) simPlayer.ensureLayers()
}

// ---- テーマ・背景 ----
const themeBtn = document.getElementById('theme-btn') as HTMLButtonElement
const renderThemeBtn = (): void => {
  themeBtn.textContent = theme === 'dark' ? '☀️' : '🌙'
}
function reloadStyle(): void {
  map.setStyle(getBasemapStyle(base, theme), { diff: false })
  map.once('idle', () => addDataLayers())
}
themeBtn.addEventListener('click', () => {
  theme = theme === 'dark' ? 'light' : 'dark'
  applyThemeAttr(theme)
  renderThemeBtn()
  reloadStyle()
})

class BasemapControl implements maplibregl.IControl {
  private el!: HTMLElement
  onAdd(): HTMLElement {
    this.el = document.createElement('div')
    this.el.className = 'maplibregl-ctrl basemap-switch'
    const defs: [Basemap, string][] = [
      ['pale', '地図'],
      ['photo', '写真'],
    ]
    for (const [b, label] of defs) {
      const btn = document.createElement('button')
      btn.type = 'button'
      btn.textContent = label
      btn.dataset.base = b
      btn.setAttribute('aria-selected', String(b === base))
      btn.addEventListener('click', () => setBase(b))
      this.el.append(btn)
    }
    return this.el
  }
  onRemove(): void {
    this.el.remove()
  }
  sync(): void {
    for (const btn of this.el.querySelectorAll<HTMLButtonElement>('button')) {
      btn.setAttribute('aria-selected', String(btn.dataset.base === base))
    }
  }
}
const basemapCtrl = new BasemapControl()
map.addControl(basemapCtrl, 'bottom-right')

function setBase(next: Basemap): void {
  if (next === base) return
  base = next
  basemapCtrl.sync()
  reloadStyle()
}

// ---- パネル ----
const panel = document.getElementById('panel') as HTMLElement
const collapseBtn = document.getElementById('collapse-btn') as HTMLButtonElement
const renderCollapseBtn = (): void => {
  collapseBtn.textContent = panel.classList.contains('collapsed') ? '▾' : '▴'
}
collapseBtn.addEventListener('click', () => {
  panel.classList.toggle('collapsed')
  renderCollapseBtn()
})

const layersDiv = document.getElementById('layers') as HTMLElement

function legendMarkup(def: LayerDef): string {
  return legendFor(def)
    .map(
      (it) =>
        `<span class="lg-row"><span class="lg-sw lg-${it.shape}" style="background:${it.color}"></span>${it.label}</span>`,
    )
    .join('')
}

function buildToggles(): void {
  for (const def of LAYERS) {
    const item = document.createElement('div')
    item.className = 'layer-item'
    item.dataset.key = def.key

    const label = document.createElement('label')
    label.className = 'toggle'

    const input = document.createElement('input')
    input.type = 'checkbox'
    input.checked = def.on
    input.addEventListener('change', () => setLayerVisible(def, input.checked))

    const sw = document.createElement('span')
    sw.className = 'switch'
    const text = document.createElement('span')
    text.className = 't-label'
    text.textContent = def.name

    const desc = document.createElement('div')
    desc.className = 'layer-desc'
    desc.hidden = true
    desc.textContent = def.desc

    const info = document.createElement('button')
    info.type = 'button'
    info.className = 'info-btn'
    info.textContent = 'i'
    info.setAttribute('aria-label', `${def.name}の説明`)
    info.setAttribute('aria-expanded', 'false')
    info.addEventListener('click', (e) => {
      e.preventDefault()
      e.stopPropagation()
      const open = desc.hidden
      desc.hidden = !open
      info.setAttribute('aria-expanded', String(open))
    })

    label.append(input, sw, text, info)

    const opac = document.createElement('div')
    opac.className = 'layer-opacity'
    opac.hidden = !def.on
    const range = document.createElement('input')
    range.type = 'range'
    range.min = '0'
    range.max = '1'
    range.step = '0.05'
    range.value = String(opacityOf(def))
    range.setAttribute('aria-label', `${def.name}の不透明度`)
    const val = document.createElement('span')
    val.className = 'op-val'
    val.textContent = `${Math.round(opacityOf(def) * 100)}%`
    range.addEventListener('input', () => {
      const v = Number(range.value)
      val.textContent = `${Math.round(v * 100)}%`
      setLayerOpacity(def, v)
    })
    opac.append(range, val)

    const legend = document.createElement('div')
    legend.className = 'layer-legend'
    legend.innerHTML = legendMarkup(def)
    legend.hidden = !def.on

    item.append(label, desc, opac, legend)
    layersDiv.append(item)
  }
}

function setLayerVisible(def: LayerDef, on: boolean): void {
  def.on = on
  if (on) void ensureLayer(def)
  else removeLayer(def)
  const item = layersDiv.querySelector<HTMLElement>(`.layer-item[data-key="${def.key}"]`)
  item?.querySelector<HTMLElement>('.layer-legend')?.toggleAttribute('hidden', !on)
  item?.querySelector<HTMLElement>('.layer-opacity')?.toggleAttribute('hidden', !on)
}

function setLayerOpacity(def: LayerDef, v: number): void {
  def.opacity = v
  for (const u of opacityUpdates(def, v)) {
    if (map.getLayer(u.id)) map.setPaintProperty(u.id, u.prop, u.value)
  }
}

function setAll(on: boolean): void {
  for (const def of LAYERS) {
    if (def.on === on) continue
    const input = layersDiv.querySelector<HTMLInputElement>(`.layer-item[data-key="${def.key}"] input[type=checkbox]`)
    if (input) input.checked = on
    setLayerVisible(def, on)
  }
}
;(document.getElementById('all-on') as HTMLButtonElement).addEventListener('click', () => setAll(true))
;(document.getElementById('all-off') as HTMLButtonElement).addEventListener('click', () => setAll(false))

// ---- 統計パネル ----
async function renderStats(): Promise<void> {
  try {
    const r = await fetch(`${DATA_BASE}/meta.json`)
    if (!r.ok) return
    const m = (await r.json()) as Meta
    const dl = document.getElementById('stats') as HTMLElement
    const rows: [string, string][] = [
      ['区域', `${m.cordon_size[0]} × ${m.cordon_size[1]} m`],
      ['グリッド方位', `${m.grid_bearing.toFixed(3)}°`],
      ['リンク / ノード', `${m.n_links} / ${m.n_nodes}`],
      ['交差点(次数3+)', String(m.n_junctions)],
      ['通し街路', String(m.n_through)],
    ]
    if (m.signals?.n_in_cordon != null) {
      rows.push(['信号(制御情報)', `${m.signals.n_in_cordon} 交差点`])
      const c = m.signals.cycle
      if (c?.min != null) rows.push(['サイクル長', `${c.min}–${c.max} 秒`])
    }
    if (m.regulations?.n_in_cordon != null) rows.push(['規制(区域内)', `${m.regulations.n_in_cordon} 件`])
    if (m.census?.n_in_cordon != null) {
      const cov = m.census.coverage != null ? ` / 被覆 ${(m.census.coverage * 100).toFixed(0)}%` : ''
      rows.push(['センサス', `${m.census.n_in_cordon} 区間${cov}`])
    }
    dl.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('')
    const sub = document.getElementById('brand-sub')
    if (sub && m.regulations?.target_month) sub.textContent = `Phase 0 / JARTIC ${m.regulations.target_month}`
  } catch {
    // meta.json 不在でも地図は動かす
  }
}

// ---- ホバー・クリック ----
const activePickIds = (): string[] =>
  LAYERS.filter((d) => d.on)
    .flatMap((d) => pickLayerIdsOf(d))
    .filter((id) => map.getLayer(id))

if (window.matchMedia('(hover: hover)').matches) {
  map.on('mousemove', (e) => {
    const ids = activePickIds()
    const hit = ids.length > 0 && map.queryRenderedFeatures(e.point, { layers: ids }).length > 0
    map.getCanvas().style.cursor = hit ? 'pointer' : ''
  })
}

let popup: maplibregl.Popup | null = null
map.on('click', (e) => {
  const ids = activePickIds()
  const feats = ids.length ? map.queryRenderedFeatures(e.point, { layers: ids }) : []
  if (!feats.length) {
    setHighlight(null)
    return
  }
  const f = feats[0]
  const def = LAYERS.find((d) => pickLayerIdsOf(d).includes(f.layer.id))
  if (!def) return
  if (popup) {
    const old = popup
    popup = null
    old.remove()
  }
  setHighlight(f)
  const p = new maplibregl.Popup({ closeButton: true, maxWidth: '320px' })
    .setLngLat(e.lngLat)
    .setHTML(popupHtml(def, f.properties as Record<string, unknown>, e.lngLat.lng, e.lngLat.lat))
    .addTo(map)
  p.on('close', () => {
    if (popup === p) {
      popup = null
      setHighlight(null)
    }
  })
  popup = p
})

// ---- シミュレーション再生 ----
const simPlayer = new SimPlayer(map)
initSimUI(simPlayer)

// ---- 初期化 ----
const buildEl = document.getElementById('build-ver')
if (buildEl) buildEl.textContent = `build: ${__BUILD_TIME__}`
renderThemeBtn()
buildToggles()
if (isMobile) panel.classList.add('collapsed')
renderCollapseBtn()
map.on('load', addDataLayers)
void renderStats()

// WebGL コンテキスト消失からの復帰（参照実装と同じ理由: iOS Safari 等の GL 消失対策）
const canvas = map.getCanvas()
canvas.addEventListener(
  'webglcontextlost',
  (e) => e.preventDefault(),
  false,
)
canvas.addEventListener(
  'webglcontextrestored',
  () => {
    if (map.isStyleLoaded()) addDataLayers()
    else map.once('idle', addDataLayers)
  },
  false,
)
