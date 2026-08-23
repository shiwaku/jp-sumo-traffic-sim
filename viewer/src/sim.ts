import type maplibregl from 'maplibre-gl'

/**
 * 簡易ミクロシミュレーションの再生。
 * scripts/20_run_simple_sim.py が書き出す data/sim_{scenario}.json を読み、
 * 1秒刻みのフレームを ID なし点群として描画する(フレーム間は非補間、
 * 再生速度で見かけの滑らかさを調整する)。
 */

interface SimFrame {
  t: number
  /** [dx, dy, v] × 台数。単位 0.1m / 0.1m/s、原点は origin */
  p: number[]
  /** ノードごとの信号状態。'1'=東西青 / '0'=南北青 / '2'=全赤 */
  sig: string
}

interface SimData {
  scenario: string
  origin: [number, number]
  frame_s: number
  nodes: [number, number][]
  frames: SimFrame[]
}

const SRC_VEH = 'sim-vehicles'
const SRC_SIG = 'sim-signals'
const LYR_VEH = 'sim-vehicles-lyr'
const LYR_SIG = 'sim-signals-lyr'
const EMPTY: GeoJSON.FeatureCollection = { type: 'FeatureCollection', features: [] }

/** 速度[m/s] → 色。渋滞=赤 → 自由流=青緑 */
const SPEED_COLOR = [
  'step', ['get', 'v'],
  '#d13434', 1.5,
  '#e07b39', 4,
  '#e0c23a', 8,
  '#59b04f', 12,
  '#2e9e8f',
] as unknown as import('maplibre-gl').ExpressionSpecification

export class SimPlayer {
  private map: maplibregl.Map
  private data: SimData | null = null
  private cache = new Map<string, SimData>()
  private frame = 0
  private playing = false
  private speed = 4
  private timer: number | null = null
  private mPerDegLon = 1
  private mPerDegLat = 110574

  constructor(map: maplibregl.Map) {
    this.map = map
  }

  /** スタイル再構築後にも呼ばれる。ソースとレイヤーを(再)作成する。 */
  ensureLayers(): void {
    const m = this.map
    if (!m.getSource(SRC_VEH)) m.addSource(SRC_VEH, { type: 'geojson', data: EMPTY })
    if (!m.getSource(SRC_SIG)) m.addSource(SRC_SIG, { type: 'geojson', data: EMPTY })
    if (!m.getLayer(LYR_SIG)) {
      m.addLayer({
        id: LYR_SIG, type: 'circle', source: SRC_SIG,
        paint: {
          'circle-radius': ['interpolate', ['linear'], ['zoom'], 12, 2, 16, 4.5],
          'circle-color': ['match', ['get', 's'], 1, '#3d8b40', 0, '#d64545', '#8a8f98'],
          'circle-opacity': 0.9,
          'circle-stroke-width': 0.6,
          'circle-stroke-color': '#fff',
        },
      })
    }
    if (!m.getLayer(LYR_VEH)) {
      m.addLayer({
        id: LYR_VEH, type: 'circle', source: SRC_VEH,
        paint: {
          'circle-radius': ['interpolate', ['linear'], ['zoom'], 12, 1.8, 15, 3.4, 17, 6],
          'circle-color': SPEED_COLOR,
          'circle-opacity': 0.95,
        },
      })
    }
    this.render()
  }

  removeLayers(): void {
    for (const id of [LYR_VEH, LYR_SIG]) if (this.map.getLayer(id)) this.map.removeLayer(id)
    for (const id of [SRC_VEH, SRC_SIG]) if (this.map.getSource(id)) this.map.removeSource(id)
  }

  get loaded(): boolean {
    return this.data !== null
  }
  get nFrames(): number {
    return this.data?.frames.length ?? 0
  }
  get currentFrame(): number {
    return this.frame
  }
  get isPlaying(): boolean {
    return this.playing
  }

  async load(scenario: string): Promise<void> {
    let d = this.cache.get(scenario)
    if (!d) {
      const r = await fetch(`data/sim_${scenario}.json`)
      if (!r.ok) throw new Error(`sim_${scenario}.json: ${r.status}`)
      d = (await r.json()) as SimData
      this.cache.set(scenario, d)
    }
    this.data = d
    const lat0 = d.origin[1]
    this.mPerDegLon = 111320 * Math.cos((lat0 * Math.PI) / 180)
    this.frame = Math.min(this.frame, d.frames.length - 1)
    this.ensureLayers()
  }

  unload(): void {
    this.pause()
    this.data = null
    this.render()
  }

  setSpeed(mult: number): void {
    this.speed = mult
    if (this.playing) {
      this.pause()
      this.play()
    }
  }

  play(): void {
    if (this.playing || !this.data) return
    this.playing = true
    const intervalMs = (this.data.frame_s * 1000) / this.speed
    this.timer = window.setInterval(() => {
      if (!this.data) return
      this.frame = (this.frame + 1) % this.data.frames.length
      this.render()
      this.onFrame?.(this.frame)
    }, Math.max(30, intervalMs))
  }

  pause(): void {
    this.playing = false
    if (this.timer !== null) {
      window.clearInterval(this.timer)
      this.timer = null
    }
  }

  seek(frame: number): void {
    this.frame = Math.max(0, Math.min(frame, this.nFrames - 1))
    this.render()
  }

  /** 再生ヘッドが動いたときの通知(UI 同期用) */
  onFrame: ((frame: number) => void) | null = null

  frameInfo(): { t: number; n: number; meanKmh: number } | null {
    if (!this.data) return null
    const f = this.data.frames[this.frame]
    const n = f.p.length / 3
    let sum = 0
    for (let i = 2; i < f.p.length; i += 3) sum += f.p[i]
    return { t: f.t, n, meanKmh: n ? (sum / n / 10) * 3.6 : 0 }
  }

  private toLngLat(dx: number, dy: number): [number, number] {
    const d = this.data!
    return [d.origin[0] + dx / 10 / this.mPerDegLon, d.origin[1] + dy / 10 / this.mPerDegLat]
  }

  private render(): void {
    const vehSrc = this.map.getSource(SRC_VEH) as maplibregl.GeoJSONSource | undefined
    const sigSrc = this.map.getSource(SRC_SIG) as maplibregl.GeoJSONSource | undefined
    if (!vehSrc || !sigSrc) return
    if (!this.data) {
      vehSrc.setData(EMPTY)
      sigSrc.setData(EMPTY)
      return
    }
    const f = this.data.frames[this.frame]
    const veh: GeoJSON.Feature[] = []
    for (let i = 0; i < f.p.length; i += 3) {
      veh.push({
        type: 'Feature',
        geometry: { type: 'Point', coordinates: this.toLngLat(f.p[i], f.p[i + 1]) },
        properties: { v: f.p[i + 2] / 10 },
      })
    }
    const sig: GeoJSON.Feature[] = this.data.nodes.map((xy, k) => ({
      type: 'Feature',
      geometry: { type: 'Point', coordinates: this.toLngLat(xy[0], xy[1]) },
      properties: { s: f.sig.charCodeAt(k) - 48 },
    }))
    vehSrc.setData({ type: 'FeatureCollection', features: veh })
    sigSrc.setData({ type: 'FeatureCollection', features: sig })
  }
}

/** パネルの「シミュレーション」節の UI を組み立てる。 */
export function initSimUI(player: SimPlayer): void {
  const scenarioSel = document.getElementById('sim-scenario') as HTMLSelectElement
  const playBtn = document.getElementById('sim-play') as HTMLButtonElement
  const slider = document.getElementById('sim-slider') as HTMLInputElement
  const speedSel = document.getElementById('sim-speed') as HTMLSelectElement
  const readout = document.getElementById('sim-readout') as HTMLElement

  const sync = (): void => {
    const info = player.frameInfo()
    if (!info) {
      readout.textContent = 'シナリオを選ぶと読み込みます'
      playBtn.disabled = true
      slider.disabled = true
      return
    }
    playBtn.disabled = false
    slider.disabled = false
    slider.max = String(player.nFrames - 1)
    slider.value = String(player.currentFrame)
    playBtn.textContent = player.isPlaying ? '❙❙ 停止' : '▶ 再生'
    readout.textContent =
      `t=${Math.round(info.t)}s  ${info.n}台  平均 ${info.meanKmh.toFixed(1)} km/h`
  }
  player.onFrame = sync

  scenarioSel.addEventListener('change', () => {
    const v = scenarioSel.value
    if (!v) {
      player.unload()
      sync()
      return
    }
    readout.textContent = '読み込み中…'
    player
      .load(v)
      .then(() => {
        sync()
        if (!player.isPlaying) player.play()
        sync()
      })
      .catch(() => {
        readout.textContent = 'データが無い。make sim を実行して再生成する'
      })
  })
  playBtn.addEventListener('click', () => {
    if (player.isPlaying) player.pause()
    else player.play()
    sync()
  })
  slider.addEventListener('input', () => {
    player.pause()
    player.seek(Number(slider.value))
    sync()
  })
  speedSel.addEventListener('change', () => player.setSpeed(Number(speedSel.value)))
  sync()
}
