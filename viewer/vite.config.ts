import { defineConfig } from 'vite'

// データ(GeoJSON + meta.json)は scripts/05_export_viewer.py が
// viewer/public/data/ に書き出す。public/ 配下なので dev / build とも
// 追加のミドルウェアなしでそのまま配信される。
export default defineConfig({
  base: './',
  server: { port: 8002 },
  define: {
    __BUILD_TIME__: JSON.stringify(
      new Date().toISOString().replace('T', ' ').slice(0, 16) + ' UTC',
    ),
  },
})
