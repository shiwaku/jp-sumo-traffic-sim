export type Theme = 'light' | 'dark'

const STORAGE_KEY = 'sapporo-sim-theme'

export function initialTheme(): Theme {
  // 既定はダーク。ユーザーが切り替えたらその選択を記憶する。
  const saved = localStorage.getItem(STORAGE_KEY)
  return saved === 'light' || saved === 'dark' ? saved : 'dark'
}

/** <html data-theme="…"> を更新して現在テーマを保存する。 */
export function applyThemeAttr(theme: Theme): void {
  document.documentElement.dataset.theme = theme
  localStorage.setItem(STORAGE_KEY, theme)
}
