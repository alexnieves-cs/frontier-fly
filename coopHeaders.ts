/** COOP/COEP so SharedArrayBuffer is available. No secrets required. */
export const COOP_COEP_HEADERS: Record<string, string> = {
  'Cross-Origin-Opener-Policy': 'same-origin',
  'Cross-Origin-Embedder-Policy': 'require-corp',
};
