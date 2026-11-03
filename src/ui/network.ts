/** True when the visitor asked to save data (Save-Data, Lite mode) or is on a 2G-class link. Big optional downloads wait for a click then. */
export function savingData(): boolean {
  const connection = (navigator as Navigator & { connection?: { saveData?: boolean; effectiveType?: string } }).connection;
  return Boolean(connection?.saveData) || /2g/.test(connection?.effectiveType ?? '');
}
