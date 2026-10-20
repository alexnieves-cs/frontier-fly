import type { Detection } from '../perception';

/** Fill the detection panel: the top detections, the clear/not-clear verdict and the count. */
export function renderDetections(root: ParentNode, detections: Detection[], clearText: string): void {
  const list = root.querySelector('#detect-list');
  if (list) {
    const rows = detections.slice(0, 8).map((det) => {
      const cls = det.kind === 'fp' ? ' class="fp"' : '';
      return `<li${cls}><span>${det.classId}</span><span>${det.confidence.toFixed(2)} · ${det.distance.toFixed(1)}m</span></li>`;
    });
    list.innerHTML = rows.join('') || '<li class="muted">no detections</li>';
  }
  const clear = root.querySelector('#detect-clear');
  if (clear) clear.textContent = clearText;
  const count = root.querySelector('#detect-count');
  if (count) count.textContent = String(detections.length);
}
