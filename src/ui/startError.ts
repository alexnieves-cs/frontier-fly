/**
 * What a page shows when its 3D view cannot start. Without WebGL three.js throws "Error creating WebGL
 * context", which tells a visitor nothing, so that case gets a plain explanation and a way to still see
 * the drone fly: the recorded flights in the README.
 */

const README_URL = 'https://github.com/alexnieves-cs/frontier-fly#readme';

export const NO_WEBGL_TEXT =
  'This page draws its 3D view with WebGL, which this browser has turned off or does not support. ' +
  'Turning on hardware (graphics) acceleration in the browser settings and reloading usually fixes it.';

export function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

export function isWebglError(error: unknown): boolean {
  return /webgl/i.test(errorMessage(error));
}

/** Fill `el` with the reason the page could not start. */
export function showStartError(el: HTMLElement, error: unknown): void {
  if (!isWebglError(error)) {
    el.textContent = errorMessage(error);
    return;
  }
  // One inline block, so pages whose boot box is a flex container keep the text and link together.
  const note = document.createElement('span');
  note.style.cssText = 'display:inline-block;max-width:36em;text-align:center;line-height:1.5';
  note.textContent = `${NO_WEBGL_TEXT} `;
  const link = document.createElement('a');
  link.href = README_URL;
  link.textContent = 'Watch the recorded flights instead.';
  note.append(link);
  el.replaceChildren(note);
}
