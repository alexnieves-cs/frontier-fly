import type { FrameDetector, VisionDetection } from '../../vision/detector';
import { parseHumanoidPoses, type HumanoidPoses } from '../../vision/humanoid';
import type { Frame } from '../../vision/scene';

export interface VisionKit {
  poses: HumanoidPoses;
  detector: FrameDetector;
}

let kit: Promise<VisionKit> | null = null;

/** The baked people and the shipped detector, fetched once. TF.js and the model load only in camera mode. */
export function loadVisionKit(): Promise<VisionKit> {
  kit ??= (async () => {
    const { frameDetector, loadDetectorFromUrl } = await import('../../vision/detector');
    const [buffer, bundle] = await Promise.all([
      fetch(`${import.meta.env.BASE_URL}assets/targets/humanoid-poses.bin`).then((r) => {
        if (!r.ok) throw new Error(`humanoid poses: HTTP ${r.status}`);
        return r.arrayBuffer();
      }),
      loadDetectorFromUrl(),
    ]);
    return { poses: parseHumanoidPoses(buffer), detector: frameDetector(bundle) };
  })();
  kit.catch(() => (kit = null));
  return kit;
}

const LABEL: Record<string, string> = { person: 'person' };

/** Draws the latest camera frame at 2x with the detector's boxes. */
export class CameraInset {
  private frame: Frame | null = null;
  private detections: VisionDetection[] = [];
  private dirty = false;
  private readonly image: ImageData;
  frames = 0;

  constructor(private readonly canvas: HTMLCanvasElement) {
    canvas.width = 320;
    canvas.height = 240;
    this.image = new ImageData(160, 120);
  }

  push(frame: Frame, detections: VisionDetection[]): void {
    this.frame = frame;
    this.detections = detections;
    this.dirty = true;
    this.frames++;
  }

  draw(): void {
    if (!this.dirty || !this.frame) return;
    this.dirty = false;
    const ctx = this.canvas.getContext('2d');
    if (!ctx) return;
    const f = this.frame;
    const px = this.image.data;
    for (let i = 0, j = 0; i < f.width * f.height; i++, j += 3) {
      px[i * 4] = f.rgb[j]!;
      px[i * 4 + 1] = f.rgb[j + 1]!;
      px[i * 4 + 2] = f.rgb[j + 2]!;
      px[i * 4 + 3] = 255;
    }
    const small = new OffscreenCanvas(f.width, f.height);
    small.getContext('2d')!.putImageData(this.image, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(small, 0, 0, this.canvas.width, this.canvas.height);
    ctx.lineWidth = 2;
    ctx.font = '600 11px system-ui, sans-serif';
    for (const d of this.detections) {
      if (d.confidence < 0.3) continue;
      const x = d.bbox.x * this.canvas.width;
      const y = d.bbox.y * this.canvas.height;
      const w = d.bbox.w * this.canvas.width;
      const h = d.bbox.h * this.canvas.height;
      const color = d.classId === 'person' ? '#ff5a4f' : '#ffc53d';
      ctx.strokeStyle = color;
      ctx.strokeRect(x, y, w, h);
      const text = `${LABEL[d.classId] ?? d.classId} ${Math.round(d.confidence * 100)}%`;
      const tw = ctx.measureText(text).width + 6;
      ctx.fillStyle = color;
      ctx.fillRect(x - 1, Math.max(0, y - 15), tw, 15);
      ctx.fillStyle = '#101010';
      ctx.fillText(text, x + 2, Math.max(11, y - 4));
    }
  }
}
