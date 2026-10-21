/**
 * Local east-north-up meters to WGS84 and back.
 *
 * The sim frame is x east, y north, z up, in meters, with no geodetic anchor.
 * An export pins the local origin (0, 0, 0) to a latitude, longitude and AMSL
 * altitude, then converts through ECEF so the mapping is exact on the
 * ellipsoid (no flat-earth drift across a 1 km site).
 */

/** WGS84 semi-major axis, meters. */
const A = 6378137;
/** WGS84 flattening. */
const F = 1 / 298.257223563;
const E2 = F * (2 - F);
const B = A * (1 - F);
const EP2 = (A * A - B * B) / (B * B);

const RAD = Math.PI / 180;

export interface GeoPoint {
  latDeg: number;
  lonDeg: number;
  /** Height above mean sea level, meters. Treated as ellipsoid height; the geoid offset cancels across a small site. */
  altM: number;
}

export interface GeoOrigin extends GeoPoint {
  id: string;
  label: string;
}

/**
 * Where the local origin lands. The SITL presets are the default spawn points
 * of the PX4 and ArduPilot software-in-the-loop simulators, so an exported plan
 * opens right next to a simulated vehicle in QGroundControl.
 */
export const GEO_ORIGINS = {
  'px4-sitl': { id: 'px4-sitl', label: 'PX4 SITL default home', latDeg: 47.397742, lonDeg: 8.545594, altM: 488 },
  'ardupilot-sitl': {
    id: 'ardupilot-sitl',
    label: 'ArduPilot SITL default home',
    latDeg: -35.363261,
    lonDeg: 149.16523,
    altM: 584,
  },
} as const satisfies Record<string, GeoOrigin>;

export type GeoOriginId = keyof typeof GEO_ORIGINS;

export function isValidOrigin(origin: GeoPoint): boolean {
  return (
    Number.isFinite(origin.latDeg) &&
    Number.isFinite(origin.lonDeg) &&
    Number.isFinite(origin.altM) &&
    Math.abs(origin.latDeg) <= 85 &&
    Math.abs(origin.lonDeg) <= 180 &&
    origin.altM > -500 &&
    origin.altM < 9000
  );
}

type Vec3 = [number, number, number];

function geodeticToEcef(p: GeoPoint): Vec3 {
  const lat = p.latDeg * RAD;
  const lon = p.lonDeg * RAD;
  const s = Math.sin(lat);
  const n = A / Math.sqrt(1 - E2 * s * s);
  const r = (n + p.altM) * Math.cos(lat);
  return [r * Math.cos(lon), r * Math.sin(lon), (n * (1 - E2) + p.altM) * s];
}

function ecefToGeodetic([x, y, z]: Vec3): GeoPoint {
  const lon = Math.atan2(y, x);
  const p = Math.hypot(x, y);
  // Bowring's closed form, then two Newton refinements: sub-millimeter anywhere off the poles.
  let lat = Math.atan2(z * A, p * B);
  {
    const s = Math.sin(lat);
    const c = Math.cos(lat);
    lat = Math.atan2(z + EP2 * B * s * s * s, p - E2 * A * c * c * c);
  }
  let h = 0;
  for (let i = 0; i < 2; i++) {
    const s = Math.sin(lat);
    const n = A / Math.sqrt(1 - E2 * s * s);
    h = p / Math.cos(lat) - n;
    lat = Math.atan2(z, p * (1 - (E2 * n) / (n + h)));
  }
  return { latDeg: lat / RAD, lonDeg: lon / RAD, altM: h };
}

/** East-north-up frame tangent at `origin`. */
export interface LocalFrame {
  readonly origin: GeoPoint;
  toGeo(east: number, north: number, up?: number): GeoPoint;
  toLocal(point: GeoPoint): { east: number; north: number; up: number };
}

export function localFrame(origin: GeoPoint): LocalFrame {
  if (!isValidOrigin(origin)) throw new RangeError('The geodetic origin is out of range.');
  const o = geodeticToEcef(origin);
  const lat = origin.latDeg * RAD;
  const lon = origin.lonDeg * RAD;
  const sl = Math.sin(lat);
  const cl = Math.cos(lat);
  const so = Math.sin(lon);
  const co = Math.cos(lon);
  return {
    origin: { ...origin },
    toGeo(east, north, up = 0) {
      const dx = -so * east - sl * co * north + cl * co * up;
      const dy = co * east - sl * so * north + cl * so * up;
      const dz = cl * north + sl * up;
      return ecefToGeodetic([o[0] + dx, o[1] + dy, o[2] + dz]);
    },
    toLocal(point) {
      const [x, y, z] = geodeticToEcef(point);
      const dx = x - o[0];
      const dy = y - o[1];
      const dz = z - o[2];
      return {
        east: -so * dx + co * dy,
        north: -sl * co * dx - sl * so * dy + cl * dz,
        up: cl * co * dx + cl * so * dy + sl * dz,
      };
    },
  };
}

/** Great-circle distance on the mean sphere, meters. Good enough for sanity checks. */
export function haversineM(a: GeoPoint, b: GeoPoint): number {
  const r = 6371008.8;
  const dLat = (b.latDeg - a.latDeg) * RAD;
  const dLon = (b.lonDeg - a.lonDeg) * RAD;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(a.latDeg * RAD) * Math.cos(b.latDeg * RAD) * Math.sin(dLon / 2) ** 2;
  return 2 * r * Math.asin(Math.min(1, Math.sqrt(h)));
}
