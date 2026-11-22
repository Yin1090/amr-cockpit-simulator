/**
 * 360-degree 2D LiDAR raycasting, 8-sector zone calculation, collision resolution,
 * and 0.05 m Bresenham SLAM occupancy grid mapping.
 * Pure ESM module — zero DOM or browser global dependencies.
 */

import { ROBOT_RADIUS_M } from './kinematics.js';

export const LIDAR_RAYS = 360;
export const LIDAR_MIN_RANGE_M = 0.05;
export const LIDAR_ZONE_MAX_M = 3.2;
export const LIDAR_MAX_RANGE_M = 6.0;

export const ZONE_NAMES = Object.freeze([
  'front',
  'front_left',
  'left',
  'back_left',
  'back',
  'back_right',
  'right',
  'front_right',
]);

export const DEFAULT_GRID_CONFIG = Object.freeze({
  res: 0.05,
  w: 120,
  h: 120,
  ox: -3.0,
  oy: -3.0,
  yaw: 0,
});

/**
 * Ray intersection against axis-aligned arena boundary walls.
 * @param {number} ox
 * @param {number} oy
 * @param {number} dx
 * @param {number} dy
 * @param {{ minX: number, maxX: number, minY: number, maxY: number }|null} bounds
 * @returns {number}
 */
export function intersectRayBounds(ox, oy, dx, dy, bounds) {
  if (!bounds) return Infinity;
  let best = Infinity;
  if (Math.abs(dx) > 1e-9) {
    const tMinX = (bounds.minX - ox) / dx;
    if (tMinX > 0) {
      const y = oy + tMinX * dy;
      if (y >= bounds.minY - 1e-6 && y <= bounds.maxY + 1e-6) best = Math.min(best, tMinX);
    }
    const tMaxX = (bounds.maxX - ox) / dx;
    if (tMaxX > 0) {
      const y = oy + tMaxX * dy;
      if (y >= bounds.minY - 1e-6 && y <= bounds.maxY + 1e-6) best = Math.min(best, tMaxX);
    }
  }
  if (Math.abs(dy) > 1e-9) {
    const tMinY = (bounds.minY - oy) / dy;
    if (tMinY > 0) {
      const x = ox + tMinY * dx;
      if (x >= bounds.minX - 1e-6 && x <= bounds.maxX + 1e-6) best = Math.min(best, tMinY);
    }
    const tMaxY = (bounds.maxY - oy) / dy;
    if (tMaxY > 0) {
      const x = ox + tMaxY * dx;
      if (x >= bounds.minX - 1e-6 && x <= bounds.maxX + 1e-6) best = Math.min(best, tMaxY);
    }
  }
  return best;
}

/**
 * Ray intersection against a circular obstacle.
 * @param {number} ox
 * @param {number} oy
 * @param {number} dx
 * @param {number} dy
 * @param {{ x: number, y: number, r: number }} circle
 * @returns {number}
 */
export function intersectRayCircle(ox, oy, dx, dy, circle) {
  const fx = ox - circle.x;
  const fy = oy - circle.y;
  const b = 2 * (fx * dx + fy * dy);
  const c = fx * fx + fy * fy - circle.r * circle.r;
  if (c <= 0) {
    return LIDAR_MIN_RANGE_M;
  }
  const disc = b * b - 4 * c;
  if (disc < 0) return Infinity;
  const sqrtDisc = Math.sqrt(disc);
  const t1 = (-b - sqrtDisc) / 2;
  if (t1 > 0) return t1;
  const t2 = (-b + sqrtDisc) / 2;
  return t2 > 0 ? t2 : Infinity;
}

/**
 * Ray intersection against an oriented bounding box (OBB).
 * @param {number} ox
 * @param {number} oy
 * @param {number} dx
 * @param {number} dy
 * @param {{ x: number, y: number, w: number, h: number, yaw?: number }} rect
 * @returns {number}
 */
export function intersectRayRect(ox, oy, dx, dy, rect) {
  const yaw = rect.yaw ?? 0;
  const cos = Math.cos(-yaw);
  const sin = Math.sin(-yaw);
  const relX = ox - rect.x;
  const relY = oy - rect.y;
  const lx = relX * cos - relY * sin;
  const ly = relX * sin + relY * cos;
  const ldx = dx * cos - dy * sin;
  const ldy = dx * sin + dy * cos;

  const hx = rect.w / 2;
  const hy = rect.h / 2;

  let tMin = -Infinity;
  let tMax = Infinity;

  if (Math.abs(ldx) < 1e-9) {
    if (lx < -hx || lx > hx) return Infinity;
  } else {
    let tx1 = (-hx - lx) / ldx;
    let tx2 = (hx - lx) / ldx;
    if (tx1 > tx2) [tx1, tx2] = [tx2, tx1];
    tMin = Math.max(tMin, tx1);
    tMax = Math.min(tMax, tx2);
  }

  if (Math.abs(ldy) < 1e-9) {
    if (ly < -hy || ly > hy) return Infinity;
  } else {
    let ty1 = (-hy - ly) / ldy;
    let ty2 = (hy - ly) / ldy;
    if (ty1 > ty2) [ty1, ty2] = [ty2, ty1];
    tMin = Math.max(tMin, ty1);
    tMax = Math.min(tMax, ty2);
  }

  if (tMax < 0 || tMin > tMax) return Infinity;
  if (tMin <= 0 && tMax >= 0) return LIDAR_MIN_RANGE_M;
  return tMin > 0 ? tMin : Infinity;
}

/**
 * Casts a 360-degree planar LiDAR scan from robot pose against world bounds and obstacles.
 * @param {{ x: number, y: number, th: number }} pose
 * @param {{ bounds?: object, obstacles?: Array<object> }} world
 * @param {{ numRays?: number, minAng?: number, maxRangeM?: number, minRangeM?: number }} [options]
 * @returns {{ ranges: number[], hits: boolean[], min_ang: number }}
 */
export function raycastWorld(pose, world, options = {}) {
  const numRays = options.numRays ?? LIDAR_RAYS;
  const minAng = options.minAng ?? 0;
  const maxRangeM = options.maxRangeM ?? LIDAR_MAX_RANGE_M;
  const minRangeM = options.minRangeM ?? LIDAR_MIN_RANGE_M;
  const stepDeg = 360 / numRays;
  const obstacles = world?.obstacles ?? [];
  const bounds = world?.bounds ?? null;

  const ranges = new Array(numRays);
  const hits = new Array(numRays);

  for (let i = 0; i < numRays; i++) {
    const relDeg = minAng + i * stepDeg;
    const bearing = pose.th + (relDeg * Math.PI) / 180;
    const dx = Math.cos(bearing);
    const dy = Math.sin(bearing);

    let closest = intersectRayBounds(pose.x, pose.y, dx, dy, bounds);
    for (let j = 0; j < obstacles.length; j++) {
      const obs = obstacles[j];
      if (obs.enabled === false) continue;
      let t = Infinity;
      if (obs.type === 'circle') {
        t = intersectRayCircle(pose.x, pose.y, dx, dy, obs);
      } else if (obs.type === 'rect') {
        t = intersectRayRect(pose.x, pose.y, dx, dy, obs);
      }
      if (t < closest) {
        closest = t;
      }
    }

    const hit = Number.isFinite(closest) && closest <= maxRangeM;
    const clamped = hit ? Math.max(minRangeM, closest) : maxRangeM;
    hits[i] = hit;
    ranges[i] = Number(clamped.toFixed(3));
  }

  return { ranges, hits, min_ang: minAng };
}

/**
 * Maps a CCW robot-relative angle in degrees [0, 360) to one of the 8 safety zones.
 * @param {number} relDeg
 * @returns {string}
 */
export function angleDegToZone(relDeg) {
  const deg = ((relDeg % 360) + 360) % 360;
  if (deg >= 337.5 || deg < 22.5) return 'front';
  if (deg < 67.5) return 'front_left';
  if (deg < 112.5) return 'left';
  if (deg < 157.5) return 'back_left';
  if (deg < 202.5) return 'back';
  if (deg < 247.5) return 'back_right';
  if (deg < 292.5) return 'right';
  return 'front_right';
}

/**
 * Computes minimum distance (in cm, 999.0 when clear) per 8 CCW zones from a 360-ray scan.
 * @param {number[]} ranges
 * @param {number} [minAng=0]
 * @param {number} [maxZoneRangeM=3.2]
 * @returns {{ zones: Record<string, number>, min_m: number, min_ang: number }}
 */
export function computeZonesFromRanges(ranges, minAng = 0, maxZoneRangeM = LIDAR_ZONE_MAX_M) {
  const minMeters = {
    front: Infinity,
    front_left: Infinity,
    left: Infinity,
    back_left: Infinity,
    back: Infinity,
    back_right: Infinity,
    right: Infinity,
    front_right: Infinity,
  };

  let globalMinM = Infinity;
  let globalMinAng = 0;
  const n = ranges?.length ?? 0;
  const stepDeg = n > 0 ? 360 / n : 1;

  for (let i = 0; i < n; i++) {
    const d = ranges[i];
    if (!Number.isFinite(d) || d < LIDAR_MIN_RANGE_M) continue;
    const relDeg = ((minAng + i * stepDeg) % 360 + 360) % 360;
    if (d < globalMinM) {
      globalMinM = d;
      globalMinAng = Number(relDeg.toFixed(1));
    }
    if (d <= maxZoneRangeM) {
      const zone = angleDegToZone(relDeg);
      if (d < minMeters[zone]) {
        minMeters[zone] = d;
      }
    }
  }

  const zones = {};
  for (const name of ZONE_NAMES) {
    const m = minMeters[name];
    zones[name] = Number.isFinite(m) ? Math.round(m * 1000) / 10 : 999.0;
  }

  return {
    zones,
    min_m: Number.isFinite(globalMinM) ? Number(globalMinM.toFixed(3)) : 999.0,
    min_ang: globalMinAng,
  };
}

/**
 * Resolves physical robot footprint penetration against arena bounds and obstacles.
 * @param {{ x: number, y: number, th: number }} pose
 * @param {{ bounds?: object, obstacles?: Array<object> }} world
 * @param {number} [robotRadiusM=0.14]
 * @returns {{ x: number, y: number, th: number, collided: boolean }}
 */
export function resolveRobotCollision(pose, world, robotRadiusM = ROBOT_RADIUS_M) {
  let x = pose.x;
  let y = pose.y;
  let collided = false;

  const bounds = world?.bounds;
  if (bounds) {
    const minX = bounds.minX + robotRadiusM;
    const maxX = bounds.maxX - robotRadiusM;
    const minY = bounds.minY + robotRadiusM;
    const maxY = bounds.maxY - robotRadiusM;
    if (x < minX) { x = minX; collided = true; }
    if (x > maxX) { x = maxX; collided = true; }
    if (y < minY) { y = minY; collided = true; }
    if (y > maxY) { y = maxY; collided = true; }
  }

  const obstacles = world?.obstacles ?? [];
  for (let i = 0; i < obstacles.length; i++) {
    const obs = obstacles[i];
    if (obs.enabled === false) continue;

    if (obs.type === 'circle') {
      const dx = x - obs.x;
      const dy = y - obs.y;
      const dist = Math.hypot(dx, dy);
      const minSep = robotRadiusM + obs.r;
      if (dist < minSep) {
        collided = true;
        if (dist > 1e-6) {
          const push = minSep - dist;
          x += (dx / dist) * push;
          y += (dy / dist) * push;
        } else {
          y -= minSep;
        }
      }
    } else if (obs.type === 'rect') {
      const yaw = obs.yaw ?? 0;
      const cos = Math.cos(-yaw);
      const sin = Math.sin(-yaw);
      const relX = x - obs.x;
      const relY = y - obs.y;
      const lx = relX * cos - relY * sin;
      const ly = relX * sin + relY * cos;

      const hx = obs.w / 2;
      const hy = obs.h / 2;
      const cx = Math.max(-hx, Math.min(hx, lx));
      const cy = Math.max(-hy, Math.min(hy, ly));
      const dx = lx - cx;
      const dy = ly - cy;
      const dist = Math.hypot(dx, dy);

      if (dist < robotRadiusM) {
        collided = true;
        let pushLx = 0;
        let pushLy = 0;
        if (dist > 1e-6) {
          const push = robotRadiusM - dist;
          pushLx = (dx / dist) * push;
          pushLy = (dy / dist) * push;
        } else {
          // Center inside rect: push out along nearest edge
          const penX = hx - Math.abs(lx);
          const penY = hy - Math.abs(ly);
          if (penX < penY) {
            pushLx = (lx >= 0 ? 1 : -1) * (penX + robotRadiusM);
          } else {
            pushLy = (ly >= 0 ? 1 : -1) * (penY + robotRadiusM);
          }
        }
        const cosW = Math.cos(yaw);
        const sinW = Math.sin(yaw);
        x += pushLx * cosW - pushLy * sinW;
        y += pushLx * sinW + pushLy * cosW;
      }
    }
  }

  return { x, y, th: pose.th, collided };
}

/**
 * Creates a SLAM occupancy grid initialized to -1 (unknown).
 * @param {{ res?: number, w?: number, h?: number, ox?: number, oy?: number, yaw?: number }} [config]
 */
export function createOccupancyGrid(config = {}) {
  const res = config.res ?? DEFAULT_GRID_CONFIG.res;
  const w = config.w ?? DEFAULT_GRID_CONFIG.w;
  const h = config.h ?? DEFAULT_GRID_CONFIG.h;
  if (w * h > 4_000_000) {
    throw new RangeError('Occupancy grid exceeds 4,000,000 cells');
  }
  const ox = config.ox ?? DEFAULT_GRID_CONFIG.ox;
  const oy = config.oy ?? DEFAULT_GRID_CONFIG.oy;
  const yaw = config.yaw ?? DEFAULT_GRID_CONFIG.yaw;
  return {
    res,
    w,
    h,
    ox,
    oy,
    yaw,
    data: new Int8Array(w * h).fill(-1),
  };
}

/**
 * Resets all SLAM occupancy cells back to -1 (unknown).
 * @param {{ data: Int8Array }} grid
 */
export function resetOccupancyGrid(grid) {
  grid.data.fill(-1);
}

/**
 * Updates the occupancy grid with a 360-degree scan via integer Bresenham ray tracing.
 * @param {{ res: number, w: number, h: number, ox: number, oy: number, data: Int8Array }} grid
 * @param {{ x: number, y: number, th: number }} pose
 * @param {number[]} ranges
 * @param {boolean[]} hits
 * @param {number} [minAng=0]
 */
export function updateOccupancyGrid(grid, pose, ranges, hits, minAng = 0) {
  const { res, w, h, ox, oy, data } = grid;
  const gx0 = Math.floor((pose.x - ox) / res);
  const gy0 = Math.floor((pose.y - oy) / res);
  const n = ranges.length;
  const stepDeg = n > 0 ? 360 / n : 1;

  for (let i = 0; i < n; i++) {
    const dist = ranges[i];
    if (!Number.isFinite(dist) || dist <= 0) continue;
    const bearing = pose.th + ((minAng + i * stepDeg) * Math.PI) / 180;
    const wx = pose.x + Math.cos(bearing) * dist;
    const wy = pose.y + Math.sin(bearing) * dist;
    const gx1 = Math.floor((wx - ox) / res);
    const gy1 = Math.floor((wy - oy) / res);

    // Integer Bresenham line from (gx0, gy0) to (gx1, gy1)
    let cx = gx0;
    let cy = gy0;
    const dx = Math.abs(gx1 - gx0);
    const dy = Math.abs(gy1 - gy0);
    const sx = gx0 < gx1 ? 1 : -1;
    const sy = gy0 < gy1 ? 1 : -1;
    let err = dx - dy;

    let guard = 0;
    const maxSteps = w + h + 4;
    while (guard++ < maxSteps) {
      const atEnd = cx === gx1 && cy === gy1;
      if (cx >= 0 && cx < w && cy >= 0 && cy < h) {
        const idx = cy * w + cx;
        if (atEnd) {
          data[idx] = hits[i] ? 100 : 0;
        } else {
          data[idx] = 0;
        }
      }
      if (atEnd) break;
      const e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        cx += sx;
      }
      if (e2 < dx) {
        err += dx;
        cy += sy;
      }
    }
  }
}

/**
 * Serializes occupancy grid into the /api/map_full JSON wire schema.
 * @param {{ w: number, h: number, res: number, ox: number, oy: number, yaw: number, data: Int8Array }} grid
 * @returns {{ w: number, h: number, res: number, ox: number, oy: number, yaw: number, data: number[] }}
 */
export function serializeOccupancyGrid(grid) {
  return {
    w: grid.w,
    h: grid.h,
    res: grid.res,
    ox: grid.ox,
    oy: grid.oy,
    yaw: grid.yaw,
    data: Array.from(grid.data),
  };
}
