/**
 * Arena presets, dynamic moving obstacles, and custom polar/world obstacle helpers.
 * Pure ESM module — zero DOM or browser global dependencies.
 */

import { ROBOT_RADIUS_M } from './kinematics.js';

export const DEFAULT_ARENA_BOUNDS = Object.freeze({
  minX: -2.5,
  maxX: 2.5,
  minY: -2.5,
  maxY: 2.5,
});

export const SCENARIOS = Object.freeze({
  clear: Object.freeze({
    id: 'clear',
    label: 'Clear Arena',
    description: 'Open 5.0 x 5.0 m walled arena for free 8-direction kinematics and SLAM mapping.',
    initialPose: Object.freeze({ x: 0, y: 0, th: Math.PI / 2 }),
    bounds: DEFAULT_ARENA_BOUNDS,
    obstacles: Object.freeze([]),
  }),
  front_obstacle: Object.freeze({
    id: 'front_obstacle',
    label: 'Front Obstacle (< 40 cm)',
    description: 'Static block 33 cm ahead: blocks F immediately while keeping B/L/R open for recovery.',
    initialPose: Object.freeze({ x: 0, y: 0, th: Math.PI / 2 }),
    bounds: DEFAULT_ARENA_BOUNDS,
    obstacles: Object.freeze([
      Object.freeze({
        id: 'obs-front',
        type: 'rect',
        x: 0,
        y: 0.43,
        w: 0.24,
        h: 0.20,
        yaw: 0,
      }),
    ]),
  }),
  corridor: Object.freeze({
    id: 'corridor',
    label: 'Corridor',
    description: 'Narrow 1.10 m corridor with ~48 cm lateral wall clearance and forward end-stop.',
    initialPose: Object.freeze({ x: 0, y: -1.2, th: Math.PI / 2 }),
    bounds: DEFAULT_ARENA_BOUNDS,
    obstacles: Object.freeze([
      Object.freeze({
        id: 'obs-wall-left',
        type: 'rect',
        x: -0.55,
        y: 0.0,
        w: 0.14,
        h: 2.8,
        yaw: 0,
      }),
      Object.freeze({
        id: 'obs-wall-right',
        type: 'rect',
        x: 0.55,
        y: 0.0,
        w: 0.14,
        h: 2.8,
        yaw: 0,
      }),
      Object.freeze({
        id: 'obs-corridor-end',
        type: 'rect',
        x: 0.0,
        y: 1.45,
        w: 1.24,
        h: 0.16,
        yaw: 0,
      }),
    ]),
  }),
  dynamic_moving: Object.freeze({
    id: 'dynamic_moving',
    label: 'Dynamic Moving',
    description: 'Approaching obstacle starting 1.54 m ahead at 0.35 m/s to trigger mid-motion auto-halt.',
    initialPose: Object.freeze({ x: 0, y: -0.8, th: Math.PI / 2 }),
    bounds: DEFAULT_ARENA_BOUNDS,
    obstacles: Object.freeze([
      Object.freeze({
        id: 'obs-dynamic',
        type: 'circle',
        x: 0,
        y: 0.90,
        r: 0.16,
        dynamic: true,
        vx: 0,
        vy: -0.35,
        minX: -1.5,
        maxX: 1.5,
        minY: -0.35,
        maxY: 1.20,
      }),
    ]),
  }),
});

/**
 * Clones a scenario definition into a mutable simulation world.
 * @param {string} [scenarioId='clear']
 * @returns {{ scenarioId: string, initialPose: {x:number,y:number,th:number}, bounds: object, obstacles: Array<object>, dynamicPaused: boolean }}
 */
export function createWorldFromScenario(scenarioId = 'clear') {
  const preset = SCENARIOS[scenarioId] ?? SCENARIOS.clear;
  return {
    scenarioId: preset.id,
    initialPose: { ...preset.initialPose },
    bounds: { ...preset.bounds },
    obstacles: preset.obstacles.map((obs) => ({ ...obs })),
    dynamicPaused: false,
  };
}

/**
 * Advances dynamic obstacles by dtSec, reflecting velocities at patrol bounds.
 * @param {{ obstacles: Array<object>, dynamicPaused?: boolean }} world
 * @param {number} dtSec
 */
export function stepDynamicObstacles(world, dtSec) {
  if (!world || world.dynamicPaused || !Number.isFinite(dtSec) || dtSec <= 0) {
    return;
  }
  for (let i = 0; i < world.obstacles.length; i++) {
    const obs = world.obstacles[i];
    if (!obs.dynamic || obs.enabled === false) continue;

    if (Number.isFinite(obs.vx) && obs.vx !== 0) {
      obs.x += obs.vx * dtSec;
      const minX = obs.minX ?? -2.0;
      const maxX = obs.maxX ?? 2.0;
      if (obs.x <= minX) {
        obs.x = minX;
        obs.vx = Math.abs(obs.vx);
      } else if (obs.x >= maxX) {
        obs.x = maxX;
        obs.vx = -Math.abs(obs.vx);
      }
    }

    if (Number.isFinite(obs.vy) && obs.vy !== 0) {
      obs.y += obs.vy * dtSec;
      const minY = obs.minY ?? -2.0;
      const maxY = obs.maxY ?? 2.0;
      if (obs.y <= minY) {
        obs.y = minY;
        obs.vy = Math.abs(obs.vy);
      } else if (obs.y >= maxY) {
        obs.y = maxY;
        obs.vy = -Math.abs(obs.vy);
      }
    }
  }
}

/**
 * Places or updates the custom polar obstacle relative to the robot pose.
 * distanceM represents the surface distance from the robot center LiDAR origin
 * so setting distanceM = 0.30 produces a ~30.0 cm reading in the target zone.
 * @param {{ bounds: object, obstacles: Array<object> }} world
 * @param {{ x: number, y: number, th: number }} pose
 * @param {{ distanceM: number, angleDeg?: number, radiusM?: number }} params
 * @returns {object}
 */
export function setCustomObstaclePolar(world, pose, { distanceM, angleDeg = 0, radiusM = 0.15 }) {
  const safeRadius = Math.max(0.08, Math.min(0.35, Number(radiusM) || 0.15));
  const safeDist = Math.max(0.15, Math.min(2.4, Number(distanceM) || 0.5));
  const centerDist = Math.max(ROBOT_RADIUS_M + safeRadius + 0.01, safeDist + safeRadius);
  const rad = ((Number(angleDeg) || 0) * Math.PI) / 180;
  const bearing = pose.th + rad;

  const x = pose.x + Math.cos(bearing) * centerDist;
  const y = pose.y + Math.sin(bearing) * centerDist;

  return upsertCustomObstacle(world, x, y, safeRadius);
}

/**
 * Places or updates the custom obstacle at an exact world coordinate (clamped inside arena).
 * If pose is provided, enforces minimum radial clearance from the robot footprint so
 * click-on-map placement never overlaps the robot body.
 * @param {{ bounds: object, obstacles: Array<object> }} world
 * @param {{ x: number, y: number, radiusM?: number }} params
 * @param {{ x: number, y: number, th: number }|null} [pose=null]
 * @returns {object}
 */
export function setCustomObstacleWorld(world, { x, y, radiusM = 0.14 }, pose = null) {
  const safeRadius = Math.max(0.08, Math.min(0.35, Number(radiusM) || 0.14));
  let cx = Number.isFinite(Number(x)) ? Number(x) : 0;
  let cy = Number.isFinite(Number(y)) ? Number(y) : 0;

  if (pose && Number.isFinite(pose.x) && Number.isFinite(pose.y)) {
    const dx = cx - pose.x;
    const dy = cy - pose.y;
    const dist = Math.hypot(dx, dy);
    const minCenterDist = ROBOT_RADIUS_M + safeRadius + 0.01;
    if (dist < minCenterDist) {
      const angle = dist > 1e-6 ? Math.atan2(dy, dx) : (Number.isFinite(pose.th) ? pose.th : Math.PI / 2);
      cx = pose.x + Math.cos(angle) * minCenterDist;
      cy = pose.y + Math.sin(angle) * minCenterDist;
    }
  }

  const b = world.bounds ?? DEFAULT_ARENA_BOUNDS;
  cx = Math.max(b.minX + safeRadius + 0.02, Math.min(b.maxX - safeRadius - 0.02, cx));
  cy = Math.max(b.minY + safeRadius + 0.02, Math.min(b.maxY - safeRadius - 0.02, cy));
  return upsertCustomObstacle(world, cx, cy, safeRadius);
}

function upsertCustomObstacle(world, x, y, r) {
  const idx = world.obstacles.findIndex((o) => o.id === 'obs-custom');
  const obs = {
    id: 'obs-custom',
    type: 'circle',
    x: Number(x.toFixed(4)),
    y: Number(y.toFixed(4)),
    r,
    custom: true,
    enabled: true,
  };
  if (idx >= 0) {
    world.obstacles[idx] = obs;
  } else {
    world.obstacles.push(obs);
  }
  return obs;
}

/**
 * Removes the custom obstacle from the world if present.
 * @param {{ obstacles: Array<object> }} world
 * @returns {boolean}
 */
export function clearCustomObstacle(world) {
  const idx = world.obstacles.findIndex((o) => o.id === 'obs-custom');
  if (idx >= 0) {
    world.obstacles.splice(idx, 1);
    return true;
  }
  return false;
}
