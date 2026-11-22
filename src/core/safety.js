/**
 * 8-zone < 40 cm obstacle safety interlock, LiDAR health check, and telemetry watchdog.
 * Pure ESM module — zero DOM or browser global dependencies.
 */

export const OBSTACLE_THRESHOLD_CM = 40.0;
export const OBSTACLE_THRESHOLD_M = 0.40;
export const WATCHDOG_TIMEOUT_MS = 1500;
export const LIDAR_MAX_AGE_S = 0.5;
export const ZONE_CLEAR_CM = 999.0;

export const DIRECTION_TO_ZONE = Object.freeze({
  F: 'front',
  B: 'back',
  L: 'left',
  R: 'right',
  FL: 'front_left',
  FR: 'front_right',
  BL: 'back_left',
  BR: 'back_right',
});

export const ZONE_RECOVERY_INFO = Object.freeze({
  front: Object.freeze({
    label: 'FRONT',
    recoveryDirs: Object.freeze(['B', 'L', 'R']),
    message: 'Obstacle FRONT < 40 cm. Reverse or turn away.',
  }),
  back: Object.freeze({
    label: 'BACK',
    recoveryDirs: Object.freeze(['F', 'L', 'R']),
    message: 'Obstacle BACK < 40 cm. Drive forward or turn away.',
  }),
  left: Object.freeze({
    label: 'LEFT',
    recoveryDirs: Object.freeze(['R', 'F', 'B']),
    message: 'Obstacle LEFT < 40 cm. Turn right or drive clear.',
  }),
  right: Object.freeze({
    label: 'RIGHT',
    recoveryDirs: Object.freeze(['L', 'F', 'B']),
    message: 'Obstacle RIGHT < 40 cm. Turn left or drive clear.',
  }),
  front_left: Object.freeze({
    label: 'FRONT-LEFT',
    recoveryDirs: Object.freeze(['B', 'R', 'BR']),
    message: 'Obstacle FRONT-LEFT < 40 cm. Reverse or turn right.',
  }),
  front_right: Object.freeze({
    label: 'FRONT-RIGHT',
    recoveryDirs: Object.freeze(['B', 'L', 'BL']),
    message: 'Obstacle FRONT-RIGHT < 40 cm. Reverse or turn left.',
  }),
  back_left: Object.freeze({
    label: 'BACK-LEFT',
    recoveryDirs: Object.freeze(['F', 'R', 'FR']),
    message: 'Obstacle BACK-LEFT < 40 cm. Drive forward or turn right.',
  }),
  back_right: Object.freeze({
    label: 'BACK-RIGHT',
    recoveryDirs: Object.freeze(['F', 'L', 'FL']),
    message: 'Obstacle BACK-RIGHT < 40 cm. Drive forward or turn left.',
  }),
});

/**
 * Checks whether the LiDAR sensor feed is connected and fresh (< 0.5 s).
 * @param {{ connected?: boolean, age_s?: number }|null} lidar
 * @returns {boolean}
 */
export function isLidarHealthy(lidar) {
  return Boolean(
    lidar &&
    lidar.connected === true &&
    Number.isFinite(lidar.age_s) &&
    lidar.age_s >= 0 &&
    lidar.age_s < LIDAR_MAX_AGE_S
  );
}

/**
 * Checks whether the telemetry watchdog (> 1500 ms without fresh telemetry) has tripped.
 * @param {{ nowMs: number, lastTelemetryAtMs: number, timeoutMs?: number }} params
 * @returns {boolean}
 */
export function isTelemetryWatchdogTripped({ nowMs, lastTelemetryAtMs, timeoutMs = WATCHDOG_TIMEOUT_MS }) {
  return lastTelemetryAtMs > 0 && (nowMs - lastTelemetryAtMs) > timeoutMs;
}

/**
 * Returns all zones currently below thresholdCm, sorted ascending by distance.
 * @param {Record<string, number>} zones
 * @param {number} [thresholdCm=40.0]
 * @returns {Array<{ zone: string, distanceCm: number }>}
 */
export function getBlockedZones(zones, thresholdCm = OBSTACLE_THRESHOLD_CM) {
  if (!zones) return [];
  const list = [];
  for (const [zone, dist] of Object.entries(zones)) {
    if (Number.isFinite(dist) && dist < thresholdCm) {
      list.push({ zone, distanceCm: dist });
    }
  }
  list.sort((a, b) => a.distanceCm - b.distanceCm);
  return list;
}

/**
 * Computes overall safety status level matching the vehicle backend thresholds.
 * @param {Record<string, number>} zones
 * @param {number} minM
 * @returns {'SAFE'|'WARNING'|'DANGER_STOP'}
 */
export function computeSafetyStatus(zones, minM) {
  const frontCm = zones?.front ?? ZONE_CLEAR_CM;
  if (frontCm < 25.0 || (Number.isFinite(minM) && minM <= 0.15)) {
    return 'DANGER_STOP';
  }
  if (frontCm < 50.0 || (Number.isFinite(minM) && minM <= 0.35)) {
    return 'WARNING';
  }
  return 'SAFE';
}

/**
 * Evaluates whether driving in `direction` is safe under current system & LiDAR state.
 * @param {{
 *   armed: boolean,
 *   fault: string|null,
 *   stopPending: boolean,
 *   lidar: { connected: boolean, age_s: number },
 *   telemetryStale: boolean,
 *   direction: string|null,
 *   zones: Record<string, number>,
 *   thresholdCm?: number
 * }} state
 * @returns {{
 *   allowed: boolean,
 *   reasonCode: string|null,
 *   blockedZone: string|null,
 *   distanceCm: number|null,
 *   recoveryDirections: string[],
 *   message: string
 * }}
 */
export function evaluateDriveSafety({
  armed,
  fault,
  stopPending,
  lidar,
  telemetryStale,
  direction,
  zones,
  thresholdCm = OBSTACLE_THRESHOLD_CM,
}) {
  if (fault) {
    return {
      allowed: false,
      reasonCode: 'FAULT',
      blockedZone: null,
      distanceCm: null,
      recoveryDirections: [],
      message: `Vehicle fault active (${fault}). Clear fault before driving.`,
    };
  }
  if (stopPending) {
    return {
      allowed: false,
      reasonCode: 'STOP_PENDING',
      blockedZone: null,
      distanceCm: null,
      recoveryDirections: [],
      message: 'Stop command pending. Release and retry.',
    };
  }
  if (!armed) {
    return {
      allowed: false,
      reasonCode: 'DISARMED',
      blockedZone: null,
      distanceCm: null,
      recoveryDirections: [],
      message: 'Vehicle DISARMED — press ARM to enable drive.',
    };
  }
  if (telemetryStale) {
    return {
      allowed: false,
      reasonCode: 'TELEMETRY_STALE',
      blockedZone: null,
      distanceCm: null,
      recoveryDirections: [],
      message: 'Telemetry watchdog tripped (> 1500 ms) — drive locked.',
    };
  }
  if (!isLidarHealthy(lidar)) {
    return {
      allowed: false,
      reasonCode: 'LIDAR_UNHEALTHY',
      blockedZone: null,
      distanceCm: null,
      recoveryDirections: [],
      message: 'LiDAR unhealthy or stale — drive locked.',
    };
  }

  const zone = DIRECTION_TO_ZONE[direction];
  if (!zone) {
    return {
      allowed: false,
      reasonCode: 'INVALID_DIRECTION',
      blockedZone: null,
      distanceCm: null,
      recoveryDirections: [],
      message: `Unknown drive direction: ${String(direction)}`,
    };
  }

  const distanceCm = zones?.[zone] ?? ZONE_CLEAR_CM;
  if (distanceCm < thresholdCm) {
    const info = ZONE_RECOVERY_INFO[zone];
    return {
      allowed: false,
      reasonCode: 'ZONE_BLOCKED',
      blockedZone: zone,
      distanceCm,
      recoveryDirections: [...info.recoveryDirs],
      message: `${info.message} (${distanceCm.toFixed(1)} cm)`,
    };
  }

  return {
    allowed: true,
    reasonCode: null,
    blockedZone: null,
    distanceCm,
    recoveryDirections: [],
    message: '',
  };
}
