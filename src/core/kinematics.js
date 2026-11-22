/**
 * Differential-drive kinematics and velocity command math for the AMR cockpit.
 * Pure ESM module — zero DOM or browser global dependencies.
 */

export const WHEEL_RADIUS_M = 0.0366;
export const HALF_WHEELBASE_M = 0.096;
export const ROBOT_RADIUS_M = 0.14;
export const RPM_TO_MPS = (2 * Math.PI * WHEEL_RADIUS_M) / 60;

export const RPM_MIN = 40;
export const RPM_MAX = 135;
export const RPM_DEFAULT = 40;
export const RPM_STEP = 10;

export const MAX_OMEGA_RAD_S = 1.80;
export const ANGULAR_GAIN = 0.60;
export const DIAG_FACTOR = Math.SQRT1_2;

export const DIRECTIONS = Object.freeze(['F', 'B', 'L', 'R', 'FL', 'FR', 'BL', 'BR']);

export const DIRECTION_VECTORS = Object.freeze({
  F: Object.freeze({ lin: 1, ang: 0, diag: false, zone: 'front' }),
  B: Object.freeze({ lin: -1, ang: 0, diag: false, zone: 'back' }),
  L: Object.freeze({ lin: 0, ang: 1, diag: false, zone: 'left' }),
  R: Object.freeze({ lin: 0, ang: -1, diag: false, zone: 'right' }),
  FL: Object.freeze({ lin: 1, ang: 1, diag: true, zone: 'front_left' }),
  FR: Object.freeze({ lin: 1, ang: -1, diag: true, zone: 'front_right' }),
  BL: Object.freeze({ lin: -1, ang: 1, diag: true, zone: 'back_left' }),
  BR: Object.freeze({ lin: -1, ang: -1, diag: true, zone: 'back_right' }),
});

/**
 * Clamps a wheel RPM target to [40, 135], defaulting non-finite values to 40.
 * @param {number} rpm
 * @returns {number}
 */
export function clampRpm(rpm) {
  const num = Number(rpm);
  if (!Number.isFinite(num)) {
    return RPM_DEFAULT;
  }
  return Math.min(RPM_MAX, Math.max(RPM_MIN, Math.round(num)));
}

/**
 * Steps RPM by deltaSteps * 10 RPM within [40, 135].
 * @param {number} currentRpm
 * @param {number} [deltaSteps=1]
 * @returns {number}
 */
export function stepRpm(currentRpm, deltaSteps = 1) {
  const base = clampRpm(currentRpm);
  return clampRpm(base + deltaSteps * RPM_STEP);
}

/**
 * Computes base linear (m/s) and angular (rad/s) speeds for a given RPM.
 * @param {number} rpm
 * @returns {{ rpm: number, v: number, w: number }}
 */
export function computeBaseTwist(rpm) {
  const safeRpm = clampRpm(rpm);
  const v = safeRpm * RPM_TO_MPS;
  const w = Math.min(MAX_OMEGA_RAD_S, (ANGULAR_GAIN * v) / HALF_WHEELBASE_M);
  return { rpm: safeRpm, v, w };
}

/**
 * Inverse differential-drive kinematics from body twist (v, w) to wheel RPMs.
 * @param {number} v
 * @param {number} w
 * @returns {{ l_rpm: number, r_rpm: number }}
 */
export function twistToWheelRpms(v, w) {
  const l_rpm = (v - w * HALF_WHEELBASE_M) / RPM_TO_MPS;
  const r_rpm = (v + w * HALF_WHEELBASE_M) / RPM_TO_MPS;
  return { l_rpm, r_rpm };
}

/**
 * Formats a body velocity command string without negative zero artifacts.
 * @param {number} v
 * @param {number} w
 * @returns {string}
 */
export function formatVelocityCommand(v, w) {
  const vn = Math.abs(v) < 1e-6 ? 0 : v;
  const wn = Math.abs(w) < 1e-6 ? 0 : w;
  return `V ${vn.toFixed(3)} ${wn.toFixed(3)}`;
}

/**
 * Computes full velocity command, wheel RPMs, and wire payload for an 8-dir or STOP input.
 * @param {string|null} direction
 * @param {number} rpm
 * @returns {{ direction: string, v: number, w: number, l_rpm: number, r_rpm: number, cmdVal: string }}
 */
export function computeDirectionCommand(direction, rpm) {
  if (!direction || direction === 'STOP') {
    return {
      direction: 'STOP',
      v: 0,
      w: 0,
      l_rpm: 0,
      r_rpm: 0,
      cmdVal: 'V 0.000 0.000',
    };
  }
  const vec = DIRECTION_VECTORS[direction];
  if (!vec) {
    throw new RangeError(`Unknown drive direction: ${String(direction)}`);
  }
  const { v: vBase, w: wBase } = computeBaseTwist(rpm);
  const scale = vec.diag ? DIAG_FACTOR : 1;
  const v = vec.lin * scale * vBase;
  const w = vec.ang * scale * wBase;
  const { l_rpm, r_rpm } = twistToWheelRpms(v, w);
  const cmdVal = formatVelocityCommand(v, w);
  return { direction, v, w, l_rpm, r_rpm, cmdVal };
}

/**
 * Normalizes an angle in radians to (-PI, PI].
 * @param {number} theta
 * @returns {number}
 */
export function normalizeAngle(theta) {
  const a = Math.atan2(Math.sin(theta), Math.cos(theta));
  return a <= -Math.PI ? Math.PI : a;
}

/**
 * Exact ICC arc integration for a unicycle pose over dtSec seconds.
 * @param {{ x: number, y: number, th: number }} pose
 * @param {number} v
 * @param {number} w
 * @param {number} dtSec
 * @returns {{ x: number, y: number, th: number }}
 */
export function integrateUnicyclePose(pose, v, w, dtSec) {
  if (!Number.isFinite(dtSec) || dtSec <= 0) {
    return { x: pose.x, y: pose.y, th: normalizeAngle(pose.th) };
  }
  if (Math.abs(w) < 1e-6) {
    return {
      x: pose.x + v * dtSec * Math.cos(pose.th),
      y: pose.y + v * dtSec * Math.sin(pose.th),
      th: normalizeAngle(pose.th),
    };
  }
  const dth = w * dtSec;
  const r = v / w;
  return {
    x: pose.x + r * (Math.sin(pose.th + dth) - Math.sin(pose.th)),
    y: pose.y - r * (Math.cos(pose.th + dth) - Math.cos(pose.th)),
    th: normalizeAngle(pose.th + dth),
  };
}
