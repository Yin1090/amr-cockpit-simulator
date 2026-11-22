import test from 'node:test';
import assert from 'node:assert/strict';
import {
  WHEEL_RADIUS_M,
  HALF_WHEELBASE_M,
  RPM_TO_MPS,
  RPM_MIN,
  RPM_MAX,
  RPM_DEFAULT,
  RPM_STEP,
  MAX_OMEGA_RAD_S,
  DIAG_FACTOR,
  DIRECTIONS,
  clampRpm,
  stepRpm,
  computeBaseTwist,
  twistToWheelRpms,
  formatVelocityCommand,
  computeDirectionCommand,
  integrateUnicyclePose,
} from '../src/core/kinematics.js';

test('kinematic constants match AMR hardware contract', () => {
  assert.equal(WHEEL_RADIUS_M, 0.0366);
  assert.equal(HALF_WHEELBASE_M, 0.096);
  assert.ok(Math.abs(RPM_TO_MPS - (2 * Math.PI * 0.0366) / 60) < 1e-12);
  assert.ok(Math.abs(RPM_TO_MPS - 0.003832743) < 1e-6);
  assert.equal(RPM_MIN, 40);
  assert.equal(RPM_MAX, 135);
  assert.equal(RPM_DEFAULT, 40);
  assert.equal(RPM_STEP, 10);
  assert.equal(MAX_OMEGA_RAD_S, 1.80);
  assert.ok(Math.abs(DIAG_FACTOR - Math.SQRT1_2) < 1e-12);
  assert.deepEqual(DIRECTIONS, ['F', 'B', 'L', 'R', 'FL', 'FR', 'BL', 'BR']);
});

test('clampRpm and stepRpm enforce [40, 135] bounds and ±10 steps', () => {
  assert.equal(clampRpm(10), 40);
  assert.equal(clampRpm(40), 40);
  assert.equal(clampRpm(135), 135);
  assert.equal(clampRpm(200), 135);
  assert.equal(clampRpm(NaN), 40);
  assert.equal(clampRpm(64.6), 65);

  assert.equal(stepRpm(40, -1), 40);
  assert.equal(stepRpm(40, 1), 50);
  assert.equal(stepRpm(130, 1), 135);
  assert.equal(stepRpm(135, 1), 135);
});

test('computeBaseTwist computes exact v and capped w', () => {
  const t40 = computeBaseTwist(40);
  assert.ok(Math.abs(t40.v - 0.1533097) < 1e-5);
  assert.ok(Math.abs(t40.w - 0.958186) < 1e-4);

  const t65 = computeBaseTwist(65);
  assert.ok(Math.abs(t65.v - 0.249128) < 1e-5);
  assert.ok(Math.abs(t65.w - 1.55705) < 1e-4);

  const t135 = computeBaseTwist(135);
  assert.ok(Math.abs(t135.v - 0.51742) < 1e-5);
  assert.equal(t135.w, 1.80);
});

test('8-direction commands, diagonal scaling, inverse wheel RPMs, and STOP format', () => {
  const f = computeDirectionCommand('F', 40);
  assert.ok(f.v > 0 && f.w === 0);
  assert.ok(Math.abs(f.l_rpm - 40) < 1e-6 && Math.abs(f.r_rpm - 40) < 1e-6);
  assert.equal(f.cmdVal, 'V 0.153 0.000');

  const b = computeDirectionCommand('B', 40);
  assert.ok(b.v < 0 && b.w === 0);
  assert.ok(Math.abs(b.l_rpm + 40) < 1e-6 && Math.abs(b.r_rpm + 40) < 1e-6);

  const l = computeDirectionCommand('L', 40);
  assert.equal(l.v, 0);
  assert.ok(l.w > 0);
  assert.ok(l.l_rpm < 0 && l.r_rpm > 0);

  const r = computeDirectionCommand('R', 40);
  assert.equal(r.v, 0);
  assert.ok(r.w < 0);
  assert.ok(r.l_rpm > 0 && r.r_rpm < 0);

  const fl = computeDirectionCommand('FL', 40);
  assert.ok(Math.abs(fl.v - f.v * Math.SQRT1_2) < 1e-9);
  assert.ok(Math.abs(fl.w - l.w * Math.SQRT1_2) < 1e-9);

  const fr = computeDirectionCommand('FR', 40);
  assert.ok(fr.v > 0 && fr.w < 0);
  const bl = computeDirectionCommand('BL', 40);
  assert.ok(bl.v < 0 && bl.w > 0);
  const br = computeDirectionCommand('BR', 40);
  assert.ok(br.v < 0 && br.w < 0);

  const stop = computeDirectionCommand('STOP', 100);
  assert.equal(stop.cmdVal, 'V 0.000 0.000');
  assert.equal(stop.l_rpm, 0);
  assert.equal(stop.r_rpm, 0);

  assert.equal(formatVelocityCommand(-1e-9, -1e-9), 'V 0.000 0.000');
  assert.throws(() => computeDirectionCommand('INVALID', 40), RangeError);

  const inv = twistToWheelRpms(fl.v, fl.w);
  assert.ok(Math.abs(inv.l_rpm - fl.l_rpm) < 1e-9);
  assert.ok(Math.abs(inv.r_rpm - fl.r_rpm) < 1e-9);
});

test('integrateUnicyclePose integrates straight lines and ICC circular arcs accurately', () => {
  const p0 = { x: 0, y: 0, th: Math.PI / 2 };
  const straight = integrateUnicyclePose(p0, 0.5, 0, 1.0);
  assert.ok(Math.abs(straight.x) < 1e-9);
  assert.ok(Math.abs(straight.y - 0.5) < 1e-9);
  assert.ok(Math.abs(straight.th - Math.PI / 2) < 1e-9);

  // Full 2*PI circle at v=1, w=1 over dt=2*PI returns to origin
  const circle = integrateUnicyclePose({ x: 0, y: 0, th: 0 }, 1.0, 1.0, 2 * Math.PI);
  assert.ok(Math.abs(circle.x) < 1e-6);
  assert.ok(Math.abs(circle.y) < 1e-6);
  assert.ok(Math.abs(circle.th) < 1e-6);
});
