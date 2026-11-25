import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createAmrSimulator } from '../src/core/simulator.js';
import { DIRECTION_TO_ZONE, computeSafetyStatus } from '../src/core/safety.js';
import { LIDAR_MIN_RANGE_M, computeZonesFromRanges } from '../src/core/lidar.js';
import { ROBOT_RADIUS_M } from '../src/core/kinematics.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const CORE_DIR = path.resolve(__dirname, '../src/core');

test('src/core modules contain zero DOM or browser global references', () => {
  const files = fs.readdirSync(CORE_DIR).filter((f) => f.endsWith('.js'));
  const forbidden = /\b(window|document|localStorage|sessionStorage|XMLHttpRequest|fetch)\b/;
  for (const file of files) {
    const src = fs.readFileSync(path.join(CORE_DIR, file), 'utf8');
    const match = src.match(forbidden);
    assert.equal(match, null, `Found forbidden browser global in src/core/${file}: ${match?.[0]}`);
  }
});

test('state machine handles DISARMED, ARMED, FAULT -> CLEAR_FAULT -> ARM', () => {
  const sim = createAmrSimulator({ scenarioId: 'clear' });
  assert.equal(sim.getState().armed, false);

  // Refuses drive while disarmed
  const blocked = sim.startHold('F');
  assert.equal(blocked.accepted, false);
  assert.equal(blocked.safety.reasonCode, 'DISARMED');

  // Inject fault -> ARM rejected -> CLEAR_FAULT -> ARM succeeds
  sim.injectFault('ENCODER_TIMEOUT');
  assert.equal(sim.getState().fault, 'ENCODER_TIMEOUT');
  const armRejected = sim.arm();
  assert.equal(armRejected.status, 'REJECTED');
  assert.equal(armRejected.reason, 'ENCODER_TIMEOUT');

  const cleared = sim.clearFault();
  assert.equal(cleared.status, 'SENT');
  assert.equal(sim.getState().fault, null);

  const armOk = sim.arm();
  assert.equal(armOk.status, 'SENT');
  assert.equal(sim.getState().armed, true);
});

test('hold-to-drive repeats every 150 ms and release-to-stop emits V 0.000 0.000', () => {
  const sim = createAmrSimulator({ scenarioId: 'clear' });
  sim.arm();
  const startY = sim.getState().pose.y;
  const seq0 = sim.getState().seq;

  const held = sim.startHold('F');
  assert.equal(held.accepted, true);
  assert.equal(sim.getState().seq, seq0 + 1);

  sim.step(500);
  const moved = sim.getState();
  assert.ok(moved.pose.y > startY + 0.05);
  assert.ok(moved.seq >= seq0 + 4); // initial + >=3 repeats at 150ms

  const rel = sim.releaseHold();
  assert.equal(rel.cmdVal, 'V 0.000 0.000');
  assert.equal(sim.getState().cmdV, 0);
  assert.equal(sim.getState().wheelRpms.l_rpm, 0);
  assert.equal(sim.getState().wheelRpms.r_rpm, 0);
  const lastLog = sim.getCommandLog().at(-1);
  assert.ok(lastLog.url.includes('V%200.000%200.000'));
});

test('8-zone < 40 cm interlock blocks matching direction and allows opposite recovery', () => {
  const sim = createAmrSimulator({ scenarioId: 'clear' });
  sim.arm();

  const angleForZone = {
    F: 0,
    FL: 45,
    L: 90,
    BL: 135,
    B: 180,
    BR: 225,
    R: 270,
    FR: 315,
  };
  const oppositeDir = {
    F: 'B',
    B: 'F',
    L: 'R',
    R: 'L',
    FL: 'BR',
    FR: 'BL',
    BL: 'FR',
    BR: 'FL',
  };

  for (const [dir, angleDeg] of Object.entries(angleForZone)) {
    sim.resetPose();
    sim.setCustomObstaclePolar({ distanceM: 0.30, angleDeg, radiusM: 0.15 });
    const zones = sim.getState().zones;
    const zoneKey = DIRECTION_TO_ZONE[dir];
    assert.ok(zones[zoneKey] < 40.0, `Expected ${zoneKey} < 40 cm, got ${zones[zoneKey]}`);

    const blocked = sim.startHold(dir);
    assert.equal(blocked.accepted, false, `Expected ${dir} to be blocked`);
    assert.equal(blocked.safety.reasonCode, 'ZONE_BLOCKED');
    assert.equal(blocked.safety.blockedZone, zoneKey);

    const opp = sim.startHold(oppositeDir[dir]);
    assert.equal(opp.accepted, true, `Expected opposite ${oppositeDir[dir]} to be allowed`);
    sim.releaseHold();
  }

  // Exact boundary >= 40.0 cm allows drive
  sim.resetPose();
  sim.setCustomObstaclePolar({ distanceM: 0.42, angleDeg: 0, radiusM: 0.15 });
  assert.ok(sim.getState().zones.front >= 40.0);
  assert.equal(sim.startHold('F').accepted, true);
  sim.releaseHold();
});

test('dynamic_moving scenario triggers mid-motion < 40 cm auto-halt and preserves ARM for recovery', () => {
  const sim = createAmrSimulator({ scenarioId: 'dynamic_moving' });
  sim.arm();

  const started = sim.startHold('F');
  assert.equal(started.accepted, true);

  // Advance until dynamic obstacle crosses < 40 cm ahead
  for (let i = 0; i < 40; i++) {
    sim.step(100);
    if (sim.getState().autoHaltEvent?.triggered) break;
  }

  const st = sim.getState();
  assert.equal(st.autoHaltEvent?.triggered, true);
  assert.equal(st.autoHaltEvent.blockedZone, 'front');
  assert.ok(st.autoHaltEvent.distanceCm < 40.0);
  assert.equal(st.cmdV, 0);
  assert.equal(st.cmdW, 0);
  assert.equal(st.activeDirection, null);
  // Kept armed so operator can reverse immediately
  assert.equal(st.armed, true);

  const lastLog = sim.getCommandLog().at(-1);
  assert.ok(lastLog.url.includes('V%200.000%200.000'));

  const recovery = sim.startHold('B');
  assert.equal(recovery.accepted, true);
  sim.releaseHold();
});

test('telemetry watchdog (> 1500 ms) and LiDAR loss lock driving', () => {
  const sim = createAmrSimulator({ scenarioId: 'clear' });
  sim.arm();
  sim.startHold('F');

  sim.setTelemetryStall(true);
  sim.step(1700);
  const stalled = sim.getState();
  assert.equal(stalled.telemetryStale, true);
  assert.equal(stalled.armed, false);
  assert.equal(stalled.cmdV, 0);
  assert.equal(sim.startHold('F').safety.reasonCode, 'DISARMED');

  sim.setTelemetryStall(false);
  sim.step(100);
  sim.arm();
  sim.setLidarConnected(false);
  assert.equal(sim.startHold('F').safety.reasonCode, 'LIDAR_UNHEALTHY');
});

test('REST endpoints return valid telemetry, map_full, auto_config, pid, reset_map, and 404', () => {
  const sim = createAmrSimulator({ scenarioId: 'clear' });
  const tele = sim.request('/api/telemetry');
  assert.equal(tele.statusCode, 200);
  assert.equal(tele.body.ranges.length, 360);
  assert.equal(typeof tele.body.zones.front, 'number');
  assert.equal(tele.body.map_info.robot_pose.length, 3);

  const map = sim.request('/api/map_full');
  assert.equal(map.statusCode, 200);
  assert.equal(map.body.data.length, map.body.w * map.body.h);

  const speed = sim.request('/api/auto_config?speed=90&turn=90');
  assert.equal(speed.statusCode, 200);
  assert.equal(sim.getState().desiredRpm, 90);

  const pid = sim.request('/api/pid?kp=0.6&ki=1.2&kd=0.02');
  assert.equal(pid.statusCode, 200);
  assert.deepEqual(sim.getState().pid, { kp: 0.6, ki: 1.2, kd: 0.02 });

  const reset = sim.request('/api/reset_map');
  assert.equal(reset.statusCode, 200);

  const notFound = sim.request('/api/unknown');
  assert.equal(notFound.statusCode, 404);
});

test('0.05 m close-range / overlapping obstacles yield DANGER_STOP and block driving', () => {
  // 1. Direct 0.05 m ray reading yields 5.0 cm (never 999.0 cm SAFE)
  const ranges = new Array(360).fill(2.5);
  ranges[0] = LIDAR_MIN_RANGE_M;
  const z = computeZonesFromRanges(ranges, 0);
  assert.equal(z.zones.front, 5.0);
  assert.equal(z.min_m, 0.05);

  // 2. Close-range setCustomObstacleWorld pushes out of robot footprint & blocks F
  const sim = createAmrSimulator({ scenarioId: 'clear' });
  sim.arm();

  const obs1 = sim.setCustomObstacleWorld({ x: 0, y: 0.17, radiusM: 0.14 });
  const st1 = sim.getState();
  const dist1 = Math.hypot(obs1.x - st1.pose.x, obs1.y - st1.pose.y);
  assert.ok(dist1 >= ROBOT_RADIUS_M + 0.14 + 0.01 - 1e-3, `Expected non-overlapping clearance, got ${dist1}`);
  assert.ok(st1.zones.front < 40.0, `Expected front < 40 cm, got ${st1.zones.front}`);
  assert.equal(st1.safetyStatus, 'DANGER_STOP');
  const blocked1 = sim.startHold('F');
  assert.equal(blocked1.accepted, false);
  assert.equal(blocked1.safety.reasonCode, 'ZONE_BLOCKED');

  // 3. Exact center overlap click (0, 0) pushes along heading (pose.th = pi/2 => +Y) and blocks F
  sim.clearCustomObstacle();
  const obs2 = sim.setCustomObstacleWorld({ x: 0, y: 0, radiusM: 0.14 });
  const st2 = sim.getState();
  assert.ok(obs2.y > st2.pose.y + ROBOT_RADIUS_M + 0.14, 'Center click should push ahead along pose.th');
  assert.ok(st2.zones.front < 40.0);
  assert.equal(st2.safetyStatus, 'DANGER_STOP');
  assert.equal(sim.startHold('F').accepted, false);
});

test('setCustomObstacleWorld synchronously halts a moving robot when placed < 40 cm ahead', () => {
  const sim = createAmrSimulator({ scenarioId: 'clear' });
  sim.arm();
  const held = sim.startHold('F');
  assert.equal(held.accepted, true);
  sim.step(100);
  assert.equal(sim.getState().activeDirection, 'F');

  const pose = sim.getState().pose;
  sim.setCustomObstacleWorld({ x: pose.x, y: pose.y + 0.35, radiusM: 0.14 });
  // Immediately halted before any subsequent step()
  const after = sim.getState();
  assert.equal(after.activeDirection, null);
  assert.equal(after.cmdV, 0);
  assert.equal(after.autoHaltEvent?.triggered, true);
  assert.equal(after.armed, true);
  assert.ok(sim.getCommandLog().at(-1).url.includes('V%200.000%200.000'));
});

test('dynamic_moving obstacle approaching a parked robot resolves collision without ghosting to SAFE', () => {
  const sim = createAmrSimulator({ scenarioId: 'dynamic_moving' });
  sim.arm();
  sim.startHold('F');

  // Drive toward obstacle until near, then release hold to stay stationary
  for (let i = 0; i < 30; i++) {
    sim.step(50);
    if (sim.getState().zones.front < 60) break;
  }
  sim.releaseHold();

  // Step until dynamic obstacle reaches its minY=-0.35 turnaround against the parked robot
  let reachedClosest = false;
  for (let i = 0; i < 60; i++) {
    sim.step(50);
    const st = sim.getState();
    const dyn = st.world.obstacles.find((o) => o.id === 'obs-dynamic');
    const centerDist = Math.hypot(dyn.x - st.pose.x, dyn.y - st.pose.y);
    assert.ok(
      centerDist >= ROBOT_RADIUS_M + dyn.r - 1e-4,
      `Dynamic obstacle penetrated stationary robot: centerDist=${centerDist}`
    );
    assert.notEqual(st.zones.front, 999.0);
    if (dyn.vy > 0) {
      reachedClosest = true;
      break;
    }
  }

  assert.equal(reachedClosest, true, 'Expected dynamic obstacle to reach minY turnaround');
  const closestSt = sim.getState();
  assert.ok(closestSt.zones.front < 40.0, `Expected front < 40 cm at closest approach, got ${closestSt.zones.front}`);
  assert.equal(closestSt.safetyStatus, 'DANGER_STOP');
  assert.equal(sim.startHold('F').accepted, false);
});

test('ARM/STOP FSM rejects duplicate ARM (422) and latches STOP until next ARM', () => {
  const sim = createAmrSimulator({ scenarioId: 'clear' });

  const arm1 = sim.request('/api/cmd?val=ARM');
  assert.equal(arm1.statusCode, 200);
  assert.equal(arm1.body.status, 'SENT');
  assert.equal(arm1.body.reason, 'ARM_CONFIRMED');
  assert.equal(arm1.body.armed, true);
  assert.equal(arm1.body.latched, false);

  const dupArm = sim.request('/api/cmd?val=ARM');
  assert.equal(dupArm.statusCode, 422);
  assert.equal(dupArm.body.status, 'FAILED');
  assert.equal(dupArm.body.reason, 'PREFLIGHT_NOT_READY');

  const stopRes = sim.request('/api/cmd?val=STOP');
  assert.equal(stopRes.statusCode, 200);
  assert.equal(stopRes.body.status, 'SENT');
  assert.equal(stopRes.body.reason, 'STOP_CONFIRMED');
  assert.equal(stopRes.body.armed, false);
  assert.equal(stopRes.body.latched, true);
  assert.equal(sim.getTelemetry().control.latched, true);

  const arm2 = sim.request('/api/cmd?val=ARM');
  assert.equal(arm2.statusCode, 200);
  assert.equal(arm2.body.status, 'SENT');
  assert.equal(arm2.body.reason, 'ARM_CONFIRMED');
  assert.equal(arm2.body.armed, true);
  assert.equal(arm2.body.latched, false);
  assert.equal(sim.getTelemetry().control.latched, false);
});

test('computeSafetyStatus covers all 8 zones < 40 cm and simulator validates PID & V tokens', () => {
  assert.equal(
    computeSafetyStatus(
      { front: 200, back: 38.0, left: 200, right: 200, front_left: 200, front_right: 200, back_left: 200, back_right: 200 },
      0.38,
    ),
    'DANGER_STOP',
  );

  const sim = createAmrSimulator({ scenarioId: 'clear' });
  const badPid = sim.request('/api/pid?kp=-1&ki=0&kd=0');
  assert.equal(badPid.statusCode, 400);
  assert.equal(badPid.body.error, 'INVALID_PID');

  const bareV = sim.request('/api/cmd?val=V');
  assert.equal(bareV.body.status, 'REJECTED');
  assert.equal(bareV.body.reason, 'INVALID_VELOCITY');

  const extraV = sim.request('/api/cmd?val=V%200.1%200.2%20extra');
  assert.equal(extraV.body.status, 'REJECTED');
  assert.equal(extraV.body.reason, 'INVALID_VELOCITY');
});
