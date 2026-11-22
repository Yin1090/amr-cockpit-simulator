import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import {
  computeDirectionCommand,
  WHEEL_RADIUS_M,
  HALF_WHEELBASE_M,
} from '../src/core/kinematics.js';
import {
  createOccupancyGrid,
  computeZonesFromRanges,
} from '../src/core/lidar.js';
import {
  computeSafetyStatus,
  evaluateDriveSafety,
} from '../src/core/safety.js';
import { createAmrSimulator } from '../src/core/simulator.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT_DIR = path.resolve(__dirname, '..');
const DATA_DIR = path.join(ROOT_DIR, 'data');

function readJson(relPath) {
  return JSON.parse(fs.readFileSync(path.join(DATA_DIR, relPath), 'utf8'));
}

test('hardware_stack_reference.json matches Topic 2 calibrated stack and preserves simulator constants', () => {
  const hw = readJson('hardware_stack_reference.json');
  assert.equal(hw.kinematics.wheel_radius_m, WHEEL_RADIUS_M);
  assert.equal(hw.kinematics.simulator_half_wheelbase_m, HALF_WHEELBASE_M);
  assert.equal(hw.kinematics.physical_track_width_m, 0.1875);
  assert.equal(hw.kinematics.physical_half_wheelbase_m, 0.09375);

  assert.equal(hw.onboard_compute.sbc, 'Raspberry Pi 4');
  assert.equal(hw.onboard_compute.ros_distro, 'ROS 2 Jazzy');
  assert.deepEqual(hw.onboard_compute.ports, {
    rest_api: 8080,
    mjpeg_camera: 8081,
    mjpeg_resolution: '640x480',
    auth_proxy: 8090,
  });

  assert.equal(hw.mcu_and_sensors.usb_serial, 'CH340');
  assert.equal(hw.mcu_and_sensors.baud_rate, 115200);
  assert.equal(hw.mcu_and_sensors.motor_driver.address, '0x42');
  assert.equal(hw.mcu_and_sensors.encoders.ppr, 937);
  assert.equal(hw.mcu_and_sensors.lidar.model, 'RPLiDAR C1');
  assert.equal(hw.mcu_and_sensors.lidar.scan_rate_hz, 10.0);
});

test('telemetry_latency_n40.json satisfies n=40 exact latency distribution and p95 < 150 ms', () => {
  const latency = readJson('telemetry_latency_n40.json');
  const samples = latency.samples_ms;
  assert.equal(samples.length, 40);

  const sorted = [...samples].sort((a, b) => a - b);
  const min = sorted[0];
  const max = sorted[sorted.length - 1];
  const median = (sorted[19] + sorted[20]) / 2;
  const mean = Number((samples.reduce((acc, v) => acc + v, 0) / samples.length).toFixed(4));
  const p95Idx = Math.ceil(0.95 * sorted.length) - 1;
  const p95 = sorted[p95Idx];

  assert.equal(min, 26.4);
  assert.equal(median, 36.0);
  assert.equal(mean, 40.9);
  assert.equal(p95, 60.3);
  assert.ok(p95 < latency.target_p95_ms);
  assert.equal(max, 163.9);
});

test('motor_tracking_benchmarks.json exercises kinematics, safety interlock, and simulator state machine', () => {
  const data = readJson('motor_tracking_benchmarks.json');
  assert.equal(data.scenarios.length, 6);
  const byId = Object.fromEntries(data.scenarios.map((s) => [s.id, s]));

  // Unarmed 40 RPM -> held at 0/0 RPM
  const sim = createAmrSimulator({ scenarioId: 'clear' });
  const unarmedCheck = evaluateDriveSafety({
    armed: byId.unarmed_40rpm.armed,
    fault: null,
    stopPending: false,
    lidar: { connected: true, age_s: 0.02 },
    telemetryStale: false,
    direction: 'F',
    zones: { front: byId.unarmed_40rpm.lidar_min_m * 100 },
  });
  assert.equal(unarmedCheck.allowed, false);
  assert.equal(unarmedCheck.reasonCode, 'DISARMED');
  assert.equal(sim.startHold('F').accepted, false);

  // Forward 40 RPM -> v ~ 0.153 m/s
  const fwdCmd = computeDirectionCommand('F', byId.forward_40rpm.setpoint_rpm);
  assert.equal(fwdCmd.l_rpm, 40);
  assert.equal(fwdCmd.r_rpm, 40);
  assert.ok(Math.abs(fwdCmd.v - 0.1533) < 0.001);

  // Reverse 40 RPM -> -40/-40 RPM and SAFE status
  const revCmd = computeDirectionCommand('B', byId.reverse_40rpm.setpoint_rpm);
  assert.equal(revCmd.l_rpm, -40);
  assert.equal(revCmd.r_rpm, -40);
  assert.equal(computeSafetyStatus({ front: byId.reverse_40rpm.lidar_min_m * 100 }, byId.reverse_40rpm.lidar_min_m), 'SAFE');

  // Max 135 RPM closed-loop tracking error < 3%
  const maxScenario = byId.max_135rpm;
  const errL = Math.abs(maxScenario.measured_l_rpm - maxScenario.setpoint_rpm) / maxScenario.setpoint_rpm;
  const errR = Math.abs(maxScenario.measured_r_rpm - maxScenario.setpoint_rpm) / maxScenario.setpoint_rpm;
  assert.ok(errL < 0.03 && errR < 0.03);

  // Obstacle lock 135 RPM @ 0.17 m -> 0/0 RPM and blocks F while allowing B
  sim.arm();
  sim.setRpm(135);
  sim.setCustomObstaclePolar({ distanceM: byId.obstacle_lock_135rpm.lidar_min_m, angleDeg: 0, radiusM: 0.08 });
  const locked = sim.startHold('F');
  assert.equal(locked.accepted, false);
  assert.equal(locked.safety.reasonCode, 'ZONE_BLOCKED');
  assert.equal(sim.getState().wheelRpms.l_rpm, 0);
  assert.equal(sim.getState().wheelRpms.r_rpm, 0);
  assert.equal(computeSafetyStatus(sim.getState().zones, byId.obstacle_lock_135rpm.lidar_min_m), 'DANGER_STOP');
  assert.equal(sim.startHold('B').accepted, true);
  sim.releaseHold();
});

test('slam_grid_snapshot.json builds 101x99 occupancy grid and 674-ray LiDAR zone check', () => {
  const snap = readJson('slam_grid_snapshot.json');
  assert.equal(snap.lidar.scan_rate_hz, 10.0);
  assert.equal(snap.lidar.scan_pts, 674);
  assert.equal(snap.lidar.age_s, 0.018);
  assert.equal(snap.grid.w, 101);
  assert.equal(snap.grid.h, 99);
  assert.equal(snap.grid.res, 0.05);
  assert.equal(snap.grid.data.length, 101 * 99);

  const grid = createOccupancyGrid({
    w: snap.grid.w,
    h: snap.grid.h,
    res: snap.grid.res,
    ox: snap.grid.ox,
    oy: snap.grid.oy,
    yaw: snap.grid.yaw,
  });
  assert.equal(grid.data.length, 9999);

  // Construct 674-ray scan with nearest obstacle 0.62 m at 268 deg (right sector)
  const ranges = Array.from({ length: snap.lidar.scan_pts }, (_, idx) => {
    const deg = (idx * 360) / snap.lidar.scan_pts;
    return Math.abs(deg - snap.lidar.nearest_obstacle_deg) < 1.0 ? snap.lidar.nearest_obstacle_m : 2.1;
  });
  const binned = computeZonesFromRanges(ranges);
  assert.equal(binned.zones.right, 62.0);
  assert.equal(binned.min_m, 0.62);
  assert.equal(computeSafetyStatus(binned.zones, binned.min_m), 'SAFE');
});

test('README.md provides complete bilingual EN + VI documentation', () => {
  const readme = fs.readFileSync(path.join(ROOT_DIR, 'README.md'), 'utf8');
  for (const token of ['Overview', 'Kinematics', 'npm run serve', 'npm test', 'data/']) {
    assert.ok(readme.includes(token), `Missing English README token: ${token}`);
  }
  for (const token of ['Tổng quan', 'Kiến trúc', 'Bộ dữ liệu', 'Cài đặt', 'Kiểm thử tự động']) {
    assert.ok(readme.includes(token), `Missing Vietnamese README token: ${token}`);
  }
});
