import test from 'node:test';
import assert from 'node:assert/strict';
import {
  LIDAR_MIN_RANGE_M,
  raycastWorld,
  angleDegToZone,
  computeZonesFromRanges,
  resolveRobotCollision,
  createOccupancyGrid,
  updateOccupancyGrid,
  resetOccupancyGrid,
  serializeOccupancyGrid,
} from '../src/core/lidar.js';
import { createWorldFromScenario } from '../src/core/scenarios.js';

test('angleDegToZone maps exact CCW 45-degree sectors', () => {
  assert.equal(angleDegToZone(0), 'front');
  assert.equal(angleDegToZone(22.4), 'front');
  assert.equal(angleDegToZone(337.5), 'front');
  assert.equal(angleDegToZone(22.5), 'front_left');
  assert.equal(angleDegToZone(45), 'front_left');
  assert.equal(angleDegToZone(67.5), 'left');
  assert.equal(angleDegToZone(90), 'left');
  assert.equal(angleDegToZone(135), 'back_left');
  assert.equal(angleDegToZone(180), 'back');
  assert.equal(angleDegToZone(225), 'back_right');
  assert.equal(angleDegToZone(270), 'right');
  assert.equal(angleDegToZone(315), 'front_right');
});

test('raycastWorld & computeZonesFromRanges detect front_obstacle < 40 cm accurately', () => {
  const world = createWorldFromScenario('front_obstacle');
  const pose = world.initialPose;
  const scan = raycastWorld(pose, world);
  assert.equal(scan.ranges.length, 360);
  // Front box is centered at (0, 0.43) with h=0.20 => front face at y=0.33 m (33.0 cm)
  assert.ok(Math.abs(scan.ranges[0] - 0.33) < 0.01);

  const { zones, min_m } = computeZonesFromRanges(scan.ranges, scan.min_ang);
  assert.ok(Math.abs(zones.front - 33.0) < 0.5);
  assert.ok(Math.abs(min_m - 0.33) < 0.01);
  assert.equal(zones.back, 250.0);
  assert.equal(zones.left, 250.0);
  assert.equal(zones.right, 250.0);
});

test('computeZonesFromRanges returns 999.0 cm when obstacles exceed 3.2 m', () => {
  const ranges = new Array(360).fill(4.5);
  const { zones } = computeZonesFromRanges(ranges, 0, 3.2);
  for (const val of Object.values(zones)) {
    assert.equal(val, 999.0);
  }
});

test('resolveRobotCollision prevents wall and obstacle penetration', () => {
  const world = createWorldFromScenario('front_obstacle');
  // Attempt to move straight into box at (0, 0.43)
  const resolved = resolveRobotCollision({ x: 0, y: 0.35, th: Math.PI / 2 }, world, 0.14);
  assert.ok(resolved.collided);
  assert.ok(resolved.y <= 0.33 - 0.14 + 1e-6);

  const wallResolved = resolveRobotCollision({ x: 10, y: -10, th: 0 }, world, 0.14);
  assert.ok(wallResolved.collided);
  assert.ok(wallResolved.x <= 2.5 - 0.14 + 1e-6);
  assert.ok(wallResolved.y >= -2.5 + 0.14 - 1e-6);
});

test('SLAM occupancy grid traces free/occupied cells and resets cleanly', () => {
  const world = createWorldFromScenario('front_obstacle');
  const grid = createOccupancyGrid();
  const scan = raycastWorld(world.initialPose, world);
  updateOccupancyGrid(grid, world.initialPose, scan.ranges, scan.hits, scan.min_ang);

  const wire = serializeOccupancyGrid(grid);
  assert.equal(wire.w, 120);
  assert.equal(wire.h, 120);
  assert.equal(wire.res, 0.05);
  assert.equal(wire.data.length, wire.w * wire.h);
  assert.ok(wire.data.includes(0));
  assert.ok(wire.data.includes(100));

  resetOccupancyGrid(grid);
  const resetWire = serializeOccupancyGrid(grid);
  assert.ok(resetWire.data.every((v) => v === -1));
});

test('computeZonesFromRanges preserves valid 0.05 m (LIDAR_MIN_RANGE_M) hits as 5.0 cm while ignoring < 0.05 m dropouts', () => {
  const ranges = new Array(360).fill(4.5);
  ranges[0] = LIDAR_MIN_RANGE_M; // 0.05 m directly ahead (front)
  ranges[90] = 0; // invalid sensor dropout (left)
  ranges[180] = Number.NaN; // invalid NaN (back)
  ranges[270] = 0.02; // below min range (right)

  const res = computeZonesFromRanges(ranges, 0, 3.2);
  assert.equal(res.zones.front, 5.0);
  assert.equal(res.min_m, 0.05);
  assert.equal(res.zones.left, 999.0);
  assert.equal(res.zones.back, 999.0);
  assert.equal(res.zones.right, 999.0);
});

test('createOccupancyGrid enforces <= 4,000,000 cells contract', () => {
  assert.throws(
    () => createOccupancyGrid({ w: 2001, h: 2001 }),
    RangeError
  );
  const maxOk = createOccupancyGrid({ w: 2000, h: 2000 });
  assert.equal(maxOk.data.length, 4_000_000);
});
