/**
 * Stateful deterministic differential-drive AMR simulator with virtual clock,
 * 360-deg LiDAR, SLAM occupancy grid, 8-zone <40 cm mid-motion auto-halt,
 * 150 ms hold repeat, 1500 ms telemetry watchdog, and REST endpoint router.
 * Pure ESM module — zero DOM or browser global dependencies.
 */

import {
  RPM_DEFAULT,
  clampRpm,
  stepRpm as stepRpmValue,
  computeDirectionCommand,
  integrateUnicyclePose,
} from './kinematics.js';
import {
  raycastWorld,
  computeZonesFromRanges,
  resolveRobotCollision,
  createOccupancyGrid,
  updateOccupancyGrid,
  resetOccupancyGrid,
  serializeOccupancyGrid,
} from './lidar.js';
import {
  createWorldFromScenario,
  stepDynamicObstacles,
  setCustomObstaclePolar as worldSetCustomPolar,
  setCustomObstacleWorld as worldSetCustomWorld,
  clearCustomObstacle as worldClearCustom,
} from './scenarios.js';
import {
  OBSTACLE_THRESHOLD_CM,
  WATCHDOG_TIMEOUT_MS,
  isLidarHealthy,
  isTelemetryWatchdogTripped,
  getBlockedZones,
  computeSafetyStatus,
  evaluateDriveSafety,
} from './safety.js';

export const HOLD_REPEAT_MS = 150;
export const WATCHDOG_CHECK_MS = 250;
const MAX_SUBSTEP_MS = 25;

/**
 * Creates a self-contained deterministic AMR simulator instance.
 * @param {{ scenarioId?: string, simulatedLatencyMs?: number }} [options]
 */
export function createAmrSimulator(options = {}) {
  let simTimeMs = 0;
  let lastTelemetryAtMs = 0;
  let lastHoldDispatchAtMs = 0;
  let lastWatchdogCheckAtMs = 0;

  let world = createWorldFromScenario(options.scenarioId ?? 'clear');
  let pose = { ...world.initialPose };
  let grid = createOccupancyGrid();
  let trail = [{ x: pose.x, y: pose.y, th: pose.th }];
  let distanceTraveledM = 0;

  let armed = false;
  let fault = null;
  let stopPending = false;
  let desiredRpm = RPM_DEFAULT;
  let activeDirection = null;
  let cmdV = 0;
  let cmdW = 0;
  let wheelRpms = { l_rpm: 0, r_rpm: 0 };

  let epoch = 1;
  let seq = 0;
  let pid = { kp: 0.5, ki: 1.5, kd: 0.01 };
  let simulatedLatencyMs = clampLatency(options.simulatedLatencyMs ?? 12);

  let lidarConnected = true;
  let telemetryStall = false;
  let telemetryStale = false;
  let lidarAgeS = 0.02;

  let lastScan = { ranges: [], hits: [], min_ang: 0 };
  let lastZonesResult = {
    zones: {
      front: 999.0,
      front_left: 999.0,
      left: 999.0,
      back_left: 999.0,
      back: 999.0,
      back_right: 999.0,
      right: 999.0,
      front_right: 999.0,
    },
    min_m: 999.0,
    min_ang: 0,
  };

  let autoHaltEvent = null;
  let lastBlockAlert = null;
  const commandLog = [];

  function clampLatency(ms) {
    const val = Number(ms);
    if (!Number.isFinite(val)) return 12;
    return Math.max(0, Math.min(2000, Math.round(val)));
  }

  function logRest(method, url, statusCode, responseBody, latencyMs = simulatedLatencyMs) {
    commandLog.push({
      id: commandLog.length + 1,
      tsMs: simTimeMs,
      method,
      url,
      statusCode,
      latencyMs,
      response: responseBody,
    });
    if (commandLog.length > 200) {
      commandLog.shift();
    }
  }

  function refreshSensorsAndGrid(updateGrid = true) {
    lastScan = raycastWorld(pose, world);
    lastZonesResult = computeZonesFromRanges(lastScan.ranges, lastScan.min_ang);
    if (updateGrid && lidarConnected) {
      updateOccupancyGrid(grid, pose, lastScan.ranges, lastScan.hits, lastScan.min_ang);
    }
  }

  function getLidarObj() {
    return {
      connected: lidarConnected,
      age_s: Number(lidarAgeS.toFixed(3)),
    };
  }

  function haltMotion(reason = 'STOP', alertInfo = null, disarm = false) {
    const wasMoving = activeDirection !== null || Math.abs(cmdV) > 1e-6 || Math.abs(cmdW) > 1e-6;
    activeDirection = null;
    cmdV = 0;
    cmdW = 0;
    wheelRpms = { l_rpm: 0, r_rpm: 0 };
    if (disarm) {
      armed = false;
      epoch += 1;
    }
    if (alertInfo) {
      lastBlockAlert = alertInfo;
    }
    if (wasMoving || reason === 'STOP' || reason === 'RELEASE') {
      const cmdVal = reason === 'STOP' ? 'STOP' : 'V 0.000 0.000';
      const res = { status: 'SENT' };
      logRest('GET', `/api/cmd?val=${encodeURIComponent(cmdVal)}&epoch=${epoch}&seq=${seq}`, 200, res);
    }
  }

  function checkWatchdogAndAges(dtSec) {
    const isStalling = telemetryStall || simulatedLatencyMs > WATCHDOG_TIMEOUT_MS;
    if (isStalling) {
      lidarAgeS += dtSec;
      if (lastTelemetryAtMs === 0) {
        lastTelemetryAtMs = Math.max(1, simTimeMs);
      }
      if (isTelemetryWatchdogTripped({ nowMs: simTimeMs, lastTelemetryAtMs, timeoutMs: WATCHDOG_TIMEOUT_MS })) {
        if (!telemetryStale) {
          telemetryStale = true;
          haltMotion(
            'STOP',
            {
              reasonCode: 'TELEMETRY_STALE',
              message: 'Telemetry watchdog tripped (> 1500 ms) — drive locked.',
              tsMs: simTimeMs,
            },
            true
          );
        }
      }
    } else {
      telemetryStale = false;
      lidarAgeS = lidarConnected ? 0.02 : 9.99;
      lastTelemetryAtMs = simTimeMs > 0 ? simTimeMs : 1;
    }

    if (!lidarConnected && armed && activeDirection) {
      haltMotion(
        'V 0.000 0.000',
        {
          reasonCode: 'LIDAR_UNHEALTHY',
          message: 'LiDAR disconnected — drive halted.',
          tsMs: simTimeMs,
        },
        false
      );
    }
  }

  function triggerAutoHalt(safetyCheck) {
    const dir = activeDirection;
    autoHaltEvent = {
      triggered: true,
      tsMs: simTimeMs,
      direction: dir,
      blockedZone: safetyCheck.blockedZone,
      distanceCm: safetyCheck.distanceCm,
      recoveryDirections: safetyCheck.recoveryDirections,
      message: safetyCheck.message,
      cmdVal: 'V 0.000 0.000',
    };
    // Keep armed=true so operator can immediately command recovery direction (e.g., B)
    haltMotion('V 0.000 0.000', autoHaltEvent, false);
  }

  // Initial scan on creation
  lastTelemetryAtMs = 1;
  refreshSensorsAndGrid(true);

  function executeCmd(val, reqEpoch = epoch, reqSeq = seq) {
    const trimmed = String(val ?? '').trim();

    if (trimmed === 'ARM') {
      if (fault) {
        const res = { status: 'REJECTED', reason: fault };
        logRest('GET', `/api/cmd?val=ARM&epoch=${reqEpoch}&seq=${reqSeq}`, 200, res);
        return res;
      }
      if (telemetryStale) {
        const res = { status: 'REJECTED', reason: 'TELEMETRY_STALE' };
        logRest('GET', `/api/cmd?val=ARM&epoch=${reqEpoch}&seq=${reqSeq}`, 200, res);
        return res;
      }
      armed = true;
      stopPending = false;
      epoch += 1;
      const res = { status: 'SENT' };
      logRest('GET', `/api/cmd?val=ARM&epoch=${epoch}&seq=${seq}`, 200, res);
      return res;
    }

    if (trimmed === 'DISARM') {
      armed = false;
      activeDirection = null;
      cmdV = 0;
      cmdW = 0;
      wheelRpms = { l_rpm: 0, r_rpm: 0 };
      epoch += 1;
      const res = { status: 'SENT' };
      logRest('GET', `/api/cmd?val=DISARM&epoch=${epoch}&seq=${seq}`, 200, res);
      return res;
    }

    if (trimmed === 'STOP') {
      stopPending = false;
      armed = false;
      activeDirection = null;
      cmdV = 0;
      cmdW = 0;
      wheelRpms = { l_rpm: 0, r_rpm: 0 };
      epoch += 1;
      const res = { status: 'SENT' };
      logRest('GET', `/api/cmd?val=STOP&epoch=${epoch}&seq=${seq}`, 200, res);
      return res;
    }

    if (trimmed === 'CLEAR_FAULT') {
      fault = null;
      stopPending = false;
      const res = { status: 'SENT' };
      logRest('GET', `/api/cmd?val=CLEAR_FAULT&epoch=${epoch}&seq=${seq}`, 200, res);
      return res;
    }

    if (trimmed.startsWith('V ')) {
      const parts = trimmed.split(/\s+/);
      const reqV = Number(parts[1]);
      const reqW = Number(parts[2]);
      if (!Number.isFinite(reqV) || !Number.isFinite(reqW)) {
        const res = { status: 'REJECTED', reason: 'INVALID_VELOCITY' };
        logRest('GET', `/api/cmd?val=${encodeURIComponent(trimmed)}&epoch=${reqEpoch}&seq=${reqSeq}`, 200, res);
        return res;
      }
      if (Math.abs(reqV) < 1e-6 && Math.abs(reqW) < 1e-6) {
        activeDirection = null;
        cmdV = 0;
        cmdW = 0;
        wheelRpms = { l_rpm: 0, r_rpm: 0 };
        const res = { status: 'SENT' };
        logRest('GET', `/api/cmd?val=V%200.000%200.000&epoch=${epoch}&seq=${seq}`, 200, res);
        return res;
      }
      if (fault || !armed || telemetryStale || !isLidarHealthy(getLidarObj())) {
        const res = { status: 'REJECTED', reason: fault || (!armed ? 'DISARMED' : 'SENSOR_LOCK') };
        logRest('GET', `/api/cmd?val=${encodeURIComponent(trimmed)}&epoch=${epoch}&seq=${seq}`, 200, res);
        return res;
      }
      cmdV = reqV;
      cmdW = reqW;
      const k = (2 * Math.PI * 0.0366) / 60;
      wheelRpms = {
        l_rpm: (cmdV - cmdW * 0.096) / k,
        r_rpm: (cmdV + cmdW * 0.096) / k,
      };
      seq += 1;
      const res = { status: 'SENT' };
      logRest('GET', `/api/cmd?val=${encodeURIComponent(trimmed)}&epoch=${epoch}&seq=${seq}`, 200, res);
      return res;
    }

    const res = { status: 'REJECTED', reason: 'UNKNOWN_CMD' };
    logRest('GET', `/api/cmd?val=${encodeURIComponent(trimmed)}&epoch=${reqEpoch}&seq=${reqSeq}`, 200, res);
    return res;
  }

  function startHold(direction) {
    if (direction === 'STOP') {
      return requestStop();
    }
    const safety = evaluateDriveSafety({
      armed,
      fault,
      stopPending,
      lidar: getLidarObj(),
      telemetryStale,
      direction,
      zones: lastZonesResult.zones,
    });

    if (!safety.allowed) {
      activeDirection = null;
      cmdV = 0;
      cmdW = 0;
      wheelRpms = { l_rpm: 0, r_rpm: 0 };
      lastBlockAlert = {
        tsMs: simTimeMs,
        direction,
        ...safety,
      };
      return { accepted: false, safety };
    }

    const cmd = computeDirectionCommand(direction, desiredRpm);
    activeDirection = direction;
    cmdV = cmd.v;
    cmdW = cmd.w;
    wheelRpms = { l_rpm: cmd.l_rpm, r_rpm: cmd.r_rpm };
    autoHaltEvent = null;
    lastBlockAlert = null;
    lastHoldDispatchAtMs = simTimeMs;
    seq += 1;
    const wireRes = { status: 'SENT' };
    logRest('GET', `/api/cmd?val=${encodeURIComponent(cmd.cmdVal)}&epoch=${epoch}&seq=${seq}`, 200, wireRes);
    return { accepted: true, safety, cmd };
  }

  function releaseHold() {
    const hadMotion = activeDirection !== null || Math.abs(cmdV) > 1e-6 || Math.abs(cmdW) > 1e-6;
    activeDirection = null;
    cmdV = 0;
    cmdW = 0;
    wheelRpms = { l_rpm: 0, r_rpm: 0 };
    if (hadMotion) {
      const res = { status: 'SENT' };
      logRest('GET', `/api/cmd?val=V%200.000%200.000&epoch=${epoch}&seq=${seq}`, 200, res);
    } else {
      // Explicit release-to-stop guarantee emits V 0.000 0.000
      const res = { status: 'SENT' };
      logRest('GET', `/api/cmd?val=V%200.000%200.000&epoch=${epoch}&seq=${seq}`, 200, res);
    }
    return { cmdVal: 'V 0.000 0.000' };
  }

  function requestStop() {
    autoHaltEvent = null;
    return executeCmd('STOP');
  }

  function stepSingleSlice(sliceMs) {
    const dtSec = sliceMs / 1000;
    simTimeMs += sliceMs;

    if (simTimeMs - lastWatchdogCheckAtMs >= WATCHDOG_CHECK_MS || telemetryStall || simulatedLatencyMs > WATCHDOG_TIMEOUT_MS) {
      lastWatchdogCheckAtMs = simTimeMs;
      checkWatchdogAndAges(dtSec);
    } else if (!telemetryStall) {
      lastTelemetryAtMs = simTimeMs;
      lidarAgeS = lidarConnected ? 0.02 : 9.99;
    }

    if (activeDirection) {
      const preSafety = evaluateDriveSafety({
        armed,
        fault,
        stopPending,
        lidar: getLidarObj(),
        telemetryStale,
        direction: activeDirection,
        zones: lastZonesResult.zones,
      });

      if (!preSafety.allowed) {
        triggerAutoHalt(preSafety);
      } else {
        // Refresh command velocity to track live RPM changes while held
        const cmd = computeDirectionCommand(activeDirection, desiredRpm);
        cmdV = cmd.v;
        cmdW = cmd.w;
        wheelRpms = { l_rpm: cmd.l_rpm, r_rpm: cmd.r_rpm };

        if (simTimeMs - lastHoldDispatchAtMs >= HOLD_REPEAT_MS) {
          lastHoldDispatchAtMs = simTimeMs;
          seq += 1;
          logRest(
            'GET',
            `/api/cmd?val=${encodeURIComponent(cmd.cmdVal)}&epoch=${epoch}&seq=${seq}`,
            200,
            { status: 'SENT' }
          );
        }

        const prevPose = { ...pose };
        const moved = integrateUnicyclePose(pose, cmdV, cmdW, dtSec);
        const resolved = resolveRobotCollision(moved, world);
        distanceTraveledM += Math.hypot(resolved.x - prevPose.x, resolved.y - prevPose.y);
        pose = { x: resolved.x, y: resolved.y, th: resolved.th };

        const lastTrail = trail[trail.length - 1];
        if (!lastTrail || Math.hypot(pose.x - lastTrail.x, pose.y - lastTrail.y) >= 0.03) {
          trail.push({ x: pose.x, y: pose.y, th: pose.th });
          if (trail.length > 600) trail.shift();
        }
      }
    }

    stepDynamicObstacles(world, dtSec);
    const dynResolved = resolveRobotCollision(pose, world);
    pose = { x: dynResolved.x, y: dynResolved.y, th: dynResolved.th };
    refreshSensorsAndGrid(true);

    if (activeDirection) {
      const postSafety = evaluateDriveSafety({
        armed,
        fault,
        stopPending,
        lidar: getLidarObj(),
        telemetryStale,
        direction: activeDirection,
        zones: lastZonesResult.zones,
      });
      if (!postSafety.allowed) {
        triggerAutoHalt(postSafety);
      }
    }
  }

  function step(dtMs) {
    const total = Number(dtMs);
    if (!Number.isFinite(total) || total <= 0) {
      return getState();
    }
    let remaining = total;
    while (remaining > 0) {
      const slice = Math.min(MAX_SUBSTEP_MS, remaining);
      stepSingleSlice(slice);
      remaining -= slice;
    }
    return getState();
  }

  function getTelemetry() {
    const safetyStatus = computeSafetyStatus(lastZonesResult.zones, lastZonesResult.min_m);
    const statusLabel = fault ? 'FAULT' : telemetryStale ? 'STALE' : safetyStatus;
    return {
      status: statusLabel,
      min_m: lastZonesResult.min_m,
      robot: {
        x: Number(pose.x.toFixed(4)),
        y: Number(pose.y.toFixed(4)),
        th: Number(pose.th.toFixed(4)),
        l_rpm: Number(wheelRpms.l_rpm.toFixed(2)),
        r_rpm: Number(wheelRpms.r_rpm.toFixed(2)),
      },
      control: {
        armed,
        fault,
      },
      lidar: getLidarObj(),
      ranges: [...lastScan.ranges],
      min_ang: lastZonesResult.min_ang,
      map_info: {
        robot_pose: [
          Number(pose.x.toFixed(4)),
          Number(pose.y.toFixed(4)),
          Number(pose.th.toFixed(4)),
        ],
      },
      zones: { ...lastZonesResult.zones },
    };
  }

  function getMapFull() {
    return serializeOccupancyGrid(grid);
  }

  function request(urlPath) {
    const url = new URL(urlPath, 'http://127.0.0.1');
    const path = url.pathname;

    if (path === '/api/telemetry') {
      const payload = getTelemetry();
      return { statusCode: 200, body: payload };
    }

    if (path === '/api/map_full') {
      const payload = getMapFull();
      return { statusCode: 200, body: payload };
    }

    if (path === '/api/cmd') {
      const val = url.searchParams.get('val') ?? '';
      const reqEpoch = Number(url.searchParams.get('epoch') ?? epoch);
      const reqSeq = Number(url.searchParams.get('seq') ?? seq);
      const payload = executeCmd(val, reqEpoch, reqSeq);
      return { statusCode: 200, body: payload };
    }

    if (path === '/api/auto_config') {
      const speedParam = Number(url.searchParams.get('speed'));
      if (!Number.isFinite(speedParam)) {
        const err = { ok: false, error: 'INVALID_SPEED' };
        logRest('GET', url.pathname + url.search, 400, err);
        return { statusCode: 400, body: err };
      }
      desiredRpm = clampRpm(speedParam);
      const res = { ok: true, speed: desiredRpm, turn: desiredRpm };
      logRest('GET', `/api/auto_config?speed=${desiredRpm}&turn=${desiredRpm}`, 200, res);
      return { statusCode: 200, body: res };
    }

    if (path === '/api/pid') {
      const kp = Number(url.searchParams.get('kp'));
      const ki = Number(url.searchParams.get('ki'));
      const kd = Number(url.searchParams.get('kd'));
      if (!Number.isFinite(kp) || !Number.isFinite(ki) || !Number.isFinite(kd)) {
        const err = { ok: false, error: 'INVALID_PID' };
        logRest('GET', url.pathname + url.search, 400, err);
        return { statusCode: 400, body: err };
      }
      pid = { kp, ki, kd };
      const res = { ok: true, ...pid };
      logRest('GET', `/api/pid?kp=${kp}&ki=${ki}&kd=${kd}`, 200, res);
      return { statusCode: 200, body: res };
    }

    if (path === '/api/reset_map') {
      resetOccupancyGrid(grid);
      trail = [{ x: pose.x, y: pose.y, th: pose.th }];
      distanceTraveledM = 0;
      refreshSensorsAndGrid(true);
      const res = { ok: true };
      logRest('GET', '/api/reset_map', 200, res);
      return { statusCode: 200, body: res };
    }

    const notFound = { ok: false, error: 'NOT_FOUND' };
    logRest('GET', url.pathname + url.search, 404, notFound);
    return { statusCode: 404, body: notFound };
  }

  function requestAsync(urlPath) {
    return new Promise((resolve) => {
      const delay = simulatedLatencyMs;
      setTimeout(() => {
        resolve(request(urlPath));
      }, delay);
    });
  }

  function setRpm(rpm) {
    const next = clampRpm(rpm);
    request(`/api/auto_config?speed=${next}&turn=${next}`);
    return desiredRpm;
  }

  function stepRpm(deltaSteps = 1) {
    const next = stepRpmValue(desiredRpm, deltaSteps);
    request(`/api/auto_config?speed=${next}&turn=${next}`);
    return desiredRpm;
  }

  function loadScenario(scenarioId) {
    haltMotion('STOP', null, false);
    world = createWorldFromScenario(scenarioId);
    pose = { ...world.initialPose };
    resetOccupancyGrid(grid);
    trail = [{ x: pose.x, y: pose.y, th: pose.th }];
    distanceTraveledM = 0;
    autoHaltEvent = null;
    lastBlockAlert = null;
    refreshSensorsAndGrid(true);
    return getState();
  }

  function resetPose() {
    haltMotion('STOP', null, false);
    pose = { ...world.initialPose };
    trail = [{ x: pose.x, y: pose.y, th: pose.th }];
    distanceTraveledM = 0;
    autoHaltEvent = null;
    lastBlockAlert = null;
    refreshSensorsAndGrid(true);
    return getState();
  }

  function setCustomObstaclePolar(params) {
    const obs = worldSetCustomPolar(world, pose, params);
    refreshSensorsAndGrid(true);
    if (activeDirection) {
      const safety = evaluateDriveSafety({
        armed,
        fault,
        stopPending,
        lidar: getLidarObj(),
        telemetryStale,
        direction: activeDirection,
        zones: lastZonesResult.zones,
      });
      if (!safety.allowed) {
        triggerAutoHalt(safety);
      }
    }
    return obs;
  }

  function setCustomObstacleWorld(params) {
    const obs = worldSetCustomWorld(world, params, pose);
    refreshSensorsAndGrid(true);
    if (activeDirection) {
      const safety = evaluateDriveSafety({
        armed,
        fault,
        stopPending,
        lidar: getLidarObj(),
        telemetryStale,
        direction: activeDirection,
        zones: lastZonesResult.zones,
      });
      if (!safety.allowed) {
        triggerAutoHalt(safety);
      }
    }
    return obs;
  }

  function clearCustomObstacle() {
    const removed = worldClearCustom(world);
    refreshSensorsAndGrid(true);
    return removed;
  }

  function setDynamicPaused(paused) {
    world.dynamicPaused = Boolean(paused);
    return world.dynamicPaused;
  }

  function injectFault(reason = 'MOTOR_OVERCURRENT') {
    fault = String(reason || 'SYSTEM_FAULT');
    haltMotion('STOP', { reasonCode: 'FAULT', message: `Injected fault: ${fault}`, tsMs: simTimeMs }, true);
    return getState();
  }

  function clearFault() {
    return executeCmd('CLEAR_FAULT');
  }

  function setTelemetryStall(stalled) {
    telemetryStall = Boolean(stalled);
    if (!telemetryStall && simulatedLatencyMs <= WATCHDOG_TIMEOUT_MS) {
      telemetryStale = false;
      lastTelemetryAtMs = simTimeMs > 0 ? simTimeMs : 1;
      lidarAgeS = lidarConnected ? 0.02 : 9.99;
    } else if (telemetryStall && lastTelemetryAtMs === 0) {
      lastTelemetryAtMs = Math.max(1, simTimeMs);
    }
    return getState();
  }

  function setLidarConnected(connected) {
    lidarConnected = Boolean(connected);
    lidarAgeS = lidarConnected ? 0.02 : 9.99;
    if (!lidarConnected && armed && activeDirection) {
      haltMotion('V 0.000 0.000', {
        reasonCode: 'LIDAR_UNHEALTHY',
        message: 'LiDAR disconnected — drive halted.',
        tsMs: simTimeMs,
      }, false);
    }
    return getState();
  }

  function setSimulatedLatencyMs(ms) {
    simulatedLatencyMs = clampLatency(ms);
    if (simulatedLatencyMs <= WATCHDOG_TIMEOUT_MS && !telemetryStall) {
      telemetryStale = false;
      lastTelemetryAtMs = simTimeMs > 0 ? simTimeMs : 1;
      lidarAgeS = lidarConnected ? 0.02 : 9.99;
    }
    return simulatedLatencyMs;
  }

  function getState() {
    const lidar = getLidarObj();
    const blockedZones = getBlockedZones(lastZonesResult.zones, OBSTACLE_THRESHOLD_CM);
    const safetyStatus = computeSafetyStatus(lastZonesResult.zones, lastZonesResult.min_m);
    let exploredCells = 0;
    for (let i = 0; i < grid.data.length; i++) {
      if (grid.data[i] !== -1) exploredCells += 1;
    }
    return {
      simTimeMs,
      scenarioId: world.scenarioId,
      armed,
      fault,
      stopPending,
      lidar,
      lidarOk: isLidarHealthy(lidar),
      telemetryStall,
      telemetryStale,
      desiredRpm,
      pose: { ...pose },
      cmdV: Number(cmdV.toFixed(4)),
      cmdW: Number(cmdW.toFixed(4)),
      wheelRpms: {
        l_rpm: Number(wheelRpms.l_rpm.toFixed(2)),
        r_rpm: Number(wheelRpms.r_rpm.toFixed(2)),
      },
      activeDirection,
      zones: { ...lastZonesResult.zones },
      min_m: lastZonesResult.min_m,
      min_ang: lastZonesResult.min_ang,
      safetyStatus,
      blockedZones,
      autoHaltEvent,
      lastBlockAlert,
      epoch,
      seq,
      pid: { ...pid },
      simulatedLatencyMs,
      distanceTraveledM: Number(distanceTraveledM.toFixed(3)),
      exploredRatio: Number((exploredCells / grid.data.length).toFixed(3)),
      world: {
        bounds: { ...world.bounds },
        obstacles: world.obstacles.map((o) => ({ ...o })),
        dynamicPaused: world.dynamicPaused,
      },
      trail: trail.map((p) => ({ ...p })),
    };
  }

  return {
    getState,
    getTelemetry,
    getMapFull,
    getCommandLog: () => commandLog.slice(),
    request,
    requestAsync,
    step,
    arm: () => executeCmd('ARM'),
    disarm: () => executeCmd('DISARM'),
    stop: requestStop,
    clearFault,
    injectFault,
    setRpm,
    stepRpm,
    startHold,
    releaseHold,
    loadScenario,
    resetPose,
    resetMap: () => request('/api/reset_map'),
    setCustomObstaclePolar,
    setCustomObstacleWorld,
    clearCustomObstacle,
    setDynamicPaused,
    setTelemetryStall,
    setLidarConnected,
    setSimulatedLatencyMs,
  };
}
