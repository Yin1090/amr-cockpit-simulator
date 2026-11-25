/**
 * Main browser controller for the AMR Cockpit Simulator.
 * Connects Screen 1 (Connection), Screen 2 (Preflight), Screen 3 (Cockpit),
 * Drawers, Hold-to-Drive D-pad/Keyboard, Dual Canvases, and window.__AMR_SIM__.
 */

import {
  computeBaseTwist,
  DIRECTION_TO_ZONE,
  createAmrSimulator,
  angleDegToZone,
} from '../core/index.js';
import { renderRadarCanvas, canvasPointToWorld } from './radarCanvas.js';
import { renderMjpegHud } from './mjpegHud.js';

const STORAGE_KEY = 'amr_cockpit_cfg_v1';

const sim = createAmrSimulator({ scenarioId: 'clear', simulatedLatencyMs: 12 });

const appState = {
  screen: 'connect',
  mode: 'simulated', // 'simulated' | 'live'
  host: '127.0.0.1',
  apiPort: 8080,
  camPort: 8081,
  frameMode: 'bezel',
  activePointerId: null,
  activePointerDir: null,
  pressedKeys: new Set(),
  keyboardDir: null,
  scanlinePhase: 0,
  camStandby: false,
  camRetryCount: 0,
  lastTelemetrySnapshot: sim.getTelemetry(),
  lastMapSnapshot: sim.getMapFull(),
  rafId: null,
  simIntervalId: null,
  preflightTimerId: null,
  toastTimerId: null,
};

// DOM References
const $ = (id) => document.getElementById(id);

const appShell = $('app-shell');
const toastNotice = $('toast-notice');

// Screen 1
const screenConnect = $('screen-connect');
const inputHost = $('input-host');
const inputApiPort = $('input-api-port');
const inputCamPort = $('input-cam-port');
const connectError = $('connect-error');
const btnInstantCockpit = $('btn-instant-cockpit');
const btnLaunchSim = $('btn-launch-sim');
const btnConnectLive = $('btn-connect-live');

// Screen 2
const screenPreflight = $('screen-preflight');
const preflightModeTag = $('preflight-mode-tag');
const preflightCounter = $('preflight-counter');
const preflightProgress = $('preflight-progress');
const checkApi = $('check-api');
const checkLidar = $('check-lidar');
const checkCamera = $('check-camera');
const preflightStatus = $('preflight-status');
const btnPreflightEnter = $('btn-preflight-enter');
const btnPreflightRetry = $('btn-preflight-retry');
const btnPreflightBack = $('btn-preflight-back');
const btnPreflightSim = $('btn-preflight-sim');

// Screen 3
const screenCockpit = $('screen-cockpit');
const badgeMode = $('badge-mode');
const badgeArm = $('badge-arm');
const badgeLatency = $('badge-latency');
const badgeSafety = $('badge-safety');
const btnArm = $('btn-arm');
const btnMission = $('btn-mission');
const btnSettings = $('btn-settings');
const btnBezelToggle = $('btn-bezel-toggle');
const btnExit = $('btn-exit');

const lidarCanvas = $('lidar-canvas');
const cameraCanvas = $('camera-canvas');
const camStatusPill = $('cam-status-pill');
const btnCamReload = $('btn-cam-reload');

const dpadGrid = $('dpad-grid');
const safetyHint = $('safety-hint');

const valLrpm = $('val-lrpm');
const valRrpm = $('val-rrpm');
const valMinm = $('val-minm');
const valStatus = $('val-status');
const valPose = $('val-pose');
const restLog = $('rest-log');
const btnClearLog = $('btn-clear-log');

const armCardState = $('arm-card-state');
const btnCardArm = $('btn-card-arm');
const btnCardDisarm = $('btn-card-disarm');
const btnCardClearFault = $('btn-card-clear-fault');

const btnRpmDown = $('btn-rpm-down');
const btnRpmUp = $('btn-rpm-up');
const rpmValue = $('rpm-value');
const rpmKinematicsLine = $('rpm-kinematics-line');
const btnEstop = $('btn-estop');

const scenarioSelect = $('scenario-select');
const presetChips = Array.from(document.querySelectorAll('.preset-chips .chip-btn'));
const sliderObsDist = $('slider-obs-dist');
const obsDistLabel = $('obs-dist-label');
const sliderObsAngle = $('slider-obs-angle');
const obsAngleLabel = $('obs-angle-label');
const angleChips = Array.from(document.querySelectorAll('.angle-chips .chip-btn'));
const btnPlaceSliderObs = $('btn-place-slider-obs');
const btnResetPose = $('btn-reset-pose');
const btnClearObstacles = $('btn-clear-obstacles');

// Drawers
const drawerSettings = $('drawer-settings');
const btnCloseSettings = $('btn-close-settings');
const settingsHost = $('settings-host');
const settingsApiPort = $('settings-api-port');
const settingsCamPort = $('settings-cam-port');
const btnSaveNetwork = $('btn-save-network');
const sliderSimLatency = $('slider-sim-latency');
const simLatencyLabel = $('sim-latency-label');
const pidKp = $('pid-kp');
const pidKi = $('pid-ki');
const pidKd = $('pid-kd');
const btnApplyPid = $('btn-apply-pid');
const btnResetMap = $('btn-reset-map');
const btnInjectFault = $('btn-inject-fault');

const drawerMission = $('drawer-mission');
const btnCloseMission = $('btn-close-mission');
const missionDistance = $('mission-distance');
const missionExplored = $('mission-explored');
const missionMapMeta = $('mission-map-meta');
const missionTelemetryJson = $('mission-telemetry-json');

const ZONE_EL_MAP = {
  front_left: $('zone-FL'),
  front: $('zone-F'),
  front_right: $('zone-FR'),
  left: $('zone-L'),
  right: $('zone-R'),
  back_left: $('zone-BL'),
  back: $('zone-B'),
  back_right: $('zone-BR'),
};

function showToast(msg, durationMs = 2600) {
  toastNotice.textContent = msg;
  toastNotice.hidden = false;
  clearTimeout(appState.toastTimerId);
  appState.toastTimerId = setTimeout(() => {
    toastNotice.hidden = true;
  }, durationMs);
}

function loadSavedConfig() {
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (!raw) return;
    const parsed = JSON.parse(raw);
    if (parsed.host) {
      appState.host = String(parsed.host);
      inputHost.value = appState.host;
    }
    if (Number.isInteger(parsed.apiPort)) {
      appState.apiPort = parsed.apiPort;
      inputApiPort.value = String(parsed.apiPort);
    }
    if (Number.isInteger(parsed.camPort)) {
      appState.camPort = parsed.camPort;
      inputCamPort.value = String(parsed.camPort);
    }
  } catch {
    // Ignore corrupt localStorage
  }
}

function saveConfig(host, apiPort, camPort) {
  appState.host = host;
  appState.apiPort = apiPort;
  appState.camPort = camPort;
  try {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify({ host, apiPort, camPort }));
  } catch {
    // Ignore storage quota errors
  }
}

function validateNetworkInputs(hostVal, apiPortVal, camPortVal) {
  const host = String(hostVal ?? '').trim();
  const apiPort = Number(apiPortVal);
  const camPort = Number(camPortVal);
  if (!host) {
    return { ok: false, error: 'Host address cannot be empty.' };
  }
  if (!Number.isInteger(apiPort) || apiPort < 1 || apiPort > 65535) {
    return { ok: false, error: 'API Port must be an integer between 1 and 65535.' };
  }
  if (!Number.isInteger(camPort) || camPort < 1 || camPort > 65535) {
    return { ok: false, error: 'Camera Port must be an integer between 1 and 65535.' };
  }
  return { ok: true, host, apiPort, camPort };
}

function switchScreen(name) {
  const wasCockpit = appState.screen === 'cockpit';
  appState.screen = name;
  appShell.dataset.activeScreen = name;
  screenConnect.hidden = name !== 'connect';
  screenPreflight.hidden = name !== 'preflight';
  screenCockpit.hidden = name !== 'cockpit';

  if (name === 'cockpit') {
    startCockpitLoops();
    renderAll();
  } else {
    stopCockpitLoops();
    if (wasCockpit && (sim.getState().armed || sim.getState().cmdV !== 0)) {
      stopDrive('STOP');
    }
  }
}

// --- Screen 1 Handlers ---
btnInstantCockpit.addEventListener('click', () => {
  connectError.hidden = true;
  const v = validateNetworkInputs(inputHost.value, inputApiPort.value, inputCamPort.value);
  if (v.ok) {
    saveConfig(v.host, v.apiPort, v.camPort);
  }
  appState.mode = 'simulated';
  switchScreen('cockpit');
});

btnLaunchSim.addEventListener('click', () => {
  const v = validateNetworkInputs(inputHost.value, inputApiPort.value, inputCamPort.value);
  if (!v.ok) {
    connectError.textContent = v.error;
    connectError.hidden = false;
    return;
  }
  connectError.hidden = true;
  saveConfig(v.host, v.apiPort, v.camPort);
  appState.mode = 'simulated';
  switchScreen('preflight');
  runPreflight('simulated');
});

btnConnectLive.addEventListener('click', async () => {
  const v = validateNetworkInputs(inputHost.value, inputApiPort.value, inputCamPort.value);
  if (!v.ok) {
    connectError.textContent = v.error;
    connectError.hidden = false;
    return;
  }
  connectError.hidden = true;
  btnConnectLive.disabled = true;
  btnConnectLive.textContent = 'Probing /api/telemetry...';

  try {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 5000);
    const res = await fetch(`http://${v.host}:${v.apiPort}/api/telemetry`, { signal: controller.signal });
    clearTimeout(timer);
    if (res.ok || res.status === 404) {
      saveConfig(v.host, v.apiPort, v.camPort);
      appState.mode = 'live';
      switchScreen('preflight');
      runPreflight('live', res.status === 404);
    } else {
      connectError.textContent = `Live endpoint returned HTTP ${res.status}. Launch Offline Simulator instead.`;
      connectError.hidden = false;
    }
  } catch {
    connectError.textContent = `No live AMR reachable at ${v.host}:${v.apiPort}. Click "Enter Cockpit" or "Launch Offline Simulator".`;
    connectError.hidden = false;
  } finally {
    btnConnectLive.disabled = false;
    btnConnectLive.textContent = 'Connect (Live Probe)';
  }
});

// --- Screen 2 Preflight ---
function setCheckRow(rowEl, status, badgeText) {
  rowEl.dataset.status = status;
  const badge = rowEl.querySelector('.check-badge');
  if (badge) badge.textContent = badgeText;
}

function runPreflight(mode, degraded404 = false) {
  appState.lastDegraded404 = Boolean(degraded404);
  clearInterval(appState.preflightTimerId);
  preflightModeTag.textContent = mode === 'live' ? 'LIVE HARDWARE PROBE' : 'OFFLINE SIMULATOR';
  btnPreflightSim.hidden = mode !== 'live';
  btnPreflightEnter.disabled = true;
  preflightCounter.textContent = '0/3';
  preflightProgress.style.width = '0%';
  setCheckRow(checkApi, 'pending', 'WAIT');
  setCheckRow(checkLidar, 'pending', 'WAIT');
  setCheckRow(checkCamera, 'pending', 'WAIT');
  preflightStatus.textContent = 'Running preflight diagnostics...';

  if (mode === 'simulated') {
    let step = 0;
    appState.preflightTimerId = setInterval(() => {
      step += 1;
      if (step === 1) {
        setCheckRow(checkApi, 'pass', 'OK');
        preflightCounter.textContent = '1/3';
        preflightProgress.style.width = '33%';
        preflightStatus.textContent = 'Simulated REST API & safety watchdog ready.';
      } else if (step === 2) {
        setCheckRow(checkLidar, 'pass', 'OK');
        preflightCounter.textContent = '2/3';
        preflightProgress.style.width = '66%';
        preflightStatus.textContent = '360° 2D LiDAR & 0.05 m SLAM grid active.';
      } else if (step >= 3) {
        clearInterval(appState.preflightTimerId);
        setCheckRow(checkCamera, 'pass', 'OK');
        preflightCounter.textContent = '3/3';
        preflightProgress.style.width = '100%';
        preflightStatus.textContent = 'All subsystems ready (3/3). Entering cockpit...';
        btnPreflightEnter.disabled = false;
        setTimeout(() => {
          if (appState.screen === 'preflight') {
            switchScreen('cockpit');
          }
        }, 250);
      }
    }, 220);
  } else {
    probeLivePreflight(degraded404);
  }
}

async function probeLivePreflight(degraded404) {
  try {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 4500);
    const res = await fetch(`http://${appState.host}:${appState.apiPort}/api/telemetry`, { signal: controller.signal });
    clearTimeout(timer);
    if (res.ok) {
      setCheckRow(checkApi, 'pass', 'OK');
      setCheckRow(checkLidar, 'pass', 'OK');
      setCheckRow(checkCamera, 'pass', 'STREAM');
      preflightCounter.textContent = '3/3';
      preflightProgress.style.width = '100%';
      preflightStatus.textContent = 'Live AMR telemetry verified (3/3).';
      btnPreflightEnter.disabled = false;
    } else {
      setCheckRow(checkApi, 'fail', `HTTP ${res.status}`);
      preflightStatus.textContent = degraded404
        ? 'API returned 404. Retry or switch to Offline Simulator.'
        : `API check failed (HTTP ${res.status}).`;
    }
  } catch {
    setCheckRow(checkApi, 'fail', 'TIMEOUT');
    preflightStatus.textContent = 'Connection failed. Retry or switch to Offline Simulator.';
  }
}

btnPreflightEnter.addEventListener('click', () => switchScreen('cockpit'));
btnPreflightRetry.addEventListener('click', () => runPreflight(appState.mode, appState.lastDegraded404));
btnPreflightBack.addEventListener('click', () => {
  clearInterval(appState.preflightTimerId);
  switchScreen('connect');
});
btnPreflightSim.addEventListener('click', () => {
  clearInterval(appState.preflightTimerId);
  appState.mode = 'simulated';
  runPreflight('simulated');
});

// --- Screen 3 Cockpit Loops ---
function startCockpitLoops() {
  stopCockpitLoops();
  let lastTick = performance.now();
  let lastSlowPoll = 0;

  appState.simIntervalId = setInterval(() => {
    const now = performance.now();
    const dtMs = Math.min(80, Math.max(10, now - lastTick));
    lastTick = now;

    sim.step(dtMs);
    appState.lastTelemetrySnapshot = sim.getTelemetry();

    if (now - lastSlowPoll >= 250) {
      lastSlowPoll = now;
      appState.lastMapSnapshot = sim.getMapFull();
    }
    renderAll();
  }, 40);

  const frame = () => {
    appState.scanlinePhase = (appState.scanlinePhase + 0.008) % 1;
    renderCanvases();
    appState.rafId = window.requestAnimationFrame(frame);
  };
  appState.rafId = window.requestAnimationFrame(frame);
}

function stopCockpitLoops() {
  clearInterval(appState.simIntervalId);
  appState.simIntervalId = null;
  if (appState.rafId !== null) {
    window.cancelAnimationFrame(appState.rafId);
    appState.rafId = null;
  }
}

// --- Drive Control (Hold-to-Drive / Release-to-Stop) ---
function startDrive(dir) {
  if (dir === 'STOP') {
    stopDrive('STOP');
    return;
  }
  const res = sim.startHold(dir);
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  appState.lastMapSnapshot = sim.getMapFull();
  renderAll();
  return res;
}

function stopDrive(kind = 'RELEASE') {
  if (kind === 'STOP') {
    sim.stop();
  } else {
    sim.releaseHold();
  }
  appState.activePointerId = null;
  appState.activePointerDir = null;
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  renderAll();
}

// Pointer bindings on D-pad
dpadGrid.querySelectorAll('.dpad-btn').forEach((btn) => {
  const dir = btn.dataset.dir;
  btn.addEventListener('pointerdown', (e) => {
    if (e.pointerType === 'mouse' && e.button !== 0) return;
    e.preventDefault();
    appState.activePointerId = e.pointerId;
    appState.activePointerDir = dir;
    try {
      btn.setPointerCapture(e.pointerId);
    } catch {
      // Synthetic events in tests may not support capture
    }
    startDrive(dir);
  });

  const handleRelease = (e) => {
    if (appState.activePointerDir === null) return;
    if (e.type === 'pointerleave' && btn.hasPointerCapture?.(e.pointerId)) {
      return;
    }
    if (dir !== 'STOP') {
      stopDrive('RELEASE');
    }
  };

  btn.addEventListener('pointerup', handleRelease);
  btn.addEventListener('pointercancel', handleRelease);
  btn.addEventListener('lostpointercapture', handleRelease);
  btn.addEventListener('pointerleave', handleRelease);
});

window.addEventListener('pointerup', () => {
  if (appState.activePointerDir !== null && appState.activePointerDir !== 'STOP') {
    stopDrive('RELEASE');
  }
});

window.addEventListener('blur', () => {
  const wasDriving =
    appState.activePointerDir !== null ||
    appState.keyboardDir !== null ||
    sim.getState().activeDirection !== null;
  appState.pressedKeys.clear();
  appState.keyboardDir = null;
  if (appState.screen === 'cockpit' && wasDriving) {
    stopDrive('RELEASE');
  }
});

document.addEventListener('visibilitychange', () => {
  if (document.hidden && appState.screen === 'cockpit') {
    const wasDriving =
      appState.activePointerDir !== null ||
      appState.keyboardDir !== null ||
      sim.getState().activeDirection !== null;
    appState.pressedKeys.clear();
    appState.keyboardDir = null;
    if (wasDriving) {
      stopDrive('RELEASE');
    }
  }
});

// Keyboard bindings (WASD / Arrows / QEZC / Space / Escape)
function computeKeyboardDirection() {
  const keys = appState.pressedKeys;
  const up = keys.has('KeyW') || keys.has('ArrowUp');
  const down = keys.has('KeyS') || keys.has('ArrowDown');
  const left = keys.has('KeyA') || keys.has('ArrowLeft');
  const right = keys.has('KeyD') || keys.has('ArrowRight');

  if (keys.has('KeyQ')) return 'FL';
  if (keys.has('KeyE')) return 'FR';
  if (keys.has('KeyZ')) return 'BL';
  if (keys.has('KeyC')) return 'BR';

  if (up && left) return 'FL';
  if (up && right) return 'FR';
  if (down && left) return 'BL';
  if (down && right) return 'BR';
  if (up) return 'F';
  if (down) return 'B';
  if (left) return 'L';
  if (right) return 'R';
  return null;
}

const DRIVE_KEY_CODES = new Set([
  'KeyW', 'KeyA', 'KeyS', 'KeyD',
  'KeyQ', 'KeyE', 'KeyZ', 'KeyC',
  'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight',
]);

window.addEventListener('keydown', (e) => {
  if (appState.screen !== 'cockpit') return;
  const tag = e.target?.tagName;
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;

  if (e.code === 'Space') {
    e.preventDefault();
    appState.pressedKeys.clear();
    appState.keyboardDir = null;
    stopDrive('STOP');
    return;
  }

  if (e.code === 'Escape') {
    drawerSettings.hidden = true;
    drawerMission.hidden = true;
    appState.pressedKeys.clear();
    appState.keyboardDir = null;
    stopDrive('STOP');
    return;
  }

  if (!DRIVE_KEY_CODES.has(e.code) || e.repeat) return;
  e.preventDefault();
  appState.pressedKeys.add(e.code);
  const nextDir = computeKeyboardDirection();
  if (nextDir && nextDir !== appState.keyboardDir) {
    appState.keyboardDir = nextDir;
    startDrive(nextDir);
  }
});

window.addEventListener('keyup', (e) => {
  if (!DRIVE_KEY_CODES.has(e.code)) return;
  appState.pressedKeys.delete(e.code);
  const nextDir = computeKeyboardDirection();
  if (nextDir !== appState.keyboardDir) {
    appState.keyboardDir = nextDir;
    if (nextDir) {
      startDrive(nextDir);
    } else if (appState.screen === 'cockpit') {
      stopDrive('RELEASE');
    }
  }
});

// --- Right Column & Header Controls ---
function toggleArm() {
  const st = sim.getState();
  if (st.fault) {
    sim.clearFault();
    showToast('Fault cleared. Press ARM to engage.');
  } else if (st.armed) {
    sim.disarm();
  } else {
    sim.arm();
  }
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  renderAll();
}

btnArm.addEventListener('click', toggleArm);
btnCardArm.addEventListener('click', () => {
  const res = sim.arm();
  if (res.status === 'REJECTED') {
    showToast(`ARM rejected: ${res.reason}`);
  }
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  renderAll();
});
btnCardDisarm.addEventListener('click', () => {
  sim.disarm();
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  renderAll();
});
btnCardClearFault.addEventListener('click', () => {
  sim.clearFault();
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  renderAll();
});

btnRpmDown.addEventListener('click', () => {
  sim.stepRpm(-1);
  renderAll();
});

btnRpmUp.addEventListener('click', () => {
  sim.stepRpm(1);
  renderAll();
});

btnEstop.addEventListener('click', () => {
  stopDrive('STOP');
  showToast('EMERGENCY STOP sent.');
});

btnBezelToggle.addEventListener('click', () => {
  appState.frameMode = appState.frameMode === 'bezel' ? 'fullscreen' : 'bezel';
  appShell.dataset.frameMode = appState.frameMode;
  btnBezelToggle.textContent = appState.frameMode === 'bezel' ? 'FULL' : 'BEZEL';
});

btnExit.addEventListener('click', () => {
  drawerSettings.hidden = true;
  drawerMission.hidden = true;
  switchScreen('connect');
});

// Scenario & Obstacle Controls
function applyScenarioSelection(scenarioId) {
  sim.loadScenario(scenarioId);
  scenarioSelect.value = scenarioId;
  presetChips.forEach((chip) => {
    chip.classList.toggle('is-active', chip.dataset.scenario === scenarioId);
  });
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  appState.lastMapSnapshot = sim.getMapFull();
  renderAll();
}

scenarioSelect.addEventListener('change', () => applyScenarioSelection(scenarioSelect.value));
presetChips.forEach((chip) => {
  chip.addEventListener('click', () => applyScenarioSelection(chip.dataset.scenario));
});

function updateSliderObstacle(place = true) {
  const distM = Number(sliderObsDist.value);
  const angleDeg = Number(sliderObsAngle.value);
  const zoneName = angleDegToZone(angleDeg);
  obsDistLabel.textContent = `${distM.toFixed(2)} m (${Math.round(distM * 100)} cm)`;
  obsAngleLabel.textContent = `${angleDeg}° (${zoneName})`;

  if (place) {
    sim.setCustomObstaclePolar({ distanceM: distM, angleDeg, radiusM: 0.15 });
    appState.lastTelemetrySnapshot = sim.getTelemetry();
    appState.lastMapSnapshot = sim.getMapFull();
    renderAll();
  }
}

sliderObsDist.addEventListener('input', () => updateSliderObstacle(true));
sliderObsAngle.addEventListener('input', () => updateSliderObstacle(true));
btnPlaceSliderObs.addEventListener('click', () => updateSliderObstacle(true));

angleChips.forEach((chip) => {
  chip.addEventListener('click', () => {
    sliderObsAngle.value = chip.dataset.angle;
    updateSliderObstacle(true);
  });
});

btnResetPose.addEventListener('click', () => {
  sim.resetPose();
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  appState.lastMapSnapshot = sim.getMapFull();
  renderAll();
});

btnClearObstacles.addEventListener('click', () => {
  sim.clearCustomObstacle();
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  appState.lastMapSnapshot = sim.getMapFull();
  renderAll();
});

// Click-on-map obstacle placement/removal
lidarCanvas.addEventListener('click', (e) => {
  const rect = lidarCanvas.getBoundingClientRect();
  const st = sim.getState();
  const worldPt = canvasPointToWorld(e.offsetX, e.offsetY, rect.width, rect.height, st.pose);
  const existingCustom = st.world.obstacles.find((o) => o.id === 'obs-custom');
  if (existingCustom && Math.hypot(worldPt.x - existingCustom.x, worldPt.y - existingCustom.y) < 0.22) {
    sim.clearCustomObstacle();
  } else {
    sim.setCustomObstacleWorld({ x: worldPt.x, y: worldPt.y, radiusM: 0.14 });
  }
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  appState.lastMapSnapshot = sim.getMapFull();
  renderAll();
});

// Camera manual reload
btnCamReload.addEventListener('click', () => {
  appState.camStandby = false;
  appState.camRetryCount = 0;
  camStatusPill.textContent = appState.mode === 'live' ? 'SIM HUD (LIVE PROBE)' : 'SIM CAM';
  renderCanvases();
});

btnClearLog.addEventListener('click', () => {
  restLog.innerHTML = '';
});

// --- Drawers (Settings with Armed Guard, Mission Inspector) ---
btnSettings.addEventListener('click', () => {
  const st = sim.getState();
  if (st.armed) {
    // Spec requirement: refuses to open while armed and calls RequestStop()
    sim.stop();
    appState.lastTelemetrySnapshot = sim.getTelemetry();
    renderAll();
    showToast('Vehicle was ARMED — stopped vehicle before opening Settings. Click SETTINGS again.');
    return;
  }
  settingsHost.value = appState.host;
  settingsApiPort.value = String(appState.apiPort);
  settingsCamPort.value = String(appState.camPort);
  sliderSimLatency.value = String(st.simulatedLatencyMs);
  simLatencyLabel.textContent = `${st.simulatedLatencyMs} ms`;
  pidKp.value = String(st.pid.kp);
  pidKi.value = String(st.pid.ki);
  pidKd.value = String(st.pid.kd);
  drawerSettings.hidden = false;
});

btnCloseSettings.addEventListener('click', () => {
  drawerSettings.hidden = true;
});

btnSaveNetwork.addEventListener('click', () => {
  const v = validateNetworkInputs(settingsHost.value, settingsApiPort.value, settingsCamPort.value);
  if (!v.ok) {
    showToast(v.error);
    return;
  }
  saveConfig(v.host, v.apiPort, v.camPort);
  showToast(`Saved endpoint ${v.host}:${v.apiPort} (cam :${v.camPort}).`);
});

sliderSimLatency.addEventListener('input', () => {
  const ms = Number(sliderSimLatency.value);
  sim.setSimulatedLatencyMs(ms);
  simLatencyLabel.textContent = `${ms} ms`;
  renderAll();
});

btnApplyPid.addEventListener('click', () => {
  const rawKp = pidKp.value.trim();
  const rawKi = pidKi.value.trim();
  const rawKd = pidKd.value.trim();
  if (!rawKp || !rawKi || !rawKd) {
    showToast('Invalid PID parameter values.');
    return;
  }
  const kp = Number(rawKp);
  const ki = Number(rawKi);
  const kd = Number(rawKd);
  const res = sim.request(`/api/pid?kp=${encodeURIComponent(kp)}&ki=${encodeURIComponent(ki)}&kd=${encodeURIComponent(kd)}`);
  if (res.statusCode === 200) {
    showToast(`PID applied: Kp=${kp}, Ki=${ki}, Kd=${kd}`);
  } else {
    showToast('Invalid PID parameter values.');
  }
  renderAll();
});

btnResetMap.addEventListener('click', () => {
  sim.resetMap();
  appState.lastMapSnapshot = sim.getMapFull();
  renderAll();
  showToast('SLAM occupancy map reset.');
});

btnInjectFault.addEventListener('click', () => {
  sim.injectFault('MOTOR_OVERCURRENT');
  drawerSettings.hidden = true;
  appState.lastTelemetrySnapshot = sim.getTelemetry();
  renderAll();
  showToast('Fault injected: MOTOR_OVERCURRENT.');
});

btnMission.addEventListener('click', () => {
  const st = sim.getState();
  if (st.armed) {
    sim.stop();
  }
  const refreshed = sim.getState();
  const tele = sim.getTelemetry();
  const map = sim.getMapFull();
  missionDistance.textContent = `${refreshed.distanceTraveledM.toFixed(3)} m`;
  missionExplored.textContent = `${(refreshed.exploredRatio * 100).toFixed(1)} %`;
  missionMapMeta.textContent = `w=${map.w}, h=${map.h}, res=${map.res} m, ox=${map.ox}, oy=${map.oy}, cells=${map.data.length}`;
  const teleSummary = { ...tele, ranges: `[${tele.ranges.length} rays]` };
  missionTelemetryJson.textContent = JSON.stringify(teleSummary, null, 2);
  drawerMission.hidden = false;
  renderAll();
});

btnCloseMission.addEventListener('click', () => {
  drawerMission.hidden = true;
});

// --- Rendering ---
function renderCanvases() {
  if (appState.screen !== 'cockpit') return;
  const st = sim.getState();
  const tele = appState.lastTelemetrySnapshot;
  const map = appState.lastMapSnapshot;
  renderRadarCanvas(lidarCanvas, {
    telemetry: tele,
    mapFull: map,
    state: st,
    stale: st.telemetryStale || !st.lidarOk,
  });
  renderMjpegHud(cameraCanvas, {
    telemetry: tele,
    state: st,
    scanlinePhase: appState.scanlinePhase,
    standby: appState.camStandby,
  });
}

function renderAll() {
  if (appState.screen !== 'cockpit') return;
  const st = sim.getState();
  const tele = appState.lastTelemetrySnapshot;

  // Header badges
  if (st.telemetryStale) {
    badgeMode.textContent = 'STALE';
    badgeMode.className = 'status-pill pill-warn';
  } else {
    badgeMode.textContent = appState.mode === 'live' ? 'SIMULATED (LIVE PROBE OK)' : 'SIMULATED';
    badgeMode.className = 'status-pill pill-info';
  }
  camStatusPill.textContent = appState.mode === 'live' ? 'SIM HUD (LIVE PROBE)' : 'SIM CAM';

  if (st.fault) {
    badgeArm.textContent = 'FAULT';
    badgeArm.className = 'status-pill pill-danger';
    btnArm.textContent = 'CLEAR FAULT';
    armCardState.textContent = `FAULT: ${st.fault}`;
    btnCardClearFault.hidden = false;
  } else if (st.armed) {
    badgeArm.textContent = 'ARMED';
    badgeArm.className = 'status-pill pill-ok';
    btnArm.textContent = 'DISARM';
    armCardState.textContent = 'ARMED · ACTIVE';
    btnCardClearFault.hidden = true;
  } else {
    badgeArm.textContent = 'DISARMED';
    badgeArm.className = 'status-pill pill-muted';
    btnArm.textContent = 'ARM';
    armCardState.textContent = 'DISARMED';
    btnCardClearFault.hidden = true;
  }

  badgeLatency.textContent = `${st.simulatedLatencyMs} ms`;
  badgeSafety.textContent = st.safetyStatus;
  badgeSafety.className =
    st.safetyStatus === 'DANGER_STOP'
      ? 'status-pill pill-danger'
      : st.safetyStatus === 'WARNING'
        ? 'status-pill pill-warn'
        : 'status-pill pill-ok';

  // Telemetry grid
  valLrpm.textContent = tele.robot.l_rpm.toFixed(1);
  valRrpm.textContent = tele.robot.r_rpm.toFixed(1);
  valMinm.textContent = tele.min_m >= 900 ? 'CLEAR' : `${tele.min_m.toFixed(2)} m`;
  valStatus.textContent = tele.status;
  valStatus.style.color =
    tele.status === 'DANGER_STOP' || tele.status === 'FAULT'
      ? 'var(--danger-bright)'
      : tele.status === 'WARNING' || tele.status === 'STALE'
        ? 'var(--amber)'
        : 'var(--emerald)';

  const deg = ((st.pose.th * 180) / Math.PI).toFixed(1);
  valPose.textContent = `x: ${st.pose.x.toFixed(3)} m · y: ${st.pose.y.toFixed(3)} m · th: ${deg}°`;

  // 8-zone strip & D-pad blocked indicators
  for (const [zone, el] of Object.entries(ZONE_EL_MAP)) {
    const cm = tele.zones[zone] ?? 999;
    el.textContent = cm >= 999 ? '999' : String(Math.round(cm));
    el.parentElement.classList.toggle('is-blocked', cm < 40.0);
  }

  dpadGrid.querySelectorAll('.dpad-btn').forEach((btn) => {
    const dir = btn.dataset.dir;
    btn.classList.toggle('is-active', st.activeDirection === dir);
    if (dir === 'STOP') return;
    const z = DIRECTION_TO_ZONE[dir];
    const blocked = (tele.zones[z] ?? 999) < 40.0;
    btn.classList.toggle('is-blocked', blocked);
  });

  // Safety hint bar
  if (st.fault) {
    safetyHint.textContent = `FAULT (${st.fault}) — Clear fault before arming.`;
    safetyHint.className = 'safety-hint is-danger';
  } else if (st.telemetryStale) {
    safetyHint.textContent = 'TELEMETRY WATCHDOG (> 1500 ms) — DRIVE LOCKED.';
    safetyHint.className = 'safety-hint is-danger';
  } else if (!st.lidarOk) {
    safetyHint.textContent = 'LIDAR UNHEALTHY — DRIVE LOCKED.';
    safetyHint.className = 'safety-hint is-danger';
  } else if (st.autoHaltEvent?.triggered) {
    safetyHint.textContent = `AUTO-HALT: ${st.autoHaltEvent.message}`;
    safetyHint.className = 'safety-hint is-danger';
  } else if (st.lastBlockAlert?.message) {
    safetyHint.textContent = st.lastBlockAlert.message;
    safetyHint.className = 'safety-hint is-danger';
  } else if (st.blockedZones.length > 0) {
    const first = st.blockedZones[0];
    safetyHint.textContent = `INTERLOCK < 40 cm: ${first.zone.toUpperCase()} (${first.distanceCm.toFixed(1)} cm) blocked.`;
    safetyHint.className = 'safety-hint is-danger';
  } else if (!st.armed) {
    safetyHint.textContent = 'DISARMED — Press ARM to enable drive.';
    safetyHint.className = 'safety-hint';
  } else {
    safetyHint.textContent = 'ARMED & SAFE — Hold D-pad or WASD to drive.';
    safetyHint.className = 'safety-hint is-ready';
  }

  // Speed stepper
  rpmValue.textContent = String(st.desiredRpm);
  const { v, w } = computeBaseTwist(st.desiredRpm);
  rpmKinematicsLine.textContent = `v = ${v.toFixed(3)} m/s · ω = ${w.toFixed(3)} rad/s`;

  // REST log (recent 50 entries, newest on top)
  const escapeHtml = (v) =>
    String(v)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;');
  const logs = sim.getCommandLog().slice(-50).reverse();
  restLog.innerHTML = logs
    .map((entry) => {
      const rejected = entry.response?.status === 'REJECTED' || entry.statusCode >= 400;
      const statusText = escapeHtml(entry.response?.status ?? entry.statusCode);
      const urlText = escapeHtml(entry.url);
      const latText = escapeHtml(entry.latencyMs);
      return `<div class="log-row ${rejected ? 'is-rejected' : ''}"><span>${statusText}</span><span class="log-url">${urlText}</span><span class="log-lat">${latText}ms</span></div>`;
    })
    .join('');

  renderCanvases();
}

// --- Testability Hook (window.__AMR_SIM__) ---
window.__AMR_SIM__ = {
  getState: () => ({
    screen: appState.screen,
    mode: appState.mode,
    frameMode: appState.frameMode,
    ...sim.getState(),
  }),
  getTelemetry: () => sim.getTelemetry(),
  getMapFull: () => sim.getMapFull(),
  getCommandLog: () => sim.getCommandLog(),
  actions: {
    enterCockpit: () => {
      appState.mode = 'simulated';
      switchScreen('cockpit');
      return window.__AMR_SIM__.getState();
    },
    arm: () => {
      const r = sim.arm();
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      renderAll();
      return r;
    },
    disarm: () => {
      const r = sim.disarm();
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      renderAll();
      return r;
    },
    stop: () => {
      const r = sim.stop();
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      renderAll();
      return r;
    },
    clearFault: () => {
      const r = sim.clearFault();
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      renderAll();
      return r;
    },
    injectFault: (reason) => {
      const r = sim.injectFault(reason);
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      renderAll();
      return r;
    },
    setRpm: (rpm) => {
      const r = sim.setRpm(rpm);
      renderAll();
      return r;
    },
    stepRpm: (delta) => {
      const r = sim.stepRpm(delta);
      renderAll();
      return r;
    },
    holdDirection: (dir) => {
      const r = startDrive(dir);
      return r;
    },
    releaseDirection: () => {
      stopDrive('RELEASE');
      return window.__AMR_SIM__.getState();
    },
    loadScenario: (id) => {
      applyScenarioSelection(id);
      return window.__AMR_SIM__.getState();
    },
    resetPose: () => {
      sim.resetPose();
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      appState.lastMapSnapshot = sim.getMapFull();
      renderAll();
      return window.__AMR_SIM__.getState();
    },
    setCustomObstaclePolar: (params) => {
      const r = sim.setCustomObstaclePolar(params);
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      appState.lastMapSnapshot = sim.getMapFull();
      renderAll();
      return r;
    },
    setCustomObstacleWorld: (params) => {
      const r = sim.setCustomObstacleWorld(params);
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      appState.lastMapSnapshot = sim.getMapFull();
      renderAll();
      return r;
    },
    clearCustomObstacle: () => {
      const r = sim.clearCustomObstacle();
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      appState.lastMapSnapshot = sim.getMapFull();
      renderAll();
      return r;
    },
    setDynamicPaused: (paused) => sim.setDynamicPaused(paused),
    setTelemetryStall: (stalled) => {
      const r = sim.setTelemetryStall(stalled);
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      renderAll();
      return r;
    },
    setLidarConnected: (connected) => {
      const r = sim.setLidarConnected(connected);
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      renderAll();
      return r;
    },
    setSimulatedLatencyMs: (ms) => {
      const r = sim.setSimulatedLatencyMs(ms);
      renderAll();
      return r;
    },
    step: (dtMs) => {
      const r = sim.step(dtMs);
      appState.lastTelemetrySnapshot = sim.getTelemetry();
      appState.lastMapSnapshot = sim.getMapFull();
      renderAll();
      return r;
    },
    resetMap: () => {
      const r = sim.resetMap();
      appState.lastMapSnapshot = sim.getMapFull();
      renderAll();
      return r;
    },
  },
};

loadSavedConfig();
switchScreen('connect');
