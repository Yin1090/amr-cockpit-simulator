import test from 'node:test';
import assert from 'node:assert/strict';
import { startStaticServer } from './serve.mjs';
import { createCdpSession } from './cdp-helper.mjs';

test('Headless browser E2E verifies 3-screen flow, D-pad hold/release, speed stepper, canvas render, <40cm auto-halt, settings guard, and viewports', async () => {
  const srv = await startStaticServer();
  const cdp = await createCdpSession({ width: 1280, height: 800 });

  try {
    await cdp.navigate(`${srv.baseUrl}/index.html`);

    // 1. Screen 1 neutral defaults & port validation
    const s1 = await cdp.evaluate(`({
      screen: window.__AMR_SIM__.getState().screen,
      host: document.getElementById('input-host').value,
      apiPort: document.getElementById('input-api-port').value,
      camPort: document.getElementById('input-cam-port').value
    })`);
    assert.equal(s1.screen, 'connect');
    assert.equal(s1.host, '127.0.0.1');
    assert.equal(s1.apiPort, '8080');
    assert.equal(s1.camPort, '8081');

    // Invalid port check
    await cdp.evaluate(`(() => {
      document.getElementById('input-api-port').value = '99999';
    })()`);
    await cdp.clickSelector('#btn-launch-sim');
    const errText = await cdp.evaluate(`document.getElementById('connect-error').textContent`);
    assert.ok(errText.includes('65535'));

    // Valid launch -> Screen 2 Preflight (0/3 -> 3/3) -> Screen 3 Cockpit
    await cdp.evaluate(`(() => {
      document.getElementById('input-api-port').value = '8080';
    })()`);
    await cdp.clickSelector('#btn-launch-sim');
    await cdp.waitFor('window.__AMR_SIM__.getState().screen === "preflight"', 2000);
    await cdp.waitFor('window.__AMR_SIM__.getState().screen === "cockpit"', 4000);

    // EXIT back to Screen 1, then test 1-click Enter Cockpit
    await cdp.clickSelector('#btn-exit');
    await cdp.waitFor('window.__AMR_SIM__.getState().screen === "connect"', 2000);
    await cdp.clickSelector('#btn-instant-cockpit');
    await cdp.waitFor('window.__AMR_SIM__.getState().screen === "cockpit"', 2000);

    // 2. Bezel / Fullscreen toggle
    assert.equal(await cdp.evaluate(`document.getElementById('app-shell').dataset.frameMode`), 'bezel');
    await cdp.clickSelector('#btn-bezel-toggle');
    assert.equal(await cdp.evaluate(`document.getElementById('app-shell').dataset.frameMode`), 'fullscreen');
    await cdp.clickSelector('#btn-bezel-toggle');
    assert.equal(await cdp.evaluate(`document.getElementById('app-shell').dataset.frameMode`), 'bezel');

    // 3. ARM + Speed Stepper (40 -> 50 RPM)
    await cdp.clickSelector('#btn-card-arm');
    assert.equal(await cdp.evaluate(`window.__AMR_SIM__.getState().armed`), true);

    await cdp.clickSelector('#btn-rpm-up');
    const rpmState = await cdp.evaluate(`({
      rpm: window.__AMR_SIM__.getState().desiredRpm,
      label: document.getElementById('rpm-value').textContent,
      log: document.getElementById('rest-log').textContent
    })`);
    assert.equal(rpmState.rpm, 50);
    assert.equal(rpmState.label, '50');
    assert.ok(rpmState.log.includes('/api/auto_config?speed=50&turn=50'));

    // 4. Settings guard while ARMED: refuses to open & stops vehicle
    await cdp.clickSelector('#btn-settings');
    const guardState = await cdp.evaluate(`({
      drawerHidden: document.getElementById('drawer-settings').hidden,
      armed: window.__AMR_SIM__.getState().armed,
      toast: document.getElementById('toast-notice').textContent
    })`);
    assert.equal(guardState.drawerHidden, true);
    assert.equal(guardState.armed, false);
    assert.ok(guardState.toast.includes('ARMED'));

    // Open Settings while DISARMED, apply PID, close
    await cdp.clickSelector('#btn-settings');
    assert.equal(await cdp.evaluate(`document.getElementById('drawer-settings').hidden`), false);
    await cdp.clickSelector('#btn-apply-pid');
    await cdp.clickSelector('#btn-close-settings');
    assert.equal(await cdp.evaluate(`document.getElementById('drawer-settings').hidden`), true);

    // Open Mission Inspector drawer and verify metadata & telemetry JSON
    await cdp.clickSelector('#btn-mission');
    assert.equal(await cdp.evaluate(`document.getElementById('drawer-mission').hidden`), false);
    const missionInfo = await cdp.evaluate(`({
      meta: document.getElementById('mission-map-meta').textContent,
      json: document.getElementById('mission-telemetry-json').textContent
    })`);
    assert.ok(missionInfo.meta.includes('w=120, h=120'));
    assert.ok(missionInfo.json.includes('"front"'));
    await cdp.clickSelector('#btn-close-mission');
    assert.equal(await cdp.evaluate(`document.getElementById('drawer-mission').hidden`), true);

    // 5. Real Pointer Hold-to-Drive & Release-to-Stop on #dpad-F
    await cdp.clickSelector('#btn-card-arm');
    const poseBefore = await cdp.evaluate(`window.__AMR_SIM__.getState().pose`);
    const box = await cdp.mouseDownSelector('#dpad-F');
    await new Promise((r) => setTimeout(r, 280));
    const poseDuring = await cdp.evaluate(`window.__AMR_SIM__.getState()`);
    assert.ok(poseDuring.pose.y > poseBefore.y + 0.02, 'Expected forward motion while holding #dpad-F');
    assert.ok(poseDuring.cmdV > 0);
    await cdp.mouseUpAt(box.x, box.y);

    await cdp.waitFor('window.__AMR_SIM__.getState().cmdV === 0', 1500);
    const afterRelease = await cdp.evaluate(`({
      cmdV: window.__AMR_SIM__.getState().cmdV,
      lRpm: document.getElementById('val-lrpm').textContent,
      lastUrl: window.__AMR_SIM__.getCommandLog().at(-1)?.url
    })`);
    assert.equal(afterRelease.cmdV, 0);
    assert.equal(afterRelease.lRpm, '0.0');
    assert.ok(afterRelease.lastUrl.includes('V%200.000%200.000'));

    // 6. Verify 2D LiDAR Canvas & Optical HUD Canvas rendered non-uniform pixels
    const canvasCheck = await cdp.evaluate(`(() => {
      const checkNonUniform = (id) => {
        const c = document.getElementById(id);
        const ctx = c.getContext('2d');
        const data = ctx.getImageData(0, 0, Math.min(c.width, 200), Math.min(c.height, 200)).data;
        const first = data[0] + ',' + data[1] + ',' + data[2];
        let diffs = 0;
        for (let i = 4; i < data.length; i += 16) {
          const px = data[i] + ',' + data[i+1] + ',' + data[i+2];
          if (px !== first) diffs++;
        }
        return { w: c.width, h: c.height, diffs };
      };
      return { lidar: checkNonUniform('lidar-canvas'), cam: checkNonUniform('camera-canvas') };
    })()`);
    assert.ok(canvasCheck.lidar.w >= 240 && canvasCheck.lidar.diffs > 20, 'LiDAR canvas should render graphics');
    assert.ok(canvasCheck.cam.w === 640 && canvasCheck.cam.diffs > 20, 'Camera HUD canvas should render graphics');

    // 7. Interactive obstacle slider < 40 cm blocks F and allows B
    await cdp.evaluate(`(() => {
      const dist = document.getElementById('slider-obs-dist');
      dist.value = '0.30';
      dist.dispatchEvent(new Event('input', { bubbles: true }));
    })()`);
    const obsBlocked = await cdp.evaluate(`({
      frontCm: window.__AMR_SIM__.getState().zones.front,
      fBlocked: document.getElementById('dpad-F').classList.contains('is-blocked'),
      hint: document.getElementById('safety-hint').textContent
    })`);
    assert.ok(obsBlocked.frontCm < 40.0);
    assert.equal(obsBlocked.fBlocked, true);
    assert.ok(obsBlocked.hint.includes('40'));

    // Try pressing #dpad-F -> must NOT move
    const yBlockedBefore = await cdp.evaluate(`window.__AMR_SIM__.getState().pose.y`);
    const fBox = await cdp.mouseDownSelector('#dpad-F');
    await new Promise((r) => setTimeout(r, 150));
    await cdp.mouseUpAt(fBox.x, fBox.y);
    const yBlockedAfter = await cdp.evaluate(`window.__AMR_SIM__.getState().pose.y`);
    assert.equal(yBlockedAfter, yBlockedBefore);

    // 8. Dynamic moving scenario mid-motion auto-halt in browser
    await cdp.evaluate(`(() => {
      const sel = document.getElementById('scenario-select');
      sel.value = 'dynamic_moving';
      sel.dispatchEvent(new Event('change', { bubbles: true }));
    })()`);
    await cdp.clickSelector('#btn-card-arm');
    const dynBox = await cdp.mouseDownSelector('#dpad-F');
    await cdp.waitFor('window.__AMR_SIM__.getState().autoHaltEvent?.triggered === true', 3500);
    await cdp.mouseUpAt(dynBox.x, dynBox.y);

    const dynState = await cdp.evaluate(`({
      autoHalt: window.__AMR_SIM__.getState().autoHaltEvent,
      cmdV: window.__AMR_SIM__.getState().cmdV,
      armed: window.__AMR_SIM__.getState().armed
    })`);
    assert.equal(dynState.autoHalt.triggered, true);
    assert.equal(dynState.cmdV, 0);
    assert.equal(dynState.armed, true);

    // 8b. Close-range click-on-map (0.17 m ahead) keeps obstacle outside robot footprint and blocks #dpad-F
    await cdp.evaluate(`(() => {
      const sel = document.getElementById('scenario-select');
      sel.value = 'clear';
      sel.dispatchEvent(new Event('change', { bubbles: true }));
    })()`);
    await cdp.clickSelector('#btn-card-arm');
    const radarBox = await cdp.getBoxCenter('#lidar-canvas');
    const pxPerM = (Math.min(radarBox.width, radarBox.height) * 0.44) / 2.0;
    const clickX = radarBox.x;
    const clickY = radarBox.y - 0.17 * pxPerM;
    await cdp.send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: clickX, y: clickY }, cdp.sessionId);
    await cdp.send(
      'Input.dispatchMouseEvent',
      { type: 'mousePressed', x: clickX, y: clickY, button: 'left', clickCount: 1 },
      cdp.sessionId
    );
    await cdp.mouseUpAt(clickX, clickY);

    const closeObs = await cdp.evaluate(`({
      frontCm: window.__AMR_SIM__.getState().zones.front,
      safetyStatus: window.__AMR_SIM__.getState().safetyStatus,
      fBlocked: document.getElementById('dpad-F').classList.contains('is-blocked'),
      poseY: window.__AMR_SIM__.getState().pose.y
    })`);
    assert.ok(closeObs.frontCm < 40.0, `Expected close click-on-map front < 40 cm, got ${closeObs.frontCm}`);
    assert.equal(closeObs.safetyStatus, 'DANGER_STOP');
    assert.equal(closeObs.fBlocked, true);

    const fCloseBox = await cdp.mouseDownSelector('#dpad-F');
    await new Promise((r) => setTimeout(r, 140));
    await cdp.mouseUpAt(fCloseBox.x, fCloseBox.y);
    const poseAfterCloseAttempt = await cdp.evaluate(`window.__AMR_SIM__.getState().pose.y`);
    assert.equal(poseAfterCloseAttempt, closeObs.poseY);
    await cdp.clickSelector('#btn-clear-obstacles');

    // 9. Responsive viewport checks (desktop 1280x800, mobile landscape 844x390, mobile portrait 390x844)
    for (const vp of [
      { w: 1280, h: 800 },
      { w: 844, h: 390 },
      { w: 390, h: 844 },
    ]) {
      await cdp.setViewport(vp.w, vp.h);
      await new Promise((r) => setTimeout(r, 80));
      const layout = await cdp.evaluate(`(() => {
        const doc = document.documentElement;
        const estop = document.getElementById('btn-estop').getBoundingClientRect();
        const dpadF = document.getElementById('dpad-F').getBoundingClientRect();
        const lidar = document.getElementById('lidar-canvas').getBoundingClientRect();
        return {
          overflowX: doc.scrollWidth > doc.clientWidth + 2,
          scrollWidth: doc.scrollWidth,
          clientWidth: doc.clientWidth,
          estopOk: estop.width > 50 && estop.height > 20,
          dpadOk: dpadF.width > 30 && dpadF.height > 30,
          lidarOk: lidar.width > 150 && lidar.height > 150
        };
      })()`);
      assert.equal(layout.overflowX, false, `Horizontal overflow at ${vp.w}x${vp.h}: ${layout.scrollWidth} > ${layout.clientWidth}`);
      assert.equal(layout.estopOk, true, `E-Stop clipped at ${vp.w}x${vp.h}`);
      assert.equal(layout.dpadOk, true, `D-pad clipped at ${vp.w}x${vp.h}`);
      assert.equal(layout.lidarOk, true, `LiDAR canvas clipped at ${vp.w}x${vp.h}`);
    }

    // 10. Zero console errors, zero page exceptions, zero HTTP failures
    const errs = cdp.getErrors();
    assert.deepEqual(errs.consoleErrors, [], `Console errors: ${errs.consoleErrors.join('; ')}`);
    assert.deepEqual(errs.pageExceptions, [], `Page exceptions: ${errs.pageExceptions.join('; ')}`);
    assert.deepEqual(errs.httpFailures, [], `HTTP failures: ${errs.httpFailures.join('; ')}`);
  } finally {
    await cdp.close();
    await srv.close();
  }
});
