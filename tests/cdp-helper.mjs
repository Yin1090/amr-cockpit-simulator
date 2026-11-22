/**
 * Zero-dependency headless Edge/Chrome CDP helper using Node 24 native WebSocket.
 */

import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn } from 'node:child_process';

const DEFAULT_BROWSER_PATHS = [
  process.env.AMR_BROWSER_PATH,
  'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe',
  '/usr/bin/google-chrome',
  '/usr/bin/chromium',
  '/usr/bin/microsoft-edge',
].filter(Boolean);

export function resolveBrowserBinary() {
  for (const p of DEFAULT_BROWSER_PATHS) {
    if (fs.existsSync(p)) return p;
  }
  throw new Error('No Chromium/Edge executable found for headless CDP verification.');
}

function launchBrowserForWs(browserPath, userDataDir) {
  return new Promise((resolve, reject) => {
    const proc = spawn(
      browserPath,
      [
        '--headless=new',
        '--disable-gpu',
        '--no-first-run',
        '--no-default-browser-check',
        '--disable-background-networking',
        '--remote-debugging-port=0',
        `--user-data-dir=${userDataDir}`,
        'about:blank',
      ],
      { stdio: ['ignore', 'pipe', 'pipe'] }
    );

    let buffer = '';
    const timer = setTimeout(() => {
      proc.kill();
      reject(new Error(`Timed out waiting for DevTools WebSocket URL. Output:\n${buffer}`));
    }, 12000);

    const onChunk = (chunk) => {
      buffer += chunk.toString();
      const match = buffer.match(/DevTools listening on (ws:\/\/[^\s]+)/);
      if (match) {
        clearTimeout(timer);
        resolve({ proc, wsUrl: match[1] });
      }
    };

    proc.stderr.on('data', onChunk);
    proc.stdout.on('data', onChunk);
    proc.on('exit', (code) => {
      clearTimeout(timer);
      reject(new Error(`Browser exited prematurely (${code}). Output:\n${buffer}`));
    });
  });
}

export async function createCdpSession({ width = 1280, height = 800 } = {}) {
  const browserPath = resolveBrowserBinary();
  const userDataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'amr-cdp-'));
  const { proc, wsUrl } = await launchBrowserForWs(browserPath, userDataDir);

  const ws = new WebSocket(wsUrl);
  await new Promise((resolve, reject) => {
    ws.addEventListener('open', resolve, { once: true });
    ws.addEventListener('error', reject, { once: true });
  });

  let nextId = 1;
  const pending = new Map();
  const consoleErrors = [];
  const pageExceptions = [];
  const httpFailures = [];

  ws.addEventListener('message', (evt) => {
    const msg = JSON.parse(evt.data);
    if (msg.id && pending.has(msg.id)) {
      const { resolve, reject } = pending.get(msg.id);
      pending.delete(msg.id);
      if (msg.error) reject(new Error(`${msg.error.code}: ${msg.error.message}`));
      else resolve(msg.result);
      return;
    }

    if (msg.method === 'Runtime.consoleAPICalled' && msg.params?.type === 'error') {
      consoleErrors.push(msg.params.args?.map((a) => a.value ?? a.description).join(' ') || 'console.error');
    } else if (msg.method === 'Runtime.exceptionThrown') {
      const det = msg.params?.exceptionDetails;
      pageExceptions.push(det?.exception?.description || det?.text || 'Uncaught exception');
    } else if (msg.method === 'Log.entryAdded' && msg.params?.entry?.level === 'error') {
      consoleErrors.push(msg.params.entry.text || 'Log.entryAdded error');
    } else if (msg.method === 'Network.responseReceived' && msg.params?.response?.status >= 400) {
      httpFailures.push(`${msg.params.response.status} ${msg.params.response.url}`);
    } else if (msg.method === 'Network.loadingFailed') {
      httpFailures.push(`FAILED ${msg.params.errorText || 'unknown'}`);
    }
  });

  function send(method, params = {}, sessionId = undefined) {
    const id = nextId++;
    const payload = { id, method, params };
    if (sessionId) payload.sessionId = sessionId;
    return new Promise((resolve, reject) => {
      pending.set(id, { resolve, reject });
      ws.send(JSON.stringify(payload));
    });
  }

  const { targetId } = await send('Target.createTarget', { url: 'about:blank' });
  const { sessionId } = await send('Target.attachToTarget', { targetId, flatten: true });

  await send('Page.enable', {}, sessionId);
  await send('Runtime.enable', {}, sessionId);
  await send('Log.enable', {}, sessionId);
  await send('Network.enable', {}, sessionId);
  await send(
    'Emulation.setDeviceMetricsOverride',
    { width, height, deviceScaleFactor: 1, mobile: false },
    sessionId
  );

  async function evaluate(expression) {
    const res = await send(
      'Runtime.evaluate',
      {
        expression,
        awaitPromise: true,
        returnByValue: true,
      },
      sessionId
    );
    if (res.exceptionDetails) {
      throw new Error(res.exceptionDetails.exception?.description || res.exceptionDetails.text || 'Eval failed');
    }
    return res.result?.value;
  }

  async function navigate(url) {
    await send('Page.navigate', { url }, sessionId);
    await waitFor('document.readyState === "complete" && Boolean(window.__AMR_SIM__)', 5000);
  }

  async function waitFor(predicateExpr, timeoutMs = 4000, pollMs = 40) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
      const ok = await evaluate(`Boolean(${predicateExpr})`);
      if (ok) return true;
      await new Promise((r) => setTimeout(r, pollMs));
    }
    throw new Error(`waitFor timed out after ${timeoutMs}ms: ${predicateExpr}`);
  }

  async function getBoxCenter(selector) {
    const box = await evaluate(`(() => {
      const el = document.querySelector(${JSON.stringify(selector)});
      if (!el) return null;
      const r = el.getBoundingClientRect();
      return { left: r.left, top: r.top, width: r.width, height: r.height, x: r.left + r.width / 2, y: r.top + r.height / 2 };
    })()`);
    if (!box || box.width === 0 || box.height === 0) {
      throw new Error(`Element not visible or missing: ${selector}`);
    }
    return box;
  }

  async function mouseDownSelector(selector) {
    const box = await getBoxCenter(selector);
    await send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: box.x, y: box.y }, sessionId);
    await send(
      'Input.dispatchMouseEvent',
      { type: 'mousePressed', x: box.x, y: box.y, button: 'left', clickCount: 1 },
      sessionId
    );
    return box;
  }

  async function mouseUpAt(x, y) {
    await send(
      'Input.dispatchMouseEvent',
      { type: 'mouseReleased', x, y, button: 'left', clickCount: 1 },
      sessionId
    );
  }

  async function clickSelector(selector) {
    const box = await mouseDownSelector(selector);
    await mouseUpAt(box.x, box.y);
  }

  async function setViewport(w, h) {
    await send(
      'Emulation.setDeviceMetricsOverride',
      { width: w, height: h, deviceScaleFactor: 1, mobile: w < 700 },
      sessionId
    );
  }

  async function close() {
    try {
      ws.close();
    } catch {
      // Ignore close error
    }
    const exitPromise = new Promise((res) => proc.once('exit', res));
    proc.kill();
    await Promise.race([exitPromise, new Promise((r) => setTimeout(r, 1500))]);
    try {
      fs.rmSync(userDataDir, { recursive: true, force: true, maxRetries: 5, retryDelay: 100 });
    } catch {
      // Best-effort temp profile cleanup on Windows
    }
  }

  return {
    send,
    sessionId,
    evaluate,
    navigate,
    waitFor,
    getBoxCenter,
    mouseDownSelector,
    mouseUpAt,
    clickSelector,
    setViewport,
    close,
    getErrors: () => ({
      consoleErrors: [...consoleErrors],
      pageExceptions: [...pageExceptions],
      httpFailures: [...httpFailures],
    }),
  };
}
