/**
 * Simulated 640x480 MJPEG Tactical HUD Canvas renderer.
 * Projects robot perspective floor/ceiling grid + LiDAR front-arc obstacles
 * with corner brackets, center reticle, scanline, and standby/fault overlay.
 */

export function renderMjpegHud(canvas, { telemetry, state, scanlinePhase = 0, standby = false }) {
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  const W = 640;
  const H = 480;
  if (canvas.width !== W || canvas.height !== H) {
    canvas.width = W;
    canvas.height = H;
  }

  // Base tactical sensor background
  const grad = ctx.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0, '#070C14');
  grad.addColorStop(0.46, '#0D1728');
  grad.addColorStop(0.5, '#09111E');
  grad.addColorStop(1, '#060A12');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, W, H);

  const horizonY = 216;
  const isStandby = standby || Boolean(state?.fault) || Boolean(state?.telemetryStale) || !state?.lidarOk;

  if (isStandby) {
    // Subtle standby noise bars
    ctx.fillStyle = 'rgba(15, 23, 42, 0.65)';
    for (let y = 0; y < H; y += 6) {
      ctx.fillRect(0, y, W, 2);
    }
    drawReticleAndBrackets(ctx, W, H, '#F59E0B');

    ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
    ctx.strokeStyle = state?.fault ? '#EF4444' : '#F59E0B';
    ctx.lineWidth = 1.5;
    ctx.fillRect(W / 2 - 140, H / 2 - 34, 280, 68);
    ctx.strokeRect(W / 2 - 140, H / 2 - 34, 280, 68);

    ctx.fillStyle = state?.fault ? '#EF4444' : '#F59E0B';
    ctx.font = '700 15px ui-monospace, Consolas, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(state?.fault ? `FAULT: ${state.fault}` : 'STREAM STANDBY / NO SIGNAL', W / 2, H / 2 - 4);
    ctx.fillStyle = '#94A3B8';
    ctx.font = '500 11px ui-monospace, Consolas, monospace';
    ctx.fillText('Auto-reconnect backoff active · Press Reload', W / 2, H / 2 + 18);
    ctx.textAlign = 'left';
    return;
  }

  // Perspective corridor floor & ceiling grid linked to robot pose
  const pose = state?.pose ?? { x: 0, y: 0, th: Math.PI / 2 };
  const phaseForward = ((pose.x * Math.cos(pose.th) + pose.y * Math.sin(pose.th)) * 2.2) % 1;
  const yawOffset = Math.sin(pose.th - Math.PI / 2) * 38;

  ctx.strokeStyle = 'rgba(37, 99, 235, 0.25)';
  ctx.lineWidth = 1;

  // Radial perspective depth lines
  for (let i = -5; i <= 5; i++) {
    const vanishX = W / 2 - yawOffset;
    const floorX = W / 2 + i * 95;
    ctx.beginPath();
    ctx.moveTo(vanishX, horizonY);
    ctx.lineTo(floorX, H);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(vanishX, horizonY);
    ctx.lineTo(floorX, 0);
    ctx.stroke();
  }

  // Scrolling horizontal depth bands
  for (let k = 1; k <= 8; k++) {
    const t = (k - ((phaseForward % 1) + 1) % 1) / 8;
    if (t <= 0.02) continue;
    const persp = t * t;
    const fy = horizonY + persp * (H - horizonY);
    const cy = horizonY - persp * horizonY;
    ctx.strokeStyle = `rgba(59, 130, 246, ${0.08 + persp * 0.22})`;
    ctx.beginPath();
    ctx.moveTo(0, fy);
    ctx.lineTo(W, fy);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(W, cy);
    ctx.stroke();
  }

  // Horizon guide
  ctx.strokeStyle = 'rgba(96, 165, 250, 0.35)';
  ctx.beginPath();
  ctx.moveTo(0, horizonY);
  ctx.lineTo(W, horizonY);
  ctx.stroke();

  // Project front FOV LiDAR rays (-48 deg .. +48 deg) as perspective obstacle blocks
  const ranges = telemetry?.ranges ?? [];
  const n = ranges.length;
  if (n > 0) {
    const fovHalfDeg = 48;
    const stepDeg = 360 / n;
    for (let deg = -fovHalfDeg; deg <= fovHalfDeg; deg += 3) {
      const normDeg = ((deg % 360) + 360) % 360;
      const idx = Math.round(normDeg / stepDeg) % n;
      const d = ranges[idx];
      if (!Number.isFinite(d) || d >= 3.2) continue;

      // Positive relDeg is left (+CCW), so screen X is mirrored: left is smaller X
      const screenX = W / 2 - (deg / fovHalfDeg) * (W * 0.46);
      const boxH = Math.min(260, Math.max(24, 155 / Math.max(0.22, d)));
      const boxW = Math.max(10, 26 / Math.max(0.35, d));

      let stroke = '#00E5FF';
      let fill = 'rgba(0, 229, 255, 0.14)';
      if (d < 0.40) {
        stroke = '#EF4444';
        fill = 'rgba(239, 68, 68, 0.28)';
      } else if (d < 0.70) {
        stroke = '#F59E0B';
        fill = 'rgba(245, 158, 11, 0.20)';
      }

      ctx.fillStyle = fill;
      ctx.strokeStyle = stroke;
      ctx.lineWidth = 1.2;
      ctx.fillRect(screenX - boxW / 2, horizonY - boxH * 0.45, boxW, boxH);
      ctx.strokeRect(screenX - boxW / 2, horizonY - boxH * 0.45, boxW, boxH);
    }
  }

  // Proximity alert banner inside optical HUD if front zone < 40 cm
  const frontCm = telemetry?.zones?.front ?? 999;
  const flCm = telemetry?.zones?.front_left ?? 999;
  const frCm = telemetry?.zones?.front_right ?? 999;
  const minFront = Math.min(frontCm, flCm, frCm);
  if (minFront < 40.0) {
    ctx.fillStyle = 'rgba(220, 38, 38, 0.85)';
    ctx.fillRect(W / 2 - 145, 22, 290, 28);
    ctx.fillStyle = '#F8FAFC';
    ctx.font = '700 12px ui-monospace, Consolas, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`PROXIMITY ALERT < 40 CM (${minFront.toFixed(1)} cm)`, W / 2, 40);
    ctx.textAlign = 'left';
  }

  // Animated translucent scanline
  const scanY = (scanlinePhase * H) % H;
  ctx.fillStyle = 'rgba(96, 165, 250, 0.14)';
  ctx.fillRect(0, scanY, W, 2);

  // Corner L-brackets & center reticle
  drawReticleAndBrackets(ctx, W, H, '#60A5FA');

  // Bottom HUD telemetry overlay
  ctx.fillStyle = 'rgba(8, 13, 22, 0.75)';
  ctx.fillRect(14, H - 34, W - 28, 22);
  ctx.fillStyle = '#94A3B8';
  ctx.font = '600 11px ui-monospace, Consolas, monospace';
  const vCmd = state?.cmdV ?? 0;
  const wCmd = state?.cmdW ?? 0;
  ctx.fillText(
    `OPTICAL HUD · V=${vCmd.toFixed(3)} m/s · W=${wCmd.toFixed(3)} rad/s · MIN=${(telemetry?.min_m ?? 999).toFixed(2)} m`,
    22,
    H - 19
  );
}

function drawReticleAndBrackets(ctx, W, H, color) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  const m = 16;
  const len = 24;

  // Top-left
  ctx.beginPath();
  ctx.moveTo(m, m + len);
  ctx.lineTo(m, m);
  ctx.lineTo(m + len, m);
  ctx.stroke();

  // Top-right
  ctx.beginPath();
  ctx.moveTo(W - m - len, m);
  ctx.lineTo(W - m, m);
  ctx.lineTo(W - m, m + len);
  ctx.stroke();

  // Bottom-left
  ctx.beginPath();
  ctx.moveTo(m, H - m - len);
  ctx.lineTo(m, H - m);
  ctx.lineTo(m + len, H - m);
  ctx.stroke();

  // Bottom-right
  ctx.beginPath();
  ctx.moveTo(W - m - len, H - m);
  ctx.lineTo(W - m, H - m);
  ctx.lineTo(W - m, H - m - len);
  ctx.stroke();

  // Center crosshair
  ctx.strokeStyle = 'rgba(96, 165, 250, 0.65)';
  ctx.lineWidth = 1.2;
  const cx = W / 2;
  const cy = H / 2 - 24;
  ctx.beginPath();
  ctx.moveTo(cx - 18, cy);
  ctx.lineTo(cx - 6, cy);
  ctx.moveTo(cx + 6, cy);
  ctx.lineTo(cx + 18, cy);
  ctx.moveTo(cx, cy - 18);
  ctx.lineTo(cx, cy - 6);
  ctx.moveTo(cx, cy + 6);
  ctx.lineTo(cx, cy + 18);
  ctx.stroke();
}
