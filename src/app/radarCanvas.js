/**
 * 2D LiDAR / SLAM Radar Canvas renderer and interactive map click mapper.
 * Coordinate convention: +X East (right), +Y MAP North (up), CCW heading `th`.
 */

import { OBSTACLE_THRESHOLD_M } from '../core/safety.js';

const SECTOR_ANGLES_DEG = [
  { zone: 'front', start: -22.5, end: 22.5 },
  { zone: 'front_left', start: 22.5, end: 67.5 },
  { zone: 'left', start: 67.5, end: 112.5 },
  { zone: 'back_left', start: 112.5, end: 157.5 },
  { zone: 'back', start: 157.5, end: 202.5 },
  { zone: 'back_right', start: 202.5, end: 247.5 },
  { zone: 'right', start: 247.5, end: 292.5 },
  { zone: 'front_right', start: 292.5, end: 337.5 },
];

export function getRadarTransform(cssWidth, cssHeight, pose) {
  const cx = cssWidth / 2;
  const cy = cssHeight / 2;
  const maxRadiusPx = Math.min(cssWidth, cssHeight) * 0.44;
  const pxPerM = maxRadiusPx / 2.0;
  return { cx, cy, pxPerM, camX: pose.x, camY: pose.y };
}

export function canvasPointToWorld(offsetX, offsetY, cssWidth, cssHeight, pose) {
  const { cx, cy, pxPerM } = getRadarTransform(cssWidth, cssHeight, pose);
  return {
    x: pose.x + (offsetX - cx) / pxPerM,
    y: pose.y - (offsetY - cy) / pxPerM,
  };
}

/**
 * Renders occupancy grid, range rings, 0.40 m safety ring, 8-sector alerts,
 * 360-deg LiDAR points, trail, obstacles, and robot pose chevron.
 */
export function renderRadarCanvas(canvas, { telemetry, mapFull, state, stale = false }) {
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  const rect = canvas.getBoundingClientRect();
  const cssW = Math.max(240, Math.round(rect.width || canvas.width || 360));
  const cssH = Math.max(220, Math.round(rect.height || canvas.height || 300));
  const dpr = Math.min(2, window.devicePixelRatio || 1);

  if (canvas.width !== Math.round(cssW * dpr) || canvas.height !== Math.round(cssH * dpr)) {
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
  }

  ctx.save();
  ctx.scale(dpr, dpr);

  // 1. Background
  ctx.fillStyle = '#080D16';
  ctx.fillRect(0, 0, cssW, cssH);

  const pose = state?.pose ?? { x: 0, y: 0, th: Math.PI / 2 };
  const { cx, cy, pxPerM, camX, camY } = getRadarTransform(cssW, cssH, pose);

  // Subtle Cartesian grid every 0.5 m
  ctx.strokeStyle = 'rgba(30, 41, 59, 0.45)';
  ctx.lineWidth = 1;
  const startGridX = Math.floor((camX - cssW / (2 * pxPerM)) * 2) / 2;
  const endGridX = Math.ceil((camX + cssW / (2 * pxPerM)) * 2) / 2;
  for (let gx = startGridX; gx <= endGridX; gx += 0.5) {
    const px = cx + (gx - camX) * pxPerM;
    ctx.beginPath();
    ctx.moveTo(px, 0);
    ctx.lineTo(px, cssH);
    ctx.stroke();
  }
  const startGridY = Math.floor((camY - cssH / (2 * pxPerM)) * 2) / 2;
  const endGridY = Math.ceil((camY + cssH / (2 * pxPerM)) * 2) / 2;
  for (let gy = startGridY; gy <= endGridY; gy += 0.5) {
    const py = cy - (gy - camY) * pxPerM;
    ctx.beginPath();
    ctx.moveTo(0, py);
    ctx.lineTo(cssW, py);
    ctx.stroke();
  }

  // 2. SLAM Occupancy Grid (validated bounds)
  if (
    mapFull &&
    mapFull.w > 0 &&
    mapFull.h > 0 &&
    mapFull.res > 0 &&
    mapFull.w * mapFull.h <= 4_000_000 &&
    Array.isArray(mapFull.data) &&
    mapFull.data.length === mapFull.w * mapFull.h
  ) {
    const cellPx = Math.max(1.5, mapFull.res * pxPerM);
    for (let gy = 0; gy < mapFull.h; gy++) {
      const wy = mapFull.oy + (gy + 0.5) * mapFull.res;
      const py = cy - (wy - camY) * pxPerM;
      if (py < -cellPx || py > cssH + cellPx) continue;
      const rowOffset = gy * mapFull.w;
      for (let gx = 0; gx < mapFull.w; gx++) {
        const val = mapFull.data[rowOffset + gx];
        if (val < 0) continue;
        const wx = mapFull.ox + (gx + 0.5) * mapFull.res;
        const px = cx + (wx - camX) * pxPerM;
        if (px < -cellPx || px > cssW + cellPx) continue;
        if (val > 50) {
          ctx.fillStyle = 'rgba(0, 229, 255, 0.72)';
        } else {
          ctx.fillStyle = 'rgba(30, 58, 95, 0.36)';
        }
        ctx.fillRect(px - cellPx / 2, py - cellPx / 2, cellPx + 0.5, cellPx + 0.5);
      }
    }
  }

  // 3. 8-Zone Sector Wedges (highlight < 40 cm sectors in translucent red)
  const zones = telemetry?.zones ?? state?.zones ?? {};
  for (const sector of SECTOR_ANGLES_DEG) {
    const zDistCm = zones[sector.zone] ?? 999;
    const isBlocked = zDistCm < 40.0;
    const a1 = -(pose.th + (sector.end * Math.PI) / 180);
    const a2 = -(pose.th + (sector.start * Math.PI) / 180);

    if (isBlocked) {
      ctx.fillStyle = 'rgba(239, 68, 68, 0.20)';
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, 0.55 * pxPerM, a1, a2, false);
      ctx.closePath();
      ctx.fill();
    }

    // Subtle radial divider
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.16)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(a2) * 2.0 * pxPerM, cy + Math.sin(a2) * 2.0 * pxPerM);
    ctx.stroke();
  }

  // 4. Concentric Range Rings (0.5, 1.0, 1.5, 2.0 m) + Dashed 0.40 m Safety Ring
  ctx.font = '600 10px ui-monospace, Consolas, monospace';
  ctx.textBaseline = 'middle';
  for (const rM of [0.5, 1.0, 1.5, 2.0]) {
    ctx.strokeStyle = rM === 2.0 ? '#26354D' : '#1E293B';
    ctx.lineWidth = rM === 2.0 ? 1.5 : 1;
    ctx.beginPath();
    ctx.arc(cx, cy, rM * pxPerM, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = '#64748B';
    ctx.fillText(`${rM.toFixed(1)}m`, cx + 4, cy - rM * pxPerM + 8);
  }

  // Dashed 0.40 m safety interlock ring
  ctx.save();
  ctx.setLineDash([4, 4]);
  ctx.strokeStyle = 'rgba(239, 68, 68, 0.78)';
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  ctx.arc(cx, cy, OBSTACLE_THRESHOLD_M * pxPerM, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();

  // 5. Render physical obstacles subtly for spatial clarity (custom / dynamic / static)
  const obstacles = state?.world?.obstacles ?? [];
  for (const obs of obstacles) {
    if (obs.enabled === false) continue;
    if (obs.type === 'circle') {
      const ox = cx + (obs.x - camX) * pxPerM;
      const oy = cy - (obs.y - camY) * pxPerM;
      ctx.fillStyle = obs.custom
        ? 'rgba(245, 158, 11, 0.25)'
        : obs.dynamic
          ? 'rgba(239, 68, 68, 0.25)'
          : 'rgba(96, 165, 250, 0.16)';
      ctx.strokeStyle = obs.custom ? '#F59E0B' : obs.dynamic ? '#EF4444' : '#3B82F6';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(ox, oy, obs.r * pxPerM, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    } else if (obs.type === 'rect') {
      const ox = cx + (obs.x - camX) * pxPerM;
      const oy = cy - (obs.y - camY) * pxPerM;
      ctx.save();
      ctx.translate(ox, oy);
      ctx.rotate(-(obs.yaw ?? 0));
      ctx.fillStyle = 'rgba(37, 99, 235, 0.15)';
      ctx.strokeStyle = 'rgba(96, 165, 250, 0.55)';
      ctx.lineWidth = 1;
      ctx.fillRect((-obs.w * pxPerM) / 2, (-obs.h * pxPerM) / 2, obs.w * pxPerM, obs.h * pxPerM);
      ctx.strokeRect((-obs.w * pxPerM) / 2, (-obs.h * pxPerM) / 2, obs.w * pxPerM, obs.h * pxPerM);
      ctx.restore();
    }
  }

  // 6. Trajectory Breadcrumb Trail
  const trail = state?.trail ?? [];
  if (trail.length > 1) {
    ctx.strokeStyle = 'rgba(59, 130, 246, 0.55)';
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    for (let i = 0; i < trail.length; i++) {
      const tx = cx + (trail[i].x - camX) * pxPerM;
      const ty = cy - (trail[i].y - camY) * pxPerM;
      if (i === 0) ctx.moveTo(tx, ty);
      else ctx.lineTo(tx, ty);
    }
    ctx.stroke();
  }

  // 7. 360-deg LiDAR Scan Points (0.05..3.2 m)
  const ranges = telemetry?.ranges ?? [];
  const minAng = telemetry?.min_ang ?? 0;
  const n = ranges.length;
  const stepDeg = n > 0 ? 360 / n : 1;
  for (let i = 0; i < n; i++) {
    const d = ranges[i];
    if (!Number.isFinite(d) || d < 0.05 || d > 3.2) continue;
    const bearing = pose.th + ((minAng + i * stepDeg) * Math.PI) / 180;
    const wx = pose.x + Math.cos(bearing) * d;
    const wy = pose.y + Math.sin(bearing) * d;
    const px = cx + (wx - camX) * pxPerM;
    const py = cy - (wy - camY) * pxPerM;
    if (d < 0.35) ctx.fillStyle = '#EF4444';
    else if (d < 0.70) ctx.fillStyle = '#F59E0B';
    else ctx.fillStyle = '#10B981';
    ctx.fillRect(px - 1.7, py - 1.7, 3.4, 3.4);
  }

  // 8. Robot Footprint & Directional Chevron
  if (telemetry) {
    const rPx = Math.max(10, 0.14 * pxPerM);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(Math.PI / 2 - pose.th);

    // Body circle
    ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
    ctx.strokeStyle = '#60A5FA';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, rPx, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Forward chevron pointing UP in rotated local frame
    ctx.fillStyle = '#F8FAFC';
    ctx.beginPath();
    ctx.moveTo(0, -rPx * 0.92);
    ctx.lineTo(rPx * 0.56, rPx * 0.55);
    ctx.lineTo(0, rPx * 0.25);
    ctx.lineTo(-rPx * 0.56, rPx * 0.55);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // 9. +Y MAP Axis Indicator & Stale Pill
  ctx.fillStyle = '#60A5FA';
  ctx.font = '700 11px ui-monospace, Consolas, monospace';
  ctx.fillText('+Y MAP', cx - 22, 14);

  if (stale || state?.telemetryStale || !state?.lidarOk) {
    ctx.fillStyle = 'rgba(245, 158, 11, 0.92)';
    ctx.fillRect(cssW - 96, 8, 88, 20);
    ctx.fillStyle = '#080D16';
    ctx.font = '700 10px ui-monospace, Consolas, monospace';
    ctx.fillText('STALE DATA', cssW - 88, 18);
  }

  ctx.restore();
}
