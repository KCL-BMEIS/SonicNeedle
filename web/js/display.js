// The main canvas: an ultrasound "M-mode" echo image scrolling through time on the
// left, and a side-on view of the needle tip, its ultrasound pings and the target on
// the right. Both share the same vertical scale: depth below the needle tip in cm.
import { HIT_RGB, proximityRGB, rgba } from './colors.js';

const FONT = 'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
const PING_EVERY_N_READINGS = 4;
const ECHO_WIDTH_CM = 0.3;

export class Display {
  constructor(canvas, historySeconds) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.historySeconds = historySeconds;
    this.minCm = -2;
    this.maxCm = 30;
    this.trace = [];  // {t, d}, d = null where there was no echo
    this.pings = [];  // {born, d}
    this.readingCount = 0;
    this.lastNeedleCm = null;
    this.lastRender = null;
    this.shiftRemainder = 0;
    this.mmode = document.createElement('canvas');
    this.mctx = this.mmode.getContext('2d');
    this.rowNoise = new Float32Array(0);
    this.resize();
    new ResizeObserver(() => this.resize()).observe(canvas);
  }

  setRange(minCm, maxCm) {
    if (minCm === this.minCm && maxCm === this.maxCm) return;
    this.minCm = minCm;
    this.maxCm = maxCm;
    this.mctx.clearRect(0, 0, this.mmode.width, this.mmode.height);
  }

  addReading(t, distanceCm) {
    this.trace.push({ t, d: distanceCm });
    while (this.trace.length && this.trace[0].t < t - this.historySeconds - 1) this.trace.shift();
    if (++this.readingCount % PING_EVERY_N_READINGS === 0) {
      this.pings.push({ born: t, d: distanceCm == null ? null : Math.max(distanceCm, 0) });
    }
  }

  resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const W = Math.max(1, Math.round(this.canvas.clientWidth * dpr));
    const H = Math.max(1, Math.round(this.canvas.clientHeight * dpr));
    if (W === this.canvas.width && H === this.canvas.height) return;
    this.canvas.width = W;
    this.canvas.height = H;
    this.dpr = dpr;

    const unit = H / 100;
    this.fontSize = Math.max(11 * dpr, unit * 2.4);
    const gutter = this.fontSize * 3.4;
    const padTop = this.fontSize * 2.8;
    const padBottom = this.fontSize * 2.6;
    const colW = Math.min(Math.max(W * 0.24, 150 * dpr), 440 * dpr);
    const scopeW = W - gutter - colW - unit * 1.5;
    this.scope = { x: gutter, y: padTop, w: scopeW, h: H - padTop - padBottom };
    this.col = { x: this.scope.x + scopeW, y: padTop, w: colW, h: this.scope.h };

    // Keep the echo history across resizes by rescaling it
    const old = document.createElement('canvas');
    old.width = this.mmode.width;
    old.height = this.mmode.height;
    if (old.width && old.height) old.getContext('2d').drawImage(this.mmode, 0, 0);
    this.mmode.width = Math.max(1, Math.round(this.scope.w));
    this.mmode.height = Math.max(1, Math.round(this.scope.h));
    this.mctx.fillStyle = '#000';
    this.mctx.fillRect(0, 0, this.mmode.width, this.mmode.height);
    if (old.width > 1 && old.height > 1) {
      this.mctx.drawImage(old, 0, 0, this.mmode.width, this.mmode.height);
    }
    this.rowNoise = Float32Array.from({ length: this.mmode.height }, () => Math.random());
  }

  // Distance to the target runs upwards: 0 cm (touching) near the bottom
  yOf(cm) {
    return this.scope.y + ((this.maxCm - cm) / (this.maxCm - this.minCm)) * this.scope.h;
  }

  render(now, view) {
    const dt = this.lastRender == null ? 0 : Math.min(now - this.lastRender, 0.25);
    this.lastRender = now;
    this.advanceEchoImage(dt, view.distance);

    const { ctx, canvas } = this;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    this.drawScope(now, view);
    this.drawNeedleView(now, view);
  }

  // ---- M-mode echo image -------------------------------------------------

  advanceEchoImage(dt, distanceCm) {
    const pxPerSecond = this.mmode.width / this.historySeconds;
    this.shiftRemainder += dt * pxPerSecond;
    const n = Math.min(Math.floor(this.shiftRemainder), this.mmode.width);
    if (n <= 0) return;
    this.shiftRemainder -= n;
    const m = this.mctx;
    m.globalCompositeOperation = 'copy';
    m.drawImage(this.mmode, -n, 0);
    m.globalCompositeOperation = 'source-over';
    m.putImageData(this.echoColumns(n, distanceCm), this.mmode.width - n, 0);
  }

  echoColumns(n, d) {
    const h = this.mmode.height;
    const img = this.mctx.createImageData(n, h);
    const px = img.data;
    const span = this.maxCm - this.minCm;
    const noise = this.rowNoise;
    for (let y = 0; y < h; y++) {
      const cm = this.maxCm - ((y + 0.5) / h) * span;
      for (let x = 0; x < n; x++) {
        // A reflectivity per row that changes only occasionally gives the horizontal
        // speckle streaks of a real M-mode image
        if (Math.random() < 0.01) noise[y] = Math.random();
        const r = noise[y];
        const grain = 0.6 + 0.8 * Math.random();
        let v;
        if (cm < 0) {
          v = 4 + Math.random() * 4;  // past the target
        } else {
          v = r * r * r * 120 * grain * Math.exp(-cm / 30);  // speckle
          if (d != null) {
            const z = (cm - d) / ECHO_WIDTH_CM;
            v += 255 * Math.exp(-z * z) * grain;
            if (cm > d) v = v * 0.7 + 45 * Math.exp(-(cm - d) / 1.2) * r * grain;  // shadow + reverberation
          }
        }
        const i = (y * n + x) * 4;
        px[i] = v * 0.78;
        px[i + 1] = v * 0.92;
        px[i + 2] = v;
        px[i + 3] = 255;
      }
    }
    return img;
  }

  // ---- Left: echo trace ----------------------------------------------------

  drawScope(now, view) {
    const { ctx, scope: s, fontSize: fs, dpr } = this;

    ctx.save();
    roundRect(ctx, s.x, s.y, s.w, s.h, 10 * dpr);
    ctx.clip();
    ctx.drawImage(this.mmode, s.x, s.y, s.w, s.h);
    const fade = ctx.createLinearGradient(s.x, 0, s.x + s.w * 0.35, 0);
    fade.addColorStop(0, 'rgba(5, 10, 18, 0.85)');
    fade.addColorStop(1, 'rgba(5, 10, 18, 0)');
    ctx.fillStyle = fade;
    ctx.fillRect(s.x, s.y, s.w, s.h);
    this.drawGrid();
    this.drawTrace(now, view);
    ctx.restore();

    // Depth labels
    ctx.font = `${fs}px ${FONT}`;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let cm = Math.ceil(this.minCm / 2) * 2; cm <= this.maxCm; cm += 2) {
      ctx.fillStyle = cm === 0 ? 'rgba(232, 241, 255, 0.95)' : 'rgba(138, 160, 189, 0.85)';
      ctx.fillText(`${cm}`, s.x - fs * 0.6, this.yOf(cm));
    }
    ctx.save();
    ctx.translate(fs * 0.9, s.y + s.h / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillStyle = 'rgba(138, 160, 189, 0.85)';
    ctx.fillText('Distance to target (cm)', 0, 0);
    ctx.restore();

    // Titles
    ctx.textBaseline = 'alphabetic';
    ctx.textAlign = 'left';
    ctx.font = `600 ${fs}px ${FONT}`;
    ctx.fillStyle = 'rgba(232, 241, 255, 0.9)';
    ctx.fillText('ECHO TRACE', s.x, s.y - fs * 0.9);
    ctx.font = `${fs * 0.9}px ${FONT}`;
    ctx.fillStyle = 'rgba(138, 160, 189, 0.9)';
    ctx.fillText(`← ${this.historySeconds} seconds ago`, s.x, s.y + s.h + fs * 1.6);
    ctx.textAlign = 'right';
    ctx.fillText('now', s.x + s.w - fs * 0.5, s.y + s.h + fs * 1.6);
  }

  drawGrid() {
    const { ctx, scope: s, dpr } = this;
    ctx.lineWidth = 1 * dpr;
    for (let cm = Math.ceil(this.minCm / 2) * 2; cm <= this.maxCm; cm += 2) {
      if (cm === 0) continue;
      const y = Math.round(this.yOf(cm)) + 0.5;
      ctx.strokeStyle = 'rgba(120, 180, 255, 0.08)';
      ctx.beginPath();
      ctx.moveTo(s.x, y);
      ctx.lineTo(s.x + s.w, y);
      ctx.stroke();
    }
    const y0 = this.yOf(0);
    ctx.strokeStyle = 'rgba(232, 241, 255, 0.45)';
    ctx.setLineDash([8 * dpr, 6 * dpr]);
    ctx.beginPath();
    ctx.moveTo(s.x, y0);
    ctx.lineTo(s.x + s.w, y0);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  drawTrace(now, view) {
    const { ctx, scope: s, dpr, trace } = this;
    const pxPerSecond = s.w / this.historySeconds;
    const xOf = (t) => s.x + s.w - (now - t) * pxPerSecond;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    for (let i = 1; i < trace.length; i++) {
      const a = trace[i - 1];
      const b = trace[i];
      if (a.d == null || b.d == null) continue;
      const age = (now - b.t) / this.historySeconds;
      if (age > 1) continue;
      const alpha = Math.max(0, 1 - age) ** 1.5;
      const rgb = proximityRGB(b.d);
      ctx.beginPath();
      ctx.moveTo(xOf(a.t), this.yOf(a.d));
      ctx.lineTo(xOf(b.t), this.yOf(b.d));
      ctx.strokeStyle = rgba(rgb, alpha * 0.22);
      ctx.lineWidth = 12 * dpr;
      ctx.stroke();
      ctx.strokeStyle = rgba(rgb, alpha);
      ctx.lineWidth = 3.5 * dpr;
      ctx.stroke();
    }

    if (view.distance != null) {
      const rgb = view.hit ? HIT_RGB : proximityRGB(view.distance);
      const x = s.x + s.w;
      const y = this.yOf(view.distance);
      glowDot(ctx, x, y, 9 * dpr, rgb);
    }
  }

  // ---- Right: needle, pings and target -----------------------------------

  // The target sits on the 0 cm line and the needle moves down towards it, on the same
  // scale as the echo trace, so the needle tip lines up with the head of the trace.
  needleViewLayout(distanceCm) {
    const pxPerCm = this.scope.h / (this.maxCm - this.minCm);
    if (distanceCm != null) this.lastNeedleCm = distanceCm;
    // Hold the needle where it was last seen, and keep its tip in view
    const needleCm = Math.min(this.lastNeedleCm ?? this.maxCm, this.maxCm);
    return { targetY: this.yOf(0), pxPerCm, tipY: this.yOf(needleCm) };
  }

  drawNeedleView(now, view) {
    const { ctx, col: c, dpr, fontSize: fs } = this;
    const cx = c.x + c.w * 0.5;
    const { targetY, pxPerCm, tipY } = this.needleViewLayout(view.distance);

    ctx.save();
    roundRect(ctx, c.x, c.y, c.w, c.h, 10 * dpr);
    const bg = ctx.createLinearGradient(0, c.y, 0, c.y + c.h);
    bg.addColorStop(0, '#0d1d2e');
    bg.addColorStop(1, '#081220');
    ctx.fillStyle = bg;
    ctx.fill();
    ctx.clip();

    const hasEcho = view.distance != null;
    const targetRGB = view.hit ? HIT_RGB : proximityRGB(view.distance);

    // Beam between tip and target
    if (hasEcho && targetY > tipY) {
      const beam = ctx.createLinearGradient(0, tipY, 0, targetY);
      beam.addColorStop(0, 'rgba(56, 225, 255, 0.10)');
      beam.addColorStop(1, rgba(targetRGB, 0.18));
      ctx.fillStyle = beam;
      const halfTop = c.w * 0.05;
      const halfBottom = c.w * 0.22;
      ctx.beginPath();
      ctx.moveTo(cx - halfTop, tipY);
      ctx.lineTo(cx + halfTop, tipY);
      ctx.lineTo(cx + halfBottom, targetY);
      ctx.lineTo(cx - halfBottom, targetY);
      ctx.closePath();
      ctx.fill();
    }

    this.drawPings(now, cx, tipY, targetY, pxPerCm);
    this.drawTarget(now, cx, targetY, targetRGB, view.hit);
    this.drawNeedle(now, cx, tipY, view.hit);
    this.drawGap(cx + c.w * 0.3, tipY, targetY, hasEcho ? view.distance : null, targetRGB);
    ctx.restore();

    // Labels
    ctx.font = `600 ${fs}px ${FONT}`;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'alphabetic';
    ctx.fillStyle = 'rgba(232, 241, 255, 0.9)';
    ctx.fillText('NEEDLE VIEW', c.x + fs * 0.2, c.y - fs * 0.9);
    // Beside the target (there's little room below it), shrunk if the view is narrow
    const clearOfTarget = cx - c.w * 0.13 * 1.25;
    let size = fs * 0.85;
    ctx.font = `${size}px ${FONT}`;
    const room = clearOfTarget - (c.x + fs * 0.4);
    const width = ctx.measureText('TARGET').width;
    if (width > room) size = Math.max(size * room / width, fs * 0.55);
    ctx.font = `${size}px ${FONT}`;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = rgba(targetRGB, 0.95);
    ctx.fillText('TARGET', clearOfTarget, targetY);
    ctx.textBaseline = 'alphabetic';
  }

  // A dimension line from the needle tip down to the target, labelled with the gap
  drawGap(x, tipY, targetY, distanceCm, rgb) {
    const { ctx, dpr, fontSize: fs } = this;
    const midY = (tipY + targetY) / 2;
    const label = distanceCm == null ? 'NO ECHO' : `${Math.max(distanceCm, 0).toFixed(1)} cm`;
    if (distanceCm != null && targetY - tipY > fs * 2.5) {
      const tick = fs * 0.35;
      ctx.strokeStyle = rgba(rgb, 0.6);
      ctx.lineWidth = 1.5 * dpr;
      ctx.setLineDash([4 * dpr, 4 * dpr]);
      ctx.beginPath();
      ctx.moveTo(x, tipY);
      ctx.lineTo(x, targetY);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath();
      ctx.moveTo(x - tick, tipY);
      ctx.lineTo(x + tick, tipY);
      ctx.moveTo(x - tick, targetY);
      ctx.lineTo(x + tick, targetY);
      ctx.stroke();
    } else if (distanceCm != null) {
      return;  // too close to fit a label; the target glow says it all
    }
    ctx.font = `600 ${fs * 0.85}px ${FONT}`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const w = ctx.measureText(label).width + fs * 0.8;
    const h = fs * 1.5;
    ctx.fillStyle = 'rgba(8, 18, 32, 0.85)';
    roundRect(ctx, x - w / 2, midY - h / 2, w, h, h / 2);
    ctx.fill();
    ctx.fillStyle = rgba(rgb, 0.95);
    ctx.fillText(label, x, midY);
    ctx.textBaseline = 'alphabetic';
  }

  drawNeedle(now, cx, tipY, hit) {
    const { ctx, col: c, dpr } = this;
    const w = Math.max(10 * dpr, c.w * 0.075);
    const bevel = w * 2.4;
    const left = cx - w / 2;
    const right = cx + w / 2;
    const top = c.y - 2;

    const metal = ctx.createLinearGradient(left, 0, right, 0);
    metal.addColorStop(0, '#5b6673');
    metal.addColorStop(0.3, '#e9eef4');
    metal.addColorStop(0.55, '#aab4bf');
    metal.addColorStop(1, '#4a535e');
    ctx.fillStyle = metal;
    ctx.beginPath();
    ctx.moveTo(left, top);
    ctx.lineTo(right, top);
    ctx.lineTo(right, tipY - bevel);
    ctx.lineTo(left, tipY);
    ctx.closePath();
    ctx.fill();

    // Bevel face
    ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
    ctx.beginPath();
    ctx.moveTo(right, tipY - bevel);
    ctx.lineTo(left + w * 0.25, tipY - bevel * 0.12);
    ctx.lineTo(left, tipY);
    ctx.closePath();
    ctx.fill();

    // Ultrasound sensor near the tip, flashing as each ping leaves
    const lastPing = this.pings.length ? this.pings[this.pings.length - 1].born : -Infinity;
    const flash = Math.min(Math.max(0, 1 - (now - lastPing) * 5), 1);
    const sensorRGB = hit ? HIT_RGB : [56, 225, 255];
    glowDot(ctx, cx - w * 0.1, tipY - bevel * 0.85, w * (0.22 + 0.12 * flash), sensorRGB, 0.6 + 0.4 * flash);
    if (hit) glowDot(ctx, left, tipY, w * 1.2, HIT_RGB, 0.8);
    ctx.lineWidth = 1 * dpr;
  }

  drawPings(now, cx, tipY, targetY, pxPerCm) {
    const { ctx, col: c, dpr } = this;
    const speed = c.h * 1.1;  // px per second, slowed right down so you can see it
    const spread = 0.55;
    this.pings = this.pings.filter((p) => now - p.born < 2.5);
    ctx.lineWidth = 2.5 * dpr;
    for (const p of this.pings) {
      const travelled = Math.max(now - p.born, 0) * speed;
      // Each ping leaves from where the tip was when it was sent
      const gap = p.d == null ? Infinity : p.d * pxPerCm;
      const originY = p.d == null ? tipY : targetY - gap;
      if (travelled < gap) {
        const fade = 1 - travelled / (c.h * 1.1);
        if (fade <= 0) continue;
        ctx.strokeStyle = `rgba(200, 245, 255, ${0.7 * fade})`;
        ctx.beginPath();
        ctx.arc(cx, originY, travelled, Math.PI / 2 - spread, Math.PI / 2 + spread);
        ctx.stroke();
      } else {
        const back = travelled - gap;
        if (back > gap) continue;
        const rgb = proximityRGB(p.d);
        ctx.strokeStyle = rgba(rgb, 0.85 * (1 - back / Math.max(gap, 1)) + 0.15);
        ctx.beginPath();
        ctx.arc(cx, targetY, back, -Math.PI / 2 - spread, -Math.PI / 2 + spread);
        ctx.stroke();
      }
    }
  }

  drawTarget(now, cx, y, rgb, hit) {
    const { ctx, col: c } = this;
    const r = c.w * 0.13 * (1 + 0.06 * Math.sin(now * (hit ? 14 : 4)));
    const glow = ctx.createRadialGradient(cx, y, 0, cx, y, r * (hit ? 3.2 : 2.3));
    glow.addColorStop(0, rgba(rgb, hit ? 0.6 : 0.4));
    glow.addColorStop(1, rgba(rgb, 0));
    ctx.fillStyle = glow;
    ctx.beginPath();
    ctx.arc(cx, y, r * (hit ? 3.2 : 2.3), 0, Math.PI * 2);
    ctx.fill();

    ctx.lineWidth = Math.max(2, r * 0.12);
    ctx.strokeStyle = rgba(rgb, 0.9);
    ctx.beginPath();
    ctx.ellipse(cx, y, r, r * 0.55, 0, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = rgba(rgb, 0.95);
    ctx.beginPath();
    ctx.ellipse(cx, y, r * 0.45, r * 0.25, 0, 0, Math.PI * 2);
    ctx.fill();
  }
}

function glowDot(ctx, x, y, r, rgb, alpha = 1) {
  const g = ctx.createRadialGradient(x, y, 0, x, y, r * 3);
  g.addColorStop(0, rgba(rgb, alpha));
  g.addColorStop(0.3, rgba(rgb, alpha * 0.5));
  g.addColorStop(1, rgba(rgb, 0));
  ctx.fillStyle = g;
  ctx.beginPath();
  ctx.arc(x, y, r * 3, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = `rgba(255, 255, 255, ${alpha})`;
  ctx.beginPath();
  ctx.arc(x, y, r * 0.45, 0, Math.PI * 2);
  ctx.fill();
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}
