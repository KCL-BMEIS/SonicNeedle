// Full-screen "Target reached!" overlay with a confetti burst.
const COLORS = ['#3dffa0', '#38e1ff', '#ff4070', '#f050dc', '#9678ff', '#ffffff'];

export class Celebration {
  constructor(root) {
    this.root = root;
    this.canvas = root.querySelector('canvas');
    this.ctx = this.canvas.getContext('2d');
    this.timeEl = root.querySelector('[data-time]');
    this.bestEl = root.querySelector('[data-best]');
    this.particles = [];
    this.running = false;
  }

  show(result, stats) {
    this.timeEl.textContent = result.elapsed == null ? '' : `in ${result.elapsed.toFixed(1)} seconds`;
    if (result.newRecord) {
      this.bestEl.textContent = '★ New all-time record! ★';
    } else if (result.newBest) {
      this.bestEl.textContent = '★ New best today! ★';
    } else if (stats.best != null) {
      this.bestEl.textContent = `Best today: ${stats.best.toFixed(1)} s`;
    } else {
      this.bestEl.textContent = '';
    }
    this.bestEl.classList.toggle('is-best', result.newBest || result.newRecord);
    this.bestEl.classList.toggle('is-record', result.newRecord);
    this.root.classList.add('show');
    this.burst();
  }

  hide() {
    this.root.classList.remove('show');
  }

  burst() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.canvas.width = this.canvas.clientWidth * dpr;
    this.canvas.height = this.canvas.clientHeight * dpr;
    const { width: W, height: H } = this.canvas;
    const scale = Math.min(W, H);
    for (let i = 0; i < 220; i++) {
      const angle = Math.random() * Math.PI * 2;
      const speed = scale * (0.4 + Math.random() * 1.1);
      this.particles.push({
        x: W / 2, y: H * 0.45,
        vx: Math.cos(angle) * speed, vy: Math.sin(angle) * speed - scale * 0.5,
        size: scale * (0.006 + Math.random() * 0.01),
        spin: (Math.random() - 0.5) * 12, angle: Math.random() * Math.PI,
        color: COLORS[i % COLORS.length], life: 2.5 + Math.random() * 1.5,
      });
    }
    this.rings = [0, 0.15, 0.3].map((delay) => ({ delay }));
    this.started = performance.now() / 1000;
    if (!this.running) {
      this.running = true;
      this.last = this.started;
      requestAnimationFrame((t) => this.frame(t));
    }
  }

  frame(tms) {
    const now = tms / 1000;
    const dt = Math.min(now - this.last, 0.05);
    this.last = now;
    const { ctx, canvas } = this;
    const { width: W, height: H } = canvas;
    const scale = Math.min(W, H);
    ctx.clearRect(0, 0, W, H);

    const age = now - this.started;
    for (const ring of this.rings) {
      const t = age - ring.delay;
      if (t < 0 || t > 1.2) continue;
      ctx.strokeStyle = `rgba(61, 255, 160, ${0.6 * (1 - t / 1.2)})`;
      ctx.lineWidth = scale * 0.01;
      ctx.beginPath();
      ctx.arc(W / 2, H * 0.45, t * scale * 0.9, 0, Math.PI * 2);
      ctx.stroke();
    }

    const gravity = scale * 1.4;
    for (const p of this.particles) {
      p.life -= dt;
      p.vy += gravity * dt;
      p.vx *= 1 - 1.6 * dt;
      p.vy *= 1 - 1.6 * dt;
      p.x += p.vx * dt;
      p.y += p.vy * dt;
      p.angle += p.spin * dt;
      ctx.save();
      ctx.globalAlpha = Math.max(0, Math.min(1, p.life));
      ctx.translate(p.x, p.y);
      ctx.rotate(p.angle);
      ctx.fillStyle = p.color;
      ctx.fillRect(-p.size, -p.size * 0.5, p.size * 2, p.size);
      ctx.restore();
    }
    this.particles = this.particles.filter((p) => p.life > 0 && p.y < H + 50);

    if (this.particles.length || age < 1.6) {
      requestAnimationFrame((t) => this.frame(t));
    } else {
      this.running = false;
      ctx.clearRect(0, 0, W, H);
    }
  }
}
