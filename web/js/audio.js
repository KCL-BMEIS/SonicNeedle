// Parking-sensor style sonar pings that speed up and rise in pitch as the needle
// nears the target, plus a fanfare when it is reached.
const FAR_CM = 15;

export class Sonar {
  constructor() {
    this.ctx = null;
    this.master = null;
    this.muted = false;
    this.nextPing = 0;
    this.ensureContext();
  }

  get ready() {
    return this.ctx != null && this.ctx.state === 'running';
  }

  ensureContext() {
    if (this.ctx) return;
    try {
      this.ctx = new AudioContext();
      this.master = this.ctx.createGain();
      this.master.gain.value = 0.35;
      this.master.connect(this.ctx.destination);
    } catch (e) {
      this.ctx = null;
    }
  }

  unlock() {
    this.ensureContext();
    if (this.ctx && this.ctx.state === 'suspended') this.ctx.resume();
  }

  toggleMute() {
    this.muted = !this.muted;
    return this.muted;
  }

  update(now, distance, enabled) {
    if (!enabled || distance == null) {
      this.nextPing = now;
      return;
    }
    if (now < this.nextPing) return;
    const far = Math.min(Math.max(distance / FAR_CM, 0), 1);
    this.ping(1500 - 800 * far);
    this.nextPing = now + 0.08 + 0.8 * far;
  }

  ping(freq) {
    if (this.muted || !this.ready) return;
    const t = this.ctx.currentTime;
    const osc = this.ctx.createOscillator();
    const gain = this.ctx.createGain();
    osc.type = 'sine';
    osc.frequency.setValueAtTime(freq, t);
    osc.frequency.exponentialRampToValueAtTime(freq * 0.85, t + 0.12);
    gain.gain.setValueAtTime(0.0001, t);
    gain.gain.exponentialRampToValueAtTime(0.5, t + 0.005);
    gain.gain.exponentialRampToValueAtTime(0.0001, t + 0.15);
    osc.connect(gain).connect(this.master);
    osc.start(t);
    osc.stop(t + 0.16);
  }

  fanfare() {
    if (this.muted || !this.ready) return;
    const t0 = this.ctx.currentTime;
    [523.25, 659.25, 783.99, 1046.5].forEach((freq, i) => {
      const last = i === 3;
      const t = t0 + i * 0.11;
      for (const type of ['triangle', 'sine']) {
        const osc = this.ctx.createOscillator();
        const gain = this.ctx.createGain();
        osc.type = type;
        osc.frequency.value = type === 'sine' ? freq * 2 : freq;
        const peak = type === 'sine' ? 0.12 : 0.4;
        const length = last ? 1.0 : 0.3;
        gain.gain.setValueAtTime(0.0001, t);
        gain.gain.exponentialRampToValueAtTime(peak, t + 0.01);
        gain.gain.exponentialRampToValueAtTime(0.0001, t + length);
        osc.connect(gain).connect(this.master);
        osc.start(t);
        osc.stop(t + length + 0.05);
      }
    });
  }
}
