// Turns a stream of distances into rounds for each visitor:
//   ready     -> waiting for the needle to approach the target
//   active    -> timing the attempt
//   celebrate -> target touched
//   withdraw  -> waiting for the needle to be pulled back for the next visitor
const STATS_KEY = 'sonic-needle-stats';

export class Game {
  constructor(config, onHit) {
    this.config = config;
    this.onHit = onHit;
    this.state = 'ready';
    this.since = 0;
    this.startTime = null;
    this.result = null;
    this.farSince = null;
    this.lostSince = null;
    this.stats = loadStats();
  }

  elapsed(now) {
    if (this.state === 'active') return now - this.startTime;
    if (this.state === 'celebrate' || this.state === 'withdraw') return this.result?.elapsed ?? null;
    return null;
  }

  reading(now, distance, hit) {
    const c = this.config;
    if (distance == null) {
      this.lostSince ??= now;
    } else {
      this.lostSince = null;
      this.farSince = distance > c.resetAboveCm ? (this.farSince ?? now) : null;
    }
    const farFor = this.farSince == null ? 0 : now - this.farSince;
    const lostFor = this.lostSince == null ? 0 : now - this.lostSince;

    switch (this.state) {
      case 'ready':
        if (hit) {
          this.finish(now, null);
        } else if (distance != null && distance < c.startBelowCm) {
          this.startTime = now;
          this.setState('active', now);
        }
        break;
      case 'active':
        if (hit) this.finish(now, now - this.startTime);
        else if (farFor > c.abandonAfterSeconds) this.setState('ready', now);
        break;
      case 'withdraw':
        if (!hit && (farFor > 0.5 || lostFor > 2)) this.setState('ready', now);
        break;
    }
  }

  tick(now) {
    const c = this.config;
    if (this.state === 'celebrate' && now - this.since > c.celebrateSeconds) {
      this.setState('withdraw', now);
    } else if (this.state === 'withdraw' && now - this.since > c.withdrawTimeoutSeconds) {
      this.setState('ready', now);
    }
  }

  reset(now) {
    this.setState('ready', now);
  }

  clearStats() {
    this.stats = { date: today(), best: null, count: 0 };
    saveStats(this.stats);
  }

  finish(now, elapsed) {
    this.stats = loadStats();  // the date may have rolled over
    const newBest = elapsed != null && (this.stats.best == null || elapsed < this.stats.best);
    this.stats.count += 1;
    if (newBest) this.stats.best = elapsed;
    saveStats(this.stats);
    this.result = { elapsed, newBest };
    this.setState('celebrate', now);
    this.onHit(this.result, this.stats);
  }

  setState(state, now) {
    this.state = state;
    this.since = now;
  }
}

function today() {
  return new Date().toISOString().slice(0, 10);
}

function loadStats() {
  try {
    const stats = JSON.parse(localStorage.getItem(STATS_KEY));
    if (stats && stats.date === today()) return stats;
  } catch (e) {
    // storage unavailable or corrupt: start afresh
  }
  return { date: today(), best: null, count: 0 };
}

function saveStats(stats) {
  try {
    localStorage.setItem(STATS_KEY, JSON.stringify(stats));
  } catch (e) {
    // storage unavailable: stats only last until the page reloads
  }
}
