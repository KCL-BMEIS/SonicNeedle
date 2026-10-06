// Turns a stream of distances into rounds for each visitor:
//   ready     -> waiting for the needle to approach the target
//   active    -> timing the attempt
//   celebrate -> target touched
//   withdraw  -> waiting for the needle to be pulled back for the next visitor
//
// Keeps today's stats and the all-time record ({best, count}) in the browser's storage,
// so they survive restarts. Today's start afresh on a new day.
const TODAY_KEY = 'sonic-needle-stats';
const ALL_TIME_KEY = 'sonic-needle-record';

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
    this.setStorageScope('sensor');
  }

  // The simulated sensor plays itself, so it keeps separate records from the real demo
  setStorageScope(mode) {
    this.keySuffix = mode === 'simulated' ? '-simulated' : '';
    this.loadRecords();
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

  clearToday() {
    this.stats = { date: today(), best: null, count: 0 };
    this.saveRecords();
  }

  clearAll() {
    this.allTime = { best: null, count: 0 };
    this.clearToday();
  }

  finish(now, elapsed) {
    this.loadRecords();  // the date may have rolled over
    const newBest = elapsed != null && (this.stats.best == null || elapsed < this.stats.best);
    const newRecord = elapsed != null && (this.allTime.best == null || elapsed < this.allTime.best);
    this.stats.count += 1;
    this.allTime.count += 1;
    if (newBest) this.stats.best = elapsed;
    if (newRecord) this.allTime.best = elapsed;
    this.saveRecords();
    this.result = { elapsed, newBest, newRecord };
    this.setState('celebrate', now);
    this.onHit(this.result, this.stats);
  }

  loadRecords() {
    const stats = load(TODAY_KEY + this.keySuffix);
    this.stats = stats && stats.date === today() ? stats : { date: today(), best: null, count: 0 };
    // Before there was an all-time record, start it from today's stats
    this.allTime = load(ALL_TIME_KEY + this.keySuffix)
      ?? { best: this.stats.best, count: this.stats.count };
  }

  saveRecords() {
    save(TODAY_KEY + this.keySuffix, this.stats);
    save(ALL_TIME_KEY + this.keySuffix, this.allTime);
  }

  setState(state, now) {
    this.state = state;
    this.since = now;
  }
}

function today() {
  // The laptop's local date (toISOString would give the UTC date)
  const d = new Date();
  return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, '0')}-${String(d.getDate()).padStart(2, '0')}`;
}

function load(key) {
  try {
    return JSON.parse(localStorage.getItem(key));
  } catch (e) {
    return null;  // storage unavailable or corrupt: start afresh
  }
}

function save(key, value) {
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch (e) {
    // storage unavailable: records only last until the page reloads
  }
}
