import { Sonar } from './audio.js';
import { Celebration } from './celebration.js';
import { HIT_RGB, proximityRGB, rgba } from './colors.js';
import { Display } from './display.js';
import { Game } from './game.js';

// Distances that start and end a round, from the needle tip to the target. Left as
// null they follow the depth range set in main.py (MAX_DISTANCE_CM): a round (the
// timer and sonar beeps) starts once the target is 2 cm inside the bottom of the
// range, and the needle counts as withdrawn beyond it. Set a number to override.
const ROUND_START_BELOW_CM = null;
const ROUND_RESET_ABOVE_CM = null;

// Tune these to the box.
const CONFIG = {
  historySeconds: 6,          // width of the echo trace
  startBelowCm: null,         // set by applyDepthRange()
  resetAboveCm: null,         // set by applyDepthRange()
  abandonAfterSeconds: 2,     // withdrawn this long mid-attempt: reset for the next visitor
  celebrateSeconds: 5,
  withdrawTimeoutSeconds: 20, // give up waiting for withdrawal and reset anyway
  echoHoldSeconds: 0.4,       // keep showing the last echo through brief dropouts
  smoothingSeconds: 0.08,     // display smoothing time constant
  hideCursorAfterSeconds: 3,
};

const $ = (id) => document.getElementById(id);
const els = {
  status: $('status'), statusText: $('status-text'), distance: $('distance'),
  readout: $('readout'), meter: $('meter-fill'), hint: $('hint'), timer: $('timer'),
  best: $('best'), echo: $('echo'), count: $('count'), debug: $('debug'), soundHint: $('sound-hint'),
};

const display = new Display($('display'), CONFIG.historySeconds);
const sonar = new Sonar();
const celebration = new Celebration($('celebration'));
const game = new Game(CONFIG, (result, stats) => {
  celebration.show(result, stats);
  sonar.fanfare();
});

let sensor = { state: 'connecting', detail: 'Connecting to sensor…' };
let serverConnected = false;
let lastReading = null;
let lastReadingAt = -Infinity;
let lastValidCm = null;
let lastValidEchoUs = null;
let lastValidAt = -Infinity;
let hitUntil = -Infinity;
let shownCm = null;
let readingTimes = [];
let lastFrame = null;
let lastPointerMove = 0;

const nowS = () => performance.now() / 1000;

function applyDepthRange(minCm, maxCm) {
  display.setRange(minCm, maxCm);
  CONFIG.startBelowCm = ROUND_START_BELOW_CM ?? maxCm - 2;
  CONFIG.resetAboveCm = ROUND_RESET_ABOVE_CM ?? maxCm;
  sonar.farCm = CONFIG.startBelowCm;  // slowest beeps at the start of a round
}
applyDepthRange(display.minCm, display.maxCm);  // until the server sends the real range

// ---- Data from the Python server ----------------------------------------

function connect() {
  const events = new EventSource('/events');
  events.addEventListener('open', () => { serverConnected = true; });
  events.addEventListener('error', () => { serverConnected = false; });
  events.addEventListener('config', (e) => {
    const config = JSON.parse(e.data);
    applyDepthRange(config.min_cm, config.max_cm);
  });
  events.addEventListener('status', (e) => { sensor = JSON.parse(e.data); });
  events.addEventListener('reading', (e) => onReading(JSON.parse(e.data)));
}

function onReading(reading) {
  const now = nowS();
  lastReading = reading;
  lastReadingAt = now;
  readingTimes.push(now);
  if (reading.distance_cm != null) {
    lastValidCm = reading.distance_cm;
    lastValidEchoUs = reading.echo_us;
    lastValidAt = now;
  }
  if (reading.target_hit) hitUntil = now + 0.5;
  display.addReading(now, reading.distance_cm);
  game.reading(now, reading.distance_cm, reading.target_hit);
}

// ---- Animation loop -------------------------------------------------------

function frame(tms) {
  requestAnimationFrame(frame);  // first, so one bad frame can't freeze the display
  const now = tms / 1000;
  const dt = lastFrame == null ? 0 : Math.min(now - lastFrame, 0.1);
  lastFrame = now;

  const target = now - lastValidAt < CONFIG.echoHoldSeconds ? lastValidCm : null;
  if (target == null) shownCm = null;
  else if (shownCm == null) shownCm = target;
  else shownCm += (target - shownCm) * (1 - Math.exp(-dt / CONFIG.smoothingSeconds));

  const hit = now < hitUntil;
  game.tick(now);
  if (game.state !== 'celebrate') celebration.hide();
  sonar.update(now, shownCm, game.state === 'active');
  display.render(now, { distance: shownCm, hit });
  updatePanel(now, hit);
}

function updatePanel(now, hit) {
  const live = serverConnected && now - lastReadingAt < 1.5;

  // Connection status
  let statusClass, statusText;
  if (!serverConnected) {
    statusClass = 'bad';
    statusText = 'Display not connected - is main.py running?';
  } else if (sensor.state === 'simulated') {
    statusClass = 'demo';
    statusText = 'Demo mode: simulated sensor';
  } else if (sensor.state === 'connected' && live) {
    statusClass = 'good';
    statusText = 'Sensor live';
  } else {
    statusClass = sensor.state === 'connecting' ? 'demo' : 'bad';
    statusText = sensor.detail;
  }
  els.status.className = `status ${statusClass}`;
  setText(els.statusText, statusText);

  // Distance readout
  const rgb = hit ? HIT_RGB : proximityRGB(shownCm);
  setText(els.distance, shownCm == null ? '--.-' : Math.max(shownCm, 0).toFixed(1));
  els.readout.style.setProperty('--prox', rgba(rgb));
  els.readout.style.setProperty('--prox-glow', rgba(rgb, 0.45));
  const closeness = shownCm == null ? 0 : 1 - Math.min(Math.max(shownCm / CONFIG.startBelowCm, 0), 1);
  els.meter.style.transform = `scaleX(${hit ? 1 : closeness})`;
  setText(els.hint, hintText(shownCm, hit, live));

  // Stats
  const elapsed = game.elapsed(now);
  setText(els.timer, elapsed == null ? '–' : `${elapsed.toFixed(1)} s`);
  els.timer.classList.toggle('running', game.state === 'active');
  setText(els.best, game.stats.best == null ? '–' : `${game.stats.best.toFixed(1)} s`);
  setText(els.count, `${game.stats.count}`);
  setText(els.echo, live && shownCm != null ? `${Math.round(lastValidEchoUs)} µs` : '–');

  els.soundHint.hidden = sonar.ready || sonar.muted;
  document.body.classList.toggle('hide-cursor', now - lastPointerMove > CONFIG.hideCursorAfterSeconds);
  if (!els.debug.hidden) updateDebug(now);
}

function hintText(cm, hit, live) {
  if (!live) return 'Waiting for the sensor…';
  if (hit || game.state === 'celebrate') return 'Target reached!';
  if (game.state === 'withdraw') return 'Well done! Gently pull the needle back out for the next player';
  if (cm == null) {
    return game.state === 'active'
      ? 'Lost the echo - try keeping the needle straight'
      : 'Push the needle into the patient to begin';
  }
  if (game.state === 'ready') return 'Guide the needle down towards the target';
  if (cm > 10) return 'Keep going - follow the echo';
  if (cm > 5) return 'Getting closer…';
  if (cm > 2) return 'Nearly there - slow down!';
  return 'Almost touching!';
}

function updateDebug(now) {
  readingTimes = readingTimes.filter((t) => now - t < 1);
  els.debug.textContent = [
    `server: ${serverConnected ? 'connected' : 'offline'}   sensor: ${sensor.state} (${sensor.detail})`,
    `readings/s: ${readingTimes.length}   game: ${game.state}   audio: ${sonar.ctx?.state ?? 'none'}${sonar.muted ? ' (muted)' : ''}`,
    `last: ${JSON.stringify(lastReading)}`,
    'keys: F fullscreen · M mute · R reset round · Shift+R clear today\'s stats · D debug',
  ].join('\n');
}

function setText(el, text) {
  if (el.textContent !== text) el.textContent = text;
}

// ---- Keyboard / pointer ----------------------------------------------------

window.addEventListener('pointerdown', () => sonar.unlock());
window.addEventListener('pointermove', () => { lastPointerMove = nowS(); });
window.addEventListener('keydown', (e) => {
  sonar.unlock();
  switch (e.key) {
    case 'f': case 'F':
      if (document.fullscreenElement) document.exitFullscreen();
      else document.documentElement.requestFullscreen().catch(() => {});
      break;
    case 'm': case 'M':
      sonar.toggleMute();
      break;
    case 'd': case 'D':
      els.debug.hidden = !els.debug.hidden;
      break;
    case 'r':
      game.reset(nowS());
      break;
    case 'R':
      game.clearStats();
      game.reset(nowS());
      break;
  }
});

connect();
requestAnimationFrame(frame);
