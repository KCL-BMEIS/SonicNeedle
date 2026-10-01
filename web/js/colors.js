// Proximity colour scale: cool cyan when far from the target, heating up through
// violet and magenta to hot pink when touching. Avoids green, which means "hit".
const STOPS = [
  [0, [255, 64, 110]],
  [3, [240, 80, 220]],
  [7, [150, 120, 255]],
  [12, [56, 225, 255]],
];

export const HIT_RGB = [61, 255, 160];
export const NEUTRAL_RGB = [138, 160, 189];

export function proximityRGB(distanceCm) {
  if (distanceCm == null) return NEUTRAL_RGB;
  if (distanceCm <= STOPS[0][0]) return STOPS[0][1];
  for (let i = 1; i < STOPS.length; i++) {
    const [d1, c1] = STOPS[i];
    if (distanceCm <= d1) {
      const [d0, c0] = STOPS[i - 1];
      const f = (distanceCm - d0) / (d1 - d0);
      return c0.map((v, k) => Math.round(v + (c1[k] - v) * f));
    }
  }
  return STOPS[STOPS.length - 1][1];
}

export function rgba(rgb, alpha = 1) {
  return `rgba(${rgb[0]}, ${rgb[1]}, ${rgb[2]}, ${alpha})`;
}
