/* =========================================================================
   Parking Solver — geometry + packing engine
   All measurements are in FEET. Pure functions, no DOM access here.
   ========================================================================= */
(function (global) {
'use strict';

/* ----------------------------- geometry utils ---------------------------- */

function polyArea(poly) {                 // shoelace, absolute area (sq ft)
  let a = 0;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++)
    a += (poly[j].x + poly[i].x) * (poly[j].y - poly[i].y);
  return Math.abs(a / 2);
}

function centroid(poly) {
  let x = 0, y = 0, a = 0;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const f = poly[j].x * poly[i].y - poly[i].x * poly[j].y;
    x += (poly[j].x + poly[i].x) * f;
    y += (poly[j].y + poly[i].y) * f;
    a += f;
  }
  if (Math.abs(a) < 1e-9) {               // degenerate -> average
    let mx = 0, my = 0; poly.forEach(p => { mx += p.x; my += p.y; });
    return { x: mx / poly.length, y: my / poly.length };
  }
  a *= 0.5;
  return { x: x / (6 * a), y: y / (6 * a) };
}

function bbox(poly) {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const p of poly) {
    if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
  }
  return { minX, minY, maxX, maxY };
}

function pointInPoly(pt, poly) {           // ray casting
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i].x, yi = poly[i].y, xj = poly[j].x, yj = poly[j].y;
    if (((yi > pt.y) !== (yj > pt.y)) &&
        (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi)) inside = !inside;
  }
  return inside;
}

function segInt(p1, p2, p3, p4) {          // proper segment intersection test
  const d = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);
  if (Math.abs(d) < 1e-12) return false;
  const ua = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) / d;
  const ub = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) / d;
  return ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1;
}

// is convex polygon A fully inside polygon B ?
function polyInPoly(A, B) {
  for (const c of A) if (!pointInPoly(c, B)) return false;
  for (let i = 0; i < A.length; i++) {
    const a = A[i], b = A[(i + 1) % A.length];
    for (let j = 0, k = B.length - 1; j < B.length; k = j++)
      if (segInt(a, b, B[k], B[j])) return false;
  }
  return true;
}

// do polygons A and B overlap (share interior area) ?
function polyOverlap(A, B) {
  for (const c of A) if (pointInPoly(c, B)) return true;
  for (const c of B) if (pointInPoly(c, A)) return true;
  for (let i = 0; i < A.length; i++) {
    const a = A[i], b = A[(i + 1) % A.length];
    for (let j = 0; j < B.length; j++) {
      const c = B[j], d = B[(j + 1) % B.length];
      if (segInt(a, b, c, d)) return true;
    }
  }
  return false;
}

function distPtSeg(p, a, b) {
  const dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
  if (l2 === 0) return Math.hypot(p.x - a.x, p.y - a.y);
  let t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

function rot(p, ang, c) {                  // rotate point p by ang(rad) around c
  const s = Math.sin(ang), co = Math.cos(ang);
  const x = p.x - c.x, y = p.y - c.y;
  return { x: c.x + x * co - y * s, y: c.y + x * s + y * co };
}
function rotPoly(poly, ang, c) { return poly.map(p => rot(p, ang, c)); }

function polyCenter(poly) {
  let x = 0, y = 0; for (const p of poly) { x += p.x; y += p.y; }
  return { x: x / poly.length, y: y / poly.length };
}

/* ------------------------- angle / module presets ------------------------ */
// vpd = vehicle projection depth (perpendicular to aisle). Typical US values.
const ANGLE_PRESETS = {
  90: { aisle: 24, vpd: 18.0, oneWay: false },
  60: { aisle: 18, vpd: 20.0, oneWay: true  },
  45: { aisle: 13, vpd: 18.5, oneWay: true  },
};
// vehicle projection depth (perpendicular-to-aisle stall depth to wall) for ANY angle,
// derived from the real stall dimensions so 45°/continuous angles & metric depths are correct
// (replaces the hand-tabulated vpd which was slightly off at 45° and never interpolated).
function vpdFor(angleDeg, w, d) {
  if (!angleDeg || angleDeg >= 89.5) return d;        // 90° → full stall depth
  const r = angleDeg * Math.PI / 180;
  return d * Math.sin(r) + w * Math.cos(r);
}

/* ----------------------------- the packer -------------------------------- */
/*
  Pack one orientation (angle theta, radians) and return {stalls, aisles, count}.
  Works in a frame rotated by -theta so module strips are axis-aligned, then
  rotates results back to world space.
*/
function packAtAngle(theta, ctx) {
  const { boundary, blockers, p } = ctx;          // p: tuned params
  const c = ctx.center;
  const poly = rotPoly(boundary, -theta, c);
  const blk = blockers.map(b => rotPoly(b, -theta, c));
  const bb = bbox(poly);

  const w = p.stallW;                 // along-row stall width
  const d = p.vpd;                    // depth perpendicular to aisle
  const a = p.aisle;
  const setback = p.setback + (p.greenBuffer || 0);   // green buffer = extra perimeter landscape
  const maxRun = p.maxRun || 0;                        // landscape island every N stalls (0 = off)
  const maxRunGap = p.maxRunGap || 0;
  const angled = p.angle !== 90;
  const rad = p.angle * Math.PI / 180;
  const pitch = angled ? w / Math.sin(rad) : w;        // along-row spacing
  const shear = angled ? d / Math.tan(rad) : 0;        // lean offset over depth
  // compact stalls: narrower width packs more cars into the trailing share of each row
  const compactFrac = Math.max(0, Math.min(0.9, (p.compactPct || 0) / 100));
  const compactW = (p.compactW && p.compactW > 0) ? p.compactW : w * 0.83;
  const compactPitch = angled ? compactW / Math.sin(rad) : compactW;
  const xThresh = bb.minX + (1 - compactFrac) * (bb.maxX - bb.minX);

  // Build double-loaded modules + place stalls for one grid PHASE (vertical start offset).
  // We sweep a few phases below and keep the densest: sliding the rows on an irregular lot
  // (L / trapezoid / a band pinched by a blocker) lands a row in a wider strip and recovers it.
  const M = 2 * d + a;
  const attempt = (yStart) => {
  const rows = [];                    // {y0, dir:+1 down / -1 up, aisle:{y0,h}}
  const aisles = [];                  // central aisle rects (rotated-space)
  let y = yStart;
  let guard = 0;
  while (guard++ < 5000) {
    if (y + M <= bb.maxY + 0.5) {                       // full double-loaded module
      const ay = y + d;
      rows.push({ y0: y,         dir: +1, aisle: { y0: ay, h: a } });
      rows.push({ y0: y + d + a, dir: -1, aisle: { y0: ay, h: a } });
      aisles.push({ x0: bb.minX, y0: ay, w: bb.maxX - bb.minX, h: a });
      y += M;
    } else if (y + d + a <= bb.maxY + 0.5) {            // single-loaded trailing row
      const ay = y + d;
      rows.push({ y0: y, dir: +1, aisle: { y0: ay, h: a } });
      aisles.push({ x0: bb.minX, y0: ay, w: bb.maxX - bb.minX, h: a });
      y += d + a;
    } else break;
  }

  const stalls = [];
  for (const row of rows) {
    let x = bb.minX;
    let g2 = 0, run = 0;
    while (g2++ < 5000) {
      // compacts occupy the trailing fraction of each row, at a narrower width
      const isC = compactFrac > 0 && x >= xThresh;
      const ww = isC ? compactW : w;                   // along-row stall width
      const adv = isC ? compactPitch : pitch;          // advance to the next stall
      // candidate stall footprint (axis-aligned for 90, parallelogram for angled)
      let corners;
      if (!angled) {
        if (x + ww > bb.maxX + 0.5) break;
        corners = [
          { x: x,      y: row.y0     },
          { x: x + ww, y: row.y0     },
          { x: x + ww, y: row.y0 + d },
          { x: x,      y: row.y0 + d },
        ];
      } else {
        if (x + adv + shear > bb.maxX + 0.5) break;
        if (row.dir > 0) {
          corners = [
            { x: x,               y: row.y0     },
            { x: x + adv,         y: row.y0     },
            { x: x + adv + shear, y: row.y0 + d },
            { x: x + shear,       y: row.y0 + d },
          ];
        } else {
          corners = [
            { x: x + shear,       y: row.y0     },
            { x: x + adv + shear, y: row.y0     },
            { x: x + adv,         y: row.y0 + d },
            { x: x,               y: row.y0 + d },
          ];
        }
      }

      x += adv;

      // --- validity tests (all in rotated frame) ---
      if (!polyInPoly(corners, poly)) continue;
      // setback from every boundary edge
      if (setback > 0) {
        let bad = false;
        for (const cc of corners) {
          for (let j = 0, k = poly.length - 1; j < poly.length; k = j++)
            if (distPtSeg(cc, poly[k], poly[j]) < setback) { bad = true; break; }
          if (bad) break;
        }
        if (bad) continue;
      }
      // not overlapping any blocker (building / obstacle)
      let blocked = false;
      for (const b of blk) if (polyOverlap(corners, b)) { blocked = true; break; }
      if (blocked) continue;

      // accessibility: aisle cell in front must be inside & clear
      const acx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4;
      const aprobe = { x: acx, y: row.aisle.y0 + row.aisle.h / 2 };
      if (!pointInPoly(aprobe, poly)) continue;
      let aBlocked = false;
      for (const b of blk) if (pointInPoly(aprobe, b)) { aBlocked = true; break; }
      if (aBlocked) continue;

      // store (rotated back to world); aprobe = point in the serving aisle
      stalls.push({
        poly: corners.map(pp => rot(pp, theta, c)),
        type: 'standard',
        compact: isC,
        cx: 0, cy: 0,
        aprobe: rot(aprobe, theta, c),
      });
      const sc = stalls[stalls.length - 1];
      const wc = polyCenter(sc.poly); sc.cx = wc.x; sc.cy = wc.y;
      if (maxRun > 0 && ++run >= maxRun) { x += maxRunGap; run = 0; }   // landscape island
    }
  }
  return { stalls, aisles };
  };
  // sweep vertical phases. f=0 is the original bottom-aligned grid, so the kept result can
  // never pack FEWER than before — a shifted phase only ever ADDS the odd row it happens to fit.
  let bestPack = null;
  for (let f = 0; f < 1; f += 0.2) {
    const res = attempt(bb.minY - f * M);
    if (!bestPack || res.stalls.length > bestPack.stalls.length) bestPack = res;
  }
  const stalls = bestPack.stalls, aisles = bestPack.aisles;

  // keep only aisles that actually have inside coverage (trim drawing noise)
  const aisleWorld = aisles.map(r => ({
    poly: [
      rot({ x: r.x0, y: r.y0 }, theta, c),
      rot({ x: r.x0 + r.w, y: r.y0 }, theta, c),
      rot({ x: r.x0 + r.w, y: r.y0 + r.h }, theta, c),
      rot({ x: r.x0, y: r.y0 + r.h }, theta, c),
    ],
  }));

  return { stalls, aisles: aisleWorld, count: stalls.length, theta };
}

/* --------------------------- orientation search -------------------------- */
function candidateAngles(boundary, orient) {
  const deg2rad = d => d * Math.PI / 180;
  if (typeof orient === 'number') return [orient];   // caller forces an exact aisle angle (e.g. garden bars)
  if (orient === '0')  return [0];
  if (orient === '90') return [Math.PI / 2];
  if (orient === 'edge') {
    // longest edge angle
    let best = 0, bestLen = -1;
    for (let i = 0; i < boundary.length; i++) {
      const a = boundary[i], b = boundary[(i + 1) % boundary.length];
      const len = Math.hypot(b.x - a.x, b.y - a.y);
      if (len > bestLen) { bestLen = len; best = Math.atan2(b.y - a.y, b.x - a.x); }
    }
    return [best];
  }
  // auto = the best-packing of only the *sensible* orientations: each parcel edge
  // direction and its perpendicular (i.e. align to the property lines), tried
  // LONGEST edge first so solve() keeps the parcel-aligned option on near-ties.
  // We deliberately do NOT inject screen 0°/90°: that is what made a parcel traced
  // at an angle on the map snap back to "upright" and flip as the angle changed.
  // Real lots are striped parallel to their boundaries, never to screen-north.
  const out = [];
  const add = ang => {
    let t = ((ang % Math.PI) + Math.PI) % Math.PI;                  // [0,PI)
    if (!out.some(x => Math.abs(x - t) < 0.02 || Math.abs(x - t - Math.PI) < 0.02)) out.push(t);
  };
  const edges = [];
  for (let i = 0; i < boundary.length; i++) {
    const a = boundary[i], b = boundary[(i + 1) % boundary.length];
    const L = Math.hypot(b.x - a.x, b.y - a.y);
    if (L > 1) edges.push({ ang: Math.atan2(b.y - a.y, b.x - a.x), L });
  }
  edges.sort((p, q) => q.L - p.L);                                  // longest edge first → preferred alignment
  for (const e of edges) { add(e.ang); add(e.ang + Math.PI / 2); }
  if (!out.length) { add(0); add(Math.PI / 2); }                    // degenerate parcel only
  if (!out.length) out.push(0);
  return out;
}

/* ------------------------------- ADA table ------------------------------- */
// 2010 ADA Standards — required accessible stalls by total provided.
function adaRequired(total) {
  if (total <= 0) return 0;
  if (total <= 25)  return 1;
  if (total <= 50)  return 2;
  if (total <= 75)  return 3;
  if (total <= 100) return 4;
  if (total <= 150) return 5;
  if (total <= 200) return 6;
  if (total <= 300) return 7;
  if (total <= 400) return 8;
  if (total <= 500) return 9;
  if (total <= 1000) return Math.ceil(total * 0.02);
  return 20 + Math.ceil((total - 1000) / 100);
}

/* ---------------------- assign stall types after solve -------------------- */
function assignTypes(sol, opts) {
  // reset — the packer already physically placed compacts at a narrower width,
  // so honour its `compact` flag instead of re-labelling standard stalls.
  sol.stalls.forEach(s => s.type = s.compact ? 'compact' : 'standard');
  const total = sol.stalls.length;
  if (!total) return;

  // ADA — nearest to building/site focus (full-size stalls only, never a compact)
  let adaN = 0;
  if (opts.adaMode === 'code') adaN = adaRequired(total);
  else if (opts.adaMode === 'manual') adaN = Math.min(opts.adaManual | 0, total);
  if (adaN > 0) {
    const focus = opts.focus;                 // {x,y}
    const sorted = sol.stalls.filter(s => s.type === 'standard').sort((a, b) =>
      Math.hypot(a.cx - focus.x, a.cy - focus.y) - Math.hypot(b.cx - focus.x, b.cy - focus.y));
    for (let i = 0; i < adaN && i < sorted.length; i++) sorted[i].type = 'ada';
  }

  // remaining pool (full-size standard only)
  const pool = sol.stalls.filter(s => s.type === 'standard');
  // EV — spread across pool
  const evN = Math.round(total * (opts.evPct || 0) / 100);
  for (let i = 0; i < evN && i < pool.length; i++) {
    const idx = Math.floor(i * pool.length / Math.max(evN, 1));
    if (pool[idx].type === 'standard') pool[idx].type = 'ev';
  }
  // MOTORCYCLE — spread across the remaining standard stalls (rendered as a smaller stall)
  const motoN = Math.round(total * (opts.motoPct || 0) / 100);
  const pool2 = sol.stalls.filter(s => s.type === 'standard');
  for (let i = 0; i < motoN && i < pool2.length; i++) {
    const idx = Math.floor(i * pool2.length / Math.max(motoN, 1));
    if (pool2[idx].type === 'standard') pool2[idx].type = 'moto';
  }

  // ADA access aisles — each accessible stall pairs with a 5ft striped access
  // aisle, taken from the nearest ordinary stall (an ADA pair occupies 2 slots).
  sol.stalls.filter(s => s.type === 'ada').forEach(ada => {
    let best = null, bd = Infinity;
    for (const s of sol.stalls) {
      if (s.type !== 'standard' && s.type !== 'compact') continue;
      const d = Math.hypot(s.cx - ada.cx, s.cy - ada.cy);
      if (d > 1 && d < bd) { bd = d; best = s; }
    }
    if (best) best.type = 'access';
  });
  sol.accessAisles = sol.stalls.filter(s => s.type === 'access');
  sol.stalls = sol.stalls.filter(s => s.type !== 'access');
  sol.count = sol.stalls.length;
}

/* ----------------------- entrance circulation network -------------------- */
// inward unit normal of the boundary edge nearest a gate — so the entrance drive comes in
// PERPENDICULAR TO ITS OWN EDGE, regardless of the parking angle.
function inwardEdgeNormal(poly, e, c) {
  let best = 0, bd = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const d = distPtSeg(e, poly[i], poly[(i + 1) % poly.length]);
    if (d < bd) { bd = d; best = i; }
  }
  const a = poly[best], b = poly[(best + 1) % poly.length];
  let nx = -(b.y - a.y), ny = (b.x - a.x); const L = Math.hypot(nx, ny) || 1; nx /= L; ny /= L;
  const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
  if (nx * (c.x - mx) + ny * (c.y - my) < 0) { nx = -nx; ny = -ny; }   // point inward
  return { x: nx, y: ny };
}
// Entrance drive enters perpendicular to the gate's OWN edge and stops at the in-lot road.
// Parallel aisles are linked by a PERIMETER connector along their ends (not a mid-lot slice).
// BFS from the gate stubs drops unreachable stalls, with a safety net against over-deletion.
function buildCirculation(sol, boundary, entrances, aisleW, ignoreRoads, buildings, obstacles) {
  sol.connectors = []; sol.unreachable = 0;
  if (ignoreRoads) return;                       // "ignore roads": keep every packed stall, carve no drive lanes
  if (!entrances || !entrances.length || !sol.stalls.length) return;
  const blockers = (buildings || []).concat(obstacles || []);
  const c = centroid(boundary);
  const th = sol.theta || 0;
  const run = { x: Math.cos(th), y: Math.sin(th) };          // aisle run direction
  const pd  = { x: -Math.sin(th), y: Math.cos(th) };         // across rows (perpendicular)
  const half = aisleW / 2;
  const step = Math.max(2, aisleW / 3);
  const toWorld = (r, p) => ({ x: run.x * r + pd.x * p, y: run.y * r + pd.y * p });
  const drivable = pt => {
    if (!pointInPoly(pt, boundary)) return false;
    for (const b of blockers) if (pointInPoly(pt, b)) return false;
    return true;
  };

  // Clip each packed aisle strip to the DRIVABLE lot → real aisle segments.
  // The packer stores aisles as full-bbox-width strips; on irregular or rotated
  // parcels those overhang the lot, which made the old "two connectors at the
  // global run-extremes" logic drop lanes in empty corners (floating segments)
  // and mis-prune whole stall rows. Re-derive each aisle's TRUE drivable span.
  const projR = q => q.x * run.x + q.y * run.y;
  const projP = q => q.x * pd.x + q.y * pd.y;
  const segPoly = (pc, lo, hi) => [toWorld(lo, pc - half), toWorld(hi, pc - half), toWorld(hi, pc + half), toWorld(lo, pc + half)];
  const segs = [];                                           // {pc, rLo, rHi, poly}
  for (const a of sol.aisles) {
    const rs = a.poly.map(projR), ps = a.poly.map(projP);
    const pc = (Math.min(...ps) + Math.max(...ps)) / 2, r0 = Math.min(...rs), r1 = Math.max(...rs);
    let lo = null;
    for (let r = r0; r <= r1 + step; r += step) {
      const ok = r <= r1 && drivable(toWorld(r, pc));
      if (ok && lo === null) lo = r;
      else if (!ok && lo !== null) { if (r - step - lo >= aisleW) segs.push({ pc, rLo: lo, rHi: r - step, poly: segPoly(pc, lo, r - step) }); lo = null; }
    }
  }
  if (!segs.length) { sol.count = sol.stalls.length; return; }
  sol.aisles = segs.map(s => ({ poly: s.poly }));            // drawing + picking follow the clipped strips

  // LADDER circulation: link each aisle to its nearest neighbouring row (in the
  // perpendicular pd direction) with a rung placed where the two ACTUALLY overlap
  // along the drive direction — at each pair's own shared end, never a global
  // extreme. Robust to tilt / L / U / cross parcels. Colinear rungs are then
  // merged so a regular lot still reads as two clean perimeter drives.
  const rungs = [];                                          // {r, p0, p1} in (run, pd)
  const tryRung = (A, B, target, dirSign) => {
    const loB = Math.min(A.pc, B.pc), hiB = Math.max(A.pc, B.pc);
    for (let k = 0; k < 10; k++) {
      const r = target + dirSign * k * step;
      if (r < Math.max(A.rLo, B.rLo) || r > Math.min(A.rHi, B.rHi)) return false;
      let ok = true;
      for (let p = loB; p <= hiB && ok; p += step) if (!drivable(toWorld(r, p))) ok = false;
      if (ok) { rungs.push({ r, p0: loB, p1: hiB }); return true; }
    }
    return false;
  };
  const byPc = segs.slice().sort((A, B) => A.pc - B.pc);
  for (let i = 0; i < byPc.length; i++) {
    const A = byPc[i];
    // Track whether A's LOW and HIGH run-ends each got a cross-rung. A blocker can split the
    // neighbouring row into two segments (one overlapping A's low end, one its high end); linking
    // only the first leaves A's other end a DEAD-END. So keep scanning neighbours until BOTH ends
    // are served (or none remain). On a plain lot the nearest neighbour serves both ends at once,
    // so this still breaks immediately — no extra cross-aisles introduced.
    let loDone = false, hiDone = false;
    for (let j = i + 1; j < byPc.length; j++) {
      const B = byPc[j];
      if (B.pc - A.pc > aisleW * 4) break;                   // neighbour too far → different field
      const oLo = Math.max(A.rLo, B.rLo), oHi = Math.min(A.rHi, B.rHi);
      if (oHi - oLo < aisleW) continue;                      // no real run-overlap → try a further row
      // a rung near A's low / high end is only useful if THIS overlap actually reaches that end
      const servesLo = !loDone && (oLo <= A.rLo + aisleW * 1.5);
      const servesHi = !hiDone && (oHi >= A.rHi - aisleW * 1.5);
      // lay a rung at each of A's UNSERVED ends that this neighbour-piece can reach (an obstacle-split
      // neighbour reaches only one of A's ends, so each end binds to whichever piece covers it)
      const r1 = servesLo ? tryRung(A, B, oLo + half, +1) : false;
      const r2 = (servesHi && oHi - oLo > aisleW) ? tryRung(A, B, oHi - half, -1) : false;
      // short overlap that is itself a far end → a single mid rung still links the pair
      const rMid = (!r1 && !r2 && oHi - oLo >= aisleW) ? tryRung(A, B, (oLo + oHi) / 2, +1) : false;
      if (r1) loDone = true;
      if (r2) hiDone = true;
      if (rMid) { if (servesLo) loDone = true; if (servesHi) hiDone = true; }
      if (loDone && hiDone) break;                            // both ends linked → done with A
    }
    // END-BIND pass: the forward scan only looks at higher-pc neighbours, so a row that a blocker
    // split into a low piece + a high piece can leave the INNER end of one piece unlinked (its only
    // spanning neighbour sits at a LOWER pc index). For each end still unserved, scan ALL neighbours
    // (nearest pc first, both directions) for one whose run-span covers that end, and lay the rung
    // AT that end. Only fires for genuinely unlinked ends, so fully-linked plain lots are untouched.
    if (!loDone || !hiDone) {
      const nbrs = byPc.map(B => ({ B, d: Math.abs(B.pc - A.pc) }))
        .filter(o => o.B !== A && o.d > 1 && o.d <= aisleW * 4).sort((a, b) => a.d - b.d);
      for (const end of [{ on: loDone, r: A.rLo, sign: +1 }, { on: hiDone, r: A.rHi, sign: -1 }]) {
        if (end.on) continue;
        for (const { B } of nbrs) {
          if (B.rLo > end.r - aisleW || B.rHi < end.r + aisleW) continue;     // piece doesn't reach this end
          const lim = { rLo: Math.max(A.rLo, B.rLo), rHi: Math.min(A.rHi, B.rHi) };
          if (lim.rHi - lim.rLo < aisleW * 0.5) continue;
          const target = end.sign > 0 ? lim.rLo + half : lim.rHi - half;
          if (tryRung(A, B, target, end.sign)) break;        // this end linked → next end
        }
      }
    }
  }
  // merge rungs sharing a run-line into continuous bands. Rungs are GROUPED onto
  // run-lines with a tolerance, then each line's p-intervals are unioned — an
  // exact r-sort fragmented tilted/skewed lots, because projecting a rotated
  // parcel jitters each colinear rung's r in the 6th decimal, so the primary
  // r-sort scrambled their p-order and the linear merge tore a band in two,
  // stranding the middle rows as an unreachable island. (Axis-aligned lots have
  // zero jitter, which is why only rotated parcels broke.)
  const connectors = [];
  const lines = [];
  for (const g of rungs) {
    let ln = lines.find(L => Math.abs(L.r - g.r) <= half);
    if (!ln) { ln = { r: g.r, ivs: [] }; lines.push(ln); }
    ln.ivs.push([g.p0, g.p1]);
  }
  for (const ln of lines) {
    ln.ivs.sort((a, b) => a[0] - b[0]);
    let cur = null;
    for (const iv of ln.ivs) {
      if (cur && iv[0] <= cur.p1 + step * 1.1) cur.p1 = Math.max(cur.p1, iv[1]);
      else { cur = { r: ln.r, p0: iv[0], p1: iv[1] }; connectors.push(cur); }
    }
  }
  connectors.forEach(cn => {
    cn.poly = [toWorld(cn.r - half, cn.p0), toWorld(cn.r + half, cn.p0), toWorld(cn.r + half, cn.p1), toWorld(cn.r - half, cn.p1)];
    cn.perimeter = true;
  });

  // PERIMETER end-bands: a drive band just inside each boundary edge that faces
  // ACROSS the aisles (inward normal ≈ ±run) — needed to bridge around an INTERNAL
  // building/obstacle (e.g. a gate above a mid-lot building reaches the lot via the
  // top-edge band). Only run when blockers exist: a plain or simply-tilted lot is
  // already fully linked by the ladder above, so skipping here avoids eating
  // perimeter stalls. Edges that run ALONG the aisles are skipped (redundant).
  const bandLo = 2, bandHi = 2 + aisleW;
  if (blockers.length) for (let i = 0; i < boundary.length; i++) {
    const a = boundary[i], b = boundary[(i + 1) % boundary.length];
    const ex = b.x - a.x, ey = b.y - a.y, L = Math.hypot(ex, ey); if (L < aisleW) continue;
    let nx = -ey / L, ny = ex / L;                                   // inward normal
    const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
    if (nx * (c.x - mx) + ny * (c.y - my) < 0) { nx = -nx; ny = -ny; }
    if (Math.abs(nx * run.x + ny * run.y) < 0.5) continue;           // edge runs ALONG the aisles → skip
    const P = (tt, ins) => ({ x: a.x + ex * tt + nx * ins, y: a.y + ey * tt + ny * ins });
    const N = Math.ceil(L / step);
    let s0 = null, prev = false;
    for (let k = 0; k <= N; k++) {
      const t = Math.min(1, k / N);
      const ok = k < N && drivable(P(t, bandLo + aisleW / 2));
      if (ok && !prev) s0 = t;
      else if (!ok && prev) {
        const tb = Math.min(1, (k - 1) / N);
        if ((tb - s0) * L >= aisleW) connectors.push({ poly: [P(s0, bandLo), P(tb, bandLo), P(tb, bandHi), P(s0, bandHi)], perimeter: true, edgeBand: true });
      }
      prev = ok;
    }
  }

  // De-duplicate the drive segments → ONE clean road graph. Crossings (T / +) survive
  // because they overlap only a little, but a lane whose footprint is mostly shadowed by
  // a bigger one (a rung sitting under a perimeter band, two bands meeting at a concave
  // corner) is dropped — so no stretch of road is ever painted twice.
  if (connectors.length > 1) {
    const bil = (q, u, v) => ({
      x: (q[0].x * (1 - u) + q[1].x * u) * (1 - v) + (q[3].x * (1 - u) + q[2].x * u) * v,
      y: (q[0].y * (1 - u) + q[1].y * u) * (1 - v) + (q[3].y * (1 - u) + q[2].y * u) * v,
    });
    // Keep PERPENDICULAR rungs ahead of slanted edge-bands of similar size, then biggest-first.
    // A far-end rung and a boundary edge-band often shadow each other; if the wider edge-band is
    // tested first it survives and the rung is dropped — and the later load-bearing prune then
    // deletes that edge-band too (the field stays connected via its NEAR-end rung), leaving the
    // far end with NO cross-aisle: a dead-end. Preferring the rung makes the perpendicular cross
    // aisle survive de-dup (the prune never touches non-edgeBand rungs), closing the loop.
    const rank = cn => (cn.edgeBand ? 0 : 1);
    connectors.sort((a, b) => (rank(b) - rank(a)) || (polyArea(b.poly) - polyArea(a.poly)));
    const keep = [];
    for (const cn of connectors) {
      let inside = 0, total = 0;
      for (let u = 0.1; u < 1; u += 0.2) for (let v = 0.1; v < 1; v += 0.2) {
        total++; const pt = bil(cn.poly, u, v);
        if (keep.some(k => pointInPoly(pt, k.poly))) inside++;
      }
      if (inside / total < 0.6) keep.push(cn);                         // <60% shadowed → it's a real, distinct lane
    }
    connectors.length = 0; connectors.push(...keep);
  }

  // ENTRANCE stubs: from each gate, ROUTE to the in-lot road through drivable space.
  // The old code marched a single straight ray inward (perpendicular to the gate edge)
  // and STOPPED at the first non-drivable cell — so any blocker sitting in front of the
  // gate (building/obstacle) froze the spine short of the network and orphaned the whole
  // aisle graph from the gate; it also mis-fired when the gate sat on a boundary edge that
  // ray-casting reports as "outside" (max-x / max-y edges), freezing the spine at ~0.8*aisleW.
  // Replace it with a grid BFS that finds the SHORTEST drivable gate→network route, then
  // lay an aisle-width drive band along that route (an L-shaped / dog-leg stub when the
  // straight path is blocked). The band is emitted as a chain of convex rectangles that all
  // carry .type, so every downstream consumer (reach-seed, drive-clear, BFS, validator) that
  // iterates `spines` keeps working unchanged.
  const spines = [];
  // The spine must reach the PARKING network — an aisle, or a connector that actually touches an
  // aisle. A connector (e.g. a boundary edge-band) that floats free of every aisle is NOT a valid
  // landing pad: routing onto it leaves the gate unable to reach any stall, and that band may even
  // be pruned later (orphaning the spine). Restricting `inNet` to aisle-connected polys makes the
  // cost-search thread a tight gap to a real aisle rather than dead-end on an isolated edge-band.
  const aislePolys0 = sol.aisles.map(a => a.poly);
  const netPolys = aislePolys0.concat(connectors.filter(cn => aislePolys0.some(ap => polyOverlap(cn.poly, ap))).map(cn => cn.poly));
  const inNet = pt => netPolys.some(poly => pointInPoly(pt, poly));
  // a band rectangle of width aisleW centred on segment u→v
  const bandSeg = (u, v) => {
    let dx = v.x - u.x, dy = v.y - u.y; const L = Math.hypot(dx, dy) || 1; dx /= L; dy /= L;
    const ox = -dy * half, oy = dx * half;
    return [{ x: u.x + ox, y: u.y + oy }, { x: u.x - ox, y: u.y - oy }, { x: v.x - ox, y: v.y - oy }, { x: v.x + ox, y: v.y + oy }];
  };
  // clip a convex polygon to the half-plane dot(n, p-A) >= 0 (keep where n points)
  const clipHalf = (poly, ax, ay, nx, ny) => {
    const f = p => nx * (p.x - ax) + ny * (p.y - ay);
    const out = [];
    for (let i = 0; i < poly.length; i++) {
      const cur = poly[i], prv = poly[(i + poly.length - 1) % poly.length];
      const fc = f(cur), fp = f(prv);
      if (fc >= -1e-9) { if (fp < -1e-9) { const t = fp / (fp - fc); out.push({ x: prv.x + t * (cur.x - prv.x), y: prv.y + t * (cur.y - prv.y) }); } out.push(cur); }
      else if (fp >= -1e-9) { const t = fp / (fp - fc); out.push({ x: prv.x + t * (cur.x - prv.x), y: prv.y + t * (cur.y - prv.y) }); }
    }
    return out;
  };
  // Last-resort: trim a band leg off any blocker it grazes. For each blocker whose interior the band
  // enters, find the blocker edge the band centreline is OUTSIDE of (the side the lane sits on) and
  // clip the band to that edge's outer half-plane — narrowing the lane to the clear gap (e.g. a tight
  // slot) instead of running over the building/obstacle. Convex-in → convex-out; connectivity kept
  // because the centreline itself stays drivable.
  const clipLegOffBlockers = (poly, u, v) => {
    let out = poly;
    const mid = { x: (u.x + v.x) / 2, y: (u.y + v.y) / 2 };
    for (const b of blockers) {
      if (!polyOverlap(out, b)) continue;
      // pick the blocker edge whose OUTWARD normal points toward the lane centreline most strongly
      let beI = -1, beDot = -Infinity, bn = null, ba = null;
      for (let i = 0; i < b.length; i++) {
        const a = b[i], c2 = b[(i + 1) % b.length];
        let nx = (c2.y - a.y), ny = -(c2.x - a.x); const L = Math.hypot(nx, ny) || 1; nx /= L; ny /= L;  // one normal
        // make it the OUTWARD normal (points away from blocker centroid)
        const bc = centroid(b); if (nx * (bc.x - a.x) + ny * (bc.y - a.y) > 0) { nx = -nx; ny = -ny; }
        const d = nx * (mid.x - a.x) + ny * (mid.y - a.y);     // how far outside this edge the lane centre sits
        if (d > beDot) { beDot = d; beI = i; bn = { x: nx, y: ny }; ba = a; }
      }
      if (beI >= 0 && bn) out = clipHalf(out, ba.x, ba.y, bn.x, bn.y);
      if (out.length < 3) break;
    }
    return out.length >= 3 ? out : poly;   // never return a degenerate leg
  };
  // grid over the lot at `step` resolution; cell drivable if its centre is inside the lot
  // (with a tiny inward tolerance so cells touching the boundary still count) and clear of blockers.
  const gb = bbox(boundary);
  const gx0 = gb.minX - step, gy0 = gb.minY - step;
  const gnx = Math.ceil((gb.maxX - gb.minX + 2 * step) / step) + 1;
  const gny = Math.ceil((gb.maxY - gb.minY + 2 * step) / step) + 1;
  const gcx = i => gx0 + i * step + step / 2, gcy = j => gy0 + j * step + step / 2;
  const cellDrivable = (i, j) => {
    const pt = { x: gcx(i), y: gcy(j) };
    if (!drivable(pt)) {
      // nudge toward the lot centroid so a cell straddling the boundary edge still seeds the gate
      const nx = c.x - pt.x, ny = c.y - pt.y, nl = Math.hypot(nx, ny) || 1;
      const pt2 = { x: pt.x + nx / nl * (step * 0.4), y: pt.y + ny / nl * (step * 0.4) };
      if (!drivable(pt2)) return false;
    }
    return true;
  };
  // CLEARANCE test: the spine lays an aisleW-wide band centred on the path, so a path cell is
  // only safe if the band's half-width stays OFF every blocker. Require the cell centre AND four
  // points offset by `half` (= aisleW/2) along ±x/±y to all be clear of blockers — i.e. the cell
  // sits at least ~half from any blocker edge, so the band cannot cut into a building/obstacle.
  const blockerClear = pt => {
    for (const b of blockers) {
      if (pointInPoly(pt, b)) return false;
      if (pointInPoly({ x: pt.x + half, y: pt.y }, b)) return false;
      if (pointInPoly({ x: pt.x - half, y: pt.y }, b)) return false;
      if (pointInPoly({ x: pt.x, y: pt.y + half }, b)) return false;
      if (pointInPoly({ x: pt.x, y: pt.y - half }, b)) return false;
    }
    return true;
  };
  const cellClear = (i, j) => cellDrivable(i, j) && blockerClear({ x: gcx(i), y: gcy(j) });
  const gi = x => Math.round((x - gx0 - step / 2) / step), gj = y => Math.round((y - gy0 - step / 2) / step);
  for (const e of entrances) {
    const dir = inwardEdgeNormal(boundary, e, c);
    // seed cell: step a little inward from the gate so we start on a real lot cell
    let si = gi(e.x + dir.x * step * 0.5), sj = gj(e.y + dir.y * step * 0.5);
    if (si < 0) si = 0; if (si >= gnx) si = gnx - 1; if (sj < 0) sj = 0; if (sj >= gny) sj = gny - 1;
    // ROUTE gate→network with a COST-WEIGHTED search (Dijkstra) over drivable cells: a cell that
    // keeps aisleW/2 clearance from every blocker costs ~1; a drivable cell that lies WITHIN half
    // of a blocker (so the band would graze it) costs a big penalty. The cheapest path therefore
    // keeps clearance whenever a clear route exists, and only "grazes" a blocker (e.g. threads a
    // gap too tight for a full-width clear lane) when that is the ONLY way through — and even then
    // it minimises the grazing length instead of taking a long clear detour or wandering off. Each
    // emitted band leg is later clipped off the blockers, so any grazed stretch contributes 0
    // overlap while still physically connecting the gate to the road graph.
    const PEN = 1e4;                                     // penalty for a near-blocker (band would graze) cell
    // seed; if the seed cell isn't even drivable, scan a small ring for one that is
    if (!cellDrivable(si, sj)) {
      for (let rad = 1, found = false; rad <= 4 && !found; rad++)
        for (let dj = -rad; dj <= rad && !found; dj++) for (let di = -rad; di <= rad && !found; di++) {
          const ii = si + di, jj = sj + dj;
          if (ii < 0 || ii >= gnx || jj < 0 || jj >= gny) continue;
          if (cellDrivable(ii, jj)) { si = ii; sj = jj; found = true; }
        }
    }
    const prev = new Int32Array(gnx * gny).fill(-1);
    const dist = new Float64Array(gnx * gny).fill(Infinity);
    const done = new Uint8Array(gnx * gny);
    let hitK = -1;
    const startK = sj * gnx + si;
    if (cellDrivable(si, sj)) {
      dist[startK] = cellClear(si, sj) ? 0 : PEN;
      // binary-heap-free Dijkstra: penalties take only 2 levels so a simple repeated-min scan is fine,
      // but to stay O(E log V)-ish on big grids we use a lightweight bucketed frontier (array + min find).
      const frontier = [startK];
      while (frontier.length) {
        // pop the min-dist frontier cell
        let bi = 0; for (let t = 1; t < frontier.length; t++) if (dist[frontier[t]] < dist[frontier[bi]]) bi = t;
        const k = frontier[bi]; frontier[bi] = frontier[frontier.length - 1]; frontier.pop();
        if (done[k]) continue; done[k] = 1;
        const ci = k % gnx, cj = (k / gnx) | 0;
        if (inNet({ x: gcx(ci), y: gcy(cj) })) { hitK = k; break; }
        const nb = [[ci + 1, cj], [ci - 1, cj], [ci, cj + 1], [ci, cj - 1]];
        for (const [ni, nj] of nb) {
          if (ni < 0 || ni >= gnx || nj < 0 || nj >= gny) continue;
          const nk = nj * gnx + ni;
          if (done[nk] || !cellDrivable(ni, nj)) continue;
          const w = cellClear(ni, nj) ? 1 : PEN;          // grazing a blocker is heavily penalised
          if (dist[k] + w < dist[nk]) { dist[nk] = dist[k] + w; prev[nk] = k; frontier.push(nk); }
        }
      }
    }
    // reconstruct path cells gate→network, then simplify to corner waypoints
    const ent = { x: e.x, y: e.y };
    let waypts;
    if (hitK >= 0) {
      const cells = [];
      for (let k = hitK; k !== -1; k = prev[k]) cells.push({ x: gcx(k % gnx), y: gcy((k / gnx) | 0) });
      cells.reverse();                                   // gate-side → network-side
      // STRING-PULL the grid staircase into the FEWEST straight legs that each stay clear
      // of blockers — a clean dog-leg instead of a jagged stair. The raw Manhattan path
      // zig-zags ~one corner per cell (7+ legs around an obstacle); from each anchor we
      // reach as far along the path as a straight, clearance-keeping line allows, then turn.
      const losClear = (p, q) => {
        const L = Math.hypot(q.x - p.x, q.y - p.y), n = Math.max(1, Math.ceil(L / (step * 0.5)));
        for (let t = 0; t <= n; t++) {
          const x = p.x + (q.x - p.x) * t / n, y = p.y + (q.y - p.y) * t / n;
          if (!drivable({ x, y }) || !blockerClear({ x, y })) return false;
        }
        return true;
      };
      const pts = [ent];
      for (let idx = 0; idx < cells.length;) {
        let far = idx;
        for (let j = cells.length - 1; j > idx; j--) { if (losClear(cells[idx], cells[j])) { far = j; break; } }
        if (far === idx) far = Math.min(idx + 1, cells.length - 1);   // can't extend → advance one cell
        pts.push(cells[far]);
        if (far === idx) break;
        idx = far;
      }
      // push one short step past the last cell so the band visibly enters the road
      const last = cells[cells.length - 1], pen = pts[pts.length - 2] || ent;
      let ex = last.x - pen.x, ey = last.y - pen.y, el = Math.hypot(ex, ey) || 1;
      pts.push({ x: last.x + ex / el * half, y: last.y + ey / el * half });
      waypts = pts;
    } else {
      // no drivable route to the network at all → keep the legacy short straight stub
      const B = { x: e.x + dir.x * aisleW * 1.2, y: e.y + dir.y * aisleW * 1.2 };
      waypts = [ent, B];
    }
    // emit one band rectangle per leg, all tagged as this gate's spine (clipped off any grazed blocker)
    for (let i = 0; i + 1 < waypts.length; i++) {
      if (Math.hypot(waypts[i + 1].x - waypts[i].x, waypts[i + 1].y - waypts[i].y) < 0.5) continue;
      const leg = clipLegOffBlockers(bandSeg(waypts[i], waypts[i + 1]), waypts[i], waypts[i + 1]);
      spines.push({ poly: leg, type: e.type || 'inout', dir, ent });
    }
    if (!spines.some(sp => sp.ent === ent)) {           // degenerate: emit a minimal stub at the gate
      const ox = -dir.y * half, oy = dir.x * half, B = { x: e.x + dir.x * aisleW, y: e.y + dir.y * aisleW };
      spines.push({ poly: [{ x: e.x + ox, y: e.y + oy }, { x: e.x - ox, y: e.y - oy }, { x: B.x - ox, y: B.y - oy }, { x: B.x + ox, y: B.y + oy }], type: e.type || 'inout', dir, ent });
    }
  }

  // Keep BOTH end cross-aisles (the ladder rungs) so the drive network is a LOOP,
  // not a dead-end tree — a car must circulate in and back out without reversing
  // down a blind aisle. We never prune the perpendicular rungs.
  //
  // Edge-bands are different: they hug a boundary edge to DETOUR around an internal
  // building, so on an irregular lot that edge can be slanted and the band comes out
  // non-perpendicular (not a clean cross-aisle). Keep an edge-band ONLY when it is
  // load-bearing — i.e. some aisle reaches the network through it and nothing else —
  // and drop spurious ones, so a lot that doesn't truly need a perimeter detour keeps
  // every connector perpendicular.
  if (connectors.some(cn => cn.edgeBand)) {
    const aislePolys = sol.aisles.map(a => a.poly), A = aislePolys.length;
    const reached = conns => {                                  // # aisles reachable from the gate; cars cross between aisles only via a connector/spine, never aisle→aisle
      const nodes = aislePolys.concat(conns.map(cn => cn.poly));
      const reach = nodes.map(poly => spines.some(sp => polyOverlap(poly, sp.poly)));
      for (let ch = true; ch;) { ch = false;
        for (let i = 0; i < nodes.length; i++) { if (reach[i]) continue;
          for (let j = 0; j < nodes.length; j++) {
            if (!reach[j] || (i < A && j < A)) continue;
            if (polyOverlap(nodes[i], nodes[j])) { reach[i] = true; ch = true; break; }
          } } }
      let n = 0; for (let i = 0; i < A; i++) if (reach[i]) n++; return n;
    };
    const target = reached(connectors);
    for (let i = 0; i < connectors.length; i++)
      if (connectors[i].edgeBand && reached(connectors.filter((_, k) => k !== i)) >= target) { connectors.splice(i, 1); i--; }
  }

  // DENSITY: drop REDUNDANT cross-aisles to recover the stalls they would eat. A connector
  // is removable only when taking it out (a) leaves every aisle still reachable from a gate
  // AND (b) leaves every aisle that was open at BOTH run-ends still open at both ends — so we
  // never strand a field or turn an aisle into a blind (back-out-only) dead-end. Greedy:
  // remove the biggest stall-eater that passes the guard first, then re-test, until none pass.
  // The load-bearing rungs (the ones that actually carry the loop) all fail the guard and stay.
  {
    const aislePolys = sol.aisles.map(a => a.poly), A = aislePolys.length;
    const spinePolys = spines.map(s => s.poly);
    const PR = q => q.x * run.x + q.y * run.y;
    // reachable-aisle count from the gates for a given connector set (cars hop aisle→aisle
    // only through a connector/spine, never aisle→aisle directly — same rule as the BFS below)
    const reachedAisles = conns => {
      const nodes = aislePolys.concat(conns.map(c => c.poly));
      const reach = nodes.map(poly => spines.some(sp => polyOverlap(poly, sp.poly)));
      for (let ch = true; ch;) { ch = false;
        for (let i = 0; i < nodes.length; i++) { if (reach[i]) continue;
          for (let j = 0; j < nodes.length; j++) {
            if (!reach[j] || (i < A && j < A)) continue;
            if (polyOverlap(nodes[i], nodes[j])) { reach[i] = true; ch = true; break; } } } }
      let n = 0; for (let i = 0; i < A; i++) if (reach[i]) n++; return n;
    };
    // is aisle `ap` served by some drive within aisleW of BOTH its run-ends? (no blind end)
    const endServed = (ap, t, drives) => drives.some(d => { const cs = d.map(PR);
      return Math.min(...cs) <= t + aisleW && Math.max(...cs) >= t - aisleW && polyOverlap(ap, d); });
    const bothEnds = (ap, conns) => { const rs = ap.map(PR), lo = Math.min(...rs), hi = Math.max(...rs);
      const drives = conns.map(c => c.poly).concat(spinePolys);
      return endServed(ap, lo, drives) && endServed(ap, hi, drives); };
    // how many packed stalls a connector would eat (same >8% area test the clear below uses)
    const eatCount = poly => sol.stalls.reduce((n, s) => {
      if (!polyOverlap(s.poly, poly)) return n;
      const bb = bbox(s.poly); let inside = 0, under = 0;
      for (let x = bb.minX; x <= bb.maxX; x += 3) for (let y = bb.minY; y <= bb.maxY; y += 3) {
        const pt = { x, y }; if (!pointInPoly(pt, s.poly)) continue; inside++;
        if (pointInPoly(pt, poly)) under++; }
      return n + (under / Math.max(inside, 1) > 0.08 ? 1 : 0); }, 0);

    const baseReach = reachedAisles(connectors);
    const baseOpen = aislePolys.map(ap => bothEnds(ap, connectors));   // protect only originally-open aisles
    for (let removed = true; removed;) {
      removed = false;
      let best = -1, bestEat = 0;
      for (let i = 0; i < connectors.length; i++) {
        const without = connectors.filter((_, k) => k !== i);
        if (reachedAisles(without) < baseReach) continue;               // would strand a field
        let ok = true;
        for (let a = 0; a < A && ok; a++) if (baseOpen[a] && !bothEnds(aislePolys[a], without)) ok = false;
        if (!ok) continue;                                              // would create a blind aisle
        const eat = eatCount(connectors[i].poly);
        if (eat > bestEat) { bestEat = eat; best = i; }
      }
      if (best >= 0 && bestEat > 0) { connectors.splice(best, 1); removed = true; }
    }
  }

  // clear stalls sitting under the perimeter drives + entrance stubs. Use an AREA test,
  // not just the centre point: a stall whose centre is just outside a band but whose body
  // is clipped by it (a cross-aisle edge cutting a row) would otherwise be drawn part-under
  // the road. Measure the true overlap fraction on a fine grid over the stall (catches a thin
  // edge strip the centre/coarse test misses) and drop the stall if >8% sits under any drive.
  const drive = connectors.concat(spines);
  sol.stalls = sol.stalls.filter(s => {
    const candidates = drive.filter(d => polyOverlap(s.poly, d.poly));
    if (!candidates.length) return true;
    const b = bbox(s.poly); let inside = 0, under = 0;
    for (let x = b.minX; x <= b.maxX; x += 1.5) for (let y = b.minY; y <= b.maxY; y += 1.5) {
      const pt = { x, y };
      if (!pointInPoly(pt, s.poly)) continue; inside++;
      if (candidates.some(d => pointInPoly(pt, d.poly))) under++;
    }
    return under / Math.max(inside, 1) <= 0.08;           // >8% of the stall under a drive → can't park there, drop it
  });

  // reachability BFS over {aisles, perimeter connectors}, seeded by the entrance stubs
  const nodes = sol.aisles.map(a => a.poly).concat(connectors.map(cn => cn.poly));
  const reach = nodes.map(poly => spines.some(sp => polyOverlap(poly, sp.poly)));
  for (let changed = true; changed;) {
    changed = false;
    for (let i = 0; i < nodes.length; i++) {
      if (reach[i]) continue;
      for (let j = 0; j < nodes.length; j++)
        if (reach[j] && polyOverlap(nodes[i], nodes[j])) { reach[i] = true; changed = true; break; }
    }
  }
  const reachable = s => {
    const p = s.aprobe || { x: s.cx, y: s.cy };
    if (spines.some(sp => pointInPoly(p, sp.poly))) return true;
    for (let i = 0; i < nodes.length; i++) if (reach[i] && pointInPoly(p, nodes[i])) return true;
    return false;
  };
  const kept = sol.stalls.filter(reachable);
  // safety net: if the circulation failed to seed (kept too few), keep all rather than
  // wrongly wiping the lot.
  const useReach = kept.length >= sol.stalls.length * 0.4;
  sol.unreachable = useReach ? (sol.stalls.length - kept.length) : 0;
  if (useReach) sol.stalls = kept;
  sol.connectors = connectors.concat(spines);
  sol.count = sol.stalls.length;
}

/* ------------------------------ main solve -------------------------------- */
function solve(input) {
  // input: {boundary:[{x,y}], buildings:[poly], obstacles:[poly], params, opts}
  const { boundary } = input;
  if (!boundary || boundary.length < 3) return null;

  const preset = ANGLE_PRESETS[input.params.angle] || ANGLE_PRESETS[90];
  const p = {
    stallW: input.params.stallW,
    vpd: vpdFor(input.params.angle, input.params.stallW, input.params.stallD),   // depth-to-wall by angle (90°→stallD)
    aisle: input.params.aisle,
    setback: input.params.setback,
    angle: input.params.angle,
    greenBuffer: input.params.greenBuffer || 0,
    maxRun: input.params.maxRun || 0,
    maxRunGap: input.params.maxRunGap || 0,
    compactPct: input.opts ? (input.opts.compactPct || 0) : 0,   // share of each row packed as compact
    compactW: input.params.compactW || 0,                        // compact stall width (0 → auto 0.83×)
  };
  const blockers = [].concat(input.buildings || [], input.obstacles || []);
  const ctx = { boundary, blockers, p, center: centroid(boundary) };

  // candidateAngles lists the cleanest orientations first (parcel edges, then
  // 0/90, then a sweep). We keep the cleaner one unless a later angle packs
  // MEANINGFULLY more (>3%) — so a rotated parcel still gets its real best, but
  // a rectangular site won't go diagonal just to gain a stall or two.
  const angles = candidateAngles(boundary, input.params.orient);
  const TOL = 1.03;
  let best = null;
  for (const t of angles) {
    const res = packAtAngle(t, ctx);
    if (!best || res.count > best.count * TOL) best = res;
  }
  if (!best) return null;

  // build the circulation network from entrances; drop unreachable stalls
  const access = input.params.access || 'multi';                // open | single | multi
  const ents = access === 'single' ? (input.entrances || []).slice(0, 1) : (input.entrances || []);
  buildCirculation(best, boundary, ents, p.aisle, access === 'open', input.buildings || [], input.obstacles || []);

  // USER ROADS are DRIVES, not no-go zones: the grid packs right up to both sides and stays reachable via
  // its own aisles — we only clear the stalls that physically sit ON the road (cars don't park on the drive).
  // So drawing a road re-flows parking around it instead of blanking a whole corridor.
  if (input.roads && input.roads.length) {
    best.stalls = best.stalls.filter(s => !input.roads.some(r => polyOverlap(s.poly, r)));
    best.count = best.stalls.length;
  }

  // PARKING ZONES: if the user drew explicit parking areas, keep only the stalls whose centre falls inside one
  // (free-shape surface parking — park HERE, not the whole site). No zones drawn → pack the whole lot as before.
  if (input.parkZones && input.parkZones.length) {
    best.stalls = best.stalls.filter(s => input.parkZones.some(z => pointInPoly({ x: s.cx, y: s.cy }, z)));
    best.aisles = (best.aisles || []).filter(a => input.parkZones.some(z => polyOverlap(a.poly, z)));   // drop the empty aisles outside the zone
    best.count = best.stalls.length;
  }

  // focus point for ADA = nearest building centroid, else site centroid
  let focus = ctx.center;
  if (input.buildings && input.buildings.length) focus = centroid(input.buildings[0]);

  assignTypes(best, {
    adaMode: input.opts.adaMode, adaManual: input.opts.adaManual,
    evPct: input.opts.evPct, compactPct: input.opts.compactPct, motoPct: input.opts.motoPct, focus,
  });

  // metrics
  const siteArea = polyArea(boundary);
  const bArea = (input.buildings || []).reduce((s, b) => s + polyArea(b), 0);
  best.metrics = {
    siteArea, buildingArea: bArea,
    bestAngleDeg: Math.round(best.theta * 180 / Math.PI),
  };
  return best;
}

/* =========================================================================
   SITE SOLVER — building massing, unit mix, yield, zoning compliance
   ========================================================================= */

function polyScaleAbout(poly, k, c) {
  c = c || centroid(poly);
  return poly.map(p => ({ x: c.x + (p.x - c.x) * k, y: c.y + (p.y - c.y) * k }));
}

// Sutherland–Hodgman clip: keep the half-plane where dot(n, p-c0) >= 0.
function clipHP(poly, nx, ny, cx, cy) {
  const f = p => nx * (p.x - cx) + ny * (p.y - cy);
  const out = [];
  for (let i = 0; i < poly.length; i++) {
    const cur = poly[i], prev = poly[(i + poly.length - 1) % poly.length];
    const fc = f(cur), fp = f(prev);
    if (fc >= 0) {
      if (fp < 0) { const t = fp / (fp - fc); out.push({ x: prev.x + t * (cur.x - prev.x), y: prev.y + t * (cur.y - prev.y) }); }
      out.push(cur);
    } else if (fp >= 0) {
      const t = fp / (fp - fc); out.push({ x: prev.x + t * (cur.x - prev.x), y: prev.y + t * (cur.y - prev.y) });
    }
  }
  return out;
}

// Buildable envelope = parcel inset by a (possibly per-edge) setback.
function buildableEnvelope(parcel, setbackOf) {
  const c = centroid(parcel);
  let poly = parcel.slice();
  for (let i = 0; i < parcel.length; i++) {
    const a = parcel[i], b = parcel[(i + 1) % parcel.length];
    const dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy) || 1;
    let nx = -dy / L, ny = dx / L;
    const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
    if (nx * (c.x - mx) + ny * (c.y - my) < 0) { nx = -nx; ny = -ny; }   // point inward
    const d = setbackOf(i, a, b);
    poly = clipHP(poly, nx, ny, mx + nx * d, my + ny * d);
    if (poly.length < 3) return [];
  }
  return poly;
}

function makeSetbackClassifier(parcel, entrances, sb) {
  const mids = parcel.map((a, i) => { const b = parcel[(i + 1) % parcel.length]; return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 }; });
  let frontIdx = 0;
  if (entrances && entrances.length) {
    let best = Infinity;
    mids.forEach((m, i) => entrances.forEach(e => { const d = Math.hypot(m.x - e.x, m.y - e.y); if (d < best) { best = d; frontIdx = i; } }));
  } else {
    let best = -1;
    parcel.forEach((a, i) => { const b = parcel[(i + 1) % parcel.length]; const L = Math.hypot(b.x - a.x, b.y - a.y); if (L > best) { best = L; frontIdx = i; } });
  }
  let rearIdx = 0, fd = -1;
  mids.forEach((m, i) => { const d = Math.hypot(m.x - mids[frontIdx].x, m.y - mids[frontIdx].y); if (d > fd) { fd = d; rearIdx = i; } });
  return i => i === frontIdx ? sb.front : i === rearIdx ? sb.rear : sb.side;
}

// unlevered IRR via bisection (cfs[0] is the t0 outflow, negative)
function irr(cfs) {
  const npv = r => cfs.reduce((s, cf, t) => s + cf / Math.pow(1 + r, t), 0);
  let lo = -0.95, hi = 2.0;
  if (npv(lo) * npv(hi) > 0) return null;            // no sign change in range → undefined
  for (let i = 0; i < 200; i++) {
    const m = (lo + hi) / 2, v = npv(m);
    if (Math.abs(v) < 1) return m;
    if (npv(lo) * v < 0) hi = m; else lo = m;
  }
  return (lo + hi) / 2;
}

function computeFinancials(f, m) {
  f = f || {};
  const hard = m.gfa * (f.hardCost || 0);
  const soft = hard * ((f.softPct || 0) / 100);
  const totalCost = (f.landCost || 0) + hard + soft;
  const annualRevenue = m.residential ? m.units * (f.rentMo || 0) * 12 : m.nrsf * (f.rentSfYr || 0);
  const noi = annualRevenue * (1 - (f.opexPct == null ? 35 : f.opexPct) / 100);
  const yieldOnCost = totalCost > 0 ? noi / totalCost * 100 : 0;
  // multi-year unlevered pro-forma: NOI grows, sale at exit cap on forward NOI
  const g = (f.rentGrowth || 0) / 100, N = Math.max(1, Math.round(f.holdYears || 5)), exitCap = (f.exitCap || 0) / 100;
  const cashflows = [-totalCost];
  for (let t = 1; t <= N; t++) cashflows.push(noi * Math.pow(1 + g, t - 1));
  let exitValue = 0;
  if (exitCap > 0) { exitValue = noi * Math.pow(1 + g, N) / exitCap; cashflows[N] += exitValue; }
  const r = totalCost > 0 ? irr(cashflows) : null;
  const equityMultiple = totalCost > 0 ? cashflows.slice(1).reduce((s, c) => s + c, 0) / totalCost : 0;
  return { totalCost, hard, soft, landCost: f.landCost || 0, annualRevenue, noi, yieldOnCost,
           exitValue, holdYears: N, irr: r == null ? null : r * 100, equityMultiple, cashflows };
}

// One structured-garage DECK packed inside `footprint`: stalls + a REAL express ramp
// (a drive bay whose stalls are physically removed, not just discounted) + a structural
// COLUMN grid. Returns geometry in WORLD coords + the per-deck stall count after the ramp.
function structuredDeck(footprint, p, entrances) {
  const deck = solve({
    boundary: footprint, buildings: [], obstacles: [], entrances,
    params: { angle: p.parkAngle || 90, stallW: 9, stallD: 18, aisle: 24, setback: 0, orient: 'auto', access: 'open' },
    opts: { adaMode: 'off', adaManual: 0, evPct: 0, compactPct: 0 },
  });
  if (!deck) return { stalls: [], aisles: [], ramp: null, columns: [], perFloor: 0, theta: 0 };
  const th = deck.theta || 0, c = centroid(footprint);
  const run = { x: Math.cos(th), y: Math.sin(th) }, pd = { x: -Math.sin(th), y: Math.cos(th) };
  const toLocal = pt => ({ r: (pt.x - c.x) * run.x + (pt.y - c.y) * run.y, q: (pt.x - c.x) * pd.x + (pt.y - c.y) * pd.y });
  const toWorld = (r, q) => ({ x: c.x + r * run.x + q * pd.x, y: c.y + r * run.y + q * pd.y });
  const fl = footprint.map(toLocal);
  const minR = Math.min(...fl.map(p => p.r)), maxR = Math.max(...fl.map(p => p.r));
  const minQ = Math.min(...fl.map(p => p.q)), maxQ = Math.max(...fl.map(p => p.q));
  // EXPRESS RAMP: a two-way drive bay along the low-r edge, centred across the deck; its stalls go away.
  const rampW = 24, rampLen = Math.min(Math.max(maxR - minR, 1) * 0.45, 64);
  const qMid = (minQ + maxQ) / 2;
  const ramp = [toWorld(minR, qMid - rampW / 2), toWorld(minR + rampLen, qMid - rampW / 2),
                toWorld(minR + rampLen, qMid + rampW / 2), toWorld(minR, qMid + rampW / 2)];
  const stalls = deck.stalls.filter(s => !polyOverlap(s.poly, ramp));
  // COLUMN GRID: posts on a ~27ft structural grid, clipped to the footprint (drawn, ~no stall loss).
  const colSp = 27, columns = [];
  for (let r = minR + colSp / 2; r < maxR; r += colSp)
    for (let q = minQ + colSp / 2; q < maxQ; q += colSp) {
      const w = toWorld(r, q);
      if (pointInPoly(w, footprint)) columns.push(w);
    }
  return { stalls, aisles: deck.aisles, ramp, columns, perFloor: stalls.length, theta: th };
}

// TOWNHOME SUBDIVISION: lay rows of townhouse lots inside the buildable envelope, aligned to its
// longest edge — double-loaded bands (access drive + two back-to-back rows). Returns the unit
// rectangles (world coords) actually fitting inside the parcel. A real lot count, not a GFA estimate.
function subdivisionLayout(envelope, p, blockers) {
  const block = blockers || [];
  if (!envelope || envelope.length < 3) return { units: [], houses: [], drives: [], theta: 0, count: 0, subType: 'townhome' };
  // LOT TYPES: townhome = attached (house fills the lot); detached/cottage = a smaller house inset in a wider lot.
  const LOT = {
    townhome: { Wu: 20, Du: 40, side: 0, front: 0, rear: 0 },
    detached: { Wu: 52, Du: 92, side: 7, front: 18, rear: 25 },
    cottage:  { Wu: 34, Du: 60, side: 5, front: 12, rear: 16 },   // small-lot / tiny homes / ADU
  };
  const L = LOT[p.subType] || LOT.townhome;
  const Wu = p.townhouseW || L.Wu, Du = p.townhouseD || L.Du, drive = p.subDrive || 24;   // ft
  let th = 0, best = -1;
  for (let i = 0; i < envelope.length; i++) { const a = envelope[i], b = envelope[(i + 1) % envelope.length]; const Le = Math.hypot(b.x - a.x, b.y - a.y); if (Le > best) { best = Le; th = Math.atan2(b.y - a.y, b.x - a.x); } }
  const c = centroid(envelope);
  const run = { x: Math.cos(th), y: Math.sin(th) }, pd = { x: -Math.sin(th), y: Math.cos(th) };
  const toLocal = pt => ({ r: (pt.x - c.x) * run.x + (pt.y - c.y) * run.y, q: (pt.x - c.x) * pd.x + (pt.y - c.y) * pd.y });
  const toWorld = (r, q) => ({ x: c.x + r * run.x + q * pd.x, y: c.y + r * run.y + q * pd.y });
  const el = envelope.map(toLocal);
  const minR = Math.min(...el.map(p => p.r)), maxR = Math.max(...el.map(p => p.r));
  const minQ = Math.min(...el.map(p => p.q)), maxQ = Math.max(...el.map(p => p.q));
  const rect = (r0, q0, r1, q1) => [toWorld(r0, q0), toWorld(r1, q0), toWorld(r1, q1), toWorld(r0, q1)];
  const units = [], houses = [], drives = [], band = drive + 2 * Du;
  for (let q0 = minQ; q0 + drive + Du <= maxQ + 1; q0 += band) {
    const dq0 = q0, dq1 = q0 + drive;                                  // the access drive of this band
    if (dq1 <= maxQ) drives.push(rect(minR, dq0, maxR, dq1));
    for (const rowQ of [q0 + drive, q0 + drive + Du]) {                // two back-to-back rows
      if (rowQ + Du > maxQ + 1) break;
      for (let r = minR; r + Wu <= maxR + 1; r += Wu) {
        const lot = rect(r, rowQ, r + Wu, rowQ + Du);
        if (!lot.every(pt => pointInPoly(pt, envelope)) || block.some(b => polyOverlap(lot, b))) continue;
        units.push(lot);
        houses.push(L.side || L.front || L.rear                       // detached/cottage: house inset within the lot
          ? rect(r + L.side, rowQ + L.front, r + Wu - L.side, rowQ + Du - L.rear)
          : lot);
      }
    }
  }
  return { units, houses, drives, theta: th, count: units.length, unitW: Wu, unitD: Du, subType: p.subType || 'townhome' };
}

// INDUSTRIAL / LOGISTICS WAREHOUSE: a single-storey clear-span box with loading docks on one
// (single-dock) or both (cross-dock) long faces, a paved truck court in front of each dock face,
// and a row of 53' trailer stalls at the far side of each court. Aligned to the parcel's longest
// edge. Returns REAL dock-door + trailer counts (not a GFA estimate). Staff cars are packed into the
// leftover end-yards by solve() with the box + courts passed as no-go zones. Returns {ok:false} when
// the parcel is too small for a real warehouse, so the caller falls back to generic massing.
function industrialLayout(envelope, p, blockers) {
  const block = blockers || [];
  const dockType = p.dockType === 'single' ? 'single' : 'cross';
  const empty = { warehouse: null, dockDoors: [], truckCourts: [], trailerStalls: [], dockType, dockCount: 0, trailerCount: 0, theta: 0, ok: false };
  if (!envelope || envelope.length < 3) return empty;
  // 1. orient to the longest edge (local r = along that edge, q = perpendicular)
  let th = 0, best = -1;
  for (let i = 0; i < envelope.length; i++) { const a = envelope[i], b = envelope[(i + 1) % envelope.length]; const L = Math.hypot(b.x - a.x, b.y - a.y); if (L > best) { best = L; th = Math.atan2(b.y - a.y, b.x - a.x); } }
  const c = centroid(envelope);
  const run = { x: Math.cos(th), y: Math.sin(th) }, pd = { x: -Math.sin(th), y: Math.cos(th) };
  const toWorld = (r, q) => ({ x: c.x + r * run.x + q * pd.x, y: c.y + r * run.y + q * pd.y });
  const el = envelope.map(pt => ({ r: (pt.x - c.x) * run.x + (pt.y - c.y) * run.y, q: (pt.x - c.x) * pd.x + (pt.y - c.y) * pd.y }));
  const minR = Math.min(...el.map(p => p.r)), maxR = Math.max(...el.map(p => p.r));
  const minQ = Math.min(...el.map(p => p.q)), maxQ = Math.max(...el.map(p => p.q));
  const Lr = maxR - minR, Dq = maxQ - minQ;
  // 2. truck court depth = a 53' trailer row + a maneuvering apron, on the dock side(s)
  const TRAILER_L = 53, TRAILER_W = 12, DOOR_SP = 12, DOOR_W = 9, DOOR_D = 4, MIN_BLDG_D = 100;
  const nCourts = dockType === 'cross' ? 2 : 1;
  let court = Math.max(p.truckCourt || 130, TRAILER_L + 40);
  let bDepth = Dq - nCourts * court;
  if (bDepth < MIN_BLDG_D) {                                  // shrink courts (to a floor) before giving up depth
    court = Math.max(TRAILER_L + 24, (Dq - MIN_BLDG_D) / nCourts);
    bDepth = Dq - nCourts * court;
  }
  if (bDepth < 60 || Lr < 170) return empty;                  // parcel too small → generic massing fallback
  // 3. front car-parking yard: sized to the staff-parking demand (est. from a full-length box), capped.
  //    Cross-dock has courts on BOTH long faces, so staff cars can only go in this front yard.
  const ratio = p.parkingRatio || 0;
  const reqStalls = Math.ceil((Lr - 30) * bDepth / 1000 * ratio);
  let frontYard = Dq > 0 ? reqStalls * 430 / Dq : 0;         // ~430 sqft per 90° stall (incl. aisle + shallow-yard packing loss + access-drive overhead)
  frontYard = Math.min(Math.max(frontYard, ratio > 0 ? 60 : 0), Lr * 0.45);
  const bLen = Lr - frontYard - (frontYard > 0 ? 10 : 0);    // 10ft buffer between the lot and the box
  if (bLen < 100) return empty;
  const r0 = maxR - bLen, r1 = maxR;                          // box at the rear; front yard fills r < r0
  // 4. building rectangle: centred in q (cross-dock) or backed to the high-q edge (single-dock)
  let q0, q1;
  if (dockType === 'cross') { q0 = minQ + court; q1 = maxQ - court; }
  else { q1 = maxQ; q0 = q1 - bDepth; }
  const warehouse = [toWorld(r0, q0), toWorld(r1, q0), toWorld(r1, q1), toWorld(r0, q1)];
  // 5. per dock face: doors on the wall, the court apron in front, a trailer row at the far side
  const faces = dockType === 'cross' ? [{ q: q0, dir: -1, edge: minQ }, { q: q1, dir: +1, edge: maxQ }]
                                     : [{ q: q0, dir: -1, edge: minQ }];
  const dockDoors = [], truckCourts = [], trailerStalls = [];
  for (const f of faces) {
    for (let r = r0 + DOOR_SP / 2; r + DOOR_W <= r1; r += DOOR_SP) {        // dock doors along the face
      const qa = f.q, qb = f.q + f.dir * DOOR_D;
      const dd = [toWorld(r, qa), toWorld(r + DOOR_W, qa), toWorld(r + DOOR_W, qb), toWorld(r, qb)];
      if (!block.some(b => polyOverlap(dd, b))) dockDoors.push(dd);
    }
    truckCourts.push([toWorld(r0, f.q), toWorld(r1, f.q), toWorld(r1, f.edge), toWorld(r0, f.edge)]);  // paved apron
    const tFar = f.edge, tInner = f.edge - f.dir * TRAILER_L;               // 53' trailer row against the parcel edge
    for (let r = r0; r + TRAILER_W <= r1; r += TRAILER_W) {
      const ts = [toWorld(r, tFar), toWorld(r + TRAILER_W, tFar), toWorld(r + TRAILER_W, tInner), toWorld(r, tInner)];
      if (!block.some(b => polyOverlap(ts, b))) trailerStalls.push(ts);
    }
  }
  return { warehouse, dockDoors, truckCourts, trailerStalls, dockType, dockCount: dockDoors.length, trailerCount: trailerStalls.length, theta: th, courtDepth: court, frontYard, clearHeight: p.floorHeight, ok: true };
}

// GARDEN / LOW-RISE WALK-UP: rows of double-loaded residential bar buildings with surface-parking drive
// bands between them — the classic suburban garden-apartment massing (MANY buildings, not one blob). Returns
// the real bar footprints (world coords); units are counted from total bar floor area back in solveSite. Cars
// pack into the bands by solve() with the bars passed as buildings. Aligned to the parcel's longest edge.
function gardenLayout(envelope, p, blockers) {
  const block = blockers || [];
  const empty = { bars: [], theta: 0, totalBarArea: 0, rows: 0, barDepth: 0, barLen: 0 };
  if (!envelope || envelope.length < 3) return empty;
  let th = 0, best = -1;
  for (let i = 0; i < envelope.length; i++) { const a = envelope[i], b = envelope[(i + 1) % envelope.length]; const L = Math.hypot(b.x - a.x, b.y - a.y); if (L > best) { best = L; th = Math.atan2(b.y - a.y, b.x - a.x); } }
  const c = centroid(envelope);
  const run = { x: Math.cos(th), y: Math.sin(th) }, pd = { x: -Math.sin(th), y: Math.cos(th) };
  const toWorld = (r, q) => ({ x: c.x + r * run.x + q * pd.x, y: c.y + r * run.y + q * pd.y });
  const el = envelope.map(pt => ({ r: (pt.x - c.x) * run.x + (pt.y - c.y) * run.y, q: (pt.x - c.x) * pd.x + (pt.y - c.y) * pd.y }));
  const minR = Math.min(...el.map(p => p.r)), maxR = Math.max(...el.map(p => p.r));
  const minQ = Math.min(...el.map(p => p.q)), maxQ = Math.max(...el.map(p => p.q));
  const barDepth = Math.max(p.gardenBarDepth || 66, 40);   // double-loaded: ~30ft units both sides + 6ft corridor
  const parkBand = 62, endMargin = 26, segGap = 26, maxSeg = 200;   // perimeter + cross breezeways = drive lanes the packer routes through
  const r0 = minR + endMargin, r1 = maxR - endMargin, span = r1 - r0;
  if (span < 90 || maxQ - minQ < barDepth + 20) return empty;
  const bars = []; let totalBarArea = 0;
  for (let q = minQ + 8; q + barDepth <= maxQ - 8 + 1e-6; q += barDepth + parkBand) {   // bar row, then a parking band
    const nSeg = Math.max(1, Math.round(span / (maxSeg + segGap)));   // split long rows into separate buildings with breezeways
    const segLen = (span - (nSeg - 1) * segGap) / nSeg;
    if (segLen < 50) continue;
    for (let k = 0; k < nSeg; k++) {
      const sr0 = r0 + k * (segLen + segGap), sr1 = sr0 + segLen;
      const bar = [toWorld(sr0, q), toWorld(sr1, q), toWorld(sr1, q + barDepth), toWorld(sr0, q + barDepth)];
      if (bar.every(pt => pointInPoly(pt, envelope)) && !block.some(b => polyOverlap(bar, b))) { bars.push(bar); totalBarArea += segLen * barDepth; }
    }
  }
  return { bars, theta: th, totalBarArea, rows: bars.length, barDepth, barLen: span };
}

// HOTEL: one double-loaded corridor slab (a single consolidated building, unlike garden's grid of bars),
// backed to a long edge so a cohesive parking field sits in front. Returns the bar footprint + corridor depth;
// room keys are counted from total floor area in solveSite. {ok:false} on a too-small parcel → generic massing.
function hotelLayout(envelope, p) {
  if (!envelope || envelope.length < 3) return { ok: false };
  let th = 0, best = -1;
  for (let i = 0; i < envelope.length; i++) { const a = envelope[i], b = envelope[(i + 1) % envelope.length]; const L = Math.hypot(b.x - a.x, b.y - a.y); if (L > best) { best = L; th = Math.atan2(b.y - a.y, b.x - a.x); } }
  const c = centroid(envelope);
  const run = { x: Math.cos(th), y: Math.sin(th) }, pd = { x: -Math.sin(th), y: Math.cos(th) };
  const toWorld = (r, q) => ({ x: c.x + r * run.x + q * pd.x, y: c.y + r * run.y + q * pd.y });
  const el = envelope.map(pt => ({ r: (pt.x - c.x) * run.x + (pt.y - c.y) * run.y, q: (pt.x - c.x) * pd.x + (pt.y - c.y) * pd.y }));
  const minR = Math.min(...el.map(p => p.r)), maxR = Math.max(...el.map(p => p.r));
  const minQ = Math.min(...el.map(p => p.q)), maxQ = Math.max(...el.map(p => p.q));
  const Lr = maxR - minR, Dq = maxQ - minQ;
  const depth = Math.min(p.hotelBarDepth || 62, Dq * 0.55);   // double-loaded corridor; keep ≥45% depth for parking
  const endMargin = Math.min(Math.max(Lr * 0.05, 20), 60);
  const r0 = minR + endMargin, r1 = maxR - endMargin;
  if (r1 - r0 < 100 || depth < 40) return { ok: false };
  const q1 = maxQ, q0 = q1 - depth;                           // slab backed to the high-q edge; parking fills q < q0
  const bar = [toWorld(r0, q0), toWorld(r1, q0), toWorld(r1, q1), toWorld(r0, q1)];
  return { bar, theta: th, barDepth: depth, barLen: r1 - r0, ok: true };
}

// RETAIL CENTRE: an anchor / inline-shop strip backed to the rear edge, a row of pad outparcels (banks,
// fast-food) near the street, and a large surface parking field between — the parking-dominated retail
// typology. Returns the anchor + pad footprints; GLA is single-storey. Cars pack the field via solve().
function retailLayout(envelope, p, blockers) {
  const block = blockers || [];
  if (!envelope || envelope.length < 3) return { ok: false };
  let th = 0, best = -1;
  for (let i = 0; i < envelope.length; i++) { const a = envelope[i], b = envelope[(i + 1) % envelope.length]; const L = Math.hypot(b.x - a.x, b.y - a.y); if (L > best) { best = L; th = Math.atan2(b.y - a.y, b.x - a.x); } }
  const c = centroid(envelope);
  const run = { x: Math.cos(th), y: Math.sin(th) }, pd = { x: -Math.sin(th), y: Math.cos(th) };
  const toWorld = (r, q) => ({ x: c.x + r * run.x + q * pd.x, y: c.y + r * run.y + q * pd.y });
  const el = envelope.map(pt => ({ r: (pt.x - c.x) * run.x + (pt.y - c.y) * run.y, q: (pt.x - c.x) * pd.x + (pt.y - c.y) * pd.y }));
  const minR = Math.min(...el.map(p => p.r)), maxR = Math.max(...el.map(p => p.r));
  const minQ = Math.min(...el.map(p => p.q)), maxQ = Math.max(...el.map(p => p.q));
  const Lr = maxR - minR, Dq = maxQ - minQ;
  if (Lr < 150 || Dq < 160) return { ok: false };
  const depth = Math.min(p.retailDepth || 120, Dq * 0.34);     // anchor + inline shops at the rear (shallow → big parking field for retail's high ratio)
  const endMargin = Math.min(Math.max(Lr * 0.05, 20), 70);
  const r0 = minR + endMargin, r1 = maxR - endMargin;
  const aq1 = maxQ, aq0 = aq1 - depth;                          // anchor backed to the high-q (rear) edge
  const anchor = [toWorld(r0, aq0), toWorld(r1, aq0), toWorld(r1, aq1), toWorld(r0, aq1)];
  // pad outparcels: a row of small boxes near the street (low-q), evenly spaced along the frontage
  const pads = [], padW = 74, padD = 64, padQ0 = minQ + 8, padQ1 = padQ0 + padD;
  const nPads = Math.max(2, Math.min(4, Math.floor(Lr / 230)));
  const gap = (r1 - r0 - nPads * padW) / (nPads + 1);
  if (gap > 30 && padQ1 < aq0 - 80) {                          // only if pads + a real parking field fit
    for (let k = 0; k < nPads; k++) {
      const pr0 = r0 + gap + k * (padW + gap), pr1 = pr0 + padW;
      const pad = [toWorld(pr0, padQ0), toWorld(pr1, padQ0), toWorld(pr1, padQ1), toWorld(pr0, padQ1)];
      if (pad.every(pt => pointInPoly(pt, envelope)) && !block.some(b => polyOverlap(pad, b))) pads.push(pad);
    }
  }
  return { anchor, pads, theta: th, gla: polyArea(anchor) + pads.reduce((s, p) => s + polyArea(p), 0), ok: true };
}

// DATA CENTRE: a large data-hall box backed to the rear, a fenced mechanical/generator yard apron in front
// of it (chillers + gensets), a substation pad at a front corner, and a small staff lot. Multi-level halls
// allowed. Returns the hall + equipment yards; the yards are no-go zones the staff lot packs around.
function datacenterLayout(envelope, p) {
  if (!envelope || envelope.length < 3) return { ok: false };
  let th = 0, best = -1;
  for (let i = 0; i < envelope.length; i++) { const a = envelope[i], b = envelope[(i + 1) % envelope.length]; const L = Math.hypot(b.x - a.x, b.y - a.y); if (L > best) { best = L; th = Math.atan2(b.y - a.y, b.x - a.x); } }
  const c = centroid(envelope);
  const run = { x: Math.cos(th), y: Math.sin(th) }, pd = { x: -Math.sin(th), y: Math.cos(th) };
  const toWorld = (r, q) => ({ x: c.x + r * run.x + q * pd.x, y: c.y + r * run.y + q * pd.y });
  const el = envelope.map(pt => ({ r: (pt.x - c.x) * run.x + (pt.y - c.y) * run.y, q: (pt.x - c.x) * pd.x + (pt.y - c.y) * pd.y }));
  const minR = Math.min(...el.map(p => p.r)), maxR = Math.max(...el.map(p => p.r));
  const minQ = Math.min(...el.map(p => p.q)), maxQ = Math.max(...el.map(p => p.q));
  const Lr = maxR - minR, Dq = maxQ - minQ;
  if (Lr < 160 || Dq < 200) return { ok: false };
  const mechD = 56;                                            // mechanical/generator yard depth in front of the hall
  const hallD = Math.min(Dq * 0.5, Dq - mechD - 90);           // data-hall depth (rear); keep room for yard + a staff lot
  if (hallD < 90) return { ok: false };
  const endMargin = Math.min(Math.max(Lr * 0.05, 24), 80);
  const r0 = minR + endMargin, r1 = maxR - endMargin;
  const hq1 = maxQ, hq0 = hq1 - hallD;                         // hall backed to the high-q (rear) edge
  const hall = [toWorld(r0, hq0), toWorld(r1, hq0), toWorld(r1, hq1), toWorld(r0, hq1)];
  const my0 = hq0 - mechD, my1 = hq0;                          // mechanical yard apron in front of the hall
  const mechYard = [toWorld(r0, my0), toWorld(r1, my0), toWorld(r1, my1), toWorld(r0, my1)];
  const subW = 70, subStation = [toWorld(r0, my0 - 56), toWorld(r0 + subW, my0 - 56), toWorld(r0 + subW, my0 - 6), toWorld(r0, my0 - 6)];  // substation pad at a front corner
  const yards = [mechYard, subStation];
  return { hall, mechYard, subStation, yards, theta: th, hallDepth: hallD, ok: true };
}

// TOWER: a compact point-tower floorplate centred on the podium base (the coverage-capped footprint).
// The residential mass uses this small plate and rises many storeys; structured parking fills the wider
// podium below it. Returns the plate (≤ ~13k sqft, the high-rise floorplate) inset within the podium.
function towerPlate(podium, p) {
  const a = polyArea(podium);
  if (a < 1) return podium.slice();
  const target = Math.min(a * 0.55, p.towerPlate || 13000);   // point-tower floorplate
  return polyScaleAbout(podium, Math.min(Math.sqrt(target / a), 0.92), centroid(podium));
}

function solveSite(input) {
  const { boundary, p } = input;
  if (!boundary || boundary.length < 3) return null;
  const parcelArea = polyArea(boundary);
  const acres = parcelArea / 43560;
  const residential = p.useType === 'multifamily' || p.useType === 'mixeduse' || p.useType === 'singlefamily' || p.useType === 'garden' || p.useType === 'tower';
  const isIndustrial = p.useType === 'industrial';
  const isGarden = p.useType === 'garden';
  const isHotel = p.useType === 'hotel';
  const isTower = p.useType === 'tower';
  const isRetail = p.useType === 'retail';
  const isData = p.useType === 'datacenter';

  // 1. buildable envelope (parcel minus setbacks)
  const baseClass = makeSetbackClassifier(boundary, input.entrances, p.setbacks);
  const eo = p.edgeSetback || {};
  const setbackOf = i => (eo[i] != null ? eo[i] : baseClass(i));   // per-edge override wins over front/side/rear
  let envelope = buildableEnvelope(boundary, setbackOf);
  if (envelope.length < 3) envelope = boundary.slice();

  // BLOCKERS = user-drawn facilities (obstacles) + internal roads. Buildings AND parking flow around them,
  // so drawing a pond / easement / road re-solves the whole site (TestFit-style responsive layout).
  const blockers = (input.obstacles || []).concat(input.roads || []);

  // INDUSTRIAL warehouse / GARDEN low-rise bars: real multi-element generators; null = parcel too small → generic massing
  let industrial = null;
  if (isIndustrial) { const ind = industrialLayout(envelope, p, blockers); if (ind.ok) industrial = ind; }
  let garden = null;
  if (isGarden) { const g = gardenLayout(envelope, p, blockers); if (g.rows > 0) garden = g; }
  let hotel = null;
  if (isHotel) { const h = hotelLayout(envelope, p); if (h.ok) hotel = h; }
  let retail = null;
  if (isRetail) { const r = retailLayout(envelope, p, blockers); if (r.ok) retail = r; }
  let datacenter = null;
  if (isData) { const d = datacenterLayout(envelope, p); if (d.ok) datacenter = d; }

  // 2. footprint = the warehouse box (industrial) / hotel slab / retail anchor / data hall, else envelope capped by coverage
  const envArea = polyArea(envelope);
  const maxCovArea = parcelArea * (p.maxCoverage / 100);
  let footprint = envelope;
  if (industrial) footprint = industrial.warehouse;
  else if (hotel) footprint = hotel.bar;
  else if (retail) footprint = retail.anchor;
  else if (datacenter) footprint = datacenter.hall;
  else if (envArea > maxCovArea && envArea > 0) footprint = polyScaleAbout(envelope, Math.sqrt(maxCovArea / envArea));
  let footArea = polyArea(footprint);
  if (garden) footArea = garden.totalBarArea;   // garden coverage / FAR / units reflect the actual bars, not the envelope blob
  if (retail) footArea = retail.gla;            // retail GLA = anchor + pad outparcels (single storey)
  // TOWER: the wide coverage-capped footprint becomes the parking PODIUM; the residential mass is a slender plate on top.
  let towerPodium = null, towerPodiumArea = 0;
  if (isTower) {
    towerPodium = footprint; towerPodiumArea = polyArea(towerPodium);
    footprint = towerPlate(towerPodium, p); footArea = polyArea(footprint);
  }
  // FACILITIES / ROADS: punch any blocker overlapping the building footprint as a void — the building wraps
  // around it. (Garden bars & single-family lots are discrete, so they skip blocked pieces inside their own layout.)
  const footVoids = (isGarden || p.useType === 'singlefamily') ? [] : blockers.filter(b => polyOverlap(b, footprint));
  if (footVoids.length) footArea = Math.max(footArea - footVoids.reduce((s, v) => s + polyArea(v), 0), Math.max(footArea * 0.15, 200));
  // WRAP type: residential wraps a structured parking CORE (an inner inset of the footprint).
  // Rentable area is the RING (footprint − core); the core is the garage.
  const isWrap = p.parkingType === 'wrap';
  const wrapCore = isWrap ? polyScaleAbout(footprint, 0.6, centroid(footprint)) : null;
  const coreArea = wrapCore ? polyArea(wrapCore) : 0;
  const resiFootArea = Math.max(footArea - coreArea, 1);

  // 3. floors — limited by height AND by FAR (industrial warehouse is single-storey clear-span)
  const floorsByHeight = Math.max(1, Math.floor(p.maxHeight / p.floorHeight));
  const floorsByFAR = Math.max(1, Math.floor((p.maxFAR * parcelArea) / Math.max(footArea, 1)));
  const floors = (industrial || retail) ? 1 : Math.max(1, Math.min(floorsByHeight, floorsByFAR, garden ? 2 : Infinity));  // industrial & retail are single-storey; garden walk-ups cap at 2
  const height = floors * p.floorHeight;
  const gfa = footArea * floors;
  const far = gfa / parcelArea;
  const coverage = (isTower ? towerPodiumArea : footArea) / parcelArea * 100;   // tower: the podium covers the ground, not the slender plate
  const nrsf = (isWrap ? resiFootArea : footArea) * floors * p.efficiency;   // wrap: only the ring is rentable
  const keys = hotel ? Math.floor(nrsf / 450) : 0;   // hotel room count (~450 sqft/key incl. corridor + lobby + back-of-house)

  // 4. units (residential) or none (commercial/industrial)
  let units = 0, unitsByType = [], densityCapped = false, subdivision = null;
  if (p.useType === 'singlefamily') {
    // TOWNHOME SUBDIVISION: real physical lot count from the row layout, not a GFA estimate
    subdivision = subdivisionLayout(envelope, p, blockers);
    units = subdivision.count;
    if (p.maxDUA > 0) { const cap = Math.floor(p.maxDUA * acres); if (units > cap) { units = cap; densityCapped = true; } }
    unitsByType = [{ type: subdivision.subType || 'townhome', count: units, size: subdivision.unitW * subdivision.unitD }];
  } else if (residential) {
    const mix = p.unitMix, sumPct = mix.reduce((s, m) => s + m.pct, 0) || 1;
    const avgSize = mix.reduce((s, m) => s + m.pct * m.size, 0) / sumPct;
    units = Math.floor(nrsf / Math.max(avgSize, 1));
    if (p.maxDUA > 0) { const cap = Math.floor(p.maxDUA * acres); if (units > cap) { units = cap; densityCapped = true; } }
    unitsByType = mix.map(m => ({ type: m.type, count: Math.round(units * m.pct / sumPct), size: m.size }));
  }

  // 5. parking required vs provided
  let parkingRequired = residential
    ? Math.ceil(units * p.parkingRatio)
    : Math.ceil(gfa / 1000 * p.parkingRatio);
  let parkCapped = false;
  const structured = p.parkingType === 'structured' || isTower;        // towers always sit on a structured parking podium
  const deckType = structured || isWrap;                               // all stack multi-level parking decks
  let parkSol, parkingProvided, parkingPerFloor = 0, garage = null;
  // levels split: above grade (podium, counts toward height) + below grade (basement, excluded from height/FAR)
  const levelsAbove = deckType ? Math.max(0, Math.round(p.parkingLevelsAbove != null ? p.parkingLevelsAbove : (p.parkingLevels || 3))) : 0;
  const levelsBelow = deckType ? Math.max(0, Math.round(p.parkingLevelsBelow || 0)) : 0;
  const parkingLevels = deckType ? Math.max(1, levelsAbove + levelsBelow) : 1;
  const structEff = (p.structEff == null ? 95 : p.structEff);          // cores/columns residual (ramp now removed explicitly)
  if (subdivision) {
    // TOWNHOMES self-park: each lot has its own driveway/garage (~2 spaces), no separate surface lot.
    parkSol = { stalls: [], aisles: [], connectors: [] };
    parkingProvided = units * 2;
  } else if (industrial) {
    // WAREHOUSE staff parking: pack the leftover end-yards (box + truck courts are no-go zones).
    parkSol = solve({
      boundary, buildings: [footprint], obstacles: (input.obstacles || []).concat(industrial.truckCourts), roads: input.roads || [],
      entrances: input.entrances,
      params: { angle: p.parkAngle || 90, stallW: 9, stallD: 18, aisle: 24, setback: p.parkSetback || 5, orient: 'auto' },
      opts: { adaMode: 'code', adaManual: 0, evPct: p.evPct || 0, compactPct: 0 },
    });
    parkingProvided = parkSol ? parkSol.stalls.length : 0;
  } else if (garden) {
    // GARDEN surface parking: pack the drive bands BETWEEN the bar buildings (each bar is a no-go zone).
    // aisles forced PARALLEL to the bars (garden.theta) so each 62ft band double-loads cleanly.
    parkSol = solve({
      boundary, buildings: garden.bars, obstacles: input.obstacles || [], roads: input.roads || [], entrances: input.entrances,
      params: { angle: p.parkAngle || 90, stallW: 9, stallD: 18, aisle: 24, setback: p.parkSetback || 5, orient: garden.theta },
      opts: { adaMode: 'code', adaManual: 0, evPct: p.evPct || 0, compactPct: 0 },
    });
    parkingProvided = parkSol ? parkSol.stalls.length : 0;
    // surface parking is the garden density limiter — trim the unit yield to what actually parks
    const parkCap = Math.floor(parkingProvided / Math.max(p.parkingRatio, 0.1));
    if (units > parkCap) {
      units = parkCap; parkCapped = true;
      const mix = p.unitMix, sumPct = mix.reduce((s, m) => s + m.pct, 0) || 1;
      unitsByType = mix.map(m => ({ type: m.type, count: Math.round(units * m.pct / sumPct), size: m.size }));
      parkingRequired = Math.ceil(units * p.parkingRatio);
    }
  } else if (retail) {
    // RETAIL surface field: cars pack the big lot between the anchor and the pad outparcels (all are no-go zones).
    parkSol = solve({
      boundary, buildings: [retail.anchor].concat(retail.pads), obstacles: input.obstacles || [], roads: input.roads || [], entrances: input.entrances,
      params: { angle: p.parkAngle || 90, stallW: 9, stallD: 18, aisle: 24, setback: p.parkSetback || 5, orient: 'auto' },
      opts: { adaMode: 'code', adaManual: 0, evPct: p.evPct || 0, compactPct: 0 },
    });
    parkingProvided = parkSol ? parkSol.stalls.length : 0;
  } else if (datacenter) {
    // DATA CENTRE staff lot: cars pack the front yard (data hall + substation are buildings, the mechanical yard a no-go zone).
    parkSol = solve({
      boundary, buildings: [datacenter.hall, datacenter.subStation], obstacles: (input.obstacles || []).concat([datacenter.mechYard]), roads: input.roads || [], entrances: input.entrances,
      params: { angle: p.parkAngle || 90, stallW: 9, stallD: 18, aisle: 24, setback: p.parkSetback || 5, orient: 'auto' },
      opts: { adaMode: 'code', adaManual: 0, evPct: p.evPct || 0, compactPct: 0 },
    });
    parkingProvided = parkSol ? parkSol.stalls.length : 0;
  } else if (deckType) {
    // MULTI-LEVEL GARAGE: structured = deck over the footprint; wrap = inner core; tower = the wider podium base.
    const deckPoly = isTower ? towerPodium : isWrap ? wrapCore : footprint;
    const d = structuredDeck(deckPoly, p, input.entrances);
    parkingPerFloor = d.perFloor;
    parkingProvided = Math.round(parkingPerFloor * parkingLevels * structEff / 100);
    parkSol = { stalls: d.stalls, aisles: d.aisles, connectors: [] };  // typical deck for the 2D plan
    garage = { ramp: d.ramp, columns: d.columns, theta: d.theta, levelsAbove, levelsBelow, floorHeight: p.floorHeight, deckPoly, wrap: isWrap };
  } else {
    parkSol = solve({
      boundary, buildings: [footprint], obstacles: input.obstacles || [], roads: input.roads || [], entrances: input.entrances,
      params: { angle: p.parkAngle || 90, stallW: 9, stallD: 18, aisle: 24, setback: p.parkSetback || 5, orient: 'auto' },
      opts: { adaMode: 'code', adaManual: 0, evPct: p.evPct || 0, compactPct: 0 },
    });
    parkingProvided = parkSol ? parkSol.stalls.length : 0;
  }

  // 6. financials
  const fin = computeFinancials(p.fin, { gfa, nrsf, units, residential });

  // 6b. AUTO CORE: a service core (egress stairs + elevators) for footprint mid/high-rise buildings
  let cores = [], coreInfo = null;
  if (floors >= 3 && !subdivision && !garden && !industrial && !retail && !datacenter && footprint && footprint.length >= 3) {
    const ca = Math.min(Math.max(footArea * 0.05, 380), 2600);
    cores = [polyScaleAbout(footprint, Math.sqrt(ca / footArea), centroid(footprint))];
    coreInfo = {
      stairs: footArea > 12000 ? 3 : 2,                                    // ≥2 egress stairs; +1 for big floorplates
      elevators: Math.min(residential ? Math.max(1, Math.ceil(units / 90)) : Math.max(1, Math.ceil(floors / 6)), 10),
    };
  }

  // 6c. TURN RADIUS: the design vehicle must clear the manoeuvring space
  const ang = p.parkAngle || 90, TURN_STD = { 90: 24, 60: 18, 45: 13, 0: 24 };
  const turn = industrial
    ? { ok: industrial.courtDepth >= 120, val: `卡車迴轉場 ${Math.round(industrial.courtDepth)}ft ≥ 53呎拖車需 120ft` }
    : { ok: 24 >= (TURN_STD[ang] || 24) - 0.5, val: `車道 24ft ≥ ${ang}° 標準 ${TURN_STD[ang] || 24}ft · 雙向迴轉淨空足` };

  // 7. zoning compliance (Site Intelligence pass/fail)
  const compliance = [
    { k: 'FAR 容積', ok: far <= p.maxFAR + 1e-6, val: `${far.toFixed(2)} / 上限 ${p.maxFAR}` },
    { k: '高度 Height', ok: height <= p.maxHeight + 1e-6, val: `${Math.round(height)}ft · ${floors}F / 上限 ${p.maxHeight}ft` },
    { k: '建蔽率 Coverage', ok: coverage <= p.maxCoverage + 0.5, val: `${coverage.toFixed(0)}% / 上限 ${p.maxCoverage}%` },
    { k: '停車 Parking', ok: parkingProvided >= parkingRequired,
      val: deckType ? `${parkingProvided} / 需 ${parkingRequired}（${isWrap ? '環繞核心 ' : ''}地上${levelsAbove}+地下${levelsBelow}層 × ${parkingPerFloor}/層 × ${structEff}%）` : `${parkingProvided} / 需 ${parkingRequired}` },
  ];
  if (residential && p.maxDUA > 0)
    compliance.push({ k: '密度 Density', ok: units / acres <= p.maxDUA + 0.5, val: `${(units / acres).toFixed(1)} / 上限 ${p.maxDUA} DU/ac` });
  if (industrial)
    compliance.push({ k: '物流規模 Logistics', ok: true, info: true, val: `${industrial.dockCount} 月台門 · ${industrial.trailerCount} 拖車位 · ${industrial.dockType === 'cross' ? '雙面對流 Cross-dock' : '單面 Single-dock'}` });
  if (hotel)
    compliance.push({ k: '客房數 Keys', ok: true, info: true, val: `${keys} 間 · 雙載走廊 ${floors}F` });
  if (retail)
    compliance.push({ k: '零售規模 Retail', ok: true, info: true, val: `主力店+${retail.pads.length} pad 外帶店 · GLA ${Math.round(retail.gla).toLocaleString()} SF` });
  if (datacenter)
    compliance.push({ k: '機房規模 Data hall', ok: true, info: true, val: `資料機房 ${Math.round(gfa).toLocaleString()} SF · ${floors} 層 · 含機電中庭＋變電站` });
  compliance.push({ k: '迴轉/車道 Turning', ok: turn.ok, val: turn.val });
  if (coreInfo)
    compliance.push({ k: '核心 Cores', ok: true, info: true, val: `${coreInfo.stairs} 逃生梯 ＋ ${coreInfo.elevators} 電梯（自動配置）` });

  return {
    envelope, footprint, floors, height, gfa, far, coverage, nrsf, units, unitsByType,
    densityCapped, parkingRequired, parkingProvided, parkSol, acres, parcelArea, residential, fin, compliance,
    structured, parkingLevels, parkingPerFloor, structEff, garage, levelsAbove, levelsBelow, subdivision,
    isWrap, wrapCore, industrial, garden, parkCapped, hotel, keys, retail, datacenter, cores, coreInfo, footVoids,
    tower: isTower ? { plate: footprint, podium: towerPodium, podiumLevels: levelsAbove } : null,
  };
}

/* ------------------------------- exports --------------------------------- */
global.PS = {
  solveSite, buildableEnvelope, polyScaleAbout, clipHP,
  solve, packAtAngle, assignTypes, adaRequired, computeFinancials, inwardEdgeNormal,
  polyArea, centroid, bbox, pointInPoly, polyInPoly, polyOverlap,
  ANGLE_PRESETS,
};

})(window);
