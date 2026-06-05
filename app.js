/* =========================================================================
   Parking Solver — UI / canvas / interaction layer
   ========================================================================= */
(function () {
'use strict';

const $ = s => document.querySelector(s);
const cv = $('#cv');
const ctx = cv.getContext('2d');

/* ----------------------------- state ------------------------------------- */
const S = {
  boundary: [],            // [{x,y}] in feet, single polygon
  buildings: [],           // [poly]
  obstacles: [],           // [poly]
  roads: [],               // [poly] road strips fed to the solver (block buildings / clear stalls)
  roadLines: [],           // [{line:[pts], width}] road centre-lines — rendered as smooth continuous roads
  roadWidth: 24,           // ft, internal road width
  parkZones: [],           // [poly] free-shape parking areas — when set, stalls pack ONLY inside these
  manualCores: [],         // [poly] user-placed service cores (stairs+elevators); override the auto core when present
  contextBuildings: [],    // [{poly,height}] surrounding existing buildings from OSM (3D context, map mode)
  entrances: [],           // [{x,y}]
  solution: null,          // result from PS.solve
  tool: 'select',
  params: { angle: 90, stallW: 9, stallD: 18, aisle: 24, setback: 5, orient: 'auto', height: 35, oneway: false, greenBuffer: 0, maxRun: 0, maxRunGap: 9, compactW: 7.5, access: 'multi' },
  opts: { adaMode: 'code', adaManual: 4, evPct: 8, compactPct: 0, motoPct: 0, gfa: 0 },
  view: { scale: 1, ox: 0, oy: 0 },          // px per foot, offset px
  is3d: false, iso: { cx: 0, cy: 0 },        // isometric massing view
  mapMode: false, map: null,                 // Leaflet real-world map base
  geo: { lat0: 23.1015, lng0: 120.2785, set: false, firstOpen: true },  // default: 南科 STSP
  mode: 'parking', site: null,               // parking | site (building massing)
  siteParkAngle: 90,                         // parking angle used inside site solver
  showTrees: true, _trees: null,             // landscape trees (cached positions)
  parcels: null, activeParcel: 0, splitPt: null,  // subdivision: sub-parcels
  draft: null,             // in-progress polygon points
  draftKind: null,         // 'boundary'|'building'|'obstacle'
  hoverWorld: null,
  dragVertex: null,        // {poly,idx} when editing
  dragRoad: null,          // {ri,pi} dragging a road centre-line vertex, or {ri,body:true,last} moving a whole road
  selRoad: null,           // index into roadLines of the selected/editable road
  dragBldgVtx: null,       // {bi,pi} dragging a building footprint corner (massing footprint edit → parking re-packs)
  selAisle: null,          // index of the selected drive aisle (manual circulation editing)
  dragSpine: null,         // {si,ni} dragging a drive-aisle spine end node → that aisle reshapes + stalls re-attach
  dragSpineBody: null,     // {si,last} dragging a whole drive aisle (the line, not an end) → move the lane + stalls
  dragBoundaryEdge: null,  // {i,last,moved} dragging a whole boundary EDGE (stretch the site)
  dragBldgEdge: null,      // {bi,ei,last} dragging a building EDGE
  dragBldgBody: null,      // {bi,last,moved} dragging a whole building (move its position)
  dragObsVtx: null,        // {oi,pi} obstacle corner · dragObsEdge {oi,ei,last} edge · dragObsBody {oi,last,moved} whole move
  dragObsEdge: null,
  dragObsBody: null,
  selObstacle: null,       // index of the selected obstacle (for Delete)
  panning: false, panStart: null,
  selStall: null, selBuilding: null,
  history: [], hIdx: -1, _restoring: false,   // undo / redo snapshot stack
  measures: [], measureStart: null,           // tape-measure dimension annotations
  selEdge: null, edgeSetback: {},             // per-edge setback overrides (site mode)
  layers: {                 // per-category visibility + lock (object-tree eye / lock)
    site:     { lock: false },
    parking:  { vis: true, lock: false },
    building: { vis: true, lock: false },
    obstacle: { vis: true, lock: false },
    entrance: { vis: true, lock: false },
    trees:    { vis: true },
    flow:     { vis: false },        // 動線體檢 congestion heat-map layer (off by default)
    earthwork:{ vis: false },        // 挖填方 cut/fill gradation layer (off by default)
    unitfit:  { vis: true },         // 戶型平面 unit-fit floor plan (on by default — the signature view)
  },
};

const COLORS = {
  standard:'#3b82f6', compact:'#22c55e', ada:'#1d4ed8', ev:'#10b981', trailer:'#f59e0b', moto:'#a855f7',
};
const LABELS = { standard:'標準 Standard', compact:'小型 Compact', ada:'♿ 無障礙 ADA', ev:'⚡ EV 充電', trailer:'拖車 Trailer', moto:'🏍️ 機車 Motorcycle' };
// object-tree eye/lock capabilities per category (site can't be hidden, trees can't be locked)
const LAYER_CAPS = { site:{lock:1}, parking:{vis:1,lock:1}, building:{vis:1,lock:1}, obstacle:{vis:1,lock:1}, entrance:{vis:1,lock:1}, trees:{vis:1}, flow:{vis:1}, earthwork:{vis:1}, unitfit:{vis:1} };
// unit-fit floor-plan colours by residential type (TestFit-style colour-coded apartment units)
const UNIT_FIT_COLORS = { studio:'#fbbf24', '1br':'#60a5fa', '2br':'#34d399', '3br':'#f472b6' };
const UNIT_FIT_LABEL = { studio:'套房', '1br':'一房', '2br':'二房', '3br':'三房' };
// beds / baths per unit type (for DU/AC · Beds · Baths tabulation)
const BEDS = { studio:0, '1br':1, '2br':2, '3br':3 }, BATHS = { studio:1, '1br':1, '2br':2, '3br':2 };
// a category is interactive only when visible AND unlocked
function pickable(k){ const L = S.layers[k]; return !!L && L.vis !== false && !L.lock; }
/* building model — each building carries its own appearance + void courtyards
   (tolerant of legacy raw point-array buildings from older saved files) */
const FLOOR_H = 11;                      // ft per floor (massing height = floors × this)
// parking demand by use — spaces required per 1,000 SF of gross floor area (editable assumptions)
const USE_PARK = { residential: 2.0, office: 3.3, retail: 4.0, hotel: 1.0, industrial: 0.6, datacenter: 0.3 };
const USE_LABEL = { residential: '住宅', office: '辦公', retail: '零售', hotel: '旅館', industrial: '工業/倉儲', datacenter: '資料中心' };
function bldgDefaults(){ return { color: '#64748b', opacity: 0.55, height: null, roof: true, voids: [], use: 'office', floors: 1 }; }
function bGFA(b){ return (PS.polyArea(b.poly) - (b.voids || []).reduce((t, v) => t + PS.polyArea(v), 0)) * Math.max(1, b.floors || 1); }   // gross floor area (ft²)
function bRequired(b){ return Math.ceil(bGFA(b) / 1000 * (USE_PARK[b.use] || 3)); }    // parking spaces this massing demands
function parkingDemand(){ return (S.buildings || []).reduce((s, b) => s + bRequired(b), 0); }   // total required across all massing
function readFin(){ return { landCost:+$('#fLand').value||0, hardCost:+$('#fHard').value||0, softPct:+$('#fSoft').value||0,
  rentMo:+$('#fRentMo').value||0, rentSfYr:+$('#fRentSf').value||0, opexPct:+$('#fOpex').value||0,
  rentGrowth:+$('#fGrowth').value||0, holdYears:+$('#fHold').value||5, exitCap:+$('#fExitCap').value||0 }; }
function massingFinancials(){   // pro-forma from the drawn massing (parking mode) — reuses the solver's computeFinancials
  const gfa = (S.buildings || []).reduce((s, b) => s + bGFA(b), 0);
  if (!(gfa > 0)) return null;
  const resiGFA = (S.buildings || []).filter(b => b.use === 'residential').reduce((s, b) => s + bGFA(b), 0);
  const residential = resiGFA > gfa * 0.5, eff = 0.82;
  return PS.computeFinancials(readFin(), { gfa, nrsf: gfa * eff, units: Math.round(resiGFA * eff / 850), residential });
}
function updateFinancials(){    // refresh the pro-forma readout (massing-driven in parking mode, site solve in site mode)
  const el = $('#finReadout'); if (!el) return;
  const fin = (S.mode === 'site') ? (S.site && S.site.fin) : massingFinancials();
  if (!fin || !(fin.totalCost > 0)) { el.textContent = S.mode === 'site' ? '按「自動配置建案」後顯示財務' : '畫建築量體＋設定參數後自動試算開發財務'; return; }
  el.innerHTML = `總開發成本 <b style="color:#e2e8f0">$${Math.round(fin.totalCost).toLocaleString()}</b>　·　年 NOI <b style="color:#e2e8f0">$${Math.round(fin.noi).toLocaleString()}</b><br>殖利率 Yield <b style="color:#4ade80">${fin.yieldOnCost.toFixed(1)}%</b>${fin.irr != null ? `　·　IRR <b style="color:#4ade80">${fin.irr.toFixed(1)}%</b>` : ''}`;
}
function makeBuilding(poly){ return Object.assign({ poly }, bldgDefaults()); }
function bPoly(b){ return Array.isArray(b) ? b : b.poly; }                 // footprint points
function bPolys(){ return S.buildings.map(bPoly); }                        // for the solver (needs raw polys)
function bHeight(b){ return (b && b.floors) ? b.floors * FLOOR_H : ((b && b.height != null && b.height > 0) ? b.height : S.params.height); }   // massing height from floor count
function normalizeBuildings(){
  S.buildings = (S.buildings || []).map(b => {
    if (Array.isArray(b)) return makeBuilding(b);
    const hadFloors = b.floors != null;
    const nb = Object.assign(bldgDefaults(), b);
    if (!hadFloors) nb.floors = Math.max(1, Math.round((nb.height || FLOOR_H) / FLOOR_H));   // old saves: derive floors from height
    return nb;
  });
}
function distSegPx(p, a, b) {                 // point→segment distance in screen px (edge picking)
  const dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
  let t = l2 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0; t = Math.max(0, Math.min(1, t));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}
// snap a world point onto the NEAREST boundary edge — gates always sit on the perimeter
function projectToBoundary(poly, pt) {
  let best = pt, bd = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    let t = l2 ? ((pt.x - a.x) * dx + (pt.y - a.y) * dy) / l2 : 0; t = Math.max(0, Math.min(1, t));
    const px = a.x + t * dx, py = a.y + t * dy, d = Math.hypot(pt.x - px, pt.y - py);
    if (d < bd) { bd = d; best = { x: px, y: py }; }
  }
  return best;
}

/* ----------------------------- units & regions --------------------------- */
// All geometry & S.params stay in FEET internally; this layer only converts
// what the user sees/types. round2 keeps inputs tidy.
const M_PER_FT = 0.3048, SQM_PER_SQFT = 0.09290304;
const round2 = n => Math.round(n * 100) / 100;
const U = {
  sys: 'imperial',
  metric() { return this.sys === 'metric'; },
  L(ft) { return this.metric() ? ft * M_PER_FT : ft; },        // ft -> display length
  Lr(v) { return this.metric() ? v / M_PER_FT : v; },          // display -> ft
  lu() { return this.metric() ? 'm' : 'ft'; },
  A(sf) { return this.metric() ? sf * SQM_PER_SQFT : sf; },    // sf -> display area
  Ar(v) { return this.metric() ? v / SQM_PER_SQFT : v; },      // display -> sf
  au() { return this.metric() ? 'm²' : 'SF'; },
  big(sf) { return this.metric() ? sf * SQM_PER_SQFT / 10000 : sf / 43560; }, // ha | acre
  bu() { return this.metric() ? 'ha' : 'ac'; },
};
// Typical, EDITABLE reference presets (NOT binding code — real zoning varies by city).
// All lengths in FEET (canonical). Metric regions just default the unit toggle to m.
const REGIONS = {
  us: { unit: 'imperial', stallW: 9.0, stallD: 18.0, aisle: 24.0, setback: 5,
        site: { floorH: 11, maxFAR: 2.0, maxHeight: 75, maxCov: 45, maxDUA: 40, sbF: 25, sbS: 10, sbR: 20, parkRatio: 1.5 } },
  tw: { unit: 'metric', stallW: 8.20, stallD: 19.69, aisle: 18.04, setback: 13.1,   // 2.5×6.0 m, 雙向車道 5.5 m（建技規則）
        site: { floorH: 11.48, maxFAR: 2.25, maxHeight: 164, maxCov: 50, maxDUA: 0, sbF: 19.7, sbS: 9.8, sbR: 13.1, parkRatio: 1.0 } },
  jp: { unit: 'metric', stallW: 8.20, stallD: 16.40, aisle: 18.04, setback: 3.3,    // 2.5×5.0 m, aisle 5.5 m
        site: { floorH: 10.5, maxFAR: 4.0, maxHeight: 148, maxCov: 60, maxDUA: 0, sbF: 6.6, sbS: 1.6, sbR: 1.6, parkRatio: 0.5 } },
  eu: { unit: 'metric', stallW: 8.20, stallD: 16.40, aisle: 19.69, setback: 9.8,    // 2.5×5.0 m, aisle 6 m
        site: { floorH: 11.5, maxFAR: 3.0, maxHeight: 98, maxCov: 50, maxDUA: 0, sbF: 16.4, sbS: 9.8, sbR: 16.4, parkRatio: 0.8 } },
};
const LEN_INPUTS = ['pW', 'pD', 'pA', 'pS', 'pH', 'pGreen', 'pMaxGap', 'cW', 'sFloorH', 'zHeight', 'zSbF', 'zSbS', 'zSbR'];
const AREA_INPUTS = ['uxStudioS', 'ux1S', 'ux2S', 'ux3S', 'pGFA'];
const LEN_LABELS = {
  pW: '車格寬 Stall W', pD: '車格深 Stall D', pA: '車道寬 Aisle', pS: '退縮 Setback', pH: '建築高度 Height',
  sFloorH: '樓層高 Floor-to-floor', zHeight: '最大高度 Height', bHeight: '建築高度 Height', cW: '🚗 Compact 車格寬',
};

function refreshUnitLabels() {
  const lu = U.lu(), au = U.au(), bu = U.bu();
  for (const id in LEN_LABELS) {
    const el = $('#' + id); if (!el) continue;
    const lab = el.closest('.field').querySelector('label');
    if (lab) lab.textContent = `${LEN_LABELS[id]} (${lu})`;
  }
  const sbLab = $('#zSbF') && $('#zSbF').closest('.field').querySelector('label');
  if (sbLab) sbLab.textContent = `退縮 前/側/後 (${lu})`;
  // metric tile labels (the .l sibling of each value)
  const setL = (vid, txt) => { const v = $('#' + vid); if (v && v.nextElementSibling) v.nextElementSibling.textContent = txt; };
  setL('mArea', `基地面積 (${bu})`);
  setL('mEff', `${au} / 車位`);
  setL('mRatio', U.metric() ? '車位比 /100 m²' : '車位比 /1000 SF');
  setL('sGFA', `總樓地板 GFA (${au})`);
  setL('sNRSF', `可租 NRSF (${au})`);
  // unit-mix header + parking GFA input label
  const uh = document.querySelector('#grpUnitMix .field span:last-child');
  if (uh) uh.textContent = `坪數 ${au}`;
  const gfaLab = $('#pGFA') && $('#pGFA').closest('.field').querySelector('label');
  if (gfaLab) gfaLab.textContent = `建築 GFA (${au})`;
}
$('#regionSel').addEventListener('change', () => applyRegion($('#regionSel').value));
document.querySelectorAll('#unitSeg button').forEach(b => b.onclick = () => setUnits(b.dataset.u));

function setUnitSystem(sys) {              // switch label/units only (no value convert)
  U.sys = sys;
  document.querySelectorAll('#unitSeg button').forEach(b => b.classList.toggle('active', b.dataset.u === sys));
  refreshUnitLabels();
}
function convertInputValues(toSys) {       // convert every length/area input in place
  if (U.sys === toSys) return;
  const lenFt = {}, areaSf = {};
  LEN_INPUTS.forEach(id => { const el = $('#' + id); if (el) lenFt[id] = U.Lr(+el.value); });
  AREA_INPUTS.forEach(id => { const el = $('#' + id); if (el) areaSf[id] = U.Ar(+el.value); });
  U.sys = toSys;
  LEN_INPUTS.forEach(id => { const el = $('#' + id); if (el) el.value = round2(U.L(lenFt[id])); });
  AREA_INPUTS.forEach(id => { const el = $('#' + id); if (el) el.value = Math.round(U.A(areaSf[id])); });
}
function setUnits(sys) {                    // ft / m toggle button
  if (U.sys === sys) return;
  convertInputValues(sys); setUnitSystem(sys);
  if (S.mode === 'site') { if (S.site) doSolveSite(); else updateSiteMetrics(); }
  else { if (S.solution) doSolve(); else updateMetrics(); }
  draw();
}
function applyRegion(r) {
  const R = REGIONS[r]; if (!R) return;
  convertInputValues(R.unit);              // convert non-overwritten fields (e.g. height)
  setUnitSystem(R.unit);
  $('#pW').value = round2(U.L(R.stallW)); $('#pD').value = round2(U.L(R.stallD));
  $('#pA').value = round2(U.L(R.aisle)); $('#pS').value = round2(U.L(R.setback));
  const s = R.site;
  $('#sFloorH').value = round2(U.L(s.floorH)); $('#zFAR').value = s.maxFAR;
  $('#zHeight').value = round2(U.L(s.maxHeight)); $('#zCov').value = s.maxCov; $('#zDUA').value = s.maxDUA;
  $('#zSbF').value = round2(U.L(s.sbF)); $('#zSbS').value = round2(U.L(s.sbS)); $('#zSbR').value = round2(U.L(s.sbR));
  $('#zPark').value = s.parkRatio;
  refreshUnitLabels();
  toast(`已套用 ${({ us: '美規', tw: '台灣', jp: '日本', eu: '歐洲' })[r]} 參考預設（可再手動調整）`);
  if (S.mode === 'site') { if (S.boundary.length >= 3) doSolveSite(); else updateSiteMetrics(); }
  else if (S.solution) doSolve();
}

/* --------------------------- canvas sizing ------------------------------- */
function resize() {
  const r = cv.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  cv.width = Math.round(r.width * dpr);
  cv.height = Math.round(r.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  cv._w = r.width; cv._h = r.height;
  if (S.map && S.mapMode) S.map.invalidateSize({ animate: false });   // keep tiles filling the pane
  draw();
}
window.addEventListener('resize', resize);

/* --------------------------- view transform ------------------------------ */
// In map mode, world coordinates (feet) project through Leaflet so drawings
// stay locked to the real-world imagery; otherwise the plain canvas transform.
const FT_PER_M = 3.280839895;
// OSM CONTEXT: pull surrounding building footprints from OpenStreetMap (free Overpass API) → existing 3D context.
async function fetchContextBuildings() {
  if (!S.mapMode || !S.map) { toast('請先開啟「地圖」底圖，再載入周邊建物'); return; }
  const bd = S.map.getBounds(), q = `[out:json][timeout:20];(way["building"](${bd.getSouth()},${bd.getWest()},${bd.getNorth()},${bd.getEast()}););out body;>;out skel qt;`;
  toast('載入周邊建物中…（OSM）');
  try {
    const r = await fetch('https://overpass-api.de/api/interpreter', { method: 'POST', body: 'data=' + encodeURIComponent(q) });
    const j = await r.json(), nodes = {};
    for (const el of j.elements) if (el.type === 'node') nodes[el.id] = { lat: el.lat, lng: el.lon };
    const bldgs = [];
    for (const el of j.elements) if (el.type === 'way' && el.nodes && el.tags && el.tags.building) {
      const poly = el.nodes.map(id => nodes[id]).filter(Boolean).map(ll => latLngToFeet(ll));
      if (poly.length >= 3) bldgs.push({ poly, height: (parseFloat(el.tags['building:levels']) || 2) * 11 });
    }
    S.contextBuildings = bldgs; draw();
    toast(bldgs.length ? `已載入 ${bldgs.length} 棟周邊既有建物 — 切「3D 量體」看 context` : '這個範圍 OSM 沒有建物資料');
  } catch (e) { toast('周邊建物載入失敗（需網路；OSM 服務忙碌時稍後再試）'); }
}
function latLngToFeet(ll) {
  const mLat = 111320, mLng = 111320 * Math.cos(S.geo.lat0 * Math.PI / 180);
  return { x: (ll.lng - S.geo.lng0) * mLng * FT_PER_M, y: -(ll.lat - S.geo.lat0) * mLat * FT_PER_M };
}
function feetToLatLng(p) {
  const mLat = 111320, mLng = 111320 * Math.cos(S.geo.lat0 * Math.PI / 180);
  return { lat: S.geo.lat0 - (p.y / FT_PER_M) / mLat, lng: S.geo.lng0 + (p.x / FT_PER_M) / mLng };
}
function toScreen(p) {
  if (S.mapMode && S.map) {
    const ll = feetToLatLng(p);
    const pt = S.map.latLngToContainerPoint([ll.lat, ll.lng]);
    return { x: pt.x, y: pt.y };
  }
  return { x: p.x * S.view.scale + S.view.ox, y: p.y * S.view.scale + S.view.oy };
}
function toWorld(s) {
  if (S.mapMode && S.map) return latLngToFeet(S.map.containerPointToLatLng([s.x, s.y]));
  return { x: (s.x - S.view.ox) / S.view.scale, y: (s.y - S.view.oy) / S.view.scale };
}

function fitView() {
  if (!cv._w || !cv._h) return;                     // container not laid out yet
  const all = [].concat(S.boundary, ...bPolys(), ...S.obstacles);   // buildings are objects → use their polys
  if (!all.length) { S.view = { scale: 1, ox: cv._w / 2, oy: cv._h / 2 }; draw(); return; }
  const bb = PS.bbox(all);
  const pad = 60;
  const w = Math.max(bb.maxX - bb.minX, 1), h = Math.max(bb.maxY - bb.minY, 1);
  const sx = (cv._w - pad * 2) / w, sy = (cv._h - pad * 2) / h;
  const scale = Math.min(sx, sy);
  S.view.scale = scale;
  S.view.ox = pad + (cv._w - pad * 2 - w * scale) / 2 - bb.minX * scale;
  S.view.oy = pad + (cv._h - pad * 2 - h * scale) / 2 - bb.minY * scale;
  draw();
}

/* ------------------------------ rendering -------------------------------- */
function draw() {
  if (S.is3d) return draw3D();
  ctx.clearRect(0, 0, cv._w, cv._h);
  if (!S.mapMode) drawGrid();

  // inactive sub-parcels (subdivision) — faint outlines + labels
  if (S.parcels && S.parcels.length > 1) {
    S.parcels.forEach((pc, i) => {
      if (i === S.activeParcel || pc.length < 3) return;
      pathPoly(pc, true);
      ctx.fillStyle = 'rgba(100,116,139,.10)'; ctx.fill();
      ctx.save(); ctx.setLineDash([6, 5]); ctx.lineWidth = 1.3; ctx.strokeStyle = 'rgba(148,163,184,.6)'; ctx.stroke(); ctx.restore();
      labelPoly(pc, '子地 ' + String.fromCharCode(65 + i), 'rgba(148,163,184,.8)');
    });
  }
  // cut-line preview (subdivide tool, first point placed)
  if (S.tool === 'subdivide' && S.splitPt && S.hoverWorld) {
    const a = toScreen(S.splitPt), b = toScreen(S.hoverWorld);
    ctx.save(); ctx.setLineDash([8, 5]); ctx.lineWidth = 2; ctx.strokeStyle = '#f59e0b';
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke(); ctx.restore();
  }

  // site fill + outline
  if (S.boundary.length >= 2) {
    pathPoly(S.boundary, true);
    if (S.boundary.length >= 3) { ctx.fillStyle = 'rgba(56,189,248,.05)'; ctx.fill(); }
    // setback line (visual inset hint) — draw dashed inside outline
    drawSetback();
    ctx.lineWidth = 2; ctx.strokeStyle = '#38bdf8'; ctx.stroke();
  }

  // buildable envelope (site mode) — dashed developable-area guide
  if (S.mode === 'site' && S.site && S.site.envelope && S.site.envelope.length >= 3) {
    ctx.save(); ctx.setLineDash([7, 5]); ctx.lineWidth = 1.5; ctx.strokeStyle = 'rgba(74,222,128,.8)';
    pathPoly(S.site.envelope, true); ctx.stroke(); ctx.restore();
  }

  // parking (standalone in parking mode, or the site's required parking field)
  const park = S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution;
  if (park && S.layers.parking.vis) {
    ctx.save();
    if (S.boundary.length >= 3) { pathPoly(S.boundary, true); ctx.clip(); }
    // DRIVE NETWORK — a CONTINUOUS, unbroken road surface with smooth rounded junctions:
    // (1) FILL every lane rect — overlapping aisle/connector rects tile into ONE solid asphalt sheet, so the
    //     roads never break where segments meet (fixes "用線段聯繫會斷開道路"). (2) round-cap STROKE each
    //     centre-line on top — the round caps fillet the junction corners into smooth curb-returns. Opaque
    //     colours → the fill and the rounding stroke never compound into uneven patches.
    ctx.save(); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    const _drvSc = (() => { const a = toScreen({ x: 0, y: 0 }), b = toScreen({ x: 1, y: 0 }); return Math.hypot(b.x - a.x, b.y - a.y); })();
    const _trace = line => { ctx.beginPath(); line.forEach((p, i) => { const s = toScreen(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); }); };
    const ASPH = '#566178', RAMP = '#4a7058';                         // entrance ramp keeps a faint green tint
    for (const a of park.aisles) { ctx.fillStyle = ASPH; pathPoly(a.poly, true); ctx.fill(); }
    for (const lane of (park.connectors || [])) { const poly = lane.poly || lane; ctx.fillStyle = lane.type ? RAMP : ASPH; pathPoly(poly, true); ctx.fill(); }
    if (park.spines) for (const sp of park.spines) {
      const ramp = sp.kind === 'conn' && park.connectors[sp.src] && park.connectors[sp.src].type;
      ctx.strokeStyle = ramp ? RAMP : ASPH; ctx.lineWidth = Math.max((sp.width || 18) * _drvSc, 2);
      _trace(sp.line); ctx.stroke();
    }
    ctx.restore();
    if (S.selAisle != null && park.aisles[S.selAisle] && S.tool === 'select') {     // highlight the selected drive aisle
      pathPoly(park.aisles[S.selAisle].poly, true);
      ctx.fillStyle = 'rgba(56,189,248,.32)'; ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = '#38bdf8'; ctx.stroke();
    }
    drawAisleArrows(park);
    ctx.restore();
    for (const s of park.stalls) drawStall(s);
    if (S.tool === 'select' && !S.is3d && park.spines) {     // editable aisle SPINES — drag an end node to reshape the lane; stalls re-attach
      ctx.save();
      for (let si = 0; si < park.spines.length; si++) {
        const sp = park.spines[si], pts = sp.line.map(toScreen), sel = S.selAisle === si, conn = sp.kind === 'conn';
        const col = conn ? '251,146,60' : '56,189,248';                   // connectors orange, drive aisles blue
        ctx.setLineDash([5, 4]); ctx.lineWidth = sel ? 2 : 1.1; ctx.strokeStyle = `rgba(${col},${sel ? .95 : .42})`;
        ctx.beginPath(); ctx.moveTo(pts[0].x, pts[0].y); for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y); ctx.stroke(); ctx.setLineDash([]);
        for (let i = 0; i < pts.length; i++) { const mid = i > 0 && i < pts.length - 1; ctx.beginPath(); ctx.arc(pts[i].x, pts[i].y, sel ? 6 : 4.5, 0, 6.2832); ctx.fillStyle = mid ? '#fbbf24' : `rgb(${col})`; ctx.fill(); ctx.lineWidth = 1.3; ctx.strokeStyle = '#0f172a'; ctx.stroke(); }   // bend nodes yellow
      }
      ctx.restore();
    }
    // structured garage: the express RAMP + the structural COLUMN grid on the typical deck
    if (S.mode === 'site' && S.site && (S.site.structured || S.site.isWrap) && S.site.garage) {
      const g = S.site.garage;
      if (g.ramp) {
        pathPoly(g.ramp, true);
        ctx.fillStyle = 'rgba(245,158,11,.5)'; ctx.fill();
        ctx.lineWidth = 1.5; ctx.strokeStyle = 'rgba(217,119,6,.95)'; ctx.stroke();
        const cx = g.ramp.reduce((s, p) => s + p.x, 0) / g.ramp.length, cy = g.ramp.reduce((s, p) => s + p.y, 0) / g.ramp.length;
        const sc = toScreen({ x: cx, y: cy });
        ctx.fillStyle = 'rgba(120,53,15,.95)'; ctx.font = 'bold 11px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText('坡道', sc.x, sc.y);
      }
      if (g.columns) { const r = Math.max(2, 1.3 * S.view.scale);
        for (const c of g.columns) { const s = toScreen(c);
          ctx.fillStyle = 'rgba(30,41,59,.92)'; ctx.fillRect(s.x - r, s.y - r, r * 2, r * 2);
          ctx.strokeStyle = 'rgba(148,163,184,.85)'; ctx.lineWidth = 1; ctx.strokeRect(s.x - r, s.y - r, r * 2, r * 2);
        }
      }
    }
    // ADA access aisles (striped diagonal hatch — the 5ft no-parking access zone)
    if (park.accessAisles) for (const a of park.accessAisles) {
      pathPoly(a.poly, true);
      ctx.fillStyle = 'rgba(29,78,216,.18)'; ctx.fill();
      ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(59,130,246,.6)'; ctx.stroke();
      ctx.save(); pathPoly(a.poly, true); ctx.clip();
      ctx.strokeStyle = 'rgba(59,130,246,.7)'; ctx.lineWidth = 1.2;
      const bb = PS.bbox(a.poly); const s0 = toScreen({ x: bb.minX, y: bb.minY }), s1 = toScreen({ x: bb.maxX, y: bb.maxY });
      for (let x = s0.x - (s1.y - s0.y); x < s1.x; x += 5) { ctx.beginPath(); ctx.moveTo(x, s0.y); ctx.lineTo(x + (s1.y - s0.y), s1.y); ctx.stroke(); }
      ctx.restore();
    }
  }

  // building massing (site mode). TOWNHOME SUBDIVISION draws individual lots + access drives;
  // every other use draws the single massing block.
  if (S.mode === 'site' && S.site && S.layers.building.vis) {
    const sub = S.site.subdivision;
    if (sub && sub.units && sub.units.length) {
      (sub.drives || []).forEach(d => { pathPoly(d, true); ctx.fillStyle = 'rgba(148,163,184,.30)'; ctx.fill(); });   // access drives
      const detached = sub.subType && sub.subType !== 'townhome';
      if (detached) sub.units.forEach(lot => { pathPoly(lot, true); ctx.fillStyle = 'rgba(56,189,248,.10)'; ctx.fill(); ctx.lineWidth = 0.8; ctx.strokeStyle = 'rgba(14,165,233,.55)'; ctx.stroke(); });  // lot lines
      (sub.houses || sub.units).forEach(h => { pathPoly(h, true); ctx.fillStyle = 'rgba(56,189,248,.55)'; ctx.fill(); ctx.lineWidth = 1; ctx.strokeStyle = '#0ea5e9'; ctx.stroke(); });   // the houses
      const tlabel = { townhome: '戶連棟', detached: '戶獨棟', cottage: '戶小宅' }[sub.subType] || '戶';
      labelPoly(S.site.footprint && S.site.footprint.length >= 3 ? S.site.footprint : S.boundary, `${sub.count} ${tlabel} · ${S.site.floors}F`, '#e2e8f0');
    } else if (S.site.garden && S.site.garden.bars.length) {
      // GARDEN walk-up: separate low-rise bar buildings, surface parking already drawn in the bands between them
      S.site.garden.bars.forEach(b => { pathPoly(b, true); ctx.fillStyle = 'rgba(56,189,248,.5)'; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = '#0ea5e9'; ctx.stroke(); });
      labelPoly(S.site.footprint && S.site.footprint.length >= 3 ? S.site.footprint : S.boundary, `${S.site.garden.rows} 棟花園公寓 · ${S.site.units} 戶 · ${S.site.floors}F`, '#e2e8f0');
    } else if (S.site.tower && S.site.tower.podium) {
      // TOWER: wide parking podium base (dashed) with the slender point-tower plate on top
      const tw = S.site.tower;
      ctx.save(); ctx.setLineDash([6, 4]); pathPoly(tw.podium, true); ctx.fillStyle = 'rgba(56,189,248,.14)'; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = 'rgba(56,189,248,.7)'; ctx.stroke(); ctx.restore();
      fillVoids(tw.plate, S.site.footVoids, 'rgba(37,99,235,.62)', '#1d4ed8', 2);
      hatch(tw.plate, '#bfdbfe', .3);
      drawUnitPlan();
      labelPoly(tw.plate, `塔樓 ${S.site.units} 戶 · ${S.site.floors}F · 基座${tw.podiumLevels}層車庫`, '#e2e8f0');
    } else if (S.site.isWrap && S.site.wrapCore && S.site.footprint && S.site.footprint.length >= 3) {
      // WRAP: residential RING = footprint with the parking core cut out (the garage deck shows in the hole)
      ctx.beginPath();
      S.site.footprint.forEach((p, i) => { const s = toScreen(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); }); ctx.closePath();
      S.site.wrapCore.forEach((p, i) => { const s = toScreen(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); }); ctx.closePath();
      ctx.fillStyle = 'rgba(56,189,248,.42)'; ctx.fill('evenodd');
      ctx.lineWidth = 2; ctx.strokeStyle = '#38bdf8'; pathPoly(S.site.footprint, true); ctx.stroke(); pathPoly(S.site.wrapCore, true); ctx.stroke();
      drawUnitPlan();
      labelPoly(S.site.footprint, `環繞 · ${S.site.units} 戶 · ${S.site.floors}F`, '#e2e8f0');
    } else if (S.site.industrial) {
      // WAREHOUSE: paved truck courts, 53' trailer stalls, the clear-span box, dock-door teeth
      const ind = S.site.industrial;
      ind.truckCourts.forEach(c => { pathPoly(c, true); ctx.fillStyle = 'rgba(100,116,139,.16)'; ctx.fill(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(100,116,139,.45)'; ctx.stroke(); });
      ind.trailerStalls.forEach(t => { pathPoly(t, true); ctx.fillStyle = 'rgba(234,179,8,.16)'; ctx.fill(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(202,138,4,.8)'; ctx.stroke(); });
      fillVoids(S.site.footprint, S.site.footVoids, 'rgba(71,85,105,.82)', '#1e293b', 2);
      ctx.fillStyle = '#0f172a'; ind.dockDoors.forEach(d => { pathPoly(d, true); ctx.fill(); });
      labelPoly(S.site.footprint, `倉儲 ${ind.dockType === 'cross' ? '雙面對流' : '單面'}卸貨 · ${ind.dockCount} 門 · ${Math.round(S.site.gfa).toLocaleString()} SF`, '#e2e8f0');
    } else if (S.site.retail) {
      // RETAIL: anchor / inline-shop strip (the footprint) + pad outparcels near the street; the big lot is drawn as parking
      const rt = S.site.retail;
      fillVoids(rt.anchor, S.site.footVoids, 'rgba(37,99,235,.55)', '#1d4ed8', 2); hatch(rt.anchor, '#bfdbfe', .3);
      rt.pads.forEach(p => { pathPoly(p, true); ctx.fillStyle = 'rgba(99,102,241,.6)'; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = '#4338ca'; ctx.stroke(); });
      labelPoly(rt.anchor, `零售中心 · ${rt.pads.length} pad 外帶店 · GLA ${Math.round(S.site.gfa).toLocaleString()} SF`, '#e2e8f0');
    } else if (S.site.datacenter) {
      // DATA CENTRE: data-hall box + fenced mechanical/generator yard + substation pad
      const dc = S.site.datacenter;
      pathPoly(dc.mechYard, true); ctx.fillStyle = 'rgba(245,158,11,.14)'; ctx.fill(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(217,119,6,.7)'; ctx.stroke(); hatch(dc.mechYard, '#f59e0b', .5);
      pathPoly(dc.subStation, true); ctx.fillStyle = 'rgba(220,38,38,.22)'; ctx.fill(); ctx.lineWidth = 1.2; ctx.strokeStyle = '#dc2626'; ctx.stroke(); hatch(dc.subStation, '#ef4444', .4);
      fillVoids(dc.hall, S.site.footVoids, 'rgba(51,65,85,.85)', '#0f172a', 2);
      labelPoly(dc.hall, `資料機房 · ${S.site.floors}F · ${Math.round(S.site.gfa).toLocaleString()} SF`, '#e2e8f0');
      const c = PS.centroid(dc.mechYard), sc = toScreen(c); ctx.fillStyle = 'rgba(120,53,15,.95)'; ctx.font = 'bold 10px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText('機電中庭', sc.x, sc.y);
    } else if (S.site.footprint && S.site.footprint.length >= 3) {
      fillVoids(S.site.footprint, S.site.footVoids, 'rgba(56,189,248,.42)', '#38bdf8', 2);
      hatch(S.site.footprint, '#bae6fd', .3);
      drawUnitPlan();
      const lbl = S.site.hotel ? `旅館 ${S.site.keys} 房 · ${S.site.floors}F`
        : (S.site.unitPlan ? `${S.site.units} 戶 · ${S.site.floors}F · ${Math.round(S.site.gfa).toLocaleString()} SF`
        : `${S.site.floors}F · ${Math.round(S.site.gfa).toLocaleString()} SF`);
      labelPoly(S.site.footprint, lbl, '#e2e8f0');
    }
    // SERVICE CORES (stairs + elevators): user-placed cores override the auto-placed one when present
    ((S.manualCores && S.manualCores.length) ? S.manualCores : (S.site.cores || [])).forEach(core => {
      pathPoly(core, true); ctx.fillStyle = 'rgba(15,23,42,.6)'; ctx.fill(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(226,232,240,.7)'; ctx.stroke();
      const cc = PS.centroid(core), sc = toScreen(cc);
      ctx.fillStyle = 'rgba(241,245,249,.95)'; ctx.font = 'bold 9px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText('核心', sc.x, sc.y);
    });
  }

  // buildings — each with its own colour / opacity, void courtyards punched out
  for (const b of (S.layers.building.vis ? S.buildings : [])) {
    const fp = b.poly, voids = b.voids || [], col = b.color || '#64748b', sel = S.selBuilding === b;
    ctx.beginPath(); subPath(fp); voids.forEach(subPath);              // outer + holes
    ctx.fillStyle = hexA(col, b.opacity != null ? b.opacity : 0.55); ctx.fill('evenodd');
    pathPoly(fp, true);
    ctx.lineWidth = sel ? 2.5 : 1.5; ctx.strokeStyle = sel ? '#fff' : shade(col, 0.3); ctx.stroke();
    if (voids.length) {                                                // dashed courtyard outlines
      ctx.save(); ctx.setLineDash([4, 3]); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(226,232,240,.6)';
      voids.forEach(v => { pathPoly(v, true); ctx.stroke(); }); ctx.restore();
    }
    hatch(fp, shade(col, 0.4), .2);
    const gfaTxt = U.metric() ? `${Math.round(U.A(bGFA(b))).toLocaleString()} m²` : `${Math.round(bGFA(b)).toLocaleString()} ft²`;
    labelPoly(fp, `${USE_LABEL[b.use] || 'BUILDING'} · ${b.floors || 1}F · ${gfaTxt}`, '#e2e8f0');
  }
  // footprint corner handles in select mode — drag a corner to reshape the massing (parking re-packs on release)
  if (S.tool === 'select' && !S.is3d && S.layers.building.vis) {
    for (const b of S.buildings) {
      const sel = S.selBuilding === b;
      for (const p of b.poly) {
        const s = toScreen(p);
        ctx.beginPath(); ctx.arc(s.x, s.y, sel ? 5.5 : 4, 0, 6.2832);
        ctx.fillStyle = sel ? '#fff' : 'rgba(226,232,240,.85)'; ctx.fill();
        ctx.lineWidth = 1.4; ctx.strokeStyle = '#0f172a'; ctx.stroke();
      }
    }
  }
  // parking zones (user-drawn) — stalls pack only inside; show as a dashed outline so the stalls read through
  for (const z of (S.parkZones || [])) {
    pathPoly(z, true);
    ctx.save(); ctx.setLineDash([8, 5]); ctx.lineWidth = 2; ctx.strokeStyle = 'rgba(34,197,94,.85)'; ctx.stroke(); ctx.restore();
    ctx.fillStyle = 'rgba(34,197,94,.06)'; ctx.fill();
  }
  // internal roads (user-drawn) — drawn as a STROKED centre-line with round joins/caps so bends are smooth
  // and continuous (curb edge under + asphalt + dashed centre stripe). Looks like a real road, not box segments.
  drawRoads();
  // obstacles
  for (const o of (S.layers.obstacle.vis ? S.obstacles : [])) {
    pathPoly(o, true);
    ctx.fillStyle = 'rgba(127,29,29,.4)'; ctx.fill();
    ctx.lineWidth = 1.5; ctx.strokeStyle = '#ef4444'; ctx.stroke();
    hatch(o, '#ef4444', .35);
  }
  if (S.tool === 'select' && !S.is3d && S.layers.obstacle.vis) {   // obstacle corner handles (drag corner / edge / whole to adjust)
    for (let oi = 0; oi < (S.obstacles || []).length; oi++) {
      const sel = S.selObstacle === oi;
      if (sel) { pathPoly(S.obstacles[oi], true); ctx.lineWidth = 2; ctx.strokeStyle = '#f87171'; ctx.stroke(); }
      for (const p of S.obstacles[oi]) { const s = toScreen(p); ctx.beginPath(); ctx.arc(s.x, s.y, sel ? 5.5 : 4, 0, 6.2832); ctx.fillStyle = '#ef4444'; ctx.fill(); ctx.lineWidth = 1.3; ctx.strokeStyle = '#0f172a'; ctx.stroke(); }
    }
  }
  // landscape trees (over parking/buildings, under markers)
  drawFlowOverlay();
  drawEarthwork();
  drawTrees();
  // entrances
  if (S.layers.entrance.vis) for (const e of S.entrances) drawEntrance(e);

  // boundary vertices (edit handles in select mode; hidden when site layer locked)
  if (S.tool === 'select' && S.boundary.length && !S.layers.site.lock) {
    for (const v of S.boundary) {
      const p = toScreen(v);
      ctx.beginPath(); ctx.arc(p.x, p.y, 5, 0, 7); ctx.fillStyle = '#38bdf8';
      ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = '#001018'; ctx.stroke();
    }
  }

  // draft in progress
  if (S.draft && S.draft.length) drawDraft();

  // selected stall highlight
  if (S.selStall) {
    pathPoly(S.selStall.poly, true);
    ctx.lineWidth = 2.5; ctx.strokeStyle = '#fff'; ctx.stroke();
  }

  drawEdgeSetbacks();
  drawCompass();
  drawMeasures();

  // entrance type popup follows the selected entrance
  if (S.selEntrance && S.entrances.indexOf(S.selEntrance) >= 0) showEntPopup(S.selEntrance);
  else hideEntPopup();
}

function pathPoly(poly, close) {
  ctx.beginPath();
  poly.forEach((p, i) => { const s = toScreen(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
  if (close) ctx.closePath();
}
function subPath(poly) {                    // add a closed sub-path to the CURRENT path (for even-odd holes)
  poly.forEach((p, i) => { const s = toScreen(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
  ctx.closePath();
}
// buffer a centre-line polyline into a chain of road strips (one rectangle per segment, ends extended to join)
function bufferPolyline(pts, width) {
  const h = width / 2, rects = [];
  for (let i = 0; i + 1 < pts.length; i++) {
    const a = pts[i], b = pts[i + 1], dx = b.x - a.x, dy = b.y - a.y, len = Math.hypot(dx, dy) || 1;
    const nx = -dy / len * h, ny = dx / len * h, ex = dx / len * h, ey = dy / len * h;   // normal + end-cap extension
    rects.push([{ x: a.x - ex + nx, y: a.y - ey + ny }, { x: b.x + ex + nx, y: b.y + ey + ny }, { x: b.x + ex - nx, y: b.y + ey - ny }, { x: a.x - ex - nx, y: a.y - ey - ny }]);
  }
  return rects;
}
// ---- road editing: roads are stored as editable centre-lines (S.roadLines); the solver gets strips (S.roads) ----
function rebuildRoadStrips() {   // re-derive solver strips from the centre-lines — call after any road edit
  S.roads = (S.roadLines || []).flatMap(rl => bufferPolyline(rl.line, rl.width || 24));
}
function roadVertexAt(w) {       // pick a road centre-line vertex near w (screen-space). Returns {ri,pi} or null
  const m = toScreen(w);
  for (let ri = 0; ri < (S.roadLines || []).length; ri++) {
    const ln = S.roadLines[ri].line;
    for (let pi = 0; pi < ln.length; pi++) { const s = toScreen(ln[pi]); if (Math.hypot(s.x - m.x, s.y - m.y) < 14) return { ri, pi }; }
  }
  return null;
}
function bldgVertexAt(w) {       // pick a building footprint corner near w (screen-space). Returns {bi,pi} or null
  if (!S.layers.building.vis || S.layers.building.lock) return null;
  const m = toScreen(w);
  for (let bi = 0; bi < (S.buildings || []).length; bi++) {
    const poly = S.buildings[bi].poly;
    for (let pi = 0; pi < poly.length; pi++) { const s = toScreen(poly[pi]); if (Math.hypot(s.x - m.x, s.y - m.y) < 14) return { bi, pi }; }
  }
  return null;
}
function roadBodyAt(w) {         // pick a road body (a segment, away from a vertex) near w. Returns {ri} or null
  const m = toScreen(w), o = toScreen({ x: 0, y: 0 }), u = toScreen({ x: 1, y: 0 }), sc = Math.hypot(u.x - o.x, u.y - o.y);
  for (let ri = 0; ri < (S.roadLines || []).length; ri++) {
    const ln = S.roadLines[ri].line, halfPx = (S.roadLines[ri].width || 24) / 2 * sc + 3;
    for (let i = 0; i + 1 < ln.length; i++) if (distSegPx(m, toScreen(ln[i]), toScreen(ln[i + 1])) <= halfPx) return { ri };
  }
  return null;
}
// ---- manual drive-aisle editing (TestFit-style "manual mode"): select an auto aisle → remove / single-load / drag ----
function activePark() { return S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution; }
function aisleAt(w) {             // index of the drive aisle under world point w (active solution), or -1
  const park = activePark(); if (!park || !park.aisles || !S.layers.parking.vis || S.layers.parking.lock) return -1;
  for (let i = 0; i < park.aisles.length; i++) if (park.aisles[i].poly && PS.pointInPoly(w, park.aisles[i].poly)) return i;
  return -1;
}
function aisleSpine(poly) {       // derive a drive aisle's editable CENTRELINE [node0,node1] + width from its rect
  const c = PS.centroid(poly); let dir = { x: 1, y: 0 }, best = -1;
  for (let i = 0; i < poly.length; i++) { const a = poly[i], b = poly[(i + 1) % poly.length], L = Math.hypot(b.x - a.x, b.y - a.y); if (L > best) { best = L; dir = { x: (b.x - a.x) / L, y: (b.y - a.y) / L }; } }
  const per = { x: -dir.y, y: dir.x }; let lo = 1e9, hi = -1e9, plo = 1e9, phi = -1e9;
  for (const p of poly) { const t = (p.x - c.x) * dir.x + (p.y - c.y) * dir.y, q = (p.x - c.x) * per.x + (p.y - c.y) * per.y; lo = Math.min(lo, t); hi = Math.max(hi, t); plo = Math.min(plo, q); phi = Math.max(phi, q); }
  return { line: [{ x: c.x + dir.x * lo, y: c.y + dir.y * lo }, { x: c.x + dir.x * hi, y: c.y + dir.y * hi }], width: Math.max(phi - plo, 8) };
}
function deriveSpines(park) {     // after a pack: every drive aisle + connector becomes an editable spine; tag stalls with their aisle spine
  if (!park || !park.aisles) return;
  const prevAisle = (park.spines || []).filter(s => s.kind === 'aisle');   // keep any hand-bent aisle centre-lines across a re-derive (index-aligned: aisles aren't re-ordered by the light reconnect)
  park.spines = park.aisles.map((a, i) => {                                  // aisle spines FIRST (index = aisle index)
    const old = prevAisle[i];
    if (old && old.line && old.line.length >= 2) return { line: old.line, width: old.width, sides: old.sides, kind: 'aisle', src: i };   // preserve manual bend
    return Object.assign(aisleSpine(a.poly), { kind: 'aisle', src: i });
  });
  (park.connectors || []).forEach((c, i) => { const poly = c.poly || c; if (poly && poly.length >= 3) park.spines.push(Object.assign(aisleSpine(poly), { kind: 'conn', src: i })); });
  for (const s of park.stalls) { s.spine = -1; for (let i = 0; i < park.aisles.length; i++) if (s.aprobe && PS.pointInPoly(s.aprobe, park.aisles[i].poly)) { s.spine = i; break; } }
  if (S.aisleEdits) for (let i = 0; i < park.aisles.length; i++)   // honour single-load edits so a later spine drag keeps one side
    { const _ae = S.aisleEdits[aisleKey(park.aisles[i].poly)]; if ((_ae === 'single' || _ae === 'single2') && park.spines[i]) park.spines[i].sides = spineKeptSide(park.spines[i], park, i); }
}
function spineNodeAt(w) {         // pick a draggable spine end-node near w → {si, ni} or null
  const park = activePark(); if (!park || !park.spines || !S.layers.parking.vis || S.layers.parking.lock) return null;
  const m = toScreen(w);
  for (let si = 0; si < park.spines.length; si++) { const ln = park.spines[si].line; for (let ni = 0; ni < ln.length; ni++) { const s = toScreen(ln[ni]); if (Math.hypot(s.x - m.x, s.y - m.y) < 14) return { si, ni }; } }
  return null;
}
function spineBodyAt(w) {         // pick a drive-aisle LINE (not an end node) → si, for moving the whole lane
  const park = activePark(); if (!park || !park.spines || !S.layers.parking.vis || S.layers.parking.lock) return -1;
  const m = toScreen(w);
  for (let si = 0; si < park.spines.length; si++) {
    const ln = park.spines[si].line;
    if (ln.some(n => { const s = toScreen(n); return Math.hypot(s.x - m.x, s.y - m.y) < 16; })) continue;   // near an end → that's the node grab
    for (let i = 0; i + 1 < ln.length; i++) if (distSegPx(m, toScreen(ln[i]), toScreen(ln[i + 1])) < 8) return si;
  }
  return -1;
}
function boundaryEdgeAt(w) {      // pick a boundary EDGE (not a corner) → edge index i, for stretching the site
  if (!pickable('site') || S.boundary.length < 2) return -1;
  const m = toScreen(w);
  if (S.boundary.some(v => { const s = toScreen(v); return Math.hypot(s.x - m.x, s.y - m.y) < 16; })) return -1;   // near a corner → vertex grab
  for (let i = 0; i < S.boundary.length; i++) { const A = toScreen(S.boundary[i]), B = toScreen(S.boundary[(i + 1) % S.boundary.length]); if (distSegPx(m, A, B) < 8) return i; }
  return -1;
}
function bldgEdgeAt(w) {          // pick a building EDGE (not a corner) → {bi, ei}
  if (!S.layers.building.vis || S.layers.building.lock) return null;
  const m = toScreen(w);
  for (let bi = 0; bi < S.buildings.length; bi++) {
    const poly = S.buildings[bi].poly;
    if (poly.some(v => { const s = toScreen(v); return Math.hypot(s.x - m.x, s.y - m.y) < 16; })) continue;   // near a corner → corner grab
    for (let ei = 0; ei < poly.length; ei++) { const A = toScreen(poly[ei]), B = toScreen(poly[(ei + 1) % poly.length]); if (distSegPx(m, A, B) < 8) return { bi, ei }; }
  }
  return null;
}
function obstacleVertexAt(w) {    // pick an obstacle corner near w → {oi, pi}
  if (!S.layers.obstacle.vis || S.layers.obstacle.lock) return null;
  const m = toScreen(w);
  for (let oi = 0; oi < (S.obstacles || []).length; oi++) { const poly = S.obstacles[oi]; for (let pi = 0; pi < poly.length; pi++) { const s = toScreen(poly[pi]); if (Math.hypot(s.x - m.x, s.y - m.y) < 14) return { oi, pi }; } }
  return null;
}
function obstacleEdgeAt(w) {      // pick an obstacle EDGE (not a corner) → {oi, ei}
  if (!S.layers.obstacle.vis || S.layers.obstacle.lock) return null;
  const m = toScreen(w);
  for (let oi = 0; oi < (S.obstacles || []).length; oi++) {
    const poly = S.obstacles[oi];
    if (poly.some(v => { const s = toScreen(v); return Math.hypot(s.x - m.x, s.y - m.y) < 16; })) continue;
    for (let ei = 0; ei < poly.length; ei++) { const A = toScreen(poly[ei]), B = toScreen(poly[(ei + 1) % poly.length]); if (distSegPx(m, A, B) < 8) return { oi, ei }; }
  }
  return null;
}
function spineToRect(line, W) {   // build the grey lane polygon from its centre-line + width — a rect for a straight line, a bent strip for a polyline
  const h = W / 2;
  if (line.length <= 2) {         // straight (fast path, identical to before)
    const a = line[0], b = line[line.length - 1], L = Math.hypot(b.x - a.x, b.y - a.y) || 1;
    const dir = { x: (b.x - a.x) / L, y: (b.y - a.y) / L }, per = { x: -dir.y, y: dir.x };
    return [{ x: a.x + per.x * h, y: a.y + per.y * h }, { x: b.x + per.x * h, y: b.y + per.y * h }, { x: b.x - per.x * h, y: b.y - per.y * h }, { x: a.x - per.x * h, y: a.y - per.y * h }];
  }
  const segN = (a, b) => { const L = Math.hypot(b.x - a.x, b.y - a.y) || 1; return { x: -(b.y - a.y) / L, y: (b.x - a.x) / L }; };
  const offset = i => {           // miter offset at node i: bisector of the adjacent segment normals, lengthened by 1/cos(half-angle) so the strip keeps width W through a bend (no pinching)
    const n1 = i > 0 ? segN(line[i - 1], line[i]) : null, n2 = i < line.length - 1 ? segN(line[i], line[i + 1]) : null;
    const a = n1 || n2, b = n2 || n1;
    let bx = a.x + b.x, by = a.y + b.y; const bl = Math.hypot(bx, by) || 1; bx /= bl; by /= bl;
    const miter = Math.min(2.5, 1 / Math.max(0.4, bx * a.x + by * a.y));   // capped so a very sharp bend doesn't spike out
    return { x: bx * h * miter, y: by * h * miter };
  };
  const left = [], right = [];
  for (let i = 0; i < line.length; i++) { const o = offset(i); left.push({ x: line[i].x + o.x, y: line[i].y + o.y }); right.push({ x: line[i].x - o.x, y: line[i].y - o.y }); }
  return left.concat(right.reverse());
}
function spineBlockers() { return bPolys().concat(S.obstacles || [], S.roads || []); }
function spineKeptSide(sp, park, aIdx) {   // single-loaded aisle → which side of the centre-line the surviving stalls sit (+1 / -1), 0 if none
  const a = sp.line[0], b = sp.line[sp.line.length - 1], L = Math.hypot(b.x - a.x, b.y - a.y) || 1;
  const per = { x: -(b.y - a.y) / L, y: (b.x - a.x) / L };   // same perp convention as tileStallsAlongSpine
  let pos = 0, neg = 0;                                       // count both sides (robust even if ever handed a mixed aisle)
  for (const s of park.stalls) { if (s.spine !== aIdx) continue; ((s.cx - a.x) * per.x + (s.cy - a.y) * per.y) >= 0 ? pos++ : neg++; }
  return (pos === 0 && neg === 0) ? 0 : (pos >= neg ? 1 : -1);
}
function retileSpine(si, avoidOverlap) {   // re-shape one spine — drag a node → an aisle re-tiles its stalls, a connector lane just reshapes
  const park = activePark(); if (!park || !park.spines || !park.spines[si]) return;
  const sp = park.spines[si];
  if (sp.kind === 'conn') {                                              // a CONNECTOR lane — reshape geometry only (no stalls attach)
    const c = park.connectors[sp.src], poly = spineToRect(sp.line, sp.width);
    park.connectors[sp.src] = (c && typeof c === 'object' && !Array.isArray(c)) ? Object.assign({}, c, { poly }) : { poly };
    return;
  }
  const aIdx = sp.src;                                                   // aisle index (= si, since aisle spines come first)
  const p = { stallW: S.params.stallW, vpd: S.params.stallD, aisle: sp.width, setback: S.params.setback, greenBuffer: S.params.greenBuffer || 0, angle: 90 };
  const others = park.stalls.filter(s => s.spine !== si);                // every OTHER aisle's stalls
  let fresh = PS.tileStallsAlongSpine(sp.line, p, S.boundary, spineBlockers(), sp.sides || 0);
  // drop a fresh stall only if it REALLY overlaps a neighbour — shrink it ~1ft first so a normal back-to-back
  // touch (rows from adjacent aisles meet edge-to-edge) is kept, not wrongly culled by edge-touch polyOverlap.
  const shrink = poly => { const cx = (poly[0].x + poly[1].x + poly[2].x + poly[3].x) / 4, cy = (poly[0].y + poly[1].y + poly[2].y + poly[3].y) / 4; return poly.map(p => ({ x: cx + (p.x - cx) * 0.88, y: cy + (p.y - cy) * 0.88 })); };
  if (avoidOverlap) fresh = fresh.filter(ns => { const ins = shrink(ns.poly); return !others.some(os => PS.polyOverlap(ins, os.poly)); });
  fresh.forEach(s => s.spine = si);
  park.stalls = others.concat(fresh);                                    // swap out this spine's stalls
  park.aisles[aIdx] = { poly: spineToRect(sp.line, sp.width) };          // keep the grey lane in sync
}
// After a manual lane edit, keep the lot drivable WITHOUT wrecking it. A lane move leaves the cross-aisles
// intact (they span the rows); only the entrance ramp can lose the aisle it landed on. So rebuild circulation
// on a THROWAWAY copy purely to get fresh entrance ramps, then graft ONLY those ramps onto the real layout —
// every stall and every cross-aisle is kept (no aisle re-clip, no stall pruning). Lossless re-link.
function reconnectNetwork(full) {   // full=true → rebuild ALL connectors incl. cross-aisles (for ADDING a new aisle); default keeps the existing cross-aisles (lossless re-link for a MOVE)
  const park = activePark();
  if (!park || !park.aisles || !park.aisles.length || !window.PS.buildCirculation) return;
  // circulation inputs: parking mode re-links against the live globals; site mode replays the EXACT boundary /
  // blockers / entrances solveSite packed this lot against (stashed on park._circ) — they ≠ the site globals.
  let boundary, ents, aisleW, open, builds, obs;
  if (S.mode === 'site') { const c = park._circ; if (!c) return; boundary = c.boundary; ents = c.entrances; aisleW = c.aisle; open = c.open; builds = c.buildings || []; obs = c.obstacles || []; }
  else { boundary = S.boundary; ents = S.entrances; aisleW = S.params.aisle; open = S.params.access === 'open'; builds = bPolys(); obs = S.obstacles || []; }
  if (!boundary || boundary.length < 3 || !ents || !ents.length) return;
  const clone = { aisles: park.aisles.map(a => ({ poly: a.poly.map(p => ({ x: p.x, y: p.y })) })), stalls: park.stalls.map(s => ({ cx: s.cx, cy: s.cy, aprobe: s.aprobe, poly: s.poly })), theta: park.theta };
  PS.buildCirculation(clone, boundary, ents, aisleW, open, builds, obs);
  const ramps = (clone.connectors || []).filter(c => c.ent != null || c.type);          // entrance ramps only
  const cross = (park.connectors || []).filter(c => c.ent == null && !c.type);           // keep the real cross-aisles
  park.connectors = full ? (clone.connectors || []).slice() : cross.concat(ramps);       // ADD → take the freshly-knit cross-aisles+ramps (connects the new aisle); MOVE → keep old cross-aisles + fresh ramps
  // drop only stalls physically sitting on a drive lane (a re-tiled lane runs its stalls across the fixed
  // cross-aisles) — can't park where cars drive. Area test (>8% under) so a stall merely grazed survives.
  const drives = park.connectors.map(c => c.poly || c);
  park.stalls = park.stalls.filter(s => {
    const cand = drives.filter(d => PS.polyOverlap(s.poly, d)); if (!cand.length) return true;
    const b = PS.bbox(s.poly); let inside = 0, under = 0;
    for (let x = b.minX; x <= b.maxX; x += 1.5) for (let y = b.minY; y <= b.maxY; y += 1.5) { const pt = { x, y }; if (!PS.pointInPoly(pt, s.poly)) continue; inside++; if (cand.some(d => PS.pointInPoly(pt, d))) under++; }
    return under / Math.max(inside, 1) <= 0.08;
  });
  deriveSpines(park);                                  // refresh editable spines + re-tag stalls + keep single-load sides
}
function insertSpineNode(si, w) {   // add a bend node into aisle spine si at the point on its centre-line nearest world point w
  const park = activePark(), sp = park && park.spines && park.spines[si]; if (!sp || sp.kind !== 'aisle') return false;
  const ln = sp.line; let best = -1, bestD = 1e9, bestPt = null;
  for (let i = 0; i + 1 < ln.length; i++) {
    const a = ln[i], b = ln[i + 1], L2 = (b.x - a.x) ** 2 + (b.y - a.y) ** 2 || 1;
    let t = ((w.x - a.x) * (b.x - a.x) + (w.y - a.y) * (b.y - a.y)) / L2; t = Math.max(0, Math.min(1, t));
    const px = a.x + (b.x - a.x) * t, py = a.y + (b.y - a.y) * t, d = Math.hypot(w.x - px, w.y - py);
    if (d < bestD) { bestD = d; best = i; bestPt = { x: px, y: py }; }
  }
  if (best < 0 || !bestPt) return false;
  if (Math.hypot(bestPt.x - ln[best].x, bestPt.y - ln[best].y) < 3 || Math.hypot(bestPt.x - ln[best + 1].x, bestPt.y - ln[best + 1].y) < 3) return false;   // too close to an existing node → don't make a zero-length segment
  ln.splice(best + 1, 0, bestPt);                                          // new node between the two it sits between
  retileSpine(si, true); reconnectNetwork(); return true;
}
// ADAPTIVE RE-FLOW after a manual lane move/reshape (TestFit-style "move a road and the lot adapts"):
// re-tile EVERY drive aisle so the moved lane AND its neighbours refill the freed space, then FULL-rebuild
// the road network so the OTHER roads come and connect to the moved lane (instead of a lossless move that
// left gaps + stale cross-aisles). Manual single-load / bend state is preserved (retileSpine honours sp.sides
// and the hand-drawn line).
function reflowAfterEdit() {
  const park = activePark();
  if (!park || !park.spines) { reconnectNetwork(true); return; }
  for (let i = 0; i < park.spines.length; i++) if (park.spines[i].kind === 'aisle') retileSpine(i, true);
  reconnectNetwork(true);
}
// MANUAL ADD (TestFit-style): a user-drawn centre-line becomes a brand-new double-loaded drive aisle —
// stalls tile along it, then the whole drive network re-links so it connects to the entrances (lossless).
function addManualAisle(line) {
  const park = activePark();
  if (!park || !park.aisles) { toast('請先「自動排車位」，再加場內道路'); return; }
  if (!line || line.length < 2) return;
  const width = S.params.aisle || 24;
  const p = { stallW: S.params.stallW, vpd: S.params.stallD, aisle: width, setback: S.params.setback, greenBuffer: S.params.greenBuffer || 0, angle: 90 };
  const idx = park.aisles.length;                                          // new aisle index (spines preserve hand-drawn lines by this index)
  park.aisles.push({ poly: spineToRect(line, width) });
  park.spines = park.spines || [];
  park.spines.push({ line: line.map(q => ({ x: q.x, y: q.y })), width, kind: 'aisle', src: idx });
  let fresh = PS.tileStallsAlongSpine(line, p, S.boundary, spineBlockers(), 0);   // double-loaded along the drawn line
  const shrink = poly => { const cx = (poly[0].x + poly[1].x + poly[2].x + poly[3].x) / 4, cy = (poly[0].y + poly[1].y + poly[2].y + poly[3].y) / 4; return poly.map(q => ({ x: cx + (q.x - cx) * 0.88, y: cy + (q.y - cy) * 0.88 })); };
  fresh = fresh.filter(ns => !park.stalls.some(os => PS.polyOverlap(shrink(ns.poly), os.poly)));   // drop only stalls that really overlap an existing one (back-to-back touch survives)
  fresh.forEach(s => s.spine = idx);
  park.stalls.push(...fresh);
  reconnectNetwork(true);                                                  // FULL rebuild → knit fresh cross-aisles so the new aisle connects to the gate network
  (S.mode === 'site' ? updateSiteMetrics : updateMetrics)();
  draw(); commit();
  toast(`已新增場內道路（＋${fresh.length} 車位，已接入動線網）`);
}
function aisleAxis(poly) {        // {c, per} centroid + unit perpendicular of an aisle strip (per points across the lane)
  const c = PS.centroid(poly); let dir = { x: 1, y: 0 }, best = -1;
  for (let i = 0; i < poly.length; i++) { const a = poly[i], b = poly[(i + 1) % poly.length], L = Math.hypot(b.x - a.x, b.y - a.y); if (L > best) { best = L; dir = { x: (b.x - a.x) / L, y: (b.y - a.y) / L }; } }
  return { c, per: { x: -dir.y, y: dir.x } };
}
function connectorAt(w) {         // index of an auto connector/spine under w (the orange/green "glue" lanes) or -1
  const park = activePark(); if (!park || !park.connectors || !S.layers.parking.vis) return -1;
  for (let i = 0; i < park.connectors.length; i++) { const poly = park.connectors[i].poly || park.connectors[i]; if (poly && poly.length >= 3 && PS.pointInPoly(w, poly)) return i; }
  return -1;
}
// REVERSIBLE aisle overrides: edits are stored by aisle position and re-applied after every pack,
// so 單邊停 / 移除 survive a re-solve AND can be undone (clear the override → re-pack restores the stalls).
function aisleKey(poly) { const c = PS.centroid(poly); return Math.round(c.x / 3) + ',' + Math.round(c.y / 3); }
function applyAisleEdits(park) {
  const E = S.aisleEdits; if (!park || !E || !park.aisles) return;
  for (let i = park.aisles.length - 1; i >= 0; i--) {
    const op = E[aisleKey(park.aisles[i].poly)]; if (!op) continue;
    const ais = park.aisles[i].poly;
    if (op === 'remove') { park.stalls = park.stalls.filter(s => !(s.aprobe && PS.pointInPoly(s.aprobe, ais))); park.aisles.splice(i, 1); }
    else if (op === 'single' || op === 'single2') { const ax = aisleAxis(ais), sgn = op === 'single2' ? -1 : 1; park.stalls = park.stalls.filter(s => !(s.aprobe && PS.pointInPoly(s.aprobe, ais)) || (((s.cx - ax.c.x) * ax.per.x + (s.cy - ax.c.y) * ax.per.y) * sgn >= 0)); }   // single = +side, single2 = flipped (−side)
  }
}
function setAisleEdit(i, op) {     // record (or clear, op=null) an override for an aisle, then apply/restore
  const park = activePark(); if (!park || !park.aisles[i]) return;
  const k = aisleKey(park.aisles[i].poly); S.aisleEdits = S.aisleEdits || {};
  if (op) S.aisleEdits[k] = op; else delete S.aisleEdits[k];
  if (op === 'single' || op === 'single2') {          // apply now to the live solution (instant); keep the aisle selected
    if (park.spines && park.spines[i] && park.spines[i].kind === 'aisle' && park.spines[i].sides) { park.spines[i].sides = 0; retileSpine(i, true); }   // restore BOTH rows first so the side-filter has both sides to pick from (lets 翻面 re-flip, and re-單邊停 after a flip, instead of filtering an already-emptied side to 0)
    applyAisleEdits(park);
    if (park.spines && park.spines[i]) park.spines[i].sides = spineKeptSide(park.spines[i], park, i);   // so a later spine drag won't revert to double
    (S.mode === 'site' ? updateSiteMetrics : updateMetrics)(); commit(); showAislePopup(i); draw();
    toast('已改為單邊停車（再按「翻面」換邊、「雙邊停」還原）');
  } else {                        // remove, or restore-to-double → re-pack so removed stalls come back / drop cleanly
    S.selAisle = null; hideAislePopup(); resolveActive();
    toast(op === 'remove' ? '已移除這條場內道路與車位' : '已還原雙邊停車');
  }
}
function removeAisle(i) { setAisleEdit(i, 'remove'); }
function singleLoadAisle(i) { setAisleEdit(i, 'single'); }
function restoreAisle(i) { setAisleEdit(i, null); }
function flipAisle(i) {              // single-loaded 場內道路 → swap which side keeps its stalls (TestFit "flip")
  const park = activePark(); if (!park || !park.aisles[i]) return;
  const cur = (S.aisleEdits || {})[aisleKey(park.aisles[i].poly)];
  if (cur !== 'single' && cur !== 'single2') { toast('先按「單邊停」，才能翻到另一邊'); return; }
  setAisleEdit(i, cur === 'single' ? 'single2' : 'single');
}
function roadChanged() {          // re-derive strips, re-pack around the roads, and record one undo step
  rebuildRoadStrips();
  const reSolve = (S.mode === 'site' ? !!S.site : !!S.solution);
  resolveActive();               // doSolve/doSolveSite commit the final state themselves
  if (!reSolve) commit();
  draw();
}
const ROAD_W = { narrow: 18, std: 24, wide: 36 };   // ft — road-width presets
function roadPopupAnchor(rl) { const ln = rl.line, mid = ln[Math.floor(ln.length / 2)] || ln[0]; return toScreen(mid); }
function showRoadPopup(ri) {
  const rl = S.roadLines[ri]; const el = $('#roadPopup'); if (!rl || !el) return;
  const a = roadPopupAnchor(rl); el.style.left = a.x + 'px'; el.style.top = (a.y - 14) + 'px'; el.classList.add('show');
  el.querySelectorAll('[data-rw]').forEach(b => b.classList.toggle('active', Math.abs((rl.width || 24) - ROAD_W[b.dataset.rw]) < 0.6));
}
function hideRoadPopup() { const el = $('#roadPopup'); if (el) el.classList.remove('show'); }
function showAislePopup(i) {
  const park = activePark(), el = $('#aislePopup'); if (!park || !park.aisles[i] || !el) return;
  const a = toScreen(PS.centroid(park.aisles[i].poly)); el.style.left = a.x + 'px'; el.style.top = (a.y - 12) + 'px'; el.classList.add('show');
}
function hideAislePopup() { const el = $('#aislePopup'); if (el) el.classList.remove('show'); }
// draw user roads as smooth continuous asphalt: stroke the centre-line with round joins/caps (curb edge under,
// asphalt body, dashed yellow centre stripe). Round joins make bends real instead of blocky box segments.
// UNIT FIT floor plan: colour-coded apartment-unit rectangles tiled into the residential plate
// (2D plan view only). Corridors drawn first (light), units on top with thin white party walls.
function drawUnitPlan() {
  const up = S.site && S.site.unitPlan;
  if (!up || !up.plan || !up.plan.length || !S.layers.unitfit || !S.layers.unitfit.vis || S.is3d || S.mapMode) return;
  (up.corridors || []).forEach(c => { pathPoly(c, true); ctx.fillStyle = 'rgba(226,232,240,.55)'; ctx.fill(); });
  ctx.lineWidth = 0.6; ctx.strokeStyle = 'rgba(255,255,255,.85)';
  for (const u of up.plan) {
    pathPoly(u.poly, true);
    ctx.fillStyle = UNIT_FIT_COLORS[u.type] || '#93c5fd'; ctx.globalAlpha = 0.82; ctx.fill(); ctx.globalAlpha = 1;
    ctx.stroke();
  }
}

function drawRoads() {
  const lines = S.roadLines || [];
  const trace = line => { ctx.beginPath(); line.forEach((p, i) => { const s = toScreen(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); }); };
  if (!lines.length) { for (const r of (S.roads || [])) { pathPoly(r, true); ctx.fillStyle = 'rgba(71,85,105,.9)'; ctx.fill(); } return; }  // legacy saves: flat strips
  const a = toScreen({ x: 0, y: 0 }), b = toScreen({ x: 1, y: 0 }), sc = Math.hypot(b.x - a.x, b.y - a.y);   // px per foot (map-safe)
  ctx.save(); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  for (const rd of lines) { const w = (rd.width || 24) * sc; trace(rd.line); ctx.lineWidth = w + 5; ctx.strokeStyle = 'rgba(30,41,59,.95)'; ctx.stroke(); }   // curb
  for (const rd of lines) { const w = (rd.width || 24) * sc; trace(rd.line); ctx.lineWidth = Math.max(w, 2); ctx.strokeStyle = 'rgba(74,85,104,.96)'; ctx.stroke(); }  // asphalt
  ctx.setLineDash([14, 11]);
  for (const rd of lines) { trace(rd.line); ctx.lineWidth = 2; ctx.strokeStyle = 'rgba(250,204,21,.92)'; ctx.stroke(); }   // centre stripe
  ctx.restore();
  // editable handles in select mode — a node on every centre-line vertex (drag ends to extend/shorten, mids to reshape)
  if (S.tool === 'select' && !S.is3d) {
    for (let ri = 0; ri < lines.length; ri++) {
      const sel = S.selRoad === ri;
      for (const p of lines[ri].line) {
        const s = toScreen(p);
        ctx.beginPath(); ctx.arc(s.x, s.y, sel ? 6 : 4.5, 0, 6.2832);
        ctx.fillStyle = sel ? '#facc15' : 'rgba(250,204,21,.9)'; ctx.fill();
        ctx.lineWidth = 1.5; ctx.strokeStyle = '#0f172a'; ctx.stroke();
      }
    }
  }
}
// fill a building footprint with facility/road VOIDS punched out (even-odd), so it wraps around them
function fillVoids(outer, voids, fill, stroke, lw) {
  ctx.beginPath(); subPath(outer); (voids || []).forEach(subPath);
  ctx.fillStyle = fill; ctx.fill('evenodd');
  if (stroke) { ctx.lineWidth = lw || 2; ctx.strokeStyle = stroke; pathPoly(outer, true); ctx.stroke(); (voids || []).forEach(v => { pathPoly(v, true); ctx.stroke(); }); }
}

function drawStall(s) {
  // compact stalls are physically packed narrower already; a motorcycle stall is a small stall, drawn inset.
  const poly = s.type === 'moto' ? s.poly.map(p => ({ x: p.x + (s.cx - p.x) * 0.42, y: p.y + (s.cy - p.y) * 0.42 })) : s.poly;
  pathPoly(poly, true);
  const col = COLORS[s.type] || COLORS.standard;
  ctx.fillStyle = hexA(col, .8); ctx.fill();
  ctx.lineWidth = 1; ctx.strokeStyle = hexA(col, 1); ctx.stroke();
  // small icon for special types when zoomed in enough
  const sizePx = S.view.scale * S.params.stallW;
  if (sizePx > 14 && s.type !== 'standard') {
    const c = toScreen({ x: s.cx, y: s.cy });
    ctx.fillStyle = '#fff'; ctx.font = `${Math.min(sizePx * .55, 13)}px system-ui`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    const ic = s.type === 'ada' ? '♿' : s.type === 'ev' ? '⚡' : s.type === 'compact' ? 'c' : s.type === 'moto' ? 'M' : '◧';
    ctx.fillText(ic, c.x, c.y);
  }
}

function drawAisleArrows(sol) {
  if (!sol) return;
  if (!S.mapMode && S.view.scale * S.params.aisle < 16) return;
  const oneway = S.params.oneway;
  ctx.strokeStyle = oneway ? 'rgba(250,204,21,.55)' : 'rgba(148,163,184,.28)';   // quiet — direction is still readable but no longer clutters the lot
  ctx.lineWidth = oneway ? 1.6 : 1.1;
  sol.aisles.forEach((a, ai) => {
    const p0 = a.poly[0], p1 = a.poly[1], p2 = a.poly[3];
    let mid0 = { x: (p0.x + p2.x) / 2, y: (p0.y + p2.y) / 2 };
    let mid1 = { x: (p1.x + a.poly[2].x) / 2, y: (p1.y + a.poly[2].y) / 2 };
    if (oneway && ai % 2 === 1) { const t = mid0; mid0 = mid1; mid1 = t; }   // alternate one-way flow
    const len = Math.hypot(mid1.x - mid0.x, mid1.y - mid0.y);
    const n = Math.max(1, Math.floor(len / (oneway ? 90 : 130)));   // sparser arrows = calmer lot
    const ang = Math.atan2(mid1.y - mid0.y, mid1.x - mid0.x);
    for (let i = 1; i <= n; i++) {
      const t = i / (n + 1);
      arrow(toScreen({ x: mid0.x + (mid1.x - mid0.x) * t, y: mid0.y + (mid1.y - mid0.y) * t }), ang, 6);
    }
  });
}
function arrow(p, ang, r) {
  ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(ang);
  ctx.beginPath(); ctx.moveTo(-r, -r * .7); ctx.lineTo(r, 0); ctx.lineTo(-r, r * .7);
  ctx.stroke(); ctx.restore();
}

function drawSetback() {
  if (S.params.setback <= 0 || S.boundary.length < 3) return;
  // visual only: draw boundary dashed slightly — true setback enforced in solver
  ctx.save();
  ctx.setLineDash([5, 4]); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(56,189,248,.4)';
  // approximate inset by scaling toward centroid
  const c = PS.centroid(S.boundary);
  const inset = S.boundary.map(p => {
    const dx = p.x - c.x, dy = p.y - c.y, d = Math.hypot(dx, dy) || 1;
    const k = Math.max(0, (d - S.params.setback) / d);
    return { x: c.x + dx * k, y: c.y + dy * k };
  });
  pathPoly(inset, true); ctx.stroke();
  ctx.restore();
}

function drawEntrance(e) {
  const p = toScreen(e);
  const type = e.type || 'inout';
  const col = type === 'in' ? '#22c55e' : type === 'out' ? '#f59e0b' : '#38bdf8';
  const label = type === 'in' ? '進' : type === 'out' ? '出' : '進出';
  // driveway / curb-cut stub + direction arrowhead(s): 進 in, 出 out, 進出 both
  if (S.boundary.length >= 3) {
    // driveway aims PERPENDICULAR to the gate's OWN edge (inward) — matching the solver's drive
    // lane — instead of toward the site centroid (which skewed it toward the middle / building).
    const nrm = PS.inwardEdgeNormal(S.boundary, e, PS.centroid(S.boundary));
    const pIn = toScreen({ x: e.x + nrm.x * 20, y: e.y + nrm.y * 20 });
    const ang = Math.atan2(pIn.y - p.y, pIn.x - p.x);
    const inEnd = { x: p.x + Math.cos(ang) * 30, y: p.y + Math.sin(ang) * 30 };
    ctx.save();
    ctx.strokeStyle = hexA(col, .6); ctx.lineWidth = 11; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(inEnd.x, inEnd.y); ctx.stroke();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2.5;
    if (type === 'in' || type === 'inout') arrow(inEnd, ang, 6);                 // into site
    if (type === 'out' || type === 'inout') arrow({ x: p.x + Math.cos(ang) * 8, y: p.y + Math.sin(ang) * 8 }, ang + Math.PI, 6); // out of site
    ctx.restore();
  }
  const sel = S.selEntrance === e;
  ctx.beginPath(); ctx.arc(p.x, p.y, sel ? 13 : 11, 0, 7);
  ctx.fillStyle = col; ctx.fill();
  ctx.lineWidth = sel ? 3 : 2; ctx.strokeStyle = sel ? '#fff' : '#001018'; ctx.stroke();
  ctx.fillStyle = '#001018'; ctx.font = 'bold 10px system-ui';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(label, p.x, p.y);
}

function hatch(poly, color, alpha) {
  const bb = PS.bbox(poly);
  ctx.save();
  pathPoly(poly, true); ctx.clip();
  ctx.strokeStyle = hexA(color, alpha); ctx.lineWidth = 1;
  const step = 10 * S.view.scale > 6 ? 10 : 20;
  const a = toScreen({ x: bb.minX, y: bb.minY }), b = toScreen({ x: bb.maxX, y: bb.maxY });
  for (let x = a.x - (b.y - a.y); x < b.x; x += 8) {
    ctx.beginPath(); ctx.moveTo(x, a.y); ctx.lineTo(x + (b.y - a.y), b.y); ctx.stroke();
  }
  ctx.restore();
}

function labelPoly(poly, txt, color) {
  const c = PS.centroid(poly), p = toScreen(c);
  ctx.fillStyle = color; ctx.font = '10px system-ui';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(txt, p.x, p.y);
}

function drawDraft() {
  const pts = S.draft.slice();
  if (S.hoverWorld) pts.push(S.hoverWorld);
  ctx.save();
  if ((S.draftKind === 'road' || S.draftKind === 'aisle') && pts.length >= 2) {            // preview the road / aisle at its real width while drawing
    const a = toScreen({ x: 0, y: 0 }), b = toScreen({ x: 1, y: 0 }), sc = Math.hypot(b.x - a.x, b.y - a.y);
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    ctx.lineWidth = (S.draftKind === 'aisle' ? (S.params.aisle || 24) : (S.roadWidth || 24)) * sc;
    ctx.strokeStyle = S.draftKind === 'aisle' ? 'rgba(148,163,184,.40)' : 'rgba(74,85,104,.55)';
    ctx.beginPath(); pts.forEach((p, i) => { const s = toScreen(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); }); ctx.stroke();
  }
  ctx.setLineDash([6, 4]); ctx.lineWidth = S.draftKind === 'road' ? 3 : 1.8;
  ctx.strokeStyle = S.draftKind === 'obstacle' ? '#ef4444' : S.draftKind === 'building' ? '#94a3b8' : S.draftKind === 'road' ? '#facc15' : S.draftKind === 'aisle' ? '#cbd5e1' : '#38bdf8';
  pathPoly(pts, false); ctx.stroke();
  ctx.setLineDash([]);
  for (const v of S.draft) {
    const p = toScreen(v);
    ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, 7); ctx.fillStyle = '#fff'; ctx.fill();
  }
  if (S.draftKind === 'road' && S.draft.length >= 2) {                  // hint: how to finish an open road
    const last = toScreen(S.draft[S.draft.length - 1]);
    ctx.fillStyle = 'rgba(250,204,21,.95)'; ctx.font = '11px system-ui'; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    ctx.fillText('雙擊或 Enter 完成道路', last.x + 10, last.y - 6);
  }
  // highlight close target
  if (S.draftKind !== 'road' && S.draft.length >= 3 && S.hoverWorld) {
    const first = toScreen(S.draft[0]);
    const hv = toScreen(S.hoverWorld);
    if (Math.hypot(first.x - hv.x, first.y - hv.y) < 12) {
      ctx.beginPath(); ctx.arc(first.x, first.y, 8, 0, 7);
      ctx.strokeStyle = '#22c55e'; ctx.lineWidth = 2; ctx.stroke();
    }
  }
  ctx.restore();
}

function drawGrid() {
  const stepFt = niceStep(80 / S.view.scale);   // ~80px target
  const px = stepFt * S.view.scale;
  if (px < 8) return;
  const x0 = -S.view.ox / S.view.scale, y0 = -S.view.oy / S.view.scale;
  const startX = Math.floor(x0 / stepFt) * stepFt, startY = Math.floor(y0 / stepFt) * stepFt;
  ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(148,163,184,.07)';
  ctx.beginPath();
  for (let x = startX; x < x0 + cv._w / S.view.scale; x += stepFt) {
    const s = toScreen({ x, y: 0 }); ctx.moveTo(s.x, 0); ctx.lineTo(s.x, cv._h);
  }
  for (let y = startY; y < y0 + cv._h / S.view.scale; y += stepFt) {
    const s = toScreen({ x: 0, y }); ctx.moveTo(0, s.y); ctx.lineTo(cv._w, s.y);
  }
  ctx.stroke();
  updateScalebar(stepFt);
}
function niceStep(v) {
  const p = Math.pow(10, Math.floor(Math.log10(v)));
  const n = v / p;
  return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * p;
}
function updateScalebar(stepFt) {
  const px = stepFt * S.view.scale;
  if (!isFinite(px) || !isFinite(stepFt) || px <= 0) return;
  $('#scaleBar').style.width = px + 'px';
  $('#scaleTxt').textContent = `${stepFt >= 1 ? stepFt : stepFt.toFixed(1)} ft`;
}

function hexA(hex, a) {
  const n = parseInt(hex.slice(1), 16);
  return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${a})`;
}
function shade(hex, amt) {                  // amt -1..1 : darken..lighten
  const n = parseInt(hex.slice(1), 16);
  let r = (n >> 16) & 255, g = (n >> 8) & 255, b = n & 255;
  const f = amt < 0 ? 0 : 255, t = Math.abs(amt);
  r = Math.round((f - r) * t) + r; g = Math.round((f - g) * t) + g; b = Math.round((f - b) * t) + b;
  return `rgb(${r},${g},${b})`;
}

/* ------------------------------ landscape trees -------------------------- */
function treePositions() {
  const out = [];
  if (S.boundary.length < 3) return out;
  const park = S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution;
  const c = PS.centroid(S.boundary);
  // sit trees in the perimeter landscape band (setback / green buffer), not on the parking
  const inset = Math.max(7, (S.params.setback || 8) * 0.55 + (S.params.greenBuffer || 0) * 0.5);
  const blocked = pt => {                       // keep trees out of stalls / drives / buildings / the gate
    for (const b of S.buildings) if (PS.pointInPoly(pt, b.poly)) return true;
    for (const o of S.obstacles) if (PS.pointInPoly(pt, o)) return true;
    for (const e of S.entrances) if (Math.hypot(pt.x - e.x, pt.y - e.y) < 24) return true;
    if (park) {
      for (const s of park.stalls) if (PS.pointInPoly(pt, s.poly)) return true;
      if (park.connectors) for (const cn of park.connectors) if (PS.pointInPoly(pt, cn.poly || cn)) return true;
      for (const a of park.aisles) if (PS.pointInPoly(pt, a.poly)) return true;
    }
    return false;
  };
  for (let i = 0; i < S.boundary.length; i++) {
    const a = S.boundary[i], b = S.boundary[(i + 1) % S.boundary.length];
    const n = Math.max(1, Math.round(Math.hypot(b.x - a.x, b.y - a.y) / 40));   // ~40 ft spacing
    for (let k = 0; k <= n; k++) {
      const t = k / n, x = a.x + (b.x - a.x) * t, y = a.y + (b.y - a.y) * t;
      const dx = c.x - x, dy = c.y - y, d = Math.hypot(dx, dy) || 1;
      const pt = { x: x + dx / d * inset, y: y + dy / d * inset, r: 7 };
      if (!PS.pointInPoly(pt, S.boundary)) continue;             // inside the lot
      if (blocked(pt)) continue;                                  // not over parking / drives / buildings
      out.push(pt);
    }
  }
  return out;
}
// CIRCULATION HEALTH: flood the drive network from the gates, route every stall's trip back to a gate, and
// accumulate traffic per cell → a congestion heat-map (red = funnels/choke points, green = quiet aisles).
// Bottlenecks = high-traffic cells with ≤2 drive neighbours (a narrow passage everything squeezes through).
function computeFlow() {
  const park = S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution;
  if (!park || !park.stalls || !park.stalls.length || S.boundary.length < 3) return { cells: [], max: 1, bottlenecks: 0, deadends: 0 };
  const drives = (park.aisles || []).map(a => a.poly).concat((park.connectors || []).map(c => c.poly || c));
  if (!drives.length) return { cells: [], max: 1, bottlenecks: 0, deadends: 0 };
  const inAny = (pt, ps) => ps.some(poly => PS.pointInPoly(pt, poly));
  const bb = PS.bbox(S.boundary), CELL = 3.5, W = Math.ceil((bb.maxX - bb.minX) / CELL) + 2, Hh = Math.ceil((bb.maxY - bb.minY) / CELL) + 2;
  const idx = (i, j) => j * W + i, ctr = (i, j) => ({ x: bb.minX + (i - .5) * CELL, y: bb.minY + (j - .5) * CELL });
  const net = new Uint8Array(W * Hh);
  for (let j = 0; j < Hh; j++) for (let i = 0; i < W; i++) { const pt = ctr(i, j); if (PS.pointInPoly(pt, S.boundary) && inAny(pt, drives)) net[idx(i, j)] = 1; }
  const parent = new Int32Array(W * Hh).fill(-1), dist = new Int32Array(W * Hh).fill(-1), q = [];
  for (const e of (S.entrances || [])) {
    let bi = Math.round((e.x - bb.minX) / CELL), bj = Math.round((e.y - bb.minY) / CELL), best = -1, bd = 1e9;
    for (let dj = -30; dj <= 30; dj++) for (let di = -30; di <= 30; di++) { const i = bi + di, j = bj + dj; if (i < 0 || j < 0 || i >= W || j >= Hh) continue; const k = idx(i, j); if (net[k]) { const d = di * di + dj * dj; if (d < bd) { bd = d; best = k; } } }
    if (best >= 0 && dist[best] < 0) { dist[best] = 0; parent[best] = best; q.push(best); }
  }
  for (let qi = 0; qi < q.length; qi++) { const k = q[qi], i = k % W, j = (k - i) / W; for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { const ni = i + di, nj = j + dj; if (ni < 0 || nj < 0 || ni >= W || nj >= Hh) continue; const nk = idx(ni, nj); if (net[nk] && dist[nk] < 0) { dist[nk] = dist[k] + 1; parent[nk] = k; q.push(nk); } } }
  const load = new Float32Array(W * Hh);
  for (const s of park.stalls) {
    let bi = Math.round((s.cx - bb.minX) / CELL), bj = Math.round((s.cy - bb.minY) / CELL), best = -1, bd = 1e9;
    for (let dj = -3; dj <= 3; dj++) for (let di = -3; di <= 3; di++) { const i = bi + di, j = bj + dj; if (i < 0 || j < 0 || i >= W || j >= Hh) continue; const k = idx(i, j); if (net[k] && dist[k] >= 0) { const d = di * di + dj * dj; if (d < bd) { bd = d; best = k; } } }
    if (best < 0) continue; let k = best, g = 0; while (k >= 0 && parent[k] !== k && g++ < W * Hh) { load[k]++; k = parent[k]; } if (k >= 0) load[k]++;
  }
  let max = 1; for (let k = 0; k < W * Hh; k++) if (load[k] > max) max = load[k];
  const cells = []; let bottlenecks = 0, deadends = 0;
  for (let j = 0; j < Hh; j++) for (let i = 0; i < W; i++) { const k = idx(i, j); if (!net[k] || load[k] <= 0) continue;
    let deg = 0; for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { const ni = i + di, nj = j + dj; if (ni >= 0 && nj >= 0 && ni < W && nj < Hh && net[idx(ni, nj)]) deg++; }
    const t = load[k] / max; cells.push({ x: ctr(i, j).x, y: ctr(i, j).y, t });
    if (t > 0.6 && deg <= 2) bottlenecks++;
    if (deg <= 1 && dist[k] > 4) deadends++;
  }
  return { cells, max, bottlenecks, deadends, cell: CELL };
}
function drawFlowOverlay() {
  if (!S.layers.flow.vis || S.is3d || S.mapMode) return;
  const f = S._flowCache || (S._flowCache = computeFlow());
  if (!f.cells.length) return;
  const a = toScreen({ x: 0, y: 0 }), b = toScreen({ x: 1, y: 0 }), px = Math.max(2, Math.hypot(b.x - a.x, b.y - a.y) * f.cell);
  ctx.save(); pathPoly(S.boundary, true); ctx.clip();
  for (const c of f.cells) {                                   // single-hue RED that fades in by traffic: faint = quiet, deep red = congested
    const s = toScreen(c), t = c.t;
    ctx.fillStyle = `rgba(${(244 - 64 * t) | 0},${(63 - 48 * t) | 0},${(63 - 43 * t) | 0},${(0.06 + 0.62 * t).toFixed(2)})`;
    ctx.fillRect(s.x - px / 2, s.y - px / 2, px + 1, px + 1);
  }
  ctx.restore();
}
// CUT & FILL: model existing ground as a bilinear surface from the 4 corner spot-elevations, sample the parcel
// on a grid, and tally cut (ground above pad) vs fill (below) volumes → earthwork cost. Balance pad = mean ground.
function computeEarthwork() {
  const out = $('#ewResult'); if (!out) return;
  if (S.boundary.length < 3) { out.textContent = '先畫基地。'; S._ewCache = null; return; }
  const NW = +$('#eNW').value, NE = +$('#eNE').value, SW = +$('#eSW').value, SE = +$('#eSE').value;
  if (!NW && !NE && !SW && !SE) { out.textContent = '輸入四角標高後自動算挖填量與土方成本。'; S._ewCache = null; return; }
  const pad = +$('#ePad').value, cutC = +$('#eCutC').value, fillC = +$('#eFillC').value, haulC = +$('#eHaulC').value;
  const bb = PS.bbox(S.boundary), CELL = 5, dx = Math.max(bb.maxX - bb.minX, 1), dy = Math.max(bb.maxY - bb.minY, 1);
  const cellA = CELL * CELL; let cut = 0, fill = 0, gSum = 0, gA = 0; const cells = [];
  for (let y = bb.minY + CELL / 2; y < bb.maxY; y += CELL) for (let x = bb.minX + CELL / 2; x < bb.maxX; x += CELL) {
    if (!PS.pointInPoly({ x, y }, S.boundary)) continue;
    const u = (x - bb.minX) / dx, v = (y - bb.minY) / dy;
    const g = (SW * (1 - u) + SE * u) * (1 - v) + (NW * (1 - u) + NE * u) * v;   // bilinear ground
    const d = g - pad; if (d > 0) cut += d * cellA; else fill += -d * cellA;
    gSum += g * cellA; gA += cellA; cells.push({ x, y, d });
  }
  const cutY = cut / 27, fillY = fill / 27, net = cutY - fillY, balance = gA ? gSum / gA : 0;   // cubic yards
  S._ewBalance = balance;
  const cost = cutY * cutC + fillY * fillC + Math.abs(net) * haulC;
  const fmt = n => Math.round(n).toLocaleString();
  out.innerHTML = `挖 <b>${fmt(cutY)}</b> yd³ ・ 填 <b>${fmt(fillY)}</b> yd³ ・ ${net >= 0 ? '外運 export' : '進土 import'} <b>${fmt(Math.abs(net))}</b> yd³<br>土方成本約 <b>$${fmt(cost)}</b> ・ 平衡整平高 ≈ <b>${balance.toFixed(1)} ft</b>（挖填相抵、免外運）`;
  S._ewCache = { cells, max: Math.max(1, ...cells.map(c => Math.abs(c.d))), cell: CELL };
  draw();
}
function drawEarthwork() {                       // cut = warm/red (above pad), fill = cool/blue (below pad)
  if (!S.layers.earthwork || !S.layers.earthwork.vis || S.is3d || S.mapMode || !S._ewCache) return;
  const f = S._ewCache; if (!f.cells.length) return;
  const a = toScreen({ x: 0, y: 0 }), b = toScreen({ x: 1, y: 0 }), px = Math.max(2, Math.hypot(b.x - a.x, b.y - a.y) * f.cell);
  ctx.save(); pathPoly(S.boundary, true); ctx.clip();
  for (const c of f.cells) { const s = toScreen(c), t = Math.min(1, Math.abs(c.d) / f.max);
    ctx.fillStyle = c.d >= 0 ? `rgba(220,38,38,${(0.08 + 0.5 * t).toFixed(2)})` : `rgba(37,99,235,${(0.08 + 0.5 * t).toFixed(2)})`;
    ctx.fillRect(s.x - px / 2, s.y - px / 2, px + 1, px + 1); }
  ctx.restore();
}
function drawTrees() {
  if (!S.layers.trees.vis || S.is3d || S.boundary.length < 3) return;
  const trees = S._trees || (S._trees = treePositions());
  ctx.save();
  pathPoly(S.boundary, true); ctx.clip();                         // keep trees inside the site
  for (const tr of trees) {
    const p = toScreen(tr), e = toScreen({ x: tr.x + tr.r, y: tr.y });
    const r = Math.max(3, Math.hypot(e.x - p.x, e.y - p.y));
    ctx.fillStyle = 'rgba(0,0,0,.16)';
    ctx.beginPath(); ctx.ellipse(p.x + r * 0.28, p.y + r * 0.32, r * 0.95, r * 0.55, 0, 0, 7); ctx.fill();
    ctx.fillStyle = '#3c7a3a';
    for (const o of [[0, 0, 1], [-0.55, -0.25, 0.7], [0.55, -0.25, 0.7], [0, -0.55, 0.72]]) {
      ctx.beginPath(); ctx.arc(p.x + o[0] * r, p.y + o[1] * r, r * o[2], 0, 7); ctx.fill();
    }
    ctx.fillStyle = '#57a657';
    ctx.beginPath(); ctx.arc(p.x - r * 0.2, p.y - r * 0.25, r * 0.5, 0, 7); ctx.fill();
  }
  ctx.restore();
}

/* ------------------------------ compass / north -------------------------- */
function drawCompass() {
  if (S.is3d || !cv._w) return;
  const x = cv._w - 46, y = 52, r = 19;
  ctx.save();
  ctx.fillStyle = 'rgba(15,23,42,.85)'; ctx.strokeStyle = 'rgba(148,163,184,.5)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); ctx.stroke();
  // 8-point rose ticks
  ctx.strokeStyle = 'rgba(148,163,184,.45)';
  for (let i = 0; i < 8; i++) { const ang = i * Math.PI / 4; ctx.beginPath(); ctx.moveTo(x + Math.sin(ang) * r * 0.55, y - Math.cos(ang) * r * 0.55); ctx.lineTo(x + Math.sin(ang) * r * 0.9, y - Math.cos(ang) * r * 0.9); ctx.stroke(); }
  // N/S needle (diamond star)
  ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.moveTo(x, y - r + 3); ctx.lineTo(x - 4, y); ctx.lineTo(x + 4, y); ctx.closePath(); ctx.fill();
  ctx.fillStyle = 'rgba(203,213,225,.75)'; ctx.beginPath(); ctx.moveTo(x, y + r - 3); ctx.lineTo(x - 4, y); ctx.lineTo(x + 4, y); ctx.closePath(); ctx.fill();
  ctx.fillStyle = '#fff'; ctx.font = 'bold 8px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText('N', x, y - r + 6.5);
  ctx.fillStyle = 'rgba(148,163,184,.8)';
  ctx.fillText('S', x, y + r - 6.5); ctx.fillText('E', x + r - 5.5, y); ctx.fillText('W', x - r + 5.5, y);
  ctx.restore();
}

/* ------------------------------ tape measure ----------------------------- */
function drawMeasures() {
  if (S.is3d) return;
  const all = S.measures.slice();
  if (S.tool === 'measure' && S.measureStart && S.hoverWorld) all.push({ a: S.measureStart, b: S.hoverWorld, live: true });
  for (const m of all) {
    const a = toScreen(m.a), b = toScreen(m.b);
    ctx.save();
    ctx.strokeStyle = m.live ? 'rgba(56,189,248,.7)' : '#38bdf8'; ctx.lineWidth = 1.5;
    ctx.setLineDash(m.live ? [5, 4] : []);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    ctx.setLineDash([]);
    for (const p of [a, b]) { ctx.beginPath(); ctx.arc(p.x, p.y, 3, 0, 7); ctx.fillStyle = '#38bdf8'; ctx.fill(); }
    const dist = Math.hypot(m.b.x - m.a.x, m.b.y - m.a.y);
    const txt = `${U.L(dist).toFixed(1)} ${U.lu()}`;
    const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
    ctx.font = 'bold 11px system-ui'; const tw = ctx.measureText(txt).width + 10;
    ctx.fillStyle = 'rgba(15,23,42,.92)'; ctx.fillRect(mx - tw / 2, my - 9, tw, 18);
    ctx.strokeStyle = 'rgba(56,189,248,.5)'; ctx.lineWidth = 1; ctx.strokeRect(mx - tw / 2, my - 9, tw, 18);
    ctx.fillStyle = '#e2e8f0'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(txt, mx, my);
    ctx.restore();
  }
}

/* per-edge setback overrides (site mode) — highlight overridden / selected edges + label */
function drawEdgeSetbacks() {
  if (S.mode !== 'site' || S.is3d || S.boundary.length < 3) return;
  for (let i = 0; i < S.boundary.length; i++) {
    const overridden = S.edgeSetback[i] != null, sel = S.selEdge === i;
    if (!overridden && !sel) continue;
    const A = toScreen(S.boundary[i]), B = toScreen(S.boundary[(i + 1) % S.boundary.length]);
    ctx.save();
    ctx.strokeStyle = sel ? '#fff' : '#f59e0b'; ctx.lineWidth = sel ? 3.5 : 2.5;
    ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y); ctx.stroke();
    if (overridden) {
      const mx = (A.x + B.x) / 2, my = (A.y + B.y) / 2, txt = round2(U.L(S.edgeSetback[i])) + U.lu();
      ctx.font = 'bold 10px system-ui'; const tw = ctx.measureText(txt).width + 8;
      ctx.fillStyle = 'rgba(245,158,11,.95)'; ctx.fillRect(mx - tw / 2, my - 8, tw, 15);
      ctx.fillStyle = '#1a1205'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(txt, mx, my);
    }
    ctx.restore();
  }
}

/* ----------------------- validation / errors panel ----------------------- */
function computeErrors() {
  const E = [];
  const area = S.boundary.length >= 3 ? PS.polyArea(S.boundary) : 0;
  if (!area) { E.push({ lv: 'warn', m: '尚未畫出基地邊界' }); return E; }
  if (S.mode === 'site') {
    if (!S.site) E.push({ lv: 'warn', m: '尚未配置建案（按「自動配置建案」）' });
    else {
      S.site.compliance.filter(c => !c.ok).forEach(c => E.push({ lv: 'error', m: `法規不符：${c.k} = ${c.val}` }));
      if (S.site.densityCapped) E.push({ lv: 'warn', m: '戶數受密度上限（DU/ac）限制' });
    }
    const sum = +$('#uxStudioP').value + +$('#ux1P').value + +$('#ux2P').value + +$('#ux3P').value;
    if (Math.abs(sum - 100) > 0.5) E.push({ lv: 'warn', m: `單元配比合計 ${sum}%（非 100%）` });
  } else {
    const sol = S.solution;
    if (!sol) E.push({ lv: 'warn', m: '尚未排車位（按「自動排車位」）' });
    else {
      if (sol.unreachable) E.push({ lv: 'error', m: `${sol.unreachable} 個車位無車道可達（已移除，建議補出入口）` });
      if (!S.entrances.length) E.push({ lv: 'warn', m: '尚未設出入口（動線無進出點）' });
      if (S.opts.target > 0 && sol.stalls.length < S.opts.target) E.push({ lv: 'error', m: `未達目標車位：${sol.stalls.length} / ${S.opts.target}（差 ${S.opts.target - sol.stalls.length}）` });
    }
  }
  return E;
}
function showErrors() {
  const E = computeErrors();
  const html = E.length
    ? '<div style="display:flex;flex-direction:column;gap:8px;">' + E.map(e =>
      `<div class="cc"><span class="badge ${e.lv === 'error' ? 'no' : ''}" style="${e.lv === 'warn' ? 'background:var(--warn)' : ''}">${e.lv === 'error' ? '✕' : '!'}</span><span class="nm">${esc(e.m)}</span></div>`).join('') + '</div>'
    : '<div class="cc"><span class="badge ok">✓</span><span class="nm">沒有問題，方案通過所有檢查。</span></div>';
  const disc = '<div style="margin-top:12px;padding:9px 11px;border:1px solid var(--line);border-radius:7px;font-size:11px;color:var(--muted);line-height:1.7;">'
    + '⚠️ <b>可行性估算工具</b>：本工具<b>尚未檢核</b>消防車道（淨寬/迴轉/到建築距離）、死巷迴轉空間（hammerhead）、閘口排隊堆疊長度、無障礙通道幾何、坡道/淨高、機車位。'
    + '輸出供量體與車位數估算，<b>不可作為施工圖或送審依據</b>，實際設計仍須依當地法規與專業技師檢核。</div>';
  openModal(`檢查 Errors — ${E.length} 項`, html + disc);
}

/* --------------------- TestFit-style tabulation bar ---------------------- */
function updateTabBar() {
  const bar = $('#tabBar');
  const area = S.boundary.length >= 3 ? PS.polyArea(S.boundary) : 0;
  if (!area) { bar.classList.remove('show'); bar.innerHTML = ''; return; }
  const park = S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution;
  const stalls = park ? park.stalls.length : 0;
  const moduleArea = S.params.stallW * (S.params.stallD + S.params.aisle / 2);
  const cover = stalls ? Math.round(stalls * moduleArea / area * 100) : 0;
  const col = (title, kvs) => `<div class="col"><h5>${title}</h5>${kvs.map(k =>
    `<div class="kv"><span class="k">${k[0]}</span><span class="v">${k[1]}</span></div>`).join('')}</div>`;
  const siteKv = [['面積', `${U.big(area).toFixed(2)} ${U.bu()}`], ['', `${Math.round(U.A(area)).toLocaleString()} ${U.au()}`]];
  if (S.mode === 'site' && S.site) siteKv.push(['FAR', S.site.far.toFixed(2)], ['建蔽', S.site.coverage.toFixed(0) + '%']);
  let html = col('SITE 基地', siteKv);
  html += col('PARKING 停車', [['車位', stalls.toLocaleString()],
    ['單格', `${Math.round(U.A(moduleArea))} ${U.au()}`], ['覆蓋', cover + '%']]);
  if (S.mode !== 'site' && park) {            // parking mode — stall-type & target breakdown
    const cc = { standard:0, compact:0, ada:0, ev:0, trailer:0 };
    park.stalls.forEach(s => cc[s.type]++);
    const tg = S.opts.target || 0;
    html += col('STALLS 車種', [['標準', cc.standard], ['⚡EV', cc.ev], ['♿ADA', cc.ada],
      tg ? ['🎯需求', `${stalls}/${tg}`] : ['小型', cc.compact]]);
  }
  if (S.mode === 'site' && S.site) {
    const s = S.site;
    html += col('BUILDING 建築', [['GFA', Math.round(U.A(s.gfa)).toLocaleString()],
      [s.residential ? '戶數' : 'NRSF', s.residential ? s.units : Math.round(U.A(s.nrsf)).toLocaleString()],
      ['停車', `${s.parkingProvided}/${s.parkingRequired}`], ['Yield', s.fin.yieldOnCost.toFixed(1) + '%']]);
    if (s.residential) {                      // unit-level tabulation: DU/AC · Beds · Baths
      let beds = 0, baths = 0;
      s.unitsByType.forEach(u => { beds += (BEDS[u.type] || 0) * u.count; baths += (BATHS[u.type] || 0) * u.count; });
      html += col('UNITS 戶量', [['DU/ac', (s.units / s.acres).toFixed(1)],
        ['🛏 Beds', beds.toLocaleString()], ['🛁 Baths', baths.toLocaleString()]]);
    }
  }
  const errs = computeErrors();
  const hasErr = errs.some(e => e.lv === 'error');
  const errColor = hasErr ? '#f87171' : errs.length ? '#fbbf24' : '#4ade80';
  html += `<div class="col" id="tabErr" style="cursor:pointer;"><h5>${errs.length ? '⚠' : '✅'} 檢查 Errors</h5>
    <div class="kv"><span class="k">問題</span><span class="v" style="color:${errColor}">${errs.length || 'OK'}</span></div>
    ${errs.length ? `<div class="kv" style="font-size:10px;color:var(--muted);">${esc(errs[0].m.slice(0, 14))}…</div>` : '<div class="kv" style="font-size:10px;color:var(--muted);">點看詳情</div>'}</div>`;
  bar.innerHTML = html; bar.classList.add('show');
  const te = $('#tabErr'); if (te) te.onclick = showErrors;
  if ($('#objTree') && $('#objTree').classList.contains('show')) buildObjTree();
}

/* ------------------------- 3D isometric massing --------------------------- */
const ISO_A = Math.PI / 6;                  // 30° isometric
function iso(wx, wy, wz) {
  const x = wx - S.iso.cx, y = wy - S.iso.cy;
  const ix = (x - y) * Math.cos(ISO_A);
  const iy = (x + y) * Math.sin(ISO_A) - (wz || 0);
  return { x: ix * S.view.scale + S.view.ox, y: iy * S.view.scale + S.view.oy };
}
function isoFlat(wx, wy, wz) {               // un-scaled, un-offset (for fitting)
  const x = wx - S.iso.cx, y = wy - S.iso.cy;
  return { x: (x - y) * Math.cos(ISO_A), y: (x + y) * Math.sin(ISO_A) - (wz || 0) };
}
function fit3D() {
  if (!cv._w || !cv._h || S.boundary.length < 3) return;
  const c = PS.centroid(S.boundary); S.iso.cx = c.x; S.iso.cy = c.y;
  const pts = [];
  const add = (poly, z) => poly.forEach(p => pts.push(isoFlat(p.x, p.y, z)));
  add(S.boundary, 0);
  S.buildings.forEach(b => { add(b.poly, 0); add(b.poly, bHeight(b)); });
  S.obstacles.forEach(o => add(o, 0));
  if (S.mode === 'site' && S.site && S.site.footprint && S.site.footprint.length >= 3) {
    const fp = S.site.footprint; let top = S.site.height, bot = 0;
    if (S.site.structured && S.site.garage) {                     // frame the whole stack: tower-on-podium + basements
      const g = S.site.garage, fh = g.floorHeight || 11;
      top = g.levelsAbove * fh + S.site.height; bot = -g.levelsBelow * fh;
    }
    add(fp, top); add(fp, bot);
  }
  if (!pts.length) return;
  const xs = pts.map(p => p.x), ys = pts.map(p => p.y);
  const bb = { minX: Math.min(...xs), maxX: Math.max(...xs), minY: Math.min(...ys), maxY: Math.max(...ys) };
  const pad = 70, w = Math.max(bb.maxX - bb.minX, 1), h = Math.max(bb.maxY - bb.minY, 1);
  const scale = Math.min((cv._w - pad * 2) / w, (cv._h - pad * 2) / h);
  S.view.scale = scale;
  S.view.ox = pad + (cv._w - pad * 2 - w * scale) / 2 - bb.minX * scale;
  S.view.oy = pad + (cv._h - pad * 2 - h * scale) / 2 - bb.minY * scale;
}

function draw3D() {
  ctx.clearRect(0, 0, cv._w, cv._h);
  hideEntPopup(); hideEdgePopup();      // popups are 2D-only
  if (S.boundary.length < 3) {
    ctx.fillStyle = '#64748b'; ctx.font = '14px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('切到 2D 畫出基地後，這裡會顯示 3D 量體', cv._w / 2, cv._h / 2);
    return;
  }
  const c = PS.centroid(S.boundary); S.iso.cx = c.x; S.iso.cy = c.y;

  // ground plate
  ctx.beginPath();
  S.boundary.forEach((p, i) => { const s = iso(p.x, p.y, 0); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
  ctx.closePath();
  const hasBasement = S.mode === 'site' && S.site && S.site.structured && S.site.garage && S.site.garage.levelsBelow > 0;
  ctx.fillStyle = hasBasement ? 'rgba(27,42,61,.45)' : 'rgba(27,42,61,.95)'; ctx.fill();   // see basements through grade
  ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 1.5; ctx.stroke();

  // collect all faces, depth-sort (painter's algorithm)
  const faces = [];
  const baseDepth = poly => Math.max(...poly.map(p => p.x + p.y));
  const structured = S.mode === 'site' && S.site && (S.site.structured || S.site.isWrap) && S.site.garage && S.site.parkSol;
  const park3d = S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution;
  if (structured) {
    // MULTI-LEVEL GARAGE: stack the typical deck at every level — podium decks up from grade,
    // basement decks below it — plus the real express ramp on each, and columns as posts.
    const g = S.site.garage, fh = g.floorHeight || 11, deck = park3d, foot = g.deckPoly || S.site.footprint;
    const zs = [];
    for (let k = 0; k < g.levelsAbove; k++) zs.push(k * fh);
    for (let k = 1; k <= g.levelsBelow; k++) zs.push(-k * fh);
    for (const z of zs) {
      faces.push({ pts: foot.map(p => ({ x: p.x, y: p.y, z })), fill: z < 0 ? 'rgba(51,65,85,.34)' : 'rgba(100,116,139,.26)',
        stroke: 'rgba(148,163,184,.55)', depth: baseDepthArr(foot) - 0.6, z: z - 0.05 });   // floor plate
      for (const s of deck.stalls) {
        const col = COLORS[s.type] || COLORS.standard;
        faces.push({ pts: s.poly.map(p => ({ x: p.x, y: p.y, z })), fill: hexA(col, .8), stroke: hexA(col, .95), depth: baseDepth(s.poly), z: z + 0.02 });
      }
      if (g.ramp) faces.push({ pts: g.ramp.map(p => ({ x: p.x, y: p.y, z })), fill: 'rgba(245,158,11,.62)', stroke: 'rgba(217,119,6,1)', depth: baseDepth(g.ramp) + 0.2, z: z + 0.06 });
    }
    const colBot = -g.levelsBelow * fh, colSpan = (g.levelsAbove + g.levelsBelow) * fh || fh, cs = 1.3;
    for (const cp of g.columns) {                                         // structural posts through the stack
      const sq = [{ x: cp.x - cs, y: cp.y - cs }, { x: cp.x + cs, y: cp.y - cs }, { x: cp.x + cs, y: cp.y + cs }, { x: cp.x - cs, y: cp.y + cs }];
      pushBox(faces, sq, colSpan, '#334155', null, false, null, colBot);
    }
  } else if (park3d) {
    for (const a of park3d.aisles)
      faces.push({ pts: a.poly.map(p => ({ x: p.x, y: p.y, z: 0 })), fill: 'rgba(203,213,225,.10)', stroke: null, depth: baseDepth(a.poly), z: 0 });
    if (park3d.connectors) for (const cn of park3d.connectors) {     // drive roads: entrance spine = green, cross-aisle = orange (mirror 2D)
      const r = cn.type ? { f: 'rgba(34,197,94,.82)', s: 'rgba(74,222,128,1)' } : { f: 'rgba(245,158,11,.82)', s: 'rgba(251,191,36,1)' };
      faces.push({ pts: cn.poly.map(p => ({ x: p.x, y: p.y, z: 0 })), fill: r.f, stroke: r.s, depth: baseDepth(cn.poly), z: 0.04 });
    }
    for (const s of park3d.stalls) {
      const col = COLORS[s.type] || COLORS.standard;
      faces.push({ pts: s.poly.map(p => ({ x: p.x, y: p.y, z: 0 })), fill: hexA(col, .9), stroke: hexA(col, 1), depth: baseDepth(s.poly), z: 0 });
    }
  }
  (S.roads || []).forEach(r => faces.push({ pts: r.map(p => ({ x: p.x, y: p.y, z: 0.04 })), fill: 'rgba(51,61,79,.85)', stroke: 'rgba(148,163,184,.6)', depth: baseDepthArr(r), z: 0.04 }));
  (S.contextBuildings || []).forEach(cb => pushBox(faces, cb.poly, cb.height, '#94a3b8', null, true, null, 0, 0.5));   // OSM surrounding context
  S.obstacles.forEach(o => pushBox(faces, o, 3, '#7f1d1d'));
  S.buildings.forEach(b => pushBox(faces, b.poly, bHeight(b), b.color || '#64748b', 'BUILDING', b.roof !== false, b.voids));
  // site-mode residential massing. TOWNHOME SUBDIVISION = many small unit blocks + drives;
  // everything else = one massing block (on top of the above-grade parking podium, if any).
  if (S.mode === 'site' && S.site) {
    const sub = S.site.subdivision;
    if (sub && sub.units && sub.units.length) {
      (sub.drives || []).forEach(d => faces.push({ pts: d.map(p => ({ x: p.x, y: p.y, z: 0.02 })), fill: 'rgba(148,163,184,.28)', stroke: 'rgba(148,163,184,.5)', depth: baseDepthArr(d), z: 0.02 }));
      const uh = Math.max(S.site.height, 20);
      if (sub.subType && sub.subType !== 'townhome') sub.units.forEach(lot => faces.push({ pts: lot.map(p => ({ x: p.x, y: p.y, z: 0.015 })), fill: 'rgba(56,189,248,.10)', stroke: 'rgba(14,165,233,.5)', depth: baseDepthArr(lot), z: 0.015 }));  // lot lines
      (sub.houses || sub.units).forEach(h => pushBox(faces, h, uh, '#0ea5e9', null, true, null, 0));
    } else if (S.site.garden && S.site.garden.bars.length) {
      // GARDEN walk-up: each low-rise bar building as its own block (surface parking packs the bands between)
      S.site.garden.bars.forEach((b, i) => pushBox(faces, b, S.site.height, '#0ea5e9', i === 0 ? `${S.site.garden.rows}棟 ${S.site.floors}F` : null, true, null, 0));
    } else if (S.site.tower && S.site.tower.podium) {
      // TOWER: translucent parking podium (decks show through) with the slender point-tower rising above
      const tw = S.site.tower, fh = (S.site.garage && S.site.garage.floorHeight) || 11, podH = tw.podiumLevels * fh;
      pushBox(faces, tw.podium, podH, '#64748b', null, true, null, 0, 0.4);
      pushBox(faces, tw.plate, S.site.height, '#2563eb', `${S.site.floors}F`, true, null, podH);
    } else if (S.site.isWrap && S.site.wrapCore && S.site.footprint && S.site.footprint.length >= 3) {
      // WRAP: residential RING = footprint with the parking core punched out (core decks show through)
      pushBox(faces, S.site.footprint, S.site.height, '#0ea5e9', `${S.site.floors}F`, true, [S.site.wrapCore], 0, 0.45);
    } else if (S.site.industrial) {
      // WAREHOUSE: paved courts + trailer aprons on the ground, then the tall single-storey clear-span box
      const ind = S.site.industrial;
      ind.truckCourts.forEach(c => faces.push({ pts: c.map(p => ({ x: p.x, y: p.y, z: 0.02 })), fill: 'rgba(100,116,139,.30)', stroke: 'rgba(100,116,139,.55)', depth: baseDepthArr(c), z: 0.02 }));
      ind.trailerStalls.forEach(t => faces.push({ pts: t.map(p => ({ x: p.x, y: p.y, z: 0.03 })), fill: 'rgba(234,179,8,.22)', stroke: 'rgba(202,138,4,.7)', depth: baseDepthArr(t), z: 0.03 }));
      pushBox(faces, S.site.footprint, S.site.height, '#475569', `倉儲 ${ind.dockCount}門`, true, null, 0);
    } else if (S.site.retail) {
      // RETAIL: single-storey anchor strip + pad outparcels (surface lot packs the rest)
      const rt = S.site.retail, rh = Math.max(S.site.height, 16);
      pushBox(faces, rt.anchor, rh, '#2563eb', `零售 GLA`, true, null, 0);
      rt.pads.forEach(p => pushBox(faces, p, Math.max(rh * 0.85, 14), '#4f46e5', null, true, null, 0));
    } else if (S.site.datacenter) {
      // DATA CENTRE: tall data hall + ground mechanical yard + substation pad
      const dc = S.site.datacenter;
      faces.push({ pts: dc.mechYard.map(p => ({ x: p.x, y: p.y, z: 0.03 })), fill: 'rgba(245,158,11,.30)', stroke: 'rgba(217,119,6,.7)', depth: baseDepthArr(dc.mechYard), z: 0.03 });
      pushBox(faces, dc.subStation, 12, '#b91c1c', null, true, null, 0);
      pushBox(faces, dc.hall, S.site.height, '#334155', `機房 ${S.site.floors}F`, true, null, 0);
    } else if (S.site.footprint && S.site.footprint.length >= 3) {
      const podium = S.site.structured ? S.site.garage.levelsAbove * (S.site.garage.floorHeight || 11) : 0;
      pushBox(faces, S.site.footprint, S.site.height, '#0ea5e9', `${S.site.floors}F`, true, S.site.footVoids, podium, S.site.structured ? 0.4 : null);
    }
  }

  faces.sort((a, b) => (a.depth - b.depth) || (a.z - b.z));
  for (const f of faces) drawFace3D(f);

  // entrances as ground markers
  for (const e of S.entrances) {
    const s = iso(e.x, e.y, 0);
    ctx.beginPath(); ctx.arc(s.x, s.y, 6, 0, 7); ctx.fillStyle = '#22c55e'; ctx.fill();
    ctx.lineWidth = 1.5; ctx.strokeStyle = '#001018'; ctx.stroke();
  }
  // landscape trees in 3D — trunk + canopy billboards, back-to-front
  if (S.layers.trees.vis && S.boundary.length >= 3) {
    const trees = (S._trees || (S._trees = treePositions())).slice().sort((a, b) => (a.x + a.y) - (b.x + b.y));
    const TH = 16;                                   // tree height (ft)
    for (const tr of trees) {
      const g = iso(tr.x, tr.y, 0), top = iso(tr.x, tr.y, TH);
      const r = Math.max(3, tr.r * S.view.scale * 0.95);
      ctx.fillStyle = 'rgba(0,0,0,.18)';             // ground shadow
      ctx.beginPath(); ctx.ellipse(g.x, g.y, r * 0.9, r * 0.45, 0, 0, 7); ctx.fill();
      ctx.strokeStyle = '#5b3a1a'; ctx.lineWidth = Math.max(1.2, r * 0.18);   // trunk
      ctx.beginPath(); ctx.moveTo(g.x, g.y); ctx.lineTo(top.x, top.y); ctx.stroke();
      ctx.fillStyle = '#3c7a3a';                     // canopy
      ctx.beginPath(); ctx.arc(top.x, top.y, r, 0, 7); ctx.fill();
      ctx.fillStyle = '#57a657';
      ctx.beginPath(); ctx.arc(top.x - r * 0.28, top.y - r * 0.3, r * 0.55, 0, 7); ctx.fill();
    }
  }
  // hud
  ctx.fillStyle = '#64748b'; ctx.font = '11px system-ui'; ctx.textAlign = 'left';
  const hud3d = (S.mode === 'site' && S.site && S.site.structured)
    ? `3D 量體　🏢 結構車庫 地上${S.site.levelsAbove}+地下${S.site.levelsBelow}層 · 共 ${S.site.parkingProvided} 車位（每層 ${S.site.parkingPerFloor}）`
    : `3D 量體　建築高度 ${S.params.height} ft　·　${S.solution ? S.solution.stalls.length + ' 車位' : ''}`;
  ctx.fillText(hud3d, 14, cv._h - 28);
}

function pushBox(faces, foot, h, baseCol, label, roof, voids, zBase, alpha) {
  const z0 = zBase || 0, z1 = z0 + h;       // zBase lets a box start above grade (podium) or below (basement)
  const fa = c => (alpha != null ? hexA(c, alpha) : c);   // alpha → translucent "ghost" massing (see garage through it)
  for (let i = 0; i < foot.length; i++) {
    const p1 = foot[i], p2 = foot[(i + 1) % foot.length];
    const dx = p2.x - p1.x, dy = p2.y - p1.y;
    const wallCol = Math.abs(dx) > Math.abs(dy) ? shade(baseCol, -0.12) : shade(baseCol, -0.42);
    faces.push({
      pts: [{ x: p1.x, y: p1.y, z: z0 }, { x: p2.x, y: p2.y, z: z0 }, { x: p2.x, y: p2.y, z: z1 }, { x: p1.x, y: p1.y, z: z1 }],
      fill: fa(wallCol), stroke: shade(baseCol, -0.55), depth: Math.max(p1.x + p1.y, p2.x + p2.y), z: z0 + h * 0.5,
    });
  }
  if (roof !== false) faces.push({          // flat roof — skipped when roof is turned off (open massing)
    pts: foot.map(p => ({ x: p.x, y: p.y, z: z1 })),
    holes: (voids && voids.length) ? voids.map(v => v.map(p => ({ x: p.x, y: p.y, z: z1 }))) : null,   // courtyard cut-outs
    fill: fa(shade(baseCol, 0.18)), stroke: shade(baseCol, -0.15),
    depth: baseDepthArr(foot) + 0.5, z: z1 + 1, label: h > 12 ? label : null,
  });
}
function baseDepthArr(poly) { return Math.max(...poly.map(p => p.x + p.y)); }

function drawFace3D(f) {
  ctx.beginPath();
  f.pts.forEach((p, i) => { const s = iso(p.x, p.y, p.z); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
  ctx.closePath();
  if (f.holes) f.holes.forEach(h => {       // courtyard voids punched out of the roof face
    h.forEach((p, i) => { const s = iso(p.x, p.y, p.z); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
    ctx.closePath();
  });
  if (f.fill) { ctx.fillStyle = f.fill; ctx.fill(f.holes ? 'evenodd' : 'nonzero'); }
  if (f.stroke) { ctx.strokeStyle = f.stroke; ctx.lineWidth = 1; ctx.stroke(); }
  if (f.label) {
    let mx = 0, my = 0; f.pts.forEach(p => { const s = iso(p.x, p.y, p.z); mx += s.x; my += s.y; });
    ctx.fillStyle = '#e2e8f0'; ctx.font = '10px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(f.label, mx / f.pts.length, my / f.pts.length);
  }
}

/* --------------------------- interaction --------------------------------- */
function evtWorld(e) {
  const r = cv.getBoundingClientRect();
  return toWorld({ x: e.clientX - r.left, y: e.clientY - r.top });
}
function entranceAt(w) {                    // pick an entrance near world point w
  const m = toScreen(w);
  for (const e of S.entrances) {
    const p = toScreen(e);
    if (Math.hypot(p.x - m.x, p.y - m.y) < 13) return e;
  }
  return null;
}

cv.addEventListener('pointerdown', e => {
  const w = evtWorld(e);
  // map mode: still allow grabbing vertices/entrances in select; otherwise
  // empty-space drag pans the underlying real map.
  if (S.mapMode && e.button === 0) {
    if (S.tool === 'select') {
      if (pickable('entrance')) {
        const eh = entranceAt(w);
        if (eh) { S.dragEntrance = eh; S.selEntrance = eh; S.selStall = null; draw(); return; }
      }
      { const rv = roadVertexAt(w); if (rv) { S.dragRoad = rv; S.selRoad = rv.ri; showRoadPopup(rv.ri); draw(); return; } }
      if (pickable('site')) for (let i = 0; i < S.boundary.length; i++) {
        const sp = toScreen(S.boundary[i]), m = toScreen(w);
        if (Math.hypot(sp.x - m.x, sp.y - m.y) < 14) { S.dragVertex = { idx: i, reflow: (S.mode === 'site' ? !!S.site : !!S.solution) }; return; }
      }
      { const rb = roadBodyAt(w); if (rb) { S.dragRoad = { ri: rb.ri, body: true, last: w }; S.selRoad = rb.ri; showRoadPopup(rb.ri); draw(); return; } }
      { const bv = bldgVertexAt(w); if (bv) { S.dragBldgVtx = bv; S.selBuilding = S.buildings[bv.bi]; refreshBldgPanel(); draw(); return; } }
      if (pickable('building')) for (const b of S.buildings) if (PS.pointInPoly(w, b.poly)) { S.selBuilding = b; refreshBldgPanel(); draw(); return; }
    }
    if (S.tool === 'select' || S.tool === 'pan') {
      S.mapPanning = { x: e.clientX, y: e.clientY }; cv.style.cursor = 'grabbing'; e.preventDefault(); return;
    }
    // drawing tools (boundary/building/obstacle/entrance) fall through below
  }
  // pan: middle button, pan tool, space held, or anywhere in 3D (view-only)
  if (e.button === 1 || S.tool === 'pan' || S.spaceDown || (S.is3d && e.button === 0)) {
    S.panning = true; S.panStart = { x: e.clientX, y: e.clientY, ox: S.view.ox, oy: S.view.oy };
    cv.style.cursor = 'grabbing'; e.preventDefault(); return;
  }
  if (e.button === 2) return;  // right handled on contextmenu

  if (S.tool === 'boundary' || S.tool === 'building' || S.tool === 'obstacle' || S.tool === 'road' || S.tool === 'aisle' || S.tool === 'parkzone') {
    if (!S.draft) { S.draft = []; S.draftKind = S.tool; }
    // closed shapes snap to the first point to close; a road / aisle is an open polyline (double-click / Enter to finish)
    if (S.tool !== 'road' && S.tool !== 'aisle' && S.draft.length >= 3) {
      const f = toScreen(S.draft[0]), sp = toScreen(w);
      if (Math.hypot(f.x - sp.x, f.y - sp.y) < 12) { finishDraft(); return; }
    }
    S.draft.push(snap(w));
    draw(); return;
  }

  if (S.tool === 'entrance') {
    const g = S.boundary.length >= 3 ? projectToBoundary(S.boundary, w) : w;   // snap the gate onto the perimeter edge
    if (S.params.access === 'single') S.entrances = [{ x: g.x, y: g.y, type: 'inout' }];   // single-gate: replace
    else S.entrances.push({ x: g.x, y: g.y, type: 'inout' });
    toast(S.params.access === 'single' ? '單一出入口：已設定唯一閘口' : '已放出入口 — 選取可拖曳移動、雙擊改進/出、右鍵刪除');
    draw(); resolveActive();              // circulation responds to the new access point
    return;
  }

  if (S.tool === 'subdivide') {           // two clicks define a cut line
    const sp = snap(w);
    if (!S.splitPt) { S.splitPt = sp; toast('再點第二點，畫一條橫跨基地的切割線'); draw(); }
    else { const a = S.splitPt; S.splitPt = null; splitParcel(a, sp); }
    return;
  }

  if (S.tool === 'stall') {               // manually drop one parking stall at the click (override the auto layout)
    const sol = S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution;
    if (!sol || !sol.stalls) { toast('請先按「自動排車位」排一次'); return; }
    const th = (sol.theta != null ? sol.theta : 0), W = U.Lr(+$('#pW').value || 9), D = U.Lr(+$('#pD').value || 18);
    const run = { x: Math.cos(th), y: Math.sin(th) }, pd = { x: -Math.sin(th), y: Math.cos(th) };
    const corner = (sr, sd) => ({ x: w.x + run.x * sr + pd.x * sd, y: w.y + run.y * sr + pd.y * sd });
    sol.stalls.push({ poly: [corner(-W / 2, -D / 2), corner(W / 2, -D / 2), corner(W / 2, D / 2), corner(-W / 2, D / 2)], cx: w.x, cy: w.y, type: 'standard', manual: true });
    sol.count = sol.stalls.length;
    updateMetrics(); if (S.mode === 'site') updateSiteMetrics();
    draw(); commit();
    toast('已加一個車位（選取工具點它再按 Delete 可刪）');
    return;
  }

  if (S.tool === 'core') {                // manually place / remove a service core (stairs + elevators), site mode
    if (S.mode !== 'site') { toast('核心是建案模式用的（先切「建案規劃」）'); return; }
    const hit = (S.manualCores || []).findIndex(c => PS.pointInPoly(w, c));
    if (hit >= 0) { S.manualCores.splice(hit, 1); toast('已移除核心'); }
    else {
      const Wc = 30, Dc = 44, cc = (sx, sy) => ({ x: w.x + sx, y: w.y + sy });   // ~stairs + elevator bank (ft)
      S.manualCores.push([cc(-Wc / 2, -Dc / 2), cc(Wc / 2, -Dc / 2), cc(Wc / 2, Dc / 2), cc(-Wc / 2, Dc / 2)]);
      toast('已放一個服務核心（再點它可移除）');
    }
    draw(); commit();
    return;
  }

  if (S.tool === 'measure') {             // two clicks → a dimension annotation
    const sp = snap(w);
    if (!S.measureStart) { S.measureStart = sp; toast('再點第二點量出距離（右鍵清除全部）'); draw(); }
    else { S.measures.push({ a: S.measureStart, b: sp }); S.measureStart = null; draw(); }
    return;
  }

  if (S.tool === 'select') {
    // entrance grab (highest priority — small targets)
    if (pickable('entrance')) {
      const eh = entranceAt(w);
      if (eh) { S.dragEntrance = eh; S.selEntrance = eh; S.selStall = null; draw(); return; }
    }
    S.selEntrance = null;
    // ROAD EDIT — grab a centre-line vertex (drag the ends to extend/shorten, mid-points to reshape). Small target → before boundary.
    { const rv = roadVertexAt(w); if (rv) { S.dragRoad = rv; S.selRoad = rv.ri; S.selStall = null; showRoadPopup(rv.ri); draw(); return; } }
    // vertex grab on boundary (corners win — few + structural; the boundary now re-flows the parking on release)
    if (pickable('site')) for (let i = 0; i < S.boundary.length; i++) {
      const sp = toScreen(S.boundary[i]);
      const m = toScreen(w);
      if (Math.hypot(sp.x - m.x, sp.y - m.y) < 14) { S.dragVertex = { idx: i, reflow: (S.mode === 'site' ? !!S.site : !!S.solution) }; return; }
    }
    // SPINE NODE — grab a drive-aisle/connector end node (after boundary corners; spine nodes are mostly mid-edge)
    { const sn = spineNodeAt(w); if (sn) { S.dragSpine = sn; S.selAisle = sn.si; S.selStall = null; S.selRoad = null; showAislePopup(sn.si); draw(); return; } }
    // SPINE BODY — grab the whole aisle line to MOVE the lane (both ends + its stalls follow)
    { const sb = spineBodyAt(w); if (sb >= 0) { S.dragSpineBody = { si: sb, last: w }; S.selAisle = sb; S.selStall = null; S.selRoad = null; showAislePopup(sb); draw(); return; } }
    // BOUNDARY EDGE — drag a whole edge to stretch the site (a click without a drag = the site-mode setback popup)
    { const be = boundaryEdgeAt(w); if (be >= 0) { S.dragBoundaryEdge = { i: be, last: w, moved: false, reflow: (S.mode === 'site' ? !!S.site : !!S.solution) }; return; } }
    // ROAD EDIT — grab the road body to move the whole road (also selects it for the width / delete popup).
    { const rb = roadBodyAt(w); if (rb) { S.dragRoad = { ri: rb.ri, body: true, last: w }; S.selRoad = rb.ri; S.selStall = null; showRoadPopup(rb.ri); draw(); return; } }
    // MASSING — grab a building footprint corner to reshape it (parking re-packs on release). Small target → before edge/body.
    { const bv = bldgVertexAt(w); if (bv) { S.dragBldgVtx = bv; S.selBuilding = S.buildings[bv.bi]; S.selStall = null; refreshBldgPanel(); draw(); return; } }
    // MASSING — grab a building EDGE to move that whole side
    { const be = bldgEdgeAt(w); if (be) { S.dragBldgEdge = { bi: be.bi, ei: be.ei, last: w }; S.selBuilding = S.buildings[be.bi]; S.selStall = null; refreshBldgPanel(); draw(); return; } }
    // grab a building BODY to MOVE the whole building (a click without a drag = just select for the massing panel)
    if (pickable('building')) {
      for (const b of S.buildings) if (PS.pointInPoly(w, b.poly)) { S.dragBldgBody = { bi: S.buildings.indexOf(b), last: w, moved: false }; S.selBuilding = b; S.selStall = null; refreshBldgPanel(); draw(); return; }
    }
    // OBSTACLE — corner / edge / whole-move (parking re-packs around it on release)
    { const ov = obstacleVertexAt(w); if (ov) { S.dragObsVtx = ov; S.selObstacle = ov.oi; S.selStall = null; draw(); return; } }
    { const oe = obstacleEdgeAt(w); if (oe) { S.dragObsEdge = { oi: oe.oi, ei: oe.ei, last: w }; S.selObstacle = oe.oi; S.selStall = null; draw(); return; } }
    if (pickable('obstacle')) { for (let oi = 0; oi < S.obstacles.length; oi++) if (PS.pointInPoly(w, S.obstacles[oi])) { S.dragObsBody = { oi, last: w, moved: false }; S.selObstacle = oi; S.selStall = null; draw(); return; } }
    // click stall to edit
    if (S.solution && pickable('parking')) {
      for (const s of S.solution.stalls) {
        if (PS.pointInPoly(w, s.poly)) { S.selStall = s; S.selAisle = null; hideAislePopup(); draw(); return; }
      }
    }
    // click a GREY drive aisle body to select it (remove / single-load via the popup)
    { const ai = aisleAt(w); if (ai >= 0) { S.selAisle = ai; S.selStall = null; S.selRoad = null; showAislePopup(ai); draw(); return; } }
    // clicking an ORANGE/GREEN connector (auto "glue" lanes, regenerated each solve) → explain why it's not directly editable
    { if (connectorAt(w) >= 0) { S.selAisle = null; hideAislePopup(); toast('場內道路：拖兩端的點可調整這條線，車位會跟著重貼'); draw(); return; } }
    S.selStall = null; S.selBuilding = null; S.selEdge = null; S.selRoad = null; S.selAisle = null; S.selObstacle = null; hideEdgePopup(); hideRoadPopup(); hideAislePopup(); draw();
  }
});

cv.addEventListener('pointermove', e => {
  const w = evtWorld(e);
  S.hoverWorld = ['boundary', 'building', 'obstacle', 'road', 'parkzone', 'subdivide', 'measure'].includes(S.tool) ? snap(w) : null;
  if ((S.tool === 'subdivide' && S.splitPt) || (S.tool === 'measure' && S.measureStart)) draw();
  $('#stCoord').textContent = `${w.x.toFixed(1)} , ${w.y.toFixed(1)} ft`;

  if (S.mapPanning) {
    const dx = e.clientX - S.mapPanning.x, dy = e.clientY - S.mapPanning.y;
    S.mapPanning = { x: e.clientX, y: e.clientY };
    if (S.map) S.map.panBy([-dx, -dy], { animate: false });   // 'move' event redraws
    return;
  }
  if (S.panning) {
    S.view.ox = S.panStart.ox + (e.clientX - S.panStart.x);
    S.view.oy = S.panStart.oy + (e.clientY - S.panStart.y);
    draw(); return;
  }
  if (S.dragEntrance) { const s = S.boundary.length >= 3 ? projectToBoundary(S.boundary, w) : snap(w); S.dragEntrance.x = s.x; S.dragEntrance.y = s.y; draw(); return; }
  if (S.dragVertex) {
    S.boundary[S.dragVertex.idx] = snap(w);
    S.solution = null; S.site = null;
    S.mode === 'site' ? updateSiteMetrics() : updateMetrics();
    draw(); return;
  }
  if (S.dragRoad) {                                  // live-drag a road centre-line vertex, or move the whole road
    const rl = S.roadLines[S.dragRoad.ri];
    if (rl) {
      if (S.dragRoad.body) { const d = { x: w.x - S.dragRoad.last.x, y: w.y - S.dragRoad.last.y }; rl.line = rl.line.map(p => ({ x: p.x + d.x, y: p.y + d.y })); S.dragRoad.last = w; }
      else rl.line[S.dragRoad.pi] = snap(w);
      rebuildRoadStrips(); draw();
    }
    return;
  }
  if (S.dragBldgVtx) {                               // live-drag a building footprint corner; parking re-packs on release
    const b = S.buildings[S.dragBldgVtx.bi];
    if (b) { b.poly[S.dragBldgVtx.pi] = snap(w); updateMassInfo(b); draw(); }
    return;
  }
  if (S.dragSpine) {                                 // dragging a spine end node → reshape the lane + re-tile its stalls LIVE
    const park = activePark(), sp = park && park.spines && park.spines[S.dragSpine.si];
    if (sp) { sp.line[S.dragSpine.ni] = snap(w); retileSpine(S.dragSpine.si); (S.mode === 'site' ? updateSiteMetrics : updateMetrics)(); draw(); }
    return;
  }
  if (S.dragSpineBody) {                             // move a WHOLE drive aisle (translate the line; its stalls follow)
    const park = activePark(), sp = park && park.spines && park.spines[S.dragSpineBody.si];
    if (sp) { const d = { x: w.x - S.dragSpineBody.last.x, y: w.y - S.dragSpineBody.last.y }; sp.line = sp.line.map(p => ({ x: p.x + d.x, y: p.y + d.y })); S.dragSpineBody.last = w; retileSpine(S.dragSpineBody.si); (S.mode === 'site' ? updateSiteMetrics : updateMetrics)(); draw(); }
    return;
  }
  if (S.dragBoundaryEdge) {                          // stretch a boundary EDGE (move both its corners); parking re-flows on release
    const e = S.dragBoundaryEdge, n = S.boundary.length, d = { x: w.x - e.last.x, y: w.y - e.last.y };
    S.boundary[e.i] = { x: S.boundary[e.i].x + d.x, y: S.boundary[e.i].y + d.y };
    S.boundary[(e.i + 1) % n] = { x: S.boundary[(e.i + 1) % n].x + d.x, y: S.boundary[(e.i + 1) % n].y + d.y };
    e.last = w; e.moved = true; S.solution = null; S.site = null; (S.mode === 'site' ? updateSiteMetrics : updateMetrics)(); draw();
    return;
  }
  if (S.dragBldgEdge) {                              // move a building EDGE (both its corners)
    const b = S.buildings[S.dragBldgEdge.bi];
    if (b) { const poly = b.poly, n = poly.length, ei = S.dragBldgEdge.ei, d = { x: w.x - S.dragBldgEdge.last.x, y: w.y - S.dragBldgEdge.last.y }; poly[ei] = { x: poly[ei].x + d.x, y: poly[ei].y + d.y }; poly[(ei + 1) % n] = { x: poly[(ei + 1) % n].x + d.x, y: poly[(ei + 1) % n].y + d.y }; S.dragBldgEdge.last = w; updateMassInfo(b); draw(); }
    return;
  }
  if (S.dragBldgBody) {                              // move a WHOLE building (footprint + its courtyards)
    const b = S.buildings[S.dragBldgBody.bi];
    if (b) { const d = { x: w.x - S.dragBldgBody.last.x, y: w.y - S.dragBldgBody.last.y }; b.poly = b.poly.map(p => ({ x: p.x + d.x, y: p.y + d.y })); (b.voids || []).forEach(v => { for (let k = 0; k < v.length; k++) v[k] = { x: v[k].x + d.x, y: v[k].y + d.y }; }); S.dragBldgBody.last = w; S.dragBldgBody.moved = true; updateMassInfo(b); draw(); }
    return;
  }
  if (S.dragObsVtx) { const o = S.obstacles[S.dragObsVtx.oi]; if (o) { o[S.dragObsVtx.pi] = snap(w); draw(); } return; }
  if (S.dragObsEdge) { const o = S.obstacles[S.dragObsEdge.oi]; if (o) { const n = o.length, ei = S.dragObsEdge.ei, d = { x: w.x - S.dragObsEdge.last.x, y: w.y - S.dragObsEdge.last.y }; o[ei] = { x: o[ei].x + d.x, y: o[ei].y + d.y }; o[(ei + 1) % n] = { x: o[(ei + 1) % n].x + d.x, y: o[(ei + 1) % n].y + d.y }; S.dragObsEdge.last = w; draw(); } return; }
  if (S.dragObsBody) { const o = S.obstacles[S.dragObsBody.oi]; if (o) { const d = { x: w.x - S.dragObsBody.last.x, y: w.y - S.dragObsBody.last.y }; S.obstacles[S.dragObsBody.oi] = o.map(p => ({ x: p.x + d.x, y: p.y + d.y })); S.dragObsBody.last = w; S.dragObsBody.moved = true; draw(); } return; }
  if (S.draft) draw();
});

window.addEventListener('pointerup', () => {
  if (S.panning) { S.panning = false; cv.style.cursor = ''; }
  if (S.mapPanning) { S.mapPanning = null; cv.style.cursor = ''; }
  if (S.dragRoad) {                                  // road edit done → re-derive strips and re-pack around the new road
    const ri = S.dragRoad.ri; S.dragRoad = null; rebuildRoadStrips();
    const reSolve = (S.mode === 'site' ? !!S.site : !!S.solution);
    resolveActive();                                 // doSolve/doSolveSite commit the final state themselves
    if (!reSolve) commit();
    if (S.roadLines[ri]) showRoadPopup(ri); else hideRoadPopup();
    return;
  }
  if (S.dragSpine) {                                 // spine node released → re-tile cleanly, then re-link the drive network
    const si = S.dragSpine.si; S.dragSpine = null;
    reflowAfterEdit();   // re-tile every lane (the moved one + neighbours refill) + FULL re-knit so other roads come connect — adaptive move
    (S.mode === 'site' ? updateSiteMetrics : updateMetrics)();
    const park = activePark(); if (S.mode === 'site' && park && park.spines && park.spines[si]) showAislePopup(si); else { S.selAisle = null; hideAislePopup(); }
    draw(); commit();
    return;
  }
  if (S.dragSpineBody) {                              // whole-aisle move done → re-tile cleanly, then re-link the drive network
    const si = S.dragSpineBody.si; S.dragSpineBody = null; reflowAfterEdit();   // re-tile every lane (refill) + FULL re-knit (other roads come connect)
    (S.mode === 'site' ? updateSiteMetrics : updateMetrics)();
    const park = activePark(); if (S.mode === 'site' && park && park.spines && park.spines[si]) showAislePopup(si); else { S.selAisle = null; hideAislePopup(); }
    draw(); commit();
    return;
  }
  if (S.dragBoundaryEdge) {                           // boundary edge released → re-flow parking, or (no drag, site mode) setback popup
    const e = S.dragBoundaryEdge; S.dragBoundaryEdge = null;
    if (e.moved) { if (e.reflow && S.boundary.length >= 3) (S.mode === 'site' ? doSolveSite() : doSolve()); else commit(); }
    else if (S.mode === 'site' && S.boundary.length >= 3) { S.selEdge = e.i; S.selStall = null; S.selBuilding = null; showEdgePopup(e.i); draw(); }
    return;
  }
  if (S.dragBldgEdge) {                               // building edge moved → re-pack
    S.dragBldgEdge = null; const reSolve = (S.mode === 'site' ? !!S.site : !!S.solution); resolveActive(); if (!reSolve) { updateMetrics(); commit(); }
    return;
  }
  if (S.dragBldgBody) {                               // building moved → re-pack (a click without a move was just a select)
    const moved = S.dragBldgBody.moved; S.dragBldgBody = null;
    if (moved) { const reSolve = (S.mode === 'site' ? !!S.site : !!S.solution); resolveActive(); if (!reSolve) { updateMetrics(); commit(); } }
    return;
  }
  if (S.dragObsVtx || S.dragObsEdge || (S.dragObsBody && S.dragObsBody.moved)) {   // obstacle reshaped/moved → re-pack around it
    S.dragObsVtx = null; S.dragObsEdge = null; S.dragObsBody = null;
    const reSolve = (S.mode === 'site' ? !!S.site : !!S.solution); resolveActive(); if (!reSolve) { updateMetrics(); commit(); }
    return;
  }
  if (S.dragObsBody) { S.dragObsBody = null; return; }   // obstacle clicked without a move → just selected
  if (S.dragBldgVtx) {                               // footprint corner moved → re-pack parking around the new massing
    S.dragBldgVtx = null;
    const reSolve = (S.mode === 'site' ? !!S.site : !!S.solution);
    resolveActive();                                 // doSolve/doSolveSite commit the final state
    if (!reSolve) { updateMetrics(); commit(); }
    return;
  }
  const moved = S.dragVertex || S.dragEntrance;
  const entReSolve = !!S.dragEntrance && (S.mode === 'site' ? !!S.site : !!S.solution);
  let vtxReflow = false;
  if (S.dragVertex) { vtxReflow = S.dragVertex.reflow && S.boundary.length >= 3; S.dragVertex = null; if (vtxReflow) (S.mode === 'site' ? doSolveSite() : doSolve()); }   // boundary reshaped → re-flow the parking (like TestFit)
  if (S.dragEntrance) { S.dragEntrance = null; resolveActive(); }   // circulation follows the moved entrance
  if (moved && !entReSolve && !vtxReflow) commit();   // re-solve paths commit themselves
});

cv.addEventListener('contextmenu', e => {
  e.preventDefault();
  const w = evtWorld(e);
  if (S.tool === 'measure') { S.measures = []; S.measureStart = null; draw(); toast('已清除量測'); return; }
  if (S.draft) { finishDraft(); return; }
  if (S.tool === 'select' && pickable('entrance')) {
    const eh = entranceAt(w);
    if (eh) { S.entrances.splice(S.entrances.indexOf(eh), 1); S.selEntrance = null; draw(); toast('已刪除出入口'); resolveActive(); return; }
  }
  if (S.tool === 'select') {                       // right-click a 場外道路 → delete it
    const rh = roadVertexAt(w) || roadBodyAt(w);
    if (rh) { S.roadLines.splice(rh.ri, 1); S.selRoad = null; hideRoadPopup(); roadChanged(); toast('已刪除場外道路'); return; }
  }
  if (S.tool === 'select' && pickable('parking')) {   // right-click a 場內道路 → open its manual menu (單邊停/翻面/移除), TestFit-style
    const ai = aisleAt(w);
    if (ai >= 0) { S.selAisle = ai; S.selStall = null; hideRoadPopup(); showAislePopup(ai); draw(); return; }
  }
  if (S.tool === 'select' && S.solution && pickable('parking')) {
    // right click stall -> delete
    for (let i = 0; i < S.solution.stalls.length; i++) {
      if (PS.pointInPoly(w, S.solution.stalls[i].poly)) {
        S.solution.stalls.splice(i, 1); S.selStall = null; updateMetrics(); draw(); commit(); return;
      }
    }
  }
});

// click selected stall again cycles type (handled via dblclick for clarity)
cv.addEventListener('dblclick', e => {
  if ((S.draftKind === 'road' || S.draftKind === 'aisle') && S.draft && S.draft.length >= 2) { finishDraft(); return; }   // double-click ends an open road / aisle
  if (S.tool !== 'select') return;
  const w = evtWorld(e);
  const eh = pickable('entrance') ? entranceAt(w) : null;
  if (eh) {
    const o = ['inout', 'in', 'out'];
    eh.type = o[(o.indexOf(eh.type || 'inout') + 1) % 3];
    draw(); toast('出入口：' + (eh.type === 'in' ? '只進 ▸' : eh.type === 'out' ? '只出 ◂' : '進出 ⇄'));
    resolveActive();                 // rebuild the drive lane so its flow arrows match
    return;
  }
  if (!activePark() || !pickable('parking')) return;   // works in both modes — activePark() is S.solution (parking) or S.site.parkSol (site)
  // double-click a BEND node (mid node) → remove it (straighten); double-click the lane body → add a bend node you can drag
  const park = activePark();
  if (park && park.spines) {
    const sn = spineNodeAt(w);
    if (sn && park.spines[sn.si] && park.spines[sn.si].kind === 'aisle') {
      const ln = park.spines[sn.si].line;
      if (sn.ni > 0 && sn.ni < ln.length - 1) { ln.splice(sn.ni, 1); retileSpine(sn.si, true); reconnectNetwork(); S.selAisle = sn.si; (S.mode === 'site' ? updateSiteMetrics : updateMetrics)(); draw(); commit(); toast('已移除彎折點'); return; }
    }
    const sb = spineBodyAt(w);
    if (sb >= 0 && park.spines[sb] && park.spines[sb].kind === 'aisle' && insertSpineNode(sb, w)) {
      S.selAisle = sb; (S.mode === 'site' ? updateSiteMetrics : updateMetrics)(); draw(); commit(); toast('已加彎折點 — 拖動黃點可彎曲車道'); return;
    }
  }
  for (const s of park.stalls) {                       // park = activePark() above → stall-type cycle works in site mode too (no S.solution crash)
    if (PS.pointInPoly(w, s.poly)) {
      const order = ['standard', 'compact', 'ev', 'ada', 'trailer'];
      s.type = order[(order.indexOf(s.type) + 1) % order.length];
      (S.mode === 'site' ? updateSiteMetrics : updateMetrics)(); draw(); commit(); return;
    }
  }
});

cv.addEventListener('wheel', e => {
  e.preventDefault();
  const f = e.deltaY < 0 ? 1.12 : 1 / 1.12;
  if (S.mapMode && S.map) {            // zoom the real map around the cursor
    const r = cv.getBoundingClientRect();
    const z = Math.max(1, Math.min(21, S.map.getZoom() + (e.deltaY < 0 ? 1 : -1)));
    S.map.setZoomAround(L.point(e.clientX - r.left, e.clientY - r.top), z); return;
  }
  if (S.is3d) {                       // iso centres on the site centroid → pure scale
    S.view.scale = Math.max(0.02, Math.min(20, S.view.scale * f)); draw(); return;
  }
  const r = cv.getBoundingClientRect();
  const mx = e.clientX - r.left, my = e.clientY - r.top;
  const before = toWorld({ x: mx, y: my });
  S.view.scale = Math.max(0.02, Math.min(20, S.view.scale * f));
  const after = toScreen(before);
  S.view.ox += mx - after.x; S.view.oy += my - after.y;
  draw();
}, { passive: false });

function snap(w) {                   // 1-ft snap when zoomed in
  if (S.view.scale > 1.2) return { x: Math.round(w.x), y: Math.round(w.y) };
  return w;
}

function finishDraft() {
  const minPts = (S.draftKind === 'road' || S.draftKind === 'aisle') ? 2 : 3;
  if (!S.draft || S.draft.length < minPts) { S.draft = null; S.draftKind = null; draw(); return; }
  const poly = S.draft.slice(), kind = S.draftKind;
  let msg = '已新增';
  if (S.draftKind === 'aisle') { S.draft = null; S.draftKind = null; addManualAisle(poly); return; }   // user-drawn drive aisle → tile stalls + re-link the network (no full re-solve, keeps the rest of the lot)
  if (S.draftKind === 'boundary') { S.boundary = poly; S.solution = null; S.edgeSetback = {}; S.selEdge = null; msg = '基地完成，按「自動排車位」'; }
  else if (S.draftKind === 'building') {
    // a smaller shape drawn INSIDE an existing building becomes its void (courtyard)
    const host = S.buildings.find(b => PS.pointInPoly(PS.centroid(poly), b.poly) && PS.polyArea(poly) < PS.polyArea(b.poly));
    if (host) { (host.voids || (host.voids = [])).push(poly); S.selBuilding = host; msg = '已在建築上挖出中庭 Void'; }
    else {
      const nb = makeBuilding(poly); S.buildings.push(nb); S.selBuilding = nb;
      msg = (S.boundary.length >= 3 && !PS.polyOverlap(poly, S.boundary)) ? '⚠️ 建築畫在基地範圍外，對排版無影響' : '已新增建築（右側可改顏色/高度/屋頂）';
    }
  }
  else if (S.draftKind === 'obstacle') {
    S.obstacles.push(poly);
    if (S.boundary.length >= 3 && !PS.polyOverlap(poly, S.boundary)) msg = '⚠️ 障礙畫在基地範圍外，對排版無影響';
  }
  else if (S.draftKind === 'road') {
    const wdt = S.roadWidth || 24;
    S.roads.push(...bufferPolyline(poly, wdt));                  // strips for the solver (block buildings / clear stalls)
    S.roadLines.push({ line: poly, width: wdt });               // centre-line for a smooth continuous render
    msg = '已新增社區道路，格局自動繞開';
  }
  else if (S.draftKind === 'parkzone') {
    S.parkZones.push(poly);                                     // stalls now pack ONLY inside the drawn zones
    msg = '已新增停車區，車位只排在框內';
  }
  S.draft = null; S.draftKind = null;
  updateMetrics();
  if (S.mapMode) draw(); else fitView();
  // if this triggers a re-solve, let doSolve/doSolveSite commit the FINAL state (one clean undo step);
  // committing here too would record a stale intermediate (shape added, stalls not yet recomputed).
  const reSolving = (kind === 'obstacle' || kind === 'building' || kind === 'road' || kind === 'parkzone') && (S.mode === 'site' ? !!S.site : !!S.solution);
  if (kind === 'obstacle' || kind === 'building' || kind === 'road' || kind === 'parkzone') resolveActive();   // re-pack so stalls clear under the new shape
  if (!reSolving) commit();
  toast(msg);
}

/* ----------------------------- keyboard ---------------------------------- */
window.addEventListener('keydown', e => {
  if (e.key === 'Escape' && $('#modal').classList.contains('show')) { closeModal(); return; }
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if ((e.ctrlKey || e.metaKey) && (e.key === 'z' || e.key === 'Z')) { e.preventDefault(); e.shiftKey ? redo() : undo(); return; }
  if ((e.ctrlKey || e.metaKey) && (e.key === 'y' || e.key === 'Y')) { e.preventDefault(); redo(); return; }
  if (e.code === 'Space') { S.spaceDown = true; cv.style.cursor = 'grab'; }
  if (e.key === 'Enter') { if (S.draft) finishDraft(); else S.mode === 'site' ? doSolveSite() : doSolve(); }
  if (e.key === 'Escape') { S.draft = null; S.draftKind = null; S.selStall = null; S.measureStart = null; draw(); }
  if (e.key === 'Delete' || e.key === 'Backspace') {
    if (S.selEntrance) {
      S.entrances.splice(S.entrances.indexOf(S.selEntrance), 1); S.selEntrance = null; draw(); resolveActive();
    } else if (S.selRoad != null && S.roadLines[S.selRoad]) {
      S.roadLines.splice(S.selRoad, 1); S.selRoad = null; hideRoadPopup(); roadChanged(); toast('已刪除道路');
    } else if (S.selAisle != null) {
      removeAisle(S.selAisle);
    } else if (S.selObstacle != null && S.obstacles[S.selObstacle]) {
      S.obstacles.splice(S.selObstacle, 1); S.selObstacle = null; resolveActive(); draw(); toast('已刪除障礙');
    } else if (S.selStall && S.solution) {
      const i = S.solution.stalls.indexOf(S.selStall);
      if (i >= 0) { S.solution.stalls.splice(i, 1); S.selStall = null; updateMetrics(); draw(); commit(); }
    }
  }
  const map = { v:'select', b:'boundary', g:'building', o:'obstacle', r:'road', a:'aisle', z:'parkzone', k:'stall', c:'core', e:'entrance', d:'subdivide', m:'measure', h:'pan' };
  if (map[e.key]) setTool(map[e.key]);
});
window.addEventListener('keyup', e => { if (e.code === 'Space') { S.spaceDown = false; cv.style.cursor = ''; } });

/* ------------------------------- tools ----------------------------------- */
function setTool(t) {
  S.tool = t;
  document.querySelectorAll('.tool').forEach(b => b.classList.toggle('active', b.dataset.tool === t));
  const names = { select:'選取/編輯', boundary:'畫基地', building:'畫建築', obstacle:'畫障礙', road:'畫場外道路', aisle:'畫場內道路', parkzone:'畫停車區', stall:'加車位', core:'放核心', entrance:'放出入口', subdivide:'切割子地', measure:'量測距離', pan:'平移' };
  $('#stTool').textContent = '工具：' + (names[t] || t);
  cv.style.cursor = t === 'pan' ? 'grab' : t === 'select' ? 'default' : 'crosshair';
  if (t !== 'boundary' && t !== 'building' && t !== 'obstacle' && t !== 'road' && t !== 'aisle' && t !== 'parkzone') { S.draft = null; }
  if (t !== 'subdivide') S.splitPt = null;
  if (t !== 'measure') S.measureStart = null;
  if (t !== 'select') { S.selRoad = null; hideRoadPopup(); S.selAisle = null; hideAislePopup(); S.selObstacle = null; }
  draw();
}
document.querySelectorAll('.tool').forEach(b => b.addEventListener('click', () => setTool(b.dataset.tool)));

/* ------------------------------ solving ---------------------------------- */
function readParams() {
  S.params.stallW = U.Lr(+$('#pW').value);
  S.params.stallD = U.Lr(+$('#pD').value);
  S.params.aisle = U.Lr(+$('#pA').value);
  S.params.setback = U.Lr(+$('#pS').value);
  S.params.orient = $('#pOrient').value;
  S.params.height = U.Lr(+$('#pH').value);
  S.params.greenBuffer = U.Lr(+$('#pGreen').value);
  S.params.maxRun = document.querySelector('#maxRunSeg button.active').dataset.mr === '1' ? (+$('#pMaxRun').value) : 0;
  S.params.maxRunGap = U.Lr(+$('#pMaxGap').value);
  S.params.compactW = U.Lr(+$('#cW').value);
  const _ab = document.querySelector('#accessSeg button.active'); S.params.access = _ab ? _ab.dataset.acc : 'multi';
  S.opts.adaMode = $('#adaMode').value;
  S.opts.adaManual = +$('#adaManual').value;
  S.opts.evPct = +$('#pEV').value;
  S.opts.compactPct = +$('#pCompact').value;
  S.opts.motoPct = $('#pMoto') ? +$('#pMoto').value : 0;
  S.opts.gfa = U.Ar(+$('#pGFA').value);
  S.opts.target = +$('#pTarget').value;
}

function doSolve() {
  readParams();
  if (S.boundary.length < 3) { toast('請先畫出基地邊界'); setTool('boundary'); return; }
  $('#busy').classList.add('show');
  $('#busyTxt').textContent = '運算最佳配置中…';
  setTimeout(() => {
    const t0 = performance.now();
    const res = PS.solve({
      boundary: S.boundary, buildings: bPolys(), obstacles: S.obstacles, roads: S.roads, parkZones: S.parkZones, entrances: S.entrances,
      params: { ...S.params, gridShift: S.gridShift }, opts: S.opts,
    });
    S.solution = res; S.selStall = null;
    applyAisleEdits(S.solution);                  // re-apply manual aisle overrides (single-load / remove) to the fresh pack
    deriveSpines(S.solution);                     // every drive aisle becomes an editable spine (centre-line + nodes)
    $('#busy').classList.remove('show');
    updateMetrics(); draw(); commit();
    const ms = Math.round(performance.now() - t0);
    if (res) toast(`完成：${res.stalls.length} 車位　(角度 ${res.metrics.bestAngleDeg}°, ${ms}ms)`);
    $('#stMsg').textContent = res ? `已排 ${res.stalls.length} 車位` : '無法排版';
  }, 30);
}

// Re-run the active solver (used when entrances move so circulation updates).
function resolveActive() {
  if (S.mode === 'site') { if (S.site) doSolveSite(); }
  else if (S.solution) doSolve();
}

/* ------------------------------ metrics ---------------------------------- */
function updateMetrics() {
  S._trees = null; updateTabBar();          // trees + bottom tabulation follow the layout
  const sol = S.solution;
  const counts = { standard:0, compact:0, ada:0, ev:0, trailer:0 };
  if (sol) sol.stalls.forEach(s => counts[s.type]++);
  const total = sol ? sol.stalls.length : 0;

  $('#mTotal').textContent = total.toLocaleString();

  const siteArea = S.boundary.length >= 3 ? PS.polyArea(S.boundary) : 0;
  $('#mArea').textContent = siteArea ? U.big(siteArea).toFixed(2) : '—';
  $('#stArea').textContent = siteArea
    ? `面積：${Math.round(U.A(siteArea)).toLocaleString()} ${U.au()} (${U.big(siteArea).toFixed(2)} ${U.bu()})` : '面積：—';

  refreshBldgPanel();
  // building GFA: manual override else sum of massing GFA (footprint × floors, minus void courtyards)
  let gfa = S.opts.gfa;
  if (!gfa) gfa = S.buildings.reduce((s, b) => s + bGFA(b), 0);
  const ratio = U.metric() ? (gfa > 0 ? total / (U.A(gfa) / 100) : 0) : (gfa > 0 ? total / (gfa / 1000) : 0);
  $('#mRatio').textContent = (gfa > 0 && total) ? ratio.toFixed(1) : '—';

  // efficiency: parking footprint area per stall
  const moduleAreaPerStall = S.params.stallW * (S.params.stallD + S.params.aisle / 2);
  $('#mEff').textContent = total ? Math.round(U.A(moduleAreaPerStall)).toLocaleString() : '—';

  // coverage = parking area / site
  const parkArea = total * moduleAreaPerStall;
  $('#mCover').textContent = (siteArea && total) ? Math.round(parkArea / siteArea * 100) + '%' : '—';

  // PARKING DEMAND — the drawn massing (use × GFA) requires N spaces; show provided vs required
  const required = parkingDemand();
  $('#mRequired').textContent = required ? required.toLocaleString() : '—';
  const dBadge = $('#parkDemandBadge');
  if (required > 0) {
    const ok = total >= required, diff = total - required;
    dBadge.style.display = '';
    dBadge.className = ok ? 'pass' : 'fail';
    dBadge.textContent = ok ? `✓ 停車充足　${total} / 需 ${required}　(多 ${diff})` : `✗ 停車不足　${total} / 需 ${required}　(缺 ${-diff})`;
  } else dBadge.style.display = 'none';
  updateFinancials();

  $('#stOrient').textContent = sol ? `配置角度 ${sol.metrics.bestAngleDeg}°` : '';

  // breakdown
  const bd = $('#breakdown'); bd.innerHTML = '';
  let any = false;
  for (const k of ['standard','compact','ev','ada','moto','trailer']) {
    if (!counts[k]) continue; any = true;
    const row = document.createElement('div'); row.className = 'brow';
    row.innerHTML = `<span class="sw" style="background:${COLORS[k]}"></span>
      <span class="nm">${LABELS[k]}</span><span class="ct">${counts[k]}</span>`;
    bd.appendChild(row);
  }
  if (counts.ada) {
    const req = PS.adaRequired(total);
    const note = document.createElement('div');
    note.className = 'hint';
    note.textContent = `法規最低 ADA：${req}（含 van 無障礙 ${Math.max(1, Math.ceil(counts.ada/6))}）`;
    bd.appendChild(note);
  }
  if (sol && S.entrances.length) {
    const cn = document.createElement('div'); cn.className = 'hint'; cn.style.marginTop = '4px';
    cn.textContent = sol.unreachable
      ? `🚗 動線：${sol.unreachable} 個車位無車道可達，已移除（建議補出入口）`
      : `🚗 動線：${S.entrances.length} 個出入口，全部車位皆可達 ✓`;
    bd.appendChild(cn);
  }
  if (S.opts.target > 0 && sol) {
    const ok = total >= S.opts.target, d = document.createElement('div');
    d.id = 'targetBadge'; d.className = ok ? 'pass' : 'fail';
    d.textContent = ok ? `🎯 達成目標 ${total} / ${S.opts.target} ✓` : `🎯 未達目標：${total} / ${S.opts.target}（差 ${S.opts.target - total}）`;
    bd.appendChild(d);
  }
  if (!any) { bd.innerHTML = '<div class="hint">尚未排車位 — 畫好基地後按「⚡ 自動排車位」。</div>'; }
}

function buildLegend() {
  const L = $('#legend'); L.innerHTML = '';
  const items = [
    ['standard','標準'], ['compact','小型'], ['ev','EV'], ['ada','ADA'], ['moto','機車'], ['trailer','拖車'],
  ];
  for (const [k, n] of items) {
    const it = document.createElement('span'); it.className = 'it';
    it.innerHTML = `<span class="sw" style="background:${COLORS[k]}"></span>${n}`;
    L.appendChild(it);
  }
  const extra = document.createElement('span'); extra.className = 'it';
  extra.innerHTML = `<span class="sw" style="background:rgba(203,213,225,.4)"></span>場內道路`;
  L.appendChild(extra);
  // residential unit-fit swatches (only when a unit plan is on screen)
  if (S.mode === 'site' && S.site && S.site.unitPlan && S.layers.unitfit && S.layers.unitfit.vis) {
    for (const t of ['studio', '1br', '2br', '3br']) {
      if (!S.site.unitPlan.byType[t]) continue;
      const it = document.createElement('span'); it.className = 'it';
      it.innerHTML = `<span class="sw" style="background:${UNIT_FIT_COLORS[t]}"></span>${UNIT_FIT_LABEL[t]}`;
      L.appendChild(it);
    }
  }
}

/* ------------------------------ schemes ---------------------------------- */
const SK = 'parkingSolver.schemes';
function getSchemes() { try { return JSON.parse(localStorage.getItem(SK) || '[]'); } catch { return []; } }
function setSchemes(a) { localStorage.setItem(SK, JSON.stringify(a)); renderSchemes(); }

function renderSchemes() {
  const list = $('#schemeList'); list.innerHTML = '';
  const arr = getSchemes();
  if (!arr.length) { list.innerHTML = '<div class="hint">尚無儲存方案。</div>'; return; }
  arr.forEach((sc, i) => {
    const el = document.createElement('div'); el.className = 'scheme';
    el.innerHTML = `<div style="flex:1;overflow:hidden;">
        <div class="nm"></div>
        <div class="sub"></div>
      </div>
      <button class="mini" title="載入">↺</button>
      <button class="mini" title="刪除">✕</button>`;
    el.querySelector('.nm').textContent = (sc.mode === 'site' ? '🏢 ' : '🅿️ ') + sc.name;   // user input -> safe
    el.querySelector('.sub').textContent = `${sc.summary || (sc.total + ' 車位')} · ${sc.date}`;
    const [loadBtn, delBtn] = el.querySelectorAll('.mini');
    loadBtn.onclick = () => loadScheme(sc);
    delBtn.onclick = () => { const a = getSchemes(); a.splice(i, 1); setSchemes(a); };
    list.appendChild(el);
  });
}
function saveScheme() {
  readParams();
  const name = ($('#schemeName').value || '').trim() || `方案 ${getSchemes().length + 1}`;
  let summary;
  if (S.mode === 'site') {
    summary = S.site ? `${S.site.residential ? S.site.units + ' 戶' : Math.round(S.site.gfa).toLocaleString() + ' SF'} · ${S.site.floors}F` : '建案';
  } else {
    const stalls = S.solution ? S.solution.stalls.length : 0, req = parkingDemand(), fin = massingFinancials();
    summary = `${stalls} 車位${req ? `/需${req}` : ''} · ${S.params.angle}°${fin && fin.yieldOnCost ? ` · Yield ${fin.yieldOnCost.toFixed(1)}%` : ''}`;
  }
  const snap = {
    name, date: new Date().toISOString().slice(0, 10),
    mode: S.mode, summary, data: serialize(),
  };
  const a = getSchemes(); a.unshift(snap); setSchemes(a);
  $('#schemeName').value = '';
  toast('已儲存方案：' + name);
}
function loadScheme(sc) { deserialize(sc.data); toast('已載入：' + sc.name); }

/* --------------------------- serialize / IO ------------------------------ */
function getSiteForm() {
  const ids = ['sUse', 'sFloorH', 'sEff', 'zFAR', 'zHeight', 'zCov', 'zDUA', 'zSbF', 'zSbS', 'zSbR', 'zPark',
    'uxStudioP', 'ux1P', 'ux2P', 'ux3P', 'uxStudioS', 'ux1S', 'ux2S', 'ux3S', 'fLand', 'fHard', 'fSoft', 'fRentMo', 'fRentSf', 'fOpex', 'fGrowth', 'fHold', 'fExitCap'];
  const o = {}; ids.forEach(id => o[id] = $('#' + id).value); return o;
}
function setSiteForm(o) { if (!o) return; Object.keys(o).forEach(id => { const el = $('#' + id); if (el) el.value = o[id]; }); }

function serialize() {
  return {
    mode: S.mode,
    boundary: S.boundary, buildings: S.buildings, obstacles: S.obstacles, roads: S.roads, roadLines: S.roadLines, parkZones: S.parkZones, manualCores: S.manualCores,
    entrances: S.entrances, params: S.params, opts: S.opts,
    parcels: S.parcels, activeParcel: S.activeParcel, edgeSetback: S.edgeSetback,
    solution: S.solution ? { stalls: S.solution.stalls, aisles: S.solution.aisles, metrics: S.solution.metrics, theta: S.solution.theta,
      connectors: S.solution.connectors, accessAisles: S.solution.accessAisles, unreachable: S.solution.unreachable } : null,
    site: S.site, siteForm: getSiteForm(),
  };
}
function deserialize(d) {
  S.boundary = d.boundary || []; S.buildings = d.buildings || [];
  S.obstacles = d.obstacles || []; S.roads = d.roads || []; S.roadLines = d.roadLines || []; S.parkZones = d.parkZones || []; S.manualCores = d.manualCores || []; S.entrances = d.entrances || [];
  S.parcels = d.parcels || null; S.activeParcel = d.activeParcel || 0;
  S.edgeSetback = d.edgeSetback || {}; S.selEdge = null;
  normalizeBuildings();                              // wrap any legacy raw-array buildings into objects
  Object.assign(S.params, d.params || {}); Object.assign(S.opts, d.opts || {});
  S.solution = d.solution || null; S.site = d.site || null; S.selStall = null; S.selBuilding = null;
  syncInputs(); setSiteForm(d.siteForm); updateUxSum();
  setMode(d.mode || 'parking');                    // restores panels + metrics + draw
  if (S.mapMode) draw(); else fitView();
}

/* ----------------------------- undo / redo ------------------------------- */
// Snapshot the editable state on each structural change; ⌘Z / ⌘⇧Z step through.
function commit() {
  if (S._restoring) return;
  S._flowCache = null;                          // layout changed → recompute the circulation heat-map lazily
  const snap = JSON.parse(JSON.stringify(serialize()));
  if (S.hIdx >= 0 && JSON.stringify(S.history[S.hIdx]) === JSON.stringify(snap)) return;   // skip no-op duplicates
  S.history = S.history.slice(0, S.hIdx + 1);
  S.history.push(snap);
  if (S.history.length > 60) S.history.shift();
  S.hIdx = S.history.length - 1;
  updateUndoButtons();
}
function applyHistory() {
  S._restoring = true;
  deserialize(JSON.parse(JSON.stringify(S.history[S.hIdx])));
  S._restoring = false;
  updateUndoButtons();
}
function undo() { if (S.hIdx > 0) { S.hIdx--; applyHistory(); toast('↶ 已復原 Undo'); } }
function redo() { if (S.hIdx < S.history.length - 1) { S.hIdx++; applyHistory(); toast('↷ 已重做 Redo'); } }
function updateUndoButtons() {
  const u = $('#btnUndo'), r = $('#btnRedo'); if (!u || !r) return;
  u.disabled = S.hIdx <= 0; r.disabled = S.hIdx >= S.history.length - 1;
}
function syncInputs() {
  $('#pW').value = round2(U.L(S.params.stallW)); $('#pD').value = round2(U.L(S.params.stallD));
  $('#pA').value = round2(U.L(S.params.aisle)); $('#pS').value = round2(U.L(S.params.setback));
  $('#pOrient').value = S.params.orient; $('#pH').value = round2(U.L(S.params.height));
  $('#pAngleNum').value = S.params.angle;
  $('#pGreen').value = round2(U.L(S.params.greenBuffer || 0));
  $('#pMaxGap').value = round2(U.L(S.params.maxRunGap || 9));
  $('#pMaxRun').value = S.params.maxRun > 0 ? S.params.maxRun : 12;
  $('#cW').value = round2(U.L(S.params.compactW || 7.5));
  document.querySelectorAll('#accessSeg button').forEach(b => b.classList.toggle('active', b.dataset.acc === (S.params.access || 'multi')));
  document.querySelectorAll('#treesSeg button').forEach(b => b.classList.toggle('active', (b.dataset.tr === '1') === (S.layers.trees.vis !== false)));
  document.querySelectorAll('#maxRunSeg button').forEach(b => b.classList.toggle('active', (b.dataset.mr === '1') === (S.params.maxRun > 0)));
  $('#maxRunRow1').style.display = $('#maxRunRow2').style.display = S.params.maxRun > 0 ? '' : 'none';
  $('#adaMode').value = S.opts.adaMode; $('#adaManual').value = S.opts.adaManual;
  $('#pEV').value = S.opts.evPct; $('#pCompact').value = S.opts.compactPct;
  $('#pGFA').value = S.opts.gfa ? Math.round(U.A(S.opts.gfa)) : 0;
  $('#pTarget').value = S.opts.target || 0;
  document.querySelectorAll('#angleSeg button').forEach(b =>
    b.classList.toggle('active', +b.dataset.ang === S.params.angle));
  document.querySelectorAll('#onewaySeg button').forEach(b =>
    b.classList.toggle('active', (b.dataset.ow === '1') === !!S.params.oneway));
  $('#adaManualRow').style.display = S.opts.adaMode === 'manual' ? '' : 'none';
}

/* ------------------------------ exports ---------------------------------- */
function dl(name, blob) {
  const u = URL.createObjectURL(blob), a = document.createElement('a');
  a.href = u; a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(u), 2000);
}
function exportPNG() {
  // render at 2x to an offscreen canvas using current view
  const tmp = document.createElement('canvas');
  const sc = 2; tmp.width = cv._w * sc; tmp.height = cv._h * sc;
  const tctx = tmp.getContext('2d');
  tctx.fillStyle = '#0b1426'; tctx.fillRect(0, 0, tmp.width, tmp.height);
  tctx.drawImage(cv, 0, 0, tmp.width, tmp.height);
  tmp.toBlob(b => dl('parking-layout.png', b));
  toast('已匯出 PNG');
}
function activePark() { return S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution; }
function exportCSV() {
  const park = activePark();
  let csv = '';
  if (S.mode === 'site' && S.site) {
    const s = S.site;
    csv += 'metric,value\n';
    csv += `use,${$('#sUse').value}\nfloors,${s.floors}\nheight_ft,${Math.round(s.height)}\n`;
    csv += `GFA_sf,${Math.round(s.gfa)}\nNRSF_sf,${Math.round(s.nrsf)}\nFAR,${s.far.toFixed(2)}\ncoverage_pct,${s.coverage.toFixed(1)}\n`;
    if (s.residential) { csv += `units,${s.units}\ndensity_du_ac,${(s.units / s.acres).toFixed(1)}\n`; s.unitsByType.forEach(u => csv += `units_${u.type},${u.count}\n`); }
    csv += `parking_required,${s.parkingRequired}\nparking_provided,${s.parkingProvided}\n`;
    csv += `total_cost,${Math.round(s.fin.totalCost)}\nNOI,${Math.round(s.fin.noi)}\nyield_on_cost_pct,${s.fin.yieldOnCost.toFixed(2)}\n\n`;
  }
  if (park && park.stalls.length) {
    csv += 'stall_id,type,center_x_ft,center_y_ft\n';
    park.stalls.forEach((s, i) => csv += `${i + 1},${s.type},${s.cx.toFixed(2)},${s.cy.toFixed(2)}\n`);
  }
  if (!csv) { toast('尚無結果'); return; }
  dl(S.mode === 'site' ? 'site-feasibility.csv' : 'parking-stalls.csv', new Blob([csv], { type: 'text/csv' }));
  toast('已匯出 CSV');
}
function exportJSON() {
  dl('parking-project.json', new Blob([JSON.stringify(serialize(), null, 2)], { type: 'application/json' }));
  toast('已匯出 JSON');
}
function exportDXF() {
  // R12 DXF: closed POLYLINE rings (CAD-native) + TEXT annotations
  let s = '0\nSECTION\n2\nENTITIES\n';
  const ring = (poly, layer) => {                 // closed LWPOLYLINE-style POLYLINE
    s += `0\nPOLYLINE\n8\n${layer}\n66\n1\n70\n1\n`;
    for (const p of poly) s += `0\nVERTEX\n8\n${layer}\n10\n${p.x.toFixed(3)}\n20\n${(-p.y).toFixed(3)}\n30\n0\n`;
    s += '0\nSEQEND\n';
  };
  const text = (p, h, str, layer) =>
    s += `0\nTEXT\n8\n${layer}\n10\n${p.x.toFixed(3)}\n20\n${(-p.y).toFixed(3)}\n30\n0\n40\n${h}\n1\n${String(str).replace(/\n/g, ' ')}\n`;
  if (S.boundary.length) ring(S.boundary, 'SITE');
  if (S.parcels) S.parcels.forEach((pc, i) => { if (pc !== S.boundary) ring(pc, 'SUBPARCEL_' + String.fromCharCode(65 + i)); });
  S.buildings.forEach(b => { ring(b.poly, 'BUILDING'); (b.voids || []).forEach(v => ring(v, 'BUILDING_VOID')); });
  S.obstacles.forEach(o => ring(o, 'OBSTACLE'));
  if (S.mode === 'site' && S.site) {
    if (S.site.envelope.length) ring(S.site.envelope, 'BUILDABLE_ENVELOPE');
    if (S.site.footprint.length) ring(S.site.footprint, 'BUILDING_MASSING');
  }
  const park = activePark();
  if (park) {
    park.stalls.forEach(st => ring(st.poly, 'STALL_' + st.type.toUpperCase()));
    park.aisles.forEach(a => ring(a.poly, 'AISLE'));
    if (park.accessAisles) park.accessAisles.forEach(a => ring(a.poly, 'ADA_ACCESS'));
  }
  // TEXT annotations (title / stall count / date) above the site
  if (S.boundary.length >= 3) {
    const bb = PS.bbox(S.boundary);
    const stalls = park ? park.stalls.length : 0;
    text({ x: bb.minX, y: bb.maxY + 24 }, 12, `TestFit Clone — ${stalls} stalls @ ${new Date().toISOString().slice(0, 10)}`, 'ANNOTATION');
    if (S.mode === 'site' && S.site) text({ x: bb.minX, y: bb.maxY + 8 }, 9, `GFA ${Math.round(S.site.gfa)} sf · FAR ${S.site.far.toFixed(2)} · ${S.site.floors}F`, 'ANNOTATION');
  }
  s += '0\nENDSEC\n0\nEOF\n';
  dl('parking-layout.dxf', new Blob([s], { type: 'application/dxf' }));
  toast('已匯出 DXF（含 POLYLINE 與文字標註）');
}

/* Wavefront OBJ — extruded 3D massing (ground + buildings + obstacles + site mass + stalls) */
function exportOBJ() {
  if (S.boundary.length < 3) { toast('請先畫出基地'); return; }
  const V = [], F = []; let n = 0;            // n = vertices written (OBJ is 1-indexed)
  const box = (foot, h, name) => {
    const k = foot.length, base = n + 1;
    foot.forEach(p => V.push(`v ${p.x.toFixed(3)} 0 ${p.y.toFixed(3)}`));                  // bottom ring
    foot.forEach(p => V.push(`v ${p.x.toFixed(3)} ${(+h).toFixed(3)} ${p.y.toFixed(3)}`)); // top ring
    n += 2 * k;
    F.push('g ' + name);
    for (let i = 0; i < k; i++) F.push(`f ${base + i} ${base + (i + 1) % k} ${base + k + (i + 1) % k} ${base + k + i}`);  // walls
    F.push('f ' + Array.from({ length: k }, (_, i) => base + k + i).join(' '));            // roof
    F.push('f ' + Array.from({ length: k }, (_, i) => base + (k - 1 - i)).join(' '));      // floor
  };
  const flat = (poly, z, name) => {
    const base = n + 1;
    poly.forEach(p => V.push(`v ${p.x.toFixed(3)} ${z.toFixed(3)} ${p.y.toFixed(3)}`));
    n += poly.length;
    F.push('g ' + name); F.push('f ' + Array.from({ length: poly.length }, (_, i) => base + i).join(' '));
  };
  flat(S.boundary, 0, 'site_ground');
  S.buildings.forEach((b, i) => box(b.poly, bHeight(b), 'building_' + (i + 1)));
  S.obstacles.forEach((o, i) => box(o, 3, 'obstacle_' + (i + 1)));
  if (S.mode === 'site' && S.site && S.site.footprint && S.site.footprint.length >= 3) box(S.site.footprint, S.site.height, 'massing');
  const park = activePark();
  if (park) park.stalls.forEach((s, i) => flat(s.poly, 0.1, 'stall_' + (i + 1) + '_' + s.type));
  const obj = '# TestFit Clone — 3D massing (Wavefront OBJ); units: feet; Y = up\n' + V.join('\n') + '\n' + F.join('\n') + '\n';
  dl((S.mode === 'site' ? 'site' : 'parking') + '-massing.obj', new Blob([obj], { type: 'text/plain' }));
  toast('已匯出 3D 模型 .obj（可匯入 SketchUp / Blender / Rhino）');
}

/* Binary glTF (.glb) — triangulated, colour-grouped 3D model (Blender / web viewers / Revit via import). */
function hexToRgb01(hex) {
  const m = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex || '#3b82f6');
  return m ? [parseInt(m[1], 16) / 255, parseInt(m[2], 16) / 255, parseInt(m[3], 16) / 255] : [0.23, 0.51, 0.96];
}
function exportGLTF() {
  if (S.boundary.length < 3) { toast('請先畫出基地'); return; }
  const pos = [];                                   // flat [x,y,z,...]  Y = up
  const groups = [];                                // {indices:[], color:[r,g,b,a]}
  const grp = (key, color) => { let g = groups.find(x => x.key === key); if (!g) { g = { key, indices: [], color }; groups.push(g); } return g; };
  const addV = (x, y, z) => { pos.push(x, y, z); return pos.length / 3 - 1; };
  const box = (foot, h, key, color, z0 = 0) => {
    const g = grp(key, color), k = foot.length;
    const bot = foot.map(p => addV(p.x, z0, p.y)), top = foot.map(p => addV(p.x, z0 + h, p.y));
    for (let i = 0; i < k; i++) { const j = (i + 1) % k; g.indices.push(bot[i], bot[j], top[j], bot[i], top[j], top[i]); }   // walls
    for (let i = 1; i < k - 1; i++) g.indices.push(top[0], top[i], top[i + 1]);          // roof fan
    for (let i = 1; i < k - 1; i++) g.indices.push(bot[0], bot[i + 1], bot[i]);          // floor fan
  };
  const flat = (poly, z, key, color) => {
    const g = grp(key, color), v = poly.map(p => addV(p.x, z, p.y));
    for (let i = 1; i < v.length - 1; i++) g.indices.push(v[0], v[i], v[i + 1]);
  };
  flat(S.boundary, 0, 'ground', [0.16, 0.21, 0.28, 1]);
  S.buildings.forEach(b => box(b.poly, bHeight(b), 'building', [0.39, 0.45, 0.55, 1]));
  S.obstacles.forEach(o => box(o, 3, 'obstacle', [0.50, 0.11, 0.11, 1]));
  if (S.mode === 'site' && S.site && S.site.footprint && S.site.footprint.length >= 3) {
    const podium = (S.site.structured && S.site.garage) ? S.site.garage.levelsAbove * (S.site.garage.floorHeight || 11) : 0;
    box(S.site.footprint, S.site.height, 'massing', [0.05, 0.65, 0.91, 1], podium);
    if (S.site.structured && S.site.garage) {       // stacked garage decks as thin slabs
      const g = S.site.garage, fh = g.floorHeight || 11;
      for (let lv = 0; lv < g.levelsAbove; lv++) box(S.site.footprint, 0.6, 'garage_deck', [0.42, 0.46, 0.55, 1], lv * fh);
      for (let lv = 1; lv <= g.levelsBelow; lv++) box(S.site.footprint, 0.6, 'garage_deck', [0.3, 0.35, 0.45, 1], -lv * fh);
    }
  }
  const park = activePark();
  if (park) {
    park.stalls.forEach(s => { const c = hexToRgb01(COLORS[s.type] || COLORS.standard); flat(s.poly, 0.3, 'stall_' + s.type, [c[0], c[1], c[2], 1]); });
    (park.connectors || []).forEach(cn => flat(cn.poly, 0.2, cn.type ? 'spine' : 'road', cn.type ? [0.13, 0.77, 0.37, 1] : [0.96, 0.62, 0.04, 1]));
  }
  if (!pos.length) { toast('沒有可匯出的幾何'); return; }
  // --- pack into a .glb (12B header + JSON chunk + BIN chunk) ---
  const posF32 = new Float32Array(pos);
  const idxArrs = groups.map(g => new Uint32Array(g.indices));
  let binLen = posF32.byteLength; const idxOff = [];
  idxArrs.forEach(a => { idxOff.push(binLen); binLen += a.byteLength; });
  const bin = new Uint8Array(binLen);
  bin.set(new Uint8Array(posF32.buffer), 0);
  idxArrs.forEach((a, i) => bin.set(new Uint8Array(a.buffer), idxOff[i]));
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < pos.length; i += 3) for (let c = 0; c < 3; c++) { mn[c] = Math.min(mn[c], pos[i + c]); mx[c] = Math.max(mx[c], pos[i + c]); }
  const bufferViews = [{ buffer: 0, byteOffset: 0, byteLength: posF32.byteLength, target: 34962 }];
  const accessors = [{ bufferView: 0, componentType: 5126, count: pos.length / 3, type: 'VEC3', min: mn, max: mx }];
  const materials = [], primitives = [];
  groups.forEach((g, i) => {
    bufferViews.push({ buffer: 0, byteOffset: idxOff[i], byteLength: idxArrs[i].byteLength, target: 34963 });
    accessors.push({ bufferView: i + 1, componentType: 5125, count: g.indices.length, type: 'SCALAR' });
    materials.push({ name: g.key, pbrMetallicRoughness: { baseColorFactor: g.color, metallicFactor: 0, roughnessFactor: 0.85 }, doubleSided: true });
    primitives.push({ attributes: { POSITION: 0 }, indices: i + 1, material: i });
  });
  const gltf = { asset: { version: '2.0', generator: 'TestFit Clone' }, scene: 0, scenes: [{ nodes: [0] }],
    nodes: [{ mesh: 0, name: (S.mode === 'site' ? 'site' : 'parking') + '_model' }], meshes: [{ primitives }],
    materials, accessors, bufferViews, buffers: [{ byteLength: binLen }] };
  const enc = new TextEncoder();
  let jsonBytes = enc.encode(JSON.stringify(gltf));
  const jsonPad = (4 - (jsonBytes.length % 4)) % 4; if (jsonPad) { const t = new Uint8Array(jsonBytes.length + jsonPad); t.set(jsonBytes); t.fill(0x20, jsonBytes.length); jsonBytes = t; }
  const binPad = (4 - (binLen % 4)) % 4;
  const total = 12 + 8 + jsonBytes.length + 8 + binLen + binPad;
  const glb = new ArrayBuffer(total), dv = new DataView(glb); let o = 0;
  dv.setUint32(o, 0x46546C67, true); o += 4; dv.setUint32(o, 2, true); o += 4; dv.setUint32(o, total, true); o += 4;   // header
  dv.setUint32(o, jsonBytes.length, true); o += 4; dv.setUint32(o, 0x4E4F534A, true); o += 4;                          // JSON chunk
  new Uint8Array(glb, o, jsonBytes.length).set(jsonBytes); o += jsonBytes.length;
  dv.setUint32(o, binLen + binPad, true); o += 4; dv.setUint32(o, 0x004E4942, true); o += 4;                           // BIN chunk
  new Uint8Array(glb, o, binLen).set(bin);
  dl((S.mode === 'site' ? 'site' : 'parking') + '-model.glb', new Blob([glb], { type: 'model/gltf-binary' }));
  toast('已匯出 3D 模型 .glb（Blender / 網頁 3D / Revit 可匯入）');
}

/* ------------------------------- toast ----------------------------------- */
let toastT;
function toast(msg) {
  const t = $('#toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(toastT); toastT = setTimeout(() => t.classList.remove('show'), 2200);
}

/* ----------------------------- sample site ------------------------------- */
function sampleSite() {
  // ~ 420 x 300 ft L-shaped parcel with a building block
  S.boundary = [
    { x: 0, y: 0 }, { x: 420, y: 0 }, { x: 420, y: 300 },
    { x: 180, y: 300 }, { x: 180, y: 180 }, { x: 0, y: 180 },
  ];
  S.buildings = [makeBuilding([
    { x: 250, y: 40 }, { x: 400, y: 40 }, { x: 400, y: 160 }, { x: 250, y: 160 },
  ])];
  S.obstacles = [[
    { x: 40, y: 40 }, { x: 110, y: 40 }, { x: 110, y: 110 }, { x: 40, y: 110 },
  ]];
  S.entrances = [{ x: 0, y: 90 }];
  S.solution = null; S.selStall = null;
  updateMetrics(); fitView(); commit();
}

/* ------------------------------ wiring ----------------------------------- */
$('#btnSolve').onclick = () => S.mode === 'site' ? doSolveSite() : doSolve();
$('#btnSample').onclick = () => {
  if (S.mode === 'site') { sampleSiteParcel(); setTool('select'); setTimeout(doSolveSite, 60); }
  else { sampleSite(); setTool('select'); setTimeout(doSolve, 60); }
};
$('#btnClear').onclick = () => {
  S.boundary = []; S.buildings = []; S.obstacles = []; S.roads = []; S.roadLines = []; S.parkZones = []; S.manualCores = []; S.gridShift = null; S.aisleEdits = null; S.contextBuildings = []; S.entrances = [];
  S.solution = null; S.site = null; S.selStall = null;
  S.parcels = null; S.activeParcel = 0; S.splitPt = null;
  S.measures = []; S.measureStart = null; S.edgeSetback = {}; S.selEdge = null;
  S.mode === 'site' ? updateSiteMetrics() : updateMetrics();
  if (S.mapMode) draw(); else fitView();
  commit();
  toast('已清空');
};
function setTrees(on) {                          // single source of truth for the tree on/off (button + panel + tree)
  S.layers.trees.vis = on;
  $('#btnTrees').classList.toggle('on', on);
  document.querySelectorAll('#treesSeg button').forEach(b => b.classList.toggle('active', (b.dataset.tr === '1') === on));
  draw();
  if ($('#objTree') && $('#objTree').classList.contains('show')) buildObjTree();
}
$('#btnTrees').onclick = () => setTrees(!S.layers.trees.vis);
document.querySelectorAll('#treesSeg button').forEach(b => b.onclick = () => setTrees(b.dataset.tr === '1'));
$('#btnUndo').onclick = undo;
$('#btnRedo').onclick = redo;
$('#zin').onclick = () => zoomBtn(1.2);
$('#zout').onclick = () => zoomBtn(1 / 1.2);
$('#zfit').onclick = () => { S.is3d ? (fit3D(), draw()) : S.mapMode ? fitMapToGeom() : fitView(); };
function zoomBtn(f) {
  if (S.mapMode && S.map) { S.map.setZoom(S.map.getZoom() + (f > 1 ? 1 : -1)); return; }
  if (S.is3d) { S.view.scale = Math.max(0.02, Math.min(20, S.view.scale * f)); draw(); return; }
  const cx = cv._w / 2, cy = cv._h / 2, before = toWorld({ x: cx, y: cy });
  S.view.scale = Math.max(0.02, Math.min(20, S.view.scale * f));
  const after = toScreen(before); S.view.ox += cx - after.x; S.view.oy += cy - after.y; draw();
}

// 2D / 3D view toggle
document.querySelectorAll('#viewSeg button').forEach(b => b.onclick = () => {
  document.querySelectorAll('#viewSeg button').forEach(x => x.classList.remove('active'));
  b.classList.add('active');
  const want3d = b.dataset.view === '3d';
  if (want3d && S.mapMode) enableMap(false);     // 3D and the real map can't co-exist
  S.is3d = want3d;
  if (S.is3d) fit3D(); else if (!S.mapMode) fitView();
  cv.style.cursor = S.is3d ? 'grab' : (S.tool === 'pan' ? 'grab' : 'default');
  draw();
});
$('#pH').addEventListener('input', () => { S.params.height = +$('#pH').value; if (S.is3d) { fit3D(); draw(); } });

// angle segmented control
// two-way vs one-way drive-aisle width per angle (ft). One-way is narrower → more
// rows fit → 擴容. Two-way is wider for two-direction circulation.
// 90° (perpendicular) parking needs ~24 ft for backing-out REGARDLESS of one-/two-way —
// one-way only saves space at angled (45°/60°) stalls. So 90° one-way stays wide.
const AISLE_BY_ANGLE = { 90: { two: 24, one: 22 }, 60: { two: 18, one: 16 }, 45: { two: 14, one: 12 } };
// aisle width for any angle (interpolated between the 45/60/90 anchors → safe for continuous angles)
function aisleForAngle(a, oneway) {
  const key = oneway ? 'one' : 'two';
  if (AISLE_BY_ANGLE[a]) return AISLE_BY_ANGLE[a][key];
  if (a <= 45) return AISLE_BY_ANGLE[45][key];
  if (a >= 90) return AISLE_BY_ANGLE[90][key];
  const lo = a < 60 ? 45 : 60, hi = a < 60 ? 60 : 90, t = (a - lo) / (hi - lo);
  return AISLE_BY_ANGLE[lo][key] + t * (AISLE_BY_ANGLE[hi][key] - AISLE_BY_ANGLE[lo][key]);
}
function applyAngleAisle() {
  // scale the angle-based aisle to the current region's standard, so Taiwan stays ~5.5 m
  // instead of jumping to the US 24 ft. Stall depth (pD) is left as the REAL depth —
  // the solver derives the angled depth-to-wall (vpd) itself.
  const r = $('#regionSel') && $('#regionSel').value;
  const base = (REGIONS[r] && REGIONS[r].aisle) || AISLE_BY_ANGLE[90].two;
  $('#pA').value = round2(U.L(aisleForAngle(S.params.angle, S.params.oneway) * base / AISLE_BY_ANGLE[90].two));
}
document.querySelectorAll('#angleSeg button').forEach(b => b.onclick = () => {
  document.querySelectorAll('#angleSeg button').forEach(x => x.classList.remove('active'));
  b.classList.add('active');
  S.params.angle = +b.dataset.ang;
  $('#pAngleNum').value = S.params.angle;        // sync continuous field
  applyAngleAisle();
  if (S.boundary.length >= 3) doSolve();
});
document.querySelectorAll('#onewaySeg button').forEach(b => b.onclick = () => {
  document.querySelectorAll('#onewaySeg button').forEach(x => x.classList.remove('active'));
  b.classList.add('active');
  S.params.oneway = b.dataset.ow === '1';
  applyAngleAisle();                                        // narrows/widens the aisle
  if (S.boundary.length >= 3) doSolve();
  toast(S.params.oneway ? '單向車道：車道較窄、可塞更多車位' : '雙向車道');
});

// continuous stall angle (any 30–90°, beyond the 90/60/45 presets)
$('#pAngleNum').addEventListener('change', () => {
  let v = Math.max(30, Math.min(90, +$('#pAngleNum').value || 90));
  $('#pAngleNum').value = v; S.params.angle = v;
  document.querySelectorAll('#angleSeg button').forEach(x => x.classList.toggle('active', +x.dataset.ang === v));
  applyAngleAisle();                              // continuous angle → update the aisle width too
  if (S.boundary.length >= 3) doSolve();
});
// Max Run (landscape islands) toggle
document.querySelectorAll('#maxRunSeg button').forEach(b => b.onclick = () => {
  document.querySelectorAll('#maxRunSeg button').forEach(x => x.classList.remove('active'));
  b.classList.add('active');
  const on = b.dataset.mr === '1';
  $('#maxRunRow1').style.display = on ? '' : 'none';
  $('#maxRunRow2').style.display = on ? '' : 'none';
  if (S.solution) doSolve();
});
// target only re-validates (no re-solve needed — it doesn't change the layout)
$('#pTarget').addEventListener('input', () => { readParams(); updateMetrics(); draw(); });
// live params -> re-solve on change (cheap enough). Any param tweak clears a manual grid-drag (fresh layout).
['#pW','#pD','#pA','#pS','#pOrient','#pGreen','#pMaxRun','#pMaxGap'].forEach(s => $(s).addEventListener('change', () => { S.gridShift = null; S.aisleEdits = null; if (S.solution) doSolve(); }));
// CHANGE ROW AXIS — cycle the circulation direction (like TestFit's Row-axis change); the whole lot re-packs cleanly
const ROW_AXIS = ['edge', '90', '0', 'auto'];
$('#btnRowAxis').onclick = () => {
  const next = ROW_AXIS[(ROW_AXIS.indexOf(String(S.params.orient)) + 1) % ROW_AXIS.length];
  S.params.orient = next; $('#pOrient').value = next; S.gridShift = null; S.aisleEdits = null;   // new direction → fresh grid phase
  if (S.boundary.length < 3) { toast('請先畫出基地邊界再換動線'); return; }
  doSolve();                                   // re-packs the whole lot around the new circulation direction (toasts the new count)
};
['#pEV','#pMoto','#adaManual','#pGFA'].forEach(s => $(s).addEventListener('input', () => { readParams(); if (S.solution) { reassign(); } updateMetrics(); draw(); }));
// Compact %/width change the PACKING (narrower stalls fit more), so re-solve, not just reassign
$('#pCompact').addEventListener('change', () => { readParams(); if (S.solution) doSolve(); });
$('#cW').addEventListener('change', () => { readParams(); if (S.solution) doSolve(); });
document.querySelectorAll('#accessSeg button').forEach(b => b.onclick = () => {
  document.querySelectorAll('#accessSeg button').forEach(x => x.classList.remove('active')); b.classList.add('active');
  readParams();
  if (S.params.access === 'single' && S.entrances.length > 1) S.entrances = S.entrances.slice(0, 1);   // enforce one gate
  if (S.solution) doSolve();
  toast({ open: '開放式：臨路面開放、不設閘口（保留全部車位）', single: '單一出入口：只用一個閘口，其餘車位需從它連通', multi: '多出入口：可設多個閘口' }[S.params.access]);
});
// per-type colour pickers — recolour stalls live (flows to canvas, legend, and 3D)
const STALL_COLOR_INPUTS = { standard: '#colStandard', compact: '#colCompact', ev: '#colEv', ada: '#colAda' };
Object.entries(STALL_COLOR_INPUTS).forEach(([k, sel]) => {
  const el = $(sel); if (!el) return;
  el.value = COLORS[k];
  el.addEventListener('input', () => { COLORS[k] = el.value; buildLegend(); draw(); });
});
$('#adaMode').addEventListener('change', () => {
  $('#adaManualRow').style.display = $('#adaMode').value === 'manual' ? '' : 'none';
  readParams(); if (S.solution) reassign(); updateMetrics(); draw();
});
function reassign() {
  let focus = PS.centroid(S.boundary);
  if (S.buildings.length) focus = PS.centroid(bPoly(S.buildings[0]));
  PS.assignTypes(S.solution, {
    adaMode: S.opts.adaMode, adaManual: S.opts.adaManual,
    evPct: S.opts.evPct, compactPct: S.opts.compactPct, motoPct: S.opts.motoPct, focus,
  });
}

$('#btnSaveScheme').onclick = saveScheme;
$('#exPNG').onclick = exportPNG;
$('#exCSV').onclick = exportCSV;
$('#exDXF').onclick = exportDXF;
$('#exOBJ').onclick = exportOBJ;
$('#exGLTF').onclick = exportGLTF;
$('#exJSON').onclick = exportJSON;
$('#impBtn').onclick = () => $('#impJSON').click();
$('#impJSON').onchange = e => {
  const f = e.target.files[0]; if (!f) return;
  const rd = new FileReader();
  rd.onload = () => { try { deserialize(JSON.parse(rd.result)); toast('已匯入'); } catch { toast('檔案格式錯誤'); } };
  rd.readAsText(f);
};
$('#btnHelp').onclick = () => $('#help').style.display = $('#help').style.display === 'none' ? 'block' : 'none';
$('#helpClose').onclick = () => $('#help').style.display = 'none';

/* ----------------------------- site solver mode -------------------------- */
const USE_PRESETS = {
  multifamily:  { floorH: 11, eff: 82, parkRatio: 1.5, resi: true },
  tower:        { floorH: 11, eff: 80, parkRatio: 1.0, resi: true },
  garden:       { floorH: 11, eff: 84, parkRatio: 1.5, resi: true },
  singlefamily: { floorH: 10, eff: 92, parkRatio: 2.0, resi: true },
  mixeduse:     { floorH: 12, eff: 80, parkRatio: 1.3, resi: true },
  office:       { floorH: 13, eff: 85, parkRatio: 3.0, resi: false },
  retail:       { floorH: 16, eff: 88, parkRatio: 4.0, resi: false },
  hotel:        { floorH: 10, eff: 70, parkRatio: 1.0, resi: false },
  industrial:   { floorH: 28, eff: 95, parkRatio: 1.0, resi: false },
  datacenter:   { floorH: 20, eff: 85, parkRatio: 0.3, resi: false },
};
const UNIT_LABEL = { studio: 'Studio 套房', '1br': '1-Bed 一房', '2br': '2-Bed 二房', '3br': '3-Bed 三房',
  townhome: '連棟 Townhome', detached: '獨棟 Detached', cottage: '小宅 Cottage/ADU' };

function setMode(mode) {
  S.mode = mode;
  S.selEdge = null; hideEdgePopup();

  document.querySelectorAll('#modeSeg button').forEach(b => b.classList.toggle('active', b.dataset.mode === mode));
  $('#grpParking').style.display = mode === 'parking' ? '' : 'none';
  $('#grpSite').style.display = mode === 'site' ? '' : 'none';
  $('#metricsParking').style.display = mode === 'parking' ? '' : 'none';
  $('#metricsSite').style.display = mode === 'site' ? '' : 'none';
  $('#btnSolve').lastChild.textContent = mode === 'site' ? '自動配置建案' : '自動排車位';
  $('#stTool').textContent = '模式：' + (mode === 'site' ? '建案規劃' : '停車場');
  if (mode === 'site') { updateSiteMetrics(); if (S.boundary.length >= 3 && !S.site) doSolveSite(); }
  else updateMetrics();
  draw();
}

function readSiteParams() {
  const use = $('#sUse').value;
  return {
    useType: use,
    edgeSetback: S.edgeSetback,
    floorHeight: U.Lr(+$('#sFloorH').value),
    efficiency: (+$('#sEff').value) / 100,
    maxFAR: +$('#zFAR').value, maxHeight: U.Lr(+$('#zHeight').value),
    maxCoverage: +$('#zCov').value, maxDUA: +$('#zDUA').value,
    setbacks: { front: U.Lr(+$('#zSbF').value), side: U.Lr(+$('#zSbS').value), rear: U.Lr(+$('#zSbR').value) },
    parkingRatio: +$('#zPark').value, parkAngle: ($('#sParkAngle') ? +$('#sParkAngle').value : (S.siteParkAngle || 90)), parkSetback: 5, evPct: 0,
    parkingType: ($('#sParkType') ? $('#sParkType').value : 'surface'),
    dockType: ($('#sDockType') ? $('#sDockType').value : 'cross'),
    subType: ($('#sSubType') ? $('#sSubType').value : 'townhome'),
    groundRetail: ($('#sGroundRetail') ? $('#sGroundRetail').checked : false),
    parkingLevelsAbove: ($('#sParkLevelsAbove') ? +$('#sParkLevelsAbove').value : 3),
    parkingLevelsBelow: ($('#sParkLevelsBelow') ? +$('#sParkLevelsBelow').value : 0),
    structEff: ($('#sStructEff') ? +$('#sStructEff').value : 95) || 95,
    unitMix: [
      { type: 'studio', pct: +$('#uxStudioP').value, size: U.Ar(+$('#uxStudioS').value) },
      { type: '1br', pct: +$('#ux1P').value, size: U.Ar(+$('#ux1S').value) },
      { type: '2br', pct: +$('#ux2P').value, size: U.Ar(+$('#ux2S').value) },
      { type: '3br', pct: +$('#ux3P').value, size: U.Ar(+$('#ux3S').value) },
    ],
    fin: {
      landCost: +$('#fLand').value, hardCost: +$('#fHard').value, softPct: +$('#fSoft').value,
      rentMo: +$('#fRentMo').value, rentSfYr: +$('#fRentSf').value, opexPct: +$('#fOpex').value,
      rentGrowth: +$('#fGrowth').value, holdYears: +$('#fHold').value, exitCap: +$('#fExitCap').value,
    },
  };
}

function doSolveSite() {
  if (S.boundary.length < 3) { toast('請先畫出基地邊界'); setTool('boundary'); return; }
  const p = readSiteParams();
  $('#busy').classList.add('show'); $('#busyTxt').textContent = '配置建案量體中…';
  setTimeout(() => {
    const t0 = performance.now();
    S.site = PS.solveSite({ boundary: S.boundary, p, entrances: S.entrances, obstacles: S.obstacles, roads: S.roads });
    if (S.site && S.site.parkSol) { applyAisleEdits(S.site.parkSol); deriveSpines(S.site.parkSol); }   // overrides + editable spines
    $('#busy').classList.remove('show');
    updateSiteMetrics(); draw(); commit();
    const ms = Math.round(performance.now() - t0);
    if (S.site) toast(`建案：${S.site.residential ? S.site.units + ' 戶' : Math.round(S.site.gfa).toLocaleString() + ' SF'} · ${S.site.floors}F · ${ms}ms`);
    $('#stMsg').textContent = S.site ? `建案 ${S.site.floors}F · FAR ${S.site.far.toFixed(2)}` : '無法配置';
  }, 30);
}

function updateSiteMetrics() {
  S._trees = null; updateTabBar(); refreshBldgPanel(); buildLegend(); updateFinancials();   // trees + tabulation + appearance + legend + pro-forma follow the layout
  const s = S.site, fmt = n => Math.round(U.A(n)).toLocaleString();
  $('#sUnits').textContent = s ? (s.residential ? s.units.toLocaleString() : '商用') : '0';
  $('#sGFA').textContent = s ? fmt(s.gfa) : '—';
  $('#sNRSF').textContent = s ? fmt(s.nrsf) : '—';
  $('#sFAR').textContent = s ? s.far.toFixed(2) : '—';
  $('#sFloors').textContent = s ? s.floors : '—';
  $('#sDensity').textContent = s && s.residential ? (s.units / s.acres).toFixed(1) : '—';
  $('#sYield').textContent = s && s.fin.totalCost > 0 ? s.fin.yieldOnCost.toFixed(1) + '%' : '—';

  const siteArea = S.boundary.length >= 3 ? PS.polyArea(S.boundary) : 0;
  $('#stArea').textContent = siteArea
    ? `面積：${Math.round(U.A(siteArea)).toLocaleString()} ${U.au()} (${U.big(siteArea).toFixed(2)} ${U.bu()})` : '面積：—';

  const bd = $('#siteBreakdown'); bd.innerHTML = '';
  if (s) {
    if (s.residential) s.unitsByType.forEach(u => {
      if (!u.count) return;
      const row = document.createElement('div'); row.className = 'brow';
      const sw = document.createElement('span'); sw.className = 'sw'; sw.style.background = '#38bdf8';
      const nm = document.createElement('span'); nm.className = 'nm'; nm.textContent = `${UNIT_LABEL[u.type]} · ${Math.round(U.A(u.size))} ${U.au()}`;
      const ct = document.createElement('span'); ct.className = 'ct'; ct.textContent = u.count;
      row.append(sw, nm, ct); bd.appendChild(row);
    });
    if (s.industrial) {                                    // warehouse: dock doors + trailer stalls + clear height
      const ind = s.industrial;
      [['🚛 卸貨月台門 Dock doors', ind.dockCount + ' 門'],
       ['🚚 拖車位 Trailer stalls', ind.trailerCount + ' 位'],
       ['📐 淨高 Clear height', Math.round(U.L(s.height)) + ' ' + U.lu()]].forEach(([label, val]) => {
        const row = document.createElement('div'); row.className = 'brow';
        const sw = document.createElement('span'); sw.className = 'sw'; sw.style.background = '#64748b';
        const nm = document.createElement('span'); nm.className = 'nm'; nm.textContent = label;
        const ct = document.createElement('span'); ct.className = 'ct'; ct.textContent = val;
        row.append(sw, nm, ct); bd.appendChild(row);
      });
    }
    if (s.hotel) {                                          // hotel: room keys
      const row = document.createElement('div'); row.className = 'brow';
      const sw = document.createElement('span'); sw.className = 'sw'; sw.style.background = '#0ea5e9';
      const nm = document.createElement('span'); nm.className = 'nm'; nm.textContent = '🛎️ 客房 Keys · 雙載走廊';
      const ct = document.createElement('span'); ct.className = 'ct'; ct.textContent = s.keys + ' 間';
      row.append(sw, nm, ct); bd.appendChild(row);
    }
    const pr = document.createElement('div'); pr.className = 'hint'; pr.style.marginTop = '4px';
    const parkTxt = s.isWrap
      ? `🏟️ 環繞車庫 地上${s.levelsAbove}+地下${s.levelsBelow}層 × ${s.parkingPerFloor}/層（核心）= 提供 ${s.parkingProvided} / 需 ${s.parkingRequired}`
      : s.structured
      ? `🏢 結構車庫 地上${s.levelsAbove}+地下${s.levelsBelow}層 × ${s.parkingPerFloor}/層 × ${s.structEff}% = 提供 ${s.parkingProvided} / 需 ${s.parkingRequired}`
      : `🅿️ 地面停車 提供 ${s.parkingProvided} / 需 ${s.parkingRequired}`;
    pr.textContent = `${parkTxt}　·　土建 $${fmt(s.fin.totalCost)}　·　NOI $${fmt(s.fin.noi)}/年`;
    bd.appendChild(pr);
  } else bd.innerHTML = '<div class="hint">畫好基地後按「自動配置建案」。</div>';
  renderCompliance(s);
}

function renderCompliance(s) {
  const box = $('#complianceList'); box.innerHTML = '';
  if (!s) { box.innerHTML = '<div class="hint">畫好基地、按「自動配置建案」後顯示。</div>'; return; }
  const checks = s.compliance.filter(c => !c.info);        // info rows (e.g. logistics scale) don't count as pass/fail
  const pass = checks.filter(c => c.ok).length, total = checks.length;
  const score = document.createElement('div');
  score.id = 'complianceScore'; score.className = pass === total ? 'pass' : 'fail';
  score.textContent = pass === total ? `✓ 全數通過 (${pass}/${total})` : `✕ ${total - pass} 項不符法規 (${pass}/${total} 通過)`;
  box.appendChild(score);
  s.compliance.forEach(c => {
    const row = document.createElement('div'); row.className = 'cc';
    const b = document.createElement('span'); b.className = 'badge ' + (c.info ? 'info' : c.ok ? 'ok' : 'no'); b.textContent = c.info ? 'ℹ' : c.ok ? '✓' : '✕';
    const nm = document.createElement('span'); nm.className = 'nm'; nm.textContent = c.k;
    const v = document.createElement('span'); v.className = 'val'; v.textContent = c.val;
    row.append(b, nm, v); box.appendChild(row);
  });
  if (s.densityCapped) {
    const h = document.createElement('div'); h.className = 'hint'; h.style.marginTop = '6px';
    h.textContent = '※ 戶數受密度上限（DU/acre）限制，未用滿容積。'; box.appendChild(h);
  }
  if (s.parkCapped) {
    const h = document.createElement('div'); h.className = 'hint'; h.style.marginTop = '6px';
    h.textContent = '※ 花園公寓戶數受地面停車容量限制（低層多棟的密度限制因子）。'; box.appendChild(h);
  }
}

function updateUxSum() {
  const sum = +$('#uxStudioP').value + +$('#ux1P').value + +$('#ux2P').value + +$('#ux3P').value;
  $('#uxSum').textContent = '佔比合計：' + sum + '%' + (sum !== 100 ? '（非 100% 會依比例換算）' : '');
}

function sampleSiteParcel() {
  S.boundary = [{ x: 0, y: 0 }, { x: 417, y: 0 }, { x: 417, y: 426 }, { x: 0, y: 426 }];
  S.buildings = []; S.obstacles = []; S.roads = []; S.roadLines = []; S.parkZones = []; S.manualCores = []; S.gridShift = null; S.aisleEdits = null; S.entrances = [{ x: 208, y: 0, type: 'inout' }];
  S.solution = null; S.site = null; S.selStall = null;
  if (S.mapMode) draw(); else fitView();
  commit();
}

/* ----------------------- 範例展示 demo gallery (one-click scenarios) ----------------------- */
const DEMO_PARCELS = {
  sample: [{ x: 0, y: 0 }, { x: 417, y: 0 }, { x: 417, y: 426 }, { x: 0, y: 426 }],
  wide:   [{ x: 0, y: 0 }, { x: 520, y: 0 }, { x: 520, y: 300 }, { x: 0, y: 300 }],
  big:    [{ x: 0, y: 0 }, { x: 760, y: 0 }, { x: 760, y: 440 }, { x: 0, y: 440 }],
  huge:   [{ x: 0, y: 0 }, { x: 1000, y: 0 }, { x: 1000, y: 640 }, { x: 0, y: 640 }],
};
const DEMOS = [
  { key: 'pk-surf',  g: '停車型態', label: '地面停車',           parcel: 'sample', use: 'multifamily', park: 'surface' },
  { key: 'pk-above', g: '停車型態', label: '結構車庫·地上3層',   parcel: 'sample', use: 'multifamily', park: 'structured', above: 3, below: 0 },
  { key: 'pk-below', g: '停車型態', label: '結構車庫·地下3層',   parcel: 'sample', use: 'multifamily', park: 'structured', above: 0, below: 3 },
  { key: 'pk-mix',   g: '停車型態', label: '地上2+地下2 混合',    parcel: 'sample', use: 'multifamily', park: 'structured', above: 2, below: 2 },
  { key: 'pk-wrap',  g: '停車型態', label: '環繞車庫 Wrap',       parcel: 'sample', use: 'multifamily', park: 'wrap', above: 5, below: 0 },
  { key: 'u-mf',  g: '建築用途', label: '多戶住宅 Multifamily',   parcel: 'sample', use: 'multifamily',  park: 'structured', above: 3, below: 0 },
  { key: 'u-twr', g: '建築用途', label: '高層塔樓 Tower（點式高樓）', parcel: 'big', use: 'tower', park: 'structured', above: 2, below: 0, far: 8, cov: 60, hgtFt: 300 },
  { key: 'u-sf',  g: '建築用途', label: '連棟 Townhome', parcel: 'sample', use: 'singlefamily', park: 'surface', far: 0.55, cov: 35, sub: 'townhome' },
  { key: 'u-sf2', g: '建築用途', label: '獨棟 Detached（大地坪+院）', parcel: 'big', use: 'singlefamily', park: 'surface', far: 0.55, cov: 35, sub: 'detached' },
  { key: 'u-sf3', g: '建築用途', label: '小宅 Cottage / ADU', parcel: 'sample', use: 'singlefamily', park: 'surface', far: 0.55, cov: 35, sub: 'cottage' },
  { key: 'u-grd', g: '建築用途', label: '花園公寓 Garden（低層多棟）', parcel: 'big', use: 'garden', park: 'surface', far: 1.0, cov: 45 },
  { key: 'u-mix', g: '建築用途', label: '複合用途 Mixed-use',     parcel: 'sample', use: 'mixeduse',     park: 'structured', above: 2, below: 1 },
  { key: 'u-off', g: '建築用途', label: '辦公 Office',            parcel: 'sample', use: 'office',       park: 'structured', above: 3, below: 1 },
  { key: 'u-ret', g: '建築用途', label: '零售中心 Retail（主力店+pad）', parcel: 'big', use: 'retail',  park: 'surface', far: 0.4, cov: 30 },
  { key: 'u-hot', g: '建築用途', label: '旅館 Hotel',             parcel: 'big',    use: 'hotel',        park: 'surface', far: 1.5, cov: 40 },
  { key: 'u-ind', g: '建築用途', label: '工業 雙面對流 Cross-dock', parcel: 'big',  use: 'industrial',   park: 'surface', far: 0.6, cov: 48, dock: 'cross' },
  { key: 'u-ind2',g: '建築用途', label: '物流 單面 Single-dock',  parcel: 'big',    use: 'industrial',   park: 'surface', far: 0.6, cov: 48, dock: 'single' },
  { key: 'u-dc',  g: '建築用途', label: '資料中心 Data Center',   parcel: 'huge',   use: 'datacenter',   park: 'surface', far: 1.0, cov: 55 },
];
function loadDemo(d) {
  if (!d) return;
  if (S.mode !== 'site') setMode('site');
  const par = DEMO_PARCELS[d.parcel] || DEMO_PARCELS.sample;
  S.boundary = par.map(p => ({ ...p }));
  const bb = PS.bbox(S.boundary);
  S.buildings = []; S.obstacles = []; S.roads = []; S.roadLines = []; S.parkZones = []; S.manualCores = []; S.gridShift = null; S.aisleEdits = null; S.edgeSetback = {};
  S.entrances = [{ x: (bb.minX + bb.maxX) / 2, y: bb.minY, type: 'inout' }];
  S.solution = null; S.site = null; S.selStall = null;
  $('#sUse').value = d.use;                                   // use type → presets + panel visibility
  const pre = USE_PRESETS[d.use] || USE_PRESETS.multifamily;
  $('#sFloorH').value = round2(U.L(pre.floorH)); $('#sEff').value = pre.eff; $('#zPark').value = pre.parkRatio;   // preset floorH is in ft → convert to the display unit
  $('#grpUnitMix').style.display = pre.resi ? '' : 'none';
  $('#fRentResiRow').style.display = pre.resi ? '' : 'none';
  $('#fRentCommRow').style.display = pre.resi ? 'none' : '';
  $('#zParkHint').textContent = pre.resi ? '住宅：車位 / 戶。' : '商用 / 工業：車位 / 1000 SF。';
  ['#rowDockType', '#dockTypeHint'].forEach(id => $(id).style.display = d.use === 'industrial' ? '' : 'none');
  if (d.dock) $('#sDockType').value = d.dock;
  $('#rowSubType').style.display = d.use === 'singlefamily' ? '' : 'none';
  if (d.sub) $('#sSubType').value = d.sub;
  $('#rowGroundRetail').style.display = ['multifamily', 'mixeduse', 'tower'].includes(d.use) ? '' : 'none';
  if ($('#sGroundRetail')) $('#sGroundRetail').checked = !!d.retail0;
  $('#zFAR').value = d.far != null ? d.far : 2.25;            // per-demo zoning (low-rise types get lower FAR/coverage)
  $('#zCov').value = d.cov != null ? d.cov : 50;
  if (d.hgtFt) $('#zHeight').value = round2(U.L(d.hgtFt));    // tall types (tower) override the height limit (ft → display unit)
  const deck = ['structured', 'wrap'].includes(d.park);       // both stack parking decks → show level controls
  $('#sParkType').value = d.park || 'surface';
  ['#rowParkLevels', '#rowParkBelow', '#rowStructEff', '#parkTypeHint'].forEach(id => $(id).style.display = deck ? '' : 'none');
  if (deck) { $('#sParkLevelsAbove').value = d.above != null ? d.above : 3; $('#sParkLevelsBelow').value = d.below != null ? d.below : 0; }
  fitView();
  doSolveSite();
  toast(`範例：${d.label}`);
}
(() => {                                                       // populate the gallery dropdown (grouped)
  const sel = $('#demoGallery'); if (!sel) return;
  let og = null, lastG = '';
  for (const d of DEMOS) {
    if (d.g !== lastG) { og = document.createElement('optgroup'); og.label = d.g; sel.appendChild(og); lastG = d.g; }
    const o = document.createElement('option'); o.value = d.key; o.textContent = d.label; og.appendChild(o);
  }
  sel.addEventListener('change', () => { const d = DEMOS.find(x => x.key === sel.value); if (d) loadDemo(d); sel.selectedIndex = 0; });
})();

// wiring
document.querySelectorAll('#modeSeg button').forEach(b => b.onclick = () => setMode(b.dataset.mode));
$('#sUse').addEventListener('change', () => {
  const use = $('#sUse').value, pre = USE_PRESETS[use];
  $('#sFloorH').value = round2(U.L(pre.floorH)); $('#sEff').value = pre.eff; $('#zPark').value = pre.parkRatio;   // preset floorH is in ft → convert to the display unit
  $('#grpUnitMix').style.display = pre.resi ? '' : 'none';
  $('#fRentResiRow').style.display = pre.resi ? '' : 'none';
  $('#fRentCommRow').style.display = pre.resi ? 'none' : '';
  $('#zParkHint').textContent = pre.resi ? '住宅：車位 / 戶。' : '商用 / 工業：車位 / 1000 SF。';
  ['#rowDockType', '#dockTypeHint'].forEach(id => $(id).style.display = use === 'industrial' ? '' : 'none');  // warehouse-only control
  $('#rowSubType').style.display = use === 'singlefamily' ? '' : 'none';                                      // single-family lot-type control
  $('#rowGroundRetail').style.display = ['multifamily', 'mixeduse', 'tower'].includes(use) ? '' : 'none';     // mixed-use only for stacked residential
  if (S.boundary.length >= 3) doSolveSite();
});
$('#sGroundRetail').addEventListener('change', () => { if (S.mode === 'site' && S.boundary.length >= 3) doSolveSite(); });
$('#sDockType').addEventListener('change', () => { if (S.mode === 'site' && S.boundary.length >= 3) doSolveSite(); });
$('#sSubType').addEventListener('change', () => { if (S.mode === 'site' && S.boundary.length >= 3) doSolveSite(); });
$('#sParkAngle').addEventListener('change', () => { S.siteParkAngle = +$('#sParkAngle').value; if (S.mode === 'site' && S.boundary.length >= 3) doSolveSite(); });
// financial pro-forma inputs → live readout (parking: cheap recompute on input; site: re-solve on change so S.site.fin updates)
['fLand', 'fHard', 'fSoft', 'fRentMo', 'fRentSf', 'fOpex', 'fGrowth', 'fHold', 'fExitCap'].forEach(id => {
  const el = $('#' + id); if (!el) return;
  el.addEventListener('input', () => { if (S.mode !== 'site') updateFinancials(); });
  el.addEventListener('change', () => { if (S.mode === 'site') { if (S.boundary.length >= 3) doSolveSite(); } else { updateFinancials(); commit(); } });
});
['#eNW', '#eNE', '#eSW', '#eSE', '#ePad', '#eCutC', '#eFillC', '#eHaulC'].forEach(s => { const el = $(s); if (el) el.addEventListener('input', computeEarthwork); });
if ($('#ePadBal')) $('#ePadBal').onclick = () => { computeEarthwork(); $('#ePad').value = (S._ewBalance || 0).toFixed(1); computeEarthwork(); toast('整平高設為挖填平衡點（免外運）'); };
$('#sParkType').addEventListener('change', () => {
  const deck = ['structured', 'wrap'].includes($('#sParkType').value);   // both stack parking decks → show level controls
  ['#rowParkLevels', '#rowParkBelow', '#rowStructEff', '#parkTypeHint'].forEach(id => $(id).style.display = deck ? '' : 'none');
  if (S.mode === 'site' && S.boundary.length >= 3) doSolveSite();
});
['#sFloorH', '#sEff', '#zFAR', '#zHeight', '#zCov', '#zDUA', '#zSbF', '#zSbS', '#zSbR', '#zPark', '#sParkLevelsAbove', '#sParkLevelsBelow', '#sStructEff',
 '#uxStudioP', '#ux1P', '#ux2P', '#ux3P', '#uxStudioS', '#ux1S', '#ux2S', '#ux3S',
 '#fLand', '#fHard', '#fSoft', '#fRentMo', '#fRentSf', '#fOpex', '#fGrowth', '#fHold', '#fExitCap'].forEach(sel =>
  $(sel).addEventListener('change', () => { updateUxSum(); if (S.mode === 'site' && S.boundary.length >= 3) doSolveSite(); }));

/* ----------------------------- real-world map ---------------------------- */
function enableMap(on) {
  if (on && typeof L === 'undefined') { toast('地圖需要網路連線（Leaflet 未載入）'); return; }
  S.mapMode = on;
  $('#btnMap').classList.toggle('active', on);
  $('#map').classList.toggle('show', on);
  $('#mapbar').classList.toggle('show', on);
  $('#scalebar').style.display = on ? 'none' : '';
  if (on) {
    if (S.is3d) {                       // map & 3D are mutually exclusive
      S.is3d = false;
      document.querySelectorAll('#viewSeg button').forEach(b => b.classList.toggle('active', b.dataset.view === '2d'));
    }
    cv.style.background = 'transparent';
    if (!S.map) {
      S.map = L.map('map', { zoomControl: false, attributionControl: true }).setView([S.geo.lat0, S.geo.lng0], 18);
      // maxNativeZoom = highest zoom with real tiles; Leaflet upscales beyond it
      // (blurry but visible) instead of showing "Map data not yet available".
      S.tileStreet = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 23, maxNativeZoom: 19, attribution: '© OpenStreetMap' });
      S.tileSat = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', { maxZoom: 23, maxNativeZoom: 19, attribution: 'Tiles © Esri' });
      S.tileSat.addTo(S.map);
      S.bases = { esri: S.tileSat, osm: S.tileStreet }; S.baseKey = 'esri'; S.twOv = {};
      L.control.scale({ imperial: true, metric: true, position: 'bottomleft' }).addTo(S.map);
      S.map.on('move zoom moveend zoomend resize', () => draw());
    }
    if (!S.geo.set) { const c = S.map.getCenter(); S.geo.lat0 = c.lat; S.geo.lng0 = c.lng; S.geo.set = true; }
    setTimeout(() => { S.map.invalidateSize(); if (S.boundary.length >= 2) fitMapToGeom(); draw(); }, 60);
    setTimeout(() => { S.map.invalidateSize(); draw(); }, 350);     // robust against late layout
    if (S.geo.firstOpen) {            // first time: snap precisely to the default 南科 address
      S.geo.firstOpen = false;
      setTimeout(() => geocode($('#addrInput').value), 200);
    }
  } else {
    cv.style.background = ''; $('#twPanel').classList.remove('show'); fitView();
  }
  draw();
}
function fitMapToGeom() {
  const all = [].concat(S.boundary, ...bPolys(), ...S.obstacles);   // buildings are objects → use their polys
  if (all.length < 2 || !S.map) return;
  S.map.fitBounds(all.map(p => { const l = feetToLatLng(p); return [l.lat, l.lng]; }), { padding: [70, 70], maxZoom: 20 });
}
async function geocode(q) {
  if (!q || !S.map) return;
  try {
    const r = await fetch('https://nominatim.openstreetmap.org/search?format=json&limit=1&q=' + encodeURIComponent(q), { headers: { 'Accept-Language': 'zh-TW,en' } });
    const j = await r.json();
    if (j && j[0]) {
      const lat = +j[0].lat, lng = +j[0].lon;
      // nothing drawn yet → move the local feet-origin here so traced coords stay clean
      if (!S.boundary.length && !S.buildings.length && !S.obstacles.length) { S.geo.lat0 = lat; S.geo.lng0 = lng; S.geo.set = true; }
      S.map.setView([lat, lng], 18); toast('已移至：' + (j[0].display_name || q).slice(0, 46)); draw();
    } else toast('找不到「' + q + '」');
  } catch (err) { toast('地址查詢失敗（可能無網路）'); }
}
/* --- Taiwan NLSC (國土測繪中心) free WMTS layers — GoogleMapsCompatible = z/x/y --- */
const NLSC_ATTR = '© 內政部國土測繪中心';
function nlscUrl(layer) { return `https://wmts.nlsc.gov.tw/wmts/${layer}/default/GoogleMapsCompatible/{z}/{y}/{x}`; }
function ensureBase(key) {
  S.bases = S.bases || {};
  if (S.bases[key]) return S.bases[key];
  let lyr;
  if (key === 'photo2') lyr = L.tileLayer(nlscUrl('PHOTO2'), { maxZoom: 23, maxNativeZoom: 20, attribution: '台灣空照 ' + NLSC_ATTR });
  else if (key === 'emap') lyr = L.tileLayer(nlscUrl('EMAP'), { maxZoom: 23, maxNativeZoom: 20, attribution: '電子地圖 ' + NLSC_ATTR });
  else lyr = key === 'osm' ? S.tileStreet : S.tileSat;
  S.bases[key] = lyr; return lyr;
}
function setBase(key) {
  if (!S.map) return;
  ['esri', 'osm', 'photo2', 'emap'].forEach(k => { const l = S.bases && S.bases[k]; if (l && S.map.hasLayer(l)) S.map.removeLayer(l); });
  const lyr = ensureBase(key); lyr.addTo(S.map); lyr.bringToBack(); S.baseKey = key;
  document.querySelectorAll('#twPanel input[name=twbase]').forEach(r => r.checked = r.value === key);
  $('#tSat').classList.toggle('active', key === 'esri'); $('#tStreet').classList.toggle('active', key === 'osm');
}
function setTile(sat) { setBase(sat ? 'esri' : 'osm'); }   // 衛星/街道 quick buttons
const OV_OPACITY = { LANDSECT: 0.9, MOI_CONTOUR: 0.85, SoilLiquefaction: 0.55, MOI_SLOPEP_GT30: 0.5, LUIMAP: 0.55 };
function setOverlay(layer, on) {
  if (!S.map) return;
  S.twOv = S.twOv || {};
  if (on) {
    if (!S.twOv[layer]) S.twOv[layer] = L.tileLayer(nlscUrl(layer), { maxZoom: 23, maxNativeZoom: 20, opacity: OV_OPACITY[layer] || 0.6, attribution: NLSC_ATTR });
    S.twOv[layer].addTo(S.map); S.twOv[layer].bringToFront();
  } else if (S.twOv[layer]) S.map.removeLayer(S.twOv[layer]);
}
$('#btnTwLayers').onclick = () => {
  if (typeof L === 'undefined') { toast('地圖需要網路連線'); return; }
  if (!S.mapMode) enableMap(true);
  $('#twPanel').classList.toggle('show');
};
document.querySelectorAll('#twPanel input[name=twbase]').forEach(r => r.addEventListener('change', () => setBase(r.value)));
document.querySelectorAll('#twPanel input[data-ov]').forEach(c => c.addEventListener('change', () => setOverlay(c.dataset.ov, c.checked)));
$('#btnMap').onclick = () => enableMap(!S.mapMode);
$('#btnFlow').onclick = () => toggleLayer('flow', 'vis');   // 動線體檢 is a toggleable layer (also in the object tree)
$('#btnMetes').onclick = openMetesModal;
$('#btnContext').onclick = fetchContextBuildings;
$('#btnCloud').onclick = openCloudModal;
$('#panelToggle').onclick = () => { const open = $('#panel').classList.toggle('open'); $('#panelToggle').innerHTML = open ? '✕ 收起面板' : '⚙ 參數'; };
$('#addrGo').onclick = () => geocode($('#addrInput').value);
$('#addrInput').addEventListener('keydown', e => { if (e.key === 'Enter') { e.stopPropagation(); geocode($('#addrInput').value); } });
$('#tSat').onclick = () => setTile(true);
$('#tStreet').onclick = () => setTile(false);
$('#mapTrace').onclick = () => { setTool('boundary'); toast('在地圖上點出基地各角，按 Enter 或點回起點收尾'); };

/* --------------------------- entrance type popup ------------------------- */
// Appears at an entrance when you click it — makes IN / OUT / two-way discoverable.
function showEntPopup(eh) {
  if (!eh || S.is3d) return;
  const pop = $('#entPopup'), s = toScreen(eh), stage = $('#stage');
  pop.classList.add('show');
  const w = pop.offsetWidth || 180, h = pop.offsetHeight || 34;   // popup is translate(-50%,-150%)
  const left = Math.max(w / 2 + 6, Math.min(stage.clientWidth - w / 2 - 6, s.x));
  const top = Math.max(h * 1.5 + 6, s.y);                          // keep below the toolbar
  pop.style.left = left + 'px'; pop.style.top = top + 'px';
  const t = eh.type || 'inout';
  pop.querySelectorAll('button').forEach(b => b.classList.toggle('active', b.dataset.t === t));
}
function hideEntPopup() { $('#entPopup').classList.remove('show'); }
$('#entPopup').querySelectorAll('button').forEach(b => b.addEventListener('mousedown', e => {
  e.stopPropagation(); e.preventDefault();
  if (!S.selEntrance) return;
  if (b.dataset.t === 'del') {
    S.entrances.splice(S.entrances.indexOf(S.selEntrance), 1); S.selEntrance = null;
    hideEntPopup(); draw(); resolveActive(); toast('已刪除出入口'); return;
  }
  S.selEntrance.type = b.dataset.t;
  showEntPopup(S.selEntrance); draw(); resolveActive();
  toast('出入口：' + ({ in: '只進 ▸', out: '只出 ◂', inout: '進出 ⇄' })[b.dataset.t]);
}));

/* per-edge setback popup (site mode — click any boundary edge) */
function showEdgePopup(i) {
  const pop = $('#edgePopup'); if (!pop || S.is3d || S.mode !== 'site') return;
  const A = toScreen(S.boundary[i]), B = toScreen(S.boundary[(i + 1) % S.boundary.length]), stage = $('#stage');
  const mx = (A.x + B.x) / 2, my = (A.y + B.y) / 2;
  pop.classList.add('show');
  const w = pop.offsetWidth || 160, h = pop.offsetHeight || 58;
  pop.style.left = Math.max(w / 2 + 6, Math.min(stage.clientWidth - w / 2 - 6, mx)) + 'px';
  pop.style.top = Math.max(h, my) + 'px';
  $('#edgeSbUnit').textContent = U.lu();
  $('#edgeSbInput').value = S.edgeSetback[i] != null ? round2(U.L(S.edgeSetback[i])) : '';
  setTimeout(() => $('#edgeSbInput').focus(), 0);
}
function hideEdgePopup() { const p = $('#edgePopup'); if (p) p.classList.remove('show'); }
$('#edgeSbApply').onclick = () => {
  if (S.selEdge == null) return;
  const v = $('#edgeSbInput').value;
  if (v === '' || +v < 0) delete S.edgeSetback[S.selEdge]; else S.edgeSetback[S.selEdge] = U.Lr(+v);
  hideEdgePopup(); S.selEdge = null;
  if (S.boundary.length >= 3) doSolveSite(); else draw();
  toast('已更新該邊退縮');
};
$('#edgeSbReset').onclick = () => {
  if (S.selEdge == null) return;
  delete S.edgeSetback[S.selEdge]; hideEdgePopup(); S.selEdge = null;
  if (S.boundary.length >= 3) doSolveSite(); else draw();
  toast('該邊退縮恢復自動');
};
$('#edgeSbInput').addEventListener('keydown', e => { if (e.key === 'Enter') { e.stopPropagation(); $('#edgeSbApply').click(); } });

// road editor popup — width presets + delete (acts on the selected road S.selRoad)
$('#roadPopup').querySelectorAll('[data-rw]').forEach(b => b.onclick = () => {
  if (S.selRoad == null || !S.roadLines[S.selRoad]) return;
  S.roadLines[S.selRoad].width = ROAD_W[b.dataset.rw];
  roadChanged(); showRoadPopup(S.selRoad);
  toast('已調整道路寬度');
});
$('#roadDel').onclick = () => {
  if (S.selRoad == null || !S.roadLines[S.selRoad]) return;
  S.roadLines.splice(S.selRoad, 1); S.selRoad = null; hideRoadPopup(); roadChanged();
  toast('已刪除道路');
};
// drive-aisle editor buttons (manual circulation editing)
$('#aisleRemove').onclick = () => { if (S.selAisle != null) removeAisle(S.selAisle); };
$('#aisleSingle').onclick = () => { if (S.selAisle != null) singleLoadAisle(S.selAisle); };
$('#aisleFlip').onclick = () => { if (S.selAisle != null) flipAisle(S.selAisle); };
$('#aisleDouble').onclick = () => { if (S.selAisle != null) restoreAisle(S.selAisle); };

/* ------------------- modal: generative options + compare ----------------- */
const esc = s => String(s == null ? '' : s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
// METES & BOUNDS: trace a survey description (quadrant bearing + distance per line) into a boundary polygon.
function parseMetes(text) {
  const pts = [{ x: 0, y: 0 }];
  for (const ln of String(text).split(/\n/)) {
    const m = ln.match(/([NS])\s*([\d.]+)(?:[°\s]+([\d.]+)\s*['′]?)?\s*([EW])[\s,]+([\d.]+)/i);
    if (!m) continue;
    const ns = m[1].toUpperCase(), deg = +m[2] + (m[3] ? +m[3] / 60 : 0), ew = m[4].toUpperCase(), dist = +m[5];
    const h = ns === 'N' ? (ew === 'E' ? deg : 360 - deg) : (ew === 'E' ? 180 - deg : 180 + deg);   // heading CW from north
    const r = h * Math.PI / 180, last = pts[pts.length - 1];
    pts.push({ x: last.x + dist * Math.sin(r), y: last.y + dist * Math.cos(r) });                    // north = +y
  }
  if (pts.length > 2) { const a = pts[0], b = pts[pts.length - 1]; if (Math.hypot(a.x - b.x, a.y - b.y) < 1) pts.pop(); }   // closes back to start
  return pts;
}
function openMetesModal() {
  openModal('📐 測量描述 Metes & Bounds', `
    <div class="hint" style="margin-bottom:6px">每行一段：方位角 + 距離（呎）。例：<code>N45E 120</code>、<code>S 30°15' W 80.5</code>。從起點順描述自動接成地界。</div>
    <textarea id="mbText" style="width:100%;height:148px;background:#0f1a2b;border:1px solid var(--line);color:var(--text);border-radius:6px;padding:8px;font-size:13px;font-family:monospace" placeholder="N 45 E 120&#10;S 45 E 100&#10;S 45 W 120&#10;N 45 W 100"></textarea>
    <button class="btn primary" id="mbGen" style="width:100%;justify-content:center;margin-top:8px">依描述產生地界</button>
    <div class="hint" id="mbErr" style="color:#fca5a5;margin-top:6px"></div>`);
  $('#mbGen').onclick = () => {
    const pts = parseMetes($('#mbText').value);
    if (pts.length < 3) { $('#mbErr').textContent = '至少 3 段才能組成地界（格式：N45E 100，每行一段）。'; return; }
    S.site = null; S.solution = null; S.boundary = pts; S.buildings = []; S.obstacles = []; S.roads = []; S.roadLines = []; S.parkZones = []; S.manualCores = []; S.gridShift = null; S.aisleEdits = null;
    S.entrances = [{ x: (pts[0].x + pts[1].x) / 2, y: (pts[0].y + pts[1].y) / 2, type: 'inout' }];
    closeModal(); fitView(); commit();
    toast(`已依測量描述產生 ${pts.length} 角地界`);
    if (S.mode === 'site') doSolveSite(); else if (S.solution !== null) doSolve();
  };
}
function openModal(title, html) { $('#modalTitle').textContent = title; $('#modalBody').innerHTML = html; $('#modal').classList.add('show'); }
function closeModal() { $('#modal').classList.remove('show'); }
$('#modalClose').onclick = closeModal;
$('#modal').addEventListener('mousedown', e => { if (e.target.id === 'modal') closeModal(); });

/* generative design — auto-run several configs, rank by KPI, apply the best */
let _genRaw = [], _genSorted = [];
// render a tiny floor-plan thumbnail of an option to a data URL (Preview)
function thumbFor(sol, footprint) {
  const W = 188, H = 116, pad = 7;
  const tc = document.createElement('canvas'); tc.width = W; tc.height = H;
  const x = tc.getContext('2d');
  x.fillStyle = '#0b1426'; x.fillRect(0, 0, W, H);
  if (S.boundary.length < 3) return tc.toDataURL();
  const all = S.boundary.concat(footprint || []);
  const bb = PS.bbox(all);
  const w = Math.max(bb.maxX - bb.minX, 1), h = Math.max(bb.maxY - bb.minY, 1);
  const sc = Math.min((W - pad * 2) / w, (H - pad * 2) / h);
  const ox = pad + (W - pad * 2 - w * sc) / 2 - bb.minX * sc, oy = pad + (H - pad * 2 - h * sc) / 2 - bb.minY * sc;
  const path = (poly, close) => { x.beginPath(); poly.forEach((p, i) => { const sx = p.x * sc + ox, sy = p.y * sc + oy; i ? x.lineTo(sx, sy) : x.moveTo(sx, sy); }); if (close) x.closePath(); };
  path(S.boundary, true); x.fillStyle = 'rgba(56,189,248,.06)'; x.fill(); x.strokeStyle = '#38bdf8'; x.lineWidth = 1; x.stroke();
  if (sol) for (const st of sol.stalls) { path(st.poly, true); x.fillStyle = hexA(COLORS[st.type] || COLORS.standard, .85); x.fill(); }
  if (footprint && footprint.length) { path(footprint, true); x.fillStyle = 'rgba(56,189,248,.42)'; x.fill(); x.strokeStyle = '#38bdf8'; x.stroke(); }
  return tc.toDataURL();
}
function renderGenGrid() {
  const key = ($('#genSort') && $('#genSort').value) || (S.mode === 'site' ? 'k_park' : 'k_count');
  const onlyPass = $('#genPass') && $('#genPass').checked;
  _genSorted = _genRaw.filter(o => !onlyPass || o.pass !== false).slice()
    .sort((x, y) => (y[key] || 0) - (x[key] || 0)).slice(0, 9);
  $('#genGrid').innerHTML = _genSorted.map((o, i) => `<div class="optcard ${i === 0 ? 'best' : ''}">
      <div class="rank">${i === 0 ? '★ 最佳' : '#' + (i + 1)}${o.pass === false ? ' · ⚠ 停車不足' : ''}</div>
      ${o.thumb ? `<img class="optthumb" src="${o.thumb}" alt="">` : ''}
      <div class="big">${esc(o.big)}</div><div class="sub">${o.sub}</div>
      <button class="apply" data-i="${i}">套用此方案</button></div>`).join('')
    || '<div class="hint">沒有符合條件的方案。</div>';
  $('#genGrid').querySelectorAll('.apply').forEach(b => b.onclick = () => applyOption(_genSorted[+b.dataset.i]));
}
function generateOptions() {
  readParams();
  if (S.boundary.length < 3) { toast('請先畫出基地邊界'); return; }
  $('#busy').classList.add('show'); $('#busyTxt').textContent = '產生多個方案中…';
  setTimeout(() => {
    const out = [];
    if (S.mode === 'site') {
      const pbase = readSiteParams();
      const resi = ['multifamily', 'singlefamily', 'mixeduse'].includes(pbase.useType);
      // vary unit-mix bias so units / yield genuinely differ (real KPI tradeoffs)
      const biases = resi ? [
        { name: '小坪密集', mix: [['studio', 30, 450], ['1br', 50, 650], ['2br', 15, 950], ['3br', 5, 1200]] },
        { name: '均衡', mix: null },
        { name: '大坪', mix: [['studio', 5, 550], ['1br', 25, 750], ['2br', 45, 1050], ['3br', 25, 1350]] },
      ] : [{ name: '', mix: null }];
      // sweep PARKING TYPE too (surface vs structured garage) — surfaces a real decision:
      // "surface can't meet the ratio → a 3-level deck does, at this yield"
      const parkCfgs = [
        { name: '地面停車', park: 'surface' },
        { name: '車庫地上3層', park: 'structured', above: 3, below: 0 },
        { name: '車庫地上2+地下2', park: 'structured', above: 2, below: 2 },
      ];
      for (const b of biases) for (const pc of parkCfgs) for (const a of [90, 60]) {
        const um = b.mix ? b.mix.map(m => ({ type: m[0], pct: m[1], size: m[2] })) : pbase.unitMix;
        const p = { ...pbase, parkAngle: a, unitMix: um, parkingType: pc.park,
          parkingLevelsAbove: pc.above != null ? pc.above : 0, parkingLevelsBelow: pc.below != null ? pc.below : 0 };
        const sol = PS.solveSite({ boundary: S.boundary, p, entrances: S.entrances, obstacles: S.obstacles, roads: S.roads });
        if (sol) out.push({
          kind: 'site', parkAngle: a, unitMix: um, park: pc.park, above: pc.above, below: pc.below,
          kpi: sol.parkingProvided, thumb: thumbFor(sol.parkSol, sol.footprint),
          k_park: sol.parkingProvided, k_yield: sol.fin.yieldOnCost, k_units: sol.units,
          big: sol.residential ? sol.units + ' 戶' : Math.round(U.A(sol.gfa)).toLocaleString() + ' ' + U.au(),
          sub: `${pc.name}${b.name ? ' · ' + b.name : ''} · ${a}°<br>提供 ${sol.parkingProvided}/${sol.parkingRequired}<br>Yield ${sol.fin.yieldOnCost.toFixed(1)}%`,
          pass: sol.parkingProvided >= sol.parkingRequired,
        });
      }
    } else {
      const seen = new Set();
      for (const o of ['auto', '0', '90', 'edge']) for (const a of [90, 60, 45]) {
        const sol = PS.solve({ boundary: S.boundary, buildings: bPolys(), obstacles: S.obstacles, roads: S.roads, parkZones: S.parkZones, entrances: S.entrances, params: { ...S.params, orient: o, angle: a }, opts: S.opts });
        if (!sol) continue;
        const key = sol.count + '_' + a; if (seen.has(key)) continue; seen.add(key);
        out.push({
          kind: 'parking', orient: o, angle: a, kpi: sol.count, k_count: sol.count, big: sol.count + ' 車位', thumb: thumbFor(sol),
          sub: `${({ auto: '自動最佳', 0: '水平 0°', 90: '垂直 90°', edge: '沿最長邊' })[o]}<br>停車角度 ${a}°<br>配置 ${sol.metrics.bestAngleDeg}°`,
        });
      }
    }
    _genRaw = out;
    $('#busy').classList.remove('show');
    if (!_genRaw.length) { toast('無法產生方案'); return; }
    // custom KPI sorting (TestFit-style): pick which KPI ranks the options
    const sorts = S.mode === 'site'
      ? [['k_park', '提供車位'], ['k_yield', 'Yield 報酬'], ['k_units', '戶數']]
      : [['k_count', '車位數']];
    const sel = `<div style="display:flex;align-items:center;gap:8px;margin-bottom:12px;font-size:12px;color:var(--muted);">
        排序依據 KPI：<select id="genSort" style="background:#0f1a2b;border:1px solid var(--line);color:var(--text);border-radius:6px;padding:5px 8px;">
        ${sorts.map(s => `<option value="${s[0]}">${s[1]}</option>`).join('')}</select>
        <label style="margin-left:auto;display:flex;align-items:center;gap:5px;"><input type="checkbox" id="genPass"> 只看停車達標</label></div>`;
    openModal(`產生方案 — ${_genRaw.length} 個（可自選 KPI 排序）`, sel + '<div class="optgrid" id="genGrid"></div>');
    renderGenGrid();
    $('#genSort').onchange = renderGenGrid;
    $('#genPass').onchange = renderGenGrid;
  }, 30);
}
function applyOption(o) {
  if (o.kind === 'parking') {
    $('#pOrient').value = o.orient; S.params.angle = o.angle;
    document.querySelectorAll('#angleSeg button').forEach(b => b.classList.toggle('active', +b.dataset.ang === o.angle));
    $('#pA').value = round2(U.L(aisleForAngle(o.angle, S.params.oneway)));   // pD stays the real stall depth
    closeModal(); doSolve();
  } else {
    S.siteParkAngle = o.parkAngle;
    if (o.park) {                                     // apply the option's parking type + levels
      $('#sParkType').value = o.park;
      const structured = o.park === 'structured';
      ['#rowParkLevels', '#rowParkBelow', '#rowStructEff', '#parkTypeHint'].forEach(id => $(id).style.display = structured ? '' : 'none');
      if (structured) { $('#sParkLevelsAbove').value = o.above != null ? o.above : 3; $('#sParkLevelsBelow').value = o.below != null ? o.below : 0; }
    }
    if (o.unitMix) {                                  // apply the option's unit-mix bias to the inputs
      const m = {}; o.unitMix.forEach(u => m[u.type] = u);
      const set = (t, p, s) => { if (m[t]) { $(p).value = m[t].pct; $(s).value = Math.round(U.A(m[t].size)); } };
      set('studio', '#uxStudioP', '#uxStudioS'); set('1br', '#ux1P', '#ux1S');
      set('2br', '#ux2P', '#ux2S'); set('3br', '#ux3P', '#ux3S');
      updateUxSum();
    }
    closeModal(); doSolveSite();
  }
  toast('已套用方案');
}

/* PDF / printable feasibility report */
function exportReport() {
  const area = S.boundary.length >= 3 ? PS.polyArea(S.boundary) : 0;
  const fmtA = n => Math.round(U.A(n)).toLocaleString() + ' ' + U.au();
  let title, rows = [], comp = '';
  if (S.mode === 'site' && S.site) {
    const s = S.site; title = '建案可行性報告 Site Feasibility';
    rows = [['用途 Use', $('#sUse').selectedOptions[0].text], ['樓層 Floors', s.floors + ' F'],
      ['建築高度 Height', Math.round(U.L(s.height)) + ' ' + U.lu()], ['總樓地板 GFA', fmtA(s.gfa)], ['可租 NRSF', fmtA(s.nrsf)],
      ['容積率 FAR', s.far.toFixed(2)], ['建蔽率 Coverage', s.coverage.toFixed(0) + '%']];
    if (s.residential) rows.push(['戶數 Units', s.units], ['密度 Density', (s.units / s.acres).toFixed(1) + ' DU/ac']);
    rows.push(['停車 供/需 Parking', `${s.parkingProvided} / ${s.parkingRequired}`],
      ['土建成本 Cost', '$' + Math.round(s.fin.totalCost).toLocaleString()], ['NOI', '$' + Math.round(s.fin.noi).toLocaleString() + ' /yr'],
      ['Yield on Cost', s.fin.yieldOnCost.toFixed(2) + '%']);
    comp = '<h3>法規檢核 Zoning Compliance</h3><table>' +
      s.compliance.map(c => `<tr><td>${esc(c.k)}</td><td>${c.ok ? '✅ 通過' : '❌ 不符'}</td><td>${esc(c.val)}</td></tr>`).join('') + '</table>';
  } else if (S.solution) {
    const sol = S.solution; title = '停車場可行性報告 Parking Feasibility';
    const counts = { standard: 0, compact: 0, ada: 0, ev: 0, trailer: 0 }; sol.stalls.forEach(s => counts[s.type]++);
    rows = [['總車位 Total Stalls', sol.stalls.length], ['標準 Standard', counts.standard], ['⚡ EV', counts.ev],
      ['♿ ADA', counts.ada], ['配置角度 Angle', sol.metrics.bestAngleDeg + '°'],
      ['基地面積 Area', `${fmtA(area)} (${U.big(area).toFixed(2)} ${U.bu()})`]];
    if (S.opts.target > 0) rows.push(['🎯 目標 Target', `${sol.stalls.length} / ${S.opts.target} ${sol.stalls.length >= S.opts.target ? '✅' : '❌'}`]);
  } else { toast('尚無結果可做報告'); return; }
  const img = cv.toDataURL('image/png');
  const date = new Date().toISOString().slice(0, 10);
  const rowsHtml = rows.map(r => `<tr><td>${esc(r[0])}</td><td>${esc(r[1])}</td></tr>`).join('');
  const html = `<!DOCTYPE html><html lang="zh-Hant"><head><meta charset="utf-8"><title>${esc(title)}</title>
<style>body{font-family:system-ui,'Noto Sans TC',sans-serif;margin:32px;color:#0f172a;}
h1{font-size:20px;margin:0 0 4px;}.meta{color:#64748b;font-size:12px;margin-bottom:14px;}
img{width:100%;max-width:760px;border:1px solid #cbd5e1;border-radius:8px;margin:6px 0 14px;background:#0b1426;}
table{border-collapse:collapse;width:100%;max-width:760px;font-size:13px;margin-bottom:14px;}
td,th{border:1px solid #cbd5e1;padding:6px 10px;}td:first-child{color:#475569;}td:last-child{text-align:right;font-weight:600;}
h3{font-size:14px;margin:14px 0 6px;}.foot{color:#94a3b8;font-size:11px;margin-top:18px;}
@media print{.noprint{display:none;}}</style></head><body>
<h1>${esc(title)}</h1>
<div class="meta">地點：${esc($('#addrInput') ? $('#addrInput').value : '')} ｜ 日期：${date} ｜ 面積：${fmtA(area)} (${U.big(area).toFixed(2)} ${U.bu()})</div>
<img src="${img}"><table>${rowsHtml}</table>${comp}
<div class="foot">由 TestFit Clone 產生 · 概念性可行性估算，非正式工程／法律文件</div>
<button class="noprint" onclick="window.print()" style="margin-top:8px;padding:9px 18px;font-size:13px;cursor:pointer;">🖨️ 列印 / 存成 PDF</button>
</body></html>`;
  const url = URL.createObjectURL(new Blob([html], { type: 'text/html' }));   // no document.write
  const w = window.open(url, '_blank');
  if (!w) { toast('請允許彈出視窗以開啟報告'); URL.revokeObjectURL(url); return; }
  setTimeout(() => URL.revokeObjectURL(url), 60000);
  toast('報告已開啟 — 用瀏覽器「列印 → 存成 PDF」');
}

/* side-by-side scheme compare */
function compareSchemes() {
  const arr = getSchemes();
  if (arr.length < 2) { toast('至少要 2 個已儲存方案才能比較'); return; }
  const metric = sc => {
    const d = sc.data || {};
    if (sc.mode === 'site' && d.site) {
      const s = d.site;
      return { '類型': '🏢 建案', '車位數': '—', '戶數/GFA': s.residential ? s.units + ' 戶' : Math.round(s.gfa).toLocaleString() + ' SF',
        FAR: s.far.toFixed(2), '樓層': s.floors + 'F', '建蔽率': s.coverage.toFixed(0) + '%',
        '停車 供/需': `${s.parkingProvided}/${s.parkingRequired}`, Yield: s.fin.yieldOnCost.toFixed(1) + '%' };
    }
    const sol = d.solution;
    return { '類型': '🅿️ 停車', '車位數': sol ? sol.stalls.length : (sc.summary || '—'),
      '戶數/GFA': '—', FAR: '—', '樓層': '—', '建蔽率': '—',
      '停車 供/需': '—', Yield: '—', '配置角度': sol && sol.metrics ? sol.metrics.bestAngleDeg + '°' : '—' };
  };
  const cols = arr.map(metric);
  const keys = ['類型', '車位數', '配置角度', '戶數/GFA', 'FAR', '樓層', '建蔽率', '停車 供/需', 'Yield'];
  let html = '<table class="cmptable"><tr><th>項目</th>' + arr.map(sc => `<th>${esc((sc.mode === 'site' ? '🏢 ' : '🅿️ ') + sc.name)}</th>`).join('') + '</tr>';
  for (const k of keys) {
    if (!cols.some(c => c[k] != null && c[k] !== '—')) continue;
    html += `<tr><td>${esc(k)}</td>` + cols.map(c => `<td>${esc(c[k] == null ? '—' : c[k])}</td>`).join('') + '</tr>';
  }
  html += '</table><div style="font-size:11px;color:#94a3b8;margin-top:10px;">日期：' + arr.map(s => esc(s.date)).join(' ｜ ') + '</div>';
  openModal('方案比較（' + arr.length + ' 個）', html);
}

/* IRR sensitivity grid — IRR vs cost (rows) × rent (cols), each ±10% */
function buildSensitivityHTML(s) {
  if (!s || !s.fin || s.fin.totalCost <= 0) return '';
  const baseF = readSiteParams().fin;
  const m = { gfa: s.gfa, nrsf: s.nrsf, units: s.units, residential: s.residential };
  const deltas = [-10, 0, 10];
  const cell = (dc, dr) => {
    const f = { ...baseF, hardCost: baseF.hardCost * (1 + dc / 100), rentMo: baseF.rentMo * (1 + dr / 100), rentSfYr: baseF.rentSfYr * (1 + dr / 100) };
    const fin = PS.computeFinancials(f, m);
    return fin.irr == null ? '—' : fin.irr.toFixed(1) + '%';
  };
  let h = '<h3 style="font-size:13px;margin:16px 0 6px;color:var(--accent);">敏感度 — IRR（直欄＝租金；橫列＝營造成本）</h3>';
  h += '<table class="cmptable" style="min-width:340px;"><tr><th>成本＼租金</th>' + deltas.map(d => `<th>${d > 0 ? '+' : ''}${d}%</th>`).join('') + '</tr>';
  for (const dc of deltas) {
    h += `<tr><td>${dc > 0 ? '+' : ''}${dc}%</td>` + deltas.map(dr => {
      const base = dc === 0 && dr === 0;
      return `<td style="${base ? 'background:rgba(56,189,248,.18);font-weight:700;color:#fff;' : ''}">${cell(dc, dr)}</td>`;
    }).join('') + '</tr>';
  }
  return h + '</table><div style="font-size:11px;color:#94a3b8;margin-top:6px;">中央藍格＝目前方案；未槓桿 IRR，持有 ' + s.fin.holdYears + ' 年。</div>';
}

/* build the itemised [label, value] takeoff rows — shared by the modal, PDF report & Excel */
function takeoffRows() {
  const fmtA = n => Math.round(U.A(n)).toLocaleString() + ' ' + U.au();
  const area = S.boundary.length >= 3 ? PS.polyArea(S.boundary) : 0;
  const rows = [];
  const hdr = t => rows.push([t, '']);
  if (S.mode === 'site' && S.site) {
    const s = S.site;
    hdr('面積 Areas');
    rows.push(['基地 Site', `${fmtA(area)} (${U.big(area).toFixed(2)} ${U.bu()})`]);
    rows.push(['可建範圍 Buildable', fmtA(PS.polyArea(s.envelope))]);
    rows.push(['建築佔地 Footprint', fmtA(PS.polyArea(s.footprint))]);
    rows.push(['樓層 Floors', s.floors]);
    rows.push(['總樓地板 GFA', fmtA(s.gfa)]);
    rows.push(['可租/售 NRSF', fmtA(s.nrsf)]);
    rows.push(['得房率 Efficiency', Math.round(s.nrsf / s.gfa * 100) + '%']);
    if (s.residential) {
      hdr('單元 Unit Mix');
      s.unitsByType.forEach(u => { if (u.count) rows.push([UNIT_LABEL[u.type], `${u.count} 戶 × ${Math.round(U.A(u.size))} = ${fmtA(u.count * u.size)}`]); });
      rows.push(['總戶數 Total Units', s.units]);
      rows.push(['密度 Density', (s.units / s.acres).toFixed(1) + ' DU/ac']);
      let beds = 0, baths = 0;
      s.unitsByType.forEach(u => { beds += (BEDS[u.type] || 0) * u.count; baths += (BATHS[u.type] || 0) * u.count; });
      rows.push(['總臥室 Total Beds', beds.toLocaleString()]);
      rows.push(['總衛浴 Total Baths', baths.toLocaleString()]);
    }
    hdr('停車 Parking');
    rows.push(['提供 Provided', s.parkingProvided]);
    rows.push(['需求 Required', s.parkingRequired]);
    rows.push(['達標', s.parkingProvided >= s.parkingRequired ? '✅' : '❌ 差 ' + (s.parkingRequired - s.parkingProvided)]);
    hdr('成本 Cost');
    rows.push(['土地 Land', '$' + Math.round(s.fin.landCost).toLocaleString()]);
    rows.push(['營造 Hard', '$' + Math.round(s.fin.hard).toLocaleString()]);
    rows.push(['軟成本 Soft', '$' + Math.round(s.fin.soft).toLocaleString()]);
    rows.push(['總成本 Total', '$' + Math.round(s.fin.totalCost).toLocaleString()]);
    hdr('收益 Revenue');
    rows.push(['年收入 Annual', '$' + Math.round(s.fin.annualRevenue).toLocaleString()]);
    rows.push(['NOI', '$' + Math.round(s.fin.noi).toLocaleString() + ' /yr']);
    rows.push(['Yield on Cost', s.fin.yieldOnCost.toFixed(2) + '%']);
    hdr('投資報酬 Returns');
    rows.push([`退場價值 Exit Value (${$('#fExitCap').value}% cap)`, '$' + Math.round(s.fin.exitValue).toLocaleString()]);
    rows.push([`IRR · ${s.fin.holdYears} 年未槓桿`, s.fin.irr == null ? '—' : s.fin.irr.toFixed(1) + '%']);
    rows.push(['權益倍數 Equity Multiple', s.fin.equityMultiple.toFixed(2) + 'x']);
  } else if (S.solution) {
    const sol = S.solution, cc = { standard: 0, compact: 0, ev: 0, ada: 0, trailer: 0 };
    sol.stalls.forEach(x => cc[x.type]++);
    const moduleArea = S.params.stallW * (S.params.stallD + S.params.aisle / 2);
    hdr('停車 Parking Takeoff');
    rows.push(['基地 Site', `${fmtA(area)} (${U.big(area).toFixed(2)} ${U.bu()})`]);
    rows.push(['總車位 Total', sol.stalls.length]);
    rows.push(['標準 Standard', cc.standard]);
    if (cc.ev) rows.push(['⚡ EV', cc.ev]);
    if (cc.ada) rows.push(['♿ ADA', cc.ada]);
    if (cc.compact) rows.push(['小型 Compact', cc.compact]);
    rows.push(['配置角度 Angle', sol.metrics.bestAngleDeg + '°']);
    rows.push(['車道循環', S.params.oneway ? '單向 One-way' : '雙向 Two-way']);
    rows.push(['單格用地 Area/Stall', fmtA(moduleArea)]);
    rows.push(['停車覆蓋率 Coverage', Math.round(sol.stalls.length * moduleArea / area * 100) + '%']);
    if (S.opts.target > 0) rows.push(['🎯 目標 Target', `${sol.stalls.length} / ${S.opts.target} ${sol.stalls.length >= S.opts.target ? '✅' : '❌'}`]);
  } else return null;
  return { rows, area };
}

function quantityTakeoff() {
  const t = takeoffRows();
  if (!t) { toast('尚無結果可計算'); return; }
  const html = '<table class="cmptable" style="min-width:440px;">' + t.rows.map(r => r[1] === ''
    ? `<tr><td colspan="2" style="background:#1a2638;color:var(--accent);text-align:center;font-weight:700;">—— ${esc(r[0])} ——</td></tr>`
    : `<tr><td>${esc(r[0])}</td><td>${esc(r[1])}</td></tr>`).join('') + '</table>';
  const sens = (S.mode === 'site' && S.site) ? buildSensitivityHTML(S.site) : '';
  openModal('📊 數量計算 Quantity Takeoff', html + sens);
}

/* lazy-load a vendored library on first use (keeps boot light; core app stays offline-capable) */
function loadVendor(src) {
  return new Promise((resolve, reject) => {
    window.__vendor = window.__vendor || {};
    if (window.__vendor[src]) return resolve();
    const sc = document.createElement('script');
    sc.src = src;
    sc.onload = () => { window.__vendor[src] = 1; resolve(); };
    sc.onerror = () => reject(new Error('load failed: ' + src));
    document.head.appendChild(sc);
  });
}

/* real Excel .xlsx via SheetJS — multi-sheet: takeoff + stall list + IRR sensitivity */
async function exportXLSX() {
  const t = takeoffRows();
  if (!t) { toast('尚無結果可匯出'); return; }
  try { await loadVendor('vendor_xlsx.min.js'); }
  catch { toast('Excel 元件載入失敗（首次需網路下載一次）'); return; }
  const XLSX = window.XLSX;
  const wb = XLSX.utils.book_new();
  const main = [['項目 Item', '數值 Value'], ...t.rows.map(r => [r[0], r[1] === '' ? '' : String(r[1])])];
  const ws = XLSX.utils.aoa_to_sheet(main); ws['!cols'] = [{ wch: 36 }, { wch: 30 }];
  XLSX.utils.book_append_sheet(wb, ws, '數量計算');
  const park = activePark();
  if (park && park.stalls.length) {
    const sa = [['#', 'type', 'x_ft', 'y_ft'], ...park.stalls.map((s, i) => [i + 1, s.type, +s.cx.toFixed(2), +s.cy.toFixed(2)])];
    XLSX.utils.book_append_sheet(wb, XLSX.utils.aoa_to_sheet(sa), '車位清單');
  }
  if (S.mode === 'site' && S.site && S.site.fin.totalCost > 0) {
    const baseF = readSiteParams().fin, m = { gfa: S.site.gfa, nrsf: S.site.nrsf, units: S.site.units, residential: S.site.residential };
    const d = [-10, 0, 10];
    const grid = [['IRR%  成本＼租金', ...d.map(x => (x > 0 ? '+' : '') + x + '%')]];
    for (const dc of d) grid.push([(dc > 0 ? '+' : '') + dc + '%', ...d.map(dr => {
      const f = { ...baseF, hardCost: baseF.hardCost * (1 + dc / 100), rentMo: baseF.rentMo * (1 + dr / 100), rentSfYr: baseF.rentSfYr * (1 + dr / 100) };
      const fin = PS.computeFinancials(f, m); return fin.irr == null ? '—' : +fin.irr.toFixed(1);
    })]);
    XLSX.utils.book_append_sheet(wb, XLSX.utils.aoa_to_sheet(grid), 'IRR敏感度');
  }
  XLSX.writeFile(wb, (S.mode === 'site' ? 'site-feasibility' : 'parking-takeoff') + '.xlsx');
  toast('已匯出 Excel .xlsx（多工作表）');
}

/* goal-driven optimization — auto-search configs to MEET the parking target,
   only resorting to angled/one-way when needed (a real solver loop). */
function autoOptimize() {
  readParams();
  if (S.boundary.length < 3) { toast('請先畫出基地邊界'); return; }
  $('#busy').classList.add('show'); $('#busyTxt').textContent = '搜尋達標配置中…';
  setTimeout(() => {
    if (S.mode === 'site') {
      let bestA = 90, bestV = -1;
      for (const a of [90, 60, 45]) {
        const sol = PS.solveSite({ boundary: S.boundary, p: { ...readSiteParams(), parkAngle: a }, entrances: S.entrances, obstacles: S.obstacles, roads: S.roads });
        if (sol && sol.parkingProvided > bestV) { bestV = sol.parkingProvided; bestA = a; }
      }
      S.siteParkAngle = bestA; $('#busy').classList.remove('show'); doSolveSite();
      toast(`已最佳化停車：角度 ${bestA}°，提供 ${bestV}`);
      return;
    }
    const target = S.opts.target;
    // keep the region's aisle scale (so Taiwan 6m stays 6m, not US 24ft)
    const baseTwo = AISLE_BY_ANGLE[S.params.angle].two;
    const regionScale = (S.params.aisle || baseTwo) / baseTwo;
    const aisleFor = (a, ow) => AISLE_BY_ANGLE[a][ow ? 'one' : 'two'] * regionScale;
    let best = null;
    for (const o of ['auto', '0', '90', 'edge']) for (const a of [90, 60, 45]) for (const ow of [false, true]) {
      const sol = PS.solve({ boundary: S.boundary, buildings: bPolys(), obstacles: S.obstacles, roads: S.roads, parkZones: S.parkZones, entrances: S.entrances,
        params: { ...S.params, orient: o, angle: a, aisle: aisleFor(a, ow), oneway: ow }, opts: S.opts });
      if (!sol) continue;
      // if a target is set: among configs that MEET it pick the cleanest (two-way/90°);
      // if none meets (or no target): just maximise the count to get closest/best.
      const score = (!target) ? sol.count
        : (sol.count >= target ? (1e7 - (ow ? 30 : 0) - (a !== 90 ? 20 : 0)) : sol.count);
      if (!best || score > best.score) best = { score, count: sol.count, o, a, ow, meets: !target || sol.count >= target };
    }
    $('#busy').classList.remove('show');
    if (!best) { toast('無法最佳化'); return; }
    $('#pOrient').value = best.o; S.params.angle = best.a; S.params.oneway = best.ow;
    document.querySelectorAll('#angleSeg button').forEach(b => b.classList.toggle('active', +b.dataset.ang === best.a));
    document.querySelectorAll('#onewaySeg button').forEach(b => b.classList.toggle('active', (b.dataset.ow === '1') === best.ow));
    $('#pA').value = round2(U.L(aisleFor(best.a, best.ow)));
    doSolve();
    toast(target ? (best.meets ? `✓ 已達標：${best.count} / ${target} 車位` : `已最佳化 ${best.count}，仍差 ${target - best.count}（建議改用斜停或縮退縮）`)
      : `已最佳化：${best.count} 車位（${best.ow ? '單向' : '雙向'} · ${best.a}°）`);
  }, 30);
}
$('#btnOptimize').onclick = autoOptimize;

$('#btnGenerate').onclick = generateOptions;
$('#exPDF').onclick = exportReport;
$('#exTakeoff').onclick = quantityTakeoff;
$('#exXLSX').onclick = exportXLSX;
$('#btnCompare').onclick = compareSchemes;

/* reusable parameter presets (templates) — distinct from full-project Schemes */
const PRESET_KEY = 'ps.presets';
function getPresets() { try { return JSON.parse(localStorage.getItem(PRESET_KEY) || '[]'); } catch (e) { return []; } }
function setPresets(a) { localStorage.setItem(PRESET_KEY, JSON.stringify(a)); renderPresets(); }
function capturePreset() {
  const o = { unit: U.sys, angle: S.params.angle, oneway: S.params.oneway, site: getSiteForm() };
  ['pW', 'pD', 'pA', 'pS', 'pH', 'pOrient', 'adaMode', 'adaManual', 'pEV', 'pCompact', 'pMoto', 'pTarget'].forEach(id => o[id] = $('#' + id).value);
  return o;
}
function applyPreset(p) {
  if (p.unit) setUnitSystem(p.unit);
  ['pW', 'pD', 'pA', 'pS', 'pH', 'pOrient', 'adaMode', 'adaManual', 'pEV', 'pCompact', 'pMoto', 'pTarget'].forEach(id => { if (p[id] != null) $('#' + id).value = p[id]; });
  S.params.angle = p.angle || 90; S.params.oneway = !!p.oneway;
  document.querySelectorAll('#angleSeg button').forEach(b => b.classList.toggle('active', +b.dataset.ang === S.params.angle));
  document.querySelectorAll('#onewaySeg button').forEach(b => b.classList.toggle('active', (b.dataset.ow === '1') === S.params.oneway));
  $('#adaManualRow').style.display = $('#adaMode').value === 'manual' ? '' : 'none';
  setSiteForm(p.site); updateUxSum(); refreshUnitLabels();
  if (S.mode === 'site') { if (S.boundary.length >= 3) doSolveSite(); } else if (S.solution) doSolve();
  toast('已套用參數預設');
}
function savePreset() {
  const name = ($('#presetName').value || '').trim() || `預設 ${getPresets().length + 1}`;
  const a = getPresets(); a.unshift({ name, date: new Date().toISOString().slice(0, 10), data: capturePreset() });
  setPresets(a); $('#presetName').value = ''; toast('已存成參數預設：' + name);
}
function renderPresets() {
  const list = $('#presetList'); if (!list) return; list.innerHTML = '';
  const arr = getPresets();
  if (!arr.length) { list.innerHTML = '<div class="hint">尚無參數預設。</div>'; return; }
  arr.forEach((p, i) => {
    const el = document.createElement('div'); el.className = 'scheme';
    el.innerHTML = `<div style="flex:1;overflow:hidden;"><div class="nm"></div><div class="sub"></div></div><button class="mini" title="套用">↺</button><button class="mini" title="刪除">✕</button>`;
    el.querySelector('.nm').textContent = p.name;
    el.querySelector('.sub').textContent = `${(p.data.site && p.data.site.sUse) || '停車'} · ${p.unit || p.data.unit || ''} · ${p.date}`;
    const [ap, del] = el.querySelectorAll('.mini');
    ap.onclick = () => applyPreset(p.data); del.onclick = () => { const x = getPresets(); x.splice(i, 1); setPresets(x); };
    list.appendChild(el);
  });
}
$('#btnSavePreset').onclick = savePreset;

/* ----------------------- building appearance editor ---------------------- */
// Per-building colour / opacity / height / roof, driven by the selected building
// (click a building on the canvas to select it). Shown only when buildings exist.
function refreshBldgPanel() {
  const g = $('#grpBldg'); if (!g) return;
  const n = S.buildings.length;
  g.style.display = n ? '' : 'none';
  if (!n) { S.selBuilding = null; $('#bSel').innerHTML = ''; return; }
  if (S.selBuilding && S.buildings.indexOf(S.selBuilding) < 0) S.selBuilding = null;
  const b = S.selBuilding || S.buildings[0];
  $('#bSel').innerHTML = S.buildings.map((bb, i) =>
    `<option value="${i}">建築 ${i + 1}${bb.voids && bb.voids.length ? '（中庭×' + bb.voids.length + '）' : ''}</option>`).join('');
  $('#bSel').value = S.buildings.indexOf(b);
  $('#bColor').value = b.color || '#64748b';
  $('#bOpacity').value = Math.round((b.opacity != null ? b.opacity : 0.55) * 100);
  $('#bUse').value = b.use || 'office';
  $('#bFloors').value = b.floors || 1;
  document.querySelectorAll('#bRoofSeg button').forEach(x => x.classList.toggle('active', (x.dataset.r === '1') === (b.roof !== false)));
  updateMassInfo(b);
}
function updateMassInfo(b) {            // live GFA + parking-demand readout for the selected massing
  const el = $('#bMassInfo'); if (!el || !b) return;
  const gfa = bGFA(b), req = bRequired(b);
  const area = U.metric() ? `${Math.round(U.A(gfa)).toLocaleString()} m²` : `${Math.round(gfa).toLocaleString()} ft²`;
  el.innerHTML = `樓地板 GFA：<b style="color:#e2e8f0">${area}</b>　(${USE_LABEL[b.use] || ''} · ${b.floors || 1} 層)<br>需要車位：<b style="color:#facc15">${req.toLocaleString()}</b> 個　(${USE_PARK[b.use] || 3}/1000SF)`;
}
function selectedBldg() { return S.selBuilding || S.buildings[0] || null; }
$('#bSel').onchange = () => { S.selBuilding = S.buildings[+$('#bSel').value] || null; refreshBldgPanel(); draw(); };
$('#bColor').oninput = () => { const b = selectedBldg(); if (b) { b.color = $('#bColor').value; draw(); } };
$('#bOpacity').oninput = () => { const b = selectedBldg(); if (b) { b.opacity = Math.max(0.1, Math.min(1, (+$('#bOpacity').value || 55) / 100)); draw(); } };
$('#bUse').onchange = () => { const b = selectedBldg(); if (b) { b.use = $('#bUse').value; updateMassInfo(b); updateMetrics(); draw(); commit(); } };
$('#bFloors').onchange = () => { const b = selectedBldg(); if (b) { b.floors = Math.max(1, Math.min(120, Math.round(+$('#bFloors').value || 1))); b.height = b.floors * FLOOR_H; updateMassInfo(b); updateMetrics(); if (S.is3d) fit3D(); draw(); commit(); } };
$('#bColor').addEventListener('change', () => { if (selectedBldg()) commit(); });
$('#bOpacity').addEventListener('change', () => { if (selectedBldg()) commit(); });
document.querySelectorAll('#bRoofSeg button').forEach(btn => btn.onclick = () => {
  document.querySelectorAll('#bRoofSeg button').forEach(x => x.classList.remove('active')); btn.classList.add('active');
  const b = selectedBldg(); if (b) { b.roof = btn.dataset.r === '1'; draw(); commit(); }
});
$('#bDelete').onclick = () => {
  const b = selectedBldg(); if (!b) return;
  const i = S.buildings.indexOf(b); if (i >= 0) S.buildings.splice(i, 1);
  S.selBuilding = null;
  if (S.mode === 'site') { S.site ? doSolveSite() : updateSiteMetrics(); }
  else { S.solution ? doSolve() : (updateMetrics(), draw()); }
  refreshBldgPanel(); draw(); commit(); toast('已刪除建築');
};

/* --------------------------- collapsible panel --------------------------- */
const COLLAPSE_KEY = 'ps.collapsed';
function saveCollapsed() {
  const st = {};
  document.querySelectorAll('#panel .group').forEach(g => { st[g.dataset.gkey] = g.classList.contains('collapsed'); });
  try { localStorage.setItem(COLLAPSE_KEY, JSON.stringify(st)); } catch (e) {}
}
function setupCollapsible() {
  const hasState = localStorage.getItem(COLLAPSE_KEY) != null;
  let st = {}; try { st = JSON.parse(localStorage.getItem(COLLAPSE_KEY) || '{}'); } catch (e) {}
  document.querySelectorAll('#panel .group').forEach((g, i) => {
    const h = g.querySelector('h3'); if (!h) return;
    const key = (h.textContent.trim() || ('g' + i)).slice(0, 24);
    g.dataset.gkey = key;
    const c = document.createElement('span'); c.className = 'chev'; c.textContent = '▾'; h.appendChild(c);
    if (st[key]) g.classList.add('collapsed');
    else if (!hasState && key.startsWith('圖例')) g.classList.add('collapsed');   // tidy default
    h.addEventListener('click', () => { g.classList.toggle('collapsed'); saveCollapsed(); });
  });
  if (!hasState) saveCollapsed();
}

/* ----------------------- object tree (site hierarchy) -------------------- */
function gotoGroup(gkey) {
  const g = [...document.querySelectorAll('#panel .group')].find(x => x.dataset.gkey && x.dataset.gkey.startsWith(gkey));
  if (g) { g.classList.remove('collapsed'); saveCollapsed(); g.scrollIntoView({ behavior: 'smooth', block: 'center' }); }
}
function layerCtl(key) {                       // eye + lock buttons for an object-tree row
  const caps = LAYER_CAPS[key]; if (!caps) return '';
  const L = S.layers[key]; let h = '<span class="otctl">';
  if (caps.vis)  h += `<span class="ot-eye${L.vis === false ? ' off' : ''}" data-lk="${key}" data-lp="vis" title="顯示 / 隱藏">${L.vis === false ? '🚫' : '👁'}</span>`;
  if (caps.lock) h += `<span class="ot-lock${L.lock ? ' on' : ''}" data-lk="${key}" data-lp="lock" title="鎖定（不可選取 / 編輯）">${L.lock ? '🔒' : '🔓'}</span>`;
  return h + '</span>';
}
function toggleLayer(key, prop) {
  if (!S.layers[key]) return;
  S.layers[key][prop] = !S.layers[key][prop];
  if (key === 'trees' && prop === 'vis') { $('#btnTrees').classList.toggle('on', S.layers.trees.vis); document.querySelectorAll('#treesSeg button').forEach(b => b.classList.toggle('active', (b.dataset.tr === '1') === S.layers.trees.vis)); }
  if (key === 'flow' && prop === 'vis') {
    if (S.layers.flow.vis && S.is3d) { S.is3d = false; document.querySelectorAll('#viewSeg button').forEach(b => b.classList.toggle('active', b.dataset.view === '2d')); }
    $('#btnFlow').classList.toggle('active', S.layers.flow.vis);
    if (S.layers.flow.vis) { const f = S._flowCache || (S._flowCache = computeFlow()); toast(f.cells.length ? `動線體檢：${f.bottlenecks} 個瓶頸卡點、${f.deadends} 個死巷端 — 越紅越塞` : '請先排一次車位再體檢'); }
  }
  if (S.selEntrance && !pickable('entrance')) S.selEntrance = null;   // drop selection if it just got hidden/locked
  if (S.selStall && !pickable('parking')) S.selStall = null;
  draw(); buildObjTree(); buildLegend();
}
function buildObjTree() {
  const body = $('#objTreeBody'); if (!body) return;
  const park = S.mode === 'site' ? (S.site && S.site.parkSol) : S.solution;
  const stalls = park ? park.stalls.length : 0;
  const rows = [];
  const row = (lv, icon, label, count, action, layer) => rows.push({ lv, icon, label, count, action, layer });
  row(1, '🗺️', '基地 Site', S.boundary.length >= 3 ? '✓' : '—', () => gotoGroup('即時數據'), 'site');
  if (S.parcels && S.parcels.length > 1) S.parcels.forEach((pc, i) =>
    row(2, i === S.activeParcel ? '📍' : '▫️', '子地 ' + String.fromCharCode(65 + i),
      U.big(PS.polyArea(pc)).toFixed(2) + U.bu(), () => setActiveParcel(i)));
  if (S.mode === 'site') {
    row(2, '🏢', '建築 Building', S.site ? (S.site.residential ? S.site.units + ' 戶' : S.site.floors + 'F') : '—', () => { setMode('site'); gotoGroup('開發類型'); }, 'building');
    if (S.site && S.site.unitPlan) row(3, '🏠', '戶型平面 Units', `${S.site.unitPlan.perFloor}/層 · ${S.layers.unitfit.vis ? '顯示' : '隱藏'}`, () => toggleLayer('unitfit', 'vis'), 'unitfit');
    row(2, '🅿️', '停車 Parking', stalls, () => gotoGroup('法規 Zoning'), 'parking');
    row(2, '✅', '法規檢核', S.site ? S.site.compliance.filter(c => c.ok).length + '/' + S.site.compliance.length : '—', () => gotoGroup('法規檢核'));
  } else {
    row(2, '🅿️', '停車場 Surface Parking', stalls, () => { setMode('parking'); gotoGroup('停車參數'); }, 'parking');
    row(3, '🚗', '車位 Stalls', stalls, () => gotoGroup('車位種類配比'), 'parking');
    row(3, '🚪', '出入口 Access', S.entrances.length, () => setTool('entrance'), 'entrance');
  }
  if (S.obstacles.length) row(2, '⛔', '障礙/排除', S.obstacles.length, () => setTool('obstacle'), 'obstacle');
  if (S.buildings.length) row(2, '🏗️', '量體', S.buildings.length, () => setTool('building'), 'building');
  row(2, '🌳', '景觀樹 Trees', S.layers.trees.vis ? '顯示' : '隱藏', () => toggleLayer('trees', 'vis'), 'trees');
  row(2, '🚦', '動線體檢 Flow', S.layers.flow.vis ? '顯示' : '隱藏', () => toggleLayer('flow', 'vis'), 'flow');
  if (S.mode === 'site') row(2, '⛰️', '挖填方 Cut/Fill', S.layers.earthwork.vis ? '顯示' : '隱藏', () => toggleLayer('earthwork', 'vis'), 'earthwork');
  body.innerHTML = rows.map((r, i) => `<div class="otrow lv${r.lv}" data-i="${i}"><span>${r.icon}</span><span>${esc(r.label)}</span><span class="oc">${esc(String(r.count))}</span>${layerCtl(r.layer)}</div>`).join('');
  body.querySelectorAll('.otrow').forEach(el => el.onclick = (e) => { if (e.target.closest('.otctl')) return; rows[+el.dataset.i].action(); });
  body.querySelectorAll('.ot-eye, .ot-lock').forEach(b => b.onclick = (e) => { e.stopPropagation(); toggleLayer(b.dataset.lk, b.dataset.lp); });
}
$('#btnObjTree').onclick = () => {
  const t = $('#objTree'); t.classList.toggle('show');
  $('#btnObjTree').classList.toggle('active', t.classList.contains('show'));
  if (t.classList.contains('show')) buildObjTree();
};

/* ----------------------------- subdivision ------------------------------- */
function splitParcel(p, q) {
  const base = (S.parcels && S.parcels[S.activeParcel]) || S.boundary;
  if (base.length < 3) { toast('請先畫出基地'); return; }
  const nx = -(q.y - p.y), ny = q.x - p.x;                 // normal to the cut line
  const a = PS.clipHP(base, nx, ny, p.x, p.y);
  const b = PS.clipHP(base, -nx, -ny, p.x, p.y);
  const parts = [a, b].filter(poly => poly.length >= 3 && PS.polyArea(poly) > 100);
  if (parts.length < 2) { toast('切割線需橫跨整塊基地'); return; }
  if (!S.parcels) S.parcels = [S.boundary];
  S.parcels.splice(S.activeParcel, 1, ...parts);
  S.boundary = S.parcels[S.activeParcel];
  S.solution = null; S.site = null; S._trees = null;
  S.edgeSetback = {}; S.selEdge = null;
  setTool('select');
  S.mode === 'site' ? doSolveSite() : doSolve();
  if ($('#objTree') && $('#objTree').classList.contains('show')) buildObjTree();
  toast(`已切割成 ${S.parcels.length} 塊子地（在物件樹點選切換）`);
}
function setActiveParcel(i) {
  if (!S.parcels || !S.parcels[i]) return;
  S.activeParcel = i; S.boundary = S.parcels[i];
  S.solution = null; S.site = null; S._trees = null; S.selStall = null; S.edgeSetback = {}; S.selEdge = null;
  S.mode === 'site' ? doSolveSite() : doSolve();
  fitView();
}

/* ============================ CLOUD (Firebase Auth + Firestore) ============================ */
/* The Firebase SDK is lazy-loaded from CDN only when the user first touches a cloud feature, so
   the offline core (localStorage) is never affected. apiKey here is a PUBLIC web key (safe in
   client code); real security is the Firestore rules + Auth, not this key. */
const FB_CFG = {
  apiKey: "AIzaSyCIeDm4-ZVVu0hKVIy5TyG9Xl703C05tak",
  authDomain: "test-23513.firebaseapp.com",
  projectId: "test-23513",
  storageBucket: "test-23513.firebasestorage.app",
  messagingSenderId: "258434107044",
  appId: "1:258434107044:web:a0fcb452642480e87b8831",
};
const FB_VER = '10.12.2';
let _fb = null, _fbUser = null;
function loadScript(src) { return new Promise((res, rej) => { const s = document.createElement('script'); s.src = src; s.onload = res; s.onerror = () => rej(new Error('SDK 載入失敗：' + src)); document.head.appendChild(s); }); }
async function cloudInit() {
  if (_fb) return _fb;
  const base = `https://www.gstatic.com/firebasejs/${FB_VER}/`;
  await loadScript(base + 'firebase-app-compat.js');
  await Promise.all([loadScript(base + 'firebase-auth-compat.js'), loadScript(base + 'firebase-firestore-compat.js')]);
  const fb = window.firebase;
  fb.initializeApp(FB_CFG);
  _fb = { auth: fb.auth(), db: fb.firestore(), fb };
  _fb.auth.onAuthStateChanged(u => { _fbUser = u; updateCloudBtn(); if ($('#modal').classList.contains('show') && $('#cloudBody')) renderCloudModal(); });
  return _fb;
}
function updateCloudBtn() { const b = $('#btnCloud'); if (b) b.classList.toggle('on', !!_fbUser); }
function cloudProject() { const s = serialize(); delete s.solution; delete s.site; return s; }   // inputs only → small doc, re-solve on load
function loadCloudData(d) { deserialize(d); setTimeout(() => { if (S.mode === 'site') { if (S.boundary.length >= 3) doSolveSite(); } else if (S.boundary.length >= 3) doSolve(); }, 40); }
async function cloudShareLink() {
  const { db } = await cloudInit();
  const ref = await db.collection('shared').add({ data: JSON.stringify(cloudProject()), created: Date.now(), by: _fbUser ? _fbUser.uid : null });
  return location.origin + location.pathname + '?p=' + ref.id;
}
async function cloudOpenShared(id) {
  try { const { db } = await cloudInit(); const doc = await db.collection('shared').doc(id).get();
    if (doc.exists) { loadCloudData(JSON.parse(doc.data().data)); toast('已載入分享的方案（唯讀檢視）'); }
    else toast('找不到這個分享連結'); }
  catch (e) { toast('載入分享失敗：' + e.message); }
}
function openCloudModal() {
  openModal('☁️ 雲端 Cloud', '<div id="cloudBody" style="min-height:120px"><div class="hint">連線中…</div></div>');
  cloudInit().then(renderCloudModal).catch(e => { const b = $('#cloudBody'); if (b) b.innerHTML = `<div class="hint" style="color:#fca5a5">雲端連線失敗：${esc(e.message)}<br>（請確認有網路；離線時雲端功能無法使用，但工具其他部分照常）</div>`; });
}
function renderCloudModal() {
  const b = $('#cloudBody'); if (!b) return;
  const inp = 'style="width:100%;background:#0f1a2b;border:1px solid var(--line);color:var(--text);border-radius:6px;padding:7px 9px;font-size:13px;margin:3px 0"';
  if (!_fbUser) {
    b.innerHTML = `
      <div class="hint" style="margin-bottom:8px">登入後可把方案存到雲端、跨裝置開啟、產生分享連結。</div>
      <button class="btn" id="cgoog" style="width:100%;justify-content:center;margin-bottom:10px">使用 Google 登入</button>
      <div class="hint">或用 Email：</div>
      <input id="cem" type="email" placeholder="email" ${inp}><input id="cpw" type="password" placeholder="密碼（至少 6 碼）" ${inp}>
      <div class="row2" style="display:flex;gap:6px;margin-top:6px"><button class="btn" id="cin" style="flex:1;justify-content:center">登入</button><button class="btn" id="creg" style="flex:1;justify-content:center">註冊新帳號</button></div>
      <div class="hint" id="cerr" style="color:#fca5a5;margin-top:6px"></div>
      <div class="hint" style="margin-top:10px;color:var(--muted)">登入後即可：存方案到雲端、跨裝置開啟、產生唯讀分享連結。</div>`;
    $('#cgoog').onclick = () => cloudInit().then(({ auth, fb }) => auth.signInWithPopup(new fb.auth.GoogleAuthProvider())).catch(e => $('#cerr').textContent = e.message);
    const em = () => $('#cem').value.trim(), pw = () => $('#cpw').value;
    $('#cin').onclick = () => cloudInit().then(({ auth }) => auth.signInWithEmailAndPassword(em(), pw())).catch(e => $('#cerr').textContent = e.message);
    $('#creg').onclick = () => cloudInit().then(({ auth }) => auth.createUserWithEmailAndPassword(em(), pw())).catch(e => $('#cerr').textContent = e.message);
  } else {
    b.innerHTML = `
      <div class="hint" style="margin-bottom:8px">已登入：<b>${esc(_fbUser.email || _fbUser.displayName || '使用者')}</b></div>
      <div class="field"><input id="cname" type="text" placeholder="方案名稱" value="${esc(($('#sUse')&&S.mode==='site'?$('#sUse').selectedOptions[0].text:'停車場')+' '+new Date().toLocaleDateString())}" ${inp.replace('width:100%','flex:1')}><button class="btn primary" id="csave" style="justify-content:center">💾 存到雲端</button></div>
      <button class="btn" id="cshare2" style="width:100%;justify-content:center;margin:8px 0">🔗 產生分享連結（唯讀）</button>
      <div class="hint" id="clink" style="word-break:break-all;color:#7dd3fc"></div>
      <hr style="border:none;border-top:1px solid var(--line);margin:10px 0">
      <div class="hint" style="margin-bottom:4px">我的雲端方案：</div>
      <div id="clist"><div class="hint">載入中…</div></div>
      <hr style="border:none;border-top:1px solid var(--line);margin:10px 0">
      <button class="btn ghost" id="cout" style="width:100%;justify-content:center">登出</button>
      <div class="hint" id="cerr" style="color:#fca5a5;margin-top:6px"></div>`;
    $('#csave').onclick = async () => { try { const { db } = await cloudInit(); await db.collection('users').doc(_fbUser.uid).collection('projects').add({ name: $('#cname').value || '未命名', data: JSON.stringify(cloudProject()), updated: Date.now() }); toast('已存到雲端'); listMine(); } catch (e) { $('#cerr').textContent = e.message; } };
    $('#cshare2').onclick = async () => { try { $('#clink').textContent = '產生中…'; const url = await cloudShareLink(); $('#clink').innerHTML = `分享連結：<a href="${esc(url)}" target="_blank" style="color:#7dd3fc">${esc(url)}</a>`; try { await navigator.clipboard.writeText(url); toast('分享連結已複製'); } catch (e) {} } catch (e) { $('#cerr').textContent = e.message; } };
    $('#cout').onclick = () => cloudInit().then(({ auth }) => auth.signOut());
    listMine();
  }
}
async function listMine() {
  const el = $('#clist'); if (!el) return;
  try { const { db } = await cloudInit();
    const snap = await db.collection('users').doc(_fbUser.uid).collection('projects').orderBy('updated', 'desc').get();
    if (snap.empty) { el.innerHTML = '<div class="hint">（還沒有存過方案）</div>'; return; }
    el.innerHTML = snap.docs.map(d => { const x = d.data(); return `<div class="field" style="gap:6px"><span style="flex:1;font-size:13px">${esc(x.name || '未命名')}</span><button class="btn ghost cload" data-id="${d.id}" style="padding:3px 8px">開啟</button><button class="btn ghost cdel" data-id="${d.id}" style="padding:3px 8px">刪</button></div>`; }).join('');
    el.querySelectorAll('.cload').forEach(bn => bn.onclick = async () => { const { db } = await cloudInit(); const doc = await db.collection('users').doc(_fbUser.uid).collection('projects').doc(bn.dataset.id).get(); if (doc.exists) { loadCloudData(JSON.parse(doc.data().data)); closeModal(); toast('已從雲端開啟'); } });
    el.querySelectorAll('.cdel').forEach(bn => bn.onclick = async () => { const { db } = await cloudInit(); await db.collection('users').doc(_fbUser.uid).collection('projects').doc(bn.dataset.id).delete(); listMine(); });
  } catch (e) { el.innerHTML = `<div class="hint" style="color:#fca5a5">${esc(e.message)}</div>`; }
}

/* ------------------------------- boot ------------------------------------ */
function boot() {
  buildLegend();
  renderSchemes();
  renderPresets();
  syncInputs();
  updateUxSum();
  setupCollapsible();
  setTool('select');
  sampleSite();                       // geometry + metrics
  $('#regionSel').value = 'tw';       // default region = Taiwan (metric + TW dims)
  applyRegion('tw');
  resize();                           // may measure 0 if layout not ready yet
  setTimeout(doSolve, 30);            // solving is size-independent

  // Drive the FIRST fit from the moment the stage actually has real
  // dimensions. A ResizeObserver fires on layout regardless of tab
  // visibility (unlike requestAnimationFrame), so this works headless too.
  let fitted = false;
  const ro = new ResizeObserver(() => {
    resize();
    if (!fitted && cv._w > 0 && cv._h > 0) { fitted = true; fitView(); }
  });
  ro.observe($('#stage'));
  setTimeout(() => { $('#help').style.display = 'none'; }, 9000);  // auto-hide help
  const shareId = new URLSearchParams(location.search).get('p');   // ?p=<id> → open a shared project (no login needed)
  if (shareId) setTimeout(() => cloudOpenShared(shareId), 200);
}
boot();

})();
