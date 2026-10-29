// The Mind page ([RNC]): what Barrymore thinks, live or replayed ([B4u]). The floor map with the route walked and
// planned, the candidates coloured by status and confidence, and the goal with its reason in words ([Ttd]); over the
// camera, the candidates it sees outlined, with the stage and reason as a caption ([D1U]). Robot side:
// houndkeeper.mind (the summary and its schema), through cockpit_data.MindFeed. Every word shown is the robot's own
// ([1XG]). Read only ([ls3]): the panel takes no session and sends nothing. Steady colours only: nothing here pulses,
// blinks or animates (safe for photosensitive viewers). A Cockpit extension ([Eir]).

export const COLOURS = { "new": "#d29922", "looked at": "#58a6ff", "ruled out": "#8b949e", "picked": "#3fb950" };
export const ROUTE = { walked: "#9aa4b0", plan: "#a371f7", path: "#e6edf3", goal: "#ffffff", robot: "#ff5c5c" };
const ORDER = ["new", "looked at", "picked", "ruled out"];

// costmap.zlib.v1 cells (0 free, 1-99 cost, 100+ lethal, 255 unknown) as RGBA, rows flipped (row 0 is the world's min y):
// the built-in map's palette, dimmed so the marks over it read
export function gridImage(cells, w, h) {
  const out = new Uint8ClampedArray(w * h * 4);
  for (let row = 0; row < h; row++) {
    const src = (h - 1 - row) * w, dst = row * w;
    for (let col = 0; col < w; col++) {
      const v = cells[src + col], o = (dst + col) * 4;
      const [r, g, b, a] = v === 0 ? [0x1e, 0x2a, 0x30, 255] : v < 100 ? [0x3d, 0x5a, 0x63, 255] : v < 255 ? [0xb8, 0xc0, 0xc8, 255] : [0, 0, 0, 0];
      out[o] = r; out[o + 1] = g; out[o + 2] = b; out[o + 3] = a;
    }
  }
  return out;
}

// with no costmap yet (or a replay without one): a placement round everything the message names, a metre spare
export function boundsPlace(m) {
  const pts = [];
  if (m.pose) pts.push(m.pose);
  if (m.goal && m.goal.x !== null) pts.push([m.goal.x, m.goal.y]);
  for (const c of m.cands || []) if (c.x !== null) pts.push([c.x, c.y]);
  for (const p of (m.route && m.route.walked) || []) pts.push(p);
  for (const p of (m.route && m.route.plan) || []) pts.push(p);
  if (!pts.length) return null;
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const x0 = Math.min(...xs) - 1, y0 = Math.min(...ys) - 1;
  return { w: Math.max(...xs) + 1 - x0, h: Math.max(...ys) + 1 - y0, res: 1, origin: [x0, y0, 0] };
}

// the whole grid fitted and centred, as the built-in map does (mapRenderer.fitTransform), y up
export function fitView(place, W, H) {
  const ww = place.w * place.res, wh = place.h * place.res, c = Math.cos(place.origin[2]), s = Math.sin(place.origin[2]);
  const bw = Math.abs(ww * c) + Math.abs(wh * s), bh = Math.abs(ww * s) + Math.abs(wh * c);
  const scale = Math.min(W / bw, H / bh);
  return { scale, originX: place.origin[0] + Math.min(0, ww * c) + Math.min(0, -wh * s),
    originY: place.origin[1] + Math.min(0, ww * s) + Math.min(0, wh * c), cx0: (W - bw * scale) / 2, cy0: H - (H - bh * scale) / 2 };
}

export function toCanvas(t, x, y) { return [t.cx0 + (x - t.originX) * t.scale, t.cy0 - (y - t.originY) * t.scale]; }

const alphaOf = (conf) => (conf === null || conf === undefined ? 0.5 : 0.25 + 0.75 * Math.max(0, Math.min(1, conf)));

// what to draw on the map, in canvas pixels: plain data, so it is tested without a canvas
export function mapScene(m, place, W, H, path = null, dpr = 1) {
  const t = fitView(place, W, H), at = (p) => toCanvas(t, p[0], p[1]), ops = [];
  const walked = (m.route && m.route.walked) || [], plan = (m.route && m.route.plan) || [];
  if (walked.length > 1) ops.push({ t: "line", pts: walked.map(at), colour: ROUTE.walked, width: 2 * dpr });
  if (plan.length) ops.push({ t: "line", pts: (m.pose ? [m.pose] : []).concat(plan).map(at), colour: ROUTE.plan, width: 2 * dpr, dash: [6 * dpr, 5 * dpr] });
  if (Array.isArray(path) && path.length > 1) ops.push({ t: "line", pts: path.map(at), colour: ROUTE.path, width: 1.5 * dpr });
  const cands = (m.cands || []).filter((c) => c.x !== null).slice().sort((a, b) => ORDER.indexOf(b.status) - ORDER.indexOf(a.status));
  for (const c of cands) {
    const [x, y] = at([c.x, c.y]), r = Math.max((c.r || 0.08) * t.scale, 5 * dpr);
    const out = c.status === "ruled out";
    ops.push({ t: "disc", id: c.id, x, y, r, colour: COLOURS[c.status] || COLOURS.new, alpha: out ? 0 : alphaOf(c.conf), stroke: 1.5 * dpr });
  }
  if (m.goal && m.goal.x !== null) {
    const g = at([m.goal.x, m.goal.y]);
    if (m.pose) ops.push({ t: "line", pts: [at(m.pose), g], colour: ROUTE.goal, width: 1.5 * dpr, dash: [3 * dpr, 4 * dpr] });
    ops.push({ t: "disc", id: "goal", x: g[0], y: g[1], r: 11 * dpr, colour: ROUTE.goal, alpha: 0, stroke: 2.5 * dpr });
  }
  if (m.pose) {
    const [x, y] = at(m.pose), a = -m.pose[2], s = 12 * dpr, c = Math.cos(a), sn = Math.sin(a);
    const tri = [[0.6 * s, 0], [-0.4 * s, 0.35 * s], [-0.4 * s, -0.35 * s]].map(([px, py]) => [x + px * c - py * sn, y + px * sn + py * c]);
    ops.push({ t: "poly", id: "robot", pts: tri, colour: ROUTE.robot });
  }
  return ops;
}

// the outlines over the camera, in the frame's pixels (the message gives fractions of its width and height)
export function videoScene(m, W, H, dpr = 1) {
  const byId = Object.fromEntries((m.cands || []).map((c) => [c.id, c])), ops = [];
  for (const v of m.view || []) {
    const c = byId[v.id] || { status: "new", conf: null }, pts = v.ring.map(([u, w]) => [u * W, w * H]);
    ops.push({ t: "ring", id: v.id, pts, colour: COLOURS[c.status] || COLOURS.new, width: 3 * dpr });
    const top = pts.reduce((a, b) => (b[1] < a[1] ? b : a));
    const words = [c.conf === null || c.conf === undefined ? null : `${Math.round(c.conf * 100)}%`, c.label].filter(Boolean).join(" ");
    if (words) ops.push({ t: "text", x: top[0], y: top[1] - 6 * dpr, text: words, colour: COLOURS[c.status] || COLOURS.new, size: 16 * dpr });
  }
  return ops;
}

export function goalWords(m) {
  if (!m || !m.goal) return "No goal";
  const kind = { waypoint: "Walking the sweep", find: "Going to a find", sidestep: "Stepping aside" }[m.goal.kind] || "Going";
  return m.goal.why ? `${kind}: ${m.goal.why}` : kind;
}

export function caption(m) {
  if (!m) return "waiting for the robot...";
  const stage = m.stage ? m.stage[0].toUpperCase() + m.stage.slice(1) : "";
  return `${stage} - ${goalWords(m)}`;
}

export function paint(ctx, ops) {
  for (const o of ops) {
    ctx.save();
    ctx.setLineDash(o.dash || []);
    if (o.t === "line" || o.t === "ring") {
      ctx.beginPath(); o.pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
      if (o.t === "ring") ctx.closePath();
      ctx.strokeStyle = o.colour; ctx.lineWidth = o.width; ctx.stroke();
    } else if (o.t === "disc") {
      ctx.beginPath(); ctx.arc(o.x, o.y, o.r, 0, 2 * Math.PI);
      if (o.alpha > 0) { ctx.globalAlpha = o.alpha; ctx.fillStyle = o.colour; ctx.fill(); ctx.globalAlpha = 1; }
      ctx.strokeStyle = o.colour; ctx.lineWidth = o.stroke; ctx.stroke();
    } else if (o.t === "poly") {
      ctx.beginPath(); o.pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.closePath();
      ctx.fillStyle = o.colour; ctx.fill();
    } else if (o.t === "text") {
      ctx.font = `600 ${o.size}px system-ui, sans-serif`; ctx.textAlign = "center"; ctx.lineWidth = 4;
      ctx.strokeStyle = "rgba(0,0,0,0.75)"; ctx.strokeText(o.text, o.x, o.y); ctx.fillStyle = o.colour; ctx.fillText(o.text, o.x, o.y);
    }
    ctx.restore();
  }
}

// costmap.zlib.v1: zlib framing, so DecompressionStream("deflate"), exactly w*h cells (the SDK's inflateCostmap)
async function inflate(value) {
  const n = value.w * value.h, cells = new Uint8Array(n);
  const reader = new Blob([value.bytes]).stream().pipeThrough(new DecompressionStream("deflate")).getReader();
  let at = 0;
  for (;;) {
    const { done, value: chunk } = await reader.read();
    if (done) break;
    if (at + chunk.length > n) throw new Error("costmap inflates past its size");
    cells.set(chunk, at); at += chunk.length;
  }
  if (at !== n) throw new Error("costmap short");
  return cells;
}

const isGrid = (v) => v && v.bytes instanceof Uint8Array && typeof v.w === "number";
const valueOf = (store, ch) => { const s = ch ? store.get(ch) : null; return s ? s.value : null; };

// redraws on a new message, grid or plan; one inflate at a time, the newest grid; nothing while the page is hidden
function startMap(store, ch, canvas) {
  if (!canvas || typeof document === "undefined") return undefined;
  const ctx = canvas.getContext("2d"), grid = document.createElement("canvas");
  let place = null, inflating = false, drawn = -1, stopped = false;
  const draw = () => {
    if (stopped || document.hidden || !ctx) return;
    const dpr = globalThis.devicePixelRatio || 1, W = Math.round(canvas.clientWidth * dpr), H = Math.round(canvas.clientHeight * dpr);
    if (!W || !H) return;
    if (canvas.width !== W || canvas.height !== H) { canvas.width = W; canvas.height = H; }
    ctx.clearRect(0, 0, W, H);
    const m = valueOf(store, ch.mind) || {};
    const p = place || boundsPlace(m);
    if (!p) return;
    if (place) {
      const t = fitView(place, W, H), [ax, ay] = toCanvas(t, place.origin[0], place.origin[1]);
      ctx.save(); ctx.imageSmoothingEnabled = false; ctx.translate(ax, ay); ctx.rotate(-place.origin[2]);
      ctx.drawImage(grid, 0, -place.h * place.res * t.scale, place.w * place.res * t.scale, place.h * place.res * t.scale); ctx.restore();
    }
    paint(ctx, mapScene(m, p, W, H, valueOf(store, ch.path), dpr));
  };
  const pump = () => {
    const slot = ch.costmap ? store.get(ch.costmap) : null;
    if (stopped || inflating || document.hidden || !slot || slot.version === drawn || !isGrid(slot.value)) return;
    const v = slot.value; inflating = true;
    inflate(v).then((cells) => {
      if (stopped) return;
      grid.width = v.w; grid.height = v.h;
      grid.getContext("2d").putImageData(new ImageData(gridImage(cells, v.w, v.h), v.w, v.h), 0, 0);
      place = { w: v.w, h: v.h, res: v.res, origin: v.origin }; draw();
    }).catch(() => {}).finally(() => { drawn = slot.version; inflating = false; pump(); });
  };
  const offs = [store.subscribe(ch.mind, draw)];
  if (ch.costmap) offs.push(store.subscribe(ch.costmap, pump));
  if (ch.path) offs.push(store.subscribe(ch.path, draw));
  const ro = new ResizeObserver(draw); ro.observe(canvas);
  const onVis = () => { pump(); draw(); };
  document.addEventListener("visibilitychange", onVis);
  pump(); draw();
  return () => { stopped = true; offs.forEach((f) => f()); ro.disconnect(); document.removeEventListener("visibilitychange", onVis); };
}

// the newest frame, one decode at a time, with the outlines from the newest message over it
function startVideo(store, ch, canvas) {
  if (!canvas || typeof document === "undefined") return undefined;
  const ctx = canvas.getContext("2d");
  let frame = null, decoding = false, drawn = -1, stopped = false;
  const draw = () => {
    if (stopped || document.hidden || !ctx || !frame) return;
    if (canvas.width !== frame.width || canvas.height !== frame.height) { canvas.width = frame.width; canvas.height = frame.height; }
    ctx.drawImage(frame, 0, 0);
    const m = valueOf(store, ch.mind);
    if (m) paint(ctx, videoScene(m, frame.width, frame.height, frame.width / 640));
  };
  const pump = () => {
    const slot = store.get(ch.video);
    if (stopped || decoding || document.hidden || !slot || slot.version === drawn || !(slot.value instanceof Uint8Array)) return;
    decoding = true;
    createImageBitmap(new Blob([slot.value], { type: "image/jpeg" })).then((bmp) => {
      if (stopped) { bmp.close(); return; }
      if (frame) frame.close();
      frame = bmp; draw();
    }).catch(() => {}).finally(() => { drawn = slot.version; decoding = false; pump(); });
  };
  const offs = [store.subscribe(ch.video, pump), store.subscribe(ch.mind, draw)];
  const onVis = () => { pump(); draw(); };
  document.addEventListener("visibilitychange", onVis);
  pump();
  return () => { stopped = true; offs.forEach((f) => f()); if (frame) frame.close(); document.removeEventListener("visibilitychange", onVis); };
}

const S = {
  page: { padding: "10px", display: "flex", flexDirection: "column", gap: "10px", fontSize: "15px", overflowY: "auto", height: "100%", boxSizing: "border-box" },
  card: { border: "1px solid var(--border, #2a2f36)", borderRadius: "10px", padding: "10px" },
  stage: { fontWeight: 700, fontSize: "20px" },
  muted: { color: "var(--fg-muted, #9aa4b0)", fontSize: "13px" },
  video: { position: "relative", borderRadius: "10px", overflow: "hidden", background: "#000" },
  cap: { position: "absolute", left: 0, right: 0, bottom: 0, padding: "6px 10px", background: "rgba(0,0,0,0.6)", color: "#fff", fontSize: "14px" },
  map: { width: "100%", height: "55vh", display: "block", borderRadius: "10px", background: "var(--bg-sunken, #0d1117)" },
  row: { display: "flex", gap: "8px", alignItems: "baseline", padding: "4px 0", borderBottom: "1px solid var(--border, #2a2f36)" },
  sw: (c, filled) => ({ flex: "0 0 12px", height: "12px", borderRadius: "50%", border: `2px solid ${c}`, background: filled ? c : "transparent", alignSelf: "center" }),
};

export function register({ React, registerPanel, PanelFrame, useStoreChannel }) {
  const h = React.createElement;
  const { useEffect, useRef } = React;

  function Candidate({ c }) {
    const colour = COLOURS[c.status] || COLOURS.new;
    return h("div", { style: S.row },
      h("span", { style: S.sw(colour, c.status !== "ruled out") }),
      h("span", { style: { flex: "0 0 auto", fontWeight: 600, color: colour } }, c.status),
      h("span", { style: { flex: "0 0 auto", fontVariantNumeric: "tabular-nums" } }, c.conf === null || c.conf === undefined ? "" : `${Math.round(c.conf * 100)}%`),
      h("span", { style: { flex: "1 1 auto", minWidth: 0 } }, [c.label, c.why].filter(Boolean).join(": ")));
  }

  // props: spec and store only. The cockpit also passes a session; this page never takes it ([ls3])
  function Mind({ spec, store }) {
    const p = spec.params || {}, mindCh = spec.channels[0];
    const snap = useStoreChannel(store, mindCh), m = snap.slot === null ? null : snap.slot.value;
    const mapRef = useRef(null), vidRef = useRef(null);
    useEffect(() => startMap(store, { mind: mindCh, costmap: p.costmap, path: p.path }, mapRef.current), [store, mindCh, p.costmap, p.path]);
    useEffect(() => (p.video ? startVideo(store, { mind: mindCh, video: p.video }, vidRef.current) : undefined), [store, mindCh, p.video]);
    const cands = m ? m.cands || [] : [];
    const counts = ORDER.map((s) => [s, cands.filter((c) => c.status === s).length]);
    const body = [
      h("div", { key: "head", style: S.card },
        h("div", { style: S.stage }, m ? (m.stage || "-") : "waiting for the robot..."),
        m && h("div", { style: S.muted }, `${m.step || ""}${m.t ? ` · ${new Date(m.t * 1000).toLocaleTimeString()}` : ""}`),
        m && h("div", { style: { marginTop: "6px", fontSize: "16px" } }, goalWords(m)),
        m && m.arm && h("div", { style: { marginTop: "4px" } }, `Arm: ${m.arm.state}${m.arm.item ? ` (${m.arm.item})` : ""}`),
        m && m.note && h("div", { style: { ...S.muted, marginTop: "4px" } }, m.note)),
      p.video && h("div", { key: "video", style: S.video },
        h("canvas", { ref: vidRef, style: { width: "100%", height: "auto", display: "block" }, role: "img", "aria-label": "camera with the candidates outlined" }),
        h("div", { style: S.cap }, caption(m))),
      h("canvas", { key: "map", ref: mapRef, style: S.map, role: "img", "aria-label": "floor map with the routes, candidates and goal" }),
      h("div", { key: "legend", style: { ...S.muted, display: "flex", flexWrap: "wrap", gap: "12px" } },
        counts.map(([s, n]) => h("span", { key: s }, h("span", { style: { ...S.sw(COLOURS[s], s !== "ruled out"), display: "inline-block", marginRight: "4px" } }), `${s} ${n}`)),
        h("span", { key: "w" }, h("span", { style: { color: ROUTE.walked } }, "--"), " walked"),
        h("span", { key: "p" }, h("span", { style: { color: ROUTE.plan } }, "- -"), " planned")),
      h("div", { key: "list", style: S.card },
        cands.length ? cands.map((c) => h(Candidate, { key: c.id, c })) : h("div", { style: S.muted }, "no candidates yet"),
        m && m.more > 0 && h("div", { style: S.muted }, `and ${m.more} more`)),
    ];
    return h(PanelFrame, { spec, children: h("div", { style: S.page }, body) });
  }

  registerPanel("hk-mind", Mind);
}
