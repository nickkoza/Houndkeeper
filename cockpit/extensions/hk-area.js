// The Area page ([daF]): the operating area drawn and edited on the house map at phone size, then confirmed and
// stored with the map in its frame; each save is logged with its time. The warning that it is no safety barrier is
// always on the page ([HZO]). Robot side: houndkeeper.operating_area_module.AreaFeed (area_state in, area_cmd out).
// Tap the map to place corners; nothing is sent until Save is confirmed. Steady colours only: nothing here pulses,
// blinks or animates (safe for photosensitive viewers). A Cockpit extension ([Eir]).

export const COLOURS = { area: "#3fb950", fill: "rgba(63, 185, 80, 0.20)", wall: "#ff7b72", draft: "#d29922", dock: "#58a6ff" };

const S = {
  page: { padding: "12px", display: "flex", flexDirection: "column", gap: "12px", fontSize: "15px", overflowY: "auto", height: "100%", boxSizing: "border-box" },
  card: { border: "1px solid var(--border, #2a2f36)", borderRadius: "10px", padding: "12px" },
  warn: { border: "2px solid #d29922", borderRadius: "10px", padding: "12px", fontSize: "15px", lineHeight: 1.4 },
  title: { fontWeight: 600, fontSize: "16px", marginBottom: "6px" },
  muted: { color: "var(--fg-muted, #9aa4b0)", fontSize: "13px" },
  row: { display: "flex", flexWrap: "wrap", gap: "8px" },
  button: { minHeight: "48px", padding: "0 14px", borderRadius: "10px", border: "none", fontSize: "15px", fontWeight: 600, cursor: "pointer", background: "var(--accent, #3b82f6)", color: "#fff" },
  quiet: { background: "transparent", color: "var(--fg, #e6e9ee)", border: "1px solid var(--border, #2a2f36)" },
  on: { background: "#d29922", color: "#111" },
  off: { opacity: 0.45, cursor: "default" },
  canvas: { width: "100%", display: "block", touchAction: "none", borderRadius: "8px", background: "#000" },
};

// ---- plain functions, tested without a canvas ----

// the map fitted to a canvas W pixels wide: pixels per metre and the canvas height
export function fit(map, W) {
  const scale = W / (map.w * map.res);
  return { scale, H: Math.round(map.h * map.res * scale) };
}

// canvas pixel (u, v) <-> the house's frame (x right, y up); the map's (x0, y0) is its bottom-left corner
export function toHouse(map, t, u, v) {
  return [round(map.x0 + u / t.scale), round(map.y0 + (t.H - v) / t.scale)];
}
export function toCanvas(map, t, x, y) {
  return [(x - map.x0) * t.scale, t.H - (y - map.y0) * t.scale];
}
const round = (v) => Math.round(v * 1000) / 1000;

// the draft being edited: the stored area's shapes, the shape being drawn (cur) and what a tap adds (mode)
export function draftOf(area) {
  return { allowed: ((area && area.allowed) || []).map((p) => p.map((q) => [...q])),
    walls: ((area && area.walls) || []).map((s) => s.map((q) => [...q])), cur: [], mode: "view", dirty: false };
}

export function setMode(d, mode) {
  return { ...finish(d), mode };
}

export function addPoint(d, p) {
  if (d.mode === "area") return { ...d, cur: [...d.cur, p], dirty: true };
  if (d.mode === "wall") {
    const cur = [...d.cur, p];
    return cur.length === 2 ? { ...d, walls: [...d.walls, cur], cur: [], dirty: true } : { ...d, cur, dirty: true };
  }
  return d;
}

// the shape being drawn, closed: an area of 3+ corners is kept, fewer are dropped
export function finish(d) {
  if (d.mode === "area" && d.cur.length >= 3) return { ...d, allowed: [...d.allowed, d.cur], cur: [] };
  return { ...d, cur: [] };
}

export function undo(d) {
  return d.cur.length ? { ...d, cur: d.cur.slice(0, -1) } : d;
}

export function removeLast(d) {
  if (d.cur.length) return { ...d, cur: [], dirty: true };
  if (d.mode === "wall" && d.walls.length) return { ...d, walls: d.walls.slice(0, -1), dirty: true };
  if (d.allowed.length) return { ...d, allowed: d.allowed.slice(0, -1), dirty: true };
  if (d.walls.length) return { ...d, walls: d.walls.slice(0, -1), dirty: true };
  return d;
}

// what Save sends: the finished shapes, the change it replaces ("seen"), and that the user confirmed it
export function saveCommand(d, seen) {
  const f = finish(d);
  return { op: "save", confirmed: true, seen: seen || "", area: { frame_id: "house", allowed: f.allowed, walls: f.walls } };
}

const plural = (n, one, many) => `${n} ${n === 1 ? one : many}`;

export function summary(d) {
  const f = finish(d);
  return `${plural(f.allowed.length, "area", "areas")} and ${plural(f.walls.length, "virtual wall", "virtual walls")}`;
}

// what to draw, in canvas pixels: plain data
export function sceneOps(map, t, d, dock, dpr = 1) {
  const at = (p) => toCanvas(map, t, p[0], p[1]), ops = [];
  for (const p of d.allowed) ops.push({ t: "poly", pts: p.map(at), stroke: COLOURS.area, fill: COLOURS.fill, width: 2 * dpr });
  for (const s of d.walls) ops.push({ t: "line", pts: s.map(at), colour: COLOURS.wall, width: 4 * dpr });
  if (d.cur.length) ops.push({ t: "line", pts: d.cur.map(at), colour: COLOURS.draft, width: 2 * dpr, closed: d.mode === "area" && d.cur.length >= 3 });
  for (const p of d.cur) ops.push({ t: "dot", at: at(p), colour: COLOURS.draft, r: 5 * dpr });
  if (dock) ops.push({ t: "dot", at: at(dock), colour: COLOURS.dock, r: 6 * dpr });
  return ops;
}

function paint(ctx, ops) {
  for (const o of ops) {
    ctx.beginPath();
    if (o.t === "dot") { ctx.fillStyle = o.colour; ctx.arc(o.at[0], o.at[1], o.r, 0, 2 * Math.PI); ctx.fill(); continue; }
    o.pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    if (o.t === "poly" || o.closed) ctx.closePath();
    if (o.fill) { ctx.fillStyle = o.fill; ctx.fill(); }
    ctx.strokeStyle = o.stroke || o.colour; ctx.lineWidth = o.width; ctx.stroke();
  }
}

export function register({ React, registerPanel, PanelFrame, useStoreChannel }) {
  const h = React.createElement;
  const { useState, useEffect, useRef } = React;

  function MapCanvas({ st, draft, onTap }) {
    const ref = useRef(null), imgRef = useRef(null);
    useEffect(() => {
      const canvas = ref.current;
      if (!canvas || !st.map || typeof document === "undefined") return undefined;
      const draw = () => {
        const dpr = globalThis.devicePixelRatio || 1, W = canvas.clientWidth * dpr, t = fit(st.map, W);
        if (canvas.width !== Math.round(W) || canvas.height !== t.H) { canvas.width = Math.round(W); canvas.height = t.H; }
        const ctx = canvas.getContext("2d");
        ctx.imageSmoothingEnabled = false; ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (imgRef.current && imgRef.current.complete) ctx.drawImage(imgRef.current, 0, 0, canvas.width, canvas.height);
        paint(ctx, sceneOps(st.map, t, draft, st.dock, dpr));
      };
      if (!imgRef.current || imgRef.current.dataset.png !== st.map.png.slice(0, 64)) {
        const img = new Image(); img.dataset.png = st.map.png.slice(0, 64); img.onload = draw;
        img.src = `data:image/png;base64,${st.map.png}`; imgRef.current = img;
      }
      draw();
      const ro = new ResizeObserver(draw); ro.observe(canvas);
      return () => ro.disconnect();
    }, [st.map, draft, st.dock]);
    const tap = (e) => {
      const canvas = ref.current, r = canvas.getBoundingClientRect(), t = fit(st.map, r.width);
      onTap(toHouse(st.map, t, e.clientX - r.left, e.clientY - r.top));
    };
    return h("canvas", { ref, style: S.canvas, onPointerUp: tap, role: "img", "data-hk": "area-map",
      "aria-label": "the house map with the operating area: tap to place corners" });
  }

  function Area({ spec, store, session }) {
    const snap = useStoreChannel(store, spec.channels[0]);
    const st = snap.slot === null ? null : snap.slot.value;
    const stored = st && st.area ? st.area : null;
    const [draft, setDraft] = useState(() => (st ? draftOf(stored) : null));
    const [seen, setSeen] = useState(() => (stored ? stored.changed : "")), [sending, setSending] = useState(false);
    useEffect(() => {                                  // the stored area becomes the draft, unless one is being edited
      if (st && (draft === null || (!draft.dirty && (stored ? stored.changed : "") !== seen))) {
        setDraft(draftOf(stored)); setSeen(stored ? stored.changed : "");
      }
    }, [st && stored ? stored.changed : "", st === null]);
    const warning = h("div", { key: "warn", style: S.warn, role: "note", "data-hk": "area-warning" },
      (st && st.warning) || "The operating area is not a safety barrier: it holds only while he knows where he is. Keep stairs, drops and anything dangerous behind a physical barrier, such as a gate.");
    let body;
    if (st === null || draft === null) body = [warning, h("div", { key: "wait", style: S.muted }, "waiting for the robot...")];
    else {
      const d = draft, mode = (m, label) => h("button", { key: m, style: { ...S.button, ...(d.mode === m ? S.on : S.quiet) },
        "aria-pressed": d.mode === m, onClick: () => setDraft(setMode(d, d.mode === m ? "view" : m)) }, label);
      const save = async () => {
        if (!session) return;
        const text = `Save the operating area: ${summary(d)}?\n\nIt is stored with the house map and logged with the time.\n\n${st.warning}`;
        if (!globalThis.confirm(text)) return;
        setSending(true);
        try { await session.publish(spec.channels[1], saveCommand(d, seen)); setDraft({ ...finish(d), dirty: false, mode: "view" }); }
        catch (e) { globalThis.alert(`not sent: ${e && e.message ? e.message : e}`); }
        setSending(false);
      };
      body = [
        warning,
        h("div", { key: "map", style: S.card },
          h("div", { style: S.title }, "The house map"),
          st.map ? h(MapCanvas, { st, draft: d, onTap: (p) => setDraft(addPoint(d, p)) })
            : h("div", { style: S.muted }, "No house map yet: he maps the house on his first nights."),
          h("div", { style: { ...S.muted, marginTop: "6px" } },
            d.mode === "area" ? "Tap each corner of the area, then Finish." : d.mode === "wall" ? "Tap the two ends of the wall." : "Choose what to draw.")),
        h("div", { key: "tools", style: S.row },
          mode("area", "Draw an area"), mode("wall", "Draw a wall"),
          h("button", { key: "fin", style: { ...S.button, ...S.quiet }, disabled: d.mode !== "area" || d.cur.length < 3, onClick: () => setDraft(finish(d)) }, "Finish"),
          h("button", { key: "undo", style: { ...S.button, ...S.quiet }, disabled: !d.cur.length, onClick: () => setDraft(undo(d)) }, "Undo corner"),
          h("button", { key: "rm", style: { ...S.button, ...S.quiet }, onClick: () => setDraft(removeLast(d)) }, "Remove last shape"),
          h("button", { key: "back", style: { ...S.button, ...S.quiet }, disabled: !d.dirty, onClick: () => setDraft(draftOf(stored)) }, "Discard changes")),
        h("div", { key: "now", style: S.card },
          h("div", { style: S.title }, d.dirty ? `Not saved: ${summary(d)}` : stored ? `Stored: ${summary(d)}` : "No area stored"),
          stored && h("div", { style: S.muted }, `changed ${stored.changed} by ${stored.by}`),
          h("button", { style: { ...S.button, marginTop: "10px", width: "100%", ...(sending || !d.dirty || finish(d).allowed.length === 0 ? S.off : {}) },
            disabled: sending || !d.dirty || finish(d).allowed.length === 0, onClick: save }, "Save the area")),
        st.message && h("div", { key: "msg", style: S.card, role: "status" }, st.message),
        (st.history || []).length > 0 && h("div", { key: "hist", style: S.card },
          h("div", { style: S.title }, "Last changes"),
          h("ul", { style: { margin: 0, paddingLeft: "18px" } }, st.history.map((c, i) => h("li", { key: i, style: S.muted },
            `${c.changed} by ${c.by}: ${plural(c.polygons, "area", "areas")}, ${plural(c.walls, "wall", "walls")}`)))),
      ];
    }
    return h(PanelFrame, { spec, children: h("div", { style: S.page, "data-hk": "area" }, body) });
  }

  registerPanel("hk-area", Area);
}
