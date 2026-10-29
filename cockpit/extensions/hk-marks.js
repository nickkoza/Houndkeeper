// The Marks page ([vO9]; decisions [hOy], [oWO]): before a real night the user lays out the toys and taps each one's
// place on the house's floor map (the Area page's map and its taps, in the house's frame), its name from a short list
// and its floor kind; a confirmed save sends the marks, which the project service writes beside the night and logs.
// Closed once the night has run. Project side: houndkeeper.night_scoring (marks_state in, marks_cmd out). Steady
// colours only: nothing here pulses, blinks or animates (safe for photosensitive viewers). Sized for a phone held
// upright (390 px). A Cockpit extension ([Eir]).
import { fit, toHouse, toCanvas } from "./hk-area.js";

export const FLOOR_COLOURS = { hard: "#58a6ff", carpet: "#d29922", rug: "#3fb950" };

const S = {
  page: { padding: "12px", display: "flex", flexDirection: "column", gap: "12px", fontSize: "15px", overflowY: "auto", height: "100%", boxSizing: "border-box" },
  card: { border: "1px solid var(--border, #2a2f36)", borderRadius: "10px", padding: "12px" },
  title: { fontWeight: 600, fontSize: "16px", marginBottom: "6px" },
  muted: { color: "var(--fg-muted, #9aa4b0)", fontSize: "13px" },
  row: { display: "flex", flexWrap: "wrap", gap: "8px" },
  chip: { minHeight: "44px", padding: "0 12px", borderRadius: "10px", fontSize: "15px", cursor: "pointer",
    background: "transparent", color: "var(--fg, #e6e9ee)", border: "1px solid var(--border, #2a2f36)" },
  chipOn: { background: "#d29922", color: "#111", border: "1px solid #d29922", fontWeight: 600 },
  button: { minHeight: "48px", padding: "0 14px", borderRadius: "10px", border: "none", fontSize: "15px", fontWeight: 600, cursor: "pointer", background: "var(--accent, #3b82f6)", color: "#fff" },
  quiet: { background: "transparent", color: "var(--fg, #e6e9ee)", border: "1px solid var(--border, #2a2f36)" },
  off: { opacity: 0.45, cursor: "default" },
  canvas: { width: "100%", display: "block", touchAction: "none", borderRadius: "8px", background: "#000" },
  item: { display: "flex", alignItems: "center", justifyContent: "space-between", gap: "8px", padding: "6px 0", borderTop: "1px solid var(--border, #2a2f36)" },
};

// ---- plain functions, tested without a canvas ----

// "20261015" as "Thu 15 Oct"
export function nightLabel(name) {
  const m = /^(\d{4})(\d{2})(\d{2})$/.exec(String(name || ""));
  if (!m) return String(name || "");
  const d = new Date(Date.UTC(+m[1], +m[2] - 1, +m[3]));
  return d.toLocaleDateString("en-GB", { weekday: "short", day: "numeric", month: "short", timeZone: "UTC" });
}

// the draft being edited: the saved marks' toys, the toy and floor a tap places
export function draftOf(marks, name = null, floor = "hard") {
  return { toys: ((marks && marks.toys) || []).map((t) => ({ name: t.name, x: t.x, y: t.y, floor: t.floor })), name, floor, dirty: false };
}

export function addToy(d, p) {
  if (!d.name) return d;
  return { ...d, toys: [...d.toys, { name: d.name, x: p[0], y: p[1], floor: d.floor }], dirty: true };
}

export function removeToy(d, i) {
  return { ...d, toys: d.toys.filter((_, j) => j !== i), dirty: true };
}

// what is laid out: toys, 1 x 1 bricks (reported apart), bricks of any size, and how many on each floor kind
export function counts(toys, list) {
  const brick1 = new Set((list || []).filter((t) => t.brick).map((t) => t.name));
  const floors = { hard: 0, carpet: 0, rug: 0 };
  for (const t of toys) floors[t.floor] = (floors[t.floor] || 0) + 1;
  return { n: toys.length, brick1: toys.filter((t) => brick1.has(t.name)).length,
    bricks: toys.filter((t) => /brick/.test(t.name)).length, floors };
}

const plural = (n, one, many) => `${n} ${n === 1 ? one : many}`;

export function summary(toys, list) {
  const c = counts(toys, list);
  return `${plural(c.n, "toy", "toys")}, ${plural(c.bricks, "brick", "bricks")} (${c.brick1} of them 1 x 1); ` +
    `hard floor ${c.floors.hard}, carpet ${c.floors.carpet}, rug ${c.floors.rug}`;
}

// what Save sends: tonight's name, the toys, the save it replaces ("seen"), and that the user confirmed it
export function saveCommand(st, d) {
  return { op: "save", confirmed: true, night: st.night, seen: (st.marks && st.marks.saved) || "",
    toys: d.toys.map((t) => ({ name: t.name, x: t.x, y: t.y, floor: t.floor })) };
}

// what to draw, in canvas pixels: a numbered dot per toy, coloured by its floor kind
export function sceneOps(map, t, toys, dpr = 1) {
  return toys.map((toy, i) => ({ t: "mark", at: toCanvas(map, t, toy.x, toy.y), colour: FLOOR_COLOURS[toy.floor] || "#e6e9ee",
    r: 9 * dpr, label: String(i + 1), font: 11 * dpr }));
}

function paint(ctx, ops) {
  for (const o of ops) {
    ctx.beginPath(); ctx.fillStyle = o.colour; ctx.arc(o.at[0], o.at[1], o.r, 0, 2 * Math.PI); ctx.fill();
    ctx.lineWidth = o.r / 5; ctx.strokeStyle = "#111"; ctx.stroke();
    ctx.fillStyle = "#111"; ctx.font = `bold ${o.font}px sans-serif`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(o.label, o.at[0], o.at[1] + 0.5);
  }
}

export function register({ React, registerPanel, PanelFrame, useStoreChannel }) {
  const h = React.createElement;
  const { useState, useEffect, useRef } = React;

  function MapCanvas({ map, toys, onTap }) {
    const ref = useRef(null), imgRef = useRef(null);
    useEffect(() => {
      const canvas = ref.current;
      if (!canvas || !map || typeof document === "undefined") return undefined;
      const draw = () => {
        const dpr = globalThis.devicePixelRatio || 1, W = canvas.clientWidth * dpr, t = fit(map, W);
        if (canvas.width !== Math.round(W) || canvas.height !== t.H) { canvas.width = Math.round(W); canvas.height = t.H; }
        const ctx = canvas.getContext("2d");
        ctx.imageSmoothingEnabled = false; ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (imgRef.current && imgRef.current.complete) ctx.drawImage(imgRef.current, 0, 0, canvas.width, canvas.height);
        paint(ctx, sceneOps(map, t, toys, dpr));
      };
      if (!imgRef.current || imgRef.current.dataset.png !== map.png.slice(0, 64)) {
        const img = new Image(); img.dataset.png = map.png.slice(0, 64); img.onload = draw;
        img.src = `data:image/png;base64,${map.png}`; imgRef.current = img;
      }
      draw();
      const ro = new ResizeObserver(draw); ro.observe(canvas);
      return () => ro.disconnect();
    }, [map, toys]);
    const tap = (e) => {
      const canvas = ref.current, r = canvas.getBoundingClientRect(), t = fit(map, r.width);
      onTap(toHouse(map, t, e.clientX - r.left, e.clientY - r.top));
    };
    return h("canvas", { ref, style: S.canvas, onPointerUp: onTap ? tap : undefined, role: "img", "data-hk": "marks-map",
      "aria-label": "the house's floor map with tonight's toys: tap a toy's place to mark it" });
  }

  function Marks({ spec, store, session }) {
    const snap = useStoreChannel(store, spec.channels[0]);
    const st = snap.slot === null ? null : snap.slot.value;
    const [draft, setDraft] = useState(() => (st ? draftOf(st.marks) : null));
    const [sending, setSending] = useState(false);
    const savedKey = st ? `${st.night}|${(st.marks && st.marks.saved) || ""}` : "";
    useEffect(() => {                                  // the saved marks become the draft, unless one is being edited
      if (st && (draft === null || !draft.dirty)) setDraft(draftOf(st.marks, draft ? draft.name : null, draft ? draft.floor : "hard"));
    }, [savedKey, st === null]);
    let body;
    if (st === null || draft === null) body = h("div", { style: S.muted }, "waiting for the project service...");
    else {
      const d = draft, open = !st.ran;
      const chip = (on, label, onClick, key) => h("button", { key, style: { ...S.chip, ...(on ? S.chipOn : {}) }, "aria-pressed": on, onClick, disabled: !open }, label);
      const save = async () => {
        if (!session) return;
        const text = `Save ${summary(d.toys, st.toys)} for the night of ${nightLabel(st.night)}?\n\nThey are kept beside the night and logged with the time. Once the night has run they cannot be changed.`;
        if (!globalThis.confirm(text)) return;
        setSending(true);
        try { await session.publish(spec.channels[1], saveCommand(st, d)); setDraft({ ...d, dirty: false }); }
        catch (e) { globalThis.alert(`not sent: ${e && e.message ? e.message : e}`); }
        setSending(false);
      };
      body = [
        h("div", { key: "head", style: S.card },
          h("div", { style: { ...S.title, fontSize: "18px" } }, `Tonight's toys · ${nightLabel(st.night)}`),
          open ? h("div", { style: { lineHeight: 1.45 } }, "Lay out 8 toys, 2 of them bricks, over hard floor, carpet and the rug, away from last night's places. Then mark each: pick it, pick its floor, tap where it lies.")
            : h("div", { style: { lineHeight: 1.45 }, role: "note" }, "This night has run: its marks are closed. Its flags are on the Review page.")),
        open && h("div", { key: "toy", style: S.card },
          h("div", { style: S.title }, "1. The toy"),
          h("div", { style: S.row }, st.toys.map((t) => chip(d.name === t.name, t.name, () => setDraft({ ...d, name: t.name }), t.name)))),
        open && h("div", { key: "floor", style: S.card },
          h("div", { style: S.title }, "2. Its floor"),
          h("div", { style: S.row }, st.floors.map((f) => chip(d.floor === f.key, h("span", null,
            h("span", { style: { display: "inline-block", width: "10px", height: "10px", borderRadius: "5px", marginRight: "6px", background: FLOOR_COLOURS[f.key] } }), f.name),
            () => setDraft({ ...d, floor: f.key }), f.key)))),
        h("div", { key: "map", style: S.card },
          h("div", { style: S.title }, open ? "3. Tap where it lies" : "The marks"),
          st.map ? h(MapCanvas, { map: st.map, toys: d.toys, onTap: open ? (p) => setDraft(addToy(d, p)) : null })
            : h("div", { style: S.muted }, "No house map yet: he maps the house on his first nights."),
          open && h("div", { style: { ...S.muted, marginTop: "6px" } }, d.name ? `A tap marks: ${d.name}, ${(st.floors.find((f) => f.key === d.floor) || {}).name || d.floor}.` : "Pick a toy first.")),
        h("div", { key: "list", style: S.card, "data-hk": "marks-list" },
          h("div", { style: S.title }, d.toys.length ? summary(d.toys, st.toys) : "Nothing marked yet"),
          d.toys.map((t, i) => h("div", { key: i, style: S.item },
            h("span", { style: { overflowWrap: "anywhere" } }, h("b", null, `${i + 1}. `), `${t.name} · ${(st.floors.find((f) => f.key === t.floor) || {}).name || t.floor}`,
              h("span", { style: S.muted }, ` · ${t.x.toFixed(2)}, ${t.y.toFixed(2)} m`)),
            open && h("button", { style: { ...S.chip, flexShrink: 0 }, onClick: () => setDraft(removeToy(d, i)), "aria-label": `remove ${i + 1}` }, "Remove")))),
        open && h("div", { key: "save", style: S.card },
          h("div", { style: S.title }, d.dirty ? "Not saved" : st.marks ? `Saved ${String(st.marks.saved).replace("T", " ")}` : "Not saved yet"),
          h("div", { style: S.row },
            h("button", { style: { ...S.button, flex: "1 1 160px", ...(sending || !d.dirty ? S.off : {}) }, disabled: sending || !d.dirty, onClick: save }, "Save the marks"),
            h("button", { style: { ...S.button, ...S.quiet, flex: "1 1 120px", ...(!d.dirty ? S.off : {}) }, disabled: !d.dirty, onClick: () => setDraft(draftOf(st.marks, d.name, d.floor)) }, "Discard changes"))),
        st.message && h("div", { key: "msg", style: S.card, role: "status" }, st.message),
      ];
    }
    return h(PanelFrame, { spec, children: h("div", { style: S.page, "data-hk": "marks" }, body) });
  }

  registerPanel("hk-marks", Marks);
}
