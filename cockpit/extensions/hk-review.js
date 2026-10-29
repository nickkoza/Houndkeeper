// The Review page ([71R]; decisions [hOy], [wTj], [Vd3], [oWO]): each flag of a real night that lies near no marked
// toy, at phone size, with its crop (people-screened on the laptop) and its place on the house's floor map; the user
// taps "False alarm" or "Real change" (something not laid out). Only false alarms count against the robot, and the
// night is final only when no flag is left unjudged. A tuning night (one of the first two) says it is not counted in
// the launch number. Project side: houndkeeper.night_scoring (review_state in, review_cmd out: a judgement, or the night
// to show). Steady colours only: nothing here pulses, blinks or animates (safe for photosensitive viewers). Sized
// for a phone held upright (390 px). A Cockpit extension ([Eir]).
import { nightLabel } from "./hk-marks.js";

const MUTED = "var(--fg-muted, #9aa4b0)", BORDER = "var(--border, #2a2f36)", FG = "var(--fg, #e6e9ee)";
export const VERDICTS = [
  { key: "false_alarm", label: "False alarm", colour: "#ff7b72", note: "nothing new there: counts against him" },
  { key: "real_change", label: "Real change", colour: "#3fb950", note: "something there that was not laid out" },
];
const FLAG = "#d29922", TOY = "#58a6ff", TOY_MISSED = "#ff7b72";
const VIEW_M = 1.5;                       // the place map shows this far round the flag each way

const S = {
  page: { padding: "12px", display: "flex", flexDirection: "column", gap: "12px", fontSize: "15px", overflowY: "auto", height: "100%", boxSizing: "border-box" },
  card: { border: `1px solid ${BORDER}`, borderRadius: "10px", padding: "12px" },
  title: { fontWeight: 600, fontSize: "16px", marginBottom: "6px", overflowWrap: "anywhere" },
  muted: { color: MUTED, fontSize: "13px", overflowWrap: "anywhere" },
  row: { display: "flex", flexWrap: "wrap", gap: "8px" },
  chip: { minHeight: "44px", padding: "0 12px", borderRadius: "10px", fontSize: "15px", cursor: "pointer", background: "transparent", color: FG, border: `1px solid ${BORDER}` },
  chipOn: { background: "#d29922", color: "#111", border: "1px solid #d29922", fontWeight: 600 },
  verdict: { flex: "1 1 140px", minHeight: "52px", borderRadius: "10px", fontSize: "16px", fontWeight: 600, cursor: "pointer", background: "transparent", color: FG },
};

// ---- plain functions, tested without a browser ----

// the flags in the order to judge them: those left first, each group by its number
export function flagOrder(flags) {
  return [...(flags || [])].sort((a, b) => (a.judged === null) === (b.judged === null) ? a.k - b.k : a.judged === null ? -1 : 1);
}

// what a tap sends: the night, every candidate in the flag, and the verdict (null takes a judgement back)
export function judgeCommand(night, flag, verdict) {
  return { op: "judge", night, ids: [...flag.ids], verdict };
}

// the night's headline: found, flags judged and left, and whether it is final
export function headline(n, flags) {
  const left = (flags || []).filter((f) => f.judged === null).length;
  return {
    found: n.laid_out ? `Found ${n.found[0]} of ${n.found[1]} laid-out toys` + (n.bricks[1] ? ` (1 x 1 bricks ${n.bricks[0]} of ${n.bricks[1]})` : "")
      : n.marked ? "No toys were marked for this night" : "No marks for this night: only its false alarms count",
    flags: `${(flags || []).length} flag${(flags || []).length === 1 ? "" : "s"} near no toy: ${n.false_alarms} false alarm${n.false_alarms === 1 ? "" : "s"}, ` +
      `${n.real_changes} real change${n.real_changes === 1 ? "" : "s"}, ${left} left to judge`,
    state: n.final ? "Final: every flag judged" : `Not final: ${left} left to judge`,
  };
}

// a house point (x, y) as the map image's pixel, and the view box round a flag (in those pixels)
export function mapPx(map, x, y) {
  return [(x - map.x0) / map.res, map.h - (y - map.y0) / map.res];
}
export function placeBox(map, x, y, m = VIEW_M) {
  const [u, v] = mapPx(map, x, y), r = m / map.res;
  return { x: u - r, y: v - r, w: 2 * r, h: 2 * r };
}

export function register({ React, registerPanel, PanelFrame, useStoreChannel }) {
  const h = React.createElement;
  const { useState, useEffect } = React;

  function Place({ map, flag, toys }) {
    if (!map) return h("div", { style: { ...S.muted, padding: "8px" } }, "No house map.");
    const b = placeBox(map, flag.x, flag.y), r = 0.12 / map.res;
    const near = (toys || []).filter((t) => Math.abs(t.x - flag.x) <= VIEW_M && Math.abs(t.y - flag.y) <= VIEW_M);
    const [u, v] = mapPx(map, flag.x, flag.y);
    return h("svg", { viewBox: `${b.x} ${b.y} ${b.w} ${b.h}`, role: "img", "aria-label": `where flag ${flag.k} lies on the house's floor map`,
      style: { width: "100%", aspectRatio: "1", display: "block", background: "#000", borderRadius: "8px", overflow: "hidden" } },
      h("image", { href: `data:image/png;base64,${map.png}`, x: 0, y: 0, width: map.w, height: map.h, preserveAspectRatio: "none", style: { imageRendering: "pixelated" } }),
      near.map((t) => { const [p, q] = mapPx(map, t.x, t.y); return h("rect", { key: t.id, x: p - r * 0.6, y: q - r * 0.6, width: r * 1.2, height: r * 1.2,
        fill: "none", stroke: t.found ? TOY : TOY_MISSED, strokeWidth: r * 0.35 }); }),
      h("circle", { cx: u, cy: v, r, fill: "none", stroke: FLAG, strokeWidth: r * 0.4 }),
      h("circle", { cx: u, cy: v, r: r * 0.25, fill: FLAG }));
  }

  function Flag({ f, map, toys, onJudge, busy }) {
    const pic = f.jpg
      ? h("img", { alt: `the floor where flag ${f.k} was raised`, src: `data:image/jpeg;base64,${f.jpg}`,
          style: { display: "block", width: "100%", aspectRatio: "1", objectFit: "cover", borderRadius: "8px" } })
      : h("div", { style: { aspectRatio: "1", display: "flex", alignItems: "center", justifyContent: "center", textAlign: "center",
          color: MUTED, fontSize: "13px", padding: "8px", border: `1px dashed ${BORDER}`, borderRadius: "8px", boxSizing: "border-box" } },
          `No picture: ${f.no_picture || "none"}`);
    const chosen = VERDICTS.find((x) => x.key === f.judged);
    return h("div", { key: `flag-${f.k}`, style: { ...S.card, borderColor: chosen ? chosen.colour : BORDER }, "data-hk": "review-flag" },
      h("div", { style: S.title }, `Flag ${f.k}`, h("span", { style: { ...S.muted, fontWeight: "normal" } }, ` · ${f.x.toFixed(2)}, ${f.y.toFixed(2)} m`)),
      h("div", { style: { display: "grid", gridTemplateColumns: "1fr 1fr", gap: "8px" } }, pic, h(Place, { map, flag: f, toys })),
      h("div", { style: { ...S.row, marginTop: "10px" } }, VERDICTS.map((x) => {
        const on = f.judged === x.key;
        return h("button", { key: x.key, "aria-pressed": on, disabled: busy || on, onClick: () => onJudge(f, x.key),
          style: { ...S.verdict, border: `2px solid ${x.colour}`, ...(on ? { background: x.colour, color: "#111", cursor: "default" } : {}) } }, x.label);
      })),
      h("div", { style: { ...S.muted, marginTop: "6px", display: "flex", alignItems: "center", justifyContent: "space-between", gap: "8px", flexWrap: "wrap" } },
        chosen ? `Judged: ${chosen.label.toLowerCase()} (${chosen.note}).` : "Not judged yet.",
        chosen ? h("button", { style: { ...S.chip, minHeight: "44px" }, disabled: busy, onClick: () => onJudge(f, null) }, "Take back") : null));
  }

  function Review({ spec, store, session }) {
    const snap = useStoreChannel(store, spec.channels[0]);
    const m = snap.slot === null ? null : snap.slot.value;
    const [busy, setBusy] = useState(false);
    useEffect(() => { setBusy(false); }, [m]);
    const send = async (cmd) => {
      if (!session) return;
      setBusy(true);
      try { await session.publish(spec.channels[1], cmd); }
      catch (e) { globalThis.alert(`not sent: ${e && e.message ? e.message : e}`); setBusy(false); }
    };
    let body;
    if (m === null) body = h("div", { style: S.muted }, "waiting for the project service...");
    else if (!m.night) body = h("div", { style: S.card }, h("div", { style: S.title }, "No real night yet"),
      h("div", { style: S.muted }, "A night's flags appear here the morning after it runs."));
    else {
      const n = m.night, hd = headline(n, m.flags), missed = m.toys.filter((t) => !t.found);
      body = [
        m.nights.length > 1 && h("div", { key: "nights", style: S.row, "data-hk": "review-nights" }, m.nights.map((x) =>
          h("button", { key: x.name, style: { ...S.chip, ...(x.name === n.name ? S.chipOn : {}) }, "aria-pressed": x.name === n.name, disabled: busy,
            onClick: () => send({ op: "show", night: x.name }) }, `${nightLabel(x.name)} · ${x.final ? "final" : `${x.unjudged} to judge`}`))),
        h("div", { key: "head", style: S.card, "data-hk": "review-head" },
          h("div", { style: { ...S.title, fontSize: "18px" } }, `The night of ${nightLabel(n.name)}`),
          n.tuning && h("div", { role: "note", style: { border: "2px solid #d29922", borderRadius: "8px", padding: "8px 10px", margin: "6px 0", lineHeight: 1.4 } },
            "A tuning night: it sets the light check's threshold, and is not counted in the launch number."),
          h("div", { style: { lineHeight: 1.5 } }, hd.found), h("div", { style: { lineHeight: 1.5 } }, hd.flags),
          h("div", { style: { fontWeight: 600, marginTop: "4px", color: n.final ? "#3fb950" : "#d29922" } }, hd.state),
          missed.length ? h("div", { style: { ...S.muted, marginTop: "6px" } }, `Not found: ${missed.map((t) => `${t.name} (${t.floor})`).join(", ")}.`) : null,
          m.pictures_note ? h("div", { style: { ...S.muted, marginTop: "6px" } }, m.pictures_note) : null),
        m.flags.length ? null : h("div", { key: "none", style: { ...S.card, ...S.muted } }, "No flag near no toy: nothing to judge."),
        ...flagOrder(m.flags).map((f) => h(Flag, { key: `f${f.k}`, f, map: m.map, toys: m.toys, busy,
          onJudge: (fl, v) => send(judgeCommand(n.name, fl, v)) })),
        m.message && h("div", { key: "msg", style: S.card, role: "status" }, m.message),
      ];
    }
    return h(PanelFrame, { spec, children: h("div", { style: S.page, "data-hk": "review" }, body) });
  }

  registerPanel("hk-review", Review);
}
