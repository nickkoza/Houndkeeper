// The Morning page ([Ndt]): the butler's morning log, the latest night's ([uoW]). His prose, a picture, time and place
// for each find, a small map of his walk, and the night's figures with the file each came from. Robot side:
// houndkeeper.morning_log (morning_message), through cockpit_data.MorningLogFeed. Read only: the panel takes no session
// and sends nothing. Steady colours only: nothing here pulses, blinks or animates (safe for photosensitive
// viewers). A Cockpit extension ([Eir]).

const MUTED = "var(--fg-muted, #9aa4b0)", BORDER = "var(--border, #2a2f36)", ACCENT = "#d29a8f", ROUTE = "#8fb3d1";

// the map's view box round the route and the finds, a metre spare (y flipped: SVG's y runs down)
export function mapBox(m) {
  const pts = (m.route || []).concat((m.finds || []).map((f) => [f.x, f.y]), m.nothing || []);
  if (!pts.length) return null;
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const x0 = Math.min(...xs) - 1, x1 = Math.max(...xs) + 1, y0 = Math.min(...ys) - 1, y1 = Math.max(...ys) + 1;
  return { x: x0, y: -y1, w: x1 - x0, h: y1 - y0 };
}

export function register({ React, registerPanel, PanelFrame, useStoreChannel }) {
  const h = React.createElement;

  function Card(f) {
    const pic = f.jpg
      ? h("img", { alt: `Find ${f.k}: ${f.what}`, src: `data:image/jpeg;base64,${f.jpg}`,
          style: { display: "block", width: "100%", aspectRatio: "1", objectFit: "cover" } })
      : h("div", { style: { aspectRatio: "1", display: "flex", alignItems: "center", justifyContent: "center", color: MUTED,
          fontStyle: "italic", padding: "12px", textAlign: "center" } }, "No picture.");
    const where = f.from_start_m === null || f.from_start_m === undefined ? "" : `${f.from_start_m} m from where I started`;
    return h("div", { key: `find-${f.k}`, style: { border: `1px solid ${BORDER}`, borderRadius: "8px", overflow: "hidden" } }, pic,
      h("div", { style: { padding: "8px 10px", fontSize: "15px", lineHeight: 1.4 } },
        h("b", null, `${f.k}. `), `${f.what.charAt(0).toUpperCase()}${f.what.slice(1)}. ${f.card}`,
        h("div", { style: { color: MUTED, fontSize: "13px" } }, [f.clock, where].filter(Boolean).join(" · "))));
  }

  function MapView(m) {
    const b = mapBox(m);
    if (b === null) return null;
    const flip = (p) => `${p[0].toFixed(2)},${(-p[1]).toFixed(2)}`, kids = [];
    if ((m.route || []).length > 1) kids.push(h("polyline", { key: "route", points: m.route.map(flip).join(" "), fill: "none",
      stroke: ROUTE, strokeWidth: 0.05, strokeLinejoin: "round" }));
    if (m.start) kids.push(h("rect", { key: "start", x: m.start[0] - 0.15, y: -m.start[1] - 0.15, width: 0.3, height: 0.3, fill: "#e6edf3" }));
    for (const [i, p] of (m.nothing || []).entries()) kids.push(h("circle", { key: `n${i}`, cx: p[0], cy: -p[1], r: 0.07, fill: "#6e6556" }));
    for (const f of m.finds || []) kids.push(h("g", { key: `f${f.k}` }, h("circle", { cx: f.x, cy: -f.y, r: 0.2, fill: ACCENT }),
      h("text", { x: f.x, y: -f.y + 0.08, fill: "#1c1a17", fontSize: 0.22, textAnchor: "middle", fontWeight: "bold" }, String(f.k))));
    return h("svg", { viewBox: `${b.x} ${b.y} ${b.w} ${b.h}`, role: "img", "aria-label": "The house from above: his walk and each find",
      style: { width: "100%", height: "auto", border: `1px solid ${BORDER}`, borderRadius: "8px" } }, kids);
  }

  function Morning({ spec, store }) {
    const snap = useStoreChannel(store, spec.channels[0]);
    const m = snap.slot === null ? null : snap.slot.value;
    const head = (t) => h("h3", { style: { fontWeight: "normal", fontVariant: "small-caps", color: ACCENT, margin: "22px 0 8px" } }, t);
    const body = m === null
      ? h("div", { style: { color: MUTED } }, "waiting for the robot: no night's log yet")
      : h("div", { style: { fontFamily: "Georgia, serif", fontSize: "17px", lineHeight: 1.55, maxWidth: "680px" } },
        h("div", { style: { color: MUTED, fontSize: "13px" } }, `Barrymore's morning log${m.night && m.night.kind === "sim" ? " · a simulated night" : ""}`),
        h("h2", { style: { fontWeight: "normal", margin: "4px 0 12px" } }, m.title),
        m.prose.map((p, i) => h("p", { key: `p${i}`, style: { margin: "0 0 0.6em" } }, p)),
        h("p", { style: { fontStyle: "italic", textAlign: "right" } }, "Barrymore"),
        m.finds.length ? head("What I found") : null,
        m.finds.length ? h("div", { style: { display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(150px, 1fr))", gap: "12px" } },
          m.finds.map(Card)) : null,
        m.pictures_note ? h("div", { style: { color: MUTED, fontSize: "14px", fontStyle: "italic", marginTop: "6px" } }, m.pictures_note) : null,
        mapBox(m) ? head("The house, and my walk") : null, MapView(m),
        h("details", { style: { marginTop: "22px" } }, h("summary", { style: { color: ACCENT } }, "The night's figures"),
          h("table", { style: { width: "100%", fontSize: "14px", borderCollapse: "collapse" } }, h("tbody", null,
            m.figures.map((f) => h("tr", { key: f.label }, h("td", null, f.label), h("td", null, f.text),
              h("td", { style: { color: MUTED } }, f.source)))))));
    return h(PanelFrame, { spec, children: h("div", { style: { padding: "12px", overflowY: "auto", height: "100%" } }, body) });
  }

  registerPanel("hk-morning-log", Morning);
}
