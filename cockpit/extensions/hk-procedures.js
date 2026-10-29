// The Procedures page ([vRJ]): pick a calibration procedure (real/calibration/), follow its steps while the robot
// records, watch its numbers live, then see the result beside the value in params/robot.yaml and write it only on
// confirm. Robot side: houndkeeper.cockpit_data (procedure_state in, procedure_cmd out). A Cockpit extension
// ([Eir]): plain JavaScript, everything from the api the cockpit passes in.

const S = {
  page: { padding: "12px", display: "flex", flexDirection: "column", gap: "12px", fontSize: "15px", overflowY: "auto", height: "100%" },
  card: { border: "1px solid var(--border, #2a2f36)", borderRadius: "10px", padding: "12px" },
  title: { fontWeight: 600, fontSize: "16px", marginBottom: "6px" },
  muted: { color: "var(--fg-muted, #9aa4b0)", fontSize: "13px" },
  button: { minHeight: "48px", padding: "0 18px", borderRadius: "10px", border: "none", fontSize: "16px", fontWeight: 600, cursor: "pointer", background: "var(--accent, #3b82f6)", color: "#fff" },
  stop: { background: "var(--danger, #ff5c5c)" },
  quiet: { background: "transparent", color: "var(--fg, #e6e9ee)", border: "1px solid var(--border, #2a2f36)" },
  table: { width: "100%", borderCollapse: "collapse", fontVariantNumeric: "tabular-nums" },
  cell: { padding: "6px 4px", borderBottom: "1px solid var(--border, #2a2f36)" },
  input: { minHeight: "44px", fontSize: "16px", width: "100%", boxSizing: "border-box", padding: "0 10px", borderRadius: "8px", border: "1px solid var(--border, #2a2f36)", background: "transparent", color: "inherit" },
};

const LABELS = {
  frames: "frames", walked_m: "walked (m)", turned_deg: "turned (deg)", from_start_m: "from the start (m)",
  drop: "frames lost singly", stall_per_min: "stalls a minute", stall_s: "stall length (s)", period_s: "frame period (s)",
  still_s: "still (s)", laps: "laps", soc: "charge (%)", power_w: "power (W)", used_pct: "charge used (%)",
};

function fmt(v) {
  if (v === null || v === undefined) return "-";
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : v.toFixed(Math.abs(v) < 1 ? 3 : 1);
  if (typeof v === "object") return Object.entries(v).map(([k, x]) => `${k} ${fmt(x)}`).join(", ");
  return String(v);
}

export function register({ React, registerPanel, PanelFrame, useStoreChannel }) {
  const h = React.createElement;
  const { useState } = React;

  function Rows({ values }) {
    const entries = Object.entries(values || {});
    if (!entries.length) return h("div", { style: S.muted }, "waiting for numbers...");
    return h("table", { style: S.table }, h("tbody", null, entries.map(([k, v]) =>
      h("tr", { key: k }, h("td", { style: S.cell }, LABELS[k] || k), h("td", { style: { ...S.cell, textAlign: "right" } }, fmt(v))))));
  }

  function Truth({ fields, send }) {
    const [vals, setVals] = useState({});
    return h("div", { style: S.card },
      h("div", { style: S.title }, "What you measured"),
      fields.map((f) => h("label", { key: f, style: { display: "block", marginBottom: "8px" } },
        h("div", { style: S.muted }, f.replace(/_/g, " ")),
        h("input", { style: S.input, inputMode: "decimal", value: vals[f] ?? "", onChange: (e) => setVals({ ...vals, [f]: e.target.value }) }))),
      h("button", {
        style: S.button,
        disabled: fields.some((f) => vals[f] === undefined || vals[f] === "" || Number.isNaN(Number(vals[f]))),
        onClick: () => send({ op: "truth", ...Object.fromEntries(fields.map((f) => [f, Number(vals[f])])) }),
      }, "Use these"));
  }

  function Procedures({ spec, store, session }) {
    const snap = useStoreChannel(store, spec.channels[0]);
    const [sending, setSending] = useState("");
    const st = snap.slot === null ? null : snap.slot.value;
    const send = async (cmd) => {
      if (!session) return;
      setSending(cmd.op);
      try { await session.publish(spec.channels[1], cmd); } catch (e) { alert(`not sent: ${e && e.message ? e.message : e}`); }
      setSending("");
    };
    let body;
    if (st === null) body = h("div", { style: S.muted }, "waiting for the robot...");
    else if (st.phase === "idle") {
      body = st.catalog.map((p) => h("div", { key: p.id, style: S.card },
        h("div", { style: S.title }, p.title),
        h("ol", { style: { margin: "0 0 10px 18px", padding: 0 } }, p.steps.map((s, i) => h("li", { key: i }, s))),
        h("div", { style: { ...S.muted, marginBottom: "10px" } }, `real/calibration/${p.doc}`),
        h("button", { style: S.button, disabled: !!sending, onClick: () => send({ op: "start", id: p.id }) }, "Start")));
    } else {
      const proc = st.catalog.find((p) => p.id === st.active) || { title: st.active, steps: [], needs_truth: [] };
      const recording = st.phase === "recording";
      const missing = proc.needs_truth.length > 0 && Object.keys(st.result || {}).length === 0;
      body = [
        h("div", { key: "head", style: S.card },
          h("div", { style: S.title }, proc.title),
          h("div", { style: S.muted }, recording ? `recording - ${fmt(st.elapsed_s)} s` : "done"),
          h("ol", { style: { margin: "8px 0 0 18px", padding: 0 } }, proc.steps.map((s, i) => h("li", { key: i }, s)))),
        recording && h("div", { key: "live", style: S.card }, h("div", { style: S.title }, "Live"), h(Rows, { values: st.live })),
        recording && h("button", { key: "stop", style: { ...S.button, ...S.stop }, disabled: !!sending, onClick: () => send({ op: "stop" }) }, "Stop"),
        !recording && missing && h(Truth, { key: "truth", fields: proc.needs_truth, send }),
        !recording && Object.keys(st.result || {}).length > 0 && h("div", { key: "result", style: S.card },
          h("div", { style: S.title }, "Result"), h(Rows, { values: st.result })),
        !recording && (st.writes || []).length > 0 && h("div", { key: "write", style: S.card },
          h("div", { style: S.title }, "Would replace in params/robot.yaml"),
          h(Rows, { values: Object.fromEntries((st.writes || []).map((w) => [w, st.current && st.current[w] ? `${fmt(st.current[w].value)} (${st.current[w].source})` : "not set"])) }),
          h("button", {
            style: { ...S.button, marginTop: "10px" }, disabled: !!sending,
            // The confirm names the values, and the write names this run: the robot writes nothing if the run it
            // holds is no longer the one confirmed here (a stale tab, another viewer).
            onClick: () => {
              const what = (st.writes || []).map((w) => `${w} = ${fmt((st.result || {})[w.split(".").pop()] ?? (st.result || {}))}`).join("\n");
              if (confirm(`Write to params/robot.yaml (run ${st.run})?\n${what}`)) send({ op: "write", run: st.run });
            },
          }, "Write to params")),
        st.message && h("div", { key: "msg", style: S.muted }, st.message),
        !recording && h("button", { key: "back", style: { ...S.button, ...S.quiet }, onClick: () => send({ op: "reset" }) }, "Back to the list"),
      ];
    }
    return h(PanelFrame, { spec, children: h("div", { style: S.page }, body) });
  }

  registerPanel("hk-procedures", Procedures);
}
