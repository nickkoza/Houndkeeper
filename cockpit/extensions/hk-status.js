// The Status page ([F2q]): the battery's charge and power draw, the camera link (frame rate, frames lost, stalls),
// odometry's rate, and the robot network's last isolation check. Robot side: houndkeeper.cockpit_data (link_status).
// A Cockpit extension ([Eir]).

export function register({ React, registerPanel, PanelFrame, useStoreChannel }) {
  const h = React.createElement;
  const fmt = (v, digits = 1, unit = "") => (v === null || v === undefined ? "-" : `${Number(v).toFixed(digits)}${unit}`);
  const tile = (label, value, warn) => h("div", {
    key: label,
    style: { border: `1px solid ${warn ? "var(--warn, #d29922)" : "var(--border, #2a2f36)"}`, borderRadius: "10px", padding: "12px" },
  }, h("div", { style: { color: "var(--fg-muted, #9aa4b0)", fontSize: "13px" } }, label),
     h("div", { style: { fontSize: "22px", fontWeight: 600, fontVariantNumeric: "tabular-nums" } }, value));

  function Status({ spec, store }) {
    const snap = useStoreChannel(store, spec.channels[0]);
    const s = snap.slot === null ? null : snap.slot.value;
    const body = s === null
      ? h("div", { style: { color: "var(--fg-muted, #9aa4b0)" } }, "waiting for the robot...")
      : h("div", { style: { display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(140px, 1fr))", gap: "10px" } },
        tile("Battery", fmt(s.battery_soc, 0, " %"), s.battery_soc !== null && s.battery_soc < 20),
        tile("Power", fmt(s.power_w, 0, " W")),
        tile("Pack voltage", fmt(s.battery_v, 2, " V")),
        tile("Camera", fmt(s.camera_fps, 1, " fps"), s.camera_fps !== null && s.camera_fps < 10),
        tile("Frames lost", s.drop === null ? "-" : `${(s.drop * 100).toFixed(1)} %`, s.drop !== null && s.drop > 0.05),
        tile("Stalls a minute", fmt(s.stalls_per_min, 1), s.stalls_per_min !== null && s.stalls_per_min > 2),
        tile("Odometry", fmt(s.odom_hz, 0, " Hz")),
        tile("Robot network", s.isolation, s.isolation !== "passed" && !String(s.isolation).startsWith("passed")));
    return h(PanelFrame, { spec, children: h("div", { style: { padding: "12px", overflowY: "auto", height: "100%" } }, body) });
  }

  registerPanel("hk-status", Status);
}
