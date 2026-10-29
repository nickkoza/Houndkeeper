// The Last night page ([RzM]): each recent night (the morning log, the night's numbers each with its source file, what
// failed) and the newest research write-ups (docs/research), each opening with its answer and readable whole on the page. Project side:
// houndkeeper.project (last_night), served with no robot or sim running ([eTc]). Read only ([Dlx]): the panel takes no
// session and sends nothing. Steady colours only: nothing here pulses, blinks or animates (safe for photosensitive
// viewers). Sized for a phone held upright (390 px): text wraps, wide tables scroll inside their own box.
// A Cockpit extension ([Eir]).

const MUTED = "var(--fg-muted, #9aa4b0)", BORDER = "var(--border, #2a2f36)", ACCENT = "#d29a8f", WARN = "#d29922";

// Markdown, the part findings use, as blocks: headings, paragraphs, lists, tables, fenced code. No HTML is passed through.
export function blocks(md) {
  const out = [], lines = String(md || "").split("\n");
  let i = 0;
  const isTable = (l) => /^\s*\|.*\|\s*$/.test(l);
  const cells = (l) => l.trim().replace(/^\|/, "").replace(/\|$/, "").split("|").map((c) => c.trim());
  while (i < lines.length) {
    const l = lines[i];
    if (!l.trim()) { i++; continue; }
    if (l.startsWith("```")) {
      const body = []; i++;
      while (i < lines.length && !lines[i].startsWith("```")) body.push(lines[i++]);
      i++; out.push({ t: "code", text: body.join("\n") }); continue;
    }
    const hm = /^(#{1,6})\s+(.*)$/.exec(l);
    if (hm) { out.push({ t: "h", level: hm[1].length, text: hm[2] }); i++; continue; }
    if (isTable(l)) {
      const raw = [];
      while (i < lines.length && isTable(lines[i])) raw.push(cells(lines[i++]));
      const sep = (c) => c.every((x) => /^:?-{2,}:?$/.test(x));
      const headed = raw.length > 1 && sep(raw[1]);
      const head = headed && raw[0].some((x) => x) ? raw[0] : null;          // an empty header row is no header
      out.push({ t: "table", head, rows: raw.slice(headed ? 2 : 0).filter((c) => !sep(c)) }); continue;
    }
    const lm = /^(\s*)([-*]|\d+\.)\s+(.*)$/.exec(l);
    if (lm) {
      const ordered = /\d/.test(lm[2]), items = [];
      while (i < lines.length) {
        const m = /^(\s*)([-*]|\d+\.)\s+(.*)$/.exec(lines[i]);
        if (m && m[1].length <= 1) { items.push(m[3]); i++; }
        else if (lines[i].trim() && /^\s+/.test(lines[i]) && items.length) { items[items.length - 1] += " " + lines[i].trim(); i++; }
        else break;
      }
      out.push({ t: ordered ? "ol" : "ul", items }); continue;
    }
    const para = [];
    while (i < lines.length && lines[i].trim() && !/^(#{1,6}\s|```|\s*([-*]|\d+\.)\s)/.test(lines[i]) && !isTable(lines[i])) para.push(lines[i++].trim());
    out.push({ t: "p", text: para.join(" ") });
  }
  return out;
}

// inline: **bold**, `code`, [text](target) (the text kept, the target shown after it), the rest plain
export function inline(text) {
  const out = [], re = /\*\*([^*]+)\*\*|`([^`]+)`|\[([^\]]+)\]\(([^)]+)\)/g;
  let last = 0, m;
  while ((m = re.exec(text)) !== null) {
    if (m.index > last) out.push({ t: "text", text: text.slice(last, m.index) });
    if (m[1] !== undefined) out.push({ t: "b", text: m[1] });
    else if (m[2] !== undefined) out.push({ t: "code", text: m[2] });
    else out.push({ t: "link", text: m[3], href: m[4] });
    last = re.lastIndex;
  }
  if (last < text.length) out.push({ t: "text", text: text.slice(last) });
  return out;
}

export function register({ React, registerPanel, PanelFrame, useStoreChannel }) {
  const h = React.createElement;
  const wrap = { overflowWrap: "anywhere" };
  const codeStyle = { fontFamily: "ui-monospace, monospace", fontSize: "inherit", background: "rgba(127,127,127,0.15)",
    borderRadius: "4px", padding: "0 3px" };

  const Inline = (text, key) => h("span", { key }, inline(text).map((p, i) =>
    p.t === "b" ? h("b", { key: i }, p.text)
      : p.t === "code" ? h("code", { key: i, style: codeStyle }, p.text)
        : p.t === "link" ? h("span", { key: i }, h("u", null, p.text), h("span", { style: { color: MUTED } }, ` (${p.href})`))
          : h("span", { key: i }, p.text)));

  function Markdown(md, key) {
    return h("div", { key, style: { ...wrap, fontSize: "15px", lineHeight: 1.5 } }, blocks(md).map((b, i) => {
      if (b.t === "h") return h(`h${Math.min(b.level + 2, 6)}`, { key: i, style: { margin: "14px 0 6px", fontSize: b.level <= 2 ? "17px" : "15px" } }, Inline(b.text));
      if (b.t === "p") return h("p", { key: i, style: { margin: "0 0 10px" } }, Inline(b.text));
      if (b.t === "ul" || b.t === "ol") return h(b.t, { key: i, style: { margin: "0 0 10px", paddingLeft: "20px" } },
        b.items.map((it, j) => h("li", { key: j, style: { marginBottom: "4px" } }, Inline(it))));
      if (b.t === "code") return h("pre", { key: i, style: { ...codeStyle, display: "block", padding: "8px", overflowX: "auto", whiteSpace: "pre" } }, b.text);
      return h("div", { key: i, style: { overflowX: "auto", margin: "0 0 10px" } },
        h("table", { style: { borderCollapse: "collapse", fontSize: "13px" } }, h("tbody", null,
          (b.head ? [b.head] : []).concat(b.rows).map((r, j) =>
            h("tr", { key: j }, r.map((c, k) => h(b.head && j === 0 ? "th" : "td", { key: k, style: { border: `1px solid ${BORDER}`,
              padding: "4px 6px", textAlign: "left", verticalAlign: "top", minWidth: "60px" } }, Inline(c))))))));
    }));
  }

  const head = (t) => h("h3", { style: { fontWeight: "normal", fontVariant: "small-caps", color: ACCENT, margin: "18px 0 8px", fontSize: "17px" } }, t);
  const small = (t) => h("div", { style: { color: MUTED, fontSize: "13px", ...wrap } }, t);

  function Numbers(n) {
    return h("div", { style: { display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(150px, 1fr))", gap: "8px" } },
      n.numbers.map((x) => h("div", { key: x.label, style: { border: `1px solid ${BORDER}`, borderRadius: "8px", padding: "8px 10px" } },
        small(x.label), h("div", { style: { fontSize: "18px", fontWeight: 600, fontVariantNumeric: "tabular-nums", ...wrap } }, x.text),
        x.note ? small(x.note) : null, small(`from ${x.source}`))));
  }

  function Morning(m) {
    if (!m) return small("No morning log for this night.");
    return h("div", null,
      h("div", { style: { fontFamily: "Georgia, serif", fontSize: "16px", lineHeight: 1.5 } },
        m.prose.map((p, i) => h("p", { key: i, style: { margin: "0 0 0.5em" } }, p))),
      m.finds.length ? h("ul", { style: { margin: "6px 0", paddingLeft: 0, listStyle: "none", fontSize: "15px", lineHeight: 1.45 } },
        m.finds.map((f, i) => h("li", { key: i }, `${f.k ?? ""}${f.k ? ". " : ""}${f.what}: ${f.fate.replace(/_/g, " ")}${f.clock ? `, ${f.clock}` : ""}`))) : null,
      m.pictures_note ? small(m.pictures_note) : null,
      h("details", { style: { marginTop: "8px" } }, h("summary", { style: { color: ACCENT } }, "The log's figures"),
        h("table", { style: { width: "100%", fontSize: "13px", borderCollapse: "collapse" } }, h("tbody", null,
          m.figures.map((f) => h("tr", { key: f.label }, h("td", { style: { padding: "3px 4px 3px 0", ...wrap } }, f.label),
            h("td", { style: { padding: "3px 4px", ...wrap } }, f.text), h("td", { style: { color: MUTED, ...wrap } }, f.source)))))),
      small(`from ${m.source}`));
  }

  function Night(n, open) {
    const body = h("div", { style: { padding: "4px 0 8px" } },
      small(`House ${n.house}${n.mode ? ` · ${n.mode}` : ""} · ${n.dir}`),
      n.scored_against_truth ? small("Found, binned and false stops are scored against the sim's truth (score.json).") : null,
      head("Numbers"), Numbers(n),
      head("What failed"),
      n.failed.length ? h("ul", { style: { margin: 0, paddingLeft: "20px", fontSize: "15px", lineHeight: 1.45, ...wrap } },
        n.failed.map((f, i) => h("li", { key: i, style: { color: WARN, marginBottom: "3px" } }, h("span", { style: { color: "inherit" } }, f))))
        : small("Nothing recorded as failed."),
      head("Morning log"), Morning(n.morning));
    return h("details", { key: n.id, open, style: { borderTop: `1px solid ${BORDER}`, padding: "10px 0" } },
      h("summary", { style: { fontSize: "17px", fontWeight: 600, ...wrap } }, `${n.when} · house ${n.house}`), body);
  }

  function Finding(f) {
    return h("div", { key: f.name, style: { border: `1px solid ${BORDER}`, borderRadius: "8px", padding: "10px 12px", marginBottom: "12px" } },
      h("div", { style: { fontSize: "16px", fontWeight: 600, lineHeight: 1.35, ...wrap } }, f.title),
      small(`${f.date} · ${f.path}${f.section ? ` · ${f.section}` : ""}`),
      h("div", { style: { marginTop: "8px" } }, Markdown(f.excerpt, "x")),
      f.excerpt_cut ? small("(opening cut short)") : null,
      f.full ? h("details", null, h("summary", { style: { color: ACCENT, padding: "6px 0" } }, "Read the whole document"),
        Markdown(f.full, "full"), f.full_cut ? small(`Cut short for the phone; the rest is in ${f.path}.`) : null)
        : small(`The whole text is in ${f.path} (left off to fit the phone).`));
  }

  function LastNight({ spec, store }) {
    const snap = useStoreChannel(store, spec.channels[0]);
    const m = snap.slot === null ? null : snap.slot.value;
    const body = m === null
      ? h("div", { style: { color: MUTED } }, "waiting for the project service...")
      : h("div", { style: { maxWidth: "680px" } },
        h("h2", { style: { fontWeight: "normal", margin: "0 0 4px", fontSize: "22px" } }, "Last night"),
        small(`As of ${m.made.replace("T", " ")}`),
        m.nights.length ? m.nights.map((n, i) => Night(n, i === 0)) : h("p", { style: { color: MUTED } }, "No night with a morning log or a score yet."),
        head("Findings, newest first"),
        m.findings.length ? m.findings.map(Finding) : small("No findings documents."),
        m.trimmed.length ? small(`Left off to fit the phone: ${m.trimmed.join("; ")}.`) : null);
    return h(PanelFrame, { spec, children: h("div", { "data-hk": "last-night", style: { padding: "12px 14px", overflowY: "auto", height: "100%", boxSizing: "border-box" } }, body) });
  }

  registerPanel("hk-last-night", LastNight);
}
