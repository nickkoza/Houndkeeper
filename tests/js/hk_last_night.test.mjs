// [RzM] [Dlx]: the Last night page (cockpit/extensions/hk-last-night.js) from a recorded message
// (tests/fixtures/last_night_message.json, made by houndkeeper.project.last_night_message from the sim nights in
// data/runs/dimos_tidy and docs/research, whole texts cut to 4000 characters): each night with its numbers and their
// sources, what failed and its morning log; the findings with their opening and their whole text; that it sends nothing
// and nothing on it moves. React is the cockpit's own (HK_REACT_DIR: the fork's web/node_modules).
// Run: node --test 'tests/js/*.test.mjs'
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const SRC = readFileSync(join(HK, "cockpit", "extensions", "hk-last-night.js"), "utf8");
const CODE = SRC.replace(/\/\/.*$/gm, "");                                  // the code, without its comments
const page = await import("data:text/javascript;base64," + Buffer.from(SRC).toString("base64"));
const MSG = JSON.parse(readFileSync(join(HK, "tests", "fixtures", "last_night_message.json"), "utf8"));
const req = createRequire(join(process.env.HK_REACT_DIR || join(process.env.HOME, "houndkeeper/dimos-ext/web/node_modules"), "x.js"));
const React = req("react"), { renderToString } = req("react-dom/server");

function render(m, props = {}) {
  let Panel = null;
  page.register({
    React, PanelFrame: ({ children }) => React.createElement("div", null, children),
    registerPanel: (kind, C) => { assert.equal(kind, "hk-last-night"); Panel = C; },
    registerDecoder: () => assert.fail("registers no decoder"),
    useStoreChannel: (store, ch) => ({ slot: store.get(ch) }),
  });
  const store = { get: (ch) => (ch === "last_night" && m ? { value: m, version: 1 } : null), subscribe: () => () => {} };
  const spec = { id: "p0", kind: "hk-last-night", title: "Last night", channels: ["last_night"], params: {} };
  return renderToString(React.createElement(Panel, { spec, store, ...props }));
}

const esc = (s) => String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;").replace(/'/g, "&#x27;");
const text = (html) => html.replace(/<[^>]+>/g, "").replace(/&amp;/g, "&").replace(/&quot;/g, '"').replace(/&#x27;/g, "'").replace(/&lt;/g, "<").replace(/&gt;/g, ">");

test("each night shows its house, its numbers each with its source, what failed and its morning log", () => {
  const html = render(MSG), t = text(html);
  assert.ok(MSG.nights.length >= 2);
  for (const n of MSG.nights) {
    assert.ok(t.includes(`${n.when} · house ${n.house}`), n.id);
    for (const x of n.numbers) assert.ok(t.includes(x.label) && t.includes(x.text) && t.includes(`from ${x.source}`), `${n.id} ${x.label}`);
    for (const f of n.failed) assert.ok(t.includes(f), f);
    if (n.morning) for (const p of n.morning.prose) assert.ok(t.includes(p), p);
  }
  assert.ok(t.includes("scored against the sim's truth"));
  // the newest night open, the older ones folded
  const opens = [...html.matchAll(/<details([^>]*)>/g)].filter((m) => !/style="margin-top/.test(m[1]));
  assert.ok(/open/.test(opens[0][1]));
});

test("each finding opens with its answer and has its whole text a tap away", () => {
  const t = text(render(MSG));
  for (const f of MSG.findings) {
    assert.ok(t.includes(f.title) && t.includes(f.path), f.name);
    const firstWords = page.blocks(f.excerpt).find((b) => b.t === "p");
    if (firstWords) assert.ok(t.includes(page.inline(firstWords.text).map((p) => p.text).join("").slice(0, 60)), f.name);
  }
  assert.equal((t.match(/Read the whole document/g) || []).length, MSG.findings.filter((f) => f.full).length);
});

test("markdown: headings, paragraphs, lists, tables and code become blocks; no HTML passes through", () => {
  const b = page.blocks("## Answer\n\n**Yes**, by `12`.\n\n- one\n  continued\n- two\n\n| a | b |\n|---|---|\n| 1 | 2 |\n\n```\nx <y>\n```\n");
  assert.deepEqual(b.map((x) => x.t), ["h", "p", "ul", "table", "code"]);
  assert.deepEqual(b[2].items, ["one continued", "two"]);
  assert.deepEqual([b[3].head, b[3].rows], [["a", "b"], [["1", "2"]]]);
  assert.deepEqual(page.blocks("| | |\n|---|---|\n| Harness | x |")[0], { t: "table", head: null, rows: [["Harness", "x"]] });
  assert.deepEqual(page.inline("**Yes**, by `12` [see](x.md)").map((p) => p.t), ["b", "text", "code", "text", "link"]);
  const html = render({ ...MSG, findings: [{ ...MSG.findings[0], excerpt: "<script>alert(1)</script> <img src=x>" }] });
  assert.ok(!html.includes("<script>") && !html.includes("<img") && html.includes("&lt;script&gt;"));
});

test("with no message yet it says so; with no night or finding it says so", () => {
  assert.ok(render(null).includes("waiting for the project service"));
  const t = text(render({ ...MSG, nights: [], findings: [], trimmed: ["night 1"] }));
  assert.ok(t.includes("No night with a morning log or a score yet") && t.includes("No findings documents") && t.includes("night 1"));
});

test("it reads only: no session, nothing sent; nothing on it moves", () => {
  assert.ok(!/session|\.send\(|publish|fetch\(|WebSocket|XMLHttpRequest/.test(CODE));
  assert.ok(!/animation|transition|setInterval|requestAnimationFrame|blink|@keyframes/i.test(CODE));
  assert.ok(!/dangerouslySetInnerHTML|innerHTML/.test(CODE));
});
