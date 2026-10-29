// [Ndt]: the Morning page (cockpit/extensions/hk-morning-log.js) from a recorded message (tests/fixtures/morning_message.json,
// made by houndkeeper.morning_log.morning_message from the morning log's test night): his prose, a card with its JPEG
// crop for each find, the map, the figures with their sources; that it sends nothing and nothing on it moves. React is
// the cockpit's own (HK_REACT_DIR: the fork's web/node_modules). Run: node --test 'tests/js/*.test.mjs'
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const SRC = readFileSync(join(HK, "cockpit", "extensions", "hk-morning-log.js"), "utf8");
const CODE = SRC.replace(/\/\/.*$/gm, "");                                  // the code, without its comments
const page = await import("data:text/javascript;base64," + Buffer.from(SRC).toString("base64"));
const MSG = JSON.parse(readFileSync(join(HK, "tests", "fixtures", "morning_message.json"), "utf8"));
const req = createRequire(join(process.env.HK_REACT_DIR || join(process.env.HOME, "houndkeeper/dimos-ext/web/node_modules"), "x.js"));
const React = req("react"), { renderToString } = req("react-dom/server");

function render(m, props = {}) {
  let Panel = null;
  page.register({
    React, PanelFrame: ({ children }) => React.createElement("div", null, children),
    registerPanel: (kind, C) => { assert.equal(kind, "hk-morning-log"); Panel = C; },
    registerDecoder: () => assert.fail("registers no decoder"),
    useStoreChannel: (store, ch) => ({ slot: store.get(ch) }),
  });
  const store = { get: (ch) => (ch === "morning_log" && m ? { value: m, version: 1 } : null), subscribe: () => () => {} };
  const spec = { id: "p6", kind: "hk-morning-log", title: "Morning", channels: ["morning_log"], params: {} };
  return renderToString(React.createElement(Panel, { spec, store, ...props }));
}

const esc = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;").replace(/'/g, "&#x27;");

test("the page renders his prose, the title and every figure with its source", () => {
  const html = render(MSG);
  assert.ok(html.includes(esc(MSG.title)));
  for (const p of MSG.prose) assert.ok(html.includes(esc(p)), p);
  for (const f of MSG.figures) assert.ok(html.includes(esc(f.label)) && html.includes(esc(f.source)), f.label);
  assert.ok(render(null).includes("waiting for the robot"));
});

test("each find has a card with its JPEG crop, time and place, and a numbered mark on the map", () => {
  const html = render(MSG);
  assert.ok(MSG.finds.length >= 2);
  for (const f of MSG.finds) {
    assert.ok(html.includes(`src="data:image/jpeg;base64,${f.jpg}"`), `find ${f.k}`);
    assert.ok(html.includes(esc(f.card)) && html.includes(f.clock), `find ${f.k}`);
  }
  assert.equal((html.match(/<circle[^>]*r="0.2"/g) || []).length, MSG.finds.length);
  const noPics = render({ ...MSG, finds: MSG.finds.map((f) => ({ ...f, jpg: null })) });
  assert.ok(!noPics.includes("data:image/jpeg") && noPics.includes("No picture."));
});

test("the map fits round the route and every find", () => {
  const b = page.mapBox(MSG);
  for (const p of MSG.route.concat(MSG.finds.map((f) => [f.x, f.y]))) {
    assert.ok(p[0] > b.x && p[0] < b.x + b.w && -p[1] > b.y && -p[1] < b.y + b.h);
  }
  assert.equal(page.mapBox({ route: [], finds: [] }), null);
});

test("the page sends nothing, whatever it is given", () => {
  const session = new Proxy({}, { get: () => () => assert.fail("the page used the session") });
  render(MSG, { session, teleop: session });
  assert.ok(!/\.publish\s*\(/.test(CODE) && !/\bsession\b/.test(CODE) && !/\bteleop\b/.test(CODE) && !/\bfetch\s*\(/.test(CODE));
});

test("nothing on the page moves by itself", () => {
  for (const w of ["animation", "transition", "@keyframes", "setInterval", "setTimeout", "requestAnimationFrame", "blink", "<animate"]) {
    assert.ok(!CODE.includes(w), w);
  }
});
