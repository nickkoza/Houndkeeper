// [daF] [HZO]: the Area page (cockpit/extensions/hk-area.js): taps on the map become corners in the house's frame, an
// area is closed and kept only with 3+ corners, a wall is two taps, Save sends one confirmed command naming the change it
// replaces; the warning that it is no safety barrier is on the page whatever its state; nothing on it moves.
// React is the cockpit's own (HK_REACT_DIR: the fork's web/node_modules). Run: node --test 'tests/js/*.test.mjs'
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const SRC = readFileSync(join(HK, "cockpit", "extensions", "hk-area.js"), "utf8");
const CODE = SRC.replace(/\/\/.*$/gm, "");
const page = await import("data:text/javascript;base64," + Buffer.from(SRC).toString("base64"));
const req = createRequire(join(process.env.HK_REACT_DIR || join(process.env.HOME, "houndkeeper/dimos-ext/web/node_modules"), "x.js"));
const React = req("react"), { renderToString } = req("react-dom/server");

const MAP = { png: "iVBORw0KGgo=", x0: -2, y0: -1, res: 0.05, w: 200, h: 100 };   // 10 m x 5 m
const WARNING = "The operating area is not a safety barrier: it holds only while he knows where he is.";

function render(st) {
  let Panel = null;
  page.register({
    React, PanelFrame: ({ children }) => React.createElement("div", null, children),
    registerPanel: (kind, C) => { assert.equal(kind, "hk-area"); Panel = C; },
    useStoreChannel: (store, ch) => ({ slot: store.get(ch) }),
  });
  const store = { get: (ch) => (ch === "area_state" && st ? { value: st, version: 1 } : null), subscribe: () => () => {} };
  const spec = { id: "p7", kind: "hk-area", title: "Area", channels: ["area_state", "area_cmd"], params: {} };
  return renderToString(React.createElement(Panel, { spec, store }));
}

test("a tap is a point in the house's frame, and back", () => {
  const t = page.fit(MAP, 390);
  assert.equal(t.H, 195);                                           // the map's 2:1 kept
  assert.deepEqual(page.toHouse(MAP, t, 0, t.H), [-2, -1]);          // bottom-left
  assert.deepEqual(page.toHouse(MAP, t, 390, 0), [8, 4]);            // top-right
  const [u, v] = page.toCanvas(MAP, t, 3, 1.5);
  assert.deepEqual(page.toHouse(MAP, t, u, v), [3, 1.5]);
});

test("an area needs three corners; a wall is two taps; undo and remove", () => {
  let d = page.setMode(page.draftOf(null), "area");
  d = page.addPoint(d, [0, 0]); d = page.addPoint(d, [2, 0]);
  assert.equal(page.finish(d).allowed.length, 0);                   // two corners: no area
  d = page.addPoint(d, [2, 2]); d = page.undo(d); d = page.addPoint(d, [2, 3]);
  d = page.finish(d);
  assert.deepEqual(d.allowed, [[[0, 0], [2, 0], [2, 3]]]);
  d = page.setMode(d, "wall"); d = page.addPoint(d, [1, 0]);
  assert.equal(d.walls.length, 0);
  d = page.addPoint(d, [1, 1]);
  assert.deepEqual(d.walls, [[[1, 0], [1, 1]]]);
  assert.equal(page.summary(d), "1 area and 1 virtual wall");
  d = page.removeLast(d);                                           // in wall mode: the last wall
  assert.equal(d.walls.length, 0); assert.equal(d.allowed.length, 1);
  assert.equal(page.addPoint(page.setMode(d, "view"), [5, 5]).allowed.length, 1);   // viewing: a tap adds nothing
});

test("save sends the finished shapes, the change it replaces, and the confirmation", () => {
  let d = page.setMode(page.draftOf({ allowed: [[[0, 0], [1, 0], [1, 1]]], walls: [] }), "area");
  for (const p of [[3, 0], [4, 0], [4, 1]]) d = page.addPoint(d, p);   // unfinished: Save finishes it
  const cmd = page.saveCommand(d, "2026-10-05T13:00:00-0700");
  assert.deepEqual(cmd, { op: "save", confirmed: true, seen: "2026-10-05T13:00:00-0700",
    area: { frame_id: "house", allowed: [[[0, 0], [1, 0], [1, 1]], [[3, 0], [4, 0], [4, 1]]], walls: [] } });
  assert.match(CODE, /confirm\(/);                                    // the page asks before sending
  assert.equal((CODE.match(/session\.publish\(/g) || []).length, 1); // and sends nothing else
});

test("the warning is on the page in every state", () => {
  assert.ok(render(null).includes("not a safety barrier"));
  const st = { map: MAP, area: { allowed: [[[0, 0], [1, 0], [1, 1]]], walls: [], changed: "2026-10-05T13:00:00-0700", by: "t" },
               dock: [0.5, 0.2], history: [{ changed: "2026-10-05T13:00:00-0700", by: "t", polygons: 1, walls: 0 }], warning: WARNING, message: "" };
  const html = render(st);
  assert.ok(html.includes(WARNING) && html.includes("Stored: 1 area and 0 virtual walls") && html.includes("Last changes"));
  assert.ok(render({ ...st, map: null, area: null, history: [] }).includes("No house map yet"));
});

test("nothing on the page moves", () => {
  assert.doesNotMatch(CODE, /animation|transition|@keyframes|setInterval|requestAnimationFrame|blink/i);
});

test("the scene draws the areas, walls, the corners being placed and the dock", () => {
  const t = page.fit(MAP, 390);
  let d = page.draftOf({ allowed: [[[0, 0], [1, 0], [1, 1]]], walls: [[[2, 0], [2, 1]]] });
  d = page.addPoint(page.setMode(d, "area"), [3, 3]);
  const ops = page.sceneOps(MAP, t, d, [0.5, 0.2]);
  assert.deepEqual(ops.map((o) => o.t), ["poly", "line", "line", "dot", "dot"]);
  assert.equal(ops[1].colour, page.COLOURS.wall);
});
