// [vO9] [oWO]: the Marks page (cockpit/extensions/hk-marks.js): a tap on the house's floor map marks the chosen toy at
// that place in the house's frame (the Area page's own map transform), with its floor kind; nothing is marked before a
// toy is picked; Save sends one confirmed command with tonight's name and the save it replaces; a night that has run
// is closed; nothing on it moves. React is the cockpit's own (HK_REACT_DIR). Run: node --test 'tests/js/*.test.mjs'
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const FILE = join(HK, "cockpit", "extensions", "hk-marks.js");
const CODE = readFileSync(FILE, "utf8").replace(/\/\/.*$/gm, "");
const page = await import(pathToFileURL(FILE).href);             // it imports the Area page's map transform beside it
const area = await import(pathToFileURL(join(HK, "cockpit", "extensions", "hk-area.js")).href);
const req = createRequire(join(process.env.HK_REACT_DIR || join(process.env.HOME, "houndkeeper/dimos-ext/web/node_modules"), "x.js"));
const React = req("react"), { renderToString } = req("react-dom/server");

const MAP = { png: "iVBORw0KGgo=", x0: -2, y0: -1, res: 0.05, w: 200, h: 100 };
const LIST = [{ name: "1 x 1 brick", brick: true, long_m: 0.008 }, { name: "2 x 2 brick", brick: false, long_m: 0.016 },
  { name: "ball", brick: false, long_m: 0.08 }];
const FLOORS = [{ key: "hard", name: "Hard floor" }, { key: "carpet", name: "Carpet" }, { key: "rug", name: "Rug" }];
const ST = { v: 1, night: "20261015", ran: false, marks: null, map: MAP, frame: "house", toys: LIST, floors: FLOORS, message: "" };

function render(st) {
  let Panel = null;
  page.register({
    React, PanelFrame: ({ children }) => React.createElement("div", null, children),
    registerPanel: (kind, C) => { assert.equal(kind, "hk-marks"); Panel = C; },
    useStoreChannel: (store, ch) => ({ slot: store.get(ch) }),
  });
  const store = { get: (ch) => (ch === "marks_state" && st ? { value: st, version: 1 } : null), subscribe: () => () => {} };
  const spec = { id: "p8", kind: "hk-marks", title: "Marks", channels: ["marks_state", "marks_cmd"], params: {} };
  return renderToString(React.createElement(Panel, { spec, store }));
}

test("a tap marks the picked toy, on its floor, where the Area page's map puts it", () => {
  const t = area.fit(MAP, 390), p = area.toHouse(MAP, t, 195, 50);
  let d = page.draftOf(null);
  assert.equal(page.addToy(d, p), d);                               // no toy picked: a tap marks nothing
  d = page.addToy({ ...d, name: "ball", floor: "rug" }, p);
  d = page.addToy({ ...d, name: "1 x 1 brick", floor: "carpet" }, [0, 0]);
  assert.deepEqual(d.toys, [{ name: "ball", x: p[0], y: p[1], floor: "rug" }, { name: "1 x 1 brick", x: 0, y: 0, floor: "carpet" }]);
  assert.ok(d.dirty);
  const ops = page.sceneOps(MAP, t, d.toys);
  assert.deepEqual(ops[0].at.map(Math.round), [195, 50]);          // drawn where it was tapped
  assert.deepEqual(ops.map((o) => [o.label, o.colour]), [["1", page.FLOOR_COLOURS.rug], ["2", page.FLOOR_COLOURS.carpet]]);
  d = page.removeToy(d, 0);
  assert.deepEqual(d.toys.map((x) => x.name), ["1 x 1 brick"]);
});

test("the count says the toys, the bricks and the floors", () => {
  const toys = [{ name: "1 x 1 brick", floor: "hard" }, { name: "2 x 2 brick", floor: "rug" }, { name: "ball", floor: "rug" }];
  assert.deepEqual(page.counts(toys, LIST), { n: 3, brick1: 1, bricks: 2, floors: { hard: 1, carpet: 0, rug: 2 } });
  assert.equal(page.summary(toys, LIST), "3 toys, 2 bricks (1 of them 1 x 1); hard floor 1, carpet 0, rug 2");
});

test("save sends tonight's name, the toys, the save it replaces, and the confirmation", () => {
  const st = { ...ST, marks: { saved: "2026-10-15T20:01:00", toys: [{ name: "ball", x: 1, y: 2, floor: "hard", id: "t1", brick: false, half_long_m: 0.04 }] } };
  const d = page.addToy({ ...page.draftOf(st.marks), name: "ball", floor: "carpet" }, [3, 1]);
  assert.deepEqual(page.saveCommand(st, d), { op: "save", confirmed: true, night: "20261015", seen: "2026-10-15T20:01:00",
    toys: [{ name: "ball", x: 1, y: 2, floor: "hard" }, { name: "ball", x: 3, y: 1, floor: "carpet" }] });
  assert.match(CODE, /confirm\(/);
  assert.equal((CODE.match(/session\.publish\(/g) || []).length, 1);
});

test("the page offers the list and the floors, and closes a night that has run", () => {
  const open = render(ST);
  assert.ok(open.includes("Tonight&#x27;s toys · Thu 15 Oct") && open.includes("1 x 1 brick") && open.includes("Rug") && open.includes("Save the marks"));
  const ran = render({ ...ST, ran: true, marks: { saved: "2026-10-15T20:01:00", toys: [{ name: "ball", x: 1, y: 2, floor: "hard" }] } });
  assert.ok(ran.includes("marks are closed") && ran.includes("1 toy") && !ran.includes("Save the marks") && !ran.includes(">Remove<"));
  assert.ok(render(null).includes("waiting"));
  assert.ok(render({ ...ST, map: null }).includes("No house map yet"));
});

test("nothing on the page moves", () => {
  assert.doesNotMatch(CODE, /animation|transition|@keyframes|setInterval|requestAnimationFrame|blink/i);
});
