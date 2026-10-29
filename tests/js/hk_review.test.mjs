// [71R] [wTj] [Vd3]: the Review page (cockpit/extensions/hk-review.js): each flag near no mark with its crop and its
// place on the house's floor map, the ones left to judge first; a tap sends one judgement naming every candidate in the
// flag (or takes it back); the night says whether it is final and whether it is a tuning night; a flag whose picture
// was withheld says why; nothing on it moves. React is the cockpit's own (HK_REACT_DIR). Run: node --test 'tests/js/*.test.mjs'
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const FILE = join(HK, "cockpit", "extensions", "hk-review.js");
const CODE = readFileSync(FILE, "utf8").replace(/\/\/.*$/gm, "");
const page = await import(pathToFileURL(FILE).href);
const req = createRequire(join(process.env.HK_REACT_DIR || join(process.env.HOME, "houndkeeper/dimos-ext/web/node_modules"), "x.js"));
const React = req("react"), { renderToString } = req("react-dom/server");
const FIX = JSON.parse(readFileSync(join(HK, "tests", "fixtures", "review_message.json"), "utf8"));

function render(st) {
  let Panel = null;
  page.register({
    React, PanelFrame: ({ children }) => React.createElement("div", null, children),
    registerPanel: (kind, C) => { assert.equal(kind, "hk-review"); Panel = C; },
    useStoreChannel: (store, ch) => ({ slot: store.get(ch) }),
  });
  const store = { get: (ch) => (ch === "review_state" && st ? { value: st, version: 1 } : null), subscribe: () => () => {} };
  const spec = { id: "p9", kind: "hk-review", title: "Review", channels: ["review_state", "review_cmd"], params: {} };
  return renderToString(React.createElement(Panel, { spec, store }));
}

test("the flags left to judge come first", () => {
  const fl = [{ k: 1, judged: "false_alarm" }, { k: 2, judged: null }, { k: 3, judged: "real_change" }, { k: 4, judged: null }];
  assert.deepEqual(page.flagOrder(fl).map((f) => f.k), [2, 4, 1, 3]);
});

test("a judgement names every candidate in the flag; null takes it back", () => {
  const f = { k: 2, ids: ["c3", "c7"], x: 1, y: 2, judged: null };
  assert.deepEqual(page.judgeCommand("20261015", f, "false_alarm"), { op: "judge", night: "20261015", ids: ["c3", "c7"], verdict: "false_alarm" });
  assert.equal(page.judgeCommand("20261015", f, null).verdict, null);
  assert.deepEqual(page.VERDICTS.map((v) => v.key), ["false_alarm", "real_change"]);
});

test("the headline: found, the flags judged and left, and final only when none is left", () => {
  const n = { laid_out: true, marked: true, found: [7, 8], bricks: [1, 2], false_alarms: 1, real_changes: 0, final: false };
  const hd = page.headline(n, [{ judged: "false_alarm" }, { judged: null }]);
  assert.equal(hd.found, "Found 7 of 8 laid-out toys (1 x 1 bricks 1 of 2)");
  assert.equal(hd.flags, "2 flags near no toy: 1 false alarm, 0 real changes, 1 left to judge");
  assert.equal(hd.state, "Not final: 1 left to judge");
  assert.equal(page.headline({ ...n, final: true }, [{ judged: "false_alarm" }]).state, "Final: every flag judged");
  assert.match(page.headline({ ...n, laid_out: false, marked: false }, []).found, /only its false alarms count/);
});

test("a flag's place is a window of the house map round it", () => {
  const map = { x0: -2, y0: -1, res: 0.05, w: 200, h: 100 };
  assert.deepEqual(page.mapPx(map, -2, -1), [0, 100]);
  const b = page.placeBox(map, 3, 1.5);
  assert.deepEqual([b.x + b.w / 2, b.y + b.h / 2, b.w], [100, 50, 60]);
});

test("the page from a recorded message: the tuning note, each flag with its crop or why not, both verdicts", () => {
  const html = render(FIX);
  assert.ok(html.includes("The night of Tue 13 Oct") && html.includes("A tuning night"));
  assert.ok(html.includes("data:image/jpeg;base64,") && html.includes("No picture: the people screen did not clear it"));
  assert.equal((html.match(/data-hk="review-flag"/g) || []).length, FIX.flags.length);
  assert.ok(html.includes("False alarm") && html.includes("Real change") && html.includes("Take back"));
  assert.ok(html.includes("Not found: 1 x 1 brick (hard), 1 x 1 brick (carpet)."));
  assert.ok(render({ ...FIX, night: null, flags: [], nights: [] }).includes("No real night yet"));
  assert.ok(render(null).includes("waiting"));
  assert.equal((CODE.match(/session\.publish\(/g) || []).length, 1);
});

test("nothing on the page moves", () => {
  assert.doesNotMatch(CODE, /animation|transition|@keyframes|setInterval|requestAnimationFrame|blink/i);
});
