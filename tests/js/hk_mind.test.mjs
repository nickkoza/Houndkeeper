// [Ttd] [D1U] [ls3]: the Mind page (cockpit/extensions/hk-mind.js) from a recorded message
// (tests/fixtures/mind_message.json, made by houndkeeper.mind): what it draws on the map and over the camera, what it
// says, and that it sends nothing. React is the cockpit's own (HK_REACT_DIR: the fork's web/node_modules); the page
// renders server-side here, so its canvases' drawing is tested as the scenes it paints. Run: node --test 'tests/js/*.test.mjs'
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const SRC = readFileSync(join(HK, "cockpit", "extensions", "hk-mind.js"), "utf8");
const CODE = SRC.replace(/\/\/.*$/gm, "");                                  // the code, without its comments
const page = await import("data:text/javascript;base64," + Buffer.from(SRC).toString("base64"));
const MSG = JSON.parse(readFileSync(join(HK, "tests", "fixtures", "mind_message.json"), "utf8"));
const req = createRequire(join(process.env.HK_REACT_DIR || join(process.env.HOME, "houndkeeper/dimos-ext/web/node_modules"), "x.js"));
const React = req("react"), { renderToString } = req("react-dom/server");

const PLACE = { w: 200, h: 200, res: 0.05, origin: [-5, -5, 0] };       // a 10 m grid round the origin
const near = (a, b) => Math.abs(a - b) < 1e-6;

test("the map shows each candidate coloured by status and confidence, both routes and the goal", () => {
  const m = { ...MSG, route: { ...MSG.route, walked: [[0, -1], [0, -0.5], [0, 0.2]] } };
  const path = [[0, 0.2], [0.3, 0.7], [0.5, 1.2]];
  const ops = page.mapScene(m, PLACE, 400, 400, path);
  const t = page.fitView(PLACE, 400, 400);
  for (const c of m.cands) {
    const d = ops.find((o) => o.t === "disc" && o.id === c.id);
    const [x, y] = page.toCanvas(t, c.x, c.y);
    assert.ok(d && near(d.x, x) && near(d.y, y), c.id);
    assert.equal(d.colour, page.COLOURS[c.status]);
    assert.ok(near(d.alpha, c.status === "ruled out" ? 0 : 0.25 + 0.75 * c.conf), c.id);   // surer, more solid
  }
  const lines = ops.filter((o) => o.t === "line").map((o) => o.colour);
  assert.deepEqual(lines, [page.ROUTE.walked, page.ROUTE.plan, page.ROUTE.path, page.ROUTE.goal]);
  const plan = ops.find((o) => o.colour === page.ROUTE.plan);
  assert.equal(plan.pts.length, 1 + m.route.plan.length);                                   // from the robot on
  const goal = ops.find((o) => o.id === "goal"), [gx, gy] = page.toCanvas(t, m.goal.x, m.goal.y);
  assert.ok(near(goal.x, gx) && near(goal.y, gy));
  assert.equal(ops.at(-1).id, "robot");                                                     // drawn last, on top
});

test("without a costmap the map fits round what the message names", () => {
  const p = page.boundsPlace(MSG);
  const ops = page.mapScene(MSG, p, 300, 600);
  for (const o of ops.filter((o) => o.t === "disc")) assert.ok(o.x >= 0 && o.x <= 300 && o.y >= 0 && o.y <= 600, o.id);
});

test("the goal's reason is said in the robot's words", () => {
  assert.equal(page.goalWords(MSG), `Going to a find: ${MSG.goal.why}`);
  assert.equal(page.caption(MSG), `Look closer - Going to a find: ${MSG.goal.why}`);
  assert.equal(page.goalWords({ ...MSG, goal: null }), "No goal");
});

test("the candidates the camera sees are outlined with their confidence", () => {
  const ops = page.videoScene(MSG, 1280, 720);
  const v = MSG.view[0], ring = ops.find((o) => o.t === "ring");
  assert.equal(ring.id, v.id);
  assert.deepEqual(ring.pts, v.ring.map(([u, w]) => [u * 1280, w * 720]));
  const c = MSG.cands.find((x) => x.id === v.id);
  assert.equal(ring.colour, page.COLOURS[c.status]);
  assert.ok(ops.find((o) => o.t === "text").text.startsWith(`${Math.round(c.conf * 100)}%`));
});

test("the costmap's cells become the picture, rows flipped", () => {
  const px = page.gridImage(Uint8Array.from([0, 255, 100, 50]), 2, 2);   // row 0 (min y) is the canvas's bottom
  assert.deepEqual([...px.slice(0, 4)], [0xb8, 0xc0, 0xc8, 255]);        // lethal, top left
  assert.equal(px[(1 * 2 + 1) * 4 + 3], 0);                              // unknown: clear
});

function render(m, props = {}) {
  let Panel = null;
  page.register({
    React, PanelFrame: ({ children }) => React.createElement("div", null, children),
    registerPanel: (kind, C) => { assert.equal(kind, "hk-mind"); Panel = C; },
    registerDecoder: () => assert.fail("registers no decoder"),
    useStoreChannel: (store, ch) => ({ slot: store.get(ch) }),
  });
  const store = { get: (ch) => (ch === "mind" && m ? { value: m, version: 1 } : null), subscribe: () => () => {} };
  const spec = { id: "p5", kind: "hk-mind", title: "Mind", channels: ["mind", "global_costmap", "path", "color_image"],
    params: { costmap: "global_costmap", path: "path", video: "color_image" } };
  return renderToString(React.createElement(Panel, { spec, store, ...props }));
}

test("the page renders the stage, goal, arm and every candidate from the recorded message", () => {
  const html = render(MSG);
  for (const s of ["look closer", "LOOK STANDING", MSG.goal.why, "Arm: looking", MSG.note, "planned", "walked"]) assert.ok(html.includes(s), s);
  for (const c of MSG.cands) assert.ok(html.includes(c.why), c.id);
  assert.ok(render(null).includes("waiting for the robot"));
});

test("the page sends nothing, whatever it is given", () => {
  const session = new Proxy({}, { get: () => () => assert.fail("the page used the session") });
  render(MSG, { session, teleop: session });
  assert.ok(!/\.publish\s*\(/.test(CODE) && !/\bsession\b/.test(CODE) && !/\bteleop\b/.test(CODE));
});

test("nothing on the page moves by itself", () => {
  for (const w of ["animation", "transition", "@keyframes", "setInterval", "requestAnimationFrame", "blink"]) {
    assert.ok(!CODE.includes(w), w);
  }
});
