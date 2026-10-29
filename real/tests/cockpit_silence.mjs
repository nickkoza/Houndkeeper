// [Mj2] Drive the robot from the Cockpit page in a headless Chromium, then make the page fall silent one way.
//
//   node cockpit_silence.mjs CASE <url>      (url optional, default http://127.0.0.1:7780/)
//
// Arms the teleop panel, holds D (turn on the spot: no wall to hit) for DRIVE_MS, then:
//   killed  SIGKILL the browser process: no key-up, no goodbye (browser crash, battery pull)
//   hidden  the page reports hidden and fires visibilitychange (phone locked, app switched)
//   frozen  the page's JS halts (CDP Debugger.pause) while its connection stays open (phone asleep, page
//           hung, Wi-Fi gone quiet): no goodbye of any kind, only the bridge's deadman can stop the robot.
//           (CDP Page.setWebLifecycleState frozen is a no-op on a visible headless page: timers keep firing.)
//   closed  the tab is closed (page unload, connection torn down)
// Prints one JSON line {"case", "armed", "silence", "end"} with Unix times in seconds; on failure {"case", "error"}.
// Leaves no browser behind: every case but "killed" closes it after HOLD_MS; "killed" kills it.
import { chromium } from "playwright";

const DRIVE_MS = 3000;
const HOLD_MS = 3000;
const [VIEW_W, VIEW_H] = (process.env.VIEWPORT ?? "412x915").split("x").map(Number);
const CASES = ["killed", "hidden", "frozen", "closed"];
const kase = process.argv[2];
const url = process.argv[3] ?? "http://127.0.0.1:7780/";
if (!CASES.includes(kase)) {
  console.error(`usage: node cockpit_silence.mjs {${CASES.join("|")}} [url]`);
  process.exit(2);
}
const now = () => Date.now() / 1000;

const srv = await chromium.launchServer({ args: ["--enable-features=WebTransport"] });
const browserPid = srv.process().pid;
const ev = { case: kase, browser_pid: browserPid };
let browser;
try {
  browser = await chromium.connect(srv.wsEndpoint());
  // a phone-sized page, as [Mj2] is about phones; it also costs the shared CPU less than a desktop layout
  const page = await browser.newPage({ viewport: { width: VIEW_W, height: VIEW_H } });
  await page.goto(url, { waitUntil: "networkidle" });
  const pad = page.locator('[role="application"][data-testid^="teleop-"]').first();
  await pad.waitFor({ state: "visible", timeout: 30000 });
  // the panel shows "connection lost" until the transport is up, and "teleop held" while a dead page from an
  // earlier case still holds the lease (the relay frees it ~25 s on): click until it arms
  const t0 = now();
  const deadline = Date.now() + 60000;
  while ((await pad.getAttribute("data-state")) !== "armed") {
    if (Date.now() > deadline) throw new Error(`teleop never armed (state ${await pad.getAttribute("data-state")})`);
    await pad.click();
    await page.waitForTimeout(500);
  }
  ev.armed = now();
  ev.arm_wait_s = Math.round((ev.armed - t0) * 10) / 10;
  await page.keyboard.down("d");
  await page.waitForTimeout(DRIVE_MS);
  if ((await pad.getAttribute("data-state")) !== "armed") throw new Error("disarmed while driving");

  ev.silence = now();
  if (kase === "killed") {
    process.kill(browserPid, "SIGKILL");
    console.log(JSON.stringify({ ...ev, end: now() }));
    process.exit(0);
  } else if (kase === "hidden") {
    await page.evaluate(() => {
      Object.defineProperty(document, "visibilityState", { configurable: true, get: () => "hidden" });
      Object.defineProperty(document, "hidden", { configurable: true, get: () => true });
      document.dispatchEvent(new Event("visibilitychange"));
    });
  } else if (kase === "frozen") {
    const cdp = await page.context().newCDPSession(page);
    await cdp.send("Debugger.enable");
    ev.silence = now();
    await cdp.send("Debugger.pause");
  } else if (kase === "closed") {
    await page.close();
  }
  await new Promise((r) => setTimeout(r, HOLD_MS));
  ev.end = now();
  console.log(JSON.stringify(ev));
} catch (e) {
  console.log(JSON.stringify({ case: kase, error: String(e?.message ?? e).split("\n")[0] }));
  process.exitCode = 1;
} finally {
  try {
    await browser?.close();
  } catch { /* already gone */ }
  try {
    await srv.close();
  } catch { /* already gone */ }
  try {
    process.kill(browserPid, "SIGKILL");
  } catch { /* already gone */ }
}
