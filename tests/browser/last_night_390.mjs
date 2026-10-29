// [RzM] [eTc]: the Last night page in a headless Chromium at a phone's width (390 x 844, the phone held upright), from
// the cockpit a relay serves with only the project service registered (no robot, no sim). Checks the page has the
// nights and the findings, that nothing is wider than the screen (no sideways scroll, no zooming), that no text is
// smaller than 13 px, and that nothing on it animates; opens a finding's whole text; saves screenshots down the page.
//
//   node tests/browser/last_night_390.mjs <cockpit url> <screenshot dir>
//
// Playwright from real/tests/node_modules (npm ci there). Prints one JSON line: {"ok": true, "shots": [...]} or
// {"ok": false, "error": ...}. Driven by tests/test_project.py's browser smoke.
import { createRequire } from "node:module";
import { mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const { chromium } = createRequire(join(HK, "real", "tests", "node_modules", "x.js"))("playwright");
const [url, outDir] = process.argv.slice(2);
mkdirSync(outDir, { recursive: true });

const browser = await chromium.launch({ args: ["--enable-features=WebTransport"] });
const res = { ok: false, shots: [] };
try {
  const page = await browser.newPage({ viewport: { width: 390, height: 844 }, deviceScaleFactor: 2 });
  await page.goto(url, { waitUntil: "networkidle" });
  await page.getByText("Findings, newest first").waitFor({ timeout: 30000 });

  // nothing wider than the screen; no text under 13 px; nothing animated or transitioning
  const audit = () => page.evaluate(() => {
    const out = { wide: [], tiny: [], moving: [] };
    const vw = document.documentElement.clientWidth;
    if (document.documentElement.scrollWidth > vw) out.wide.push(`page ${document.documentElement.scrollWidth}`);
    for (const el of document.querySelectorAll("body *")) {
      const cs = getComputedStyle(el), r = el.getBoundingClientRect();
      if (r.width === 0 || cs.display === "none" || cs.visibility === "hidden") continue;
      // a table or code block may be wider when it scrolls inside its own box
      const inScroller = el.closest("pre") || (el.closest("table") && el.closest("table").parentElement.style.overflowX === "auto");
      if (!inScroller && r.right > vw + 1 && el.closest('[class*="panel"], main')) out.wide.push(`${el.tagName} ${Math.round(r.right)}: ${(el.textContent || "").slice(0, 40)}`);
      const hasText = [...el.childNodes].some((n) => n.nodeType === 3 && n.textContent.trim());
      if (hasText && el.closest("[data-hk=last-night]") && parseFloat(cs.fontSize) < 13) out.tiny.push(`${cs.fontSize} ${el.textContent.slice(0, 40)}`);
      if (el.closest("[data-hk=last-night]") && ((cs.animationName && cs.animationName !== "none") || (parseFloat(cs.transitionDuration) > 0 && cs.transitionProperty !== "none")))
        out.moving.push(`${el.tagName}.${el.className}: ${cs.animationName} ${cs.transitionDuration}`);
    }
    return out;
  });

  const shot = async (name) => {
    const p = join(outDir, name); await page.screenshot({ path: p }); res.shots.push(p);
  };
  // the panel scrolls inside itself: step down it a screen at a time
  const step = async (prefix) => {
    const box = await page.evaluate(() => {
      const els = [...document.querySelectorAll("main *")].filter((e) => {
        const s = getComputedStyle(e); return (s.overflowY === "auto" || s.overflowY === "scroll") && e.scrollHeight > e.clientHeight;
      });
      return els.length ? els.map((e) => e.scrollHeight).reduce((a, b) => Math.max(a, b)) : 0;
    });
    for (let i = 0, y = 0; i < 12; i++, y += 700) {
      await page.evaluate((y) => {
        for (const e of document.querySelectorAll("main *")) {
          const s = getComputedStyle(e);
          if ((s.overflowY === "auto" || s.overflowY === "scroll") && e.scrollHeight > e.clientHeight) e.scrollTop = y;
        }
      }, y);
      await shot(`${prefix}-${String(i + 1).padStart(2, "0")}.png`);
      if (y + 700 >= box) break;
    }
  };

  const a1 = await audit();
  await step("last-night");
  // the newest finding, opened whole
  await page.getByText("Read the whole document").first().click();
  await page.getByText("Read the whole document").first().scrollIntoViewIfNeeded();
  await shot("finding-open.png");
  const table = page.locator("details[open]", { has: page.getByText("Read the whole document") }).locator("table").first();          // a wide table scrolls inside its own box
  if (await table.count()) {
    await table.evaluate((el) => el.scrollIntoView({ block: "center" }));
    await shot("finding-table.png");
  }
  const a2 = await audit();
  res.audit = { wide: [...a1.wide, ...a2.wide], tiny: [...a1.tiny, ...a2.tiny], moving: [...a1.moving, ...a2.moving] };
  res.text = (await page.locator("main").innerText()).slice(0, 4000);
  res.ok = !res.audit.wide.length && !res.audit.tiny.length && !res.audit.moving.length;
  if (!res.ok) res.error = "layout audit failed";
} catch (e) {
  res.error = String(e);
} finally {
  await browser.close();
}
console.log(JSON.stringify(res));
process.exit(res.ok ? 0 : 1);
