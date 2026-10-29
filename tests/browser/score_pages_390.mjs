// [vO9] [71R]: the Marks and Review pages in a headless Chromium at a phone's width (390 x 844), from the cockpit a
// loopback relay serves with only the project service registered (no robot, no sim). On each: nothing wider than the
// screen, no text under 13 px, nothing animated. Marks: a toy picked, its floor picked, its place tapped on the map,
// saved through the confirm dialog, and the service says it is saved. Review: a flag left to judge is judged a false
// alarm, and the page's count of flags left goes down.
//
//   node tests/browser/score_pages_390.mjs <cockpit url> <screenshot dir>
//
// Playwright from real/tests/node_modules (npm ci there). Prints one JSON line: {"ok": true, ...} or {"ok": false,
// "error": ...}. Driven by tests/test_real_night_scoring.py's browser smoke.
import { createRequire } from "node:module";
import { mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const { chromium } = createRequire(join(HK, "real", "tests", "node_modules", "x.js"))("playwright");
const [url, outDir] = process.argv.slice(2);
mkdirSync(outDir, { recursive: true });

const browser = await chromium.launch({ args: ["--enable-features=WebTransport"] });
const res = { ok: false, shots: [], dialogs: [], audit: { wide: [], tiny: [], moving: [] } };
try {
  const page = await browser.newPage({ viewport: { width: 390, height: 844 }, deviceScaleFactor: 2 });
  page.on("dialog", async (d) => { res.dialogs.push(d.message()); await d.accept(); });
  await page.goto(url, { waitUntil: "networkidle" });

  const audit = (hk) => page.evaluate((hk) => {
    const out = { wide: [], tiny: [], moving: [] };
    const vw = document.documentElement.clientWidth;
    if (document.documentElement.scrollWidth > vw) out.wide.push(`page ${document.documentElement.scrollWidth}`);
    for (const el of document.querySelectorAll(`[data-hk=${hk}] *`)) {
      const cs = getComputedStyle(el), r = el.getBoundingClientRect();
      if (r.width === 0 || cs.display === "none" || cs.visibility === "hidden") continue;
      if (r.right > vw + 1 && !el.closest("svg image, svg rect, svg circle")) out.wide.push(`${hk} ${el.tagName} ${Math.round(r.right)}: ${(el.textContent || "").slice(0, 40)}`);
      const hasText = [...el.childNodes].some((n) => n.nodeType === 3 && n.textContent.trim());
      if (hasText && parseFloat(cs.fontSize) < 13) out.tiny.push(`${hk} ${cs.fontSize} ${el.textContent.slice(0, 40)}`);
      if ((cs.animationName && cs.animationName !== "none") || (parseFloat(cs.transitionDuration) > 0 && cs.transitionProperty !== "none"))
        out.moving.push(`${hk} ${el.tagName}: ${cs.animationName} ${cs.transitionDuration}`);
      if (el.tagName === "BUTTON" && r.height < 44) out.tiny.push(`${hk} button ${Math.round(r.height)} px high: ${el.textContent.slice(0, 30)}`);
    }
    return out;
  }, hk);
  const add = (a) => { for (const k of Object.keys(res.audit)) res.audit[k].push(...a[k]); };
  const shot = async (name) => { const p = join(outDir, name); await page.screenshot({ path: p }); res.shots.push(p); };
  const scrollTo = (loc) => loc.evaluate((el) => el.scrollIntoView({ block: "center" }));

  // ---- Marks ----
  await page.getByRole("tab", { name: "Marks" }).click();
  await page.getByText("Tonight's toys").waitFor({ timeout: 60000 });
  await page.locator("[data-hk=marks-map]").waitFor();
  add(await audit("marks"));
  await shot("marks-1-open.png");
  await page.getByRole("button", { name: "toy car", exact: true }).click();
  await page.getByRole("button", { name: "Carpet" }).click();
  const map = page.locator("[data-hk=marks-map]"); await scrollTo(map);
  const box = await map.boundingBox();
  await page.mouse.click(box.x + 0.7 * box.width, box.y + 0.4 * box.height);
  await shot("marks-2-tapped.png");
  const save = page.getByRole("button", { name: "Save the marks" }); await scrollTo(save);
  await save.click();
  await page.getByText(/^saved .* toys? for the night of/).waitFor({ timeout: 30000 });
  await shot("marks-3-saved.png");
  res.marks_text = (await page.locator("[data-hk=marks-list]").innerText()).slice(0, 1500);
  add(await audit("marks"));

  // ---- Review ----
  await page.getByRole("tab", { name: "Review" }).click();
  await page.locator("[data-hk=review-head]").waitFor({ timeout: 60000 });
  add(await audit("review"));
  await shot("review-1-open.png");
  const before = await page.locator("[data-hk=review-head]").innerText();
  const flag = page.locator("[data-hk=review-flag]").first(); await scrollTo(flag);
  await shot("review-2-flag.png");
  await flag.getByRole("button", { name: "False alarm" }).click();
  await page.locator("[data-hk=review-head]").filter({ hasNotText: before.match(/\d+ left to judge/)[0] }).waitFor({ timeout: 30000 });
  await page.locator("[data-hk=review-head]").evaluate((el) => el.scrollIntoView({ block: "start" }));
  await shot("review-3-judged.png");
  res.review_before = before; res.review_after = await page.locator("[data-hk=review-head]").innerText();
  add(await audit("review"));
  res.ok = !res.audit.wide.length && !res.audit.tiny.length && !res.audit.moving.length;
  if (!res.ok) res.error = "layout audit failed";
} catch (e) {
  res.error = String(e);
  try { const pg = browser.contexts()[0].pages()[0]; await pg.screenshot({ path: join(outDir, "score-error.png") }); res.text = (await pg.locator("body").innerText()).slice(0, 2000); } catch {}
} finally {
  await browser.close();
}
console.log(JSON.stringify(res));
process.exit(res.ok ? 0 : 1);
