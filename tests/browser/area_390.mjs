// [daF] [HZO]: the Area page in a headless Chromium at a phone's width (390 x 844), from the cockpit a relay serves with
// only the Area page's service registered (houndkeeper.area: no robot, no sim). Checks the warning is on the page, that
// nothing is wider than the screen, no text is smaller than 13 px and nothing animates; then draws an area by tapping
// four corners on the map, saves it through the confirm dialog, and waits for the robot side to say it is saved.
//
//   node tests/browser/area_390.mjs <cockpit url> <screenshot dir>
//
// Playwright from real/tests/node_modules (npm ci there). Prints one JSON line: {"ok": true, ...} or {"ok": false,
// "error": ...}. Driven by tests/test_operating_area.py's browser smoke.
import { createRequire } from "node:module";
import { mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HK = join(dirname(fileURLToPath(import.meta.url)), "..", "..");
const { chromium } = createRequire(join(HK, "real", "tests", "node_modules", "x.js"))("playwright");
const [url, outDir] = process.argv.slice(2);
mkdirSync(outDir, { recursive: true });

const browser = await chromium.launch({ args: ["--enable-features=WebTransport"] });
const res = { ok: false, shots: [], dialogs: [] };
try {
  const page = await browser.newPage({ viewport: { width: 390, height: 844 }, deviceScaleFactor: 2 });
  page.on("dialog", async (d) => { res.dialogs.push(d.message()); await d.accept(); });
  await page.goto(url, { waitUntil: "networkidle" });
  await page.getByText("The house map").waitFor({ timeout: 60000 });

  const audit = () => page.evaluate(() => {
    const out = { wide: [], tiny: [], moving: [] };
    const vw = document.documentElement.clientWidth;
    if (document.documentElement.scrollWidth > vw) out.wide.push(`page ${document.documentElement.scrollWidth}`);
    for (const el of document.querySelectorAll("[data-hk=area] *")) {
      const cs = getComputedStyle(el), r = el.getBoundingClientRect();
      if (r.width === 0 || cs.display === "none" || cs.visibility === "hidden") continue;
      if (r.right > vw + 1) out.wide.push(`${el.tagName} ${Math.round(r.right)}: ${(el.textContent || "").slice(0, 40)}`);
      const hasText = [...el.childNodes].some((n) => n.nodeType === 3 && n.textContent.trim());
      if (hasText && parseFloat(cs.fontSize) < 13) out.tiny.push(`${cs.fontSize} ${el.textContent.slice(0, 40)}`);
      if ((cs.animationName && cs.animationName !== "none") || (parseFloat(cs.transitionDuration) > 0 && cs.transitionProperty !== "none"))
        out.moving.push(`${el.tagName}: ${cs.animationName} ${cs.transitionDuration}`);
    }
    return out;
  });
  const shot = async (name) => { const p = join(outDir, name); await page.screenshot({ path: p }); res.shots.push(p); };

  const a1 = await audit();
  res.warning = await page.locator("[data-hk=area-warning]").innerText();
  await shot("area-1-open.png");
  await page.getByRole("button", { name: "Draw an area" }).click();
  const box = await page.locator("[data-hk=area-map]").boundingBox();
  for (const [fx, fy] of [[0.55, 0.15], [0.9, 0.15], [0.9, 0.85], [0.55, 0.85]]) await page.mouse.click(box.x + fx * box.width, box.y + fy * box.height);
  await page.getByRole("button", { name: "Finish" }).click();
  await shot("area-2-drawn.png");
  await page.getByRole("button", { name: "Save the area" }).click();
  await page.getByText(/^saved /).waitFor({ timeout: 30000 });
  await page.getByText("Stored: 1 area").waitFor({ timeout: 30000 });
  await shot("area-3-saved.png");
  const a2 = await audit();
  res.audit = { wide: [...a1.wide, ...a2.wide], tiny: [...a1.tiny, ...a2.tiny], moving: [...a1.moving, ...a2.moving] };
  res.text = (await page.locator("[data-hk=area]").innerText()).slice(0, 4000);
  res.ok = !res.audit.wide.length && !res.audit.tiny.length && !res.audit.moving.length;
  if (!res.ok) res.error = "layout audit failed";
} catch (e) {
  res.error = String(e);
  try { const pg = browser.contexts()[0].pages()[0]; await pg.screenshot({ path: join(outDir, "area-error.png") }); res.text = (await pg.locator("body").innerText()).slice(0, 2000); } catch {}
} finally {
  await browser.close();
}
console.log(JSON.stringify(res));
process.exit(res.ok ? 0 : 1);
