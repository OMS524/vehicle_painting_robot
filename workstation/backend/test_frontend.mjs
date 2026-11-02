// Run: node backend/test_frontend.mjs (no browser or hardware required).
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import ts from "typescript";

async function load(relative) {
  const text = await readFile(new URL(relative, import.meta.url), "utf8");
  const { outputText } = ts.transpileModule(text, { compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.ESNext } });
  return import(`data:text/javascript;base64,${Buffer.from(outputText).toString("base64")}`);
}
const { sameControlPose } = await load("../src/lib/edits.ts");
const { defaultScanLayout, validScanLayout, loadScanLayout, selectUserPlacement, USER_PLACEMENT_ID } = await load("../src/lib/scanLayout.ts");
const a = { position: [1, 2, 3], orientation: [0, 0, 0, 1] };
assert.equal(sameControlPose(a, structuredClone(a)), true);
assert.equal(sameControlPose(a, { ...a, orientation: [0, 0, 0, -1] }), true);
assert.equal(sameControlPose(a, { ...a, position: [1.001, 2, 3] }), false);
assert.equal(sameControlPose(a, { ...a, orientation: [0, 0, 0.1, 0.995] }), false);
const layout = defaultScanLayout();
assert.equal(validScanLayout(layout), true);
assert.equal(layout.placements.length, 6);
assert.equal(layout.placements.filter((p) => p.xyz[1] > 0).length, 3);
assert.equal(layout.placements.filter((p) => p.xyz[1] < 0).length, 3);
for (const p of layout.placements) {
  assert.equal(Math.hypot(...p.direction), 1);
  assert.equal(Math.sign(p.direction[1]), Math.sign(p.xyz[1]));
}
assert.equal(validScanLayout({ ...layout, selectedId: "missing" }), false);
assert.equal(validScanLayout({ ...layout, placements: [layout.placements[0], layout.placements[0]] }), false);
assert.equal(validScanLayout({ ...layout, placements: [{ ...layout.placements[0], direction: [0, 0, 0] }] }), false);
globalThis.localStorage = { getItem: () => "broken JSON" };
assert.deepEqual(loadScanLayout(), layout);
globalThis.localStorage = { getItem: () => JSON.stringify(layout) };
assert.deepEqual(loadScanLayout(), layout);
console.log("PASS: unchanged pose, quaternion sign, six presets, direction, invalid settings, persistence fallback");
const before = structuredClone(layout);
const manual = selectUserPlacement(layout);
assert.equal(validScanLayout(manual), true);
assert.equal(manual.placements.length, 7);
assert.equal(manual.selectedId, USER_PLACEMENT_ID);
assert.deepEqual(layout, before, "Selecting manual placement must not mutate presets");
assert.deepEqual(manual.placements[6].xyz, layout.placements[0].xyz);
assert.notEqual(manual.placements[6].xyz, layout.placements[0].xyz);
manual.placements[6].xyz[0] = .425;
const again = selectUserPlacement({ ...manual, selectedId: "right-3" });
assert.equal(again.placements.length, 7, "Repeated manual selection must not add duplicates");
assert.equal(again.placements[6].xyz[0], .425);
assert.deepEqual(again.placements.slice(0, 6), before.placements);
globalThis.localStorage = { getItem: () => JSON.stringify(again) };
assert.deepEqual(loadScanLayout(), again, "Manual coordinates are restored with the existing saved layout format");
const full = { ...layout, placements: Array.from({ length: 100 }, (_, i) => ({ ...layout.placements[0], id: `slot-${i}` })), selectedId: "slot-0" };
assert.equal(selectUserPlacement(full), full, "Manual creation respects the existing placement limit");
console.log("PASS: separate manual placement, preset preservation, reuse, persistence, placement limit");
