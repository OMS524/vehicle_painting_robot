// Offline UI smoke test: node backend/test_ui.mjs after npm run build.
// Real local URDF/assets; mocked Tauri commands. Never invokes a robot or NLP.
import assert from "node:assert/strict";
import { createServer } from "node:http";
import { readFile, mkdtemp, writeFile, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { spawn, spawnSync } from "node:child_process";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../..");
const modelResult = spawnSync(`${process.env.HOME}/miniforge3/envs/vehicle_painting_robot_motion_planning/bin/python`,
  ["-B", "-s", `${root}/workstation/backend/motion_backend.py`, "model"],
  { input: JSON.stringify({ motionRoot: `${root}/motion_planning` }), encoding: "utf8", maxBuffer: 8e6 });
assert.equal(modelResult.status, 0, modelResult.stderr);
const model = JSON.parse(modelResult.stdout);
const yaml = await readFile(`${root}/path_planning/config/painting_path.yaml`, "utf8");
const csv = "row_index,point_index,s,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w,paint,frame_id\n"
  + Array.from({ length: 10 }, (_, i) => `0,${i},${i * .04},${i * .04},0,1,0,0,0,1,${i > 1 && i < 8 ? 1 : 0},world\n`).join("");
const allowed = new Set(model.links.flatMap((l) => l.visuals).map((v) => v.geometry.asset).filter(Boolean));
const server = createServer(async (request, response) => {
  try {
    const url = new URL(request.url, "http://localhost");
    let bytes, type = "application/octet-stream";
    if (url.pathname === "/test/model") { bytes = JSON.stringify(model); type = "application/json"; }
    else if (url.pathname === "/test/parameters") bytes = yaml;
    else if (url.pathname === "/test/csv") bytes = csv;
    else if (url.pathname === "/test/asset") {
      const asset = url.searchParams.get("asset"); assert(allowed.has(asset));
      bytes = await readFile(path.join(root, "description", asset));
    } else {
      const relative = url.pathname === "/" ? "index.html" : url.pathname.slice(1);
      const location = path.resolve(root, "workstation/dist", relative);
      assert(location.startsWith(`${root}/workstation/dist/`));
      bytes = await readFile(location);
      type = location.endsWith(".js") ? "application/javascript" : location.endsWith(".css") ? "text/css" : "text/html";
    }
    response.setHeader("Content-Type", type); response.end(bytes);
  } catch (error) { response.statusCode = 500; response.end(String(error)); }
});
await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
const profile = await mkdtemp(path.join(tmpdir(), "workstation-ui-test-"));
const chrome = spawn("/usr/bin/google-chrome", ["--headless=new", "--no-sandbox", "--disable-dev-shm-usage",
  "--disable-background-networking", "--no-first-run", "--no-default-browser-check", "--enable-unsafe-swiftshader",
  "--remote-debugging-port=0", `--user-data-dir=${profile}`, "about:blank"], { stdio: "ignore" });
let ws;
const pause = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
try {
  let port;
  for (let i = 0; i < 100; i++) {
    try { port = Number((await readFile(`${profile}/DevToolsActivePort`, "utf8")).split("\n")[0]); break; }
    catch { await pause(100); }
  }
  assert(port, "Chrome debugger did not start");
  const tabs = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  ws = new WebSocket(tabs.find((tab) => tab.type === "page").webSocketDebuggerUrl);
  await new Promise((resolve) => ws.addEventListener("open", resolve, { once: true }));
  let serial = 0;
  const pending = new Map();
  ws.addEventListener("message", ({ data }) => {
    const result = JSON.parse(data); const callbacks = pending.get(result.id);
    if (callbacks) { pending.delete(result.id); result.error ? callbacks.reject(result.error) : callbacks.resolve(result.result); }
  });
  const cdp = (method, params = {}) => new Promise((resolve, reject) => {
    const id = ++serial; pending.set(id, { resolve, reject }); ws.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async (expression) => {
    const result = await cdp("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
    assert(!result.exceptionDetails, JSON.stringify(result.exceptionDetails)); return result.result.value;
  };
  const until = async (expression) => {
    for (let i = 0; i < 150; i++) { if (await evaluate(expression)) return; await pause(100); }
    throw Error(`Timed out: ${expression}\n${await evaluate("document.body.innerText")}`);
  };
  function mock() {
    window.__calls = []; window.__imported = false; window.confirm = () => true;
    window.__TAURI_INTERNALS__ = { invoke: async (cmd, args) => {
      window.__calls.push(cmd);
      if (cmd === "get_motion_model") return (await fetch("/test/model")).json();
      if (cmd === "read_motion_asset") return (await fetch(`/test/asset?asset=${encodeURIComponent(args.asset)}`)).arrayBuffer();
      if (cmd === "read_default_parameters") return (await fetch("/test/parameters")).text();
      if (cmd === "get_backend_info") return { plannerRoot: "test/path_planning", pythonCommand: "offline mock", logRoot: "test/log" };
      if (cmd === "stage_scan_file") return { fileName: "test.csv", byteCount: 50, sessionDirectory: "test/session" };
      if (cmd === "import_trajectory_file" || cmd === "generate_trajectory") {
        window.__imported = cmd === "import_trajectory_file"; return { rowCount: 1, pointCount: 10, controlPointCount: window.__imported ? 0 : 6 };
      }
      if (cmd === "read_generated_trajectory") return (await fetch("/test/csv")).arrayBuffer();
      if (cmd === "read_control_points") return { points: window.__imported ? [] : Array.from({ length: 6 }, (_, j) => ({
        id: `control-0-${j + 2}`, rowIndex: 0, pointIndex: j + 2, position: [(j + 2) * .04, 0, 1],
        orientation: [0, 0, 0, 1], paint: true, planeOrigin: [0, 0, 1], planeNormal: [0, 1, 0],
      })) };
      if (cmd === "complete_trajectory") return { completedAt: "test", outputDirectory: "test/completed", trajectoryFile: "painting_trajectory.csv", scanFile: "" };
      if (cmd === "save_scan_layout") { window.__savedLayout = structuredClone(args.layout); return "test/scan_layout.json"; }
      if (cmd === "stop_motion_process") return;
      throw Error(`Unexpected command (no hardware allowed): ${cmd}`);
    } };
  }
  await cdp("Page.enable");
  await cdp("Runtime.enable");
  await cdp("Page.addScriptToEvaluateOnNewDocument", { source: `(${mock.toString()})()` });
  await cdp("Emulation.setDeviceMetricsOverride", { width: 1600, height: 1000, deviceScaleFactor: 1, mobile: false });
  await cdp("Page.navigate", { url: `http://127.0.0.1:${server.address().port}` });
  await until("!!document.querySelector('.home-screen') && document.body.innerText.includes('홈에서 공정을 시작하세요')");
  const button = (label) => `[...document.querySelectorAll('button')].find(b => b.textContent.trim() === ${JSON.stringify(label)} && b.getClientRects().length)`;
  const click = async (label) => { await evaluate(`${button(label)}.click()`); await pause(150); };
  const screenshot = async (name) => {
    const result = await cdp("Page.captureScreenshot", { format: "png" });
    await writeFile(`/tmp/workstation-${name}.png`, Buffer.from(result.data, "base64"));
  };
  assert.equal(await evaluate("document.querySelector('.workspace') === null"), true, "First launch must not mount the process workspace");
  assert.equal(await evaluate("document.querySelectorAll('canvas').length"), 0, "Home must not create a WebGL renderer");
  assert.equal(await evaluate("window.__calls.includes('get_motion_model')"), false, "Home must not preload URDF or launch a process");
  await screenshot("home");
  await cdp("Emulation.setDeviceMetricsOverride", { width: 1120, height: 700, deviceScaleFactor: 1, mobile: false });
  await pause(150);
  assert.equal(await evaluate("document.documentElement.scrollWidth > innerWidth"), false);
  assert.equal(await evaluate(`${button("공정 시작")}.getBoundingClientRect().bottom < innerHeight - 28`), true, "Start action visible at minimum window size");
  await screenshot("home-minimum");
  await cdp("Emulation.setDeviceMetricsOverride", { width: 1600, height: 1000, deviceScaleFactor: 1, mobile: false });
  await click("공정 시작");
  await until("document.querySelectorAll('.scan-presets button').length === 6 && !document.querySelector('.error-log-dialog')");
  await until("document.querySelectorAll('.viewer-panel:not([style*=none]) .empty-scene').length === 0");
  assert.equal(await evaluate("document.querySelectorAll('.viewer-panel').length"), 2);
  const checkLayout = async () => {
    assert.equal(await evaluate("document.documentElement.scrollWidth > innerWidth"), false, "No horizontal page overflow");
    assert.equal(await evaluate(`(() => { const panel = [...document.querySelectorAll('.workstation-inspector')].find(e => e.getClientRects().length);
      const rect = panel.getBoundingClientRect(); const footer = panel.querySelector('.sticky-actions').getBoundingClientRect();
      return footer.top >= rect.top && footer.bottom <= rect.bottom + 1; })()`), true, "Stage actions must remain visible");
    assert.equal(await evaluate(`(() => {
      const panels = [...document.querySelectorAll('.workstation-inspector')].filter(e => e.getClientRects().length);
      const views = [...document.querySelectorAll('.viewer-panel')].filter(e => e.getClientRects().length);
      if (panels.length !== 1 || views.length !== 1) return false;
      const panel = panels[0].getBoundingClientRect(), view = views[0].getBoundingClientRect();
      const rail = document.querySelector('.process-navigation').getBoundingClientRect();
      return rail.right <= panel.left + 1 && panel.right <= view.left + 1 && view.right >= innerWidth - 1
        && view.width >= innerWidth * .55 && rail.top === panel.top && panel.top === view.top;
    })()`), true, "Rail, single left inspector and full remaining 3D viewport must be ordered correctly");
    assert.equal(await evaluate("document.querySelectorAll('.process-stepper [aria-current=step]').length"), 1);
    assert.equal(await evaluate("getComputedStyle(document.querySelector('.layer-toggle')).fontSize"), "13px");
  };
  assert.deepEqual(await evaluate("[...document.querySelectorAll('.scan-presets button')].map(b => b.textContent)"),
    ["왼쪽 1", "오른쪽 1", "왼쪽 2", "오른쪽 2", "왼쪽 3", "오른쪽 3"]);
  await checkLayout();
  await screenshot("scan-preview");
  await click("조명 · 배경");
  await click("검정");
  assert.equal(await evaluate(`${button("검정")}.classList.contains('active')`), true);
  await click("회색");
  await click("조명 · 배경");
  await cdp("Emulation.setDeviceMetricsOverride", { width: 1366, height: 768, deviceScaleFactor: 1, mobile: false });
  await pause(200); await checkLayout(); await screenshot("scan-compact");
  await cdp("Emulation.setDeviceMetricsOverride", { width: 1120, height: 700, deviceScaleFactor: 1, mobile: false });
  await pause(200); await checkLayout(); await screenshot("scan-minimum");
  await evaluate("document.querySelector('.scan-layout-panel').closest('.process-scroll').scrollTop = 10000");
  await pause(100);
  assert.equal(await evaluate(`(() => {
    const section = document.querySelector('.scan-layout-panel .scene-settings');
    const last = section.querySelectorAll('.foldout-heading')[2].getBoundingClientRect();
    const footer = document.querySelector('.scan-layout-panel .sticky-actions').getBoundingClientRect();
    return last.bottom <= footer.top + 1 && last.top > 0;
  })()`), true, "Small windows must be able to scroll to settings below the coordinate fields");
  await evaluate("document.querySelector('.scan-layout-panel').closest('.process-scroll').scrollTop = 0");
  await cdp("Emulation.setDeviceMetricsOverride", { width: 1600, height: 1000, deviceScaleFactor: 1, mobile: false });
  await evaluate("window.__originalPresets = JSON.parse(localStorage.getItem('painting-workstation-scan-layout-v1')).placements");
  await click("사용자 설정");
  assert.equal(await evaluate("document.querySelectorAll('.scan-presets button').length"), 6);
  assert.equal(await evaluate(`${button("사용자 설정")}.getAttribute('aria-pressed')`), "true");
  const editScanField = async (section, text) => {
    await evaluate(`(() => { const input = document.querySelectorAll('.scan-layout-panel .transform-section')[${section}].querySelector('input'); input.focus(); input.select(); })()`);
    await cdp("Input.insertText", { text });
    await evaluate("document.activeElement.blur()"); await pause(100);
  };
  await editScanField(0, "425");
  await editScanField(1, "25");
  await editScanField(2, "1");
  await click("왼쪽 1");
  await click("사용자 설정");
  await click("사용자 설정");
  const manual = await evaluate("JSON.parse(localStorage.getItem('painting-workstation-scan-layout-v1'))");
  assert.equal(manual.placements.length, 7);
  assert.equal(manual.selectedId, "user-defined");
  assert.deepEqual(manual.placements.slice(0, 6), await evaluate("window.__originalPresets"));
  const custom = manual.placements.find(p => p.id === "user-defined");
  assert.equal(custom.xyz[0], .425);
  assert(Math.abs(custom.rpy[0] - 25 * Math.PI / 180) < 1e-10);
  assert(Math.abs(custom.direction[0] - Math.SQRT1_2) < 1e-10);
  assert(Math.abs(Math.hypot(...custom.direction) - 1) < 1e-10);
  await screenshot("scan-user-settings");
  await click("+ 배치 추가");
  assert.equal(await evaluate("document.querySelectorAll('.scan-presets button').length"), 7);
  await click("배치 설정 파일 저장");
  await until("document.querySelector('.scan-layout-panel .completion-card')?.textContent.includes('test/scan_layout.json')");
  assert.deepEqual(await evaluate("window.__savedLayout.placements.find(p => p.id === 'user-defined')"), custom);
  await evaluate("window.__scanCanvas = [...document.querySelectorAll('canvas')].find(c => c.getClientRects().length); window.__callCount = window.__calls.length");
  await click("홈");
  assert.equal(await evaluate("document.querySelector('.workspace').getClientRects().length"), 0);
  assert.equal(await evaluate("document.querySelector('.home-current-stage strong').textContent"), "스캔");
  await click("작업 이어서");
  assert.equal(await evaluate("document.querySelectorAll('.scan-presets button').length"), 7);
  assert.equal(await evaluate("window.__scanCanvas.isConnected && window.__scanCanvas.getClientRects().length > 0"), true, "Home round trip must retain the same renderer and camera");
  assert.equal(await evaluate("window.__calls.length === window.__callCount"), true, "Resume must not restart model/backend work");
  await click("다음: 도장 경로 생성");
  await evaluate(`(async () => { const transfer = new DataTransfer(); transfer.items.add(new File([await (await fetch('/test/csv')).text()], 'saved.csv'));
    const input = document.querySelector('input[type=file][accept=".csv"]'); input.files = transfer.files; input.dispatchEvent(new Event('change', {bubbles:true})); })()`);
  await until("document.body.innerText.includes('경로 불러오기 완료')");
  await click("다음: 도장 경로 수정");
  assert.equal(await evaluate(`Boolean(${button("경로 재생성")})`), false);
  assert.equal(await evaluate(`${button("완료 및 저장")}.disabled`), false);
  await click("완료 및 저장");
  await until(`${button("다음: 모션 플래닝")}.disabled === false`);
  assert.equal(await evaluate("window.__calls.includes('regenerate_trajectory')"), false);
  await click("다음: 모션 플래닝");
  await until("document.querySelector('.motion-panel') !== null");
  await until("document.querySelectorAll('.viewer-panel:not([style*=none]) .empty-scene').length === 0");
  await checkLayout(); await screenshot("motion-preview");
  assert.equal(await evaluate("window.__calls.includes('start_motion_job')"), false);
  // Fresh generated path: enter editing and save without any regeneration.
  await evaluate("document.querySelectorAll('.process-stepper button')[0].click()"); await pause(150);
  await click("스캔 파일");
  await evaluate(`(() => { const transfer = new DataTransfer(); transfer.items.add(new File(['x,y,z\\n0,0,1\\n0.36,0,1\\n0.18,0.001,1\\n'], 'test.csv'));
    const input = document.querySelector('input[type=file][accept=".ply,.csv"]'); input.files = transfer.files; input.dispatchEvent(new Event('change',{bubbles:true})); })()`);
  await until("document.body.innerText.includes('스캔 데이터를 불러왔습니다')");
  await click("다음: 도장 경로 생성");
  assert.equal(await evaluate("document.querySelector('.advanced-parameters').open"), false);
  assert.equal(await evaluate("document.querySelector('.parameter-editor select').value"), "surface_adaptive", "Translated labels must retain YAML values");
  assert.equal(await evaluate("[...document.querySelectorAll('.editable-parameter')].some(e => e.textContent.includes('법선 이웃 개수'))"), true, "Advanced parameters are retained");
  await checkLayout(); await screenshot("generation-settings");
  await cdp("Emulation.setDeviceMetricsOverride", { width: 1366, height: 768, deviceScaleFactor: 1, mobile: false });
  await pause(200); await checkLayout(); await screenshot("generation-compact");
  await cdp("Emulation.setDeviceMetricsOverride", { width: 1600, height: 1000, deviceScaleFactor: 1, mobile: false });
  await click("도장 경로 생성");
  await until("document.body.innerText.includes('경로 생성 완료')");
  await click("다음: 도장 경로 수정");
  assert.equal(await evaluate(`${button("완료 및 저장")}.disabled`), false);
  assert.equal(await evaluate(`${button("경로 재생성")}.disabled`), true);
  const center = await evaluate(`(() => { const box = [...document.querySelectorAll('canvas')].find(c => c.getClientRects().length).getBoundingClientRect(); return {x:box.x+box.width/2, y:box.y+box.height/2}; })()`);
  await cdp("Input.dispatchMouseEvent", { type: "mousePressed", button: "left", clickCount: 1, ...center });
  await cdp("Input.dispatchMouseEvent", { type: "mouseReleased", button: "left", clickCount: 1, ...center });
  await until("!!document.querySelector('.selected-point-heading')");
  await checkLayout(); await screenshot("edit-panel");
  await evaluate("document.querySelector('.edit-panel .transform-grid input').focus(); document.querySelector('.edit-panel .transform-grid input').blur()");
  await pause(100);
  assert.equal(await evaluate(`${button("완료 및 저장")}.disabled`), false, "Unchanged point focus must not mark dirty");
  await click("홈");
  assert.equal(await evaluate("document.querySelector('.home-current-stage strong').textContent"), "도장 경로 수정");
  assert.equal(await evaluate("document.querySelector('.home-session-data').textContent.includes('10개 포인트')"), true);
  await screenshot("home-resume");
  await click("작업 이어서");
  assert.equal(await evaluate(`${button("완료 및 저장")}.disabled`), false);
  await click("완료 및 저장");
  assert.equal(await evaluate("window.__calls.includes('regenerate_trajectory')"), false);
  assert.equal(await evaluate("!!document.querySelector('.error-log-dialog')"), false);
  // Commit a numeric draft by physically clicking Home (blur before navigation).
  // This catches selection-clearing handlers that would otherwise lose the draft.
  await evaluate("document.querySelector('.edit-panel .transform-grid input').focus(); document.querySelector('.edit-panel .transform-grid input').select()");
  await cdp("Input.insertText", { text: "225" });
  const pointerClick = async (label) => {
    const target = await evaluate(`(() => { const r = ${button(label)}.getBoundingClientRect(); return {x:r.x+r.width/2,y:r.y+r.height/2}; })()`);
    await cdp("Input.dispatchMouseEvent", { type: "mousePressed", button: "left", clickCount: 1, ...target });
    await cdp("Input.dispatchMouseEvent", { type: "mouseReleased", button: "left", clickCount: 1, ...target });
    await pause(150);
  };
  await pointerClick("홈");
  assert.equal(await evaluate("document.querySelector('.home-session-state').textContent.includes('재생성 필요')"), true);
  await pointerClick("작업 이어서");
  assert.equal(await evaluate("document.querySelector('.edit-panel .transform-grid input').value"), "225", "Unsaved point coordinates must survive Home and resume");
  assert.equal(await evaluate(`${button("완료 및 저장")}.disabled`), true, "Dirty state must also survive Home");
  // The same React UI is also used by the read-only, embedded HTML exporter.
  const surface = "x,y,z\n0,0,1\n0.36,0,1\n0.18,0.001,1\n";
  const embedded = { info: { name: "Offline review", surfaceFile: "test.csv", trajectoryFile: "path.csv",
    surfaceBytes: surface.length, trajectoryBytes: csv.length },
    surfaceBase64: Buffer.from(surface).toString("base64"), trajectoryBase64: Buffer.from(csv).toString("base64"),
    parametersBase64: Buffer.from(yaml).toString("base64"), sourceLabel: "test" };
  await cdp("Page.addScriptToEvaluateOnNewDocument", { source:
    `delete window.__TAURI_INTERNALS__; window.__PAINTING_TRAJECTORY_STANDALONE__ = ${JSON.stringify(embedded)};` });
  await cdp("Page.reload");
  await until("document.body.innerText.includes('단일 HTML 읽기 전용 검토 모드')");
  assert.equal(await evaluate("document.querySelector('.home-screen') === null"), true, "Standalone HTML should open its review directly");
  assert.equal(await evaluate("document.querySelectorAll('.process-stepper button').length"), 1);
  assert.equal(await evaluate("document.querySelector('.process-stepper [aria-current=step] strong').textContent"), "도장 경로 수정");
  assert.equal(await evaluate("window.__calls.length"), 0, "Standalone review must not call any backend");
  assert.equal(await evaluate("!!document.querySelector('.error-log-dialog')"), false);
  assert.equal(await evaluate("document.querySelectorAll('.scene-settings .layer-toggle').length"), 6);
  console.log("PASS: left workbench at three resolutions, paired presets, lighting, actual URDF preview, custom/save layout, import without scan, unchanged completion, motion navigation, no automatic NLP");
  console.log("PASS: scroll access to lower settings and standalone read-only review without backend");
  console.log("PASS: Home first launch, lazy workspace/URDF, start/resume, scan and path state preservation, no backend restart");
  console.log("PASS: manual scan XYZ/RPY/direction editing, normalization, preset switching without reset, no duplicate manual slots, JSON save");
  console.log("Screenshot: /tmp/workstation-scan-preview.png");
} finally {
  ws?.close(); chrome.kill(); server.close();
  await pause(500); await rm(profile, { recursive: true, force: true, maxRetries: 5, retryDelay: 100 });
}
