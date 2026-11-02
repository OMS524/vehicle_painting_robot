import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { buildRobot, disposeObject, setPose } from "../lib/motion";
import type { MotionModel } from "../lib/motion";
import type { LayerVisibility, TrajectoryDataset, Vector3Tuple, VisualizationSettings } from "../types";
import type { ScanLayout } from "../lib/scanLayout";

interface Props {
  model: MotionModel;
  trajectory: TrajectoryDataset;
  xyz: Vector3Tuple;
  rpy: Vector3Tuple;
  layers: LayerVisibility;
  visualization: VisualizationSettings;
  onReady: (ready: boolean) => void;
  onError: (error: string) => void;
  scanLayout?: ScanLayout;
  onSelectPlacement?: (id: string) => void;
}

interface Runtime {
  scene: THREE.Scene; camera: THREE.PerspectiveCamera; controls: OrbitControls;
  robot: THREE.Group; path: THREE.Group; on: THREE.Group; off: THREE.Group;
  directions: THREE.Group; points: THREE.Group; axes: THREE.AxesHelper;
  light: THREE.DirectionalLight; rim: THREE.DirectionalLight;
  scan: THREE.Group;
}

function fit(runtime: Runtime, pathOnly = false) {
  const bounds = new THREE.Box3().setFromObject(runtime.path);
  bounds.expandByObject(runtime.scan);
  if (!pathOnly) bounds.expandByObject(runtime.robot);
  if (bounds.isEmpty()) return;
  const center = bounds.getCenter(new THREE.Vector3());
  const radius = bounds.getSize(new THREE.Vector3()).length() / 2;
  const vertical = THREE.MathUtils.degToRad(runtime.camera.fov) / 2;
  const angle = Math.min(vertical, Math.atan(Math.tan(vertical) * runtime.camera.aspect));
  const distance = Math.max(0.5, radius / Math.sin(angle) * 1.15);
  const direction = runtime.camera.position.clone().sub(runtime.controls.target).normalize();
  if (direction.lengthSq() === 0) direction.set(1, -1, 0.8).normalize();
  runtime.controls.target.copy(center);
  runtime.camera.position.copy(center).addScaledVector(direction, distance);
  runtime.controls.update();
}

export function MotionPreview({ model, trajectory, xyz, rpy, layers, visualization, onReady, onError, scanLayout, onSelectPlacement }: Props) {
  const container = useRef<HTMLDivElement>(null);
  const runtime = useRef<Runtime | null>(null);
  const callbacks = useRef({ onReady, onError, onSelectPlacement });
  callbacks.current = { onReady, onError, onSelectPlacement };
  useEffect(() => {
    if (!container.current) return;
    const element = container.current;
    let renderer: THREE.WebGLRenderer;
    try { renderer = new THREE.WebGLRenderer({ antialias: true }); }
    catch (error) { callbacks.current.onError(String(error)); return; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    element.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x25282c);
    const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 2000);
    camera.up.set(0, 0, 1);
    camera.position.set(7, -9, 6);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    const robot = new THREE.Group(), path = new THREE.Group(), scan = new THREE.Group();
    const on = new THREE.Group(), off = new THREE.Group(), directions = new THREE.Group(), points = new THREE.Group();
    path.add(on, off, directions, points);
    path.add(new THREE.AxesHelper(0.25)); // original CSV origin after placement.
    const axes = new THREE.AxesHelper(1);
    const grid = new THREE.GridHelper(10, 20, 0x65717a, 0x3b444b);
    grid.rotation.x = Math.PI / 2;
    scene.add(robot, path, scan, axes, grid, new THREE.HemisphereLight(0xe6efff, 0x6a7380, 2));
    const light = new THREE.DirectionalLight(0xffffff, 2.5);
    light.position.set(3, -5, 8);
    const rim = new THREE.DirectionalLight(0xadd9ff, 1.4);
    rim.position.set(-3, 4, 5);
    scene.add(light, rim);
    const rt: Runtime = { scene, camera, controls, robot, path, on, off, directions, points, axes, light, rim, scan };
    runtime.current = rt;
    let down: [number, number] | null = null;
    const pointerDown = (event: PointerEvent) => { if (event.button === 0) down = [event.clientX, event.clientY]; };
    const pointerUp = (event: PointerEvent) => {
      const start = down; down = null;
      if (!start || Math.hypot(event.clientX - start[0], event.clientY - start[1]) > 4) return;
      const rect = renderer.domElement.getBoundingClientRect();
      const ray = new THREE.Raycaster();
      ray.setFromCamera(new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1,
        -(event.clientY - rect.top) / rect.height * 2 + 1), camera);
      const hit = ray.intersectObject(scan, true).find((item) => item.object.userData.placementId);
      if (hit) callbacks.current.onSelectPlacement?.(hit.object.userData.placementId as string);
    };
    renderer.domElement.addEventListener("pointerdown", pointerDown);
    renderer.domElement.addEventListener("pointerup", pointerUp);
    const resize = () => {
      const width = element.clientWidth, height = element.clientHeight;
      if (!width || !height) return;
      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(element);
    resize();
    let disposed = false;
    callbacks.current.onReady(false);
    void buildRobot(model).then((object) => {
      if (disposed) { disposeObject(object); return; }
      robot.add(object);
      fit(rt);
      callbacks.current.onReady(true);
    }).catch((error: unknown) => { if (!disposed) callbacks.current.onError(String(error)); });
    renderer.setAnimationLoop(() => {
      if (!element.clientWidth) return;
      controls.update();
      renderer.render(scene, camera);
    });
    return () => {
      disposed = true;
      observer.disconnect();
      renderer.setAnimationLoop(null);
      controls.dispose();
      renderer.domElement.removeEventListener("pointerdown", pointerDown);
      renderer.domElement.removeEventListener("pointerup", pointerUp);
      disposeObject(scene);
      renderer.dispose();
      renderer.domElement.remove();
      runtime.current = null;
    };
  }, [model]);

  useEffect(() => {
    const rt = runtime.current;
    if (!rt) return;
    for (const group of [rt.on, rt.off, rt.directions, rt.points]) {
      disposeObject(group); group.clear();
    }
    const on: THREE.Vector3[] = [], off: THREE.Vector3[] = [], directions: THREE.Vector3[] = [];
    const all = trajectory.rows.flatMap((row) => row.points);
    for (const row of trajectory.rows) {
      row.points.forEach((point, i) => {
        const position = new THREE.Vector3(...point.position);
        const direction = new THREE.Vector3(0, 0, 1).applyQuaternion(new THREE.Quaternion(...point.orientation).normalize());
        directions.push(position, position.clone().addScaledVector(direction, 0.12));
        if (i + 1 < row.points.length) {
          (point.paint ? on : off).push(position, new THREE.Vector3(...row.points[i + 1].position));
        }
      });
    }
    for (const [vertices, group, color] of [[on, rt.on, 0x20e66a], [off, rt.off, 0xffc857], [directions, rt.directions, 0x4cc9f0]] as const) {
      group.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints([...vertices]), new THREE.LineBasicMaterial({ color })));
    }
    const spheres = new THREE.InstancedMesh(new THREE.SphereGeometry(0.007, 10, 8),
      new THREE.MeshBasicMaterial({ color: 0xff4fa3 }), all.length);
    all.forEach((point, index) => spheres.setMatrixAt(index, new THREE.Matrix4().makeTranslation(...point.position)));
    spheres.instanceMatrix.needsUpdate = true;
    rt.points.add(spheres);
  }, [model, trajectory]);

  useEffect(() => {
    const rt = runtime.current;
    if (!rt) return;
    disposeObject(rt.scan); rt.scan.clear();
    if (!scanLayout) return;
    for (const placement of scanLayout.placements) {
      const selected = placement.id === scanLayout.selectedId;
      const object = new THREE.Group();
      setPose(object, placement.xyz, placement.rpy);
      const shape = new THREE.BoxGeometry(1.2, 0.9, 0.035);
      const panel = new THREE.Mesh(shape, new THREE.MeshStandardMaterial({
        color: selected ? 0x47c9f5 : 0x718393, opacity: selected ? 0.7 : 0.18,
        transparent: true, depthWrite: false, side: THREE.DoubleSide,
      }));
      panel.userData.placementId = placement.id;
      object.add(panel, new THREE.LineSegments(new THREE.EdgesGeometry(shape),
        new THREE.LineBasicMaterial({ color: selected ? 0x8ee8ff : 0x647381 })));
      if (selected && layers.axes) object.add(new THREE.AxesHelper(0.35));
      rt.scan.add(object);
      const direction = new THREE.Vector3(...placement.direction).normalize();
      const center = new THREE.Vector3(...placement.xyz);
      const arrow = new THREE.ArrowHelper(direction, center.clone().addScaledVector(direction, -0.8),
        0.8, selected ? 0xffc857 : 0x807650, 0.14, 0.08);
      rt.scan.add(arrow);
      // Camera marker is illustrative, not a commanded or IK-validated pose.
      const camera = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.06),
        new THREE.MeshBasicMaterial({ color: selected ? 0xffc857 : 0x807650 }));
      camera.position.copy(center).addScaledVector(direction, -0.8);
      camera.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), direction);
      rt.scan.add(camera);
    }
  }, [model, scanLayout, layers.axes]);

  useEffect(() => {
    if (runtime.current) setPose(runtime.current.path, xyz, rpy);
    // Placement changes deliberately do not refit/reset the orbit camera.
  }, [model, xyz, rpy]);

  useEffect(() => {
    const rt = runtime.current;
    if (!rt) return;
    rt.robot.visible = layers.surface;
    rt.on.visible = layers.trajectory;
    rt.off.visible = layers.sprayOff;
    rt.directions.visible = layers.sprayDirections;
    rt.points.visible = layers.controlPoints;
    rt.axes.visible = layers.axes;
    rt.path.children.filter((node) => node instanceof THREE.AxesHelper).forEach((node) => { node.visible = layers.axes; });
    rt.scene.background = new THREE.Color(visualization.background === "gray" ? 0x25282c : 0x08090b);
    rt.light.intensity = 2.5 * visualization.lightIntensity;
    rt.rim.visible = visualization.rimLight;
  }, [model, layers, visualization]);

  return <div className="viewer-shell">
    <div className="viewer-canvas" ref={container} />
    <div className="view-controls">
      <button onClick={() => runtime.current && fit(runtime.current)}>셀 전체</button>
      <button onClick={() => runtime.current && fit(runtime.current, true)}>{scanLayout ? "배치 확대" : "경로 확대"}</button>
      <button onClick={() => {
        const rt = runtime.current;
        if (!rt) return;
        const distance = rt.camera.position.distanceTo(rt.controls.target);
        rt.camera.position.copy(rt.controls.target).add(new THREE.Vector3(0, -0.001, distance));
        rt.controls.update();
      }}>TOP</button>
    </div>
    <div className="viewer-hint">world_base · 바닥 격자 500 mm · {scanLayout ? "노란 화살표: 카메라 → 부품 · 배치 미리보기, 로봇 제어 아님" : "로봇은 설정 초기 자세 (실시간 위치 아님)"}</div>
  </div>;
}
