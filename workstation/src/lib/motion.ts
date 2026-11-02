import { invoke } from "@tauri-apps/api/core";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader.js";
import type { Vector3Tuple } from "../types";

export interface MotionModel {
  root: string;
  modelKey: string;
  name: string;
  title: string;
  urdfPath: string;
  motionRoot: string;
  python: string;
  initialJoint: number[];
  velocity: number;
  acceleration: number;
  links: { name: string; visuals: {
    xyz: Vector3Tuple; rpy: Vector3Tuple; color: number[] | null;
    geometry: { type: string; asset?: string; scale?: Vector3Tuple;
      size?: Vector3Tuple; radius?: string; length?: string };
  }[] }[];
  joints: { name: string; type: string; parent: string; child: string;
    xyz: Vector3Tuple; rpy: Vector3Tuple; axis: Vector3Tuple; position: number }[];
}

export interface MotionRequest {
  modelKey: string;
  sourceDirectory: string;
  xyz: Vector3Tuple;
  rpy: Vector3Tuple; // radians, fixed-axis roll/pitch/yaw (Rz Ry Rx)
  velocity: number;
  acceleration: number;
}

export interface MotionJob {
  status: "running" | "succeeded" | "failed" | "cancelled";
  directory: string;
  log: string;
  result: { runDirectory: string; collisionChecked: boolean } | null;
}
export interface MotionStatus { planning: MotionJob | null; rviz: MotionJob | null }
export const getMotionModel = () => invoke<MotionModel>("get_motion_model");
export const startMotionJob = (request: MotionRequest) => invoke("start_motion_job", { request });
export const getMotionStatus = () => invoke<MotionStatus>("motion_job_status");
export const stopMotionProcess = (kind: "planning" | "rviz") => invoke("stop_motion_process", { kind });
export const openMotionRviz = () => invoke("open_motion_rviz");

// URDF and the planner both use fixed-axis RPY, NOT editor-local XYZ Euler angles.
export function setPose(object: THREE.Object3D, xyz: Vector3Tuple, rpy: Vector3Tuple) {
  object.position.set(...xyz);
  object.quaternion.setFromEuler(new THREE.Euler(...rpy, "ZYX"));
}

export function disposeObject(root: THREE.Object3D) {
  root.traverse((object) => {
    const renderable = object as THREE.Mesh;
    renderable.geometry?.dispose();
    if (renderable.material) {
      const materials = Array.isArray(renderable.material) ? renderable.material : [renderable.material];
      materials.forEach((material) => material.dispose());
    }
  });
}

export async function buildRobot(model: MotionModel): Promise<THREE.Group> {
  const links = new Map(model.links.map((link) => {
    const group = new THREE.Group();
    group.name = link.name;
    return [link.name, group];
  }));
  const root = links.get(model.root)!;
  const assets = new Map<string, Promise<ArrayBuffer>>();
  // Link hierarchy implements T_parent_child = T_origin * T_joint(q).
  for (const joint of model.joints) {
    const origin = new THREE.Group();
    setPose(origin, joint.xyz, joint.rpy);
    const motion = new THREE.Group();
    const axis = new THREE.Vector3(...joint.axis).normalize();
    if (joint.type === "prismatic") motion.position.copy(axis.multiplyScalar(joint.position));
    if (joint.type === "revolute" || joint.type === "continuous") {
      motion.quaternion.setFromAxisAngle(axis, joint.position);
    }
    links.get(joint.parent)!.add(origin);
    origin.add(motion);
    motion.add(links.get(joint.child)!);
  }
  try {
    // Sequential parsing limits simultaneous large DAE allocations.
    for (const link of model.links) {
      for (const visual of link.visuals) {
        const spec = visual.geometry;
        let object: THREE.Object3D;
        if (spec.type === "mesh") {
          const asset = spec.asset!;
          if (!assets.has(asset)) assets.set(asset, invoke<ArrayBuffer>("read_motion_asset", { asset }));
          const bytes = await assets.get(asset)!;
          if (asset.toLowerCase().endsWith(".dae")) {
            const source = new TextDecoder().decode(bytes);
            object = new ColladaLoader().parse(source, "").scene;
            // ColladaLoader auto-converts Z_UP to Y_UP; URDF uses native mesh
            // coordinates in a Z-up scene. Undo only that wrapper conversion.
            if (/<up_axis>\s*Z_UP\s*<\/up_axis>/.test(source)) object.quaternion.identity();
            // Exported CAD lights/cameras must not change scene illumination.
            const extras: THREE.Object3D[] = [];
            object.traverse((node) => {
              if ((node as THREE.Light).isLight || (node as THREE.Camera).isCamera) extras.push(node);
            });
            extras.forEach((node) => node.removeFromParent());
          } else if (asset.toLowerCase().endsWith(".stl")) {
            object = new THREE.Mesh(new STLLoader().parse(bytes), new THREE.MeshStandardMaterial({ color: 0xaeb6bf }));
          } else {
            throw new Error(`지원하지 않는 메시: ${asset}`);
          }
          object.scale.multiply(new THREE.Vector3(...spec.scale!));
        } else {
          let geometry: THREE.BufferGeometry;
          if (spec.type === "box") geometry = new THREE.BoxGeometry(...spec.size!);
          else if (spec.type === "sphere") geometry = new THREE.SphereGeometry(Number(spec.radius), 20, 12);
          else {
            geometry = new THREE.CylinderGeometry(Number(spec.radius), Number(spec.radius), Number(spec.length), 24);
            geometry.rotateX(Math.PI / 2); // URDF cylinder axis = Z.
          }
          object = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0x858d96, roughness: 0.7 }));
        }
        if (visual.color) {
          const [r, g, b, a] = visual.color;
          object.traverse((node) => {
            if (!(node as THREE.Mesh).isMesh) return;
            const mesh = node as THREE.Mesh;
            (Array.isArray(mesh.material) ? mesh.material : [mesh.material]).forEach((m) => m.dispose());
            mesh.material = new THREE.MeshStandardMaterial({ color: new THREE.Color(r, g, b),
              opacity: a, transparent: a < 1, roughness: 0.7 });
          });
        }
        const origin = new THREE.Group();
        setPose(origin, visual.xyz, visual.rpy);
        origin.add(object);
        links.get(link.name)!.add(origin);
      }
    }
    root.updateMatrixWorld(true);
    return root;
  } catch (error) {
    disposeObject(root);
    throw error;
  }
}
