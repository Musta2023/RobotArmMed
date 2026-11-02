/**
 * @file ObjectManipulator.ts
 * Direct 3D Mouse Object Manipulation & Free 3D Positioning System (Sections 1 & 2).
 *
 * Capabilities:
 * - Raycast hover detection with visual emissive highlighting
 * - Left-click grab and smooth physics-based 3D dragging (spring/velocity PD controller
 *   on the dynamic Rapier rigid body — never teleports through floor or other objects)
 * - Horizontal (XZ), Vertical (XY), and Camera-plane dragging + depth/height adjustment
 * - Free 3D orientation control (Euler X/Y/Z or Quaternion)
 * - Clean mouse release that restores full gravity, mass, friction, and natural settling
 */

import * as THREE from 'three';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import {
  InstrumentFactory,
  ActiveSurgicalInstrument
} from '../objects/InstrumentFactory.js';
import { GraspManager } from '../manipulation/GraspManager.js';

export type DragPlaneMode = 'horizontal' | 'vertical' | 'camera';

export class ObjectManipulator {
  public instrumentFactory: InstrumentFactory;
  public graspManager: GraspManager;

  public raycaster: THREE.Raycaster = new THREE.Raycaster();
  public hoveredObjectId: string | null = null;
  public selectedObjectId: string | null = null;
  public grabbedObjectId: string | null = null;
  public isDragging: boolean = false;
  public dragPlaneMode: DragPlaneMode = 'horizontal';

  // 3D TransformControls Gizmo for direct object manipulation & rotation
  public transformControls: TransformControls | null = null;
  public transformControlsHelper: THREE.Object3D | null = null;
  public isGizmoActive: boolean = false;
  public isDraggingGizmo: boolean = false;
  public gizmoMode: 'translate' | 'rotate' = 'translate';
  public scene: THREE.Scene | null = null;
  public camera: THREE.Camera | null = null;
  public orbitControlsRef: { enabled: boolean } | null = null;

  public targetPosition: THREE.Vector3 = new THREE.Vector3();
  public targetQuaternion: THREE.Quaternion = new THREE.Quaternion();

  private dragPlane: THREE.Plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  private grabOffset: THREE.Vector3 = new THREE.Vector3();
  private smoothedVelocity: THREE.Vector3 = new THREE.Vector3();
  private savedEmissiveColors: Map<string, number> = new Map();

  constructor(instrumentFactory: InstrumentFactory, graspManager: GraspManager) {
    this.instrumentFactory = instrumentFactory;
    this.graspManager = graspManager;
  }

  /**
   * Resolves an intersected Three.js Object3D up its parent chain to an instrument ID.
   */
  private _resolveInstrumentIdFromObject(obj: THREE.Object3D | null): string | null {
    let curr: THREE.Object3D | null = obj;
    while (curr) {
      if (typeof curr.userData?.instrumentId === 'string') {
        return curr.userData.instrumentId;
      }
      curr = curr.parent;
    }
    return null;
  }

  /**
   * Raycasts from normalized device coordinates `[-1..1]` against all dynamic objects.
   */
  public raycastObject(
    ndcX: number,
    ndcY: number,
    camera: THREE.Camera
  ): {
    instrument: ActiveSurgicalInstrument;
    hitPoint: THREE.Vector3;
    distance: number;
  } | null {
    this.raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), camera);
    const meshes: THREE.Object3D[] = [];
    for (const [, inst] of this.instrumentFactory.instruments) {
      inst.mesh.updateMatrixWorld(true);
      meshes.push(inst.mesh);
    }

    const hits = this.raycaster.intersectObjects(meshes, true);
    for (const hit of hits) {
      const id = this._resolveInstrumentIdFromObject(hit.object);
      if (id && this.instrumentFactory.instruments.has(id)) {
        return {
          instrument: this.instrumentFactory.instruments.get(id)!,
          hitPoint: hit.point.clone(),
          distance: hit.distance
        };
      }
    }
    return null;
  }

  /**
   * Updates hover highlight state based on mouse raycast or explicit object ID.
   */
  public setHoveredObject(id: string | null): void {
    if (this.hoveredObjectId === id) return;

    // Restore previous hovered mesh material
    if (this.hoveredObjectId) {
      const prev = this.instrumentFactory.instruments.get(this.hoveredObjectId);
      if (prev) {
        this._setMeshHighlight(prev, false);
      }
    }

    this.hoveredObjectId = id;

    if (id) {
      const next = this.instrumentFactory.instruments.get(id);
      if (next) {
        this._setMeshHighlight(next, true);
      }
    }
  }

  public updateHoverByRay(
    ndcX: number,
    ndcY: number,
    camera: THREE.Camera
  ): ActiveSurgicalInstrument | null {
    const hit = this.raycastObject(ndcX, ndcY, camera);
    this.setHoveredObject(hit ? hit.instrument.id : null);
    return hit ? hit.instrument : null;
  }

  private _setMeshHighlight(inst: ActiveSurgicalInstrument, highlighted: boolean): void {
    inst.mesh.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) {
        const mat = (child as THREE.Mesh).material as THREE.MeshStandardMaterial;
        if (mat && mat.emissive) {
          if (highlighted) {
            if (!this.savedEmissiveColors.has(inst.id)) {
              this.savedEmissiveColors.set(inst.id, mat.emissive.getHex());
            }
            mat.emissive.setHex(0x0ea5e9);
            mat.emissiveIntensity = 0.45;
          } else {
            const orig = this.savedEmissiveColors.get(inst.id) ?? 0x000000;
            mat.emissive.setHex(orig);
            mat.emissiveIntensity = 0.0;
          }
        }
      }
    });
  }

  /**
   * Grabs a physical object by ID (either from a mouse ray hit or programmatic selection)
   * and starts physics-controlled 3D dragging.
   */
  public grabObject(
    id: string,
    grabPointWorld?: THREE.Vector3,
    camera?: THREE.Camera
  ): ActiveSurgicalInstrument | null {
    const inst = this.instrumentFactory.instruments.get(id);
    if (!inst) return null;

    // If the robot gripper is currently holding this object, release the robot grasp first
    if (
      this.graspManager.isGrasping &&
      this.graspManager.graspedInstrumentId === id
    ) {
      this.graspManager.releaseGrasp();
    }

    this.selectedObjectId = id;
    this.grabbedObjectId = id;
    this.isDragging = true;
    this.instrumentFactory.activeInstrumentId = id;
    this.setHoveredObject(id);
    if (this.transformControls && this.isGizmoActive) {
      this.attachGizmoToObject(id);
    }

    const t = inst.managedBody.body.translation();
    const r = inst.managedBody.body.rotation();
    this.targetPosition.set(t.x, t.y, t.z);
    this.targetQuaternion.set(r.x, r.y, r.z, r.w);
    this.smoothedVelocity.set(0, 0, 0);

    const anchorPoint = grabPointWorld ?? this.targetPosition.clone();
    this.grabOffset.subVectors(this.targetPosition, anchorPoint);
    this._updateDragPlane(anchorPoint, camera);

    inst.managedBody.body.wakeUp();
    return inst;
  }

  /**
   * Attempts to grab an object under the cursor ray.
   */
  public grabObjectByRay(
    ndcX: number,
    ndcY: number,
    camera: THREE.Camera
  ): ActiveSurgicalInstrument | null {
    const hit = this.raycastObject(ndcX, ndcY, camera);
    if (!hit) return null;
    return this.grabObject(hit.instrument.id, hit.hitPoint, camera);
  }

  private _updateDragPlane(anchorPoint: THREE.Vector3, camera?: THREE.Camera): void {
    if (this.dragPlaneMode === 'horizontal') {
      this.dragPlane.setFromNormalAndCoplanarPoint(
        new THREE.Vector3(0, 1, 0),
        anchorPoint
      );
    } else if (this.dragPlaneMode === 'vertical') {
      const normal = new THREE.Vector3(0, 0, 1);
      if (camera) {
        camera.getWorldDirection(normal);
        normal.y = 0;
        if (normal.lengthSq() > 1e-5) normal.normalize();
        else normal.set(0, 0, 1);
      }
      this.dragPlane.setFromNormalAndCoplanarPoint(normal, anchorPoint);
    } else {
      const normal = new THREE.Vector3(0, 0, 1);
      if (camera) {
        camera.getWorldDirection(normal).normalize();
      }
      this.dragPlane.setFromNormalAndCoplanarPoint(normal, anchorPoint);
    }
  }

  /**
   * Changes the active drag plane mode ('horizontal' XZ, 'vertical' XY, or 'camera').
   */
  public setDragPlaneMode(mode: DragPlaneMode, camera?: THREE.Camera): void {
    this.dragPlaneMode = mode;
    if (this.isDragging) {
      this._updateDragPlane(this.targetPosition, camera);
    }
  }

  /**
   * Updates the 3D drag target position from a mouse ray intersection with the active drag plane.
   */
  public dragByRay(
    ndcX: number,
    ndcY: number,
    camera: THREE.Camera
  ): [number, number, number] | null {
    if (!this.isDragging || !this.grabbedObjectId) return null;

    this.raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), camera);
    const intersectPt = new THREE.Vector3();
    if (this.raycaster.ray.intersectPlane(this.dragPlane, intersectPt)) {
      const desired = intersectPt.add(this.grabOffset);
      return this.moveGrabbedObjectTo([desired.x, desired.y, desired.z]);
    }
    return null;
  }

  /**
   * Moves the currently grabbed object's target to a desired 3D coordinate `[x, y, z]`.
   * Respects workspace bounds and minimum floor clearance so the object never passes through the floor.
   */
  public moveGrabbedObjectTo(
    position: [number, number, number]
  ): [number, number, number] {
    const inst = this.grabbedObjectId
      ? this.instrumentFactory.instruments.get(this.grabbedObjectId)
      : null;

    const halfHeight = inst ? (inst.config.dimensions.height ?? 0.036) / 2 : 0.018;
    const minFloorY = halfHeight + 0.0005;

    const clampedX = Math.min(Math.max(position[0], -0.65), 0.65);
    const clampedY = Math.min(Math.max(position[1], minFloorY), 0.75);
    const clampedZ = Math.min(Math.max(position[2], -0.55), 0.65);

    this.targetPosition.set(clampedX, clampedY, clampedZ);
    return [clampedX, clampedY, clampedZ];
  }

  /**
   * Moves the grabbed object vertically along Y by `deltaY` meters.
   */
  public adjustGrabbedHeight(deltaY: number): [number, number, number] {
    return this.moveGrabbedObjectTo([
      this.targetPosition.x,
      this.targetPosition.y + deltaY,
      this.targetPosition.z
    ]);
  }

  /**
   * Moves the grabbed object in depth along Z by `deltaZ` meters.
   */
  public adjustGrabbedDepth(deltaZ: number): [number, number, number] {
    return this.moveGrabbedObjectTo([
      this.targetPosition.x,
      this.targetPosition.y,
      this.targetPosition.z + deltaZ
    ]);
  }

  /**
   * Rotates the grabbed (or selected) object by Euler angles `[rxDeg, ryDeg, rzDeg]`.
   */
  public rotateObjectEulerDeg(
    eulerDeltaDeg: [number, number, number],
    objectId?: string
  ): [number, number, number, number] | null {
    const id = objectId ?? this.grabbedObjectId ?? this.selectedObjectId;
    if (!id) return null;
    const inst = this.instrumentFactory.instruments.get(id);
    if (!inst) return null;

    const deg2rad = Math.PI / 180;
    const deltaQ = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(
        eulerDeltaDeg[0] * deg2rad,
        eulerDeltaDeg[1] * deg2rad,
        eulerDeltaDeg[2] * deg2rad,
        'XYZ'
      )
    );

    if (this.grabbedObjectId === id && this.isDragging) {
      this.targetQuaternion.premultiply(deltaQ).normalize();
      inst.managedBody.body.setRotation(
        {
          x: this.targetQuaternion.x,
          y: this.targetQuaternion.y,
          z: this.targetQuaternion.z,
          w: this.targetQuaternion.w
        },
        true
      );
      inst.managedBody.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      inst.managedBody.body.wakeUp();
      if (inst.mesh) {
        inst.mesh.quaternion.copy(this.targetQuaternion);
      }
      return [
        this.targetQuaternion.x,
        this.targetQuaternion.y,
        this.targetQuaternion.z,
        this.targetQuaternion.w
      ];
    } else {
      const r = inst.managedBody.body.rotation();
      const curQ = new THREE.Quaternion(r.x, r.y, r.z, r.w).premultiply(deltaQ).normalize();
      inst.managedBody.body.setRotation(
        { x: curQ.x, y: curQ.y, z: curQ.z, w: curQ.w },
        true
      );
      inst.managedBody.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      inst.managedBody.body.wakeUp();
      if (inst.mesh) {
        inst.mesh.quaternion.copy(curQ);
      }
      return [curQ.x, curQ.y, curQ.z, curQ.w];
    }
  }

  public initTransformControls(
    scene: THREE.Scene,
    camera: THREE.Camera,
    domElement: HTMLElement,
    orbitControls?: { enabled: boolean } | null
  ): void {
    this.scene = scene;
    this.camera = camera;
    this.orbitControlsRef = orbitControls ?? null;

    this.transformControls = new TransformControls(camera, domElement);
    this.transformControls.setSize(0.70);
    this.transformControls.setMode(this.gizmoMode);
    this.transformControls.enabled = false;

    const tcAny = this.transformControls as unknown as {
      getHelper?: () => THREE.Object3D;
    };
    this.transformControlsHelper =
      typeof tcAny.getHelper === 'function'
        ? tcAny.getHelper()
        : (this.transformControls as unknown as THREE.Object3D);
    this.transformControlsHelper.visible = false;

    this.transformControls.addEventListener('dragging-changed', (event: { value: unknown }) => {
      const dragging = Boolean(event.value);
      this.isDraggingGizmo = dragging;
      if (this.orbitControlsRef) {
        this.orbitControlsRef.enabled = !dragging;
      }
    });

    this.transformControls.addEventListener('change', () => {
      if (this.isDraggingGizmo && this.isGizmoActive) {
        this._syncGizmoTransformToBody();
      }
    });

    this.scene.add(this.transformControlsHelper);
  }

  private _syncGizmoTransformToBody(): void {
    const id = this.selectedObjectId ?? this.grabbedObjectId ?? this.instrumentFactory.activeInstrumentId;
    if (!id) return;
    const inst = this.instrumentFactory.instruments.get(id);
    if (!inst) return;

    const mesh = inst.mesh;
    const body = inst.managedBody.body;

    const halfHeight = (inst.config.dimensions.height ?? 0.036) / 2;
    const clampedY = Math.max(mesh.position.y, halfHeight + 0.001);
    mesh.position.y = clampedY;

    body.setTranslation({ x: mesh.position.x, y: clampedY, z: mesh.position.z }, true);
    body.setRotation({ x: mesh.quaternion.x, y: mesh.quaternion.y, z: mesh.quaternion.z, w: mesh.quaternion.w }, true);
    body.resetForces(true);
    body.resetTorques(true);
    body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    body.wakeUp();

    this.targetPosition.copy(mesh.position);
    this.targetQuaternion.copy(mesh.quaternion);
  }

  public attachGizmoToObject(id: string | null): void {
    if (!this.transformControls) return;
    if (!id) {
      this.transformControls.detach();
      if (this.transformControlsHelper) this.transformControlsHelper.visible = false;
      return;
    }
    const inst = this.instrumentFactory.instruments.get(id);
    if (inst) {
      this.transformControls.attach(inst.mesh);
      this.transformControls.enabled = this.isGizmoActive;
      if (this.transformControlsHelper) {
        this.transformControlsHelper.visible = this.isGizmoActive;
      }
    }
  }

  public setGizmoEnabled(enabled: boolean): void {
    this.isGizmoActive = enabled;
    if (this.transformControls) {
      this.transformControls.enabled = enabled;
      if (this.transformControlsHelper) {
        this.transformControlsHelper.visible = enabled;
      }
      if (enabled) {
        const id = this.selectedObjectId ?? this.instrumentFactory.activeInstrumentId;
        this.attachGizmoToObject(id);
      } else {
        this.transformControls.detach();
      }
    }
  }

  public setGizmoMode(mode: 'translate' | 'rotate'): void {
    this.gizmoMode = mode;
    if (this.transformControls) {
      this.transformControls.setMode(mode);
    }
  }

  public rotateSelectedYaw(deltaDeg: number, objectId?: string): [number, number, number, number] | null {
    return this.rotateObjectEulerDeg([0, deltaDeg, 0], objectId);
  }

  public rotateSelectedPitch(deltaDeg: number, objectId?: string): [number, number, number, number] | null {
    return this.rotateObjectEulerDeg([deltaDeg, 0, 0], objectId);
  }

  public rotateSelectedRoll(deltaDeg: number, objectId?: string): [number, number, number, number] | null {
    return this.rotateObjectEulerDeg([0, 0, deltaDeg], objectId);
  }

  public rotateWithMouseDelta(
    deltaX: number,
    deltaY: number,
    objectId?: string,
    camera?: THREE.Camera
  ): [number, number, number, number] | null {
    const id = objectId ?? this.grabbedObjectId ?? this.selectedObjectId ?? this.instrumentFactory.activeInstrumentId;
    if (!id) return null;
    const inst = this.instrumentFactory.instruments.get(id);
    if (!inst) return null;

    const sensitivity = 0.010;
    const yawDeltaRad = -deltaX * sensitivity;
    const pitchDeltaRad = -deltaY * sensitivity;

    const qYaw = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 1, 0),
      yawDeltaRad
    );

    const pitchAxis = new THREE.Vector3(1, 0, 0);
    if (camera) {
      camera.getWorldDirection(pitchAxis);
      pitchAxis.cross(new THREE.Vector3(0, 1, 0)).normalize();
    }
    const qPitch = new THREE.Quaternion().setFromAxisAngle(pitchAxis, pitchDeltaRad);
    const deltaQ = new THREE.Quaternion().multiplyQuaternions(qYaw, qPitch);

    if (this.grabbedObjectId === id && this.isDragging) {
      this.targetQuaternion.premultiply(deltaQ).normalize();
      inst.managedBody.body.setRotation(
        {
          x: this.targetQuaternion.x,
          y: this.targetQuaternion.y,
          z: this.targetQuaternion.z,
          w: this.targetQuaternion.w
        },
        true
      );
      inst.managedBody.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      inst.managedBody.body.wakeUp();
      if (inst.mesh) {
        inst.mesh.quaternion.copy(this.targetQuaternion);
      }
      return [
        this.targetQuaternion.x,
        this.targetQuaternion.y,
        this.targetQuaternion.z,
        this.targetQuaternion.w
      ];
    } else {
      const r = inst.managedBody.body.rotation();
      const curQ = new THREE.Quaternion(r.x, r.y, r.z, r.w).premultiply(deltaQ).normalize();
      inst.managedBody.body.setRotation(
        { x: curQ.x, y: curQ.y, z: curQ.z, w: curQ.w },
        true
      );
      inst.managedBody.body.wakeUp();
      if (inst.mesh) {
        inst.mesh.quaternion.copy(curQ);
      }
      return [curQ.x, curQ.y, curQ.z, curQ.w];
    }
  }

  public resetObjectRotation(objectId?: string): [number, number, number, number] | null {
    const id = objectId ?? this.grabbedObjectId ?? this.selectedObjectId ?? this.instrumentFactory.activeInstrumentId;
    if (!id) return null;
    const inst = this.instrumentFactory.instruments.get(id);
    if (!inst) return null;

    this.targetQuaternion.set(0, 0, 0, 1);
    inst.managedBody.body.setRotation({ x: 0, y: 0, z: 0, w: 1 }, true);
    inst.managedBody.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    inst.managedBody.body.wakeUp();
    if (inst.mesh) {
      inst.mesh.quaternion.set(0, 0, 0, 1);
    }
    return [0, 0, 0, 1];
  }

  public step(dt: number): void {
    if (!this.isDragging || !this.grabbedObjectId) return;

    const inst = this.instrumentFactory.instruments.get(this.grabbedObjectId);
    if (!inst) {
      this.releaseObject();
      return;
    }

    const rBody = inst.managedBody.body;
    rBody.wakeUp();
    rBody.resetForces(true);

    const curT = rBody.translation();
    const curPos = new THREE.Vector3(curT.x, curT.y, curT.z);
    const posErr = new THREE.Vector3().subVectors(this.targetPosition, curPos);

    // Proportional velocity servo with physical speed limit so CCD & contact constraints resolve cleanly
    const kp = 32.0;
    const maxSpeed = 3.2;
    const desiredVel = posErr.multiplyScalar(kp);
    if (desiredVel.length() > maxSpeed) {
      desiredVel.normalize().multiplyScalar(maxSpeed);
    }

    // Prevent downward driving into the floor plane
    const halfHeight = (inst.config.dimensions.height ?? 0.036) / 2;
    if (curT.y <= halfHeight + 0.002 && desiredVel.y < 0) {
      desiredVel.y = 0;
    }

    rBody.setLinvel({ x: desiredVel.x, y: desiredVel.y, z: desiredVel.z }, true);

    // Orientation PD control toward targetQuaternion
    const curR = rBody.rotation();
    const curQuat = new THREE.Quaternion(curR.x, curR.y, curR.z, curR.w);
    const qDiff = new THREE.Quaternion().multiplyQuaternions(
      this.targetQuaternion,
      curQuat.clone().invert()
    );
    if (qDiff.w < 0) {
      qDiff.x = -qDiff.x;
      qDiff.y = -qDiff.y;
      qDiff.z = -qDiff.z;
      qDiff.w = -qDiff.w;
    }
    const sinHalf = Math.hypot(qDiff.x, qDiff.y, qDiff.z);
    if (sinHalf > 1e-5) {
      const angle = 2 * Math.atan2(sinHalf, Math.min(Math.max(qDiff.w, -1), 1));
      const scale = Math.min((angle * 18.0) / sinHalf, 8.0);
      rBody.setAngvel(
        { x: qDiff.x * scale, y: qDiff.y * scale, z: qDiff.z * scale },
        true
      );
    } else {
      rBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }

    // Track exponentially smoothed velocity for release
    this.smoothedVelocity.lerp(desiredVel, Math.min(1, dt * 15));
  }

  /**
   * Releases the currently grabbed object back into natural free-fall and dynamic physics.
   * Never leaves the object kinematic or suspended in mid-air.
   */
  public releaseObject(): ActiveSurgicalInstrument | null {
    if (!this.grabbedObjectId) {
      this.isDragging = false;
      return null;
    }

    const inst = this.instrumentFactory.instruments.get(this.grabbedObjectId) ?? null;
    this.grabbedObjectId = null;
    this.isDragging = false;

    if (inst) {
      const rBody = inst.managedBody.body;
      rBody.resetForces(true);
      rBody.wakeUp();
      // Preserve a gentle, damped fraction of release velocity without flinging objects out of bounds
      const relVx = Math.min(Math.max(this.smoothedVelocity.x * 0.25, -0.8), 0.8);
      const relVy = Math.min(Math.max(this.smoothedVelocity.y * 0.15, -0.5), 0.3);
      const relVz = Math.min(Math.max(this.smoothedVelocity.z * 0.25, -0.8), 0.8);
      rBody.setLinvel({ x: relVx, y: relVy, z: relVz }, true);
      rBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }

    this.smoothedVelocity.set(0, 0, 0);
    return inst;
  }
}
