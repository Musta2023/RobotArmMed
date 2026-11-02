/**
 * @file PhysicsDebugger.ts
 * Comprehensive Physics & Collision Debugging System (Phase 7).
 *
 * Displays & tracks:
 * - Collider wireframes (color-coded by awake/sleeping/kinematic/fixed/collision state)
 * - Rigid-body world positions
 * - Collision contact points (3D spheres)
 * - Contact normals (3D arrows)
 * - Active collision pairs
 * - Object velocities (3D velocity vectors & speed telemetry)
 * - Object masses (kg)
 * - Joint axes and limits
 * - Sleeping vs awake state
 * - Objects currently in contact
 * - Console diagnostics:
 *     [COLLISION] Object_A <-> Object_B
 *     [CONTACT] Penetration detected
 *     [BLOCKED] Object_C has no significant movement
 *     [DEBUG] Body sleeping / awake
 */

import * as THREE from 'three';
import { RigidBodyManager, ManagedRigidBody } from './RigidBodyManager.js';
import { CollisionManager, ContactEventRecord } from './CollisionManager.js';
import { RobotModel } from '../robot/RobotModel.js';

export interface BodyDebugInfo {
  id: string;
  name: string;
  category: string;
  motionType: string;
  position: [number, number, number];
  velocity: [number, number, number];
  speedMetersPerSec: number;
  massKg: number;
  isSleeping: boolean;
  inContact: boolean;
  isBlocked: boolean;
}

export interface JointDebugInfo {
  id: string;
  pivotPosition: [number, number, number];
  worldAxis: [number, number, number];
  currentAngleDeg: number;
  minLimitDeg: number;
  maxLimitDeg: number;
  isAtLimit: boolean;
}

export interface PhysicsDebugSnapshot {
  enabled: boolean;
  totalBodies: number;
  dynamicBodies: number;
  bodies: BodyDebugInfo[];
  activeCollisionPairs: string[];
  contacts: ContactEventRecord[];
  sleepingBodyIds: string[];
  inContactBodyIds: string[];
  blockedBodyIds: string[];
  joints: JointDebugInfo[];
  recentLogs: string[];
}

export class PhysicsDebugger {
  public bodyManager: RigidBodyManager;
  public collisionManager: CollisionManager;
  public robotModel: RobotModel;

  public enabled: boolean = false;
  public consoleLoggingEnabled: boolean = false;
  public debugGroup: THREE.Group;

  public get root(): THREE.Group {
    return this.debugGroup;
  }

  public getSnapshot(): PhysicsDebugSnapshot {
    return this.update();
  }

  private wireframeGroup: THREE.Group;
  private contactGroup: THREE.Group;
  private velocityGroup: THREE.Group;
  private jointAxesGroup: THREE.Group;

  private prevSleepStates: Map<string, boolean> = new Map();
  private prevCollisionPairSet: Set<string> = new Set();
  public recentLogs: string[] = [];
  public latestSnapshot: PhysicsDebugSnapshot;

  constructor(
    bodyManager: RigidBodyManager,
    collisionManager: CollisionManager,
    robotModel: RobotModel,
    parentScene?: THREE.Scene
  ) {
    this.bodyManager = bodyManager;
    this.collisionManager = collisionManager;
    this.robotModel = robotModel;

    this.debugGroup = new THREE.Group();
    this.debugGroup.name = 'PhysicsDebuggerOverlayGroup';
    this.debugGroup.visible = false;

    this.wireframeGroup = new THREE.Group();
    this.contactGroup = new THREE.Group();
    this.velocityGroup = new THREE.Group();
    this.jointAxesGroup = new THREE.Group();

    this.debugGroup.add(this.wireframeGroup);
    this.debugGroup.add(this.contactGroup);
    this.debugGroup.add(this.velocityGroup);
    this.debugGroup.add(this.jointAxesGroup);

    if (parentScene) {
      parentScene.add(this.debugGroup);
    }

    this.latestSnapshot = {
      enabled: false,
      totalBodies: 0,
      dynamicBodies: 0,
      bodies: [],
      activeCollisionPairs: [],
      contacts: [],
      sleepingBodyIds: [],
      inContactBodyIds: [],
      blockedBodyIds: [],
      joints: [],
      recentLogs: []
    };
  }

  public setEnabled(enabled: boolean, enableConsoleLogs: boolean = true): void {
    this.enabled = enabled;
    this.consoleLoggingEnabled = enableConsoleLogs;
    this.debugGroup.visible = enabled;
    if (enabled) {
      this.update();
    }
  }

  private _emitDiagnostic(line: string): void {
    this.recentLogs.push(line);
    if (this.recentLogs.length > 40) {
      this.recentLogs.shift();
    }
    if (this.consoleLoggingEnabled) {
      console.log(line);
    }
  }

  /**
   * Evaluates all rigid bodies, contacts, sleeping states, blocked states, and joint axes,
   * and updates the 3D debug wireframe/vector overlay when enabled.
   */
  public update(): PhysicsDebugSnapshot {
    const contacts = this.collisionManager.activeContacts;
    const inContactSet = new Set<string>();
    const activeCollisionPairs: string[] = [];
    const currentPairSet = new Set<string>();

    for (const c of contacts) {
      inContactSet.add(c.bodyIdA);
      inContactSet.add(c.bodyIdB);
      const pairKey = `${c.bodyIdA} <-> ${c.bodyIdB}`;
      activeCollisionPairs.push(pairKey);
      currentPairSet.add(pairKey);

      if (!this.prevCollisionPairSet.has(pairKey)) {
        this._emitDiagnostic(`[COLLISION] ${c.bodyIdA} <-> ${c.bodyIdB}`);
        if (c.penetrationDepthMeters > 0.001) {
          this._emitDiagnostic(
            `[CONTACT] Penetration detected (${c.bodyIdA} <-> ${c.bodyIdB}: ${(c.penetrationDepthMeters * 1000).toFixed(2)} mm)`
          );
        }
      }
    }
    this.prevCollisionPairSet = currentPairSet;

    const bodiesInfo: BodyDebugInfo[] = [];
    const sleepingBodyIds: string[] = [];
    const inContactBodyIds: string[] = Array.from(inContactSet);
    const blockedBodyIds: string[] = [];

    for (const [id, managed] of this.bodyManager.bodies) {
      const t = managed.body.translation();
      const lv = managed.body.linvel();
      const speed = Math.sqrt(lv.x * lv.x + lv.y * lv.y + lv.z * lv.z);
      const isSleeping = managed.body.isDynamic() ? managed.body.isSleeping() : false;
      const inContact = inContactSet.has(id);

      const prevSleep = this.prevSleepStates.get(id);
      if (prevSleep !== undefined && prevSleep !== isSleeping) {
        this._emitDiagnostic(
          `[DEBUG] Body sleeping / awake: ${id} is now ${isSleeping ? 'SLEEPING' : 'AWAKE'}`
        );
      }
      this.prevSleepStates.set(id, isSleeping);

      if (isSleeping) {
        sleepingBodyIds.push(id);
      }

      // Check if a dynamic body has a significant applied horizontal/vertical force (> 0.5 N)
      // but exhibits near-zero velocity over multiple consecutive steps (unexpected blocking)
      const [fx, fy, fz] = managed.lastAppliedForceNewtons;
      const appliedForceMag = Math.sqrt(fx * fx + fy * fy + fz * fz);
      let isBlocked = false;

      if (managed.body.isDynamic() && appliedForceMag > 0.5 && speed < 0.005) {
        managed.blockedStepsCount += 1;
        if (managed.blockedStepsCount >= 5) {
          isBlocked = true;
          blockedBodyIds.push(id);
          if (managed.blockedStepsCount === 5) {
            this._emitDiagnostic(
              `[BLOCKED] ${id} has no significant movement under ${appliedForceMag.toFixed(2)} N applied force`
            );
          }
        }
      } else {
        managed.blockedStepsCount = 0;
      }

      // Reset per-step tracked force after inspection
      managed.lastAppliedForceNewtons = [0, 0, 0];

      bodiesInfo.push({
        id,
        name: managed.spec.name,
        category: managed.spec.category,
        motionType: managed.spec.motionType,
        position: [t.x, t.y, t.z],
        velocity: [lv.x, lv.y, lv.z],
        speedMetersPerSec: speed,
        massKg: managed.spec.massKg,
        isSleeping,
        inContact,
        isBlocked
      });
    }

    // Build Joint Axes & Limits debug info
    const kin = this.robotModel.syncVisualHierarchyFromJoints(false);
    const snap = this.robotModel.state.latestSnapshot;
    const jointsInfo: JointDebugInfo[] = snap.joints.map((jt, idx) => {
      const p = kin.jointWorldPivots[idx] ?? new THREE.Vector3();
      const a = kin.jointWorldAxes[idx] ?? new THREE.Vector3(0, 1, 0);
      return {
        id: jt.id,
        pivotPosition: [p.x, p.y, p.z],
        worldAxis: [a.x, a.y, a.z],
        currentAngleDeg: jt.currentAngleDeg,
        minLimitDeg: jt.minAngleDeg,
        maxLimitDeg: jt.maxAngleDeg,
        isAtLimit: jt.isAtLimit
      };
    });

    this.latestSnapshot = {
      enabled: this.enabled,
      totalBodies: bodiesInfo.length,
      dynamicBodies: bodiesInfo.filter((b) => b.motionType === 'dynamic').length,
      bodies: bodiesInfo,
      activeCollisionPairs,
      contacts: [...contacts],
      sleepingBodyIds,
      inContactBodyIds,
      blockedBodyIds,
      joints: jointsInfo,
      recentLogs: [...this.recentLogs]
    };

    if (this.enabled) {
      this._rebuildVisualOverlays(bodiesInfo, contacts, jointsInfo);
    }

    return this.latestSnapshot;
  }

  private _clearGroup(group: THREE.Group): void {
    while (group.children.length > 0) {
      group.remove(group.children[0]);
    }
  }

  private _rebuildVisualOverlays(
    bodies: BodyDebugInfo[],
    contacts: ContactEventRecord[],
    joints: JointDebugInfo[]
  ): void {
    this._clearGroup(this.wireframeGroup);
    this._clearGroup(this.contactGroup);
    this._clearGroup(this.velocityGroup);
    this._clearGroup(this.jointAxesGroup);

    // 1. Collider Wireframes & Rigid-Body Center Markers
    for (const bInfo of bodies) {
      const managed = this.bodyManager.getBody(bInfo.id);
      if (!managed) continue;

      const dims = managed.spec.dimensions;
      let geo: THREE.BufferGeometry;
      if (managed.spec.shape === 'cylinder' || managed.spec.shape === 'capsule') {
        const r = dims.radius ?? (dims.width ?? 0.03) / 2;
        geo = new THREE.CylinderGeometry(r, r, dims.height ?? 0.06, 12);
      } else if (managed.spec.shape === 'sphere') {
        const r = dims.radius ?? 0.025;
        geo = new THREE.SphereGeometry(r, 12, 12);
      } else {
        geo = new THREE.BoxGeometry(
          dims.width ?? 0.04,
          dims.height ?? 0.04,
          dims.depth ?? 0.04
        );
      }

      let colorHex = 0x10b981; // Green = awake dynamic
      if (bInfo.inContact) {
        colorHex = 0xef4444; // Red = in contact
      } else if (bInfo.isSleeping) {
        colorHex = 0x64748b; // Slate = sleeping
      } else if (managed.spec.motionType === 'kinematicPosition') {
        colorHex = 0x38bdf8; // Cyan = kinematic robot/gripper
      } else if (managed.spec.motionType === 'fixed') {
        colorHex = 0xf59e0b; // Amber = fixed platform/obstacle
      }

      const wireMat = new THREE.MeshBasicMaterial({
        color: colorHex,
        wireframe: true,
        transparent: true,
        opacity: 0.75
      });
      const wireMesh = new THREE.Mesh(geo, wireMat);
      const t = managed.body.translation();
      const r = managed.body.rotation();
      wireMesh.position.set(t.x, t.y, t.z);
      wireMesh.quaternion.set(r.x, r.y, r.z, r.w);
      this.wireframeGroup.add(wireMesh);

      // Velocity vector arrow for moving bodies
      if (bInfo.speedMetersPerSec > 0.01) {
        const dir = new THREE.Vector3(
          bInfo.velocity[0],
          bInfo.velocity[1],
          bInfo.velocity[2]
        ).normalize();
        const arrowLen = Math.min(Math.max(bInfo.speedMetersPerSec * 0.25, 0.04), 0.25);
        const velArrow = new THREE.ArrowHelper(
          dir,
          new THREE.Vector3(t.x, t.y, t.z),
          arrowLen,
          0xa855f7
        );
        this.velocityGroup.add(velArrow);
      }
    }

    // 2. Contact Points & Contact Normals
    const ptGeo = new THREE.SphereGeometry(0.006, 8, 8);
    const ptMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });

    for (const c of contacts) {
      const pt = new THREE.Vector3(
        c.contactPoint[0],
        c.contactPoint[1],
        c.contactPoint[2]
      );
      const sphere = new THREE.Mesh(ptGeo, ptMat);
      sphere.position.copy(pt);
      this.contactGroup.add(sphere);

      const nDir = new THREE.Vector3(
        c.contactNormal[0],
        c.contactNormal[1],
        c.contactNormal[2]
      ).normalize();
      const normalArrow = new THREE.ArrowHelper(nDir, pt, 0.06, 0xfacc15);
      this.contactGroup.add(normalArrow);
    }

    // 3. Joint Axes & Limits
    for (const j of joints) {
      const pivot = new THREE.Vector3(
        j.pivotPosition[0],
        j.pivotPosition[1],
        j.pivotPosition[2]
      );
      const axis = new THREE.Vector3(
        j.worldAxis[0],
        j.worldAxis[1],
        j.worldAxis[2]
      ).normalize();
      const axisColor = j.isAtLimit ? 0xef4444 : 0x22d3ee;
      const jointArrow = new THREE.ArrowHelper(axis, pivot, 0.075, axisColor);
      this.jointAxesGroup.add(jointArrow);
    }
  }
}
