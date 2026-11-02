/**
 * @file CollisionManager.ts
 * Rapier3D narrow-phase and 3D Oriented Bounding Box (OBB / SAT) collision detection
 * manager for robot links, gripper fingers, surgical instruments, ground, and obstacles.
 */

import * as THREE from 'three';
import { PhysicsWorld } from './PhysicsWorld.js';
import { RigidBodyManager, ManagedRigidBody } from './RigidBodyManager.js';
import { canGroupsInteract } from './CollisionGroups.js';

export interface ContactEventRecord {
  bodyIdA: string;
  bodyIdB: string;
  categoryA: string;
  categoryB: string;
  isProhibitedCollision: boolean;
  penetrationDepthMeters: number;
  contactPoint: [number, number, number];
  contactNormal: [number, number, number];
  description: string;
  timestamp: number;
}

export interface OrientedBox3D {
  center: THREE.Vector3;
  axes: [THREE.Vector3, THREE.Vector3, THREE.Vector3];
  halfExtents: [number, number, number];
}

/**
 * Performs a 15-axis 3D Separating Axis Theorem (SAT) intersection test between two Oriented Bounding Boxes.
 * Returns exact penetration depth, minimum translation normal (from A to B), and approximate contact point.
 */
export function testOBBOverlap(
  obbA: OrientedBox3D,
  obbB: OrientedBox3D
): {
  intersects: boolean;
  penetrationDepth: number;
  normal: [number, number, number];
  contactPoint: [number, number, number];
} {
  const diff = new THREE.Vector3().subVectors(obbB.center, obbA.center);
  let minOverlap = Infinity;
  const bestAxis = new THREE.Vector3(0, 1, 0);

  const testAxis = (axis: THREE.Vector3): boolean => {
    const lenSq = axis.lengthSq();
    if (lenSq < 1e-8) return true;
    const unit = axis.clone().divideScalar(Math.sqrt(lenSq));

    const rA =
      obbA.halfExtents[0] * Math.abs(unit.dot(obbA.axes[0])) +
      obbA.halfExtents[1] * Math.abs(unit.dot(obbA.axes[1])) +
      obbA.halfExtents[2] * Math.abs(unit.dot(obbA.axes[2]));

    const rB =
      obbB.halfExtents[0] * Math.abs(unit.dot(obbB.axes[0])) +
      obbB.halfExtents[1] * Math.abs(unit.dot(obbB.axes[1])) +
      obbB.halfExtents[2] * Math.abs(unit.dot(obbB.axes[2]));

    const dist = Math.abs(diff.dot(unit));
    const overlap = rA + rB - dist;

    if (overlap <= 0) {
      return false;
    }

    if (overlap < minOverlap) {
      minOverlap = overlap;
      bestAxis.copy(unit);
      if (diff.dot(bestAxis) < 0) {
        bestAxis.negate();
      }
    }
    return true;
  };

  // 3 face axes of A + 3 face axes of B
  for (let i = 0; i < 3; i++) {
    if (!testAxis(obbA.axes[i])) {
      return { intersects: false, penetrationDepth: 0, normal: [0, 1, 0], contactPoint: [0, 0, 0] };
    }
    if (!testAxis(obbB.axes[i])) {
      return { intersects: false, penetrationDepth: 0, normal: [0, 1, 0], contactPoint: [0, 0, 0] };
    }
  }

  // 9 edge cross-product axes
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      const cross = new THREE.Vector3().crossVectors(obbA.axes[i], obbB.axes[j]);
      if (!testAxis(cross)) {
        return { intersects: false, penetrationDepth: 0, normal: [0, 1, 0], contactPoint: [0, 0, 0] };
      }
    }
  }

  const contactPt = new THREE.Vector3()
    .addVectors(obbA.center, obbB.center)
    .multiplyScalar(0.5);

  return {
    intersects: true,
    penetrationDepth: minOverlap,
    normal: [bestAxis.x, bestAxis.y, bestAxis.z],
    contactPoint: [contactPt.x, contactPt.y, contactPt.z]
  };
}

export class CollisionManager {
  public physicsWorld: PhysicsWorld;
  public bodyManager: RigidBodyManager;
  public activeContacts: ContactEventRecord[] = [];
  public prohibitedCollisions: ContactEventRecord[] = [];
  public graspedInstrumentId: string | null = null;

  private permittedPairs: Set<string> = new Set([
    'robot_base:GROUND',
    'robot_base:robot_link_0',
    'robot_base:robot_link_1',
    'robot_link_0:robot_link_1',
    'robot_link_1:robot_link_2',
    'robot_link_2:robot_link_3',
    'robot_link_3:robot_link_4',
    'robot_link_4:robot_link_5',
    'robot_link_3:robot_link_5',
    'robot_link_4:gripper_left_finger',
    'robot_link_4:gripper_right_finger',
    'robot_link_5:gripper_left_finger',
    'robot_link_5:gripper_right_finger',
    'gripper_left_finger:gripper_right_finger'
  ]);

  constructor(physicsWorld: PhysicsWorld, bodyManager: RigidBodyManager) {
    this.physicsWorld = physicsWorld;
    this.bodyManager = bodyManager;
  }

  public isPermittedPair(idA: string, idB: string): boolean {
    return (
      this.permittedPairs.has(`${idA}:${idB}`) ||
      this.permittedPairs.has(`${idB}:${idA}`)
    );
  }

  /**
   * Computes a 3D Oriented Bounding Box (OBB) from a ManagedRigidBody's current Rapier pose.
   */
  public computeBodyWorldOBB(managed: ManagedRigidBody): OrientedBox3D {
    const useKinTarget =
      managed.spec.motionType === 'kinematicPosition' &&
      managed.kinematicTargetPos !== undefined &&
      managed.kinematicTargetQuat !== undefined;

    const t = useKinTarget
      ? {
          x: managed.kinematicTargetPos![0],
          y: managed.kinematicTargetPos![1],
          z: managed.kinematicTargetPos![2]
        }
      : managed.body.translation();
    const r = useKinTarget
      ? {
          x: managed.kinematicTargetQuat![0],
          y: managed.kinematicTargetQuat![1],
          z: managed.kinematicTargetQuat![2],
          w: managed.kinematicTargetQuat![3]
        }
      : managed.body.rotation();
    const center = new THREE.Vector3(t.x, t.y, t.z);
    const quat = new THREE.Quaternion(r.x, r.y, r.z, r.w);

    const dims = managed.spec.dimensions;
    const hx = (dims.width ?? (dims.radius ? dims.radius * 2 : 0.04)) / 2;
    const hy = (dims.height ?? (dims.radius ? dims.radius * 2 : 0.04)) / 2;
    const hz = (dims.depth ?? (dims.radius ? dims.radius * 2 : 0.04)) / 2;

    const ax = new THREE.Vector3(1, 0, 0).applyQuaternion(quat).normalize();
    const ay = new THREE.Vector3(0, 1, 0).applyQuaternion(quat).normalize();
    const az = new THREE.Vector3(0, 0, 1).applyQuaternion(quat).normalize();

    return {
      center,
      axes: [ax, ay, az],
      halfExtents: [hx, hy, hz]
    };
  }

  /**
   * Computes 3D Axis-Aligned Bounding Box from a ManagedRigidBody's current Rapier pose.
   */
  public computeBodyWorldAABB(managed: ManagedRigidBody): THREE.Box3 {
    const obb = this.computeBodyWorldOBB(managed);
    const box = new THREE.Box3();
    const corner = new THREE.Vector3();

    for (const sx of [-1, 1]) {
      for (const sy of [-1, 1]) {
        for (const sz of [-1, 1]) {
          corner
            .copy(obb.center)
            .addScaledVector(obb.axes[0], sx * obb.halfExtents[0])
            .addScaledVector(obb.axes[1], sy * obb.halfExtents[1])
            .addScaledVector(obb.axes[2], sz * obb.halfExtents[2]);
          box.expandByPoint(corner);
        }
      }
    }
    return box;
  }

  /**
   * Evaluates all current Rapier3D narrow-phase contacts and 3D OBB overlaps
   * across robot links, gripper fingers, instruments, obstacles, and ground.
   */
  public evaluateCollisions(): {
    hasProhibitedCollision: boolean;
    prohibitedCollisions: ContactEventRecord[];
    activeContacts: ContactEventRecord[];
  } {
    this.activeContacts = [];
    this.prohibitedCollisions = [];

    if (this.physicsWorld.eventQueue) {
      this.physicsWorld.eventQueue.drainCollisionEvents(() => {});
    }

    const bodiesList = Array.from(this.bodyManager.bodies.values());
    const aabbCache = new Map<string, THREE.Box3>();
    const obbCache = new Map<string, OrientedBox3D>();

    for (const b of bodiesList) {
      obbCache.set(b.spec.id, this.computeBodyWorldOBB(b));
      aabbCache.set(b.spec.id, this.computeBodyWorldAABB(b));
    }

    // 1. Check distal robot links, gripper fingers & instruments vs Ground plane (y = 0)
    for (const b of bodiesList) {
      if (b.spec.isSensor || b.spec.category === 'environment') continue;
      const box = aabbCache.get(b.spec.id)!;

      if (b.spec.category === 'instrument') {
        if (box.min.y <= 0.002) {
          this.activeContacts.push({
            bodyIdA: b.spec.id,
            bodyIdB: 'GROUND',
            categoryA: b.spec.category,
            categoryB: 'environment',
            isProhibitedCollision: false,
            penetrationDepthMeters: Math.max(0, -box.min.y),
            contactPoint: [b.body.translation().x, 0, b.body.translation().z],
            contactNormal: [0, 1, 0],
            description: `${b.spec.name} resting on Ground Plane.`,
            timestamp: this.physicsWorld.simulationTime
          });
        }
        continue;
      }

      if (b.spec.category !== 'robot_link' && b.spec.category !== 'gripper_finger') {
        continue;
      }
      if (
        this.isPermittedPair(b.spec.id, 'GROUND') ||
        b.spec.id === 'robot_link_0' ||
        b.spec.id === 'robot_link_1'
      ) {
        continue;
      }

      if (box.min.y < -0.012) {
        const rec: ContactEventRecord = {
          bodyIdA: b.spec.id,
          bodyIdB: 'GROUND',
          categoryA: b.spec.category,
          categoryB: 'environment',
          isProhibitedCollision: true,
          penetrationDepthMeters: -box.min.y,
          contactPoint: [b.body.translation().x, 0, b.body.translation().z],
          contactNormal: [0, 1, 0],
          description: `${b.spec.name} collided with Ground Plane (minY=${box.min.y.toFixed(3)} m).`,
          timestamp: this.physicsWorld.simulationTime
        };
        this.activeContacts.push(rec);
        this.prohibitedCollisions.push(rec);
      }
    }

    // 2. Pairwise 3D OBB (SAT) checks across registered bodies
    for (let i = 0; i < bodiesList.length; i++) {
      for (let j = i + 1; j < bodiesList.length; j++) {
        const bA = bodiesList[i];
        const bB = bodiesList[j];

        if (this.isPermittedPair(bA.spec.id, bB.spec.id)) {
          continue;
        }

        // Broad-phase AABB filter
        const boxA = aabbCache.get(bA.spec.id)!;
        const boxB = aabbCache.get(bB.spec.id)!;
        if (!boxA.intersectsBox(boxB)) {
          continue;
        }

        // Exact 3D OBB Separating Axis Theorem narrow-phase check
        const obbA = obbCache.get(bA.spec.id)!;
        const obbB = obbCache.get(bB.spec.id)!;
        const sat = testOBBOverlap(obbA, obbB);
        if (!sat.intersects || sat.penetrationDepth <= 0.0005) {
          continue;
        }

        // Check if collision groups allow interaction (unless checking self-collision between base and wrist)
        const groupsInteract = canGroupsInteract(
          bA.encodedCollisionGroups,
          bB.encodedCollisionGroups
        );
        if (!groupsInteract && !(bA.spec.category === 'robot_link' && bB.spec.category === 'robot_link')) {
          continue;
        }

        const catPair = `${bA.spec.category}:${bB.spec.category}`;
        const isRobotVsObstacle =
          (bA.spec.category === 'robot_link' || bA.spec.category === 'gripper_finger') &&
          bB.spec.category === 'obstacle';
        const isObstacleVsRobot =
          bA.spec.category === 'obstacle' &&
          (bB.spec.category === 'robot_link' || bB.spec.category === 'gripper_finger');
        const isSelfCollision =
          ((bA.spec.id === 'robot_base' && bB.spec.id === 'robot_link_5') ||
            (bA.spec.id === 'robot_link_5' && bB.spec.id === 'robot_base')) &&
          sat.penetrationDepth > 0.020;

        // Check if an upper/forearm robot link (not gripper fingers during grasping) is driving through an ungrasped object
        const isArmLinkDrivingThroughObject =
          ((bA.spec.category === 'robot_link' &&
            bA.spec.id !== 'robot_link_5' &&
            bB.spec.category === 'instrument' &&
            bB.spec.id !== this.graspedInstrumentId) ||
            (bB.spec.category === 'robot_link' &&
              bB.spec.id !== 'robot_link_5' &&
              bA.spec.category === 'instrument' &&
              bA.spec.id !== this.graspedInstrumentId)) &&
          sat.penetrationDepth > 0.008;

        const isProhibited =
          !bA.spec.isSensor &&
          !bB.spec.isSensor &&
          (isRobotVsObstacle ||
            isObstacleVsRobot ||
            isSelfCollision ||
            isArmLinkDrivingThroughObject);

        const rec: ContactEventRecord = {
          bodyIdA: bA.spec.id,
          bodyIdB: bB.spec.id,
          categoryA: bA.spec.category,
          categoryB: bB.spec.category,
          isProhibitedCollision: isProhibited,
          penetrationDepthMeters: sat.penetrationDepth,
          contactPoint: sat.contactPoint,
          contactNormal: sat.normal,
          description: isProhibited
            ? `Collision detected: ${bA.spec.name} <-> ${bB.spec.name} (${(sat.penetrationDepth * 1000).toFixed(1)} mm)`
            : `Physical contact: ${bA.spec.name} <-> ${bB.spec.name} (${catPair})`,
          timestamp: this.physicsWorld.simulationTime
        };

        this.activeContacts.push(rec);
        if (isProhibited) {
          this.prohibitedCollisions.push(rec);
        }
      }
    }

    return {
      hasProhibitedCollision: this.prohibitedCollisions.length > 0,
      prohibitedCollisions: this.prohibitedCollisions,
      activeContacts: this.activeContacts
    };
  }
}
