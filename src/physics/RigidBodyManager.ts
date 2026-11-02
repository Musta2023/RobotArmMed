/**
 * @file RigidBodyManager.ts
 * Manages creation, physical properties (mass, center of mass, inertia tensor,
 * friction, restitution), collision/solver groups, external forces, and Three.js
 * synchronization for Rapier3D rigid bodies.
 */

import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { PhysicsWorld } from './PhysicsWorld.js';
import { MaterialType } from './MaterialProperties.js';
import {
  COLLISION_PRESETS,
  encodeCollisionGroups
} from './CollisionGroups.js';

export type BodyMotionType = 'fixed' | 'dynamic' | 'kinematicPosition' | 'kinematicVelocity';
export type ColliderShapeType = 'box' | 'cylinder' | 'sphere' | 'capsule' | 'convexHull';

export interface RigidBodySpec {
  id: string;
  name: string;
  category:
    | 'robot_link'
    | 'gripper_finger'
    | 'instrument'
    | 'obstacle'
    | 'environment'
    | 'sensor';
  motionType: BodyMotionType;
  shape: ColliderShapeType;
  dimensions: {
    width?: number;
    height?: number;
    depth?: number;
    radius?: number;
    vertices?: Float32Array;
  };
  position: [number, number, number];
  quaternion?: [number, number, number, number];
  massKg: number;
  centerOfMass?: [number, number, number];
  friction: number;
  restitution: number;
  materialType?: MaterialType;
  linearDamping?: number;
  angularDamping?: number;
  isSensor?: boolean;
  ccdEnabled?: boolean;
  contactSkin?: number;
  collisionGroupMembership?: number;
  collisionGroupFilter?: number;
  mesh?: THREE.Object3D;
}

export interface ManagedRigidBody {
  spec: RigidBodySpec;
  body: RAPIER.RigidBody;
  collider: RAPIER.Collider;
  mesh?: THREE.Object3D;
  principalInertia: [number, number, number];
  encodedCollisionGroups: number;
  lastAppliedForceNewtons: [number, number, number];
  blockedStepsCount: number;
  kinematicTargetPos?: [number, number, number];
  kinematicTargetQuat?: [number, number, number, number];
}

/**
 * Computes analytical principal moments of inertia [Ixx, Iyy, Izz] in kg·m²
 * for a solid box, cylinder, capsule, or sphere of mass M.
 */
export function computePrincipalInertia(
  shape: ColliderShapeType,
  massKg: number,
  dims: RigidBodySpec['dimensions']
): [number, number, number] {
  const m = Math.max(massKg, 1e-4);

  if (shape === 'cylinder' || shape === 'capsule') {
    const r = dims.radius ?? (dims.width ?? 0.03) / 2;
    const h = dims.height ?? 0.06;
    const ixz = (1 / 12) * m * (3 * r * r + h * h);
    const iy = 0.5 * m * r * r;
    return [ixz, iy, ixz];
  }

  if (shape === 'sphere') {
    const r = dims.radius ?? 0.025;
    const i = 0.4 * m * r * r;
    return [i, i, i];
  }

  const w = dims.width ?? 0.04;
  const h = dims.height ?? 0.04;
  const d = dims.depth ?? 0.04;
  const ixx = (1 / 12) * m * (h * h + d * d);
  const iyy = (1 / 12) * m * (w * w + d * d);
  const izz = (1 / 12) * m * (w * w + h * h);
  return [ixx, iyy, izz];
}

/**
 * Encodes a 16-bit membership mask and 16-bit filter mask into a 32-bit Rapier interaction group integer.
 */
export function packInteractionGroups(
  membership: number = 0xffff,
  filter: number = 0xffff
): number {
  return encodeCollisionGroups(membership, filter);
}

/**
 * Resolves default collision membership & filter masks for a body spec when not explicitly provided.
 */
function resolveDefaultCollisionGroups(spec: RigidBodySpec): {
  membership: number;
  filter: number;
} {
  if (
    spec.collisionGroupMembership !== undefined &&
    spec.collisionGroupFilter !== undefined
  ) {
    return {
      membership: spec.collisionGroupMembership,
      filter: spec.collisionGroupFilter
    };
  }

  if (spec.isSensor || spec.category === 'sensor') {
    return COLLISION_PRESETS.SENSOR;
  }

  switch (spec.category) {
    case 'instrument':
      return COLLISION_PRESETS.DYNAMIC_OBJECT;
    case 'obstacle':
      return COLLISION_PRESETS.OBSTACLE;
    case 'gripper_finger':
      return COLLISION_PRESETS.GRIPPER_FINGER;
    case 'robot_link':
      if (
        spec.id === 'robot_base' ||
        spec.id === 'robot_link_0' ||
        spec.id === 'robot_link_1'
      ) {
        return COLLISION_PRESETS.ROBOT_BASE_LOWER;
      }
      if (spec.id === 'robot_link_2' || spec.id === 'robot_link_3') {
        return COLLISION_PRESETS.ROBOT_UPPER_ARM;
      }
      return COLLISION_PRESETS.ROBOT_WRIST;
    case 'environment':
    default:
      return COLLISION_PRESETS.GROUND_AND_PLATFORMS;
  }
}

export class RigidBodyManager {
  public physicsWorld: PhysicsWorld;
  public bodies: Map<string, ManagedRigidBody> = new Map();
  public colliderHandleToId: Map<number, string> = new Map();

  constructor(physicsWorld: PhysicsWorld) {
    this.physicsWorld = physicsWorld;
  }

  /**
   * Creates and registers a Rapier3D RigidBody and Collider with accurate geometry,
   * mass, inertia tensor, contact skin, CCD, and logical collision/solver groups.
   */
  public createBody(spec: RigidBodySpec): ManagedRigidBody {
    if (this.bodies.has(spec.id)) {
      this.removeBody(spec.id);
    }

    let bodyDesc: RAPIER.RigidBodyDesc;
    switch (spec.motionType) {
      case 'fixed':
        bodyDesc = RAPIER.RigidBodyDesc.fixed();
        break;
      case 'kinematicPosition':
        bodyDesc = RAPIER.RigidBodyDesc.kinematicPositionBased();
        break;
      case 'kinematicVelocity':
        bodyDesc = RAPIER.RigidBodyDesc.kinematicVelocityBased();
        break;
      case 'dynamic':
      default:
        bodyDesc = RAPIER.RigidBodyDesc.dynamic();
        break;
    }

    bodyDesc.setTranslation(spec.position[0], spec.position[1], spec.position[2]);
    if (spec.quaternion) {
      bodyDesc.setRotation({
        x: spec.quaternion[0],
        y: spec.quaternion[1],
        z: spec.quaternion[2],
        w: spec.quaternion[3]
      });
    }

    // Use low damping on dynamic bodies so applied forces and gravity act naturally without sticking
    bodyDesc.setLinearDamping(spec.linearDamping ?? 0.04);
    bodyDesc.setAngularDamping(spec.angularDamping ?? 0.06);
    bodyDesc.setCcdEnabled(spec.ccdEnabled ?? spec.motionType === 'dynamic');
    bodyDesc.setCanSleep(true);

    const principalInertia = computePrincipalInertia(
      spec.shape,
      spec.massKg,
      spec.dimensions
    );

    const body = this.physicsWorld.world.createRigidBody(bodyDesc);

    let colliderDesc: RAPIER.ColliderDesc;
    const dims = spec.dimensions;

    if (spec.shape === 'convexHull' && dims.vertices && dims.vertices.length >= 12) {
      const hull = RAPIER.ColliderDesc.convexHull(dims.vertices);
      if (hull) {
        colliderDesc = hull;
      } else {
        const hx = (dims.width ?? 0.04) / 2;
        const hy = (dims.height ?? 0.04) / 2;
        const hz = (dims.depth ?? 0.04) / 2;
        colliderDesc = RAPIER.ColliderDesc.cuboid(hx, hy, hz);
      }
    } else if (spec.shape === 'cylinder') {
      const r = dims.radius ?? (dims.width ?? 0.03) / 2;
      const halfH = (dims.height ?? 0.06) / 2;
      colliderDesc = RAPIER.ColliderDesc.cylinder(halfH, r);
    } else if (spec.shape === 'sphere') {
      const r = dims.radius ?? (dims.width ?? 0.03) / 2;
      colliderDesc = RAPIER.ColliderDesc.ball(r);
    } else if (spec.shape === 'capsule') {
      const r = dims.radius ?? 0.02;
      const halfH = Math.max(((dims.height ?? 0.08) - 2 * r) / 2, 0.005);
      colliderDesc = RAPIER.ColliderDesc.capsule(halfH, r);
    } else {
      const hx = (dims.width ?? 0.04) / 2;
      const hy = (dims.height ?? 0.04) / 2;
      const hz = (dims.depth ?? 0.04) / 2;
      colliderDesc = RAPIER.ColliderDesc.cuboid(hx, hy, hz);
    }

    if (spec.isSensor || spec.category === 'sensor') {
      colliderDesc.setSensor(true);
    } else if (spec.motionType === 'dynamic') {
      const safeMass = Math.max(spec.massKg, 0.005);
      if (typeof colliderDesc.setMass === 'function') {
        colliderDesc.setMass(safeMass);
      } else {
        const com = spec.centerOfMass ?? [0, 0, 0];
        colliderDesc.setMassProperties(
          safeMass,
          { x: com[0], y: com[1], z: com[2] },
          { x: principalInertia[0], y: principalInertia[1], z: principalInertia[2] },
          { x: 0, y: 0, z: 0, w: 1 }
        );
      }
    }

    const groups = resolveDefaultCollisionGroups(spec);
    const encodedGroups = encodeCollisionGroups(groups.membership, groups.filter);

    colliderDesc
      .setFriction(Math.max(0, spec.friction))
      .setRestitution(Math.min(Math.max(spec.restitution, 0), 1))
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Average)
      .setRestitutionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setCollisionGroups(encodedGroups)
      .setSolverGroups(encodedGroups)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS)
      .setActiveCollisionTypes(
        RAPIER.ActiveCollisionTypes.DEFAULT |
          RAPIER.ActiveCollisionTypes.KINEMATIC_FIXED
      );

    if (typeof colliderDesc.setContactSkin === 'function') {
      colliderDesc.setContactSkin(spec.contactSkin ?? 0.0005);
    }

    const collider = this.physicsWorld.world.createCollider(colliderDesc, body);

    const managed: ManagedRigidBody = {
      spec: {
        ...spec,
        collisionGroupMembership: groups.membership,
        collisionGroupFilter: groups.filter
      },
      body,
      collider,
      mesh: spec.mesh,
      principalInertia,
      encodedCollisionGroups: encodedGroups,
      lastAppliedForceNewtons: [0, 0, 0],
      blockedStepsCount: 0
    };

    this.bodies.set(spec.id, managed);
    this.colliderHandleToId.set(collider.handle, spec.id);
    return managed;
  }

  /**
   * Removes a rigid body and its collider from the Rapier world.
   */
  public removeBody(id: string): boolean {
    const managed = this.bodies.get(id);
    if (!managed) return false;

    this.colliderHandleToId.delete(managed.collider.handle);
    this.physicsWorld.world.removeRigidBody(managed.body);
    this.bodies.delete(id);
    return true;
  }

  /**
   * Retrieves a managed rigid body by ID.
   */
  public getBody(id: string): ManagedRigidBody | undefined {
    return this.bodies.get(id);
  }

  /**
   * Updates the mass and recomputes the inertia tensor of a rigid body at runtime.
   */
  public setBodyMass(id: string, newMassKg: number): void {
    const managed = this.bodies.get(id);
    if (!managed) return;

    const clampedMass = Math.max(newMassKg, 0.005);
    managed.spec.massKg = clampedMass;
    const inertia = computePrincipalInertia(
      managed.spec.shape,
      clampedMass,
      managed.spec.dimensions
    );
    managed.principalInertia = inertia;

    if (managed.body.isDynamic()) {
      if (typeof managed.collider.setMass === 'function') {
        managed.collider.setMass(clampedMass);
      } else {
        const com = managed.spec.centerOfMass ?? [0, 0, 0];
        managed.body.setAdditionalMassProperties(
          clampedMass,
          { x: com[0], y: com[1], z: com[2] },
          { x: inertia[0], y: inertia[1], z: inertia[2] },
          { x: 0, y: 0, z: 0, w: 1 },
          true
        );
      }
      managed.body.wakeUp();
    }
  }

  /**
   * Updates friction and restitution coefficients of a body's collider at runtime.
   */
  public setBodyMaterialProperties(
    id: string,
    friction: number,
    restitution: number
  ): void {
    const managed = this.bodies.get(id);
    if (!managed) return;

    managed.spec.friction = Math.max(0, friction);
    managed.spec.restitution = Math.min(Math.max(restitution, 0), 1);
    managed.collider.setFriction(managed.spec.friction);
    managed.collider.setRestitution(managed.spec.restitution);
    managed.body.wakeUp();
  }

  /**
   * Updates collision and solver group masks of an existing body at runtime.
   */
  public setBodyCollisionGroups(
    id: string,
    membership: number,
    filter: number
  ): void {
    const managed = this.bodies.get(id);
    if (!managed) return;
    managed.spec.collisionGroupMembership = membership;
    managed.spec.collisionGroupFilter = filter;
    const encoded = encodeCollisionGroups(membership, filter);
    managed.encodedCollisionGroups = encoded;
    managed.collider.setCollisionGroups(encoded);
    managed.collider.setSolverGroups(encoded);
    managed.body.wakeUp();
  }

  /**
   * Applies an external force vector (in Newtons) to a dynamic rigid body.
   * Wakes sleeping bodies immediately so applied forces always take effect.
   */
  public applyExternalForce(
    id: string,
    forceNewtons: [number, number, number],
    resetBeforeAdd: boolean = true
  ): void {
    const managed = this.bodies.get(id);
    if (!managed || !managed.body.isDynamic()) return;
    managed.body.wakeUp();
    if (resetBeforeAdd) {
      managed.body.resetForces(true);
    }
    managed.body.addForce(
      { x: forceNewtons[0], y: forceNewtons[1], z: forceNewtons[2] },
      true
    );
    managed.lastAppliedForceNewtons = [...forceNewtons];
  }

  /**
   * Applies an instantaneous linear impulse (in N·s) to a dynamic rigid body.
   */
  public applyImpulse(id: string, impulseNs: [number, number, number]): void {
    const managed = this.bodies.get(id);
    if (!managed || !managed.body.isDynamic()) return;
    managed.body.wakeUp();
    managed.body.applyImpulse(
      { x: impulseNs[0], y: impulseNs[1], z: impulseNs[2] },
      true
    );
  }

  public applyExternalImpulse(id: string, impulseNs: [number, number, number]): void {
    this.applyImpulse(id, impulseNs);
  }

  /**
   * Repositions and reorients a rigid body, zeroing its linear and angular velocities.
   */
  public setBodyPose(
    id: string,
    position: [number, number, number],
    quaternion: [number, number, number, number] = [0, 0, 0, 1]
  ): void {
    const managed = this.bodies.get(id);
    if (!managed) return;

    managed.spec.position = [...position];
    managed.spec.quaternion = [...quaternion];

    managed.body.setTranslation(
      { x: position[0], y: position[1], z: position[2] },
      true
    );
    managed.body.setRotation(
      { x: quaternion[0], y: quaternion[1], z: quaternion[2], w: quaternion[3] },
      true
    );
    if (managed.body.isDynamic()) {
      managed.body.resetForces(true);
      managed.body.resetTorques(true);
      managed.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      managed.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      managed.lastAppliedForceNewtons = [0, 0, 0];
    }
    this.syncBodyToMesh(managed);
  }

  /**
   * Synchronizes a single Rapier RigidBody world transform to its paired Three.js mesh.
   */
  public syncBodyToMesh(managed: ManagedRigidBody): void {
    if (!managed.mesh) return;
    const t = managed.body.translation();
    const r = managed.body.rotation();
    managed.mesh.position.set(t.x, t.y, t.z);
    managed.mesh.quaternion.set(r.x, r.y, r.z, r.w);
    managed.mesh.updateMatrixWorld(true);
  }

  /**
   * Synchronizes all registered dynamic and kinematic bodies to their Three.js meshes.
   */
  public syncAllMeshes(): void {
    for (const [, managed] of this.bodies) {
      this.syncBodyToMesh(managed);
    }
  }

  public syncVisualMeshes(): void {
    this.syncAllMeshes();
  }
}
