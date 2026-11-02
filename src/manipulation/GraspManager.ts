/**
 * @file GraspManager.ts
 * Physics-based Coulomb friction grasping, Rapier FixedImpulseJoint constraint management,
 * and slip dynamics manager (Phase 5).
 *
 * Rules enforced:
 * - Closing the gripper never attaches an object automatically.
 * - Both left and right fingers must establish physical contact on opposite sides
 *   of the instrument (`bilateralContact === true`).
 * - The available friction force F_friction_max = 2 * mu_eff * F_normal must exceed
 *   the gravitational + inertial load F_load = m * |g|.
 * - When a stable physical grasp is achieved, a Rapier FixedImpulseJoint is created
 *   between the gripper finger body and the dynamic object body, and removed upon release
 *   or slip.
 * - If 0 < F_friction_max < F_load, the instrument physically slips downward through
 *   the fingers under gravity!
 */

import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { Gripper } from '../robot/Gripper.js';
import { ContactDetector, FingerObjectContactResult } from './ContactDetector.js';
import { PhysicsWorld } from '../physics/PhysicsWorld.js';
import { computeEffectiveFriction } from '../physics/MaterialProperties.js';
import { ActiveSurgicalInstrument } from '../objects/InstrumentFactory.js';

export class GraspManager {
  public gripper: Gripper;
  public contactDetector: ContactDetector;
  public physicsWorld: PhysicsWorld;

  public isGrasping: boolean = false;
  public isSlipping: boolean = false;
  public graspedInstrument: ActiveSurgicalInstrument | null = null;
  public activeGraspJoint: RAPIER.ImpulseJoint | null = null;
  public frictionCapacityNewtons: number = 0;
  public requiredHoldForceNewtons: number = 0;
  public lastContactResult: FingerObjectContactResult | null = null;

  public get hasPhysicalJoint(): boolean {
    return this.activeGraspJoint !== null;
  }

  public get graspedInstrumentId(): string | null {
    return this.graspedInstrument ? this.graspedInstrument.config.id : null;
  }

  private relativeGraspMatrix: THREE.Matrix4 | null = null;

  constructor(
    gripper: Gripper,
    contactDetector: ContactDetector,
    physicsWorld: PhysicsWorld
  ) {
    this.gripper = gripper;
    this.contactDetector = contactDetector;
    this.physicsWorld = physicsWorld;
  }

  private _removeGraspJoint(): void {
    if (this.activeGraspJoint && this.physicsWorld.isInitialized) {
      try {
        this.physicsWorld.world.removeImpulseJoint(this.activeGraspJoint, true);
      } catch {
        // Joint may already have been removed if body was destroyed
      }
    }
    this.activeGraspJoint = null;
  }

  /**
   * Steps the gripper finger motors, evaluates bilateral contact and Coulomb friction
   * stability, manages the physical Rapier FixedImpulseJoint for grasped objects,
   * and applies slip dynamics when friction is insufficient.
   */
  public stepGraspPhysics(dt: number): {
    isGrasping: boolean;
    isSlipping: boolean;
    graspedInstrumentId: string | null;
    payloadMassKg: number;
    leftFingerContact: boolean;
    rightFingerContact: boolean;
    frictionCapacityNewtons: number;
  } {
    const prevObjPos =
      this.graspedInstrument
        ? new THREE.Vector3(
            this.graspedInstrument.managedBody.body.translation().x,
            this.graspedInstrument.managedBody.body.translation().y,
            this.graspedInstrument.managedBody.body.translation().z
          )
        : null;

    // Co-move already-grasped object into the current substep's grasp frame prior to contact check
    if (
      this.isGrasping &&
      this.graspedInstrument &&
      this.relativeGraspMatrix &&
      this.gripper.isClosingCommandActive
    ) {
      const graspPose = this.gripper.getGraspFrameWorldPose();
      const targetWorldMat = graspPose.matrix.clone().multiply(this.relativeGraspMatrix);
      const targetPos = new THREE.Vector3();
      const targetQuat = new THREE.Quaternion();
      targetWorldMat.decompose(targetPos, targetQuat, new THREE.Vector3());

      const rBody = this.graspedInstrument.managedBody.body;
      rBody.setTranslation({ x: targetPos.x, y: targetPos.y, z: targetPos.z }, true);
      rBody.setRotation(
        { x: targetQuat.x, y: targetQuat.y, z: targetQuat.z, w: targetQuat.w },
        true
      );
    }

    // 1. Detect if an instrument is currently between the fingers
    const preContact = this.contactDetector.detectFingerContacts();

    // If an instrument is between the jaws and the gripper is trying to close narrower than the object,
    // the solid object physically blocks the fingers at `objectGraspWidthMeters`.
    const stopWidth =
      preContact.isBetweenFingers && preContact.instrument
        ? preContact.objectGraspWidthMeters
        : undefined;

    this.gripper.stepMotorAndSyncBodies(dt, stopWidth);

    // Re-evaluate contact after finger step
    const contact = this.contactDetector.detectFingerContacts();
    this.lastContactResult = contact;

    if (!contact.instrument || !contact.bilateralContact || !this.gripper.isClosingCommandActive) {
      // No bilateral contact or gripper is opening -> remove grasp constraint & release object
      this._removeGraspJoint();
      if (this.graspedInstrument) {
        const releasedBody = this.graspedInstrument.managedBody.body;
        releasedBody.resetForces(true);
        releasedBody.wakeUp();
      }
      this.isGrasping = false;
      this.isSlipping = false;
      this.graspedInstrument = null;
      this.relativeGraspMatrix = null;
      this.frictionCapacityNewtons = 0;
      this.requiredHoldForceNewtons = 0;

      return {
        isGrasping: false,
        isSlipping: false,
        graspedInstrumentId: null,
        payloadMassKg: 0,
        leftFingerContact: contact.leftFingerContact,
        rightFingerContact: contact.rightFingerContact,
        frictionCapacityNewtons: 0
      };
    }

    const inst = contact.instrument;
    const muEff = computeEffectiveFriction(
      this.gripper.config.fingerFriction,
      inst.config.friction,
      'rubber',
      inst.config.materialType
    );

    const normalForceN = this.gripper.appliedNormalForceNewtons;
    // Two opposing fingers each contribute mu_eff * F_normal static friction capacity
    const maxFrictionForceN = 2.0 * muEff * normalForceN;
    const gravityAccel = Math.abs(this.physicsWorld.gravityY);
    const weightLoadN = inst.config.massKg * gravityAccel;

    this.frictionCapacityNewtons = maxFrictionForceN;
    this.requiredHoldForceNewtons = weightLoadN;

    const graspPose = this.gripper.getGraspFrameWorldPose();
    const rBody = inst.managedBody.body;

    // Condition for stable friction grasp: F_friction_max >= 1.05 * weightLoadN
    if (maxFrictionForceN >= weightLoadN * 1.05 && normalForceN > 0.1) {
      if (!this.isGrasping || this.graspedInstrument !== inst || !this.relativeGraspMatrix) {
        this._removeGraspJoint();

        // Record initial physical contact offset in GraspFrame coordinates
        const t = rBody.translation();
        const r = rBody.rotation();
        const objWorldMat = new THREE.Matrix4().compose(
          new THREE.Vector3(t.x, t.y, t.z),
          new THREE.Quaternion(r.x, r.y, r.z, r.w),
          new THREE.Vector3(1, 1, 1)
        );
        const invGrasp = graspPose.matrix.clone().invert();
        this.relativeGraspMatrix = invGrasp.multiply(objWorldMat);

        // Center offset between the symmetric fingers in GraspFrame coordinates
        const relPos = new THREE.Vector3();
        const relQuat = new THREE.Quaternion();
        this.relativeGraspMatrix.decompose(relPos, relQuat, new THREE.Vector3());
        relPos.set(0, 0, 0);
        this.relativeGraspMatrix.compose(relPos, relQuat, new THREE.Vector3(1, 1, 1));

        // Create physical Rapier FixedImpulseJoint between left finger body and grasped dynamic object
        if (this.physicsWorld.isInitialized) {
          const halfGap = this.gripper.currentOpeningMeters / 2;
          const fixedData = RAPIER.JointData.fixed(
            { x: halfGap, y: 0, z: 0 },
            { x: 0, y: 0, z: 0, w: 1 },
            { x: 0, y: 0, z: 0 },
            { x: relQuat.x, y: relQuat.y, z: relQuat.z, w: relQuat.w }
          );
          this.activeGraspJoint = this.physicsWorld.world.createImpulseJoint(
            fixedData,
            this.gripper.leftFingerBody.body,
            rBody,
            true
          );
        }
      }

      this.isGrasping = true;
      this.isSlipping = false;
      this.graspedInstrument = inst;

      // Update dynamic body pose & velocity to move with the physical finger contact patch
      const targetWorldMat = graspPose.matrix.clone().multiply(this.relativeGraspMatrix);
      const targetPos = new THREE.Vector3();
      const targetQuat = new THREE.Quaternion();
      targetWorldMat.decompose(targetPos, targetQuat, new THREE.Vector3());

      const refPrev = prevObjPos ?? targetPos;
      const vx = (targetPos.x - refPrev.x) / Math.max(dt, 1e-4);
      const vy = (targetPos.y - refPrev.y) / Math.max(dt, 1e-4);
      const vz = (targetPos.z - refPrev.z) / Math.max(dt, 1e-4);

      rBody.setTranslation({ x: targetPos.x, y: targetPos.y, z: targetPos.z }, true);
      rBody.setRotation(
        { x: targetQuat.x, y: targetQuat.y, z: targetQuat.z, w: targetQuat.w },
        true
      );
      rBody.setLinvel({ x: vx, y: vy, z: vz }, true);
      rBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
      inst.managedBody.mesh?.position.copy(targetPos);
      inst.managedBody.mesh?.quaternion.copy(targetQuat);

      return {
        isGrasping: true,
        isSlipping: false,
        graspedInstrumentId: inst.config.id,
        payloadMassKg: inst.config.massKg,
        leftFingerContact: true,
        rightFingerContact: true,
        frictionCapacityNewtons: maxFrictionForceN
      };
    }

    // Otherwise: 0 <= F_friction_max < 1.05 * weightLoadN -> OBJECT SLIPS UNDER GRAVITY!
    this._removeGraspJoint();
    this.isGrasping = false;
    this.isSlipping = true;
    this.graspedInstrument = null;
    this.relativeGraspMatrix = null;

    rBody.wakeUp();
    rBody.resetForces(true);
    // Apply partial upward kinetic friction force (< weightLoadN), so net acceleration is downward!
    if (maxFrictionForceN > 0) {
      rBody.addForce({ x: 0, y: maxFrictionForceN, z: 0 }, true);
    }

    return {
      isGrasping: false,
      isSlipping: true,
      graspedInstrumentId: null,
      payloadMassKg: 0,
      leftFingerContact: contact.leftFingerContact,
      rightFingerContact: contact.rightFingerContact,
      frictionCapacityNewtons: maxFrictionForceN
    };
  }

  /**
   * Immediately removes any active grasp joint and wakes the dynamic instrument body
   * so it falls and settles naturally under gravity.
   */
  public releaseGrasp(): void {
    this._removeGraspJoint();
    if (this.graspedInstrument) {
      const body = this.graspedInstrument.managedBody.body;
      body.resetForces(true);
      body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      body.wakeUp();
    }
    this.isGrasping = false;
    this.isSlipping = false;
    this.graspedInstrument = null;
    this.relativeGraspMatrix = null;
    this.frictionCapacityNewtons = 0;
    this.requiredHoldForceNewtons = 0;
    this.gripper.open(true);
  }
}
