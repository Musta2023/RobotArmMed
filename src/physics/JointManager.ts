/**
 * @file JointManager.ts
 * Manages physically simulated Rapier3D RevoluteImpulseJoints, PD motor controllers,
 * gravity + payload torque dynamics, and link rigid-body synchronization.
 */

import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { PhysicsWorld } from './PhysicsWorld.js';
import { ManagedRigidBody } from './RigidBodyManager.js';

export interface RevoluteJointSpec {
  id: string;
  name: string;
  index: number;
  parentAnchor: [number, number, number];
  childAnchor: [number, number, number];
  axis: [number, number, number];
  minAngleRad: number;
  maxAngleRad: number;
  maxVelocityRadPerSec: number;
  maxAccelerationRadPerSec2: number;
  maxMotorTorqueNm: number;
  stiffness: number;
  damping: number;
  linkMassKg: number;
  linkLengthMeters: number;
}

export interface ManagedRevoluteJoint {
  spec: RevoluteJointSpec;
  rapierJoint?: RAPIER.RevoluteImpulseJoint;
  currentAngleRad: number;
  targetAngleRad: number;
  angularVelocityRadPerSec: number;
  targetVelocityRadPerSec: number;
  motorTorqueNm: number;
  gravityLoadTorqueNm: number;
  isAtLimit: boolean;
  isTorqueSaturated: boolean;
  controlMode: 'position' | 'velocity' | 'torque';
}

export class JointManager {
  public physicsWorld: PhysicsWorld;
  public joints: Map<string, ManagedRevoluteJoint> = new Map();
  public orderedJoints: ManagedRevoluteJoint[] = [];

  public globalStiffnessScale: number = 1.0;
  public globalDampingScale: number = 1.0;
  public globalMaxTorqueScale: number = 1.0;

  constructor(physicsWorld: PhysicsWorld) {
    this.physicsWorld = physicsWorld;
  }

  /**
   * Updates global motor stiffness, damping, and maximum torque scaling factors.
   */
  public setMotorTuningScales(
    stiffnessScale: number,
    dampingScale: number,
    maxTorqueScale: number
  ): void {
    this.globalStiffnessScale = Math.max(0.05, stiffnessScale);
    this.globalDampingScale = Math.max(0.05, dampingScale);
    this.globalMaxTorqueScale = Math.max(0.05, maxTorqueScale);
  }

  /**
   * Creates a physically simulated revolute joint between two Rapier RigidBodies
   * and initializes its motorized PD + gravity/payload dynamics state.
   */
  public createRevoluteJoint(
    spec: RevoluteJointSpec,
    parentBody?: ManagedRigidBody,
    childBody?: ManagedRigidBody,
    initialAngleRad: number = 0
  ): ManagedRevoluteJoint {
    let rapierJoint: RAPIER.RevoluteImpulseJoint | undefined;

    if (parentBody && childBody && this.physicsWorld.isInitialized) {
      const jointData = RAPIER.JointData.revolute(
        { x: spec.parentAnchor[0], y: spec.parentAnchor[1], z: spec.parentAnchor[2] },
        { x: spec.childAnchor[0], y: spec.childAnchor[1], z: spec.childAnchor[2] },
        { x: spec.axis[0], y: spec.axis[1], z: spec.axis[2] }
      );
      jointData.limitsEnabled = true;
      jointData.limits = [spec.minAngleRad, spec.maxAngleRad];

      const joint = this.physicsWorld.world.createImpulseJoint(
        jointData,
        parentBody.body,
        childBody.body,
        true
      ) as RAPIER.RevoluteImpulseJoint;

      joint.setContactsEnabled(false);
      joint.configureMotorPosition(initialAngleRad, spec.stiffness, spec.damping);
      rapierJoint = joint;
    }

    const clampedInit = Math.min(
      Math.max(initialAngleRad, spec.minAngleRad),
      spec.maxAngleRad
    );

    const managed: ManagedRevoluteJoint = {
      spec: { ...spec },
      rapierJoint,
      currentAngleRad: clampedInit,
      targetAngleRad: clampedInit,
      angularVelocityRadPerSec: 0,
      targetVelocityRadPerSec: 0,
      motorTorqueNm: 0,
      gravityLoadTorqueNm: 0,
      isAtLimit: false,
      isTorqueSaturated: false,
      controlMode: 'position'
    };

    this.joints.set(spec.id, managed);
    this.orderedJoints[spec.index] = managed;
    return managed;
  }

  /**
   * Steps all joint motors for time increment `dt` (seconds) under:
   * - Motor stiffness (Kp) and damping (Kd)
   * - Gravity and held payload moment-arm torque
   * - Maximum motor torque saturation (`maxMotorTorqueNm`)
   * - Maximum angular velocity & acceleration bounds
   * - Mechanical joint limits `[minAngleRad, maxAngleRad]`
   */
  public stepMotors(
    dt: number,
    jointWorldPivots: THREE.Vector3[],
    jointWorldAxes: THREE.Vector3[],
    linkWorldCOMs: THREE.Vector3[],
    tcpWorldPos: THREE.Vector3,
    payloadMassKg: number = 0
  ): void {
    const gravityVec = new THREE.Vector3(0, this.physicsWorld.gravityY, 0);

    for (let i = 0; i < this.orderedJoints.length; i++) {
      const j = this.orderedJoints[i];
      if (!j) continue;

      const spec = j.spec;
      const kp = spec.stiffness * this.globalStiffnessScale;
      const kd = spec.damping * this.globalDampingScale;
      const maxTorque = spec.maxMotorTorqueNm * this.globalMaxTorqueScale;

      // 1. Compute gravitational + payload moment torque around joint i's world axis
      const pivotPos = jointWorldPivots[i] || new THREE.Vector3();
      const worldAxis = jointWorldAxes[i] || new THREE.Vector3(0, 1, 0);

      let gravityMomentNm = 0;

      // Downstream robot links jIdx >= i
      for (let jIdx = i; jIdx < this.orderedJoints.length; jIdx++) {
        const downstreamJoint = this.orderedJoints[jIdx];
        const comPos = linkWorldCOMs[jIdx];
        if (!downstreamJoint || !comPos) continue;

        const rArm = new THREE.Vector3().subVectors(comPos, pivotPos);
        const weightForce = gravityVec.clone().multiplyScalar(downstreamJoint.spec.linkMassKg);
        const torqueVec = new THREE.Vector3().crossVectors(rArm, weightForce);
        gravityMomentNm += torqueVec.dot(worldAxis);
      }

      // Grasped payload contribution at TCP
      if (payloadMassKg > 0) {
        const rPayload = new THREE.Vector3().subVectors(tcpWorldPos, pivotPos);
        const payloadWeight = gravityVec.clone().multiplyScalar(payloadMassKg);
        const payloadTorqueVec = new THREE.Vector3().crossVectors(rPayload, payloadWeight);
        gravityMomentNm += payloadTorqueVec.dot(worldAxis);
      }

      j.gravityLoadTorqueNm = gravityMomentNm;

      // 2. Compute discrete-time stable cascaded PD control torque
      const effectiveInertia = Math.max(
        0.05,
        spec.linkMassKg * spec.linkLengthMeters * spec.linkLengthMeters +
          payloadMassKg * 0.12
      );

      let desiredVel = 0;
      if (j.controlMode === 'velocity') {
        desiredVel = Math.min(
          Math.max(j.targetVelocityRadPerSec, -spec.maxVelocityRadPerSec),
          spec.maxVelocityRadPerSec
        );
      } else {
        const clampedTarget = Math.min(
          Math.max(j.targetAngleRad, spec.minAngleRad),
          spec.maxAngleRad
        );
        j.targetAngleRad = clampedTarget;
        const posError = clampedTarget - j.currentAngleRad;
        const posGain = 7.0 * this.globalStiffnessScale;
        desiredVel = Math.min(
          Math.max(posError * posGain, -spec.maxVelocityRadPerSec),
          spec.maxVelocityRadPerSec
        );
      }

      const velTimeConstant = Math.max(dt * 2.5, 0.045 / this.globalDampingScale);
      const desiredAccel = Math.min(
        Math.max(
          (desiredVel - j.angularVelocityRadPerSec) / velTimeConstant,
          -spec.maxAccelerationRadPerSec2
        ),
        spec.maxAccelerationRadPerSec2
      );

      const pdControlTorque = effectiveInertia * desiredAccel;

      // Total motor torque must overcome gravity load AND provide PD acceleration
      const requiredMotorTorque = pdControlTorque - gravityMomentNm;
      const appliedMotorTorque = Math.min(
        Math.max(requiredMotorTorque, -maxTorque),
        maxTorque
      );

      j.motorTorqueNm = appliedMotorTorque;
      // Motor is overloaded (torque saturated) when gravitational/payload load exceeds maxTorque
      j.isTorqueSaturated = Math.abs(gravityMomentNm) > maxTorque;

      // Net physical torque acting on the joint: appliedMotorTorque + gravityMomentNm
      const netJointTorque = appliedMotorTorque + gravityMomentNm;
      const rawAngAccel = netJointTorque / effectiveInertia;
      const clampedAngAccel = Math.min(
        Math.max(rawAngAccel, -spec.maxAccelerationRadPerSec2),
        spec.maxAccelerationRadPerSec2
      );

      let nextVel = j.angularVelocityRadPerSec + clampedAngAccel * dt;
      nextVel = Math.min(
        Math.max(nextVel, -spec.maxVelocityRadPerSec),
        spec.maxVelocityRadPerSec
      );

      // Snap smoothly when very close to target in position mode and not torque-starved
      if (
        j.controlMode === 'position' &&
        !j.isTorqueSaturated &&
        Math.abs(j.targetAngleRad - j.currentAngleRad) < 1e-3 &&
        Math.abs(nextVel) < 0.05
      ) {
        j.currentAngleRad = j.targetAngleRad;
        nextVel = 0;
      } else {
        j.currentAngleRad += nextVel * dt;
      }

      // Enforce hard mechanical joint limits
      if (j.currentAngleRad <= spec.minAngleRad) {
        j.currentAngleRad = spec.minAngleRad;
        if (nextVel < 0) nextVel = 0;
        j.isAtLimit = true;
      } else if (j.currentAngleRad >= spec.maxAngleRad) {
        j.currentAngleRad = spec.maxAngleRad;
        if (nextVel > 0) nextVel = 0;
        j.isAtLimit = true;
      } else {
        j.isAtLimit = false;
      }

      j.angularVelocityRadPerSec = nextVel;

      if (j.rapierJoint) {
        j.rapierJoint.configureMotorPosition(j.targetAngleRad, kp, kd);
      }
    }
  }
}
