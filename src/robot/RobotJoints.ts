/**
 * @file RobotJoints.ts
 * Manages the physical Rapier3D rigid bodies and motorized RevoluteImpulseJoints
 * for the articulated robot links (Base + Links 1..N).
 */

import * as THREE from 'three';
import {
  RobotSystemConfiguration,
  DEFAULT_ROBOT_CONFIGURATION
} from './RobotConfiguration.js';
import {
  RigidBodyManager,
  ManagedRigidBody
} from '../physics/RigidBodyManager.js';
import {
  JointManager,
  ManagedRevoluteJoint
} from '../physics/JointManager.js';

export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;

export class RobotJoints {
  public config: RobotSystemConfiguration;
  public bodyManager: RigidBodyManager;
  public jointManager: JointManager;

  public baseBody!: ManagedRigidBody;
  public linkBodies: ManagedRigidBody[] = [];
  public joints: ManagedRevoluteJoint[] = [];

  constructor(
    bodyManager: RigidBodyManager,
    jointManager: JointManager,
    config: RobotSystemConfiguration = DEFAULT_ROBOT_CONFIGURATION
  ) {
    this.config = config;
    this.bodyManager = bodyManager;
    this.jointManager = jointManager;

    this._createPhysicalBaseAndLinks();
  }

  private _createPhysicalBaseAndLinks(): void {
    // 1. Fixed Robot Base RigidBody
    this.baseBody = this.bodyManager.createBody({
      id: 'robot_base',
      name: 'Robot Base Assembly',
      category: 'robot_link',
      motionType: 'fixed',
      shape: 'cylinder',
      dimensions: {
        radius: 0.10,
        height: this.config.baseHeightMeters
      },
      position: [
        this.config.basePosition[0],
        this.config.baseHeightMeters / 2,
        this.config.basePosition[2]
      ],
      massKg: this.config.baseMassKg,
      friction: 0.6,
      restitution: 0.05,
      materialType: 'anodized_aluminum'
    });

    // 2. Create separate RigidBodies for each articulated link and connect via RevoluteJoints
    let parentBody = this.baseBody;

    for (let i = 0; i < this.config.joints.length; i++) {
      const jCfg = this.config.joints[i];
      const initRad = jCfg.homeAngleDeg * DEG2RAD;

      const linkBody = this.bodyManager.createBody({
        id: `robot_link_${i}`,
        name: `${jCfg.id} Link (${jCfg.shortName})`,
        category: 'robot_link',
        motionType: 'kinematicPosition',
        shape: 'box',
        dimensions: {
          width: jCfg.colliderDimensions[0],
          height: jCfg.colliderDimensions[1],
          depth: jCfg.colliderDimensions[2]
        },
        position: [0, 0.2 + i * 0.12, 0],
        massKg: jCfg.linkMassKg,
        centerOfMass: jCfg.centerOfMassLocal,
        friction: 0.5,
        restitution: 0.05,
        materialType: 'anodized_aluminum'
      });

      this.linkBodies.push(linkBody);

      const managedJoint = this.jointManager.createRevoluteJoint(
        {
          id: jCfg.id,
          name: jCfg.name,
          index: i,
          parentAnchor: jCfg.pivotOffset,
          childAnchor: [0, 0, 0],
          axis: jCfg.axis,
          minAngleRad: jCfg.minAngleDeg * DEG2RAD,
          maxAngleRad: jCfg.maxAngleDeg * DEG2RAD,
          maxVelocityRadPerSec: jCfg.maxVelocityRadPerSec,
          maxAccelerationRadPerSec2: jCfg.maxAccelerationRadPerSec2,
          maxMotorTorqueNm: jCfg.maxMotorTorqueNm,
          stiffness: jCfg.stiffness,
          damping: jCfg.damping,
          linkMassKg: jCfg.linkMassKg,
          linkLengthMeters: jCfg.linkLengthMeters
        },
        parentBody,
        linkBody,
        initRad
      );

      this.joints.push(managedJoint);
      parentBody = linkBody;
    }
  }

  /**
   * Clamps a requested joint angle in degrees to that joint's configured limits.
   */
  public clampJointAngleDeg(jointIndex: number, angleDeg: number): number {
    const jCfg = this.config.joints[jointIndex];
    if (!jCfg) return 0;
    const safe = Number.isFinite(angleDeg) ? angleDeg : jCfg.homeAngleDeg;
    return Math.min(Math.max(safe, jCfg.minAngleDeg), jCfg.maxAngleDeg);
  }

  /**
   * Sets the motor target angle (in degrees) for a single joint.
   * The physical motor will drive the joint toward this target during physics stepping.
   */
  public setTargetJointAngleDeg(jointIndex: number, angleDeg: number): number {
    const j = this.joints[jointIndex];
    if (!j) return 0;
    const clampedDeg = this.clampJointAngleDeg(jointIndex, angleDeg);
    j.controlMode = 'position';
    j.targetAngleRad = clampedDeg * DEG2RAD;
    return clampedDeg;
  }

  /**
   * Sets the motor target angles (in degrees) for all joints.
   */
  public setTargetJointAnglesDeg(anglesDeg: number[]): number[] {
    return this.joints.map((_, idx) =>
      this.setTargetJointAngleDeg(idx, anglesDeg[idx] ?? 0)
    );
  }

  /**
   * Sets target angular velocity (rad/s) for velocity control mode on a joint.
   */
  public setTargetJointVelocityRadPerSec(jointIndex: number, velocityRadPerSec: number): void {
    const j = this.joints[jointIndex];
    if (!j) return;
    j.controlMode = 'velocity';
    j.targetVelocityRadPerSec = Math.min(
      Math.max(velocityRadPerSec, -j.spec.maxVelocityRadPerSec),
      j.spec.maxVelocityRadPerSec
    );
  }

  /**
   * Returns current physical joint angles in degrees.
   */
  public getCurrentAnglesDeg(): number[] {
    return this.joints.map((j) => j.currentAngleRad * RAD2DEG);
  }

  /**
   * Returns current physical joint angles in radians.
   */
  public getCurrentAnglesRad(): number[] {
    return this.joints.map((j) => j.currentAngleRad);
  }

  /**
   * Returns target joint angles in degrees.
   */
  public getTargetAnglesDeg(): number[] {
    return this.joints.map((j) => j.targetAngleRad * RAD2DEG);
  }

  /**
   * Resets all joints to their configured Home pose.
   */
  public resetToHomePose(immediate: boolean = false): void {
    for (let i = 0; i < this.joints.length; i++) {
      const homeRad = this.config.joints[i].homeAngleDeg * DEG2RAD;
      const j = this.joints[i];
      j.controlMode = 'position';
      j.targetAngleRad = homeRad;
      j.targetVelocityRadPerSec = 0;
      if (immediate) {
        j.currentAngleRad = homeRad;
        j.angularVelocityRadPerSec = 0;
        j.motorTorqueNm = 0;
        j.isTorqueSaturated = false;
      }
    }
  }

  /**
   * Updates all link Rapier RigidBodies to match the current physical joint transforms
   * so Rapier colliders accurately interact with instruments and obstacles in the scene.
   *
   * When `immediate` is false (during physics stepping), uses `setNextKinematicTranslation`
   * and `setNextKinematicRotation` so Rapier3D computes exact kinematic velocities for
   * realistic momentum transfer when pushing or contacting dynamic objects.
   */
  public syncLinkBodiesToTransforms(
    linkWorldCenters: THREE.Vector3[],
    linkWorldQuaternions: THREE.Quaternion[],
    immediate: boolean = false
  ): void {
    for (let i = 0; i < this.linkBodies.length; i++) {
      const managed = this.linkBodies[i];
      const pos = linkWorldCenters[i];
      const quat = linkWorldQuaternions[i];
      if (!managed || !pos || !quat) continue;

      managed.kinematicTargetPos = [pos.x, pos.y, pos.z];
      managed.kinematicTargetQuat = [quat.x, quat.y, quat.z, quat.w];
      managed.body.setNextKinematicTranslation({ x: pos.x, y: pos.y, z: pos.z });
      managed.body.setNextKinematicRotation({
        x: quat.x,
        y: quat.y,
        z: quat.z,
        w: quat.w
      });
      if (immediate) {
        managed.body.setTranslation({ x: pos.x, y: pos.y, z: pos.z }, true);
        managed.body.setRotation(
          { x: quat.x, y: quat.y, z: quat.z, w: quat.w },
          true
        );
      }
    }
  }
}
