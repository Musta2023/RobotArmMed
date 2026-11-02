/**
 * @file MotionController.ts
 * High-level motion planner and physical trajectory executor for the Digital Twin.
 * Computes IK joint targets and feeds interpolated setpoints to the physical
 * JointController motors while monitoring collisions and torque limits.
 */

import * as THREE from 'three';
import { JointController } from './JointController.js';
import {
  solveInverseKinematics,
  IKSolverResult,
  createTopDownQuaternion
} from './InverseKinematics.js';
import { CollisionManager } from '../physics/CollisionManager.js';
import { clampAnglesDegToConfig } from './ForwardKinematics.js';

export class MotionController {
  public jointController: JointController;
  public collisionManager: CollisionManager;

  public lastSafeAnglesDeg: number[];
  public finalTargetAnglesDeg: number[];
  public lastIKResult: IKSolverResult | null = null;
  public isBlockedByCollision: boolean = false;
  public lastBlockReason: string = '';

  constructor(
    jointController: JointController,
    collisionManager: CollisionManager
  ) {
    this.jointController = jointController;
    this.collisionManager = collisionManager;

    const current = this.jointController.robotModel.robotJoints.getCurrentAnglesDeg();
    this.lastSafeAnglesDeg = [...current];
    this.finalTargetAnglesDeg = [...current];
  }

  /**
   * Commands the joint motors toward a target 6-DOF joint configuration (in degrees).
   */
  public commandJointAnglesDeg(targetAnglesDeg: number[]): number[] {
    const clamped = clampAnglesDegToConfig(
      targetAnglesDeg,
      this.jointController.robotModel.config
    );
    this.finalTargetAnglesDeg = [...clamped];
    this.isBlockedByCollision = false;
    this.lastBlockReason = '';
    return this.jointController.setAllJointTargetPositionsDeg(clamped);
  }

  /**
   * Computes an Inverse Kinematics solution for a target Cartesian position and
   * orientation, then commands the physical joint motors to drive toward that target.
   * Does NOT directly set mesh positions!
   */
  public commandCartesianPose(
    targetPosition: [number, number, number] | THREE.Vector3,
    targetOrientation?: [number, number, number, number] | THREE.Quaternion | null
  ): IKSolverResult {
    const currentAngles =
      this.jointController.robotModel.robotJoints.getCurrentAnglesDeg();

    const ikResult = solveInverseKinematics(
      {
        targetPosition,
        targetQuaternion: targetOrientation ?? createTopDownQuaternion(0),
        currentAnglesDeg: currentAngles
      },
      this.jointController.robotModel.config
    );

    this.lastIKResult = ikResult;

    if (ikResult.success) {
      this.commandJointAnglesDeg(ikResult.targetAnglesDeg);
    }

    return ikResult;
  }

  public setCartesianTarget(
    targetPosition: [number, number, number] | THREE.Vector3,
    targetOrientation?: [number, number, number, number] | THREE.Quaternion | null
  ): IKSolverResult {
    return this.commandCartesianPose(targetPosition, targetOrientation);
  }

  /**
   * Executes one fixed-timestep motor & collision update step.
   * Called before each `PhysicsWorld.stepSingle()` substep.
   */
  public stepControlAndMotors(dt: number, payloadMassKg: number = 0): void {
    const robot = this.jointController.robotModel;
    const prevAnglesRad = robot.robotJoints.getCurrentAnglesRad();

    // Sync current visual & kinematic transforms to obtain exact moment arms
    const kin = robot.syncVisualHierarchyFromJoints(false);
    const prevColStatus = this.collisionManager.evaluateCollisions();
    const prevMaxPenetration = prevColStatus.prohibitedCollisions.reduce(
      (maxP, c) => Math.max(maxP, c.penetrationDepthMeters),
      0
    );

    // Step the physical joint motors under PD + gravity + payload torque
    this.jointController.jointManager.stepMotors(
      dt,
      kin.jointWorldPivots,
      kin.jointWorldAxes,
      kin.linkWorldCOMs,
      kin.tcpWorldPos,
      payloadMassKg
    );

    // Update link transforms & Rapier rigid bodies after motor step
    robot.syncVisualHierarchyFromJoints(false);

    // Evaluate collisions against obstacles, dynamic objects, and ground
    const colStatus = this.collisionManager.evaluateCollisions();
    const currMaxPenetration = colStatus.prohibitedCollisions.reduce(
      (maxP, c) => Math.max(maxP, c.penetrationDepthMeters),
      0
    );

    // Unilateral contact rule: only block/revert when the step creates a new collision
    // or increases/maintains penetration depth. Always allow separating motion (retraction).
    if (
      colStatus.hasProhibitedCollision &&
      currMaxPenetration >= prevMaxPenetration - 1e-5
    ) {
      // Transfer physical contact impulse to any dynamic object contacted by the arm
      for (const col of colStatus.prohibitedCollisions) {
        const dynId =
          col.categoryA === 'instrument'
            ? col.bodyIdA
            : col.categoryB === 'instrument'
            ? col.bodyIdB
            : null;
        if (dynId) {
          const sign = col.bodyIdB === dynId ? 1 : -1;
          const impScale = 0.025;
          this.collisionManager.bodyManager.applyImpulse(dynId, [
            col.contactNormal[0] * sign * impScale,
            Math.max(0, col.contactNormal[1] * sign * impScale),
            col.contactNormal[2] * sign * impScale
          ]);
        }
      }

      // Revert the penetrating joint increment so robot links cannot pass through solid objects/obstacles,
      // without locking the controller against future retraction commands.
      for (let i = 0; i < robot.robotJoints.joints.length; i++) {
        const mj = robot.robotJoints.joints[i];
        mj.currentAngleRad = prevAnglesRad[i];
        mj.angularVelocityRadPerSec = 0;
      }
      robot.syncVisualHierarchyFromJoints(true);
      this.isBlockedByCollision = true;
      this.lastBlockReason =
        colStatus.prohibitedCollisions[0]?.description ||
        'Motion blocked by physical collision.';
    } else {
      this.lastSafeAnglesDeg = robot.robotJoints.getCurrentAnglesDeg();
    }
  }

  /**
   * Returns true if the physical joints have converged to the commanded target pose.
   */
  public hasReachedTarget(toleranceDeg: number = 1.5): boolean {
    return this.jointController.areJointsAtTargets(toleranceDeg);
  }

  /**
   * Commands the robot motors to return to Home pose.
   */
  public moveToHomePose(immediate: boolean = false): void {
    this.isBlockedByCollision = false;
    this.lastBlockReason = '';
    this.jointController.robotModel.robotJoints.resetToHomePose(immediate);
    this.finalTargetAnglesDeg =
      this.jointController.robotModel.robotJoints.getTargetAnglesDeg();
    if (immediate) {
      this.jointController.robotModel.syncVisualHierarchyFromJoints(true);
    }
  }
}
