/**
 * @file JointController.ts
 * Low-level physical joint motor controller supporting position control,
 * velocity control, and torque-limited motor control with stiffness/damping tuning.
 */

import { RobotModel } from '../robot/RobotModel.js';
import { JointManager } from '../physics/JointManager.js';
import { DEG2RAD, RAD2DEG } from '../robot/RobotJoints.js';

export class JointController {
  public robotModel: RobotModel;
  public jointManager: JointManager;

  constructor(robotModel: RobotModel, jointManager: JointManager) {
    this.robotModel = robotModel;
    this.jointManager = jointManager;
  }

  /**
   * Sets target position (in degrees) for a single joint motor.
   * The physics substep will drive the joint motor toward this target.
   */
  public setJointTargetPositionDeg(jointIndex: number, targetDeg: number): number {
    return this.robotModel.robotJoints.setTargetJointAngleDeg(jointIndex, targetDeg);
  }

  /**
   * Sets target positions (in degrees) for all 6 joint motors.
   */
  public setAllJointTargetPositionsDeg(targetsDeg: number[]): number[] {
    return this.robotModel.robotJoints.setTargetJointAnglesDeg(targetsDeg);
  }

  /**
   * Sets target angular velocity (in rad/s) for a joint motor.
   */
  public setJointTargetVelocityRadPerSec(
    jointIndex: number,
    velocityRadPerSec: number
  ): void {
    this.robotModel.robotJoints.setTargetJointVelocityRadPerSec(
      jointIndex,
      velocityRadPerSec
    );
  }

  /**
   * Updates motor stiffness (Kp), damping (Kd), and maximum motor torque scale.
   */
  public setMotorParameters(params: {
    stiffnessScale?: number;
    dampingScale?: number;
    maxTorqueScale?: number;
  }): void {
    if (params.stiffnessScale !== undefined && Number.isFinite(params.stiffnessScale)) {
      this.jointManager.globalStiffnessScale = Math.max(0.05, params.stiffnessScale);
    }
    if (params.dampingScale !== undefined && Number.isFinite(params.dampingScale)) {
      this.jointManager.globalDampingScale = Math.max(0.05, params.dampingScale);
    }
    if (params.maxTorqueScale !== undefined && Number.isFinite(params.maxTorqueScale)) {
      this.jointManager.globalMaxTorqueScale = Math.max(0.05, params.maxTorqueScale);
    }
  }

  /**
   * Updates a specific joint's maximum motor torque limit in N·m.
   */
  public setJointMaxTorqueNm(jointIndex: number, maxTorqueNm: number): void {
    const mj = this.jointManager.orderedJoints[jointIndex];
    if (mj) {
      mj.spec.maxMotorTorqueNm = Math.max(0.2, maxTorqueNm);
    }
  }

  /**
   * Returns true if all joints have converged within `toleranceDeg` of their target angles.
   */
  public areJointsAtTargets(toleranceDeg: number = 1.2): boolean {
    const tolRad = toleranceDeg * DEG2RAD;
    for (const mj of this.jointManager.orderedJoints) {
      if (!mj) continue;
      if (Math.abs(mj.targetAngleRad - mj.currentAngleRad) > tolRad) {
        return false;
      }
      if (Math.abs(mj.angularVelocityRadPerSec) > 0.12) {
        return false;
      }
    }
    return true;
  }

  /**
   * Commands all joints to hold their current physical positions (safe motor brake).
   */
  public holdCurrentPositions(): void {
    for (const mj of this.jointManager.orderedJoints) {
      if (!mj) continue;
      mj.controlMode = 'position';
      mj.targetAngleRad = mj.currentAngleRad;
      mj.targetVelocityRadPerSec = 0;
    }
  }

  /**
   * Returns maximum absolute joint error in degrees across all joints.
   */
  public getMaxTrackingErrorDeg(): number {
    let maxErrRad = 0;
    for (const mj of this.jointManager.orderedJoints) {
      if (!mj) continue;
      const err = Math.abs(mj.targetAngleRad - mj.currentAngleRad);
      if (err > maxErrRad) maxErrRad = err;
    }
    return maxErrRad * RAD2DEG;
  }
}
