/**
 * @file ActionInterface.ts
 * Standardized AI Policy Action Interface supporting Cartesian target poses,
 * joint-space targets, torque-limited motor commands, and gripper commands.
 */

import { MotionController } from '../control/MotionController.js';
import { Gripper } from '../robot/Gripper.js';

export interface PolicyActionCommand {
  target_position?: [number, number, number];
  target_orientation?: [number, number, number, number];
  joint_targets_deg?: number[];
  max_torques_nm?: number[];
  gripper_command?: 'open' | 'close' | 'hold';
  gripper_opening_meters?: number;
  gripping_force_n?: number;
}

export interface PolicyActionResult {
  accepted: boolean;
  appliedAction: PolicyActionCommand;
  message: string;
}

export class ActionInterface {
  public motionController: MotionController;
  public gripper: Gripper;
  public lastAction: PolicyActionCommand = { gripper_command: 'hold' };

  constructor(motionController: MotionController, gripper: Gripper) {
    this.motionController = motionController;
    this.gripper = gripper;
  }

  /**
   * Validates and dispatches an external AI policy command to the physical Digital Twin.
   */
  public applyAction(action: PolicyActionCommand): PolicyActionResult {
    if (!action || typeof action !== 'object') {
      return {
        accepted: false,
        appliedAction: {},
        message: 'Invalid action object.'
      };
    }

    this.lastAction = { ...action };

    // 1. Apply optional per-joint torque limits
    if (Array.isArray(action.max_torques_nm)) {
      action.max_torques_nm.forEach((torque, idx) => {
        if (Number.isFinite(torque)) {
          this.motionController.jointController.setJointMaxTorqueNm(idx, torque);
        }
      });
    }

    // 2. Apply gripper commands
    if (action.gripping_force_n !== undefined && Number.isFinite(action.gripping_force_n)) {
      this.gripper.setGrippingForceNewtons(action.gripping_force_n);
    }

    if (action.gripper_command === 'open') {
      this.gripper.open(false);
    } else if (action.gripper_command === 'close') {
      this.gripper.close(false);
    } else if (
      action.gripper_opening_meters !== undefined &&
      Number.isFinite(action.gripper_opening_meters)
    ) {
      this.gripper.setTargetOpeningMeters(action.gripper_opening_meters, false);
    }

    // 3. Apply joint-space target angles if provided
    if (Array.isArray(action.joint_targets_deg) && action.joint_targets_deg.length > 0) {
      this.motionController.commandJointAnglesDeg(action.joint_targets_deg);
      return {
        accepted: true,
        appliedAction: action,
        message: 'Joint-space motor targets applied.'
      };
    }

    // 4. Apply Cartesian end-effector pose via IK -> Joint Motors
    if (Array.isArray(action.target_position) && action.target_position.length === 3) {
      const ikRes = this.motionController.commandCartesianPose(
        action.target_position,
        action.target_orientation ?? null
      );
      return {
        accepted: ikRes.success,
        appliedAction: action,
        message: ikRes.message
      };
    }

    return {
      accepted: true,
      appliedAction: action,
      message: 'Gripper / torque command applied.'
    };
  }
}
