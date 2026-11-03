/**
 * @file ActionInterface.ts
 * Standardized AI Policy Action Interface supporting:
 * - 7-DOF Normalized Relative Joint Actions: A = [Δq1..Δq6, gripper] ∈ [-1, 1]^7
 * - Cartesian target poses (IK -> PD Joint Motors)
 * - Absolute joint-space targets
 * - Torque-limited motor commands
 * - Action clipping and safety enforcement
 */

import { MotionController } from '../control/MotionController.js';
import { Gripper } from '../robot/Gripper.js';
import { clampAnglesDegToConfig } from '../control/ForwardKinematics.js';
import { RAD2DEG, DEG2RAD } from '../robot/RobotJoints.js';

export interface PolicyActionCommand {
  // 7-DOF normalized relative action: [Δq1, Δq2, Δq3, Δq4, Δq5, Δq6, gripper] ∈ [-1, 1]^7
  delta_joint_actions_normalized?: number[];

  // Traditional absolute target interfaces (preserved for backward compatibility)
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
  wasClipped: boolean;
  commandedDeltaDeg: number[];
  actualResultingAnglesDeg: number[];
  message: string;
}

export class ActionInterface {
  public motionController: MotionController;
  public gripper: Gripper;
  public lastAction: PolicyActionCommand = { gripper_command: 'hold' };

  // Maximum angular step per control tick (20 Hz control timestep: dt = 0.05s)
  // Max physical velocity ~ 1.5 rad/s -> max delta per tick = 1.5 * 0.05 = 0.075 rad ≈ 4.3°
  public maxJointVelocityRadPerSec: number = 1.6;
  public controlTimestepSec: number = 0.05;

  constructor(motionController: MotionController, gripper: Gripper) {
    this.motionController = motionController;
    this.gripper = gripper;
  }

  /**
   * Applies a 7-DOF normalized relative action vector:
   * a = [Δq1, Δq2, Δq3, Δq4, Δq5, Δq6, gripper_command] ∈ [-1, 1]^7
   */
  public applyNormalizedAction(action7D: number[]): PolicyActionResult {
    if (!Array.isArray(action7D) || action7D.length < 7) {
      return {
        accepted: false,
        appliedAction: {},
        wasClipped: false,
        commandedDeltaDeg: [],
        actualResultingAnglesDeg: [],
        message: 'Invalid action: expected 7-dimensional normalized action array.'
      };
    }

    const robot = this.motionController.jointController.robotModel;
    const currentAnglesDeg = robot.robotJoints.getCurrentAnglesDeg();
    const config = robot.config;

    const maxDeltaRad = this.maxJointVelocityRadPerSec * this.controlTimestepSec;
    const maxDeltaDeg = maxDeltaRad * RAD2DEG;

    let wasClipped = false;
    const commandedDeltaDeg: number[] = [];
    const targetAnglesDeg: number[] = [];

    // 1. Process 6-DOF joint deltas
    for (let i = 0; i < 6; i++) {
      const rawVal = action7D[i];
      if (!Number.isFinite(rawVal)) {
        wasClipped = true;
        commandedDeltaDeg.push(0);
        targetAnglesDeg.push(currentAnglesDeg[i]);
        continue;
      }

      // Clip normalized action to [-1, 1]
      const clippedVal = Math.min(Math.max(rawVal, -1.0), 1.0);
      if (clippedVal !== rawVal) wasClipped = true;

      const deltaDeg = clippedVal * maxDeltaDeg;
      commandedDeltaDeg.push(deltaDeg);

      // Desired unclipped joint target
      const unclippedTarget = currentAnglesDeg[i] + deltaDeg;

      // Clamp to physical joint limits
      const jointCfg = config.joints[i];
      const clampedTarget = Math.min(Math.max(unclippedTarget, jointCfg.minAngleDeg), jointCfg.maxAngleDeg);
      if (clampedTarget !== unclippedTarget) wasClipped = true;

      targetAnglesDeg.push(clampedTarget);
    }

    // 2. Process gripper action (dimension index 6)
    const rawGripper = action7D[6];
    const clippedGripper = Math.min(Math.max(rawGripper, -1.0), 1.0);
    let gripperCommand: 'open' | 'close' | 'hold' = 'hold';

    if (clippedGripper > 0.33) {
      gripperCommand = 'close';
      this.gripper.close(false);
    } else if (clippedGripper < -0.33) {
      gripperCommand = 'open';
      this.gripper.open(false);
    }

    // 3. Command joint controllers
    this.motionController.commandJointAnglesDeg(targetAnglesDeg);

    const resultingAnglesDeg = robot.robotJoints.getCurrentAnglesDeg();

    const applied: PolicyActionCommand = {
      delta_joint_actions_normalized: action7D.slice(0, 7),
      joint_targets_deg: targetAnglesDeg,
      gripper_command: gripperCommand
    };
    this.lastAction = applied;

    return {
      accepted: true,
      appliedAction: applied,
      wasClipped,
      commandedDeltaDeg,
      actualResultingAnglesDeg: resultingAnglesDeg,
      message: wasClipped
        ? 'Action applied with joint limit or velocity clipping.'
        : 'Action applied successfully.'
    };
  }

  /**
   * Validates and dispatches an external AI policy command to the physical Digital Twin.
   */
  public applyAction(action: PolicyActionCommand): PolicyActionResult {
    if (!action || typeof action !== 'object') {
      return {
        accepted: false,
        appliedAction: {},
        wasClipped: false,
        commandedDeltaDeg: [],
        actualResultingAnglesDeg: [],
        message: 'Invalid action object.'
      };
    }

    this.lastAction = { ...action };

    // Check if 7-DOF normalized action is provided
    if (
      Array.isArray(action.delta_joint_actions_normalized) &&
      action.delta_joint_actions_normalized.length >= 7
    ) {
      return this.applyNormalizedAction(action.delta_joint_actions_normalized);
    }

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

    const currentDeg = this.motionController.jointController.robotModel.robotJoints.getCurrentAnglesDeg();

    // 3. Apply joint-space target angles if provided
    if (Array.isArray(action.joint_targets_deg) && action.joint_targets_deg.length > 0) {
      this.motionController.commandJointAnglesDeg(action.joint_targets_deg);
      return {
        accepted: true,
        appliedAction: action,
        wasClipped: false,
        commandedDeltaDeg: action.joint_targets_deg.map((t, i) => t - currentDeg[i]),
        actualResultingAnglesDeg: this.motionController.jointController.robotModel.robotJoints.getCurrentAnglesDeg(),
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
        wasClipped: !ikRes.success,
        commandedDeltaDeg: ikRes.targetAnglesDeg.map((t, i) => t - currentDeg[i]),
        actualResultingAnglesDeg: this.motionController.jointController.robotModel.robotJoints.getCurrentAnglesDeg(),
        message: ikRes.message
      };
    }

    return {
      accepted: true,
      appliedAction: action,
      wasClipped: false,
      commandedDeltaDeg: [],
      actualResultingAnglesDeg: currentDeg,
      message: 'Gripper / torque command applied.'
    };
  }
}
