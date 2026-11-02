/**
 * @file RecoveryManager.ts
 * Orchestrated failure recovery system for the Surgical Action AI Digital Twin.
 *
 * Implements Section 8 recovery strategies:
 * - Safely arrest robot joint motors on failure
 * - Wait for explicit orchestrator authorization before retrying
 * - Reopen the gripper
 * - Reposition the end-effector above the current actual object location
 * - Adjust grasp offset / gripping force if slipping occurred
 * - Recalculate trajectory and resume manipulation
 */

import {
  FailureDetector,
  OrchestratorFailurePayload,
  SurgicalErrorCode
} from './FailureDetector.js';
import { MotionController } from '../control/MotionController.js';
import { Gripper } from '../robot/Gripper.js';

export type RecoveryStrategyType =
  | 'reopen_and_realign_grasp'
  | 'increase_force_and_regrasp'
  | 'retract_and_recalculate_trajectory'
  | 'reset_to_safe_home';

export interface RecoveryPlan {
  strategy: RecoveryStrategyType;
  authorized: boolean;
  description: string;
  targetError: SurgicalErrorCode;
}

export class RecoveryManager {
  public failureDetector: FailureDetector;
  public motionController: MotionController;
  public gripper: Gripper;

  public awaitingAuthorization: boolean = false;
  public pendingPlan: RecoveryPlan | null = null;
  public recoveryAttempts: number = 0;

  constructor(
    failureDetector: FailureDetector,
    motionController: MotionController,
    gripper: Gripper
  ) {
    this.failureDetector = failureDetector;
    this.motionController = motionController;
    this.gripper = gripper;
  }

  /**
   * Arrests the robot safely upon detecting a failure and prepares a recovery plan
   * waiting for orchestrator authorization.
   */
  public handleFailure(failure: OrchestratorFailurePayload): RecoveryPlan {
    // 1. Stop/hold current joint positions safely
    this.motionController.jointController.holdCurrentPositions();

    let strategy: RecoveryStrategyType = 'reopen_and_realign_grasp';
    let description =
      'Reopen gripper, reposition end-effector above live instrument pose, and retry grasp.';

    switch (failure.error) {
      case 'object_slipping':
      case 'object_falling':
        strategy = 'increase_force_and_regrasp';
        description =
          'Reopen gripper, increase gripping normal force, re-center over fallen instrument, and re-grasp.';
        break;
      case 'collision_detected':
        strategy = 'retract_and_recalculate_trajectory';
        description =
          'Retract manipulator to high-clearance corridor pose and recalculate collision-free trajectory.';
        break;
      case 'excessive_motor_torque':
        strategy = 'reset_to_safe_home';
        description =
          'Release overload or increase motor torque allocation and return to safe posture.';
        break;
      case 'grasp_failed':
      case 'placement_failed':
      default:
        strategy = 'reopen_and_realign_grasp';
        break;
    }

    this.pendingPlan = {
      strategy,
      authorized: false,
      description,
      targetError: failure.error ?? 'grasp_failed'
    };
    this.awaitingAuthorization = true;

    return this.pendingPlan;
  }

  /**
   * Authorizes and executes the pending recovery strategy on behalf of the orchestrator.
   */
  public authorizeAndExecuteRecovery(onRestartTask?: () => void): {
    executed: boolean;
    strategy: RecoveryStrategyType | null;
    message: string;
  } {
    if (!this.pendingPlan || !this.awaitingAuthorization) {
      return {
        executed: false,
        strategy: null,
        message: 'No pending failure recovery plan awaiting authorization.'
      };
    }

    this.pendingPlan.authorized = true;
    this.awaitingAuthorization = false;
    this.recoveryAttempts += 1;

    const strategy = this.pendingPlan.strategy;

    // Apply physical corrective actions before restarting task trajectory
    this.gripper.open(true);
    this.motionController.moveToHomePose(true);

    if (strategy === 'increase_force_and_regrasp') {
      const boostedForce = Math.min(
        this.gripper.config.maxGrippingForceNewtons,
        Math.max(this.gripper.grippingForceNewtons * 1.6, 18.0)
      );
      this.gripper.setGrippingForceNewtons(boostedForce);
    } else if (strategy === 'reset_to_safe_home') {
      this.motionController.jointController.setMotorParameters({
        maxTorqueScale: 1.5
      });
      this.motionController.moveToHomePose(false);
    }

    this.failureDetector.clearActiveFailure();
    const msg = `Récupération autorisée [${strategy}] exécutée (tentative #${this.recoveryAttempts}).`;
    this.pendingPlan = null;

    if (onRestartTask) {
      onRestartTask();
    }

    return {
      executed: true,
      strategy,
      message: msg
    };
  }
}

