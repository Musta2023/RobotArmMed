/**
 * @file FailureDetector.ts
 * Detects manipulation, physical, and mechanical failures during robotic execution:
 * - grasp_failed
 * - object_slipping
 * - object_falling
 * - collision_detected
 * - joint_limit_violation
 * - excessive_motor_torque
 * - placement_failed
 * - unstable_manipulation
 */

import { RobotStateSnapshot } from '../robot/RobotState.js';
import { ContactEventRecord } from '../physics/CollisionManager.js';

export type SurgicalErrorCode =
  | 'grasp_failed'
  | 'object_slipping'
  | 'object_falling'
  | 'collision_detected'
  | 'joint_limit_violation'
  | 'excessive_motor_torque'
  | 'placement_failed'
  | 'unstable_manipulation';

export type ObjectSecurityState =
  | 'secured'
  | 'not_secured'
  | 'slipping'
  | 'dropped'
  | 'misplaced';

export interface OrchestratorFailurePayload {
  task_id: string;
  status: 'failure' | 'success' | 'running';
  error: SurgicalErrorCode | null;
  retry_allowed: boolean;
  object_state: ObjectSecurityState;
  details?: string;
  timestamp?: number;
}

export class FailureDetector {
  public taskId: string = 'surgical_001';
  public activeFailure: OrchestratorFailurePayload | null = null;
  public failureHistory: OrchestratorFailurePayload[] = [];
  private wasPreviouslyHolding: boolean = false;

  /**
   * Generates a standardized orchestrator failure response matching Section 8 specification.
   */
  public recordFailure(
    error: SurgicalErrorCode,
    objectState: ObjectSecurityState,
    details: string,
    retryAllowed: boolean = true,
    timestamp: number = 0
  ): OrchestratorFailurePayload {
    const payload: OrchestratorFailurePayload = {
      task_id: this.taskId,
      status: 'failure',
      error,
      retry_allowed: retryAllowed,
      object_state: objectState,
      details,
      timestamp
    };

    this.activeFailure = payload;
    this.failureHistory.push(payload);
    return payload;
  }

  /**
   * Monitors real-time robot telemetry, collision state, and grasp physics
   * during active manipulation to detect physical anomalies.
   */
  public monitorStep(params: {
    robotSnapshot: RobotStateSnapshot;
    prohibitedCollisions: ContactEventRecord[];
    expectingObjectHeld: boolean;
  }): OrchestratorFailurePayload | null {
    const { robotSnapshot, prohibitedCollisions, expectingObjectHeld } = params;

    // 1. Check prohibited obstacle or self-collision
    if (prohibitedCollisions.length > 0) {
      return this.recordFailure(
        'collision_detected',
        robotSnapshot.gripper.isGrasping ? 'secured' : 'not_secured',
        prohibitedCollisions[0].description,
        true,
        robotSnapshot.timestamp
      );
    }

    // 2. Check excessive motor torque saturation
    if (robotSnapshot.anyTorqueExceeded) {
      const satJoint = robotSnapshot.joints.find((j) => j.isTorqueSaturated);
      return this.recordFailure(
        'excessive_motor_torque',
        robotSnapshot.gripper.isGrasping ? 'secured' : 'not_secured',
        `Excessive motor torque on ${satJoint?.id ?? 'joint'} (${satJoint?.motorTorqueNm.toFixed(2)} N·m limit reached).`,
        true,
        robotSnapshot.timestamp
      );
    }

    // 3. Check object slipping between fingers during transport/lift
    if (expectingObjectHeld && robotSnapshot.gripper.isSlipping) {
      return this.recordFailure(
        'object_slipping',
        'slipping',
        `Insufficient friction capacity (${robotSnapshot.gripper.frictionCapacityNewtons.toFixed(2)} N) to hold instrument against gravity.`,
        true,
        robotSnapshot.timestamp
      );
    }

    // 4. Check unexpected object drop while transporting
    if (expectingObjectHeld && this.wasPreviouslyHolding && !robotSnapshot.gripper.isGrasping) {
      this.wasPreviouslyHolding = false;
      return this.recordFailure(
        'object_falling',
        'dropped',
        'Instrument lost physical contact during transport and fell under gravity.',
        true,
        robotSnapshot.timestamp
      );
    }

    this.wasPreviouslyHolding = robotSnapshot.gripper.isGrasping;
    return null;
  }

  public clearActiveFailure(): void {
    this.activeFailure = null;
    this.wasPreviouslyHolding = false;
  }
}
