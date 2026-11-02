/**
 * @file PickAndPlace.ts
 * 9-State Physics-Based Pick and Place Manipulation State Machine for the
 * Surgical Action AI Digital Twin.
 *
 * States required by Section 7:
 * 1. IDLE
 * 2. APPROACH
 * 3. GRASP
 * 4. GRASP_VERIFICATION
 * 5. LIFT
 * 6. TRANSPORT
 * 7. PLACE
 * 8. VERIFY_PLACEMENT
 * 9. COMPLETE
 * (plus FAILED when halted by FailureDetector)
 */

import * as THREE from 'three';
import { MotionController } from '../control/MotionController.js';
import { Gripper } from '../robot/Gripper.js';
import { GraspManager } from './GraspManager.js';
import { PlacementVerifier, PlacementVerificationReport } from './PlacementVerifier.js';
import { InstrumentFactory } from '../objects/InstrumentFactory.js';
import { TARGET_TRAY_POS } from '../objects/ObjectProperties.js';
import {
  FailureDetector,
  OrchestratorFailurePayload
} from '../recovery/FailureDetector.js';
import { RecoveryManager } from '../recovery/RecoveryManager.js';

export type PickAndPlaceState =
  | 'IDLE'
  | 'APPROACH'
  | 'GRASP'
  | 'GRASP_VERIFICATION'
  | 'LIFT'
  | 'TRANSPORT'
  | 'PLACE'
  | 'VERIFY_PLACEMENT'
  | 'COMPLETE'
  | 'FAILED';

export const PICK_AND_PLACE_SEQUENCE: PickAndPlaceState[] = [
  'IDLE',
  'APPROACH',
  'GRASP',
  'GRASP_VERIFICATION',
  'LIFT',
  'TRANSPORT',
  'PLACE',
  'VERIFY_PLACEMENT',
  'COMPLETE'
];

export class PickAndPlace {
  public motionController: MotionController;
  public gripper: Gripper;
  public graspManager: GraspManager;
  public placementVerifier: PlacementVerifier;
  public instrumentFactory: InstrumentFactory;
  public failureDetector: FailureDetector;
  public recoveryManager: RecoveryManager;

  public state: PickAndPlaceState = 'IDLE';
  public stateHistory: PickAndPlaceState[] = ['IDLE'];
  public subPhase: number = 0;
  public stateTimerSeconds: number = 0;
  public targetPosition: [number, number, number] = [...TARGET_TRAY_POS];
  public lastPlacementReport: PlacementVerificationReport | null = null;
  public lastOrchestratorResult: OrchestratorFailurePayload | null = null;
  public statusMessage: string = 'Ready at Home Pose.';
  public injectFailureNextGrasp: boolean = false;

  constructor(params: {
    motionController: MotionController;
    gripper: Gripper;
    graspManager: GraspManager;
    placementVerifier: PlacementVerifier;
    instrumentFactory: InstrumentFactory;
    failureDetector: FailureDetector;
    recoveryManager: RecoveryManager;
  }) {
    this.motionController = params.motionController;
    this.gripper = params.gripper;
    this.graspManager = params.graspManager;
    this.placementVerifier = params.placementVerifier;
    this.instrumentFactory = params.instrumentFactory;
    this.failureDetector = params.failureDetector;
    this.recoveryManager = params.recoveryManager;
  }

  private _transitionTo(nextState: PickAndPlaceState): void {
    if (this.state !== nextState) {
      this.state = nextState;
      this.stateHistory.push(nextState);
    }
    this.subPhase = 0;
    this.stateTimerSeconds = 0;
    this._onEnterState(nextState);
  }

  /**
   * Starts the 9-state Pick and Place manipulation task.
   */
  public start(customTargetPos?: [number, number, number]): boolean {
    const inst = this.instrumentFactory.getActiveInstrument();
    if (!inst) {
      this._triggerFailure(
        'grasp_failed',
        'not_secured',
        'Cannot start PickAndPlace: no active surgical instrument in scene.'
      );
      return false;
    }

    if (customTargetPos) {
      this.targetPosition = [...customTargetPos];
      this.placementVerifier.targetTrayCenter.set(
        customTargetPos[0],
        customTargetPos[1],
        customTargetPos[2]
      );
    }

    this.failureDetector.clearActiveFailure();
    this.lastPlacementReport = null;
    this.state = 'IDLE';
    this.stateHistory = ['IDLE'];
    this._transitionTo('APPROACH');
    return true;
  }

  private _triggerFailure(
    errorCode: Parameters<FailureDetector['recordFailure']>[0],
    objectState: Parameters<FailureDetector['recordFailure']>[1],
    details: string
  ): OrchestratorFailurePayload {
    const payload = this.failureDetector.recordFailure(
      errorCode,
      objectState,
      details,
      true,
      this.graspManager.physicsWorld.simulationTime
    );
    this.lastOrchestratorResult = payload;
    this.statusMessage = `FAILURE [${errorCode}]: ${details}`;
    this.recoveryManager.handleFailure(payload);
    this.state = 'FAILED';
    this.stateHistory.push('FAILED');
    return payload;
  }

  /**
   * Configures whether the next grasp attempt should intentionally miss to test Section 8 Failure & Recovery.
   */
  public injectIntentionalGraspFailure(enabled: boolean): void {
    this.injectFailureNextGrasp = enabled;
  }

  /**
   * Authorizes recovery via RecoveryManager and restarts the Pick and Place state machine.
   */
  public authorizeAndRecover(): boolean {
    this.injectFailureNextGrasp = false;
    const res = this.recoveryManager.authorizeAndExecuteRecovery(() => {
      this.start();
    });
    if (res.executed) {
      this.statusMessage = res.message;
    }
    return res.executed;
  }

  private _onEnterState(state: PickAndPlaceState): void {
    const inst = this.instrumentFactory.getActiveInstrument();
    if (!inst) return;

    const bodyPos = inst.managedBody.body.translation();
    const safeHoverY = 0.24;

    switch (state) {
      case 'APPROACH': {
        this.statusMessage = 'APPROACH: Aligning end-effector above surgical instrument...';
        this.gripper.open(false);
        // Subphase 0: move above instrument (with intentional offset if failure test requested)
        const offsetX = this.injectFailureNextGrasp ? 0.095 : 0.0;
        const ikRes = this.motionController.commandCartesianPose([
          bodyPos.x + offsetX,
          safeHoverY,
          bodyPos.z
        ]);
        if (!ikRes.success) {
          this._triggerFailure('grasp_failed', 'not_secured', ikRes.message);
        }
        break;
      }

      case 'GRASP': {
        this.statusMessage = 'GRASP: Closing motorized gripper fingers...';
        this.gripper.close(false);
        break;
      }

      case 'GRASP_VERIFICATION': {
        this.statusMessage = 'GRASP_VERIFICATION: Checking bilateral contact & Coulomb friction...';
        // Evaluated in step()
        break;
      }

      case 'LIFT': {
        this.statusMessage = 'LIFT: Raising grasped surgical instrument to safe clearance...';
        const ikLift = this.motionController.commandCartesianPose([
          bodyPos.x,
          safeHoverY,
          bodyPos.z
        ]);
        if (!ikLift.success) {
          this._triggerFailure('unstable_manipulation', 'secured', ikLift.message);
        }
        break;
      }

      case 'TRANSPORT': {
        this.statusMessage = 'TRANSPORT: Moving surgical instrument across workcell to Target Tray...';
        const ikTrans = this.motionController.commandCartesianPose([
          this.targetPosition[0],
          safeHoverY,
          this.targetPosition[2]
        ]);
        if (!ikTrans.success) {
          this._triggerFailure('placement_failed', 'secured', ikTrans.message);
        }
        break;
      }

      case 'PLACE': {
        this.statusMessage = 'PLACE: Lowering instrument onto Target Surgical Tray & releasing...';
        const placeY =
          this.targetPosition[1] + inst.config.dimensions.height / 2 + 0.002;
        const ikPlace = this.motionController.commandCartesianPose([
          this.targetPosition[0],
          placeY,
          this.targetPosition[2]
        ]);
        if (!ikPlace.success) {
          this._triggerFailure('placement_failed', 'secured', ikPlace.message);
        }
        break;
      }

      case 'VERIFY_PLACEMENT': {
        this.statusMessage = 'VERIFY_PLACEMENT: Checking tray position, orientation & stability...';
        // Evaluated in step()
        break;
      }

      case 'COMPLETE': {
        this.statusMessage = 'COMPLETE: Surgical instrument placed and verified on Target Tray.';
        this.lastOrchestratorResult = {
          task_id: this.failureDetector.taskId,
          status: 'success',
          error: null,
          retry_allowed: false,
          object_state: 'secured',
          details: 'Pick and place manipulation task completed and verified.'
        };
        break;
      }

      default:
        break;
    }
  }

  /**
   * Advances the Pick and Place state machine by one fixed timestep `dt`.
   */
  public step(dt: number): void {
    if (this.state === 'IDLE' || this.state === 'COMPLETE' || this.state === 'FAILED') {
      return;
    }

    this.stateTimerSeconds += dt;
    const inst = this.instrumentFactory.getActiveInstrument();
    if (!inst) {
      this._triggerFailure('grasp_failed', 'not_secured', 'Instrument removed during task.');
      return;
    }

    // Monitor continuous physical safety during active states
    const expectingHeld = this.state === 'LIFT' || this.state === 'TRANSPORT';
    const monitorFail = this.failureDetector.monitorStep({
      robotSnapshot: this.motionController.jointController.robotModel.state.latestSnapshot,
      prohibitedCollisions: this.motionController.collisionManager.prohibitedCollisions,
      expectingObjectHeld: expectingHeld
    });

    if (monitorFail) {
      this.lastOrchestratorResult = monitorFail;
      this.recoveryManager.handleFailure(monitorFail);
      this.state = 'FAILED';
      this.stateHistory.push('FAILED');
      return;
    }

    const bodyPos = inst.managedBody.body.translation();

    switch (this.state) {
      case 'APPROACH': {
        if (this.subPhase === 0) {
          if (this.motionController.hasReachedTarget(0.6)) {
            this.subPhase = 1;
            const offsetX = this.injectFailureNextGrasp ? 0.095 : 0.0;
            const descendIk = this.motionController.commandCartesianPose([
              bodyPos.x + offsetX,
              bodyPos.y,
              bodyPos.z
            ]);
            if (!descendIk.success) {
              this._triggerFailure('grasp_failed', 'not_secured', descendIk.message);
            }
          }
        } else if (this.subPhase === 1) {
          if (this.motionController.hasReachedTarget(0.35)) {
            this._transitionTo('GRASP');
          }
        }
        break;
      }

      case 'GRASP': {
        // Wait for motorized fingers to close onto object or reach minimum opening
        if (this.stateTimerSeconds >= 0.50) {
          this._transitionTo('GRASP_VERIFICATION');
        }
        break;
      }

      case 'GRASP_VERIFICATION': {
        // Verify physical contact AND friction stability
        if (this.graspManager.isGrasping && this.graspManager.graspedInstrument === inst) {
          this._transitionTo('LIFT');
        } else {
          this._triggerFailure(
            'grasp_failed',
            this.graspManager.isSlipping ? 'slipping' : 'not_secured',
            'Physical grasp verification failed: insufficient bilateral contact or friction force.'
          );
        }
        break;
      }

      case 'LIFT': {
        if (!this.graspManager.isGrasping) {
          this._triggerFailure(
            'object_falling',
            'dropped',
            'Instrument slipped or fell during LIFT state.'
          );
          return;
        }
        if (this.motionController.hasReachedTarget(0.6)) {
          this._transitionTo('TRANSPORT');
        }
        break;
      }

      case 'TRANSPORT': {
        if (!this.graspManager.isGrasping) {
          this._triggerFailure(
            'object_falling',
            'dropped',
            'Instrument lost during TRANSPORT state.'
          );
          return;
        }
        if (this.motionController.hasReachedTarget(0.6)) {
          this._transitionTo('PLACE');
        }
        break;
      }

      case 'PLACE': {
        if (this.subPhase === 0) {
          if (this.motionController.hasReachedTarget(0.4)) {
            this.subPhase = 1;
            this.stateTimerSeconds = 0;
            this.gripper.open(false);
          }
        } else if (this.subPhase === 1) {
          if (this.stateTimerSeconds >= 0.35 && !this.graspManager.isGrasping) {
            this.subPhase = 2;
            this.motionController.commandCartesianPose([
              this.targetPosition[0],
              0.22,
              this.targetPosition[2]
            ]);
          }
        } else if (this.subPhase === 2) {
          if (this.motionController.hasReachedTarget(0.8)) {
            this._transitionTo('VERIFY_PLACEMENT');
          }
        }
        break;
      }

      case 'VERIFY_PLACEMENT': {
        const report = this.placementVerifier.verify(inst, this.graspManager.isGrasping);
        this.lastPlacementReport = report;
        if (report.verified) {
          this._transitionTo('COMPLETE');
        } else if (this.stateTimerSeconds >= 0.35) {
          this._triggerFailure('placement_failed', 'misplaced', report.reason);
        }
        break;
      }

      default:
        break;
    }
  }

  /**
   * Resets the Pick and Place state machine to IDLE.
   */
  public reset(): void {
    this.state = 'IDLE';
    this.stateHistory = ['IDLE'];
    this.subPhase = 0;
    this.stateTimerSeconds = 0;
    this.lastPlacementReport = null;
    this.lastOrchestratorResult = null;
    this.failureDetector.clearActiveFailure();
  }
}
