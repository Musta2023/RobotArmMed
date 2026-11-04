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
import { InstrumentFactory, ActiveSurgicalInstrument } from '../objects/InstrumentFactory.js';
import { TARGET_TRAY_POS, SIX_OBJECT_DEFINITIONS } from '../objects/ObjectProperties.js';
import { WorkZoneManager } from '../objects/WorkZoneManager.js';
import { createTopDownQuaternion } from '../control/InverseKinematics.js';
import {
  FailureScenarioController,
  FailureScenarioType
} from './FailureScenarioController.js';
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
  public workZoneManager?: WorkZoneManager;

  public isMultiObjectSequence: boolean = false;
  public currentSequenceIndex: number = 0;
  public multiObjectCompletedCount: number = 0;

  public state: PickAndPlaceState = 'IDLE';
  public stateHistory: PickAndPlaceState[] = ['IDLE'];
  public subPhase: number = 0;
  public stateTimerSeconds: number = 0;
  public targetPosition: [number, number, number] = [...TARGET_TRAY_POS];
  public lastPlacementReport: PlacementVerificationReport | null = null;
  public lastOrchestratorResult: OrchestratorFailurePayload | null = null;
  public statusMessage: string = 'Ready at Home Pose.';
  public injectFailureNextGrasp: boolean = false;
  public failureController: FailureScenarioController;

  constructor(params: {
    motionController: MotionController;
    gripper: Gripper;
    graspManager: GraspManager;
    placementVerifier: PlacementVerifier;
    instrumentFactory: InstrumentFactory;
    failureDetector: FailureDetector;
    recoveryManager: RecoveryManager;
    workZoneManager?: WorkZoneManager;
    failureController?: FailureScenarioController;
  }) {
    this.motionController = params.motionController;
    this.gripper = params.gripper;
    this.graspManager = params.graspManager;
    this.placementVerifier = params.placementVerifier;
    this.instrumentFactory = params.instrumentFactory;
    this.failureDetector = params.failureDetector;
    this.recoveryManager = params.recoveryManager;
    this.workZoneManager = params.workZoneManager;
    this.failureController =
      params.failureController ??
      new FailureScenarioController({
        gripper: this.gripper,
        motionController: this.motionController,
        failureDetector: this.failureDetector,
        instrumentFactory: this.instrumentFactory
      });
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
  public start(
    customTargetPos?: [number, number, number],
    options?: { multiObject?: boolean }
  ): boolean {
    if (options?.multiObject) {
      this.isMultiObjectSequence = true;
      this.currentSequenceIndex = 0;
      this.multiObjectCompletedCount = 0;
      if (this.workZoneManager) {
        this.workZoneManager.resetSlotOccupancy();
      }
      return this._prepareAndStartObjectInSequence(0);
    }

    this.isMultiObjectSequence = false;
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
    } else {
      this.targetPosition = [...TARGET_TRAY_POS];
      this.placementVerifier.targetTrayCenter.set(
        TARGET_TRAY_POS[0],
        TARGET_TRAY_POS[1],
        TARGET_TRAY_POS[2]
      );
    }

    this.failureDetector.clearActiveFailure();
    this.lastPlacementReport = null;
    this.state = 'IDLE';
    this.stateHistory = ['IDLE'];
    this._transitionTo('APPROACH');
    return true;
  }

  /**
   * Starts the sequential 6-object pick and place task from Zone A to Zone B.
   */
  public startMultiObjectSequence(): boolean {
    return this.start(undefined, { multiObject: true });
  }

  /**
   * Prepares and approaches the next object in the 6-object sequence.
   */
  private _prepareAndStartObjectInSequence(index: number): boolean {
    const sixIds = ['Object_01', 'Object_02', 'Object_03', 'Object_04', 'Object_05', 'Object_06'];
    if (index >= sixIds.length) {
      this._transitionTo('COMPLETE');
      return true;
    }

    const nextId = sixIds[index];
    const inst = this.instrumentFactory.instruments.get(nextId);
    if (!inst) {
      this._triggerFailure(
        'grasp_failed',
        'not_secured',
        `Cannot start sequence: ${nextId} not found in scene.`
      );
      return false;
    }

    this.instrumentFactory.activeInstrumentId = nextId;
    this.currentSequenceIndex = index;

    const slots = this.workZoneManager?.zoneB.slots ?? [
      [0.22, 0.02, 0.20],
      [0.30, 0.02, 0.20],
      [0.38, 0.02, 0.20],
      [0.22, 0.02, 0.29],
      [0.30, 0.02, 0.29],
      [0.38, 0.02, 0.29]
    ];
    const targetSlot = slots[index] ?? TARGET_TRAY_POS;
    this.targetPosition = [...targetSlot];
    this.placementVerifier.targetTrayCenter.set(targetSlot[0], targetSlot[1], targetSlot[2]);

    if (this.workZoneManager) {
      this.workZoneManager.assignTargetSlot(index, nextId);
    }

    this.failureDetector.clearActiveFailure();
    this.lastPlacementReport = null;
    this.state = 'IDLE';
    this.stateHistory = ['IDLE'];
    this._transitionTo('APPROACH');
    return true;
  }

  /**
   * Computes the optimal top-down grasp quaternion aligned with the object's principal faces,
   * choosing the 90-degree box symmetry branch that minimizes deviation from the arm's radial reach.
   */
  public computeOptimalGraspQuaternion(inst: ActiveSurgicalInstrument): THREE.Quaternion {
    const bodyPos = inst.managedBody.body.translation();
    const bodyRot = inst.managedBody.body.rotation();
    const azimuth = Math.atan2(bodyPos.x, bodyPos.z);

    const objQuat = new THREE.Quaternion(bodyRot.x, bodyRot.y, bodyRot.z, bodyRot.w);
    const objEuler = new THREE.Euler().setFromQuaternion(objQuat, 'YXZ');
    const objYaw = objEuler.y;

    if (inst.config.shape === 'box') {
      // Box has 4-fold 90-degree symmetry. Align jaws with the flat face closest to radial azimuth
      let bestYaw = objYaw;
      let minDiff = Infinity;
      for (const k of [0, 1, 2, 3]) {
        const candidateYaw = objYaw + (k * Math.PI) / 2;
        let diff = (candidateYaw - azimuth) % (2 * Math.PI);
        if (diff > Math.PI) diff -= 2 * Math.PI;
        if (diff < -Math.PI) diff += 2 * Math.PI;
        if (Math.abs(diff) < minDiff) {
          minDiff = Math.abs(diff);
          bestYaw = candidateYaw;
        }
      }
      return createTopDownQuaternion(bestYaw);
    } else {
      return createTopDownQuaternion(azimuth);
    }
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
    this.statusMessage = `ÉCHEC [${errorCode}] : ${details}`;
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
    if (enabled) {
      this.failureController.armScenario('MISSED_APPROACH');
    } else {
      this.failureController.disarm();
    }
  }

  /**
   * Arms a specific failure scenario on the failure controller for the next grasp.
   */
  public armFailureScenario(mode: FailureScenarioType): void {
    this.injectFailureNextGrasp = mode !== 'NONE';
    this.failureController.armScenario(mode);
  }

  /**
   * Authorizes recovery via RecoveryManager and restarts the Pick and Place state machine.
   */
  public authorizeAndRecover(): boolean {
    this.injectFailureNextGrasp = false;
    this.failureController.disarm();
    const isMulti = this.isMultiObjectSequence;
    const curIdx = this.currentSequenceIndex;

    // Rescue objects trapped in inner dead-zone, fallen out of reach, or tipped over on target slot
    const inst = this.instrumentFactory.getActiveInstrument();
    if (inst) {
      const pos = inst.managedBody.body.translation();
      const horizR = Math.hypot(pos.x, pos.z);
      if (horizR < 0.23 || pos.y < 0.005) {
        const def = SIX_OBJECT_DEFINITIONS.find((d) => d.id === inst.config.id);
        if (def) {
          this.instrumentFactory.setInstrumentPose(def.id, def.initialPosition, def.initialQuaternion);
        }
      } else if (this.lastOrchestratorResult?.error === 'placement_failed') {
        const height = inst.config.dimensions.height;
        const targetSlot = this.targetPosition;
        this.instrumentFactory.setInstrumentPose(
          inst.config.id,
          [targetSlot[0], targetSlot[1] + height / 2 + 0.002, targetSlot[2]],
          [0, 0, 0, 1]
        );
      }
    }

    const res = this.recoveryManager.authorizeAndExecuteRecovery(() => {
      if (isMulti) {
        this.isMultiObjectSequence = true;
        this.currentSequenceIndex = curIdx;
        this._prepareAndStartObjectInSequence(curIdx);
      } else {
        this.start();
      }
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
        this.statusMessage = 'APPROCHE : Alignement du préhenseur au-dessus de l\'instrument...';
        this.gripper.open(false);
        // Subphase 0: move above instrument (with intentional offset if failure test requested)
        const graspQuat = this.failureController.getModifiedGraspOrientation(
          this.computeOptimalGraspQuaternion(inst)
        );
        const approachPos = this.failureController.getModifiedApproachTarget([
          bodyPos.x,
          safeHoverY,
          bodyPos.z
        ]);
        const ikRes = this.motionController.commandCartesianPose(
          approachPos,
          graspQuat
        );
        if (!ikRes.success) {
          this._triggerFailure('grasp_failed', 'not_secured', ikRes.message);
        }
        break;
      }

      case 'GRASP': {
        this.statusMessage = 'PRISE : Fermeture des mors motorisés de la pince...';
        this.failureController.onGraspEnter();
        this.gripper.close(false);
        break;
      }

      case 'GRASP_VERIFICATION': {
        this.statusMessage = 'VÉRIF. PRISE : Contrôle du contact bilatéral et du frottement...';
        // Evaluated in step()
        break;
      }

      case 'LIFT': {
        this.statusMessage = 'LEVAGE : Élévation de l\'instrument à la hauteur de sécurité...';
        const graspQuat = this.failureController.getModifiedGraspOrientation(
          this.computeOptimalGraspQuaternion(inst)
        );
        const ikLift = this.motionController.commandCartesianPose(
          [bodyPos.x, safeHoverY, bodyPos.z],
          graspQuat
        );
        if (!ikLift.success) {
          this._triggerFailure('unstable_manipulation', 'secured', ikLift.message);
        }
        break;
      }

      case 'TRANSPORT': {
        this.statusMessage = 'TRANSPORT : Transfert de l\'instrument vers la Zone Cible B...';
        const transAlt = this.failureController.getModifiedTransportAltitude(safeHoverY);
        const ikTrans = this.motionController.commandCartesianPose([
          this.targetPosition[0],
          transAlt,
          this.targetPosition[2]
        ]);
        if (!ikTrans.success) {
          this._triggerFailure('placement_failed', 'secured', ikTrans.message);
        }
        break;
      }

      case 'PLACE': {
        this.statusMessage = 'DÉPOSE : Descente de l\'instrument sur le plateau cible et relâchement...';
        const nominalPlaceY =
          this.targetPosition[1] + inst.config.dimensions.height / 2 + 0.002;
        const graspQuat = this.failureController.getModifiedGraspOrientation(
          this.computeOptimalGraspQuaternion(inst)
        );
        const placeConfig = this.failureController.getModifiedPlacementPose(
          [this.targetPosition[0], nominalPlaceY, this.targetPosition[2]],
          graspQuat
        );
        const ikPlace = this.motionController.commandCartesianPose(
          placeConfig.targetPosition,
          placeConfig.targetOrientation
        );
        if (!ikPlace.success) {
          this._triggerFailure('placement_failed', 'secured', ikPlace.message);
        }
        break;
      }

      case 'VERIFY_PLACEMENT': {
        this.statusMessage = 'VÉRIF. DÉPOSE : Vérification de la position, orientation et stabilité...';
        // Evaluated in step()
        break;
      }

      case 'COMPLETE': {
        this.statusMessage = 'TERMINÉ : Instrument chirurgical déposé et vérifié sur le plateau cible.';
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
            const graspQuat = this.failureController.getModifiedGraspOrientation(
          this.computeOptimalGraspQuaternion(inst)
        );
            const descendTarget = this.failureController.getModifiedApproachTarget([
              bodyPos.x,
              Math.max(bodyPos.y, 0.035),
              bodyPos.z
            ]);
            const descendIk = this.motionController.commandCartesianPose(
              descendTarget,
              graspQuat
            );
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
        this.failureController.onTransportStep(
          dt,
          this.gripper.getGraspFrameWorldPose().position.y
        );
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
          // Wait until robot has decelerated and settled cleanly at the placement pose
          if (this.motionController.hasReachedTarget(0.08) || this.stateTimerSeconds >= 1.2) {
            this.subPhase = 1;
            this.stateTimerSeconds = 0;
            if (this.failureController.shouldPreventFullRelease()) {
              // Jammed release: gripper stays partially closed and retains attachment
              this.gripper.targetOpeningMeters = 0.015;
            } else {
              this.graspManager.releaseGrasp();
              this.gripper.open(false);
            }
          }
        } else if (this.subPhase === 1) {
          // Allow 0.35s for fingers to open fully and object to settle flat under gravity
          if (this.stateTimerSeconds >= 0.35) {
            this.subPhase = 2;
            this.stateTimerSeconds = 0;
            this.motionController.commandCartesianPose([
              this.targetPosition[0],
              0.24,
              this.targetPosition[2]
            ]);
          }
        } else if (this.subPhase === 2) {
          // Wait for gripper to clear vertically before inspecting placement
          if (this.motionController.hasReachedTarget(0.15) || this.stateTimerSeconds >= 0.9) {
            this._transitionTo('VERIFY_PLACEMENT');
          }
        }
        break;
      }

      case 'VERIFY_PLACEMENT': {
        const report = this.placementVerifier.verify(inst, this.graspManager.isGrasping);
        this.lastPlacementReport = report;
        if (report.verified) {
          if (this.workZoneManager) {
            this.workZoneManager.markObjectPlacedAndStable(this.currentSequenceIndex, inst.config.id);
          }
          this.multiObjectCompletedCount++;

          if (this.isMultiObjectSequence) {
            const nextIdx = this.currentSequenceIndex + 1;
            if (nextIdx < 6) {
              this.statusMessage = `VÉRIFIÉ : ${inst.config.name} placé dans le Slot B${this.currentSequenceIndex + 1}. Passage à l'objet ${nextIdx + 1}/6...`;
              this._prepareAndStartObjectInSequence(nextIdx);
            } else {
              this.statusMessage = 'TERMINÉ : Les 6 objets ont été transférés avec succès de la Zone A vers la Zone B !';
              this._transitionTo('COMPLETE');
            }
          } else {
            this._transitionTo('COMPLETE');
          }
        } else if (this.stateTimerSeconds >= 0.65) {
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
    this.failureController.disarm();
    this.injectFailureNextGrasp = false;
    this.state = 'IDLE';
    this.stateHistory = ['IDLE'];
    this.subPhase = 0;
    this.stateTimerSeconds = 0;
    this.lastPlacementReport = null;
    this.lastOrchestratorResult = null;
    this.isMultiObjectSequence = false;
    this.currentSequenceIndex = 0;
    this.multiObjectCompletedCount = 0;
    if (this.workZoneManager) {
      this.workZoneManager.resetSlotOccupancy();
    }
    this.failureDetector.clearActiveFailure();
  }

  /**
   * Returns current 6-object sequence progress and status.
   */
  public getMultiObjectProgress(): {
    total: number;
    placed: number;
    currentObject: string | null;
    currentIndex: number;
    progressPercent: number;
    isSequenceRunning: boolean;
  } {
    const total = 6;
    const placed = this.workZoneManager?.verifiedPlacedObjectIds.size ?? this.multiObjectCompletedCount;
    const percent = Math.round((placed / total) * 100);
    const sixIds = ['Object_01', 'Object_02', 'Object_03', 'Object_04', 'Object_05', 'Object_06'];
    const currentId =
      this.isMultiObjectSequence && this.currentSequenceIndex < 6
        ? sixIds[this.currentSequenceIndex]
        : null;

    return {
      total,
      placed,
      currentObject: currentId,
      currentIndex: this.currentSequenceIndex,
      progressPercent: percent,
      isSequenceRunning:
        this.isMultiObjectSequence &&
        this.state !== 'IDLE' &&
        this.state !== 'COMPLETE' &&
        this.state !== 'FAILED'
    };
  }
}
