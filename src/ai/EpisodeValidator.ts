/**
 * @file EpisodeValidator.ts
 * Rigorous 7-stage automated episode evaluation and metric calculation system
 * for robotic manipulation training demonstrations (Phase E).
 *
 * Evaluates:
 * 1. Reach: robot approaches object within contact threshold
 * 2. Grasp: physical bilateral contact and normal clamping force verified
 * 3. Lift: object cleanly elevated from support surface without slipping
 * 4. Transport: object transported across workspace maintaining grasp
 * 5. Placement: object positioned inside target slot boundary
 * 6. Release: gripper opens and detaches cleanly without tipping
 * 7. Stability: object remains stationary and upright under gravity
 */

import * as THREE from 'three';
import { DigitalTwinObservation } from './ObservationBuilder.js';

export interface StageStatus {
  reached: boolean;
  timestamp: number | null;
  error_metric: number;
}

export interface EpisodeValidationReport {
  episode_id: string;
  is_valid: boolean;
  outcome: 'success' | 'failure' | 'incomplete';
  failure_reason: string | null;

  stages: {
    reach: StageStatus;
    grasp: StageStatus;
    lift: StageStatus;
    transport: StageStatus;
    placement: StageStatus;
    release: StageStatus;
    stability: StageStatus;
  };

  metrics: {
    grasp_success: boolean;
    lift_success: boolean;
    placement_success: boolean;
    final_position_error_m: number;
    final_orientation_error_rad: number;
    collision_count: number;
    slipping_events: number;
    episode_duration_seconds: number;
  };
}

export class EpisodeValidator {
  public positionToleranceMeters: number = 0.045;
  public stabilityVelocityThresholdMps: number = 0.02;
  public minLiftElevationMeters: number = 0.03;

  /**
   * Evaluates an entire trajectory of recorded observations.
   */
  public evaluateEpisode(
    episodeId: string,
    observations: DigitalTwinObservation[],
    prohibitedCollisionCount: number = 0
  ): EpisodeValidationReport {
    if (!observations || observations.length === 0) {
      return this._createEmptyReport(episodeId, 'No observation frames in episode.');
    }

    const firstFrame = observations[0];
    const lastFrame = observations[observations.length - 1];
    const duration = lastFrame.timestamp - firstFrame.timestamp;

    const initialObjY = firstFrame.object_position[1];
    const targetPos = new THREE.Vector3(...lastFrame.target_position);

    let reachedStep: number | null = null;
    let graspedStep: number | null = null;
    let liftedStep: number | null = null;
    let transportedStep: number | null = null;
    let placedStep: number | null = null;
    let releasedStep: number | null = null;
    let stableStep: number | null = null;

    let slipCount = 0;
    let maxLiftHeight = 0;
    let wasEverGrasped = false;

    // Track stages chronologically
    for (let i = 0; i < observations.length; i++) {
      const obs = observations[i];
      const tcp = new THREE.Vector3(...obs.end_effector_position);
      const objPos = new THREE.Vector3(...obs.object_position);
      const distToObj = tcp.distanceTo(objPos);

      // Track slipping events
      if (obs.gripper_state.is_slipping) {
        slipCount++;
      }

      // Stage 1: Reach (TCP within 35mm of object)
      if (reachedStep === null && distToObj <= 0.038) {
        reachedStep = i;
      }

      // Stage 2: Grasp (Physical grasp established)
      if (graspedStep === null && reachedStep !== null && obs.gripper_state.is_grasping) {
        graspedStep = i;
        wasEverGrasped = true;
      }

      // Stage 3: Lift (Object elevated by minLiftElevationMeters)
      const liftDelta = objPos.y - initialObjY;
      if (liftDelta > maxLiftHeight) maxLiftHeight = liftDelta;

      if (
        liftedStep === null &&
        graspedStep !== null &&
        liftDelta >= this.minLiftElevationMeters &&
        obs.gripper_state.is_grasping
      ) {
        liftedStep = i;
      }

      // Stage 4: Transport (Object moved towards target tray while grasped)
      const distToTarget = new THREE.Vector2(objPos.x - targetPos.x, objPos.z - targetPos.z).length();
      if (
        transportedStep === null &&
        liftedStep !== null &&
        distToTarget <= 0.12 &&
        obs.gripper_state.is_grasping
      ) {
        transportedStep = i;
      }

      // Stage 5: Placement (Object in target zone)
      if (
        placedStep === null &&
        transportedStep !== null &&
        distToTarget <= this.positionToleranceMeters
      ) {
        placedStep = i;
      }

      // Stage 6: Release (Gripper opened and detached after placing)
      if (
        releasedStep === null &&
        placedStep !== null &&
        !obs.gripper_state.is_grasping &&
        obs.gripper_state.opening_meters > 0.02
      ) {
        releasedStep = i;
      }

      // Stage 7: Stability (Object stationary on target surface after release)
      if (releasedStep !== null && i >= releasedStep + 5) {
        const lv = obs.object_linear_velocity ?? [0, 0, 0];
        const linSpeed = Math.hypot(lv[0], lv[1], lv[2]);
        if (linSpeed <= this.stabilityVelocityThresholdMps && distToTarget <= this.positionToleranceMeters) {
          stableStep = i;
        }
      }
    }

    // Final position & orientation errors
    const finalObjPos = new THREE.Vector3(...lastFrame.object_position);
    const finalHorizErr = Math.hypot(
      finalObjPos.x - targetPos.x,
      finalObjPos.z - targetPos.z
    );

    const finalQuat = new THREE.Quaternion(...lastFrame.object_orientation);
    const localUp = new THREE.Vector3(0, 1, 0).applyQuaternion(finalQuat);
    const uprightCos = Math.abs(localUp.y);
    const finalOriErrRad = Math.acos(Math.min(Math.max(uprightCos, 0), 1));

    // Determine outcome and failure reason
    let outcome: 'success' | 'failure' | 'incomplete' = 'incomplete';
    let failureReason: string | null = null;

    if (prohibitedCollisionCount > 0) {
      outcome = 'failure';
      failureReason = `prohibited_collision: ${prohibitedCollisionCount} unauthorized collision event(s).`;
    } else if (reachedStep === null) {
      outcome = 'failure';
      failureReason = 'reach_failed: Robot failed to approach within contact radius.';
    } else if (graspedStep === null) {
      outcome = 'failure';
      failureReason = 'grasp_failed: Gripper failed to achieve bilateral physical contact.';
    } else if (liftedStep === null) {
      outcome = 'failure';
      failureReason = 'lift_failed: Object was not lifted cleanly off support plane.';
    } else if (transportedStep === null) {
      outcome = 'failure';
      failureReason = 'transport_failed: Object slipped or was lost during transit.';
    } else if (placedStep === null) {
      outcome = 'failure';
      failureReason = 'placement_failed: Object did not enter target slot radius.';
    } else if (releasedStep === null) {
      outcome = 'failure';
      failureReason = 'release_failed: Gripper did not release object cleanly.';
    } else if (stableStep === null || finalHorizErr > this.positionToleranceMeters || uprightCos < 0.75) {
      outcome = 'failure';
      failureReason = uprightCos < 0.75
        ? 'unstable_orientation: Object tipped over or misaligned on tray.'
        : `placement_error_exceeded: Final position error ${(finalHorizErr * 1000).toFixed(1)} mm > tolerance.`;
    } else {
      outcome = 'success';
      failureReason = null;
    }

    return {
      episode_id: episodeId,
      is_valid: outcome === 'success',
      outcome,
      failure_reason: failureReason,
      stages: {
        reach: {
          reached: reachedStep !== null,
          timestamp: reachedStep !== null ? observations[reachedStep].timestamp : null,
          error_metric: reachedStep !== null ? 0 : 1
        },
        grasp: {
          reached: graspedStep !== null,
          timestamp: graspedStep !== null ? observations[graspedStep].timestamp : null,
          error_metric: wasEverGrasped ? 0 : 1
        },
        lift: {
          reached: liftedStep !== null,
          timestamp: liftedStep !== null ? observations[liftedStep].timestamp : null,
          error_metric: Math.max(0, this.minLiftElevationMeters - maxLiftHeight)
        },
        transport: {
          reached: transportedStep !== null,
          timestamp: transportedStep !== null ? observations[transportedStep].timestamp : null,
          error_metric: transportedStep !== null ? 0 : 1
        },
        placement: {
          reached: placedStep !== null,
          timestamp: placedStep !== null ? observations[placedStep].timestamp : null,
          error_metric: finalHorizErr
        },
        release: {
          reached: releasedStep !== null,
          timestamp: releasedStep !== null ? observations[releasedStep].timestamp : null,
          error_metric: releasedStep !== null ? 0 : 1
        },
        stability: {
          reached: stableStep !== null,
          timestamp: stableStep !== null ? observations[stableStep].timestamp : null,
          error_metric: finalOriErrRad
        }
      },
      metrics: {
        grasp_success: graspedStep !== null,
        lift_success: liftedStep !== null,
        placement_success: placedStep !== null && finalHorizErr <= this.positionToleranceMeters,
        final_position_error_m: finalHorizErr,
        final_orientation_error_rad: finalOriErrRad,
        collision_count: prohibitedCollisionCount,
        slipping_events: slipCount,
        episode_duration_seconds: duration
      }
    };
  }

  private _createEmptyReport(episodeId: string, reason: string): EpisodeValidationReport {
    const dummyStage: StageStatus = { reached: false, timestamp: null, error_metric: 1 };
    return {
      episode_id: episodeId,
      is_valid: false,
      outcome: 'failure',
      failure_reason: reason,
      stages: {
        reach: dummyStage,
        grasp: dummyStage,
        lift: dummyStage,
        transport: dummyStage,
        placement: dummyStage,
        release: dummyStage,
        stability: dummyStage
      },
      metrics: {
        grasp_success: false,
        lift_success: false,
        placement_success: false,
        final_position_error_m: Infinity,
        final_orientation_error_rad: Infinity,
        collision_count: 0,
        slipping_events: 0,
        episode_duration_seconds: 0
      }
    };
  }
}

