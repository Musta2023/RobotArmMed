/**
 * @file failureScenarioValidation.test.ts
 * Automated Test Suite for the FailureScenarioController and Failure Data Collection Pipeline.
 */

import { describe, it, expect, beforeEach } from 'vitest';
import * as THREE from 'three';
import { createDigitalTwinApp } from '../src/main.js';
import {
  FailureScenarioController,
  FAILURE_SCENARIO_DEFINITIONS
} from '../src/manipulation/FailureScenarioController.js';

describe('Robotic Arm Failure Scenario & Data Collection Suite', () => {
  let app: Awaited<ReturnType<typeof createDigitalTwinApp>>;
  let controller: FailureScenarioController;

  beforeEach(async () => {
    app = await createDigitalTwinApp({ headless: true });
    controller = app.pickAndPlace.failureController;
  });

  // TEST 1: Initial state & nominal mode
  it('TEST 1 — Default State: controller initializes in nominal mode without fault injection', () => {
    expect(controller.activeType).toBe('NONE');
    expect(controller.resolvedType).toBe('NONE');
    expect(controller.isInjecting).toBe(false);
    expect(controller.faultTriggered).toBe(false);

    const target: [number, number, number] = [0.30, 0.04, 0.20];
    const modified = controller.getModifiedApproachTarget(target);
    expect(modified).toEqual(target);

    const alt = controller.getModifiedTransportAltitude(0.24);
    expect(alt).toBe(0.24);
  });

  // TEST 2: MISSED_APPROACH spatial offset
  it('TEST 2 — Missed Approach: injects 9.5 cm lateral offset causing jaws to close on empty air', () => {
    controller.armScenario('MISSED_APPROACH');
    expect(controller.isInjecting).toBe(true);
    expect(controller.resolvedType).toBe('MISSED_APPROACH');

    const nominal: [number, number, number] = [0.25, 0.04, 0.20];
    const modified = controller.getModifiedApproachTarget(nominal);
    expect(modified[0]).toBeCloseTo(nominal[0] + 0.095, 3);
    expect(modified[2]).toBeCloseTo(nominal[2] + 0.030, 3);
    expect(modified[1]).toBe(nominal[1]);
  });

  // TEST 3: SLIP_LOW_FRICTION weak grip force
  it('TEST 3 — Low Friction Slip: reduces grip force to 0.35 N causing insufficient friction', () => {
    controller.armScenario('SLIP_LOW_FRICTION');
    expect(controller.isInjecting).toBe(true);

    controller.onGraspEnter();
    expect(controller.faultTriggered).toBe(true);
    expect(app.gripper.grippingForceNewtons).toBeCloseTo(0.35, 2);

    // Disarm restores normal force
    controller.disarm();
    expect(app.gripper.grippingForceNewtons).toBe(app.gripper.config.defaultGrippingForceNewtons);
  });

  // TEST 4: DROP_IN_TRANSIT mid-air release
  it('TEST 4 — Drop In Transit: opens gripper after 0.6 seconds of transport time', () => {
    controller.armScenario('DROP_IN_TRANSIT');
    app.gripper.close(true);
    expect(app.gripper.isClosingCommandActive).toBe(true);

    // Simulate 0.3s of transport (should not drop yet)
    controller.onTransportStep(0.3, 0.24);
    expect(controller.faultTriggered).toBe(false);
    expect(app.gripper.isClosingCommandActive).toBe(true);

    // Advance past 0.6s threshold
    controller.onTransportStep(0.35, 0.24);
    expect(controller.faultTriggered).toBe(true);
    expect(app.gripper.isClosingCommandActive).toBe(false);
  });

  // TEST 5: COLLISION_OBSTACLE low altitude transit
  it('TEST 5 — Collision Obstacle: forces dangerous 0.045 m transit altitude crossing obstacles', () => {
    controller.armScenario('COLLISION_OBSTACLE');
    const dangerousAlt = controller.getModifiedTransportAltitude(0.24);
    expect(dangerousAlt).toBe(0.045);
    expect(controller.faultTriggered).toBe(true);
  });

  // TEST 6: JOINT_LIMIT_REACH unreachable targets
  it('TEST 6 — Joint Limit Reach: shifts coordinates beyond reachable kinematic boundaries', () => {
    controller.armScenario('JOINT_LIMIT_REACH');
    const nominal: [number, number, number] = [0.30, 0.24, 0.20];
    const extreme = controller.getModifiedApproachTarget(nominal);
    expect(extreme[0]).toBeGreaterThan(0.80);
  });

  // TEST 7: EXCESSIVE_TORQUE payload overload
  it('TEST 7 — Excessive Torque: injects 3.8 kg overload exceeding rated payload', () => {
    controller.armScenario('EXCESSIVE_TORQUE');
    const simulatedMass = controller.getSimulatedPayloadMassKg(0.18);
    expect(simulatedMass).toBe(3.8);
    expect(controller.faultTriggered).toBe(true);
  });

  // TEST 8: TIPPED_PLACEMENT high release with tilt
  it('TEST 8 — Tipped Placement: elevates release to 12 cm with 38-degree tilt angle', () => {
    controller.armScenario('TIPPED_PLACEMENT');
    const nominalSlot: [number, number, number] = [0.38, 0.02, 0.29];
    const nominalQuat = new THREE.Quaternion(0, 0, 0, 1);

    const placeConfig = controller.getModifiedPlacementPose(nominalSlot, nominalQuat);
    expect(placeConfig.targetPosition[1]).toBeCloseTo(nominalSlot[1] + 0.120, 3);
    expect(placeConfig.targetOrientation.equals(nominalQuat)).toBe(false);
    expect(controller.faultTriggered).toBe(true);
  });

  // TEST 9: KNOCK_ON_APPROACH low descent impact
  it('TEST 9 — Knock On Approach: forces descent 4.5 cm below nominal hitting object directly', () => {
    controller.armScenario('KNOCK_ON_APPROACH');
    const nominal: [number, number, number] = [0.25, 0.05, 0.20];
    const target = controller.getModifiedApproachTarget(nominal);
    expect(target[1]).toBeCloseTo(nominal[1] - 0.045, 3);
    expect(controller.faultTriggered).toBe(true);
  });

  // TEST 10: ORIENTATION_MISALIGNMENT roll error
  it('TEST 10 — Orientation Misalignment: applies 45-degree angle offset to grasp pose', () => {
    controller.armScenario('ORIENTATION_MISALIGNMENT');
    const nominalQuat = new THREE.Quaternion(0, 0, 0, 1);
    const modifiedQuat = controller.getModifiedGraspOrientation(nominalQuat);
    expect(modifiedQuat.equals(nominalQuat)).toBe(false);
    expect(controller.faultTriggered).toBe(true);
  });

  // TEST 11: DYNAMIC_JERK_SLIP transit jerk
  it('TEST 11 — Dynamic Jerk Slip: drops grip force during transit to trigger slip', () => {
    controller.armScenario('DYNAMIC_JERK_SLIP');
    controller.onTransportStep(0.2, 0.24);
    expect(controller.faultTriggered).toBe(false);

    controller.onTransportStep(0.2, 0.24); // total 0.40s >= 0.35s
    expect(controller.faultTriggered).toBe(true);
    expect(app.gripper.grippingForceNewtons).toBeCloseTo(0.1, 2);
  });

  // TEST 12: INCOMPLETE_RELEASE jammed gripper
  it('TEST 12 — Incomplete Release: flags jammed release preventing full opening', () => {
    controller.armScenario('INCOMPLETE_RELEASE');
    expect(controller.shouldPreventFullRelease()).toBe(true);
    expect(controller.faultTriggered).toBe(true);
  });

  // TEST 13: RANDOM_FAILURE resolution
  it('TEST 13 — Random Failure: resolves randomly across all 11 deterministic failure types', () => {
    controller.setScenarioType('RANDOM_FAILURE');
    expect(controller.activeType).toBe('RANDOM_FAILURE');
    expect(controller.resolvedType).not.toBe('NONE');
    expect(controller.resolvedType).not.toBe('RANDOM_FAILURE');

    const validTypes = Object.keys(FAILURE_SCENARIO_DEFINITIONS).filter(
      (k) => k !== 'NONE' && k !== 'RANDOM_FAILURE'
    );
    expect(validTypes).toContain(controller.resolvedType);
    expect(validTypes.length).toBe(11);
  });

  // TEST 14: Complete Failure Episode Recording & Dataset Export
  it('TEST 14 — Labeled Failure Data Collection: records episode with failure_mode metadata', () => {
    app.episodeRecorder.clear();
    app.episodeRecorder.startRecording('failure_test_ep_001', undefined, 'SLIP_LOW_FRICTION');
    expect(app.episodeRecorder.activeFailureMode).toBe('SLIP_LOW_FRICTION');

    // Step a few frames to register data
    for (let i = 0; i < 5; i++) {
      app.stepSimulation(1 / 20);
    }

    app.episodeRecorder.stopRecording('failure');
    const dataset = app.episodeRecorder.exportDataset();

    expect(dataset.outcome).toBe('failure');
    expect(dataset.failure_mode).toBe('SLIP_LOW_FRICTION');

    const bundle = app.episodeRecorder.exportConsolidatedBundle();
    expect(bundle.episodes.length).toBeGreaterThanOrEqual(1);
    expect(bundle.episodes[0].failure_mode).toBe('SLIP_LOW_FRICTION');
  });
});
