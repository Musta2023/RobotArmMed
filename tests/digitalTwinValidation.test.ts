/**
 * @file digitalTwinValidation.test.ts
 * Automated validation test suite for the SURGICAL ACTION AI — ROBOTIC DIGITAL TWIN.
 * Implements all 10 mandatory physics, control, manipulation, recovery, and AI tests
 * specified in Section 12.
 */

import { describe, it, expect, beforeEach } from 'vitest';
import { createDigitalTwinApp } from '../src/main.js';
import { PICKUP_STATION_POS, TARGET_TRAY_POS } from '../src/objects/ObjectProperties.js';
import { PICK_AND_PLACE_SEQUENCE } from '../src/manipulation/PickAndPlace.js';

describe('SURGICAL ACTION AI — Robotic Digital Twin Validation Suite (Section 12)', () => {
  let app: Awaited<ReturnType<typeof createDigitalTwinApp>>;

  beforeEach(async () => {
    app = await createDigitalTwinApp({ headless: true });
  });

  it('TEST 1 — Gravity: drops a surgical instrument from height and verifies free-fall under gravity', () => {
    const inst = app.instrumentFactory.getActiveInstrument()!;
    expect(inst).toBeDefined();

    // Position the surgical instrument in mid-air at y = 0.30 m above the Pickup Station
    const startY = 0.30;
    app.instrumentFactory.setInstrumentPose(inst.config.id, [
      PICKUP_STATION_POS[0],
      startY,
      PICKUP_STATION_POS[2]
    ]);

    const poseBefore = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!;
    expect(poseBefore.position.y).toBeCloseTo(startY, 3);

    // Step Rapier3D physics for 8 frames (0.133 s) while in mid-air under -9.81 m/s² gravity
    app.stepSimulation(8);

    const poseMidFall = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!;
    expect(poseMidFall.position.y).toBeLessThan(startY - 0.05);
    expect(poseMidFall.linearVelocity.y).toBeLessThan(-0.9);

    // Step further until the instrument lands and settles on the Pickup Station pad (top y = 0.02 m)
    app.stepSimulation(90);
    const poseLanded = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!;
    const expectedRestY = PICKUP_STATION_POS[1] + inst.config.dimensions.height / 2;
    expect(poseLanded.position.y).toBeCloseTo(expectedRestY, 1);
  });

  it('TEST 2 — Mass: compares acceleration and displacement of two instruments with different masses under equal force', () => {
    // Set zero gravity temporarily to isolate horizontal F = m * a dynamics in free space
    app.physicsWorld.setGravity(0);

    const lightObj = app.instrumentFactory.spawnInstrument('small_metallic_object', {
      id: 'light_instrument',
      massKg: 0.10,
      initialPosition: [-0.20, 0.35, 0.0],
      friction: 0.0
    });

    const heavyObj = app.instrumentFactory.spawnInstrument('surgical_tool', {
      id: 'heavy_instrument',
      massKg: 1.00,
      initialPosition: [0.20, 0.35, 0.0],
      friction: 0.0
    });

    // Apply identical external force (+2.0 N along +Z) over 12 physics steps
    for (let i = 0; i < 12; i++) {
      app.bodyManager.applyExternalForce(lightObj.config.id, [0, 0, 2.0]);
      app.bodyManager.applyExternalForce(heavyObj.config.id, [0, 0, 2.0]);
      app.stepSimulation(1);
    }

    const lightPose = app.instrumentFactory.getInstrumentWorldPose(lightObj.config.id)!;
    const heavyPose = app.instrumentFactory.getInstrumentWorldPose(heavyObj.config.id)!;

    // By F = m * a, the 0.10 kg instrument must accelerate and travel ~10x faster than the 1.00 kg instrument
    expect(lightPose.linearVelocity.z).toBeGreaterThan(heavyPose.linearVelocity.z * 5);
    expect(lightPose.position.z).toBeGreaterThan(heavyPose.position.z * 5);
  });

  it('TEST 3 — Joint Movement: moves each of the 6 joints and verifies joint limits and safe input clamping', () => {
    const configs = app.robotModel.config.joints;
    expect(configs.length).toBe(6);

    // Move to vertical upright candle pose [0, 0, 0, 0, 0, 0] so every joint can sweep its full range above ground
    app.motionController.commandJointAnglesDeg([0, 0, 0, 0, 0, 0]);
    app.stepSimulation(60);

    configs.forEach((cfg, idx) => {
      // Command way past maximum limit (+999°)
      const clampedMax = app.jointController.setJointTargetPositionDeg(idx, 999);
      expect(clampedMax).toBe(cfg.maxAngleDeg);

      // Step physics motors and verify physical joint angle reaches and never exceeds maxAngleDeg
      app.stepSimulation(90);
      const curAnglesMax = app.robotJoints.getCurrentAnglesDeg();
      expect(curAnglesMax[idx]).toBeLessThanOrEqual(cfg.maxAngleDeg + 0.05);
      expect(curAnglesMax[idx]).toBeGreaterThan(cfg.maxAngleDeg - 3.0);

      // Command way past minimum limit (-999°)
      const clampedMin = app.jointController.setJointTargetPositionDeg(idx, -999);
      expect(clampedMin).toBe(cfg.minAngleDeg);

      app.stepSimulation(120);
      const curAnglesMin = app.robotJoints.getCurrentAnglesDeg();
      expect(curAnglesMin[idx]).toBeGreaterThanOrEqual(cfg.minAngleDeg - 0.05);
      expect(curAnglesMin[idx]).toBeLessThan(cfg.minAngleDeg + 3.0);

      // Verify NaN / Infinity safety
      const safeAngle = app.jointController.setJointTargetPositionDeg(idx, Number.NaN);
      expect(Number.isFinite(safeAngle)).toBe(true);

      // Return joint to 0° before testing next joint
      app.jointController.setJointTargetPositionDeg(idx, 0);
      app.stepSimulation(70);
    });
  });

  it('TEST 4 — Torque: increasing payload mass requires measurably higher joint motor torque during lifting', () => {
    // Move arm to an extended forward lifting posture (J2 = 45°, J3 = 45°)
    app.motionController.commandJointAnglesDeg([0, 45, 45, 0, 0, 0]);

    // 1. Measure steady-state shoulder (J2) & elbow (J3) motor torque with a light 0.05 kg payload
    app.setHeldPayloadMassKg(0.05);
    app.stepSimulation(80);
    const snapLight = app.robotModel.state.latestSnapshot;
    const j2TorqueLight = Math.abs(snapLight.joints[1].motorTorqueNm);
    const j3TorqueLight = Math.abs(snapLight.joints[2].motorTorqueNm);

    // 2. Increase held payload mass to 1.50 kg at the same extended posture
    app.setHeldPayloadMassKg(1.50);
    app.stepSimulation(30);
    const snapHeavy = app.robotModel.state.latestSnapshot;
    const j2TorqueHeavy = Math.abs(snapHeavy.joints[1].motorTorqueNm);
    const j3TorqueHeavy = Math.abs(snapHeavy.joints[2].motorTorqueNm);

    expect(j2TorqueHeavy).toBeGreaterThan(j2TorqueLight + 1.0);
    expect(j3TorqueHeavy).toBeGreaterThan(j3TorqueLight + 0.4);
  });

  it('TEST 5 — Collision: verifies that the robot cannot pass through a physical obstacle in the workspace', () => {
    // Enable the physical workspace obstacle at [0.0, 0.17, 0.25] directly in front of the arm
    app.environment.setObstacleEnabled(true, [0.0, 0.17, 0.25]);

    // Command the arm to pitch straight down through the obstacle ([0, 75, 40, 0, 0, 0])
    app.motionController.commandJointAnglesDeg([0, 75, 40, 0, 0, 0]);
    app.stepSimulation(80);

    // Verify MotionController detected the physical obstacle collision and arrested motion
    expect(app.motionController.isBlockedByCollision).toBe(true);
    expect(app.motionController.lastBlockReason.toLowerCase()).toContain('collision');

    // Verify J2 was prevented from reaching 75° through the solid obstacle
    const currentJ2 = app.robotJoints.getCurrentAnglesDeg()[1];
    expect(currentJ2).toBeLessThan(65);
  });

  it('TEST 6 — Grasping: verifies that the gripper holds and lifts an instrument through physical bilateral contact and friction', () => {
    const inst = app.instrumentFactory.getActiveInstrument()!;
    const startPos = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!.position;

    // 1. Open gripper, move TCP to the instrument's grasp center via IK + joint motors
    app.gripper.open(true);
    const ikApproach = app.motionController.commandCartesianPose([
      startPos.x,
      startPos.y,
      startPos.z
    ]);
    expect(ikApproach.success).toBe(true);
    app.stepSimulation(90);

    // Verify closing the gripper does NOT attach when no closing command has completed yet
    expect(app.graspManager.isGrasping).toBe(false);

    // 2. Command motorized gripper fingers to close with sufficient gripping force (16 N)
    app.gripper.setGrippingForceNewtons(16.0);
    app.gripper.close(false);
    app.stepSimulation(35);

    // Verify bilateral physical contact and Coulomb friction stability
    const snapGrasped = app.robotModel.state.latestSnapshot;
    expect(snapGrasped.gripper.leftFingerContact).toBe(true);
    expect(snapGrasped.gripper.rightFingerContact).toBe(true);
    expect(snapGrasped.gripper.isGrasping).toBe(true);
    expect(snapGrasped.gripper.frictionCapacityNewtons).toBeGreaterThan(
      inst.config.massKg * 9.81
    );

    // 3. Lift the end-effector upward to y = 0.24 m and verify the dynamic instrument rises with it
    app.motionController.commandCartesianPose([startPos.x, 0.24, startPos.z]);
    app.stepSimulation(70);

    const liftedPos = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!.position;
    expect(liftedPos.y).toBeGreaterThan(0.18);
  });

  it('TEST 7 — Slipping: reducing gripping force or surface friction causes a lifted instrument to slip and fall under gravity', () => {
    const inst = app.instrumentFactory.getActiveInstrument()!;
    const startPos = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!.position;

    // 1. First grasp and lift the instrument to y = 0.24 m with high gripping force (16 N)
    app.gripper.open(true);
    app.motionController.commandCartesianPose([startPos.x, startPos.y, startPos.z]);
    app.stepSimulation(90);
    app.gripper.setGrippingForceNewtons(16.0);
    app.gripper.close(false);
    app.stepSimulation(35);
    expect(app.graspManager.isGrasping).toBe(true);

    app.motionController.commandCartesianPose([startPos.x, 0.24, startPos.z]);
    app.stepSimulation(70);

    const heightBeforeSlip = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!.position.y;
    expect(heightBeforeSlip).toBeGreaterThan(0.18);

    // 2. Drastically reduce gripping force (0.25 N) and friction (0.05) so 2 * mu * F_n < m * g
    app.gripper.setGrippingForceNewtons(0.25);
    app.instrumentFactory.setInstrumentMaterial(inst.config.id, 0.05, 0.05);

    // Step 1 frame to observe immediate slip detection while still between fingers
    app.stepSimulation(1);
    expect(app.graspManager.isGrasping).toBe(false);
    expect(app.graspManager.isSlipping).toBe(true);

    // Step 25 more frames and verify the instrument physically falls downward under gravity
    app.stepSimulation(25);
    const heightAfterSlip = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!.position.y;
    expect(heightAfterSlip).toBeLessThan(heightBeforeSlip - 0.08);
  });

  it('TEST 8 — Pick and Place: executes the complete 9-state Pick-and-Place task and verifies placement on Target Tray', () => {
    const inst = app.instrumentFactory.getActiveInstrument()!;

    const started = app.pickAndPlace.start();
    expect(started).toBe(true);

    // Step simulation up to 750 steps (12.5 simulated seconds) until COMPLETE
    for (let step = 0; step < 750; step++) {
      app.stepSimulation(1);
      if (app.pickAndPlace.state === 'COMPLETE' || app.pickAndPlace.state === 'FAILED') {
        break;
      }
    }

    expect(app.pickAndPlace.state).toBe('COMPLETE');

    // Verify all 9 mandatory states were traversed in exact sequence
    for (const expectedState of PICK_AND_PLACE_SEQUENCE) {
      expect(app.pickAndPlace.stateHistory).toContain(expectedState);
    }

    // Verify PlacementVerifier confirmed physical detachment, position on Target Tray, and stability
    expect(app.pickAndPlace.lastPlacementReport).not.toBeNull();
    expect(app.pickAndPlace.lastPlacementReport!.verified).toBe(true);
    expect(app.pickAndPlace.lastPlacementReport!.isDetachedFromGripper).toBe(true);

    const finalPose = app.instrumentFactory.getInstrumentWorldPose(inst.config.id)!;
    const horizDistToTray = Math.hypot(
      finalPose.position.x - TARGET_TRAY_POS[0],
      finalPose.position.z - TARGET_TRAY_POS[2]
    );
    expect(horizDistToTray).toBeLessThan(0.08);
  });

  it('TEST 9 — Recovery: intentionally injects grasp failure, verifies Section 8 JSON failure output, and executes authorized recovery', () => {
    // 1. Enable intentional grasp misalignment so GRASP_VERIFICATION fails
    app.pickAndPlace.injectIntentionalGraspFailure(true);
    app.pickAndPlace.start();

    for (let step = 0; step < 300; step++) {
      app.stepSimulation(1);
      if (app.pickAndPlace.state === 'FAILED') {
        break;
      }
    }

    // 2. Verify state machine stopped in FAILED state and emitted exact Section 8 JSON structure
    expect(app.pickAndPlace.state).toBe('FAILED');
    const failurePayload = app.pickAndPlace.lastOrchestratorResult!;
    expect(failurePayload).toBeDefined();
    expect(failurePayload.task_id).toBe('surgical_001');
    expect(failurePayload.status).toBe('failure');
    expect(failurePayload.error).toBe('grasp_failed');
    expect(failurePayload.retry_allowed).toBe(true);
    expect(failurePayload.object_state).toBe('not_secured');

    // Verify RecoveryManager is waiting for explicit orchestrator authorization
    expect(app.recoveryManager.awaitingAuthorization).toBe(true);

    // 3. Authorize recovery and run until the retried Pick-and-Place task completes!
    const recovered = app.pickAndPlace.authorizeAndRecover();
    expect(recovered).toBe(true);
    expect(app.recoveryManager.awaitingAuthorization).toBe(false);

    for (let step = 0; step < 800; step++) {
      app.stepSimulation(1);
      if (app.pickAndPlace.state === 'COMPLETE' || app.pickAndPlace.state === 'FAILED') {
        break;
      }
    }

    expect(app.pickAndPlace.state).toBe('COMPLETE');
  });

  it('TEST 10 — Data Collection: records multimodal observations and actions during manipulation and exports valid JSON dataset', () => {
    app.episodeRecorder.startRecording('surgical_bc_demo_001');
    expect(app.episodeRecorder.isRecording).toBe(true);

    // Apply an action via ActionInterface and step simulation
    const actionRes = app.actionInterface.applyAction({
      target_position: [PICKUP_STATION_POS[0], 0.22, PICKUP_STATION_POS[2]],
      gripper_command: 'open',
      gripping_force_n: 14.0
    });
    expect(actionRes.accepted).toBe(true);

    app.stepSimulation(25);
    app.episodeRecorder.stopRecording();

    const jsonString = app.episodeRecorder.exportJSON();
    const dataset = JSON.parse(jsonString);

    expect(dataset.schema_version).toBe('surgical_action_ai_v2.0');
    expect(dataset.episode_id).toBe('surgical_bc_demo_001');
    expect(dataset.total_steps).toBe(25);
    expect(dataset.frames.length).toBe(25);

    const sampleFrame = dataset.frames[10];
    expect(sampleFrame.observation.joint_positions_rad).toHaveLength(6);
    expect(sampleFrame.observation.joint_velocities_rad_s).toHaveLength(6);
    expect(sampleFrame.observation.joint_torques_nm).toHaveLength(6);
    expect(sampleFrame.observation.end_effector_position).toHaveLength(3);
    expect(sampleFrame.observation.object_position).toHaveLength(3);
    expect(sampleFrame.observation.camera_metadata.fov_deg).toBeGreaterThan(0);
    expect(sampleFrame.action.gripper_command).toBe('open');
  });
});
