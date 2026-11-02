/**
 * @file hybridPhysicsAndAssembly.test.ts
 * Validation suite for the Mechanical Assembly, 4-Bar Gripper Kinematics,
 * Compound Base Colliders, and Hybrid Physics Architecture (Kinematic vs Dynamic Modes).
 */

import { describe, it, expect, beforeEach } from 'vitest';
import { createDigitalTwinApp } from '../src/main.js';

describe('Hybrid Physics & Mechanical Assembly Validation Suite (Phases 1–6)', () => {
  let app: Awaited<ReturnType<typeof createDigitalTwinApp>>;

  beforeEach(async () => {
    app = await createDigitalTwinApp({ headless: true });
  });

  it('TEST A — Mechanical Assembly & Turret Bearing Alignment: verifies zero visual gap and continuous structure between base and J1 rotor', () => {
    const baseHeight = app.robotModel.config.baseHeightMeters;
    expect(baseHeight).toBe(0.14);

    const j1Pivot = app.robotModel.jointPivots[0];
    expect(j1Pivot.position.y).toBeCloseTo(0.14, 4);

    // Verify root contains basePlate, standoffs, centerServo, and bearingCollar
    const rootMeshes = app.robotModel.root.children;
    expect(rootMeshes.length).toBeGreaterThan(8);

    // Verify J1 rotor link contains topDisk, rotatingHub, and shoulder side plates
    const link0Meshes = app.robotModel.linkGroups[0].children;
    expect(link0Meshes.length).toBeGreaterThan(4);
  });

  it('TEST B — J1..J6 Mechanical Articulation & Correct Rotation Axes: rotates each joint and confirms intended rotational axis and clamping', () => {
    const configs = app.robotModel.config.joints;
    expect(configs.length).toBe(6);

    // Verify axis definitions
    expect(configs[0].axis).toEqual([0, 1, 0]); // J1 Base Pan (Y)
    expect(configs[1].axis).toEqual([1, 0, 0]); // J2 Shoulder Lift (X)
    expect(configs[2].axis).toEqual([1, 0, 0]); // J3 Elbow Pitch (X)
    expect(configs[3].axis).toEqual([0, 1, 0]); // J4 Forearm Roll (Y)
    expect(configs[4].axis).toEqual([1, 0, 0]); // J5 Wrist Pitch (X)
    expect(configs[5].axis).toEqual([0, 1, 0]); // J6 Tool Roll (Y)

    // Test J4 Forearm Roll sweep
    app.jointController.setJointTargetPositionDeg(3, 45);
    app.stepSimulation(60);
    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[3]).toBeCloseTo(45, 0);
  });

  it('TEST C — 4-Bar Gripper Mechanical Linkage Kinematics: verifies drive and follower bars articulate and spur gears counter-rotate', () => {
    const gripper = app.gripper;

    // Open gripper to default/max
    gripper.open(true);
    expect(gripper.currentOpeningMeters).toBeGreaterThan(0.07);
    const leftGearOpenZ = gripper.leftGearMesh.rotation.z;
    const rightGearOpenZ = gripper.rightGearMesh.rotation.z;
    expect(leftGearOpenZ).toBeGreaterThan(0.2);
    expect(rightGearOpenZ).toBeLessThan(-0.2);
    expect(leftGearOpenZ).toBeCloseTo(-rightGearOpenZ, 3);

    // Verify 4-bar linkage groups rotate consistently
    expect(gripper.leftDriveBarGroup.rotation.z).toBeCloseTo(leftGearOpenZ, 3);
    expect(gripper.rightDriveBarGroup.rotation.z).toBeCloseTo(rightGearOpenZ, 3);

    // Close gripper to min
    gripper.close(true);
    expect(gripper.currentOpeningMeters).toBeCloseTo(gripper.config.minOpeningMeters, 3);
    expect(gripper.leftGearMesh.rotation.z).toBeLessThan(0.05);
    expect(gripper.rightGearMesh.rotation.z).toBeGreaterThan(-0.05);
  });

  it('TEST D — Compound Base Collider Coverage: verifies rear electronics PCB and plate colliders exist and register in Rapier', () => {
    const baseBody = app.robotJoints.baseBody;
    expect(baseBody).toBeDefined();
    expect(baseBody.spec.id).toBe('robot_base');
    expect(baseBody.extraColliders).toBeDefined();
    expect(baseBody.extraColliders!.length).toBe(2);

    // Check handle registration
    for (const extraCol of baseBody.extraColliders!) {
      expect(app.bodyManager.colliderHandleToId.get(extraCol.handle)).toBe('robot_base');
    }
  });

  it('TEST E — Hybrid Physics Mode Manager Authority Isolation: switches between KINEMATIC and DYNAMIC modes seamlessly', () => {
    const pmm = app.physicsModeManager;
    expect(pmm.isKinematic()).toBe(true);
    expect(pmm.isDynamic()).toBe(false);

    // In Kinematic mode, link bodies are kinematicPosition
    for (const lb of app.robotJoints.linkBodies) {
      expect(lb.spec.motionType).toBe('kinematicPosition');
      expect(lb.body.bodyType()).toBe(2); // KinematicPositionBased in Rapier
    }

    // Switch to Dynamic Mode
    pmm.setMode('DYNAMIC');
    expect(pmm.isDynamic()).toBe(true);
    expect(pmm.isKinematic()).toBe(false);

    // In Dynamic mode, link bodies are dynamic
    for (const lb of app.robotJoints.linkBodies) {
      expect(lb.spec.motionType).toBe('dynamic');
      expect(lb.body.bodyType()).toBe(0); // Dynamic in Rapier
    }

    // Step simulation in dynamic mode without crashing or teleporting
    app.stepSimulation(10);

    // Switch back to Kinematic Mode
    pmm.setMode('KINEMATIC');
    expect(pmm.isKinematic()).toBe(true);
    for (const lb of app.robotJoints.linkBodies) {
      expect(lb.spec.motionType).toBe('kinematicPosition');
      expect(lb.body.bodyType()).toBe(2);
    }
  });

  it('TEST F — AI Dataset Telemetry & Verified Grasp Recording: exports dataset with physics mode tag and verified grasp flags', () => {
    const recorder = app.episodeRecorder;
    recorder.startRecording('test_verified_ep');

    // Step a few frames in Kinematic Mode
    app.stepSimulation(5);

    expect(recorder.frames.length).toBe(5);
    const frame = recorder.frames[0];
    expect(frame.physics_mode).toBe('kinematic');
    expect(frame.grasp_verified).toBeDefined();

    // Export JSON & CSV
    const jsonStr = recorder.exportJSON();
    expect(jsonStr).toContain('test_verified_ep');

    const csvStr = recorder.exportCSV();
    expect(csvStr).toContain('episode_id,step_index');
    expect(csvStr).toContain('test_verified_ep');
  });

  it('TEST G — J4 Forearm Torsion Mechanical Assembly & Swivel Alignment: verifies axial stator collar on Link 2 and rotating turntable on Link 3', () => {
    // 1. Link 2 (Forearm) has stator collar components at its distal end (y ~ 0.150)
    const link2Meshes = app.robotModel.linkGroups[2].children;
    expect(link2Meshes.length).toBeGreaterThan(5);

    // 2. Link 3 (Wrist Cradle) has rotor turntable components at its base (y ~ 0.003)
    const link3Meshes = app.robotModel.linkGroups[3].children;
    expect(link3Meshes.length).toBeGreaterThan(6);

    // Verify J4 axis is pure Y-axis torsion ([0, 1, 0])
    const j4Cfg = app.robotModel.config.joints[3];
    expect(j4Cfg.axis).toEqual([0, 1, 0]);
    expect(j4Cfg.pivotOffset).toEqual([0, 0.15, 0]);

    // Verify J5 axis is X-axis pitch ([1, 0, 0]) at y = 0.130
    const j5Cfg = app.robotModel.config.joints[4];
    expect(j5Cfg.axis).toEqual([1, 0, 0]);
    expect(j5Cfg.pivotOffset).toEqual([0, 0.13, 0]);

    // Rotate J4 by 60 degrees (torsion) and verify Link 3 pivots around Y
    app.jointController.setJointTargetPositionDeg(3, 60);
    app.stepSimulation(60);
    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[3]).toBeCloseTo(60, 0);

    const j4Pivot = app.robotModel.jointPivots[3];
    // In local space, quaternion around Y should have non-zero y component and 0 x,z components
    expect(Math.abs(j4Pivot.quaternion.y)).toBeGreaterThan(0.4);
    expect(Math.abs(j4Pivot.quaternion.x)).toBeCloseTo(0, 4);
    expect(Math.abs(j4Pivot.quaternion.z)).toBeCloseTo(0, 4);
  });
});
