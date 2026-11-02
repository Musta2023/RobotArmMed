/**
 * @file mouseInteractionValidation.test.ts
 * Automated validation test suite for Section 10 (Tests 1–9):
 * Full 3D Mouse Interaction, Free 3D Object Positioning, Direct Gripper Target Control
 * (6D Translation + Orientation via IK & Joint Motors), Object-Independent Manipulation,
 * Interaction Modes (Object / Robot / AI), and Demonstration Dataset Recording (JSON + CSV).
 */

import { describe, it, expect, beforeEach, afterEach } from 'vitest';
import { createDigitalTwinApp } from '../src/main.js';
import { PICKUP_STATION_POS } from '../src/objects/ObjectProperties.js';

describe('Section 10 — Full 3D Mouse Interaction & Robotic Manipulation Suite (Tests 1–9)', () => {
  let app: Awaited<ReturnType<typeof createDigitalTwinApp>>;

  beforeEach(async () => {
    app = await createDigitalTwinApp({ headless: true });
  });

  afterEach(() => {
    app.dispose();
  });

  it('TEST 1 — Mouse Object Grab, Lift & Release: selects a box, lifts it in 3D, and releases it to fall naturally under gravity', () => {
    app.interactionModeManager.setMode('object');
    const inst = app.instrumentFactory.getActiveInstrument()!;

    // 1. Hover & grab the box
    app.objectManipulator.setHoveredObject(inst.id);
    expect(app.objectManipulator.hoveredObjectId).toBe(inst.id);

    const grabbed = app.objectManipulator.grabObject(inst.id);
    expect(grabbed).not.toBeNull();
    expect(app.objectManipulator.isDragging).toBe(true);

    // 2. Lift the box to y = 0.28 m using the physics-based drag controller
    app.objectManipulator.moveGrabbedObjectTo([
      PICKUP_STATION_POS[0],
      0.28,
      PICKUP_STATION_POS[2]
    ]);
    app.stepSimulation(35);

    const liftedY = inst.getPosition()[1];
    expect(liftedY).toBeGreaterThan(0.24);

    // 3. Release the mouse: verify object is dynamic and falls naturally under gravity
    app.objectManipulator.releaseObject();
    expect(app.objectManipulator.isDragging).toBe(false);
    expect(inst.managedBody.body.isDynamic()).toBe(true);

    // Step 12 frames: verify downward free-fall velocity
    app.stepSimulation(12);
    expect(inst.getPosition()[1]).toBeLessThan(liftedY - 0.05);
    expect(inst.getLinearVelocity()[1]).toBeLessThan(-0.4);

    // Step 100 frames: verify natural settling on the pickup platform (y ≈ 0.041 m)
    app.stepSimulation(100);
    const settledPos = inst.getPosition();
    expect(settledPos[1]).toBeGreaterThan(0.036);
    expect(settledPos[1]).toBeLessThan(0.048);
    expect(inst.getSpeed()).toBeLessThan(0.03);
  });

  it('TEST 2 — Floor Dragging: drags an object across the floor without passing through the floor', () => {
    app.interactionModeManager.setMode('object');
    const inst = app.instrumentFactory.getActiveInstrument()!;

    // Place object on open floor at [-0.15, 0.022, 0.22]
    app.instrumentFactory.setInstrumentPose(inst.id, [-0.15, 0.022, 0.22]);
    app.stepSimulation(10);

    app.objectManipulator.grabObject(inst.id);

    // Drag across the floor from x = -0.15 m to x = +0.15 m while commanding a negative Y (-0.10 m)
    for (let step = 0; step < 40; step++) {
      const targetX = -0.15 + (0.30 * (step + 1)) / 40;
      app.objectManipulator.moveGrabbedObjectTo([targetX, -0.10, 0.22]);
      app.stepSimulation(1);

      const pos = inst.getPosition();
      // Object half-height is 0.021 m -> center Y must never pass through the floor (y = 0)
      expect(pos[1]).toBeGreaterThan(0.018);
    }

    expect(inst.getPosition()[0]).toBeGreaterThan(0.08);
    app.objectManipulator.releaseObject();
  });

  it('TEST 3 — Vertical Move & Stacking Release: moves an object vertically and releases it above another object to rest on top', () => {
    app.interactionModeManager.setMode('object');
    const bottomBox = app.instrumentFactory.getActiveInstrument()!;

    // Spawn a second box to grab and position above bottomBox
    const topBox = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'mouse_stack_top',
      massKg: 0.15,
      position: [0.0, 0.022, 0.24]
    });
    app.stepSimulation(15);

    // Grab topBox, move it horizontally and vertically above bottomBox, then release
    app.objectManipulator.grabObject(topBox.id);
    const basePos = bottomBox.getPosition();
    app.objectManipulator.moveGrabbedObjectTo([basePos[0], basePos[1] + 0.10, basePos[2]]);
    app.stepSimulation(35);

    expect(topBox.getPosition()[1]).toBeGreaterThan(basePos[1] + 0.07);

    // Release topBox so it falls onto bottomBox
    app.objectManipulator.releaseObject();
    app.stepSimulation(110);

    const finalBottomY = bottomBox.getPosition()[1];
    const finalTopY = topBox.getPosition()[1];

    // Both boxes are 0.042 m tall -> topBox rests stably on bottomBox
    expect(finalTopY - finalBottomY).toBeGreaterThan(0.038);
    expect(finalTopY - finalBottomY).toBeLessThan(0.047);
    expect(topBox.getSpeed()).toBeLessThan(0.03);
  });

  it('TEST 4 — Direct Gripper Target Control: selects Robot Mode and moves the gripper target via IK and joint motors', () => {
    app.interactionModeManager.setMode('robot');
    expect(app.interactionModeManager.mode).toBe('robot');
    expect(app.gripperTargetController.isGizmoActive).toBe(true);

    const desiredTarget: [number, number, number] = [0.22, 0.20, 0.24];
    const ikRes = app.gripperTargetController.setTargetPosition(desiredTarget);
    expect(ikRes.isReachable).toBe(true);

    // Step simulation so physical joint PD motors drive the arm to the new target
    app.stepSimulation(90);

    const eePos = app.robotModel.getEndEffectorPosition();
    const err = Math.hypot(
      eePos[0] - desiredTarget[0],
      eePos[1] - desiredTarget[1],
      eePos[2] - desiredTarget[2]
    );
    expect(err).toBeLessThan(0.025);
  });

  it('TEST 5 — Gripper Orientation Control: rotates the gripper using the 3D gizmo while respecting joint limits', () => {
    app.interactionModeManager.setMode('robot');
    app.interactionModeManager.setGizmoTransformMode('rotate');
    expect(app.gripperTargetController.gizmoMode).toBe('rotate');

    // Position gripper in central reachable workspace first
    app.gripperTargetController.setTargetPosition([0.0, 0.24, 0.26]);
    app.stepSimulation(70);

    const beforeSnap = app.robotModel.state.latestSnapshot;
    const initialJ6Deg = beforeSnap.joints[5].currentAngleDeg;

    // Rotate the gripper target around its yaw/tool axis by +35 deg and pitch by +15 deg
    const rotRes = app.gripperTargetController.setTargetOrientationEulerDeg([15, 35, 0], true);
    expect(rotRes.isReachable).toBe(true);

    app.stepSimulation(80);

    const afterSnap = app.robotModel.state.latestSnapshot;
    // Verify gripper orientation and wrist/tool joints changed physically
    expect(Math.abs(afterSnap.joints[5].currentAngleDeg - initialJ6Deg)).toBeGreaterThan(8);

    // Verify all 6 joints strictly respect their configured min/max limits
    for (const jt of afterSnap.joints) {
      expect(jt.currentAngleDeg).toBeGreaterThanOrEqual(jt.minAngleDeg - 0.1);
      expect(jt.currentAngleDeg).toBeLessThanOrEqual(jt.maxAngleDeg + 0.1);
    }
  });

  it('TEST 6 — Object-Independent Manipulation at Random Workspace Position: places an object at a new position and approaches/grasps it', () => {
    // Spawn a cylindrical instrument at an arbitrary workspace coordinate [0.18, 0.029, 0.25]
    const randomPos: [number, number, number] = [0.18, 0.029, 0.25];
    const cylObj = app.instrumentFactory.spawnInstrument('cylindrical_instrument', {
      id: 'random_workspace_cylinder',
      massKg: 0.14,
      position: randomPos
    });
    app.instrumentFactory.activeInstrumentId = cylObj.id;
    app.stepSimulation(20);

    // Verify generic object interface & graspability evaluation
    expect(cylObj.id).toBe('random_workspace_cylinder');
    expect(cylObj.getMass()).toBeCloseTo(0.14, 3);
    expect(cylObj.getDimensions().width).toBeGreaterThan(0.02);
    expect(cylObj.getCollider()).toBeDefined();
    const graspInfo = cylObj.getGraspabilityInfo();
    expect(graspInfo.isGraspable).toBe(true);

    // Command robot to approach the object at its live random position
    app.gripper.open(false);
    const livePos = cylObj.getPosition();
    app.gripperTargetController.setTargetPosition([livePos[0], 0.22, livePos[2]]);
    app.stepSimulation(75);

    const approachRes = app.interactionModeManager.commandRobotToApproachObject(cylObj.id);
    expect(approachRes.accepted).toBe(true);
    app.stepSimulation(85);

    // Close gripper to grasp the cylindrical object at the new location
    app.gripper.close(false);
    app.stepSimulation(45);

    expect(app.graspManager.isGrasping).toBe(true);
    expect(app.graspManager.graspedInstrumentId).toBe(cylObj.id);
  });

  it('TEST 7 — Grasp, Move with Gripper Gizmo & Release: object follows gripper during grasp and falls naturally after release', () => {
    const inst = app.instrumentFactory.getActiveInstrument()!;
    const startPos = inst.getPosition();

    // 1. Move gripper to object using GripperTargetController and grasp it
    app.interactionModeManager.setMode('robot');
    app.gripper.open(false);
    app.gripperTargetController.setTargetPosition([startPos[0], 0.22, startPos[2]]);
    app.stepSimulation(75);
    app.gripperTargetController.setTargetPosition([startPos[0], startPos[1], startPos[2]]);
    app.stepSimulation(85);

    app.gripper.close(false);
    app.stepSimulation(45);
    expect(app.graspManager.isGrasping).toBe(true);

    // 2. Move the gripper target to a new 3D location [0.16, 0.22, 0.24]
    const transportTarget: [number, number, number] = [0.16, 0.22, 0.24];
    app.gripperTargetController.setTargetPosition(transportTarget);
    app.stepSimulation(95);

    const carriedPos = inst.getPosition();
    expect(carriedPos[0]).toBeGreaterThan(0.05);
    expect(carriedPos[1]).toBeGreaterThan(0.14);
    expect(app.graspManager.isGrasping).toBe(true);

    // 3. Open the gripper and verify the object is released and falls naturally under gravity
    app.gripper.open(false);
    app.stepSimulation(15);
    expect(app.graspManager.isGrasping).toBe(false);

    app.stepSimulation(100);
    const settledPos = inst.getPosition();
    expect(settledPos[1]).toBeLessThan(0.055);
    expect(settledPos[1]).toBeGreaterThan(0.015);
    expect(inst.getSpeed()).toBeLessThan(0.04);
  });

  it('TEST 8 — Unreachable Gripper Target: stops at a valid reachable pose with warning and never breaks joint limits', () => {
    app.interactionModeManager.setMode('robot');

    // Command an unreachable position far outside the 0.79m workspace
    const unreachablePos: [number, number, number] = [1.65, 1.35, 1.50];
    const res = app.gripperTargetController.setTargetPosition(unreachablePos);

    expect(res.isReachable).toBe(false);
    expect(res.status).toBe('UNREACHABLE');
    expect(app.gripperTargetController.isTargetReachable).toBe(false);
    expect(app.gripperTargetController.warningMessage).toContain('WARNING');

    // Step simulation and verify robot stops smoothly at a valid reachable pose within all joint limits
    app.stepSimulation(90);

    const snap = app.robotModel.state.latestSnapshot;
    const eePos = snap.endEffector.position;
    expect(Number.isFinite(eePos[0])).toBe(true);
    expect(Number.isFinite(eePos[1])).toBe(true);
    expect(Number.isFinite(eePos[2])).toBe(true);

    for (const jt of snap.joints) {
      expect(Number.isFinite(jt.currentAngleDeg)).toBe(true);
      expect(jt.currentAngleDeg).toBeGreaterThanOrEqual(jt.minAngleDeg - 0.1);
      expect(jt.currentAngleDeg).toBeLessThanOrEqual(jt.maxAngleDeg + 0.1);
    }
  });

  it('TEST 9 — Manual Pick-and-Place Demonstration Recording & Export (JSON + CSV): records full sequence including initial/final poses and trajectory', () => {
    const inst = app.instrumentFactory.getActiveInstrument()!;

    // 1. Start recording a manual demonstration
    app.episodeRecorder.startRecording('manual_demo_pick_place');
    expect(app.episodeRecorder.isRecording).toBe(true);

    // 2. Reposition object slightly with mouse in Object Mode, then switch to Robot Mode
    app.interactionModeManager.setMode('object');
    app.objectManipulator.grabObject(inst.id);
    app.objectManipulator.moveGrabbedObjectTo([-0.26, 0.042, 0.24]);
    app.stepSimulation(20);
    app.objectManipulator.releaseObject();
    app.stepSimulation(15);

    // 3. Move gripper via GripperTargetController, grasp, lift, move, and release
    app.interactionModeManager.setMode('robot');
    const pickPos = inst.getPosition();
    app.gripper.open(false);
    app.gripperTargetController.setTargetPosition([pickPos[0], 0.22, pickPos[2]]);
    app.stepSimulation(70);
    app.gripperTargetController.setTargetPosition([pickPos[0], pickPos[1], pickPos[2]]);
    app.stepSimulation(80);

    app.gripper.close(false);
    app.stepSimulation(45);
    expect(app.graspManager.isGrasping).toBe(true);

    app.gripperTargetController.setTargetPosition([0.20, 0.22, 0.24]);
    app.stepSimulation(85);

    app.gripper.open(false);
    app.stepSimulation(40);

    app.episodeRecorder.stopRecording('success');
    expect(app.episodeRecorder.isRecording).toBe(false);

    // 4. Verify exported dataset structure, JSON, and CSV
    const dataset = app.episodeRecorder.exportDataset();
    expect(dataset.episode_id).toContain('manual_demo_pick_place');
    expect(dataset.total_steps).toBeGreaterThan(200);
    expect(dataset.outcome).toBe('success');
    expect(dataset.grasp_succeeded).toBe(true);
    expect(dataset.object_metadata.id).toBe(inst.id);
    expect(dataset.object_metadata.mass_kg).toBeCloseTo(inst.getMass(), 3);
    expect(dataset.initial_object_pose.position.length).toBe(3);
    expect(dataset.final_object_pose.position.length).toBe(3);
    expect(dataset.initial_joint_positions_rad.length).toBe(6);
    expect(dataset.final_joint_positions_rad.length).toBe(6);
    expect(dataset.object_trajectory.length).toBe(dataset.total_steps);

    const jsonStr = app.episodeRecorder.exportJSON();
    const parsed = JSON.parse(jsonStr);
    expect(parsed.episode_id).toBe(dataset.episode_id);
    expect(parsed.frames[0].gripper_target_pose).toBeDefined();

    const csvStr = app.episodeRecorder.exportCSV();
    const csvLines = csvStr.trim().split('\n');
    expect(csvLines.length).toBe(dataset.total_steps + 1);
    expect(csvLines[0]).toContain('episode_id,step_index,timestamp,outcome,grasp_succeeded');
  });
});
