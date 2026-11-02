/**
 * @file physicsCollisionValidation.test.ts
 * Automated validation test suite for Phase 8 (Tests 1–8):
 * Collision detection, multi-box stacking, momentum transfer on pushing,
 * object blocking stability, robot-object physical collision response,
 * physical FixedImpulseJoint grasp & lift of a 100g object, gravity release,
 * multi-object proximity stability, and PhysicsDebugger telemetry.
 */

import { describe, it, expect, beforeEach, afterEach } from 'vitest';
import { createDigitalTwinApp } from '../src/main.js';
import { PICKUP_STATION_POS, PLACEMENT_TRAY_POS } from '../src/objects/ObjectProperties.js';
import { canGroupsInteract, COLLISION_PRESETS } from '../src/physics/CollisionGroups.js';

describe('Phase 8 — Physics & Collision Validation Suite (Tests 1–8)', () => {
  let app: Awaited<ReturnType<typeof createDigitalTwinApp>>;

  beforeEach(async () => {
    app = await createDigitalTwinApp({ headless: true });
  });

  afterEach(() => {
    app.dispose();
  });

  it('TEST 1 — FALLING OBJECT: drops a 100g object from a known height and settles naturally without tunneling', () => {
    // Clear default instrument so Pickup Station pad (top surface y = 0.020 m) is empty
    app.instrumentFactory.removeInstrument('surgical_instrument_01');

    // Spawn a 100g (0.10 kg) object (height = 0.036 m, half-height = 0.018 m) at height y = 0.30 m
    const obj = app.instrumentFactory.spawnInstrument('small_metallic_object', {
      id: 'drop_100g_obj',
      massKg: 0.10,
      position: [PICKUP_STATION_POS[0], 0.30, PICKUP_STATION_POS[2]]
    });

    expect(obj.config.massKg).toBeCloseTo(0.10, 4);
    const startY = obj.getPosition()[1];
    expect(startY).toBeCloseTo(0.30, 3);

    // Step 12 frames (0.20 s): verify object accelerates downward in free-fall
    app.stepSimulation(12);
    const midY = obj.getPosition()[1];
    const midVelY = obj.getLinearVelocity()[1];
    expect(midY).toBeLessThan(startY - 0.08);
    expect(midVelY).toBeLessThan(-0.5);

    // Step another 110 frames so object collides with the pickup platform and settles
    app.stepSimulation(110);
    const finalPos = obj.getPosition();
    const finalSpeed = obj.getSpeed();

    // Pickup station top surface is y = 0.020 m; object half-height is 0.018 m -> expected center y ≈ 0.038 m
    expect(finalPos[1]).toBeGreaterThan(0.034);
    expect(finalPos[1]).toBeLessThan(0.044);
    // Settled naturally with near-zero velocity (no micro-bouncing or tunneling through floor)
    expect(finalSpeed).toBeLessThan(0.03);
  });

  it('TEST 2 — STACKING: stacks 3 boxes on top of each other stably without penetration or collapse', () => {
    // Remove default instrument so pickup platform (top y = 0.020 m) is clear
    app.instrumentFactory.removeInstrument('surgical_instrument_01');

    // Box dimensions: [0.042, 0.042, 0.042] -> height = 0.042 m, half-height = 0.021 m
    const x = PICKUP_STATION_POS[0];
    const z = PICKUP_STATION_POS[2];

    const box1 = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'stack_box_1',
      massKg: 0.15,
      position: [x, 0.020 + 0.0215, z]
    });
    const box2 = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'stack_box_2',
      massKg: 0.15,
      position: [x, 0.020 + 0.042 + 0.0225, z]
    });
    const box3 = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'stack_box_3',
      massKg: 0.15,
      position: [x, 0.020 + 0.084 + 0.0235, z]
    });

    // Step 150 frames (2.5 seconds) under gravity
    app.stepSimulation(150);

    const y1 = box1.getPosition()[1];
    const y2 = box2.getPosition()[1];
    const y3 = box3.getPosition()[1];

    // Verify strict vertical ordering (no interpenetration or collapse)
    expect(y1).toBeGreaterThan(0.038);
    expect(y2 - y1).toBeGreaterThan(0.039); // box height is 0.042m (< 3mm penetration)
    expect(y3 - y2).toBeGreaterThan(0.039);

    // Verify all 3 stacked boxes are stable and not shaking
    expect(box1.getSpeed()).toBeLessThan(0.04);
    expect(box2.getSpeed()).toBeLessThan(0.04);
    expect(box3.getSpeed()).toBeLessThan(0.04);

    // Verify top box can still be moved horizontally when a force is applied (not stuck)
    app.bodyManager.applyExternalImpulse('stack_box_3', [0.05, 0.0, 0.0]);
    app.stepSimulation(15);
    expect(Math.abs(box3.getPosition()[0] - x)).toBeGreaterThan(0.005);
  });

  it('TEST 3 — PUSHING: pushes one object into another and transfers momentum according to mass and friction', () => {
    app.instrumentFactory.removeInstrument('surgical_instrument_01');

    // Spawn two 0.042m cubes side by side along X on the floor (half-height = 0.021 m)
    const pusher = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'push_box_A',
      massKg: 0.22,
      friction: 0.20,
      position: [-0.046, 0.0215, 0.22]
    });
    const target = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'push_box_B',
      massKg: 0.12,
      friction: 0.20,
      position: [0.0, 0.0215, 0.22]
    });

    app.stepSimulation(15);
    const targetInitialX = target.getPosition()[0];

    // Push box A in +X toward box B
    app.bodyManager.applyExternalImpulse('push_box_A', [0.14, 0.0, 0.0]);
    app.stepSimulation(35);

    const pusherAfterX = pusher.getPosition()[0];
    const targetAfterX = target.getPosition()[0];

    // Both objects moved in +X
    expect(pusherAfterX).toBeGreaterThan(-0.038);
    expect(targetAfterX).toBeGreaterThan(targetInitialX + 0.012);

    // Verify no interpenetration: centers along X remain separated by at least width (0.042 m) - tolerance
    expect(targetAfterX - pusherAfterX).toBeGreaterThan(0.038);
  });

  it('TEST 4 — OBJECT BLOCKING: places an object between two others without numerical lockup and allows unblocked movement', () => {
    app.instrumentFactory.removeInstrument('surgical_instrument_01');

    // Place 3 0.042m boxes along X on the floor with small clearances
    const leftBlock = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'block_left',
      massKg: 0.80,
      friction: 0.95,
      position: [-0.046, 0.0215, 0.22]
    });
    const middleObj = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'block_middle',
      massKg: 0.12,
      friction: 0.35,
      position: [0.0, 0.0215, 0.22]
    });
    const rightBlock = app.instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'block_right',
      massKg: 0.80,
      friction: 0.95,
      position: [0.046, 0.0215, 0.22]
    });

    app.stepSimulation(20);

    // Apply small force on middle object against heavy right block
    for (let i = 0; i < 15; i++) {
      app.bodyManager.applyExternalForce('block_middle', [0.25, 0, 0]);
      app.stepSimulation(1);
    }

    // Verify all coordinates remain finite and objects did not pass through each other
    const pLeft = leftBlock.getPosition();
    const pMid = middleObj.getPosition();
    const pRight = rightBlock.getPosition();
    expect(Number.isFinite(pMid[0])).toBe(true);
    expect(pMid[0]).toBeGreaterThan(pLeft[0] + 0.036);
    expect(pRight[0]).toBeGreaterThan(pMid[0] + 0.036);

    // Now push middle object in +Z (unblocked direction) and verify it moves freely (not permanently stuck)
    const startZ = middleObj.getPosition()[2];
    app.bodyManager.applyExternalImpulse('block_middle', [0.0, 0.0, 0.07]);
    app.stepSimulation(25);
    expect(middleObj.getPosition()[2]).toBeGreaterThan(startZ + 0.015);
  });

  it('TEST 5 — ROBOT COLLISION: robotic arm respects physical collisions with objects and does not freeze permanently', () => {
    // Verify collision group filtering permits robot links & gripper fingers to collide with dynamic objects
    expect(
      canGroupsInteract(COLLISION_PRESETS.ROBOT_WRIST, COLLISION_PRESETS.DYNAMIC_OBJECT)
    ).toBe(true);
    expect(
      canGroupsInteract(COLLISION_PRESETS.GRIPPER_FINGER, COLLISION_PRESETS.DYNAMIC_OBJECT)
    ).toBe(true);
    // Adjacent robot links do not self-interfere
    expect(
      canGroupsInteract(COLLISION_PRESETS.ROBOT_BASE_LOWER, COLLISION_PRESETS.ROBOT_UPPER_ARM)
    ).toBe(false);
    // Non-adjacent robot links (wrist vs base) do collide
    expect(
      canGroupsInteract(COLLISION_PRESETS.ROBOT_WRIST, COLLISION_PRESETS.ROBOT_BASE_LOWER)
    ).toBe(true);

    const inst = app.instrumentFactory.getActiveInstrument()!;
    const initialPos = inst.getPosition();

    // Command the robot end-effector downward onto/through the object with closed gripper
    app.gripper.closeGripper();
    app.stepSimulation(15);

    app.motionController.setCartesianTarget([
      initialPos[0],
      initialPos[1] - 0.03,
      initialPos[2]
    ]);
    app.stepSimulation(75);

    // Verify the robot end-effector did not tunnel underground or freeze with NaN angles
    const eePos = app.robotModel.getEndEffectorPosition();
    expect(Number.isFinite(eePos[0])).toBe(true);
    expect(Number.isFinite(eePos[1])).toBe(true);
    expect(eePos[1]).toBeGreaterThan(0.018);

    // Command the robot back to home pose and verify it moves freely without being locked
    app.motionController.moveToHomePose(false);
    app.stepSimulation(90);
    const homeEePos = app.robotModel.getEndEffectorPosition();
    expect(homeEePos[1]).toBeGreaterThan(0.18);
  });

  it('TEST 6 — GRASP AND LIFT: gripper grasps a 100g object via FixedImpulseJoint and lifts it stably', () => {
    // Configure active object to 100g (0.10 kg)
    const inst = app.instrumentFactory.getActiveInstrument()!;
    app.instrumentFactory.setInstrumentMass(inst.config.id, 0.10);
    expect(inst.config.massKg).toBeCloseTo(0.10, 4);

    // Open gripper and approach the 100g object on Pickup Station using physical joint motors
    app.gripper.open(false);
    const objPos = inst.getPosition();

    app.motionController.commandCartesianPose([objPos[0], 0.24, objPos[2]]);
    app.stepSimulation(80);
    app.motionController.commandCartesianPose([objPos[0], objPos[1], objPos[2]]);
    app.stepSimulation(90);

    // Verify object does NOT follow the gripper before grasping
    expect(app.graspManager.isGrasping).toBe(false);
    expect(app.graspManager.hasPhysicalJoint).toBe(false);

    // Close gripper to establish bilateral contact + Coulomb friction + Rapier FixedImpulseJoint
    app.gripper.close(false);
    app.stepSimulation(45);

    expect(app.graspManager.isGrasping).toBe(true);
    expect(app.graspManager.hasPhysicalJoint).toBe(true);
    expect(app.graspManager.graspedInstrumentId).toBe(inst.config.id);

    // Lift the 100g object upward to y = 0.22 m
    app.motionController.commandCartesianPose([objPos[0], 0.22, objPos[2]]);
    app.stepSimulation(80);

    const liftedPos = inst.getPosition();
    expect(liftedPos[1]).toBeGreaterThan(0.14);
    expect(app.graspManager.isGrasping).toBe(true);
  });

  it('TEST 7 — RELEASE: releases a grasped 100g object above the platform and verifies natural free-fall and settling', () => {
    const inst = app.instrumentFactory.getActiveInstrument()!;
    app.instrumentFactory.setInstrumentMass(inst.config.id, 0.10);
    const objPos = inst.getPosition();

    // Grasp and lift the 100g object first
    app.gripper.open(false);
    app.motionController.commandCartesianPose([objPos[0], 0.24, objPos[2]]);
    app.stepSimulation(80);
    app.motionController.commandCartesianPose([objPos[0], objPos[1], objPos[2]]);
    app.stepSimulation(90);
    app.gripper.close(false);
    app.stepSimulation(45);
    app.motionController.commandCartesianPose([objPos[0], 0.22, objPos[2]]);
    app.stepSimulation(80);

    expect(app.graspManager.isGrasping).toBe(true);
    expect(inst.getPosition()[1]).toBeGreaterThan(0.14);

    // Open gripper above the platform to release the object
    app.gripper.open(false);
    app.stepSimulation(15);
    expect(app.graspManager.isGrasping).toBe(false);
    expect(app.graspManager.hasPhysicalJoint).toBe(false);

    // Step 110 frames so the released object falls and settles on the platform/floor
    app.stepSimulation(110);
    const settledPos = inst.getPosition();
    expect(settledPos[1]).toBeLessThan(0.065);
    expect(settledPos[1]).toBeGreaterThan(0.015);
    expect(inst.getSpeed()).toBeLessThan(0.04);
  });

  it('TEST 8 — MULTIPLE OBJECTS: places multiple objects in close proximity without freezing and verifies PhysicsDebugger', () => {
    // Enable PhysicsDebugger
    app.physicsDebugger.setEnabled(true, false);

    // Spawn 3 additional instruments in close proximity on the floor and placement tray
    const objA = app.instrumentFactory.spawnInstrument('cylindrical_instrument', {
      id: 'multi_obj_A',
      position: [-0.04, 0.04, 0.22]
    });
    const objB = app.instrumentFactory.spawnInstrument('small_metallic_object', {
      id: 'multi_obj_B',
      position: [0.04, 0.04, 0.22]
    });
    const objC = app.instrumentFactory.spawnInstrument('surgical_tool', {
      id: 'multi_obj_C',
      position: [PLACEMENT_TRAY_POS[0], 0.08, PLACEMENT_TRAY_POS[2]]
    });

    app.stepSimulation(90);

    // Verify all objects settled cleanly without NaN, freezing, or floor penetration
    for (const inst of [objA, objB, objC]) {
      const pos = inst.getPosition();
      expect(Number.isFinite(pos[0])).toBe(true);
      expect(Number.isFinite(pos[1])).toBe(true);
      expect(Number.isFinite(pos[2])).toBe(true);
      expect(pos[1]).toBeGreaterThan(0.008);
      expect(inst.getSpeed()).toBeLessThan(0.05);
    }

    // Verify PhysicsDebugger snapshot and 3D wireframe helpers
    const snapshot = app.physicsDebugger.getSnapshot();
    expect(snapshot.totalBodies).toBeGreaterThanOrEqual(15);
    expect(snapshot.dynamicBodies).toBe(4);
    expect(app.physicsDebugger.root.visible).toBe(true);
    expect(app.physicsDebugger.root.children.length).toBeGreaterThan(0);
  });
});
