/**
 * @file dualObjectZonesValidation.test.ts
 * Comprehensive validation suite for DUAL OBJECT ZONES (ZONE A — START ➔ ZONE B — TARGET):
 * - Large physical work platforms for Zone A and Zone B with Rapier colliders
 * - 6 distinct dynamic objects (Object_01 to Object_06) with unique IDs, geometries, and materials
 * - Simultaneous non-overlapping placement of 6 objects in Zone A
 * - Kinematic reachability of all 12 slot positions (6 in Zone A, 6 in Zone B) by the robot arm
 * - Zone boundary classification (Zone A, Zone B, In Transit, Outside)
 * - Exclusive target slot reservation and placement stability verification in Zone B
 * - Multi-object sequential pick-and-place workflow execution from Zone A to Zone B
 * - Randomization of Zone A starting positions with minimum separation guarantees
 * - Direct 3D mouse interaction with multiple objects
 * - Real-time task monitoring dashboard telemetry updates
 */

import { describe, it, expect, beforeEach, afterEach } from 'vitest';
import { createDigitalTwinApp } from '../src/main.js';
import {
  ZONE_A_CONFIG,
  ZONE_B_CONFIG,
  SIX_OBJECT_DEFINITIONS
} from '../src/objects/ObjectProperties.js';

describe('DUAL OBJECT ZONES (START A ➔ TARGET B) Validation Suite', () => {
  let app: Awaited<ReturnType<typeof createDigitalTwinApp>>;

  beforeEach(async () => {
    app = await createDigitalTwinApp({ headless: true, sixObjects: true });
  });

  afterEach(() => {
    app.dispose();
  });

  it('TEST 1 — Large Physical Work Zones Geometry & Rapier Colliders: verifies dimensions, separation, and physics registration', () => {
    // 1. Verify Zone A dimensions: width = 0.32m, height = 0.02m, depth = 0.24m
    expect(ZONE_A_CONFIG.dimensions.width).toBe(0.32);
    expect(ZONE_A_CONFIG.dimensions.height).toBe(0.02);
    expect(ZONE_A_CONFIG.dimensions.depth).toBe(0.24);
    expect(ZONE_A_CONFIG.center[0]).toBe(-0.30);
    expect(ZONE_A_CONFIG.center[2]).toBe(0.245);

    // 2. Verify Zone B dimensions: width = 0.32m, height = 0.02m, depth = 0.24m
    expect(ZONE_B_CONFIG.dimensions.width).toBe(0.32);
    expect(ZONE_B_CONFIG.dimensions.height).toBe(0.02);
    expect(ZONE_B_CONFIG.dimensions.depth).toBe(0.24);
    expect(ZONE_B_CONFIG.center[0]).toBe(0.30);
    expect(ZONE_B_CONFIG.center[2]).toBe(0.245);

    // 3. Verify separation: Zone A (maxX = -0.14) and Zone B (minX = 0.14) have 0.28m of open central workspace
    expect(ZONE_B_CONFIG.bounds.minX - ZONE_A_CONFIG.bounds.maxX).toBeCloseTo(0.28, 2);

    // 4. Verify physical Rapier bodies exist and are static/fixed with box colliders
    const bodyA = app.bodyManager.bodies.get('zone_a_start_platform');
    expect(bodyA).toBeDefined();
    expect(bodyA!.spec.motionType).toBe('fixed');
    expect(bodyA!.spec.dimensions.width).toBe(0.32);
    expect(bodyA!.spec.dimensions.depth).toBe(0.24);

    const bodyB = app.bodyManager.bodies.get('zone_b_target_platform');
    expect(bodyB).toBeDefined();
    expect(bodyB!.spec.motionType).toBe('fixed');
    expect(bodyB!.spec.dimensions.width).toBe(0.32);
    expect(bodyB!.spec.dimensions.depth).toBe(0.24);
  });

  it('TEST 2 — Six Distinct Objects Inventory: verifies 6 unique IDs, shapes, materials, and dynamic bodies', () => {
    const instruments = app.instrumentFactory.instruments;
    expect(instruments.size).toBe(6);

    const expectedIds = ['Object_01', 'Object_02', 'Object_03', 'Object_04', 'Object_05', 'Object_06'];
    for (const id of expectedIds) {
      expect(instruments.has(id)).toBe(true);
      const inst = instruments.get(id)!;

      // Verify dynamic Rapier body
      expect(inst.managedBody.body.isDynamic()).toBe(true);
      // Verify valid mass and friction
      expect(inst.config.massKg).toBeGreaterThan(0.09);
      expect(inst.config.massKg).toBeLessThan(0.30);
      expect(inst.config.friction).toBeGreaterThan(0.4);

      // Verify graspability
      const graspInfo = inst.getGraspabilityInfo();
      expect(graspInfo.isGraspable).toBe(true);
      expect(graspInfo.fitsGripperOpening).toBe(true);
      expect(graspInfo.isWithinReachableWorkspace).toBe(true);
    }

    // Verify distinct shapes exist across the 6 objects (boxes and cylinders)
    const shapes = expectedIds.map((id) => instruments.get(id)!.config.shape);
    expect(shapes).toContain('box');
    expect(shapes).toContain('cylinder');

    // Verify distinct colors across all 6 objects
    const colors = expectedIds.map((id) => instruments.get(id)!.config.colorHex);
    const uniqueColors = new Set(colors);
    expect(uniqueColors.size).toBe(6);
  });

  it('TEST 3 — Simultaneous Placement in Zone A: verifies all 6 objects rest stably without overlapping or falling through', () => {
    // Step physics for 60 frames (1 second) to allow objects to settle under gravity
    app.stepSimulation(60);

    const positions: [number, number, number][] = [];

    for (const def of SIX_OBJECT_DEFINITIONS) {
      const inst = app.instrumentFactory.instruments.get(def.id)!;
      const pos = inst.getPosition();
      const speed = inst.getSpeed();

      // 1. Verify object is inside Zone A boundaries
      expect(app.workZoneManager.isInsideZoneA(pos)).toBe(true);
      expect(app.workZoneManager.getObjectZone(inst)).toBe('ZONE_A');

      // 2. Verify resting on platform (top surface y = 0.02m) without falling through
      expect(pos[1]).toBeGreaterThan(0.025);
      expect(pos[1]).toBeLessThan(0.055);

      // 3. Verify physically stable (settled with near-zero residual velocity)
      expect(speed).toBeLessThan(0.12);

      positions.push(pos);
    }

    // 4. Verify no two objects overlap (center-to-center distance >= 60 mm)
    for (let i = 0; i < positions.length; i++) {
      for (let j = i + 1; j < positions.length; j++) {
        const dist = Math.hypot(
          positions[i][0] - positions[j][0],
          positions[i][2] - positions[j][2]
        );
        expect(dist).toBeGreaterThanOrEqual(0.060);
      }
    }
  });

  it('TEST 4 — Kinematic Reachability: confirms DLS IK converges for all 6 Zone A slots and all 6 Zone B slots', () => {
    const slotsA = ZONE_A_CONFIG.slots;
    expect(slotsA.length).toBe(6);

    for (let i = 0; i < slotsA.length; i++) {
      const slot = slotsA[i];
      const targetPos: [number, number, number] = [slot[0], 0.040, slot[2]];
      const ikRes = app.motionController.commandCartesianPose(targetPos);
      expect(ikRes.success).toBe(true);
      expect(ikRes.positionErrorMeters).toBeLessThan(0.005);
    }

    const slotsB = ZONE_B_CONFIG.slots;
    expect(slotsB.length).toBe(6);

    for (let i = 0; i < slotsB.length; i++) {
      const slot = slotsB[i];
      const targetPos: [number, number, number] = [slot[0], 0.040, slot[2]];
      const ikRes = app.motionController.commandCartesianPose(targetPos);
      expect(ikRes.success).toBe(true);
      expect(ikRes.positionErrorMeters).toBeLessThan(0.005);
    }
  });

  it('TEST 5 — Zone Boundary & Spatial Classification: correctly distinguishes Zone A, Zone B, in transit, and outside', () => {
    const wz = app.workZoneManager;

    // Inside Zone A
    expect(wz.getZoneForPosition([-0.30, 0.03, 0.24])).toBe('ZONE_A');
    expect(wz.getZoneForPosition([-0.38, 0.03, 0.20])).toBe('ZONE_A');

    // Inside Zone B
    expect(wz.getZoneForPosition([0.30, 0.03, 0.24])).toBe('ZONE_B');
    expect(wz.getZoneForPosition([0.38, 0.03, 0.29])).toBe('ZONE_B');

    // Elevated between zones during transit
    expect(wz.getZoneForPosition([0.0, 0.22, 0.24])).toBe('IN_TRANSIT');

    // Outside the workspace
    expect(wz.getZoneForPosition([0.0, 0.02, -0.40])).toBe('OUTSIDE');
    expect(wz.getZoneForPosition([0.80, 0.02, 0.24])).toBe('OUTSIDE');
  });

  it('TEST 6 — Predefined Target Slot Assignment: reserves slots exclusively and prevents duplicate occupancy in Zone B', () => {
    const wz = app.workZoneManager;
    wz.resetSlotOccupancy();

    expect(wz.getNextAvailableTargetSlotIndex()).toBe(0);

    wz.assignTargetSlot(0, 'Object_01');
    expect(wz.getNextAvailableTargetSlotIndex()).toBe(1);

    wz.assignTargetSlot(1, 'Object_02');
    expect(wz.getNextAvailableTargetSlotIndex()).toBe(2);

    wz.assignTargetSlot(2, 'Object_03');
    wz.assignTargetSlot(3, 'Object_04');
    wz.assignTargetSlot(4, 'Object_05');
    wz.assignTargetSlot(5, 'Object_06');

    // All 6 slots occupied
    expect(wz.getNextAvailableTargetSlotIndex()).toBeNull();
    expect(wz.targetSlotOccupants.length).toBe(6);
    expect(new Set(wz.targetSlotOccupants).size).toBe(6);
  });

  it('TEST 7 — Pick and Place Object Transfer: picks Object_01 from Zone A and places it stably in Slot B1 of Zone B', () => {
    const inst1 = app.instrumentFactory.instruments.get('Object_01')!;
    expect(inst1).toBeDefined();

    const initialPos = inst1.getPosition();
    expect(app.workZoneManager.getObjectZone(inst1)).toBe('ZONE_A');

    // Execute pick and place for Object_01 to Slot B1 ([0.22, 0.02, 0.20])
    const slotB1 = ZONE_B_CONFIG.slots[0];
    const started = app.pickAndPlace.start(slotB1);
    expect(started).toBe(true);

    for (let step = 0; step < 750; step++) {
      app.stepSimulation(1);
      if (app.pickAndPlace.state === 'COMPLETE' || app.pickAndPlace.state === 'FAILED') {
        break;
      }
    }

    expect(app.pickAndPlace.state).toBe('COMPLETE');
    expect(app.pickAndPlace.lastPlacementReport).not.toBeNull();
    expect(app.pickAndPlace.lastPlacementReport!.verified).toBe(true);

    // Verify Object_01 is now physically resting in Zone B near Slot B1
    const finalPos = inst1.getPosition();
    expect(app.workZoneManager.isInsideZoneB(finalPos)).toBe(true);
    expect(app.workZoneManager.getObjectZone(inst1)).toBe('ZONE_B');

    const horizDistToSlotB1 = Math.hypot(finalPos[0] - slotB1[0], finalPos[2] - slotB1[2]);
    expect(horizDistToSlotB1).toBeLessThan(0.06);

    // Verify distance moved from Zone A to Zone B is greater than 0.40 m
    const totalDist = Math.hypot(finalPos[0] - initialPos[0], finalPos[2] - initialPos[2]);
    expect(totalDist).toBeGreaterThan(0.40);
  });

  it('TEST 8 — Randomization of Zone A Starting Positions: randomizes positions inside Zone A with zero overlaps', () => {
    const wz = app.workZoneManager;
    const randomized = wz.randomizeZoneAPositions(app.instrumentFactory);
    expect(randomized).toBe(true);

    // Step physics to let objects settle
    app.stepSimulation(40);

    const positions: [number, number, number][] = [];

    for (const def of SIX_OBJECT_DEFINITIONS) {
      const inst = app.instrumentFactory.instruments.get(def.id)!;
      const pos = inst.getPosition();

      // All objects must remain strictly within Zone A bounds
      expect(wz.isInsideZoneA(pos)).toBe(true);
      expect(pos[1]).toBeGreaterThan(0.025);
      expect(pos[1]).toBeLessThan(0.060);

      positions.push(pos);
    }

    // Verify no two objects collide (minimum 55 mm center-to-center separation)
    for (let i = 0; i < positions.length; i++) {
      for (let j = i + 1; j < positions.length; j++) {
        const dist = Math.hypot(
          positions[i][0] - positions[j][0],
          positions[i][2] - positions[j][2]
        );
        expect(dist).toBeGreaterThanOrEqual(0.055);
      }
    }
  });

  it('TEST 9 — Direct 3D Mouse Interaction on Multiple Objects: grabs, moves, and releases Object_03 with physics', () => {
    app.interactionModeManager.setMode('object');
    const inst3 = app.instrumentFactory.instruments.get('Object_03')!;
    expect(inst3).toBeDefined();

    // Select and grab Object_03
    app.objectManipulator.setHoveredObject(inst3.id);
    const grabbed = app.objectManipulator.grabObject(inst3.id);
    expect(grabbed).not.toBeNull();
    expect(app.objectManipulator.isDragging).toBe(true);
    expect(app.objectManipulator.grabbedObjectId).toBe('Object_03');

    // Move to elevated mid-air position
    app.objectManipulator.moveGrabbedObjectTo([-0.22, 0.25, 0.20]);
    app.stepSimulation(30);

    const liftedY = inst3.getPosition()[1];
    expect(liftedY).toBeGreaterThan(0.20);

    // Release: verify full gravity acts on Object_03 and it settles naturally
    app.objectManipulator.releaseObject();
    expect(app.objectManipulator.isDragging).toBe(false);

    app.stepSimulation(100);
    const settledPos = inst3.getPosition();
    expect(settledPos[1]).toBeGreaterThan(0.030);
    expect(settledPos[1]).toBeLessThan(0.055);
    expect(inst3.getSpeed()).toBeLessThan(0.05);
  });

  it('TEST 10 — Task Monitoring Dashboard Telemetry: computes accurate object counts, progress percent, and slot status', () => {
    const wz = app.workZoneManager;
    wz.resetSlotOccupancy();

    const telemInitial = wz.getTelemetry(app.instrumentFactory);
    expect(telemInitial.totalObjects).toBe(6);
    expect(telemInitial.remainingInZoneA).toBe(6);
    expect(telemInitial.placedInZoneB).toBe(0);
    expect(telemInitial.progressPercent).toBe(0);
    expect(telemInitial.isTaskComplete).toBe(false);
    expect(telemInitial.targetSlots.length).toBe(6);

    // Simulate placing 2 objects in Zone B
    wz.markObjectPlacedAndStable(0, 'Object_01');
    wz.markObjectPlacedAndStable(1, 'Object_02');

    const telemMid = wz.getTelemetry(app.instrumentFactory);
    expect(telemMid.placedInZoneB).toBe(2);
    expect(telemMid.progressPercent).toBe(33); // 2/6 = 33%
    expect(telemMid.targetSlots[0].isOccupied).toBe(true);
    expect(telemMid.targetSlots[0].occupyingObjectId).toBe('Object_01');
    expect(telemMid.targetSlots[1].isOccupied).toBe(true);
    expect(telemMid.targetSlots[1].occupyingObjectId).toBe('Object_02');
    expect(telemMid.targetSlots[2].isOccupied).toBe(false);

    // Simulate placing all 6 objects
    for (let i = 2; i < 6; i++) {
      wz.markObjectPlacedAndStable(i, `Object_0${i + 1}`);
    }

    const telemComplete = wz.getTelemetry(app.instrumentFactory);
    expect(telemComplete.placedInZoneB).toBe(6);
    expect(telemComplete.progressPercent).toBe(100);
    expect(telemComplete.isTaskComplete).toBe(true);
  });
});

