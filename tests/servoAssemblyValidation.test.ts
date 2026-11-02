/**
 * @file servoAssemblyValidation.test.ts
 * Validation suite for the Realistic Servo Motor Joint Assembly.
 * Verifies that each joint's servo housing remains firmly attached to its parent link,
 * that the rotating horn is aligned with the joint axis and mechanically connected to the child link,
 * and that rotating each joint maintains continuous physical connection with zero floating components.
 */

import { describe, it, expect, beforeEach } from 'vitest';
import * as THREE from 'three';
import { createDigitalTwinApp } from '../src/main.js';

describe('Realistic Servo Motor Joint Assembly Validation', () => {
  let app: Awaited<ReturnType<typeof createDigitalTwinApp>>;

  beforeEach(async () => {
    app = await createDigitalTwinApp({ headless: true });
  });

  it('TEST 1 — Parent-Child Relationship Verification: Every servo housing attached to parent, horn attached to child', () => {
    // 1. Servo housings parentage (Fixed to supporting link)
    const j1Housing = app.robotModel.root.getObjectByName('J1_ServoHousing');
    const j2Housing = app.robotModel.root.getObjectByName('J2_ServoHousing');
    const j3Housing = app.robotModel.root.getObjectByName('J3_ServoHousing');
    const j4Housing = app.robotModel.root.getObjectByName('J4_ServoHousing');
    const j5Housing = app.robotModel.root.getObjectByName('J5_ServoHousing');
    const j6Housing = app.robotModel.root.getObjectByName('J6_ServoHousing');
    const gripHousing = app.robotModel.root.getObjectByName('Gripper_ServoHousing');

    expect(j1Housing).toBeDefined();
    expect(j1Housing!.parent).toBe(app.robotModel.root);

    expect(j2Housing).toBeDefined();
    expect(j2Housing!.parent).toBe(app.robotModel.linkGroups[0]);

    expect(j3Housing).toBeDefined();
    expect(j3Housing!.parent).toBe(app.robotModel.linkGroups[1]);

    expect(j4Housing).toBeDefined();
    expect(j4Housing!.parent).toBe(app.robotModel.linkGroups[2]);

    expect(j5Housing).toBeDefined();
    expect(j5Housing!.parent).toBe(app.robotModel.linkGroups[3]);

    expect(j6Housing).toBeDefined();
    expect(j6Housing!.parent).toBe(app.robotModel.linkGroups[4]);

    expect(gripHousing).toBeDefined();
    expect(gripHousing!.parent).toBe(app.gripper.root);

    // 2. Rotating horns parentage (Mechanically connected to child link)
    const j2Horn = app.robotModel.root.getObjectByName('J2_RotatingHorn');
    const j3Horn = app.robotModel.root.getObjectByName('J3_RotatingHorn');
    const j4Turntable = app.robotModel.root.getObjectByName('J4_RotatingTurntable');
    const j5Horn = app.robotModel.root.getObjectByName('J5_RotatingHorn');
    const j6Flange = app.robotModel.root.getObjectByName('J6_RotatingToolFlange');

    expect(j2Horn).toBeDefined();
    expect(j2Horn!.parent).toBe(app.robotModel.linkGroups[1]);

    expect(j3Horn).toBeDefined();
    expect(j3Horn!.parent).toBe(app.robotModel.linkGroups[2]);

    expect(j4Turntable).toBeDefined();
    expect(j4Turntable!.parent).toBe(app.robotModel.linkGroups[3]);

    expect(j5Horn).toBeDefined();
    expect(j5Horn!.parent).toBe(app.robotModel.linkGroups[4]);

    expect(j6Flange).toBeDefined();
    expect(j6Flange!.parent).toBe(app.gripper.root);
  });

  it('TEST 2 — J1 Base Pan Servo Assembly: Housing fixed in base root, turntable rotates with Link 0', () => {
    const root = app.robotModel.root;
    const link0 = app.robotModel.linkGroups[0];
    const j1Housing = app.robotModel.root.getObjectByName('J1_ServoHousing')!;

    const initialHousingPos = j1Housing.position.clone();

    // Rotate J1 by +45 degrees
    app.jointController.setJointTargetPositionDeg(0, 45);
    app.stepSimulation(30);

    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[0]).toBeCloseTo(45, 0);

    // Housing remains strictly in place in root
    expect(j1Housing.position.distanceTo(initialHousingPos)).toBeCloseTo(0, 4);

    // Link 0 rotated around Y
    const link0Quat = new THREE.Quaternion();
    link0.getWorldQuaternion(link0Quat);
    const euler = new THREE.Euler().setFromQuaternion(link0Quat, 'YXZ');
    expect(Math.abs(euler.y)).toBeGreaterThan(0.6); // ~45 deg in rad is 0.785
  });

  it('TEST 3 — J2 Shoulder Lift Servo Assembly: Housing fixed in Link 0, horn & Link 1 rotate around X', () => {
    const link0 = app.robotModel.linkGroups[0];
    const link1 = app.robotModel.linkGroups[1];
    const j2Housing = app.robotModel.root.getObjectByName('J2_ServoHousing')!;
    const j2Horn = app.robotModel.root.getObjectByName('J2_RotatingHorn')!;

    const initialHousingPos = j2Housing.position.clone();
    const initialHousingRot = j2Housing.rotation.clone();

    // Rotate J2 by +30 degrees
    app.jointController.setJointTargetPositionDeg(1, 30);
    app.stepSimulation(30);

    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[1]).toBeCloseTo(30, 0);

    // J2 Servo housing remains strictly fixed in Link 0
    expect(j2Housing.position.distanceTo(initialHousingPos)).toBeCloseTo(0, 4);
    expect(j2Housing.rotation.equals(initialHousingRot)).toBe(true);

    // Link 0 did not pitch
    const link0Quat = new THREE.Quaternion();
    link0.getWorldQuaternion(link0Quat);
    const link0Euler = new THREE.Euler().setFromQuaternion(link0Quat, 'XYZ');
    expect(Math.abs(link0Euler.x)).toBeCloseTo(0, 3);

    // Link 1 and J2 Rotating Horn rotated together around X
    const link1Quat = new THREE.Quaternion();
    link1.getWorldQuaternion(link1Quat);
    const link1Euler = new THREE.Euler().setFromQuaternion(link1Quat, 'XYZ');
    expect(Math.abs(link1Euler.x)).toBeGreaterThan(0.4);

    // Horn is concentric with rotation axis at local (0.030, 0, 0) in Link 1
    expect(j2Horn.position.y).toBeCloseTo(0, 3);
    expect(j2Horn.position.z).toBeCloseTo(0, 3);
  });

  it('TEST 4 — J3 Elbow Pitch Servo Assembly: Housing fixed in Link 1, horn & Link 2 rotate around X', () => {
    const link1 = app.robotModel.linkGroups[1];
    const link2 = app.robotModel.linkGroups[2];
    const j3Housing = app.robotModel.root.getObjectByName('J3_ServoHousing')!;
    const j3Horn = app.robotModel.root.getObjectByName('J3_RotatingHorn')!;

    const initialHousingPos = j3Housing.position.clone();
    const initialHousingRot = j3Housing.rotation.clone();

    // Rotate J3 by +45 degrees
    app.jointController.setJointTargetPositionDeg(2, 45);
    app.stepSimulation(30);

    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[2]).toBeCloseTo(45, 0);

    // J3 Servo housing remains strictly fixed in Link 1
    expect(j3Housing.position.distanceTo(initialHousingPos)).toBeCloseTo(0, 4);
    expect(j3Housing.rotation.equals(initialHousingRot)).toBe(true);

    // Link 2 orientation relative to Link 1 reflects J3 angle
    const q1 = new THREE.Quaternion();
    link1.getWorldQuaternion(q1);
    const q2 = new THREE.Quaternion();
    link2.getWorldQuaternion(q2);

    const qRel = q1.clone().invert().multiply(q2);
    const eRel = new THREE.Euler().setFromQuaternion(qRel, 'XYZ');
    expect(Math.abs(eRel.x)).toBeGreaterThan(0.6);

    // Horn is concentric with rotation axis at local (0.028, 0, 0) in Link 2
    expect(j3Horn.position.y).toBeCloseTo(0, 3);
    expect(j3Horn.position.z).toBeCloseTo(0, 3);
  });

  it('TEST 5 — J4 Forearm Roll Servo Assembly: Housing & stator fixed in Link 2, rotor & Link 3 rotate around Y', () => {
    const link2 = app.robotModel.linkGroups[2];
    const link3 = app.robotModel.linkGroups[3];
    const j4Housing = app.robotModel.root.getObjectByName('J4_ServoHousing')!;
    const j4Turntable = app.robotModel.root.getObjectByName('J4_RotatingTurntable')!;

    const initialHousingPos = j4Housing.position.clone();

    // Rotate J4 by +60 degrees (torsion around Y)
    app.jointController.setJointTargetPositionDeg(3, 60);
    app.stepSimulation(60);

    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[3]).toBeCloseTo(60, 0);

    // J4 Servo housing remains strictly fixed in Link 2
    expect(j4Housing.position.distanceTo(initialHousingPos)).toBeCloseTo(0, 4);

    const q2 = new THREE.Quaternion();
    link2.getWorldQuaternion(q2);
    const q3 = new THREE.Quaternion();
    link3.getWorldQuaternion(q3);

    const qRel = q2.clone().invert().multiply(q3);
    const eRel = new THREE.Euler().setFromQuaternion(qRel, 'YXZ');
    expect(Math.abs(eRel.y)).toBeGreaterThan(0.8);
    expect(Math.abs(eRel.x)).toBeCloseTo(0, 3);
    expect(Math.abs(eRel.z)).toBeCloseTo(0, 3);

    // Turntable is centered on Y-axis in Link 3
    expect(j4Turntable.position.x).toBeCloseTo(0, 3);
    expect(j4Turntable.position.z).toBeCloseTo(0, 3);
  });

  it('TEST 6 — J5 Wrist Pitch Servo Assembly: Housing fixed in Link 3, horn & Link 4 rotate around X', () => {
    const link3 = app.robotModel.linkGroups[3];
    const link4 = app.robotModel.linkGroups[4];
    const j5Housing = app.robotModel.root.getObjectByName('J5_ServoHousing')!;
    const j5Horn = app.robotModel.root.getObjectByName('J5_RotatingHorn')!;

    const initialHousingPos = j5Housing.position.clone();

    // Rotate J5 by +35 degrees
    app.jointController.setJointTargetPositionDeg(4, 35);
    app.stepSimulation(30);

    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[4]).toBeCloseTo(35, 0);

    // J5 Servo housing remains strictly fixed in Link 3
    expect(j5Housing.position.distanceTo(initialHousingPos)).toBeCloseTo(0, 4);

    const q3 = new THREE.Quaternion();
    link3.getWorldQuaternion(q3);
    const q4 = new THREE.Quaternion();
    link4.getWorldQuaternion(q4);

    const qRel = q3.clone().invert().multiply(q4);
    const eRel = new THREE.Euler().setFromQuaternion(qRel, 'XYZ');
    expect(Math.abs(eRel.x)).toBeGreaterThan(0.5);

    // Horn is concentric with rotation axis at local (0.019, 0, 0) in Link 4
    expect(j5Horn.position.y).toBeCloseTo(0, 3);
    expect(j5Horn.position.z).toBeCloseTo(0, 3);
  });

  it('TEST 7 — J6 Tool Roll Servo Assembly: Housing & collar fixed in Link 4, flange & Gripper rotate around Y', () => {
    const link4 = app.robotModel.linkGroups[4];
    const gripperRoot = app.gripper.root;
    const j6Housing = app.robotModel.root.getObjectByName('J6_ServoHousing')!;
    const j6Flange = app.robotModel.root.getObjectByName('J6_RotatingToolFlange')!;

    const initialHousingPos = j6Housing.position.clone();

    // Rotate J6 by +50 degrees
    app.jointController.setJointTargetPositionDeg(5, 50);
    app.stepSimulation(30);

    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[5]).toBeCloseTo(50, 0);

    // J6 Servo housing remains strictly fixed in Link 4
    expect(j6Housing.position.distanceTo(initialHousingPos)).toBeCloseTo(0, 4);

    const q4 = new THREE.Quaternion();
    link4.getWorldQuaternion(q4);
    const qG = new THREE.Quaternion();
    gripperRoot.getWorldQuaternion(qG);

    const qRel = q4.clone().invert().multiply(qG);
    const eRel = new THREE.Euler().setFromQuaternion(qRel, 'YXZ');
    expect(Math.abs(eRel.y)).toBeGreaterThan(0.7);
    expect(Math.abs(eRel.x)).toBeCloseTo(0, 3);
    expect(Math.abs(eRel.z)).toBeCloseTo(0, 3);

    // Flange is centered on Y-axis in gripper root
    expect(j6Flange.position.x).toBeCloseTo(0, 3);
    expect(j6Flange.position.z).toBeCloseTo(0, 3);
  });

  it('TEST 8 — Multi-Joint Simultaneous Motion: All servos remain physically anchored to parent links without detaching', () => {
    // Drive J1, J2, J3, J4, J5 simultaneously
    app.jointController.setJointTargetPositionDeg(0, 30);
    app.jointController.setJointTargetPositionDeg(1, 25);
    app.jointController.setJointTargetPositionDeg(2, 40);
    app.jointController.setJointTargetPositionDeg(3, -30);
    app.jointController.setJointTargetPositionDeg(4, 20);
    app.stepSimulation(60);

    const angles = app.robotJoints.getCurrentAnglesDeg();
    expect(angles[0]).toBeCloseTo(30, 0);
    expect(angles[1]).toBeCloseTo(25, 0);
    expect(angles[2]).toBeCloseTo(40, 0);

    // Verify all servo housings remain attached to their parent links
    const j1Housing = app.robotModel.root.getObjectByName('J1_ServoHousing')!;
    const j2Housing = app.robotModel.root.getObjectByName('J2_ServoHousing')!;
    const j3Housing = app.robotModel.root.getObjectByName('J3_ServoHousing')!;
    const j4Housing = app.robotModel.root.getObjectByName('J4_ServoHousing')!;
    const j5Housing = app.robotModel.root.getObjectByName('J5_ServoHousing')!;
    const j6Housing = app.robotModel.root.getObjectByName('J6_ServoHousing')!;

    expect(j1Housing.parent).toBe(app.robotModel.root);
    expect(j2Housing.parent).toBe(app.robotModel.linkGroups[0]);
    expect(j3Housing.parent).toBe(app.robotModel.linkGroups[1]);
    expect(j4Housing.parent).toBe(app.robotModel.linkGroups[2]);
    expect(j5Housing.parent).toBe(app.robotModel.linkGroups[3]);
    expect(j6Housing.parent).toBe(app.robotModel.linkGroups[4]);

    // Verify output spline on J2 housing and J2 horn are coincident in world space
    const j2OutputWorld = new THREE.Vector3(0.030, 0.120, 0);
    app.robotModel.linkGroups[0].localToWorld(j2OutputWorld);

    const j2HornWorld = new THREE.Vector3();
    app.robotModel.root.getObjectByName('J2_RotatingHorn')!.getWorldPosition(j2HornWorld);

    expect(j2OutputWorld.distanceTo(j2HornWorld)).toBeLessThan(0.002);
  });
});
