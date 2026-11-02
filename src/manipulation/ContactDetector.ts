/**
 * @file ContactDetector.ts
 * Evaluates bilateral physical contact between the mechanical gripper's left/right
 * fingers and dynamic surgical instruments in the Rapier3D physics world.
 */

import * as THREE from 'three';
import { Gripper } from '../robot/Gripper.js';
import {
  InstrumentFactory,
  ActiveSurgicalInstrument
} from '../objects/InstrumentFactory.js';

export interface FingerObjectContactResult {
  instrument: ActiveSurgicalInstrument | null;
  isBetweenFingers: boolean;
  leftFingerContact: boolean;
  rightFingerContact: boolean;
  bilateralContact: boolean;
  localOffsetFromGraspFrame: THREE.Vector3;
  objectGraspWidthMeters: number;
  distanceToGraspCenterMeters: number;
}

export class ContactDetector {
  public gripper: Gripper;
  public instrumentFactory: InstrumentFactory;

  constructor(gripper: Gripper, instrumentFactory: InstrumentFactory) {
    this.gripper = gripper;
    this.instrumentFactory = instrumentFactory;
  }

  /**
   * Evaluates physical contact geometry between the gripper fingers and all
   * active surgical instruments, returning the closest candidate in the grasp region.
   */
  public detectFingerContacts(): FingerObjectContactResult {
    const graspPose = this.gripper.getGraspFrameWorldPose();
    const invGraspMatrix = graspPose.matrix.clone().invert();

    let bestCandidate: ActiveSurgicalInstrument | null = null;
    let bestDist = Infinity;
    let bestLocalPos = new THREE.Vector3();

    for (const [, inst] of this.instrumentFactory.instruments) {
      const t = inst.managedBody.body.translation();
      const worldPos = new THREE.Vector3(t.x, t.y, t.z);
      const dist = worldPos.distanceTo(graspPose.position);

      if (dist < bestDist) {
        bestDist = dist;
        bestCandidate = inst;
        bestLocalPos = worldPos.clone().applyMatrix4(invGraspMatrix);
      }
    }

    if (!bestCandidate) {
      return {
        instrument: null,
        isBetweenFingers: false,
        leftFingerContact: false,
        rightFingerContact: false,
        bilateralContact: false,
        localOffsetFromGraspFrame: new THREE.Vector3(),
        objectGraspWidthMeters: 0,
        distanceToGraspCenterMeters: Infinity
      };
    }

    // Compute effective width of the object along the gripper's jaw closing axis (GraspFrame local X)
    // accounting for arbitrary 3D object orientation and gripper orientation
    const r = bestCandidate.managedBody.body.rotation();
    const objQuat = new THREE.Quaternion(r.x, r.y, r.z, r.w);
    const graspXWorld = new THREE.Vector3(1, 0, 0)
      .applyQuaternion(graspPose.quaternion)
      .normalize();
    const graspXInObjLocal = graspXWorld
      .clone()
      .applyQuaternion(objQuat.clone().invert())
      .normalize();

    const dims = bestCandidate.config.dimensions;
    let objWidth = bestCandidate.config.graspWidthMeters;
    if (bestCandidate.config.shape === 'box') {
      const projectedSpan =
        Math.abs(graspXInObjLocal.x) * dims.width +
        Math.abs(graspXInObjLocal.y) * dims.height +
        Math.abs(graspXInObjLocal.z) * dims.depth;
      objWidth = Math.max(0.006, projectedSpan);
    }

    const halfObjWidth = objWidth / 2;
    const halfOpening = this.gripper.currentOpeningMeters / 2;

    // Check if object lies within the longitudinal (Y) and depth (Z) envelope of the finger pads
    const withinPadLengthY = Math.abs(bestLocalPos.y) <= 0.036;
    const withinPadDepthZ = Math.abs(bestLocalPos.z) <= 0.034;
    const withinJawSpanX = Math.abs(bestLocalPos.x) <= halfOpening + 0.012;

    const isBetweenFingers = withinPadLengthY && withinPadDepthZ && withinJawSpanX;

    // Distance from left finger inner pad (-halfOpening) to object left face (localX - halfObjWidth)
    const leftClearance = (bestLocalPos.x - halfObjWidth) - (-halfOpening);
    // Distance from right finger inner pad (+halfOpening) to object right face (localX + halfObjWidth)
    const rightClearance = halfOpening - (bestLocalPos.x + halfObjWidth);

    const contactTol = 0.005; // 5 mm pad compression/contact tolerance
    const leftFingerContact = isBetweenFingers && Math.abs(leftClearance) <= contactTol;
    const rightFingerContact = isBetweenFingers && Math.abs(rightClearance) <= contactTol;
    const bilateralContact = leftFingerContact && rightFingerContact;

    return {
      instrument: bestCandidate,
      isBetweenFingers,
      leftFingerContact,
      rightFingerContact,
      bilateralContact,
      localOffsetFromGraspFrame: bestLocalPos,
      objectGraspWidthMeters: objWidth,
      distanceToGraspCenterMeters: bestDist
    };
  }
}

