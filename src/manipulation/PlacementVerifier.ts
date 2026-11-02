/**
 * @file PlacementVerifier.ts
 * Verifies final instrument placement position, orientation, and physical stability
 * on the target surgical tray.
 */

import * as THREE from 'three';
import { ActiveSurgicalInstrument } from '../objects/InstrumentFactory.js';
import { TARGET_TRAY_POS } from '../objects/ObjectProperties.js';

export interface PlacementVerificationReport {
  verified: boolean;
  withinPositionTolerance: boolean;
  isUprightAndAligned: boolean;
  isPhysicallyStable: boolean;
  isDetachedFromGripper: boolean;
  horizontalDistanceMeters: number;
  verticalErrorMeters: number;
  speedMetersPerSec: number;
  reason: string;
}

export class PlacementVerifier {
  public targetTrayCenter: THREE.Vector3;
  public horizontalRadiusToleranceMeters: number = 0.095;
  public trayTopSurfaceY: number = TARGET_TRAY_POS[1]; // 0.02 m

  constructor(targetPosition: [number, number, number] = TARGET_TRAY_POS) {
    this.targetTrayCenter = new THREE.Vector3(
      targetPosition[0],
      targetPosition[1],
      targetPosition[2]
    );
  }

  /**
   * Evaluates whether the instrument is detached, located within the target surgical tray,
   * upright, and physically stable.
   */
  public verify(
    instrument: ActiveSurgicalInstrument | undefined,
    isCurrentlyGrasped: boolean
  ): PlacementVerificationReport {
    if (!instrument) {
      return {
        verified: false,
        withinPositionTolerance: false,
        isUprightAndAligned: false,
        isPhysicallyStable: false,
        isDetachedFromGripper: !isCurrentlyGrasped,
        horizontalDistanceMeters: Infinity,
        verticalErrorMeters: Infinity,
        speedMetersPerSec: Infinity,
        reason: 'No active surgical instrument found for placement verification.'
      };
    }

    const body = instrument.managedBody.body;
    const t = body.translation();
    const r = body.rotation();
    const lv = body.linvel();
    const av = body.angvel();

    const pos = new THREE.Vector3(t.x, t.y, t.z);
    const quat = new THREE.Quaternion(r.x, r.y, r.z, r.w);
    const linSpeed = Math.sqrt(lv.x * lv.x + lv.y * lv.y + lv.z * lv.z);
    const angSpeed = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);

    const dx = pos.x - this.targetTrayCenter.x;
    const dz = pos.z - this.targetTrayCenter.z;
    const horizontalDist = Math.sqrt(dx * dx + dz * dz);

    const expectedCenterY =
      this.trayTopSurfaceY + instrument.config.dimensions.height / 2;
    const verticalError = Math.abs(pos.y - expectedCenterY);

    const withinPositionTolerance =
      horizontalDist <= this.horizontalRadiusToleranceMeters &&
      verticalError <= 0.032 &&
      pos.y >= 0.01;

    // Check upright orientation (local +Y axis roughly aligned with world +Y)
    const localUp = new THREE.Vector3(0, 1, 0).applyQuaternion(quat);
    const isUprightAndAligned = Math.abs(localUp.y) >= 0.75;

    const isPhysicallyStable = linSpeed <= 0.18 && angSpeed <= 0.8;
    const isDetachedFromGripper = !isCurrentlyGrasped;

    const verified =
      isDetachedFromGripper &&
      withinPositionTolerance &&
      isUprightAndAligned &&
      isPhysicallyStable;

    let reason = 'Dépose vérifiée : l\'instrument est stable sur le plateau cible.';
    if (!isDetachedFromGripper) {
      reason = 'Échec dépose : l\'instrument est toujours tenu par la pince.';
    } else if (!withinPositionTolerance) {
      reason = `Échec dépose : instrument hors du plateau cible (errHoriz=${(horizontalDist * 1000).toFixed(1)} mm, errVert=${(verticalError * 1000).toFixed(1)} mm).`;
    } else if (!isUprightAndAligned) {
      reason = 'Échec dépose : instrument renversé / mal orienté.';
    } else if (!isPhysicallyStable) {
      reason = `Dépose instable : vitesse résiduelle de ${linSpeed.toFixed(3)} m/s.`;
    }

    return {
      verified,
      withinPositionTolerance,
      isUprightAndAligned,
      isPhysicallyStable,
      isDetachedFromGripper,
      horizontalDistanceMeters: horizontalDist,
      verticalErrorMeters: verticalError,
      speedMetersPerSec: linSpeed,
      reason
    };
  }
}
