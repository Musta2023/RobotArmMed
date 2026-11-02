/**
 * @file WorkZoneManager.ts
 * Manages dual physical workspaces (ZONE A — START and ZONE B — TARGET),
 * 6-object slot allocation, zone boundary detection, and physical placement stability.
 */

import * as THREE from 'three';
import {
  ZONE_A_CONFIG,
  ZONE_B_CONFIG,
  WorkZoneConfig,
  SIX_OBJECT_DEFINITIONS,
  SurgicalInstrumentConfig
} from './ObjectProperties.js';
import {
  InstrumentFactory,
  ActiveSurgicalInstrument
} from './InstrumentFactory.js';

export type ZoneClassification = 'ZONE_A' | 'ZONE_B' | 'IN_TRANSIT' | 'OUTSIDE';

export interface TargetSlotStatus {
  index: number;
  id: string; // 'SLOT_B1'..'SLOT_B6'
  position: [number, number, number];
  isOccupied: boolean;
  occupyingObjectId: string | null;
  isVerifiedStable: boolean;
}

export interface DualZoneTelemetry {
  totalObjects: number;
  remainingInZoneA: number;
  placedInZoneB: number;
  inTransit: number;
  outsideWorkspace: number;
  progressPercent: number;
  isTaskComplete: boolean;
  targetSlots: TargetSlotStatus[];
}

export class WorkZoneManager {
  public zoneA: WorkZoneConfig = ZONE_A_CONFIG;
  public zoneB: WorkZoneConfig = ZONE_B_CONFIG;
  public targetSlotOccupants: (string | null)[] = [null, null, null, null, null, null];
  public verifiedPlacedObjectIds: Set<string> = new Set();

  /**
   * Classifies a 3D coordinate into ZONE_A, ZONE_B, IN_TRANSIT, or OUTSIDE.
   */
  public getZoneForPosition(pos: [number, number, number]): ZoneClassification {
    const [x, y, z] = pos;

    // Check Zone A bounds
    const bA = this.zoneA.bounds;
    if (
      x >= bA.minX &&
      x <= bA.maxX &&
      z >= bA.minZ &&
      z <= bA.maxZ &&
      y >= bA.minY &&
      y <= bA.maxY
    ) {
      return 'ZONE_A';
    }

    // Check Zone B bounds
    const bB = this.zoneB.bounds;
    if (
      x >= bB.minX &&
      x <= bB.maxX &&
      z >= bB.minZ &&
      z <= bB.maxZ &&
      y >= bB.minY &&
      y <= bB.maxY
    ) {
      return 'ZONE_B';
    }

    // If elevated in the workspace corridor between zones (in robot grasp or transport)
    if (y > 0.04 && z >= 0.10 && z <= 0.45 && Math.abs(x) <= 0.50) {
      return 'IN_TRANSIT';
    }

    return 'OUTSIDE';
  }

  /**
   * Returns the current zone for a given active instrument.
   */
  public getObjectZone(instrument: ActiveSurgicalInstrument): ZoneClassification {
    return this.getZoneForPosition(instrument.getPosition());
  }

  /**
   * Checks if an object is inside Zone A boundaries.
   */
  public isInsideZoneA(pos: [number, number, number]): boolean {
    const [x, , z] = pos;
    const b = this.zoneA.bounds;
    return x >= b.minX && x <= b.maxX && z >= b.minZ && z <= b.maxZ;
  }

  /**
   * Checks if an object is inside Zone B boundaries.
   */
  public isInsideZoneB(pos: [number, number, number]): boolean {
    const [x, , z] = pos;
    const b = this.zoneB.bounds;
    return x >= b.minX && x <= b.maxX && z >= b.minZ && z <= b.maxZ;
  }

  /**
   * Returns the next available (unoccupied) target slot index (0..5) in Zone B, or null if all filled.
   */
  public getNextAvailableTargetSlotIndex(): number | null {
    for (let i = 0; i < 6; i++) {
      if (!this.targetSlotOccupants[i]) {
        return i;
      }
    }
    return null;
  }

  /**
   * Reserves / assigns a target slot in Zone B for an object.
   */
  public assignTargetSlot(slotIndex: number, objectId: string): boolean {
    if (slotIndex < 0 || slotIndex >= 6) return false;
    this.targetSlotOccupants[slotIndex] = objectId;
    return true;
  }

  /**
   * Marks an object as verified and stable in its assigned target slot in Zone B.
   */
  public markObjectPlacedAndStable(slotIndex: number, objectId: string): void {
    this.assignTargetSlot(slotIndex, objectId);
    this.verifiedPlacedObjectIds.add(objectId);
  }

  /**
   * Resets all slot assignments and verified placement flags.
   */
  public resetSlotOccupancy(): void {
    this.targetSlotOccupants = [null, null, null, null, null, null];
    this.verifiedPlacedObjectIds.clear();
  }

  /**
   * Checks if an object is within physical placement tolerance and stable at a specific Zone B slot.
   */
  public isObjectStableInSlot(
    instrument: ActiveSurgicalInstrument,
    slotIndex: number,
    horizontalRadiusTolerance: number = 0.055
  ): boolean {
    if (slotIndex < 0 || slotIndex >= 6) return false;
    const slotPos = this.zoneB.slots[slotIndex];
    const pos = instrument.getPosition();
    const speed = instrument.getSpeed();

    const dx = pos[0] - slotPos[0];
    const dz = pos[2] - slotPos[2];
    const horizDist = Math.hypot(dx, dz);

    const expectedY = slotPos[1] + instrument.config.dimensions.height / 2;
    const vertError = Math.abs(pos[1] - expectedY);

    const isInsideB = this.isInsideZoneB(pos);
    const isAtSlot = horizDist <= horizontalRadiusTolerance;
    const isRestingOnSurface = vertError <= 0.025;
    const isPhysicallyStable = speed <= 0.15;

    return isInsideB && isAtSlot && isRestingOnSurface && isPhysicallyStable;
  }

  /**
   * Computes a full real-time telemetry snapshot of the Dual Zone system across all objects.
   */
  public getTelemetry(instrumentFactory: InstrumentFactory): DualZoneTelemetry {
    let countA = 0;
    let countB = 0;
    let countTransit = 0;
    let countOutside = 0;

    const targetSlots: TargetSlotStatus[] = this.zoneB.slots.map((pos, idx) => ({
      index: idx,
      id: `SLOT_B${idx + 1}`,
      position: [...pos],
      isOccupied: this.targetSlotOccupants[idx] !== null,
      occupyingObjectId: this.targetSlotOccupants[idx],
      isVerifiedStable:
        this.targetSlotOccupants[idx] !== null &&
        this.verifiedPlacedObjectIds.has(this.targetSlotOccupants[idx]!)
    }));

    const sixIds = SIX_OBJECT_DEFINITIONS.map((o) => o.id);
    let totalCount = 0;

    for (const id of sixIds) {
      const inst = instrumentFactory.instruments.get(id);
      if (!inst) continue;
      totalCount++;
      const zone = this.getObjectZone(inst);

      if (zone === 'ZONE_A') {
        countA++;
      } else if (zone === 'ZONE_B') {
        countB++;
      } else if (zone === 'IN_TRANSIT') {
        countTransit++;
      } else {
        countOutside++;
      }
    }

    const placedCount = this.verifiedPlacedObjectIds.size;
    const progressPercent = totalCount > 0 ? Math.round((placedCount / totalCount) * 100) : 0;
    const isTaskComplete = totalCount > 0 && placedCount >= totalCount;

    return {
      totalObjects: totalCount,
      remainingInZoneA: countA,
      placedInZoneB: placedCount,
      inTransit: countTransit,
      outsideWorkspace: countOutside,
      progressPercent,
      isTaskComplete,
      targetSlots
    };
  }

  /**
   * Randomizes the 6 object positions inside Zone A bounds while guaranteeing
   * non-overlapping spacing (minimum 65 mm between object centers).
   */
  public randomizeZoneAPositions(instrumentFactory: InstrumentFactory): boolean {
    const minX = this.zoneA.bounds.minX + 0.040; // -0.420
    const maxX = this.zoneA.bounds.maxX - 0.040; // -0.180
    const minZ = this.zoneA.bounds.minZ + 0.035; // 0.160
    const maxZ = this.zoneA.bounds.maxZ - 0.035; // 0.330
    const minSeparation = 0.065; // 65 mm center-to-center minimum spacing

    const newPositions: [number, number, number][] = [];

    for (let i = 0; i < 6; i++) {
      let attempts = 0;
      let placed = false;
      let candX = 0;
      let candZ = 0;

      while (attempts < 100 && !placed) {
        attempts++;
        candX = minX + Math.random() * (maxX - minX);
        candZ = minZ + Math.random() * (maxZ - minZ);

        let collision = false;
        for (const p of newPositions) {
          if (Math.hypot(candX - p[0], candZ - p[2]) < minSeparation) {
            collision = true;
            break;
          }
        }
        if (!collision) {
          placed = true;
        }
      }

      if (!placed) {
        // Fall back to predefined slot position if random sampling didn't converge
        const defSlot = this.zoneA.slots[i];
        candX = defSlot[0];
        candZ = defSlot[2];
      }

      newPositions.push([candX, this.zoneA.slots[i][1], candZ]);
    }

    // Teleport instruments to new non-overlapping poses in Zone A
    const sixIds = SIX_OBJECT_DEFINITIONS.map((o) => o.id);
    for (let i = 0; i < sixIds.length; i++) {
      const id = sixIds[i];
      const inst = instrumentFactory.instruments.get(id);
      if (inst) {
        const height = inst.config.dimensions.height;
        const targetY = 0.02 + height / 2 + 0.002;
        instrumentFactory.setInstrumentPose(id, [
          newPositions[i][0],
          targetY,
          newPositions[i][2]
        ], [0, 0, 0, 1]);
      }
    }

    this.resetSlotOccupancy();
    return true;
  }
}

