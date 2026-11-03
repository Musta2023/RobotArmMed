/**
 * @file ScenarioGenerator.ts
 * Deterministic, seedable scenario generator for reproducible ML training demonstrations.
 * Supports Difficulty Levels 1 through 5, non-overlapping spatial validation,
 * kinematic workspace constraints, and robot configuration perturbations.
 */

import * as THREE from 'three';
import { InstrumentFactory, ActiveSurgicalInstrument } from './InstrumentFactory.js';
import { WorkZoneManager } from './WorkZoneManager.js';
import { SIX_OBJECT_DEFINITIONS, ZONE_A_CONFIG, ZONE_B_CONFIG } from './ObjectProperties.js';
import { RobotModel } from '../robot/RobotModel.js';

export type ScenarioDifficultyLevel = 1 | 2 | 3 | 4 | 5;

export interface ObjectSpawnSpec {
  id: string;
  name: string;
  type: 'box' | 'cylinder' | 'irregular';
  position: [number, number, number];
  quaternion: [number, number, number, number];
  dimensions: { width: number; height: number; depth: number; radius?: number };
  mass_kg: number;
  friction: number;
  target_slot_index: number;
  target_position: [number, number, number];
}

export interface ScenarioDefinition {
  scenario_id: string;
  seed: number;
  difficulty_level: ScenarioDifficultyLevel;
  description: string;
  objects: ObjectSpawnSpec[];
  initial_robot_joint_angles_deg: number[];
  workspace_radius_min_m: number;
  workspace_radius_max_m: number;
  target_tolerance_m: number;
  is_perturbation_scenario: boolean;
}

/**
 * Deterministic Mulberry32 Pseudo-Random Number Generator.
 * Produces high-quality, reproducible uniform random numbers in [0, 1).
 */
export class SeededPRNG {
  private _state: number;

  constructor(seed: number = 42) {
    this._state = Math.floor(Math.abs(seed)) || 1;
  }

  public next(): number {
    let t = (this._state += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  public range(min: number, max: number): number {
    return min + this.next() * (max - min);
  }

  public choice<T>(items: T[]): T {
    const idx = Math.floor(this.next() * items.length);
    return items[Math.min(idx, items.length - 1)];
  }
}

export class ScenarioGenerator {
  public instrumentFactory: InstrumentFactory;
  public workZoneManager?: WorkZoneManager;
  public robotModel?: RobotModel;

  public currentScenario: ScenarioDefinition | null = null;
  public minSpawnSeparationMeters: number = 0.065;
  public minRadialReachMeters: number = 0.23;
  public maxRadialReachMeters: number = 0.48;

  constructor(params: {
    instrumentFactory: InstrumentFactory;
    workZoneManager?: WorkZoneManager;
    robotModel?: RobotModel;
  }) {
    this.instrumentFactory = params.instrumentFactory;
    this.workZoneManager = params.workZoneManager;
    this.robotModel = params.robotModel;
  }

  /**
   * Generates a fully validated scenario with reproducible seed and difficulty level.
   */
  public generateScenario(
    level: ScenarioDifficultyLevel = 2,
    seed: number = Date.now()
  ): ScenarioDefinition {
    const prng = new SeededPRNG(seed);
    const scenarioId = `scen_lvl${level}_s${seed.toString(36)}_${Math.floor(prng.next() * 1000).toString().padStart(3, '0')}`;

    let objectCount = 1;
    let isPerturbed = false;

    switch (level) {
      case 1:
        objectCount = 1;
        break;
      case 2:
        objectCount = 1;
        break;
      case 3:
        objectCount = 1;
        break;
      case 4:
        objectCount = 6;
        break;
      case 5:
        objectCount = 2;
        isPerturbed = true;
        break;
    }

    const spawnedObjects: ObjectSpawnSpec[] = [];
    const usedPositions: [number, number, number][] = [];

    const slotTargets = this.workZoneManager?.zoneB.slots ?? [
      [0.22, 0.02, 0.20],
      [0.30, 0.02, 0.20],
      [0.38, 0.02, 0.20],
      [0.22, 0.02, 0.29],
      [0.30, 0.02, 0.29],
      [0.38, 0.02, 0.29]
    ];

    for (let i = 0; i < objectCount; i++) {
      const baseDef = SIX_OBJECT_DEFINITIONS[i % SIX_OBJECT_DEFINITIONS.length];
      const targetSlot = slotTargets[i % slotTargets.length];

      let spawnPos: [number, number, number];
      let spawnQuat: [number, number, number, number] = [0, 0, 0, 1];
      let attempts = 0;
      let valid = false;

      const halfHeight = baseDef.dimensions.height / 2;
      const topY = ZONE_A_CONFIG.center[1] + ZONE_A_CONFIG.dimensions.height / 2;
      const spawnY = topY + halfHeight + 0.002;

      const minX = ZONE_A_CONFIG.center[0] - ZONE_A_CONFIG.dimensions.width / 2;
      const maxX = ZONE_A_CONFIG.center[0] + ZONE_A_CONFIG.dimensions.width / 2;
      const minZ = ZONE_A_CONFIG.center[2] - ZONE_A_CONFIG.dimensions.depth / 2;
      const maxZ = ZONE_A_CONFIG.center[2] + ZONE_A_CONFIG.dimensions.depth / 2;

      while (attempts < 200 && !valid) {
        attempts++;

        if (level === 1) {
          // Easy fixed canonical position in Zone A center
          spawnPos = [ZONE_A_CONFIG.center[0], spawnY, ZONE_A_CONFIG.center[2]];
          spawnQuat = [0, 0, 0, 1];
        } else if (level === 2) {
          // Randomized position & yaw within Zone A
          const rx = prng.range(minX + 0.04, maxX - 0.04);
          const rz = prng.range(minZ + 0.04, maxZ - 0.04);
          spawnPos = [rx, spawnY, rz];

          const yaw = prng.range(-Math.PI, Math.PI);
          const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
          spawnQuat = [q.x, q.y, q.z, q.w];
        } else if (level === 3) {
          // Difficult reachable positions near workspace boundary
          const angle = prng.range(-2.2, -0.6); // azimuthal quadrant towards Zone A
          const radius = prng.range(0.24, 0.46);
          const rx = Math.sin(angle) * radius;
          const rz = Math.cos(angle) * radius;
          spawnPos = [rx, spawnY, rz];

          const yaw = prng.range(-Math.PI, Math.PI);
          const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
          spawnQuat = [q.x, q.y, q.z, q.w];
        } else if (level === 4) {
          // Multi-object grid slots with small jitter
          const col = i % 3;
          const row = Math.floor(i / 3);
          const gridX = -0.38 + col * 0.08 + prng.range(-0.015, 0.015);
          const gridZ = 0.20 + row * 0.09 + prng.range(-0.015, 0.015);
          spawnPos = [gridX, spawnY, gridZ];

          const yaw = prng.range(-0.4, 0.4);
          const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
          spawnQuat = [q.x, q.y, q.z, q.w];
        } else {
          // Level 5: Perturbed scenario
          const rx = prng.range(minX + 0.05, maxX - 0.05);
          const rz = prng.range(minZ + 0.05, maxZ - 0.05);
          spawnPos = [rx, spawnY, rz];
          const yaw = prng.range(-Math.PI, Math.PI);
          const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
          spawnQuat = [q.x, q.y, q.z, q.w];
        }

        // Check radial reachability from shoulder origin (0, 0)
        const rad = Math.hypot(spawnPos[0], spawnPos[2]);
        if (rad < this.minRadialReachMeters || rad > this.maxRadialReachMeters) {
          continue;
        }

        // Check pairwise distance to prevent mesh overlap
        const isOverlap = usedPositions.some((p) => {
          const dist = Math.hypot(p[0] - spawnPos[0], p[2] - spawnPos[2]);
          return dist < this.minSpawnSeparationMeters;
        });

        if (!isOverlap) {
          valid = true;
          usedPositions.push(spawnPos);
        }
      }

      // Physics randomization for controlled domain randomization in level 5
      const massJitter = isPerturbed ? prng.range(0.9, 1.15) : 1.0;
      const frictionJitter = isPerturbed ? prng.range(0.85, 1.15) : 1.0;

      spawnedObjects.push({
        id: baseDef.id,
        name: baseDef.name,
        type: baseDef.type,
        position: spawnPos!,
        quaternion: spawnQuat,
        dimensions: { ...baseDef.dimensions },
        mass_kg: baseDef.mass_kg * massJitter,
        friction: 0.95 * frictionJitter,
        target_slot_index: i,
        target_position: [targetSlot[0], targetSlot[1], targetSlot[2]]
      });
    }

    // Default robot home configuration with slight level-dependent perturbation
    const homeAnglesDeg = [0, 45, -30, 0, -15, 0];
    if (level >= 3) {
      homeAnglesDeg[0] += prng.range(-5, 5);
      homeAnglesDeg[1] += prng.range(-3, 3);
      homeAnglesDeg[2] += prng.range(-3, 3);
    }

    const scenario: ScenarioDefinition = {
      scenario_id: scenarioId,
      seed,
      difficulty_level: level,
      description: `Level ${level} manipulation scenario with ${objectCount} object(s). Seed=${seed}.`,
      objects: spawnedObjects,
      initial_robot_joint_angles_deg: homeAnglesDeg,
      workspace_radius_min_m: this.minRadialReachMeters,
      workspace_radius_max_m: this.maxRadialReachMeters,
      target_tolerance_m: 0.045,
      is_perturbation_scenario: isPerturbed
    };

    this.currentScenario = scenario;
    return scenario;
  }

  /**
   * Applies the scenario definition directly to the simulation world.
   */
  public applyScenario(scenario: ScenarioDefinition): void {
    this.currentScenario = scenario;

    // Reset zone slots
    if (this.workZoneManager) {
      this.workZoneManager.resetSlotOccupancy();
    }

    // Position all objects according to scenario specifications
    scenario.objects.forEach((objSpec, index) => {
      const inst = this.instrumentFactory.instruments.get(objSpec.id);
      if (inst) {
        this.instrumentFactory.setInstrumentPose(
          objSpec.id,
          objSpec.position,
          objSpec.quaternion
        );
        // Apply randomized physics properties
        inst.managedBody.body.setMass(objSpec.mass_kg, true);
        inst.managedBody.colliders.forEach((c) => {
          c.setFriction(objSpec.friction);
        });
      }
    });

    // Set first object active
    if (scenario.objects.length > 0) {
      this.instrumentFactory.activeInstrumentId = scenario.objects[0].id;
    }

    // Configure initial robot joints
    if (this.robotModel) {
      this.robotModel.setJointAnglesDeg(scenario.initial_robot_joint_angles_deg);
    }
  }
}
