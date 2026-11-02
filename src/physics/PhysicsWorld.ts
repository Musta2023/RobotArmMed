/**
 * @file PhysicsWorld.ts
 * Core Rapier3D physics world wrapper for the Surgical Action AI Digital Twin.
 * Uses SI units: meters (m), kilograms (kg), seconds (s), Newtons (N), Newton-meters (N·m).
 */

import RAPIER from '@dimforge/rapier3d-compat';
import { COLLISION_PRESETS, encodeCollisionGroups } from './CollisionGroups.js';

export interface PhysicsWorldConfig {
  gravityY?: number;
  fixedTimestep?: number;
  solverIterations?: number;
  additionalFrictionIterations?: number;
}

export class PhysicsWorld {
  public world!: RAPIER.World;
  public eventQueue!: RAPIER.EventQueue;
  public groundBody!: RAPIER.RigidBody;
  public groundCollider!: RAPIER.Collider;

  public gravityY: number;
  public fixedTimestep: number;
  public solverIterations: number;
  public additionalFrictionIterations: number;
  public simulationSpeed: number = 1.0;
  public simulationTime: number = 0;
  public stepCount: number = 0;
  public isInitialized: boolean = false;

  private accumulator: number = 0;

  constructor(config: PhysicsWorldConfig = {}) {
    this.gravityY = config.gravityY ?? -9.81;
    this.fixedTimestep = config.fixedTimestep ?? 1 / 60;
    this.solverIterations = config.solverIterations ?? 16;
    this.additionalFrictionIterations = config.additionalFrictionIterations ?? 4;
  }

  /**
   * Initializes the Rapier3D WASM module and creates the physical World and Ground collider.
   */
  public async init(): Promise<void> {
    await RAPIER.init();

    const gravity = new RAPIER.Vector3(0.0, this.gravityY, 0.0);
    this.world = new RAPIER.World(gravity);
    this.world.timestep = this.fixedTimestep;
    this.world.numSolverIterations = this.solverIterations;
    this.world.numAdditionalFrictionIterations = this.additionalFrictionIterations;
    if ('numInternalPgsIterations' in this.world.integrationParameters) {
      this.world.integrationParameters.numInternalPgsIterations = 2;
    }

    this.eventQueue = new RAPIER.EventQueue(true);

    // Create fixed ground slab (top surface at y = 0.0 m)
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.05, 0);
    this.groundBody = this.world.createRigidBody(groundBodyDesc);

    const groundGroups = encodeCollisionGroups(
      COLLISION_PRESETS.GROUND_AND_PLATFORMS.membership,
      COLLISION_PRESETS.GROUND_AND_PLATFORMS.filter
    );

    const groundColliderDesc = RAPIER.ColliderDesc.cuboid(2.0, 0.05, 2.0)
      .setFriction(0.60)
      .setRestitution(0.02)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Average)
      .setRestitutionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setCollisionGroups(groundGroups)
      .setSolverGroups(groundGroups)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS)
      .setActiveCollisionTypes(
        RAPIER.ActiveCollisionTypes.DEFAULT |
          RAPIER.ActiveCollisionTypes.KINEMATIC_FIXED
      );

    if (typeof groundColliderDesc.setContactSkin === 'function') {
      groundColliderDesc.setContactSkin(0.0005);
    }

    this.groundCollider = this.world.createCollider(groundColliderDesc, this.groundBody);
    this.isInitialized = true;
  }

  /**
   * Updates vertical gravity acceleration in m/s² (e.g., -9.81 for standard Earth gravity).
   * Wakes up all sleeping dynamic rigid bodies so the change takes effect immediately.
   */
  public setGravity(gravityY: number): void {
    this.gravityY = Number.isFinite(gravityY) ? gravityY : -9.81;
    if (this.world) {
      this.world.gravity = new RAPIER.Vector3(0, this.gravityY, 0);
      this.world.bodies.forEach((body) => {
        if (body.isDynamic()) {
          body.wakeUp();
        }
      });
    }
  }

  /**
   * Executes a single deterministic fixed-timestep physics step.
   */
  public stepSingle(): void {
    if (!this.isInitialized) return;
    this.world.timestep = this.fixedTimestep;
    this.world.step(this.eventQueue);
    this.simulationTime += this.fixedTimestep;
    this.stepCount += 1;
  }

  /**
   * Advances the physics simulation using an accumulator to guarantee a fixed physics timestep
   * regardless of rendering frame rate.
   *
   * @param deltaSeconds Elapsed wall-clock time since last frame
   * @param onBeforeSubstep Optional hook executed before each fixed physics substep
   * @param onAfterSubstep Optional hook executed after each fixed physics substep
   * @returns Number of fixed substeps executed
   */
  public step(
    deltaSeconds: number,
    onBeforeSubstep?: (dt: number) => void,
    onAfterSubstep?: (dt: number) => void
  ): number {
    if (!this.isInitialized) return 0;

    const scaledDelta = Math.min(Math.max(deltaSeconds, 0), 0.1) * this.simulationSpeed;
    this.accumulator += scaledDelta;

    let substeps = 0;
    const maxSubsteps = 8;

    while (this.accumulator >= this.fixedTimestep && substeps < maxSubsteps) {
      if (onBeforeSubstep) {
        onBeforeSubstep(this.fixedTimestep);
      }
      this.stepSingle();
      if (onAfterSubstep) {
        onAfterSubstep(this.fixedTimestep);
      }
      this.accumulator -= this.fixedTimestep;
      substeps += 1;
    }

    return substeps;
  }

  /**
   * Resets simulation clock and step counters.
   */
  public resetClock(): void {
    this.simulationTime = 0;
    this.stepCount = 0;
    this.accumulator = 0;
  }
}
