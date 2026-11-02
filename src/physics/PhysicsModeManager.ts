/**
 * @file PhysicsModeManager.ts
 * Centralized mode authority manager for the Hybrid Physics Architecture (Phase 4).
 *
 * Supports two mutually exclusive simulation modes:
 * - KINEMATIC MODE (Default): Three.js FK/IK motion controller has primary motion authority;
 *   Rapier kinematic bodies follow Three.js transforms and handle contact detection.
 * - DYNAMIC MODE (Experimental): Rapier Physics has primary motion authority;
 *   dynamic rigid bodies and revolute joint motors drive the arm under gravity and contacts,
 *   and Three.js visual meshes synchronize FROM Rapier body poses.
 */

import RAPIER from '@dimforge/rapier3d-compat';
import { RobotJoints } from '../robot/RobotJoints.js';
import { RobotModel } from '../robot/RobotModel.js';
import { JointManager } from './JointManager.js';
import { PhysicsWorld } from './PhysicsWorld.js';

export type SimulationPhysicsMode = 'KINEMATIC' | 'DYNAMIC';

export interface PhysicsModeChangeCallback {
  (mode: SimulationPhysicsMode): void;
}

export class PhysicsModeManager {
  public mode: SimulationPhysicsMode = 'KINEMATIC';
  public robotJoints: RobotJoints;
  public robotModel: RobotModel;
  public jointManager: JointManager;
  public physicsWorld: PhysicsWorld;

  private listeners: PhysicsModeChangeCallback[] = [];

  constructor(params: {
    robotJoints: RobotJoints;
    robotModel: RobotModel;
    jointManager: JointManager;
    physicsWorld: PhysicsWorld;
    initialMode?: SimulationPhysicsMode;
  }) {
    this.robotJoints = params.robotJoints;
    this.robotModel = params.robotModel;
    this.jointManager = params.jointManager;
    this.physicsWorld = params.physicsWorld;

    if (params.initialMode) {
      this.setMode(params.initialMode);
    }
  }

  public get currentMode(): SimulationPhysicsMode {
    return this.mode;
  }

  public isKinematic(): boolean {
    return this.mode === 'KINEMATIC';
  }

  public isDynamic(): boolean {
    return this.mode === 'DYNAMIC';
  }

  public onModeChange(callback: PhysicsModeChangeCallback): () => void {
    this.listeners.push(callback);
    return () => {
      this.listeners = this.listeners.filter((cb) => cb !== callback);
    };
  }

  /**
   * Sets the authoritative simulation mode ('KINEMATIC' or 'DYNAMIC').
   * Guarantees safe transition without teleportation, transform fighting, or duplicate bodies.
   */
  public setMode(newMode: SimulationPhysicsMode): void {
    if (this.mode === newMode) return;

    if (newMode === 'DYNAMIC') {
      this._transitionToDynamic();
    } else {
      this._transitionToKinematic();
    }

    this.mode = newMode;
    for (const listener of this.listeners) {
      listener(this.mode);
    }
  }

  public toggleMode(): SimulationPhysicsMode {
    const next: SimulationPhysicsMode = this.mode === 'KINEMATIC' ? 'DYNAMIC' : 'KINEMATIC';
    this.setMode(next);
    return this.mode;
  }

  /**
   * Transitions arm links to dynamic rigid bodies driven by Rapier revolute joint motors.
   */
  private _transitionToDynamic(): void {
    // 1. Synchronize visual hierarchy and ensure current physical angles match targets
    const currentAnglesRad = this.robotJoints.getCurrentAnglesRad();

    for (let i = 0; i < this.robotJoints.joints.length; i++) {
      const j = this.robotJoints.joints[i];
      if (!j) continue;
      const angle = currentAnglesRad[i] ?? (this.robotJoints.config.joints[i]?.homeAngleDeg ?? 0) * (Math.PI / 180);
      j.currentAngleRad = angle;
      j.targetAngleRad = angle;
      j.angularVelocityRadPerSec = 0;
      j.targetVelocityRadPerSec = 0;
      j.motorTorqueNm = 0;
      j.isTorqueSaturated = false;

      // Re-configure Rapier joint motor to current target position
      if (j.rapierJoint) {
        j.rapierJoint.configureMotorPosition(
          angle,
          j.spec.stiffness * this.jointManager.globalStiffnessScale,
          j.spec.damping * this.jointManager.globalDampingScale
        );
        j.rapierJoint.setContactsEnabled(false);
      }
    }

    // 2. Switch all articulated link bodies from kinematic to dynamic
    for (let i = 0; i < this.robotJoints.linkBodies.length; i++) {
      const managed = this.robotJoints.linkBodies[i];
      if (!managed || !managed.body) continue;

      managed.spec.motionType = 'dynamic';
      managed.body.setBodyType(RAPIER.RigidBodyType.Dynamic, true);
      managed.body.resetForces(true);
      managed.body.resetTorques(true);
      managed.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      managed.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      managed.body.wakeUp();
    }
  }

  /**
   * Transitions arm links to kinematic position bodies driven by Three.js FK/IK.
   */
  private _transitionToKinematic(): void {
    // 1. Switch all articulated link bodies from dynamic back to kinematic
    for (let i = 0; i < this.robotJoints.linkBodies.length; i++) {
      const managed = this.robotJoints.linkBodies[i];
      if (!managed || !managed.body) continue;

      managed.spec.motionType = 'kinematicPosition';
      managed.body.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased, true);
      managed.body.resetForces(true);
      managed.body.resetTorques(true);
      managed.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      managed.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      managed.body.wakeUp();
    }

    // 2. Re-sync visual hierarchy to kinematic transforms
    this.robotModel.syncVisualHierarchyFromJoints(true);
  }
}
