/**
 * @file InstrumentFactory.ts
 * Spawns, removes, repositions, and modifies dummy surgical instruments with
 * paired Three.js meshes and dynamic Rapier3D RigidBodies.
 */

import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import {
  InstrumentPresetType,
  SurgicalInstrumentConfig,
  INSTRUMENT_PRESETS
} from './ObjectProperties.js';
import {
  RigidBodyManager,
  ManagedRigidBody
} from '../physics/RigidBodyManager.js';
import { MATERIAL_LIBRARY, MaterialType, computeEffectiveFriction } from '../physics/MaterialProperties.js';

export interface ObjectGraspabilityInfo {
  isGraspable: boolean;
  fitsGripperOpening: boolean;
  isWithinReachableWorkspace: boolean;
  hasSufficientFrictionCapacity: boolean;
  graspWidthMeters: number;
  distanceFromShoulderMeters: number;
  requiredNormalForceNewtons: number;
  reason: string;
}

export interface ActiveSurgicalInstrument {
  id: string;
  config: SurgicalInstrumentConfig;
  managedBody: ManagedRigidBody;
  mesh: THREE.Group;
  getPosition(): [number, number, number];
  getOrientation(): [number, number, number, number];
  getDimensions(): { width: number; height: number; depth: number; radius?: number };
  getMass(): number;
  getCollider(): RAPIER.Collider;
  getPhysicalProperties(): {
    massKg: number;
    friction: number;
    restitution: number;
    materialType: MaterialType;
  };
  getGraspabilityInfo(
    maxGripperOpeningMeters?: number,
    maxGrippingForceN?: number
  ): ObjectGraspabilityInfo;
  getLinearVelocity(): [number, number, number];
  getSpeed(): number;
}

export class InstrumentFactory {
  public bodyManager: RigidBodyManager;
  public sceneGroup: THREE.Group;
  public instruments: Map<string, ActiveSurgicalInstrument> = new Map();
  public activeInstrumentId: string | null = null;
  private counter: number = 1;

  constructor(bodyManager: RigidBodyManager, parentScene?: THREE.Scene | THREE.Group) {
    this.bodyManager = bodyManager;
    this.sceneGroup = new THREE.Group();
    this.sceneGroup.name = 'SurgicalInstrumentsGroup';
    if (parentScene) {
      parentScene.add(this.sceneGroup);
    }
  }

  /**
   * Spawns a new dynamic surgical instrument in the Rapier3D physics world and Three.js scene.
   */
  public spawnInstrument(
    presetType: InstrumentPresetType = 'rectangular_instrument',
    overrides: Partial<SurgicalInstrumentConfig> & { position?: [number, number, number] } = {}
  ): ActiveSurgicalInstrument {
    const preset = INSTRUMENT_PRESETS[presetType];
    const id = overrides.id ?? `instrument_${this.counter++}`;
    const resolvedPos = overrides.position ?? overrides.initialPosition ?? preset.initialPosition;

    const cfg: SurgicalInstrumentConfig = {
      ...preset,
      ...overrides,
      id,
      dimensions: {
        ...preset.dimensions,
        ...(overrides.dimensions ?? {})
      },
      initialPosition: [...resolvedPos],
      initialQuaternion: overrides.initialQuaternion
        ? [...overrides.initialQuaternion]
        : [...preset.initialQuaternion]
    };

    const meshGroup = this._createInstrumentMesh(cfg);
    meshGroup.position.set(
      cfg.initialPosition[0],
      cfg.initialPosition[1],
      cfg.initialPosition[2]
    );
    meshGroup.quaternion.set(
      cfg.initialQuaternion[0],
      cfg.initialQuaternion[1],
      cfg.initialQuaternion[2],
      cfg.initialQuaternion[3]
    );
    this.sceneGroup.add(meshGroup);

    const managedBody = this.bodyManager.createBody({
      id: cfg.id,
      name: cfg.name,
      category: 'instrument',
      motionType: 'dynamic',
      shape: cfg.shape,
      dimensions: cfg.dimensions,
      position: cfg.initialPosition,
      quaternion: cfg.initialQuaternion,
      massKg: cfg.massKg,
      friction: cfg.friction,
      restitution: cfg.restitution,
      materialType: cfg.materialType,
      mesh: meshGroup
    });

    const active: ActiveSurgicalInstrument = {
      id: cfg.id,
      config: cfg,
      managedBody,
      mesh: meshGroup,
      getPosition: (): [number, number, number] => {
        const t = managedBody.body.translation();
        return [t.x, t.y, t.z];
      },
      getOrientation: (): [number, number, number, number] => {
        const r = managedBody.body.rotation();
        return [r.x, r.y, r.z, r.w];
      },
      getDimensions: () => ({ ...cfg.dimensions }),
      getMass: (): number => cfg.massKg,
      getCollider: (): RAPIER.Collider => managedBody.collider,
      getPhysicalProperties: () => ({
        massKg: cfg.massKg,
        friction: cfg.friction,
        restitution: cfg.restitution,
        materialType: cfg.materialType
      }),
      getGraspabilityInfo: (
        maxGripperOpeningMeters: number = 0.090,
        maxGrippingForceN: number = 35.0
      ): ObjectGraspabilityInfo => {
        const t = managedBody.body.translation();
        const shoulderOrigin = new THREE.Vector3(0, 0.26, 0);
        const objPos = new THREE.Vector3(t.x, t.y, t.z);
        const distFromShoulder = objPos.distanceTo(shoulderOrigin);
        const maxReach = 0.78;

        const graspWidth = Math.min(cfg.dimensions.width, cfg.dimensions.depth);
        const fitsOpening =
          graspWidth >= 0.003 && graspWidth <= maxGripperOpeningMeters - 0.004;
        const isReachable = distFromShoulder <= maxReach && t.y >= 0.005;

        const muEff = computeEffectiveFriction(0.90, cfg.friction, 'rubber', cfg.materialType);
        const requiredNormalForce = (cfg.massKg * 9.81 * 1.05) / Math.max(2 * muEff, 1e-4);
        const hasForce = requiredNormalForce <= maxGrippingForceN;

        const isGraspable = fitsOpening && isReachable && hasForce;
        let reason = 'Object is compatible and within reachable workspace.';
        if (!fitsOpening) {
          reason = `Object width (${(graspWidth * 1000).toFixed(1)} mm) exceeds gripper opening (${(maxGripperOpeningMeters * 1000).toFixed(1)} mm).`;
        } else if (!isReachable) {
          reason = `Object at distance ${distFromShoulder.toFixed(2)} m exceeds max workspace reach (${maxReach.toFixed(2)} m).`;
        } else if (!hasForce) {
          reason = `Object mass (${cfg.massKg.toFixed(2)} kg) requires ${requiredNormalForce.toFixed(1)} N normal force (max ${maxGrippingForceN.toFixed(1)} N).`;
        }

        return {
          isGraspable,
          fitsGripperOpening: fitsOpening,
          isWithinReachableWorkspace: isReachable,
          hasSufficientFrictionCapacity: hasForce,
          graspWidthMeters: graspWidth,
          distanceFromShoulderMeters: distFromShoulder,
          requiredNormalForceNewtons: requiredNormalForce,
          reason
        };
      },
      getLinearVelocity: (): [number, number, number] => {
        const lv = managedBody.body.linvel();
        return [lv.x, lv.y, lv.z];
      },
      getSpeed: (): number => {
        const lv = managedBody.body.linvel();
        return Math.hypot(lv.x, lv.y, lv.z);
      }
    };

    this.instruments.set(cfg.id, active);
    if (!this.activeInstrumentId) {
      this.activeInstrumentId = cfg.id;
    }

    return active;
  }

  private _createInstrumentMesh(cfg: SurgicalInstrumentConfig): THREE.Group {
    const group = new THREE.Group();
    group.name = `InstrumentMesh_${cfg.id}`;
    group.userData.instrumentId = cfg.id;

    const matDef = MATERIAL_LIBRARY[cfg.materialType] ?? MATERIAL_LIBRARY.surgical_steel;
    const mainMat = new THREE.MeshStandardMaterial({
      color: cfg.colorHex,
      metalness: matDef.metalness,
      roughness: matDef.roughness
    });

    const dims = cfg.dimensions;
    let bodyGeo: THREE.BufferGeometry;
    if (cfg.shape === 'cylinder') {
      const r = dims.radius ?? dims.width / 2;
      bodyGeo = new THREE.CylinderGeometry(r, r, dims.height, 28);
    } else {
      bodyGeo = new THREE.BoxGeometry(dims.width, dims.height, dims.depth);
    }

    const bodyMesh = new THREE.Mesh(bodyGeo, mainMat);
    bodyMesh.castShadow = true;
    bodyMesh.receiveShadow = true;
    bodyMesh.userData.instrumentId = cfg.id;
    group.add(bodyMesh);

    // Subtle surgical grip band around the grasp center
    const bandGeo = new THREE.BoxGeometry(
      dims.width + 0.0015,
      0.006,
      dims.depth + 0.0015
    );
    const bandMat = new THREE.MeshBasicMaterial({ color: 0xfacc15 });
    const bandMesh = new THREE.Mesh(bandGeo, bandMat);
    bandMesh.userData.instrumentId = cfg.id;
    group.add(bandMesh);

    return group;
  }

  /**
   * Removes an instrument from both Three.js and Rapier3D.
   */
  public removeInstrument(id: string): boolean {
    const inst = this.instruments.get(id);
    if (!inst) return false;

    this.sceneGroup.remove(inst.mesh);
    this.bodyManager.removeBody(id);
    this.instruments.delete(id);

    if (this.activeInstrumentId === id) {
      const firstKey = this.instruments.keys().next().value;
      this.activeInstrumentId = firstKey ?? null;
    }
    return true;
  }

  /**
   * Returns the primary active surgical instrument.
   */
  public getActiveInstrument(): ActiveSurgicalInstrument | undefined {
    if (this.activeInstrumentId && this.instruments.has(this.activeInstrumentId)) {
      return this.instruments.get(this.activeInstrumentId);
    }
    const first = this.instruments.values().next().value;
    if (first) {
      this.activeInstrumentId = first.config.id;
    }
    return first;
  }

  /**
   * Updates an instrument's mass (kg) and inertia tensor in Rapier3D.
   */
  public setInstrumentMass(id: string, massKg: number): void {
    const inst = this.instruments.get(id);
    if (!inst) return;
    inst.config.massKg = Math.max(0.01, massKg);
    this.bodyManager.setBodyMass(id, inst.config.massKg);
  }

  /**
   * Updates an instrument's friction, restitution, and optional material preset.
   */
  public setInstrumentMaterial(
    id: string,
    friction: number,
    restitution: number,
    materialType?: MaterialType
  ): void {
    const inst = this.instruments.get(id);
    if (!inst) return;
    if (materialType) {
      inst.config.materialType = materialType;
    }
    inst.config.friction = Math.max(0, friction);
    inst.config.restitution = Math.min(Math.max(restitution, 0), 1);
    this.bodyManager.setBodyMaterialProperties(
      id,
      inst.config.friction,
      inst.config.restitution
    );
  }

  /**
   * Repositions and reorients an instrument in the physics world.
   */
  public setInstrumentPose(
    id: string,
    position: [number, number, number],
    quaternion: [number, number, number, number] = [0, 0, 0, 1]
  ): void {
    const inst = this.instruments.get(id);
    if (!inst) return;
    this.bodyManager.setBodyPose(id, position, quaternion);
  }

  /**
   * Resets all existing instruments to their initial positions and zero velocity.
   */
  public resetAllInstruments(): void {
    for (const [id, inst] of this.instruments) {
      this.bodyManager.setBodyPose(
        id,
        inst.config.initialPosition,
        inst.config.initialQuaternion
      );
    }
  }

  /**
   * Returns current world position and quaternion of an instrument from its Rapier body.
   */
  public getInstrumentWorldPose(id: string): {
    position: THREE.Vector3;
    quaternion: THREE.Quaternion;
    linearVelocity: THREE.Vector3;
    angularVelocity: THREE.Vector3;
  } | null {
    const inst = this.instruments.get(id);
    if (!inst) return null;

    const t = inst.managedBody.body.translation();
    const r = inst.managedBody.body.rotation();
    const lv = inst.managedBody.body.linvel();
    const av = inst.managedBody.body.angvel();

    return {
      position: new THREE.Vector3(t.x, t.y, t.z),
      quaternion: new THREE.Quaternion(r.x, r.y, r.z, r.w),
      linearVelocity: new THREE.Vector3(lv.x, lv.y, lv.z),
      angularVelocity: new THREE.Vector3(av.x, av.y, av.z)
    };
  }
}
