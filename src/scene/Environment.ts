/**
 * @file Environment.ts
 * 3D Surgical Workcell Environment inspired by the NVIDIA Isaac Sim reference viewport:
 * - Dark reflective ground plane with glowing orange/amber engineering grid lines
 * - Pickup Station Pad (Left: [-0.30, 0, 0.24]) with fixed Rapier3D collider
 * - Target Surgical Tray (Right: [+0.30, 0, 0.24]) with fixed Rapier3D collider
 * - Configurable physical obstacle block for collision validation
 */

import * as THREE from 'three';
import { RigidBodyManager } from '../physics/RigidBodyManager.js';
import {
  PICKUP_STATION_POS,
  TARGET_TRAY_POS
} from '../objects/ObjectProperties.js';

export class Environment {
  public bodyManager: RigidBodyManager;
  public root: THREE.Group;
  public obstacleMesh!: THREE.Mesh;
  public obstacleEnabled: boolean = false;

  constructor(bodyManager: RigidBodyManager) {
    this.bodyManager = bodyManager;
    this.root = new THREE.Group();
    this.root.name = 'SurgicalEnvironmentRoot';

    this._buildFloorAndIsaacGrid();
    this._buildPickupAndTargetTrays();
  }

  private _buildFloorAndIsaacGrid(): void {
    // Dark obsidian ground plane at y = 0
    const floorGeo = new THREE.PlaneGeometry(4.0, 4.0);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x121316,
      metalness: 0.25,
      roughness: 0.72
    });
    const floorMesh = new THREE.Mesh(floorGeo, floorMat);
    floorMesh.rotation.x = -Math.PI / 2;
    floorMesh.receiveShadow = true;
    this.root.add(floorMesh);

    // Isaac Sim style glowing amber/orange major + minor grid
    const majorGrid = new THREE.GridHelper(3.2, 16, 0xf59e0b, 0x92400e);
    majorGrid.position.set(0, 0.0008, 0);
    this.root.add(majorGrid);

    const fineGrid = new THREE.GridHelper(1.6, 32, 0xb45309, 0x3f2305);
    fineGrid.position.set(0, 0.0004, 0);
    this.root.add(fineGrid);

    // World reference coordinate axes near origin
    const worldAxes = new THREE.AxesHelper(0.12);
    worldAxes.position.set(-0.48, 0.002, -0.25);
    this.root.add(worldAxes);
  }

  private _buildPickupAndTargetTrays(): void {
    const padHeight = 0.02;
    const padWidth = 0.16;
    const padDepth = 0.16;

    // 1. Left Pickup Station Pad ([-0.30, 0.01, 0.24], top surface at y = 0.02 m)
    const pickupMat = new THREE.MeshStandardMaterial({
      color: 0x065f46,
      emissive: 0x064e3b,
      emissiveIntensity: 0.35,
      metalness: 0.35,
      roughness: 0.45
    });
    const pickupMesh = new THREE.Mesh(
      new THREE.BoxGeometry(padWidth, padHeight, padDepth),
      pickupMat
    );
    pickupMesh.position.set(
      PICKUP_STATION_POS[0],
      padHeight / 2,
      PICKUP_STATION_POS[2]
    );
    pickupMesh.receiveShadow = true;
    this.root.add(pickupMesh);

    this.bodyManager.createBody({
      id: 'pickup_station_pad',
      name: 'Pickup Station Pad (Zone A)',
      category: 'environment',
      motionType: 'fixed',
      shape: 'box',
      dimensions: { width: padWidth, height: padHeight, depth: padDepth },
      position: [PICKUP_STATION_POS[0], padHeight / 2, PICKUP_STATION_POS[2]],
      massKg: 10,
      friction: 0.75,
      restitution: 0.05
    });

    // 2. Right Target Surgical Tray ([+0.30, 0.01, 0.24], top surface at y = 0.02 m)
    const trayMat = new THREE.MeshStandardMaterial({
      color: 0x1e40af,
      emissive: 0x1e3a8a,
      emissiveIntensity: 0.35,
      metalness: 0.4,
      roughness: 0.4
    });
    const trayMesh = new THREE.Mesh(
      new THREE.BoxGeometry(padWidth, padHeight, padDepth),
      trayMat
    );
    trayMesh.position.set(
      TARGET_TRAY_POS[0],
      padHeight / 2,
      TARGET_TRAY_POS[2]
    );
    trayMesh.receiveShadow = true;
    this.root.add(trayMesh);

    // Target reticle ring on Target Tray
    const ringMesh = new THREE.Mesh(
      new THREE.RingGeometry(0.026, 0.032, 28).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: 0x60a5fa, side: THREE.DoubleSide })
    );
    ringMesh.position.set(TARGET_TRAY_POS[0], padHeight + 0.001, TARGET_TRAY_POS[2]);
    this.root.add(ringMesh);

    this.bodyManager.createBody({
      id: 'target_surgical_tray',
      name: 'Target Surgical Tray (Zone B)',
      category: 'environment',
      motionType: 'fixed',
      shape: 'box',
      dimensions: { width: padWidth, height: padHeight, depth: padDepth },
      position: [TARGET_TRAY_POS[0], padHeight / 2, TARGET_TRAY_POS[2]],
      massKg: 10,
      friction: 0.75,
      restitution: 0.05
    });

    // 3. Configurable Obstacle Pillar (disabled by default; can be enabled for collision testing)
    const obsMat = new THREE.MeshStandardMaterial({
      color: 0xef4444,
      metalness: 0.3,
      roughness: 0.5,
      transparent: true,
      opacity: 0.85
    });
    this.obstacleMesh = new THREE.Mesh(
      new THREE.BoxGeometry(0.10, 0.34, 0.12),
      obsMat
    );
    this.obstacleMesh.position.set(0.0, 0.17, 0.25);
    this.obstacleMesh.visible = false;
    this.root.add(this.obstacleMesh);
  }

  /**
   * Enables or disables the physical test obstacle in the workspace corridor.
   */
  public setObstacleEnabled(
    enabled: boolean,
    position: [number, number, number] = [0.0, 0.17, 0.25]
  ): void {
    this.obstacleEnabled = enabled;
    this.obstacleMesh.position.set(position[0], position[1], position[2]);
    this.obstacleMesh.visible = enabled;

    if (enabled) {
      this.bodyManager.createBody({
        id: 'workcell_obstacle',
        name: 'Surgical Workspace Obstacle',
        category: 'obstacle',
        motionType: 'fixed',
        shape: 'box',
        dimensions: { width: 0.10, height: 0.34, depth: 0.12 },
        position,
        massKg: 25,
        friction: 0.6,
        restitution: 0.1
      });
    } else {
      this.bodyManager.removeBody('workcell_obstacle');
    }
  }
}
