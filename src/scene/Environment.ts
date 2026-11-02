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
  TARGET_TRAY_POS,
  ZONE_A_CONFIG,
  ZONE_B_CONFIG
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
    const padH = ZONE_A_CONFIG.dimensions.height; // 0.02 m
    const padW = ZONE_A_CONFIG.dimensions.width;  // 0.32 m
    const padD = ZONE_A_CONFIG.dimensions.depth;  // 0.24 m

    // =========================================================================
    // 1. ZONE A — START (Large Blue Physical Pickup Platform)
    // =========================================================================
    const zoneAMat = new THREE.MeshStandardMaterial({
      color: 0x1d4ed8, // Vibrant Cobalt Blue
      emissive: 0x1e3a8a,
      emissiveIntensity: 0.40,
      metalness: 0.35,
      roughness: 0.42
    });
    const zoneAMesh = new THREE.Mesh(
      new THREE.BoxGeometry(padW, padH, padD),
      zoneAMat
    );
    zoneAMesh.name = 'Platform_ZoneA_Start';
    zoneAMesh.position.set(
      ZONE_A_CONFIG.center[0],
      padH / 2,
      ZONE_A_CONFIG.center[2]
    );
    zoneAMesh.receiveShadow = true;
    this.root.add(zoneAMesh);

    // Glowing border frame around Zone A
    const frameGeoA = new THREE.BoxGeometry(padW + 0.008, 0.004, padD + 0.008);
    const frameMatA = new THREE.MeshBasicMaterial({ color: 0x60a5fa, wireframe: true });
    const frameA = new THREE.Mesh(frameGeoA, frameMatA);
    frameA.position.set(ZONE_A_CONFIG.center[0], padH, ZONE_A_CONFIG.center[2]);
    this.root.add(frameA);

    // 6 Clearly marked pickup bays/slots [A1..A6]
    for (let i = 0; i < ZONE_A_CONFIG.slots.length; i++) {
      const slot = ZONE_A_CONFIG.slots[i];
      // Subtle slot base pad
      const slotPad = new THREE.Mesh(
        new THREE.BoxGeometry(0.054, 0.001, 0.054),
        new THREE.MeshStandardMaterial({
          color: 0x1e3a8a,
          roughness: 0.3,
          metalness: 0.5
        })
      );
      slotPad.position.set(slot[0], padH + 0.0005, slot[2]);
      this.root.add(slotPad);

      // Cyan ring indicator
      const slotRing = new THREE.Mesh(
        new THREE.RingGeometry(0.016, 0.021, 24).rotateX(-Math.PI / 2),
        new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide })
      );
      slotRing.position.set(slot[0], padH + 0.001, slot[2]);
      this.root.add(slotRing);
    }

    // Register physical Rapier body for Zone A
    const zoneABody = this.bodyManager.createBody({
      id: 'zone_a_start_platform',
      name: 'ZONE A — START Platform',
      category: 'environment',
      motionType: 'fixed',
      shape: 'box',
      dimensions: { width: padW, height: padH, depth: padD },
      position: [ZONE_A_CONFIG.center[0], padH / 2, ZONE_A_CONFIG.center[2]],
      massKg: 15,
      friction: 0.75,
      restitution: 0.05
    });
    // Alias for backwards compatibility with tests expecting pickup_station_pad
    this.bodyManager.bodies.set('pickup_station_pad', zoneABody);

    // =========================================================================
    // 2. ZONE B — TARGET (Large Green Physical Placement Platform)
    // =========================================================================
    const zoneBMat = new THREE.MeshStandardMaterial({
      color: 0x059669, // Vibrant Emerald Green
      emissive: 0x064e3b,
      emissiveIntensity: 0.40,
      metalness: 0.35,
      roughness: 0.42
    });
    const zoneBMesh = new THREE.Mesh(
      new THREE.BoxGeometry(padW, padH, padD),
      zoneBMat
    );
    zoneBMesh.name = 'Platform_ZoneB_Target';
    zoneBMesh.position.set(
      ZONE_B_CONFIG.center[0],
      padH / 2,
      ZONE_B_CONFIG.center[2]
    );
    zoneBMesh.receiveShadow = true;
    this.root.add(zoneBMesh);

    // Glowing border frame around Zone B
    const frameGeoB = new THREE.BoxGeometry(padW + 0.008, 0.004, padD + 0.008);
    const frameMatB = new THREE.MeshBasicMaterial({ color: 0x34d399, wireframe: true });
    const frameB = new THREE.Mesh(frameGeoB, frameMatB);
    frameB.position.set(ZONE_B_CONFIG.center[0], padH, ZONE_B_CONFIG.center[2]);
    this.root.add(frameB);

    // 6 Clearly marked target placement reticles [B1..B6] with concentric target rings
    for (let i = 0; i < ZONE_B_CONFIG.slots.length; i++) {
      const slot = ZONE_B_CONFIG.slots[i];
      // Target reticle outer ring
      const outerRing = new THREE.Mesh(
        new THREE.RingGeometry(0.024, 0.030, 28).rotateX(-Math.PI / 2),
        new THREE.MeshBasicMaterial({ color: 0x34d399, side: THREE.DoubleSide })
      );
      outerRing.position.set(slot[0], padH + 0.001, slot[2]);
      this.root.add(outerRing);

      // Inner target bullseye dot
      const innerDot = new THREE.Mesh(
        new THREE.CircleGeometry(0.007, 18).rotateX(-Math.PI / 2),
        new THREE.MeshBasicMaterial({ color: 0x10b981, side: THREE.DoubleSide })
      );
      innerDot.position.set(slot[0], padH + 0.0012, slot[2]);
      this.root.add(innerDot);

      // Crosshair alignment ticks
      const tickH = new THREE.Mesh(
        new THREE.PlaneGeometry(0.036, 0.002).rotateX(-Math.PI / 2),
        new THREE.MeshBasicMaterial({ color: 0x6ee7b7, side: THREE.DoubleSide })
      );
      tickH.position.set(slot[0], padH + 0.0011, slot[2]);
      this.root.add(tickH);

      const tickV = new THREE.Mesh(
        new THREE.PlaneGeometry(0.002, 0.036).rotateX(-Math.PI / 2),
        new THREE.MeshBasicMaterial({ color: 0x6ee7b7, side: THREE.DoubleSide })
      );
      tickV.position.set(slot[0], padH + 0.0011, slot[2]);
      this.root.add(tickV);
    }

    // Register physical Rapier body for Zone B
    const zoneBBody = this.bodyManager.createBody({
      id: 'zone_b_target_platform',
      name: 'ZONE B — TARGET Platform',
      category: 'environment',
      motionType: 'fixed',
      shape: 'box',
      dimensions: { width: padW, height: padH, depth: padD },
      position: [ZONE_B_CONFIG.center[0], padH / 2, ZONE_B_CONFIG.center[2]],
      massKg: 15,
      friction: 0.75,
      restitution: 0.05
    });
    // Alias for backwards compatibility with tests expecting target_surgical_tray
    this.bodyManager.bodies.set('target_surgical_tray', zoneBBody);

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
