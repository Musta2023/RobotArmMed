/**
 * @file Gripper.ts
 * DofBot-inspired Mechanical Geared Two-Finger Gripper with Rapier3D finger
 * RigidBodies, motorized opening/closing, normal squeezing force control,
 * and 4-bar linkage 3D visualization.
 */

import * as THREE from 'three';
import {
  GripperConfig,
  DEFAULT_ROBOT_CONFIGURATION
} from './RobotConfiguration.js';
import {
  RigidBodyManager,
  ManagedRigidBody
} from '../physics/RigidBodyManager.js';

export class Gripper {
  public config: GripperConfig;
  public bodyManager: RigidBodyManager;

  public root: THREE.Group;
  public leftFingerGroup: THREE.Group;
  public rightFingerGroup: THREE.Group;
  public leftGearMesh!: THREE.Mesh;
  public rightGearMesh!: THREE.Mesh;
  public leftDriveBarGroup!: THREE.Group;
  public rightDriveBarGroup!: THREE.Group;
  public leftFollowerBarGroup!: THREE.Group;
  public rightFollowerBarGroup!: THREE.Group;
  public graspFrame: THREE.Group;

  public leftFingerBody!: ManagedRigidBody;
  public rightFingerBody!: ManagedRigidBody;

  public currentOpeningMeters: number;
  public targetOpeningMeters: number;
  public grippingForceNewtons: number;
  public appliedNormalForceNewtons: number = 0;
  public isClosingCommandActive: boolean = false;

  constructor(
    bodyManager: RigidBodyManager,
    config: GripperConfig = DEFAULT_ROBOT_CONFIGURATION.gripper
  ) {
    this.config = { ...config };
    this.bodyManager = bodyManager;

    this.currentOpeningMeters = this.config.defaultOpeningMeters;
    this.targetOpeningMeters = this.config.defaultOpeningMeters;
    this.grippingForceNewtons = this.config.defaultGrippingForceNewtons;

    this.root = new THREE.Group();
    this.root.name = 'GripperAssembly';

    this.leftFingerGroup = new THREE.Group();
    this.leftFingerGroup.name = 'LeftFinger';

    this.rightFingerGroup = new THREE.Group();
    this.rightFingerGroup.name = 'RightFinger';

    this.graspFrame = new THREE.Group();
    this.graspFrame.name = 'GraspFrame';
    this.graspFrame.position.set(
      this.config.graspFrameOffset[0],
      this.config.graspFrameOffset[1],
      this.config.graspFrameOffset[2]
    );

    this._buildVisualGeometry();
    this._createFingerRigidBodies();
    this._applyFingerPositions(this.currentOpeningMeters);
  }

  /**
   * Builds the DofBot-inspired dark grey 4-bar linkage mechanical gripper with
   * twin synchronous gears, brass standoffs, servo housing, and inner rubber pads.
   */
  private _buildVisualGeometry(): void {
    const darkAnodizedMat = new THREE.MeshStandardMaterial({
      color: 0x262930,
      metalness: 0.68,
      roughness: 0.34
    });
    const linkageGreyMat = new THREE.MeshStandardMaterial({
      color: 0x4b5563,
      metalness: 0.62,
      roughness: 0.36
    });
    const brassStandoffMat = new THREE.MeshStandardMaterial({
      color: 0xeab308,
      metalness: 0.85,
      roughness: 0.22
    });
    const silverScrewMat = new THREE.MeshStandardMaterial({
      color: 0xe2e8f0,
      metalness: 0.88,
      roughness: 0.18
    });
    const rubberPadMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      metalness: 0.08,
      roughness: 0.92
    });

    // =========================================================================
    // J6 Rotating Tool Flange & Mechanical Wrist Interface (Zero Gap Coupling)
    // =========================================================================
    // 1. Circular CNC silver tool flange / rotating servo horn mating flush with Link 4 distal collar
    const j6FlangeGeo = new THREE.CylinderGeometry(0.016, 0.016, 0.004, 28);
    const j6Flange = new THREE.Mesh(j6FlangeGeo, silverScrewMat);
    j6Flange.position.set(0, 0.002, 0);
    j6Flange.castShadow = true;
    this.root.add(j6Flange);

    // Central drive spindle socket receiving the J6 motor shaft from Link 4
    const j6SocketGeo = new THREE.CylinderGeometry(0.007, 0.007, 0.006, 20);
    const j6Socket = new THREE.Mesh(j6SocketGeo, darkAnodizedMat);
    j6Socket.position.set(0, 0.003, 0);
    this.root.add(j6Socket);

    // Perimeter retaining hex bolts on J6 tool flange (6 bolts)
    const flangeBoltGeo = new THREE.CylinderGeometry(0.0016, 0.0016, 0.004, 10);
    for (let k = 0; k < 6; k++) {
      const angle = (k * Math.PI) / 3;
      const bx = Math.cos(angle) * 0.012;
      const bz = Math.sin(angle) * 0.012;
      const bolt = new THREE.Mesh(flangeBoltGeo, silverScrewMat);
      bolt.position.set(bx, 0.004, bz);
      this.root.add(bolt);
    }

    // Heavy-duty structural adapter base block (rigidly attaches rotating horn to gripper chassis)
    const adapterBlockGeo = new THREE.BoxGeometry(0.052, 0.018, 0.038);
    const adapterBlock = new THREE.Mesh(adapterBlockGeo, darkAnodizedMat);
    adapterBlock.position.set(0, 0.011, -0.004);
    adapterBlock.castShadow = true;
    adapterBlock.receiveShadow = true;
    this.root.add(adapterBlock);

    // Twin lateral mounting brackets / gussets bridging adapter base to chassis
    const gussetGeo = new THREE.BoxGeometry(0.005, 0.026, 0.036);
    const gussetScrewGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.007, 10).rotateZ(Math.PI / 2);
    for (const gx of [-0.026, 0.026]) {
      const gusset = new THREE.Mesh(gussetGeo, linkageGreyMat);
      gusset.position.set(gx, 0.015, -0.004);
      gusset.castShadow = true;
      this.root.add(gusset);

      for (const gy of [0.006, 0.018]) {
        const gussetScrew = new THREE.Mesh(gussetScrewGeo, silverScrewMat);
        gussetScrew.position.set(gx * 1.05, gy, -0.004);
        this.root.add(gussetScrew);
      }
    }

    // =========================================================================
    // 2. Gripper Actuator Servo Box & Realistic Mounting System
    // =========================================================================
    const servoBox = new THREE.Mesh(
      new THREE.BoxGeometry(0.044, 0.042, 0.038),
      darkAnodizedMat
    );
    servoBox.position.set(0, 0.025, -0.008);
    servoBox.castShadow = true;
    servoBox.receiveShadow = true;
    this.root.add(servoBox);

    // Servo casing mounting tabs / ears clamping it to the chassis plates and base
    const servoEarGeo = new THREE.BoxGeometry(0.054, 0.004, 0.034);
    const earTop = new THREE.Mesh(servoEarGeo, darkAnodizedMat);
    earTop.position.set(0, 0.042, -0.008);
    earTop.castShadow = true;
    this.root.add(earTop);

    const earBottom = new THREE.Mesh(servoEarGeo, darkAnodizedMat);
    earBottom.position.set(0, 0.008, -0.008);
    earBottom.castShadow = true;
    this.root.add(earBottom);

    // 4 Silver M2.5 fastening screws clamping servo ears to the mounting gussets
    const servoClampScrewGeo = new THREE.CylinderGeometry(0.0018, 0.0018, 0.006, 10).rotateZ(Math.PI / 2);
    for (const ex of [-0.024, 0.024]) {
      for (const ey of [0.008, 0.042]) {
        const clampScrew = new THREE.Mesh(servoClampScrewGeo, silverScrewMat);
        clampScrew.position.set(ex * 1.06, ey, -0.008);
        this.root.add(clampScrew);
      }
    }

    // Drive pinion boss coupling servo output spline to the driving gear
    const pinionBossGeo = new THREE.CylinderGeometry(0.005, 0.005, 0.012, 14).rotateX(Math.PI / 2);
    const pinionBoss = new THREE.Mesh(pinionBossGeo, silverScrewMat);
    pinionBoss.position.set(-0.015, 0.046, 0.008);
    pinionBoss.castShadow = true;
    this.root.add(pinionBoss);

    // =========================================================================
    // 3. Continuous Linkage Chassis Plates (Zero Gap, y: 0.008 -> 0.056 m)
    // =========================================================================
    const plateGeo = new THREE.BoxGeometry(0.082, 0.048, 0.004);
    const frontPlate = new THREE.Mesh(plateGeo, linkageGreyMat);
    frontPlate.position.set(0, 0.032, 0.016);
    frontPlate.castShadow = true;
    this.root.add(frontPlate);

    const backPlate = new THREE.Mesh(plateGeo, linkageGreyMat);
    backPlate.position.set(0, 0.032, -0.016);
    backPlate.castShadow = true;
    this.root.add(backPlate);

    // Silver socket-cap screws bolting front and back plates directly into adapter base block
    const plateScrewGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.008, 10).rotateX(Math.PI / 2);
    for (const px of [-0.022, 0.022]) {
      const frontScrew = new THREE.Mesh(plateScrewGeo, silverScrewMat);
      frontScrew.position.set(px, 0.012, 0.019);
      this.root.add(frontScrew);

      const backScrew = new THREE.Mesh(plateScrewGeo, silverScrewMat);
      backScrew.position.set(px, 0.012, -0.019);
      this.root.add(backScrew);
    }

    // Brass standoffs between gripper plates
    const standoffGeo = new THREE.CylinderGeometry(0.003, 0.003, 0.032, 10);
    standoffGeo.rotateX(Math.PI / 2);
    for (const sx of [-0.032, 0.032]) {
      for (const sy of [0.022, 0.048]) {
        const pillar = new THREE.Mesh(standoffGeo, brassStandoffMat);
        pillar.position.set(sx, sy, 0);
        this.root.add(pillar);
      }
    }

    // 3. Twin Synchronous Spur Gears at base of linkage (y = 0.050 m)
    const gearGeo = new THREE.CylinderGeometry(0.014, 0.014, 0.008, 18);
    gearGeo.rotateX(Math.PI / 2);
    this.leftGearMesh = new THREE.Mesh(gearGeo, darkAnodizedMat);
    this.leftGearMesh.position.set(-0.015, 0.050, 0.018);
    this.root.add(this.leftGearMesh);

    this.rightGearMesh = new THREE.Mesh(gearGeo, darkAnodizedMat);
    this.rightGearMesh.position.set(0.015, 0.050, 0.018);
    this.root.add(this.rightGearMesh);

    // Silver pivot screws on gears
    const screwGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.012, 12);
    screwGeo.rotateX(Math.PI / 2);
    const leftScrew = new THREE.Mesh(screwGeo, silverScrewMat);
    leftScrew.position.set(-0.015, 0.050, 0.019);
    this.root.add(leftScrew);

    const rightScrew = new THREE.Mesh(screwGeo, silverScrewMat);
    rightScrew.position.set(0.015, 0.050, 0.019);
    this.root.add(rightScrew);

    // 3b. 4-Bar Parallel Linkage Drive and Follower Bars (left & right)
    const barLength = 0.042;
    const barThick = 0.005;
    const barWidth = 0.008;

    const buildLinkageBar = (length: number, mat: THREE.Material): THREE.Group => {
      const g = new THREE.Group();
      const arm = new THREE.Mesh(
        new THREE.BoxGeometry(barWidth, length, barThick),
        mat
      );
      arm.position.set(0, length / 2, 0);
      arm.castShadow = true;
      g.add(arm);
      const endPin = new THREE.Mesh(
        new THREE.CylinderGeometry(0.003, 0.003, barThick + 0.004, 10).rotateX(Math.PI / 2),
        silverScrewMat
      );
      endPin.position.set(0, length, 0);
      g.add(endPin);
      return g;
    };

    this.leftDriveBarGroup = buildLinkageBar(barLength, darkAnodizedMat);
    this.leftDriveBarGroup.position.set(-0.015, 0.050, 0.008);
    this.root.add(this.leftDriveBarGroup);

    this.rightDriveBarGroup = buildLinkageBar(barLength, darkAnodizedMat);
    this.rightDriveBarGroup.position.set(0.015, 0.050, 0.008);
    this.root.add(this.rightDriveBarGroup);

    this.leftFollowerBarGroup = buildLinkageBar(barLength, linkageGreyMat);
    this.leftFollowerBarGroup.position.set(-0.028, 0.038, -0.008);
    this.root.add(this.leftFollowerBarGroup);

    this.rightFollowerBarGroup = buildLinkageBar(barLength, linkageGreyMat);
    this.rightFollowerBarGroup.position.set(0.028, 0.038, -0.008);
    this.root.add(this.rightFollowerBarGroup);

    // 4. Left & Right Movable Finger Assemblies (y: 0.060 -> 0.132 m)
    const [fThick, fLen, fDepth] = this.config.fingerDimensions;
    const padThick = 0.003;

    const buildFingerSide = (sideSign: -1 | 1, group: THREE.Group) => {
      // Main long dark grey jaw bar
      const jawMesh = new THREE.Mesh(
        new THREE.BoxGeometry(fThick, fLen, fDepth),
         linkageGreyMat
      );
      jawMesh.position.set(
        sideSign * (padThick + fThick / 2),
        0.060 + fLen / 2,
        0
      );
      jawMesh.castShadow = true;
      jawMesh.receiveShadow = true;
      group.add(jawMesh);

      // Parallel linkage strut
      const strutMesh = new THREE.Mesh(
        new THREE.BoxGeometry(0.007, 0.042, 0.018),
        darkAnodizedMat
      );
      strutMesh.position.set(sideSign * (padThick + fThick + 0.004), 0.072, 0);
      group.add(strutMesh);

      // Pivot pins on finger linkage
      for (const py of [0.066, 0.092, 0.118]) {
        const pin = new THREE.Mesh(
          new THREE.CylinderGeometry(0.0025, 0.0025, fDepth + 0.004, 10).rotateX(Math.PI / 2),
          silverScrewMat
        );
        pin.position.set(sideSign * (padThick + fThick / 2), py, 0);
        group.add(pin);
      }

      // High-friction inner elastomer contact pad (inner surface at local x = 0)
      const padMesh = new THREE.Mesh(
        new THREE.BoxGeometry(padThick, 0.044, fDepth * 0.92),
        rubberPadMat
      );
      padMesh.position.set(
        sideSign * (padThick / 2),
        this.config.graspFrameOffset[1],
        0
      );
      group.add(padMesh);
    };

    buildFingerSide(-1, this.leftFingerGroup);
    buildFingerSide(1, this.rightFingerGroup);

    const tcpAxes = new THREE.AxesHelper(0.032);
    this.graspFrame.add(tcpAxes);

    this.root.add(this.leftFingerGroup);
    this.root.add(this.rightFingerGroup);
    this.root.add(this.graspFrame);
  }

  private _createFingerRigidBodies(): void {
    const [fThick, fLen, fDepth] = this.config.fingerDimensions;

    this.leftFingerBody = this.bodyManager.createBody({
      id: 'gripper_left_finger',
      name: 'Gripper Left Finger',
      category: 'gripper_finger',
      motionType: 'kinematicPosition',
      shape: 'box',
      dimensions: { width: fThick, height: fLen, depth: fDepth },
      position: [-0.04, 0.3, 0],
      massKg: this.config.fingerMassKg,
      friction: this.config.fingerFriction,
      restitution: this.config.fingerRestitution,
      materialType: 'rubber'
    });

    this.rightFingerBody = this.bodyManager.createBody({
      id: 'gripper_right_finger',
      name: 'Gripper Right Finger',
      category: 'gripper_finger',
      motionType: 'kinematicPosition',
      shape: 'box',
      dimensions: { width: fThick, height: fLen, depth: fDepth },
      position: [0.04, 0.3, 0],
      massKg: this.config.fingerMassKg,
      friction: this.config.fingerFriction,
      restitution: this.config.fingerRestitution,
      materialType: 'rubber'
    });
  }

  private _applyFingerPositions(openingMeters: number): void {
    const clamped = Math.min(
      Math.max(openingMeters, this.config.minOpeningMeters),
      this.config.maxOpeningMeters
    );
    this.currentOpeningMeters = clamped;
    const halfGap = clamped / 2;

    this.leftFingerGroup.position.set(-halfGap, 0, 0);
    this.rightFingerGroup.position.set(halfGap, 0, 0);

    // Rotate synchronous gears proportionally to finger opening
    const gearAngle = (clamped - this.config.minOpeningMeters) * 12;
    // Closed-loop 4-bar linkage kinematics:
    const strokeRange = this.config.maxOpeningMeters - this.config.minOpeningMeters;
    const strokeRatio = strokeRange > 0 ? (clamped - this.config.minOpeningMeters) / strokeRange : 0;
    const crankAngleRad = strokeRatio * 0.52;

    if (this.leftDriveBarGroup && this.rightDriveBarGroup) {
      this.leftDriveBarGroup.rotation.z = crankAngleRad;
      this.rightDriveBarGroup.rotation.z = -crankAngleRad;
    }
    if (this.leftFollowerBarGroup && this.rightFollowerBarGroup) {
      this.leftFollowerBarGroup.rotation.z = crankAngleRad;
      this.rightFollowerBarGroup.rotation.z = -crankAngleRad;
    }
    if (this.leftGearMesh && this.rightGearMesh) {
      this.leftGearMesh.rotation.z = crankAngleRad;
      this.rightGearMesh.rotation.z = -crankAngleRad;
    }

    this.root.updateMatrixWorld(true);
  }

  /**
   * Commands the motorized fingers to open to maximum/default opening.
   */
  public open(immediate: boolean = false): void {
    this.isClosingCommandActive = false;
    this.targetOpeningMeters = this.config.defaultOpeningMeters;
    this.appliedNormalForceNewtons = 0;
    if (immediate) {
      this._applyFingerPositions(this.targetOpeningMeters);
    }
  }

  public openGripper(immediate: boolean = false): void {
    this.open(immediate);
  }

  /**
   * Commands the motorized fingers to close.
   * Does NOT visually attach any object; grasping occurs only when physical contact
   * and friction stability conditions are satisfied in `GraspManager`.
   */
  public close(immediate: boolean = false): void {
    this.isClosingCommandActive = true;
    this.targetOpeningMeters = this.config.minOpeningMeters;
    if (immediate) {
      this._applyFingerPositions(this.targetOpeningMeters);
    }
  }

  public closeGripper(immediate: boolean = false): void {
    this.close(immediate);
  }

  /**
   * Sets a specific target gripper opening distance in meters.
   */
  public setTargetOpeningMeters(widthMeters: number, immediate: boolean = false): number {
    const clamped = Math.min(
      Math.max(widthMeters, this.config.minOpeningMeters),
      this.config.maxOpeningMeters
    );
    this.targetOpeningMeters = clamped;
    this.isClosingCommandActive = clamped < this.currentOpeningMeters;
    if (immediate) {
      this._applyFingerPositions(clamped);
    }
    return clamped;
  }

  /**
   * Sets the maximum gripping squeeze force in Newtons.
   */
  public setGrippingForceNewtons(forceN: number): void {
    this.grippingForceNewtons = Math.min(
      Math.max(forceN, 0),
      this.config.maxGrippingForceNewtons
    );
  }

  /**
   * Steps the motorized finger actuators over `dt` seconds and synchronizes
   * the left and right finger Rapier3D RigidBodies in world space.
   *
   * @param dt Fixed physics timestep in seconds
   * @param minPhysicalStopWidthMeters Optional physical stop width imposed by an object between the jaws
   */
  public stepMotorAndSyncBodies(
    dt: number,
    minPhysicalStopWidthMeters?: number
  ): void {
    const effectiveTarget =
      minPhysicalStopWidthMeters !== undefined
        ? Math.max(this.targetOpeningMeters, minPhysicalStopWidthMeters)
        : this.targetOpeningMeters;

    const diff = effectiveTarget - this.currentOpeningMeters;
    const maxStep = this.config.motorSpeedMetersPerSec * dt;

    if (Math.abs(diff) <= maxStep) {
      this._applyFingerPositions(effectiveTarget);
    } else {
      this._applyFingerPositions(
        this.currentOpeningMeters + Math.sign(diff) * maxStep
      );
    }

    // If finger closure is physically stopped by an object wider than targetOpeningMeters,
    // the finger motor applies normal squeezing force up to `grippingForceNewtons`.
    if (
      minPhysicalStopWidthMeters !== undefined &&
      this.targetOpeningMeters < minPhysicalStopWidthMeters - 0.0005 &&
      Math.abs(this.currentOpeningMeters - minPhysicalStopWidthMeters) <= 0.0105
    ) {
      this.appliedNormalForceNewtons = this.grippingForceNewtons;
    } else {
      this.appliedNormalForceNewtons = 0;
    }

    // Sync Rapier3D RigidBodies for LeftFinger and RightFinger to their Three.js world transforms
    this.syncFingerBodies(false);
  }

  /**
   * Synchronizes LeftFinger and RightFinger Rapier3D RigidBodies to their current Three.js world transforms.
   */
  public syncFingerBodies(immediate: boolean = false): void {
    this.root.updateMatrixWorld(true);
    const [fThick] = this.config.fingerDimensions;
    const offsetLocalY = this.config.graspFrameOffset[1];

    const syncFingerBody = (
      fingerGroup: THREE.Group,
      managedBody: ManagedRigidBody,
      sideSign: -1 | 1
    ) => {
      const localCenter = new THREE.Vector3(
        sideSign * (0.003 + fThick / 2),
        offsetLocalY,
        0
      );
      const worldPos = localCenter.applyMatrix4(fingerGroup.matrixWorld);
      const worldQuat = new THREE.Quaternion();
      fingerGroup.getWorldQuaternion(worldQuat);

      managedBody.kinematicTargetPos = [worldPos.x, worldPos.y, worldPos.z];
      managedBody.kinematicTargetQuat = [
        worldQuat.x,
        worldQuat.y,
        worldQuat.z,
        worldQuat.w
      ];
      managedBody.body.setNextKinematicTranslation({
        x: worldPos.x,
        y: worldPos.y,
        z: worldPos.z
      });
      managedBody.body.setNextKinematicRotation({
        x: worldQuat.x,
        y: worldQuat.y,
        z: worldQuat.z,
        w: worldQuat.w
      });
      if (immediate) {
        managedBody.body.setTranslation(
          { x: worldPos.x, y: worldPos.y, z: worldPos.z },
          true
        );
        managedBody.body.setRotation(
          { x: worldQuat.x, y: worldQuat.y, z: worldQuat.z, w: worldQuat.w },
          true
        );
      }
    };

    syncFingerBody(this.leftFingerGroup, this.leftFingerBody, -1);
    syncFingerBody(this.rightFingerGroup, this.rightFingerBody, 1);
  }

  /**
   * Returns the world position and quaternion of the GraspFrame (TCP).
   */
  public getGraspFrameWorldPose(): {
    position: THREE.Vector3;
    quaternion: THREE.Quaternion;
    matrix: THREE.Matrix4;
  } {
    this.root.updateMatrixWorld(true);
    this.graspFrame.updateMatrixWorld(true);
    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    this.graspFrame.matrixWorld.decompose(position, quaternion, scale);
    return {
      position,
      quaternion,
      matrix: this.graspFrame.matrixWorld.clone()
    };
  }
}
