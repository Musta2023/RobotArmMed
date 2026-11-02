/**
 * @file RobotModel.ts
 * 3D Visual and Physical Digital Twin Model of the DofBot-inspired Green 6-DOF
 * Robotic Manipulator (with stacked green base disks, brass standoffs, controller PCB
 * with dual antennas, green anodized U-brackets, black servo actuators, wrist camera
 * assembly, and 4-bar mechanical gripper).
 */

import * as THREE from 'three';
import {
  RobotSystemConfiguration,
  DEFAULT_ROBOT_CONFIGURATION
} from './RobotConfiguration.js';
import { RobotJoints } from './RobotJoints.js';
import { Gripper } from './Gripper.js';
import {
  RobotState,
  RobotStateSnapshot,
  JointTelemetry
} from './RobotState.js';
import { RAD2DEG } from './RobotJoints.js';

export class RobotModel {
  public config: RobotSystemConfiguration;
  public robotJoints: RobotJoints;
  public gripper: Gripper;
  public state: RobotState;

  public root: THREE.Group;
  public jointPivots: THREE.Group[] = [];
  public linkGroups: THREE.Group[] = [];
  public wristCamera: THREE.PerspectiveCamera;
  public wristCameraHelper: THREE.CameraHelper;

  constructor(
    robotJoints: RobotJoints,
    gripper: Gripper,
    config: RobotSystemConfiguration = DEFAULT_ROBOT_CONFIGURATION
  ) {
    this.config = config;
    this.robotJoints = robotJoints;
    this.gripper = gripper;

    this.root = new THREE.Group();
    this.root.name = 'DofBotDigitalTwinRoot';
    this.root.position.set(
      config.basePosition[0],
      config.basePosition[1],
      config.basePosition[2]
    );

    const camCfg = config.wristCamera;
    this.wristCamera = new THREE.PerspectiveCamera(
      camCfg.fovDeg,
      camCfg.resolution[0] / camCfg.resolution[1],
      camCfg.near,
      camCfg.far
    );
    this.wristCamera.name = 'EndEffectorWristCamera';
    this.wristCameraHelper = new THREE.CameraHelper(this.wristCamera);
    this.wristCameraHelper.visible = false;

    this._buildDofBotGeometry();
    this.syncVisualHierarchyFromJoints(true);

    this.state = new RobotState(this.captureStateSnapshot(0, false, false, null, 0));
  }

  private _buildDofBotGeometry(): void {
    // Materials matching the DofBot reference image
    const greenBracketMat = new THREE.MeshStandardMaterial({
      color: 0x00c853,
      metalness: 0.48,
      roughness: 0.32
    });
    const darkServoMat = new THREE.MeshStandardMaterial({
      color: 0x181a20,
      metalness: 0.35,
      roughness: 0.55
    });
    const basePlateMat = new THREE.MeshStandardMaterial({
      color: 0x6b7280,
      metalness: 0.55,
      roughness: 0.42
    });
    const brassStandoffMat = new THREE.MeshStandardMaterial({
      color: 0xeab308,
      metalness: 0.85,
      roughness: 0.22
    });
    const silverHornMat = new THREE.MeshStandardMaterial({
      color: 0xe2e8f0,
      metalness: 0.85,
      roughness: 0.20
    });
    const whiteFootMat = new THREE.MeshStandardMaterial({
      color: 0xf8fafc,
      metalness: 0.1,
      roughness: 0.6
    });
    const pcbMat = new THREE.MeshStandardMaterial({
      color: 0x111827,
      metalness: 0.4,
      roughness: 0.5
    });

    // =========================================================================
    // A. DofBot Base Plate, 4 White Corner Feet, Stacked Green Disks & PCB
    // =========================================================================
    const basePlate = new THREE.Mesh(
      new THREE.BoxGeometry(0.21, 0.010, 0.25),
      basePlateMat
    );
    basePlate.position.set(0, 0.005, -0.025);
    basePlate.receiveShadow = true;
    basePlate.castShadow = true;
    this.root.add(basePlate);

    // 4 White cylindrical corner feet/bumpers on top/bottom of base plate
    const footGeo = new THREE.CylinderGeometry(0.012, 0.012, 0.014, 16);
    for (const [fx, fz] of [
      [-0.085, 0.075],
      [0.085, 0.075],
      [-0.085, -0.125],
      [0.085, -0.125]
    ]) {
      const foot = new THREE.Mesh(footGeo, whiteFootMat);
      foot.position.set(fx, 0.011, fz);
      this.root.add(foot);
    }

    // Rear Controller Expansion PCB + Heatsink + 2 Vertical Black Antennas
    const pcbMesh = new THREE.Mesh(
      new THREE.BoxGeometry(0.11, 0.028, 0.095),
      pcbMat
    );
    pcbMesh.position.set(0, 0.024, -0.085);
    pcbMesh.castShadow = true;
    this.root.add(pcbMesh);

    const antennaGeo = new THREE.CylinderGeometry(0.0035, 0.0045, 0.135, 12);
    for (const ax of [-0.028, 0.028]) {
      const antenna = new THREE.Mesh(antennaGeo, basePlateMat);
      antenna.position.set(ax, 0.095, -0.12);
      antenna.castShadow = true;
      this.root.add(antenna);
    }

    // Lower & Middle Stacked Green Circular Turret Disks with Brass Standoffs
    const diskGeo = new THREE.CylinderGeometry(0.068, 0.068, 0.007, 32);
    const lowerDisk = new THREE.Mesh(diskGeo, greenBracketMat);
    lowerDisk.position.set(0, 0.055, 0);
    lowerDisk.castShadow = true;
    this.root.add(lowerDisk);

    const middleDisk = new THREE.Mesh(diskGeo, greenBracketMat);
    middleDisk.position.set(0, 0.115, 0);
    middleDisk.castShadow = true;
    this.root.add(middleDisk);

    // 6 Brass hexagonal standoffs supporting the green base disks
    const standoffGeo = new THREE.CylinderGeometry(0.0035, 0.0035, 0.115, 6);
    for (let k = 0; k < 6; k++) {
      const angle = (k * Math.PI) / 3 + Math.PI / 6;
      const sx = Math.cos(angle) * 0.056;
      const sz = Math.sin(angle) * 0.056;
      const post = new THREE.Mesh(standoffGeo, brassStandoffMat);
      post.position.set(sx, 0.062, sz);
      this.root.add(post);
    }

    // Base center servo column
    const baseServo = new THREE.Mesh(
      new THREE.BoxGeometry(0.048, 0.095, 0.048),
      darkServoMat
    );
    baseServo.position.set(0, 0.062, 0);
    this.root.add(baseServo);

    // Stationary turntable bearing collar sleeve extending through the middle disk
    const bearingCollar = new THREE.Mesh(
      new THREE.CylinderGeometry(0.028, 0.032, 0.016, 24),
      silverHornMat
    );
    bearingCollar.position.set(0, 0.115, 0);
    bearingCollar.castShadow = true;
    this.root.add(bearingCollar);

    // =========================================================================
    // B. Articulated Kinematic Chain (J1..J6)
    // =========================================================================
    let parentGroup: THREE.Group = this.root;

    for (let i = 0; i < this.config.joints.length; i++) {
      const jCfg = this.config.joints[i];

      const pivotGroup = new THREE.Group();
      pivotGroup.name = `${jCfg.id}_Pivot`;
      pivotGroup.position.set(
        jCfg.pivotOffset[0],
        jCfg.pivotOffset[1],
        jCfg.pivotOffset[2]
      );

      const linkGroup = new THREE.Group();
      linkGroup.name = `${jCfg.id}_LinkGroup`;
      pivotGroup.add(linkGroup);
      parentGroup.add(pivotGroup);

      this.jointPivots.push(pivotGroup);
      this.linkGroups.push(linkGroup);
      parentGroup = linkGroup;
    }

    // Populate each link's DofBot green U-bracket & servo geometry
    this._buildLink0Turret(this.linkGroups[0], greenBracketMat, darkServoMat, silverHornMat);
    this._buildGreenUBracketLink(
      this.linkGroups[1],
      0.32,
      0.072,
      greenBracketMat,
      darkServoMat,
      silverHornMat
    );
    this._buildGreenUBracketLink(
      this.linkGroups[2],
      0.15,
      0.066,
      greenBracketMat,
      darkServoMat,
      silverHornMat
    );
    this._buildGreenUBracketLink(
      this.linkGroups[3],
      0.13,
      0.060,
      greenBracketMat,
      darkServoMat,
      silverHornMat
    );
    this._buildWristAndCameraLink(
      this.linkGroups[4],
      greenBracketMat,
      darkServoMat,
      silverHornMat,
      brassStandoffMat
    );

    // Attach Gripper to J6 link group
    this.linkGroups[5].add(this.gripper.root);
  }

  private _buildLink0Turret(
    group: THREE.Group,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material
  ): void {
    // Rotating top green circular plate at y = 0
    const topDisk = new THREE.Mesh(
      new THREE.CylinderGeometry(0.066, 0.066, 0.008, 32),
      greenMat
    );
    topDisk.position.set(0, -0.008, 0);
    topDisk.castShadow = true;
    group.add(topDisk);

    // Rotating turntable bearing hub ring (mechanically mates with base stator collar)
    const rotatingHub = new THREE.Mesh(
      new THREE.CylinderGeometry(0.038, 0.038, 0.010, 28),
      hornMat
    );
    rotatingHub.position.set(0, -0.017, 0);
    rotatingHub.castShadow = true;
    group.add(rotatingHub);

    const shaftCoupler = new THREE.Mesh(
      new THREE.CylinderGeometry(0.022, 0.022, 0.014, 20),
      servoMat
    );
    shaftCoupler.position.set(0, -0.017, 0);
    group.add(shaftCoupler);

    // Green shoulder U-bracket cradle + black digital servo
    const servoBody = new THREE.Mesh(
      new THREE.BoxGeometry(0.046, 0.088, 0.044),
      servoMat
    );
    servoBody.position.set(0, 0.055, 0);
    servoBody.castShadow = true;
    group.add(servoBody);

    for (const sx of [-0.032, 0.032]) {
      const sidePlate = new THREE.Mesh(
        new THREE.BoxGeometry(0.006, 0.115, 0.054),
        greenMat
      );
      sidePlate.position.set(sx, 0.060, 0);
      sidePlate.castShadow = true;
      group.add(sidePlate);

      const horn = new THREE.Mesh(
        new THREE.CylinderGeometry(0.018, 0.018, 0.008, 20).rotateZ(Math.PI / 2),
        hornMat
      );
      horn.position.set(sx * 1.08, 0.12, 0);
      group.add(horn);
    }
  }

  private _buildGreenUBracketLink(
    group: THREE.Group,
    lengthY: number,
    widthX: number,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material
  ): void {
    const halfX = widthX / 2;

    // Left and right green anodized aluminum side plates
    for (const sx of [-halfX, halfX]) {
      const sideWall = new THREE.Mesh(
        new THREE.BoxGeometry(0.006, lengthY, 0.048),
        greenMat
      );
      sideWall.position.set(sx, lengthY / 2, 0);
      sideWall.castShadow = true;
      sideWall.receiveShadow = true;
      group.add(sideWall);

      // Silver circular servo horn at base pivot
      const horn = new THREE.Mesh(
        new THREE.CylinderGeometry(0.017, 0.017, 0.008, 20).rotateZ(Math.PI / 2),
        hornMat
      );
      horn.position.set(sx * 1.06, 0, 0);
      group.add(horn);
    }

    // Green cross-bridge plates
    const bridgeGeo = new THREE.BoxGeometry(widthX, 0.018, 0.048);
    const bottomBridge = new THREE.Mesh(bridgeGeo, greenMat);
    bottomBridge.position.set(0, 0.025, 0);
    group.add(bottomBridge);

    const topBridge = new THREE.Mesh(bridgeGeo, greenMat);
    topBridge.position.set(0, lengthY - 0.025, 0);
    group.add(topBridge);

    // Internal black serial bus servo actuator
    const servo = new THREE.Mesh(
      new THREE.BoxGeometry(widthX - 0.014, Math.min(lengthY * 0.55, 0.075), 0.038),
      servoMat
    );
    servo.position.set(0, lengthY - 0.04, 0);
    servo.castShadow = true;
    group.add(servo);
  }

  /**
   * Builds the J5 Wrist link AND the top-mounted DofBot USB Camera bracket + lens
   * shown in the reference image.
   */
  private _buildWristAndCameraLink(
    group: THREE.Group,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material,
    brassMat: THREE.Material
  ): void {
    this._buildGreenUBracketLink(group, 0.08, 0.058, greenMat, servoMat, hornMat);

    // Camera Mounting Bracket on the front/top face (+Z) of the wrist link
    const camMountGroup = new THREE.Group();
    camMountGroup.name = 'WristCameraAssembly';
    const [mx, my, mz] = this.config.wristCamera.mountOffset;
    camMountGroup.position.set(mx, my, mz);

    // Black camera PCB housing plate
    const camPlate = new THREE.Mesh(
      new THREE.BoxGeometry(0.048, 0.052, 0.008),
      servoMat
    );
    camPlate.castShadow = true;
    camMountGroup.add(camPlate);

    // 4 Yellow/Brass corner standoffs on camera bracket (just like reference image)
    const standoffGeo = new THREE.CylinderGeometry(0.0025, 0.0025, 0.014, 8).rotateX(Math.PI / 2);
    for (const cx of [-0.018, 0.018]) {
      for (const cy of [-0.018, 0.018]) {
        const post = new THREE.Mesh(standoffGeo, brassMat);
        post.position.set(cx, cy, -0.006);
        camMountGroup.add(post);
      }
    }

    // Silver cylindrical wide-angle camera barrel + dark glass lens
    const barrelMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.014, 0.015, 0.024, 24),
      hornMat
    );
    barrelMesh.position.set(0, 0.012, 0.010);
    barrelMesh.castShadow = true;
    camMountGroup.add(barrelMesh);

    const glassMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.010, 0.010, 0.026, 20),
      servoMat
    );
    glassMesh.position.set(0, 0.012, 0.010);
    camMountGroup.add(glassMesh);

    // Attach the Three.js PerspectiveCamera looking forward along the tool (+Y) toward the gripper
    this.wristCamera.position.set(0, 0.026, 0.012);
    const [lx, ly, lz] = this.config.wristCamera.lookAtOffset;
    this.wristCamera.lookAt(lx, ly, lz);
    camMountGroup.add(this.wristCamera);

    group.add(camMountGroup);
  }

  /**
   * Updates the Three.js joint pivot quaternions from the physical joint angles
   * in `RobotJoints`, and synchronizes all link Rapier3D RigidBodies.
   */
  public syncVisualHierarchyFromJoints(immediate: boolean = false): {
    jointWorldPivots: THREE.Vector3[];
    jointWorldAxes: THREE.Vector3[];
    linkWorldCOMs: THREE.Vector3[];
    tcpWorldPos: THREE.Vector3;
    tcpWorldQuat: THREE.Quaternion;
  } {
    const anglesRad = this.robotJoints.getCurrentAnglesRad();

    for (let i = 0; i < this.jointPivots.length; i++) {
      const jCfg = this.config.joints[i];
      const pivot = this.jointPivots[i];
      const localAxis = new THREE.Vector3(
        jCfg.axis[0],
        jCfg.axis[1],
        jCfg.axis[2]
      ).normalize();
      pivot.quaternion.setFromAxisAngle(localAxis, anglesRad[i] ?? 0);
    }

    this.root.updateMatrixWorld(true);

    const jointWorldPivots: THREE.Vector3[] = [];
    const jointWorldAxes: THREE.Vector3[] = [];
    const linkWorldCOMs: THREE.Vector3[] = [];
    const linkWorldQuats: THREE.Quaternion[] = [];

    for (let i = 0; i < this.jointPivots.length; i++) {
      const jCfg = this.config.joints[i];
      const pivot = this.jointPivots[i];

      const pPos = new THREE.Vector3();
      const pQuat = new THREE.Quaternion();
      pivot.matrixWorld.decompose(pPos, pQuat, new THREE.Vector3());

      const wAxis = new THREE.Vector3(jCfg.axis[0], jCfg.axis[1], jCfg.axis[2])
        .normalize()
        .applyQuaternion(pQuat)
        .normalize();

      const comLocal = new THREE.Vector3(
        jCfg.centerOfMassLocal[0],
        jCfg.centerOfMassLocal[1],
        jCfg.centerOfMassLocal[2]
      );
      const comWorld = comLocal.applyMatrix4(pivot.matrixWorld);

      jointWorldPivots.push(pPos);
      jointWorldAxes.push(wAxis);
      linkWorldCOMs.push(comWorld);
      linkWorldQuats.push(pQuat);
    }

    // Sync link & finger Rapier RigidBodies to world COM & quaternion
    this.robotJoints.syncLinkBodiesToTransforms(linkWorldCOMs, linkWorldQuats, immediate);
    this.gripper.syncFingerBodies(immediate);

    const tcpPose = this.gripper.getGraspFrameWorldPose();
    if (this.wristCameraHelper.visible) {
      this.wristCameraHelper.update();
    }

    return {
      jointWorldPivots,
      jointWorldAxes,
      linkWorldCOMs,
      tcpWorldPos: tcpPose.position,
      tcpWorldQuat: tcpPose.quaternion
    };
  }

  /**
   * Returns the current end-effector (TCP) world position `[x, y, z]` in meters.
   */
  public getEndEffectorPosition(): [number, number, number] {
    const tcp = this.gripper.getGraspFrameWorldPose();
    return [tcp.position.x, tcp.position.y, tcp.position.z];
  }

  /**
   * Captures a full `RobotStateSnapshot` of joints, velocities, torques, TCP pose, and gripper state.
   */
  public captureStateSnapshot(
    timestamp: number,
    isGrasping: boolean,
    isSlipping: boolean,
    graspedObjectId: string | null,
    payloadMassKg: number,
    leftFingerContact: boolean = false,
    rightFingerContact: boolean = false,
    frictionCapacityNewtons: number = 0
  ): RobotStateSnapshot {
    const tcpPose = this.gripper.getGraspFrameWorldPose();
    const euler = new THREE.Euler().setFromQuaternion(tcpPose.quaternion, 'XYZ');

    const jointsTelemetry: JointTelemetry[] = this.robotJoints.joints.map((mj, idx) => {
      const cfg = this.config.joints[idx];
      return {
        id: cfg.id,
        name: cfg.name,
        index: idx,
        currentAngleDeg: mj.currentAngleRad * RAD2DEG,
        currentAngleRad: mj.currentAngleRad,
        targetAngleDeg: mj.targetAngleRad * RAD2DEG,
        targetAngleRad: mj.targetAngleRad,
        angularVelocityRadPerSec: mj.angularVelocityRadPerSec,
        motorTorqueNm: mj.motorTorqueNm,
        gravityLoadTorqueNm: mj.gravityLoadTorqueNm,
        minAngleDeg: cfg.minAngleDeg,
        maxAngleDeg: cfg.maxAngleDeg,
        maxMotorTorqueNm: cfg.maxMotorTorqueNm * this.robotJoints.jointManager.globalMaxTorqueScale,
        isAtLimit: mj.isAtLimit,
        isTorqueSaturated: mj.isTorqueSaturated
      };
    });

    const snapshot: RobotStateSnapshot = {
      timestamp,
      joints: jointsTelemetry,
      endEffector: {
        position: [tcpPose.position.x, tcpPose.position.y, tcpPose.position.z],
        quaternion: [
          tcpPose.quaternion.x,
          tcpPose.quaternion.y,
          tcpPose.quaternion.z,
          tcpPose.quaternion.w
        ],
        eulerDeg: [
          euler.x * RAD2DEG,
          euler.y * RAD2DEG,
          euler.z * RAD2DEG
        ]
      },
      gripper: {
        openingMeters: this.gripper.currentOpeningMeters,
        openingMillimeters: this.gripper.currentOpeningMeters * 1000,
        targetOpeningMeters: this.gripper.targetOpeningMeters,
        grippingForceNewtons: this.gripper.grippingForceNewtons,
        appliedNormalForceNewtons: this.gripper.appliedNormalForceNewtons,
        frictionCapacityNewtons,
        isGrasping,
        isSlipping,
        graspedObjectId,
        leftFingerContact,
        rightFingerContact
      },
      totalPayloadMassKg: payloadMassKg,
      anyJointAtLimit: jointsTelemetry.some((j) => j.isAtLimit),
      anyTorqueExceeded: jointsTelemetry.some((j) => j.isTorqueSaturated)
    };

    if (this.state) {
      this.state.update(snapshot);
    }
    return snapshot;
  }
}
