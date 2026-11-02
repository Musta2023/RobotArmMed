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

  private _createChannelPlateGeometry(
    lengthY: number,
    depthZ: number,
    thicknessX: number,
    hasBottomEar: boolean = true,
    hasTopEar: boolean = true,
    cutoutCount: number = 2
  ): THREE.BufferGeometry {
    const shape = new THREE.Shape();
    const halfD = depthZ / 2;
    const radius = halfD;

    shape.moveTo(halfD, 0);
    if (hasBottomEar) {
      shape.absarc(0, 0, radius, 0, -Math.PI, true);
    } else {
      shape.lineTo(-halfD, 0);
    }
    shape.lineTo(-halfD, lengthY);
    if (hasTopEar) {
      shape.absarc(0, lengthY, radius, Math.PI, 0, true);
    } else {
      shape.lineTo(halfD, lengthY);
    }
    shape.lineTo(halfD, 0);

    if (cutoutCount > 0 && lengthY > 0.09) {
      const usableH = lengthY - 2 * radius;
      if (usableH > 0.02) {
        const windowH = Math.min(0.045, usableH / (cutoutCount + 0.6));
        const windowW = depthZ * 0.48;
        const spacing = usableH / cutoutCount;
        for (let k = 0; k < cutoutCount; k++) {
          const cy = radius + spacing * (k + 0.5);
          const hole = new THREE.Path();
          const hw = windowW / 2;
          const hh = windowH / 2;
          hole.moveTo(-hw, cy - hh);
          hole.lineTo(hw, cy - hh);
          hole.lineTo(hw, cy + hh);
          hole.lineTo(-hw, cy + hh);
          hole.closePath();
          shape.holes.push(hole);
        }
      }
    }

    const geo = new THREE.ExtrudeGeometry(shape, {
      steps: 1,
      depth: thicknessX,
      bevelEnabled: false
    });
    geo.rotateY(Math.PI / 2);
    geo.translate(-thicknessX / 2, 0, 0);
    geo.computeVertexNormals();
    return geo;
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

    const plateGeo = this._createChannelPlateGeometry(0.120, 0.048, 0.004, false, true, 1);
    const screwGeo = new THREE.CylinderGeometry(0.0035, 0.0035, 0.010, 12).rotateZ(Math.PI / 2);

    for (const sx of [-0.031, 0.031]) {
      const sidePlate = new THREE.Mesh(plateGeo, greenMat);
      sidePlate.position.set(sx, 0, 0);
      sidePlate.castShadow = true;
      group.add(sidePlate);

      // Bearing & horn flange at top ear (y = 0.120), concentric with J2 pivot!
      const horn = new THREE.Mesh(
        new THREE.CylinderGeometry(0.015, 0.015, 0.004, 20).rotateZ(Math.PI / 2),
        hornMat
      );
      horn.position.set(sx * 1.05, 0.120, 0);
      horn.castShadow = true;
      group.add(horn);

      const centerPin = new THREE.Mesh(screwGeo, servoMat);
      centerPin.position.set(sx * 1.10, 0.120, 0);
      group.add(centerPin);
    }
  }

  private _buildGreenUBracketLink(
    group: THREE.Group,
    lengthY: number,
    widthX: number,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material,
    hasBottomEar: boolean = true,
    hasTopEar: boolean = true,
    cutoutCount: number = 2
  ): void {
    const halfX = widthX / 2;
    const plateThickness = 0.004;
    const depthZ = 0.048;
    const plateGeo = this._createChannelPlateGeometry(
      lengthY,
      depthZ,
      plateThickness,
      hasBottomEar,
      hasTopEar,
      cutoutCount
    );

    const screwGeo = new THREE.CylinderGeometry(0.0035, 0.0035, 0.008, 12).rotateZ(Math.PI / 2);

    // Left and right profiled aluminum side plates with concentric pivot ears
    for (const sx of [-halfX, halfX]) {
      const sideWall = new THREE.Mesh(plateGeo, greenMat);
      sideWall.position.set(sx, 0, 0);
      sideWall.castShadow = true;
      sideWall.receiveShadow = true;
      group.add(sideWall);

      // Concentric silver pivot bearing cap & retaining pin at bottom pivot (y = 0)
      if (hasBottomEar) {
        const horn = new THREE.Mesh(
          new THREE.CylinderGeometry(0.015, 0.015, 0.003, 20).rotateZ(Math.PI / 2),
          hornMat
        );
        horn.position.set(sx * 1.04, 0, 0);
        horn.castShadow = true;
        group.add(horn);

        const centerPin = new THREE.Mesh(screwGeo, servoMat);
        centerPin.position.set(sx * 1.08, 0, 0);
        group.add(centerPin);
      }
    }

    // Green cross-bridge stiffener plates
    const bridgeW = widthX - plateThickness * 2;
    const bridgeGeo = new THREE.BoxGeometry(bridgeW, 0.014, depthZ * 0.72);

    const bottomBridge = new THREE.Mesh(bridgeGeo, greenMat);
    bottomBridge.position.set(0, Math.min(0.035, lengthY * 0.25), 0);
    group.add(bottomBridge);

    if (lengthY > 0.09) {
      const topBridge = new THREE.Mesh(bridgeGeo, greenMat);
      topBridge.position.set(0, lengthY - 0.030, 0);
      group.add(topBridge);
    }

    // Internal black digital servo actuator
    const servoW = Math.min(bridgeW - 0.004, 0.044);
    const servoH = Math.min(lengthY * 0.5, 0.075);
    const servo = new THREE.Mesh(
      new THREE.BoxGeometry(servoW, servoH, depthZ * 0.78),
      servoMat
    );
    servo.position.set(0, lengthY - servoH / 2 - 0.012, 0);
    servo.castShadow = true;
    group.add(servo);
  }

  private _buildWristAndCameraLink(
    group: THREE.Group,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material,
    brassMat: THREE.Material
  ): void {
    this._buildGreenUBracketLink(group, 0.08, 0.046, greenMat, servoMat, hornMat, true, false, 0);

    // 1. Rigid Black Metal L-Bracket connecting green wrist channel to camera
    // Horizontal foot plate anchored flush to front face of green bracket (z = 0.024)
    const footPlateGeo = new THREE.BoxGeometry(0.048, 0.005, 0.028);
    const footPlate = new THREE.Mesh(footPlateGeo, servoMat);
    footPlate.position.set(0, 0.015, 0.036);
    footPlate.castShadow = true;
    group.add(footPlate);

    // Silver mounting screws bolting the L-bracket foot plate to the green channel
    const screwGeo = new THREE.CylinderGeometry(0.003, 0.003, 0.007, 12);
    for (const sx of [-0.016, 0.016]) {
      const screw = new THREE.Mesh(screwGeo, hornMat);
      screw.position.set(sx, 0.018, 0.026);
      group.add(screw);
    }

    // Vertical upright mounting plate of the L-bracket
    const uprightPlateGeo = new THREE.BoxGeometry(0.048, 0.048, 0.005);
    const uprightPlate = new THREE.Mesh(uprightPlateGeo, servoMat);
    uprightPlate.position.set(0, 0.0365, 0.0485);
    uprightPlate.castShadow = true;
    group.add(uprightPlate);

    // 2. Camera Mounting Assembly on the front/top face (+Z) of the wrist link
    const camMountGroup = new THREE.Group();
    camMountGroup.name = 'WristCameraAssembly';
    const [mx, my, mz] = this.config.wristCamera.mountOffset;
    camMountGroup.position.set(mx, my, mz);

    // Black camera PCB housing plate
    const camPlate = new THREE.Mesh(
      new THREE.BoxGeometry(0.046, 0.048, 0.004),
      servoMat
    );
    camPlate.castShadow = true;
    camMountGroup.add(camPlate);

    // 4 Yellow/Brass corner standoffs securely connecting upright L-bracket to camera PCB plate
    // Spans from upright plate (local z = -0.0095) to PCB (local z = -0.002)
    const standoffLength = 0.008;
    const standoffGeo = new THREE.CylinderGeometry(0.0025, 0.0025, standoffLength, 12).rotateX(Math.PI / 2);
    for (const cx of [-0.016, 0.016]) {
      for (const cy of [-0.016, 0.016]) {
        const post = new THREE.Mesh(standoffGeo, brassMat);
        post.position.set(cx, cy, -0.0055);
        post.castShadow = true;
        camMountGroup.add(post);
      }
    }

    // Silver cylindrical wide-angle camera barrel + dark glass lens
    const barrelMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.014, 0.015, 0.022, 24),
      hornMat
    );
    barrelMesh.position.set(0, 0.008, 0.012);
    barrelMesh.castShadow = true;
    camMountGroup.add(barrelMesh);

    const glassMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.010, 0.010, 0.024, 20),
      servoMat
    );
    glassMesh.position.set(0, 0.008, 0.012);
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
