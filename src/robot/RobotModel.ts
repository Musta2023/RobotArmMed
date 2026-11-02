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

    // Base center digital servo actuator (driving J1 pan)
    const baseServo = new THREE.Mesh(
      new THREE.BoxGeometry(0.048, 0.095, 0.048),
      darkServoMat
    );
    baseServo.name = 'J1_ServoHousing';
    baseServo.position.set(0, 0.062, 0);
    this.root.add(baseServo);

    // Realistic servo mounting flanges / ears bolting base servo to platform
    const baseEarGeo = new THREE.BoxGeometry(0.064, 0.005, 0.048);
    const baseEarBottom = new THREE.Mesh(baseEarGeo, darkServoMat);
    baseEarBottom.position.set(0, 0.020, 0);
    this.root.add(baseEarBottom);

    const baseEarTop = new THREE.Mesh(baseEarGeo, darkServoMat);
    baseEarTop.position.set(0, 0.105, 0);
    this.root.add(baseEarTop);

    // 4 Silver mounting screws securing base servo to bottom plate and middle disk
    const baseScrewGeo = new THREE.CylinderGeometry(0.0025, 0.0025, 0.008, 10);
    for (const bx of [-0.026, 0.026]) {
      const screwBot = new THREE.Mesh(baseScrewGeo, silverHornMat);
      screwBot.position.set(bx, 0.022, 0);
      this.root.add(screwBot);

      const screwTop = new THREE.Mesh(baseScrewGeo, silverHornMat);
      screwTop.position.set(bx, 0.108, 0);
      this.root.add(screwTop);
    }

    // Output drive shaft boss coupling base servo to rotating turntable
    const baseShaftCoupler = new THREE.Mesh(
      new THREE.CylinderGeometry(0.016, 0.016, 0.022, 20),
      silverHornMat
    );
    baseShaftCoupler.position.set(0, 0.118, 0);
    baseShaftCoupler.castShadow = true;
    this.root.add(baseShaftCoupler);

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
    this._buildLink1UpperArm(this.linkGroups[1], greenBracketMat, darkServoMat, silverHornMat);
    this._buildLink2Forearm(
      this.linkGroups[2],
      greenBracketMat,
      darkServoMat,
      silverHornMat
    );
    this._buildLink3WristCradle(
      this.linkGroups[3],
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
    cutoutCount: number = 2,
    startY: number = 0
  ): THREE.BufferGeometry {
    const shape = new THREE.Shape();
    const halfD = depthZ / 2;
    const radius = halfD;

    shape.moveTo(halfD, startY);
    if (hasBottomEar) {
      shape.absarc(0, startY, radius, 0, -Math.PI, true);
    } else {
      shape.lineTo(-halfD, startY);
    }
    shape.lineTo(-halfD, lengthY);
    if (hasTopEar) {
      shape.absarc(0, lengthY, radius, Math.PI, 0, true);
    } else {
      shape.lineTo(halfD, lengthY);
    }
    shape.lineTo(halfD, startY);

    if (cutoutCount > 0 && (lengthY - startY) > 0.08) {
      const botBound = hasBottomEar ? (startY + radius) : (startY + 0.015);
      const topBound = hasTopEar ? (lengthY - radius) : (lengthY - 0.015);
      const usableH = topBound - botBound;
      if (usableH > 0.02) {
        const windowH = Math.min(0.040, usableH / (cutoutCount + 0.6));
        const windowW = depthZ * 0.48;
        const spacing = usableH / cutoutCount;
        for (let k = 0; k < cutoutCount; k++) {
          const cy = botBound + spacing * (k + 0.5);
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

  /**
   * Builds Link 2 (Forearm, between J3 Elbow Pitch and J4 Forearm Roll).
   * - Base at y = 0: Concentric rounded ears mating with J3 elbow pitch (X-axis).
   * - Distal end at y = 0.150: Flat termination with a transverse circular stator collar
   *   and bearing sleeve in the XZ plane (normal to Y) for J4 axial torsion.
   * - Internal actuator: Black servo driving the J4 axial torsion roll.
   */
  /**
   * Builds Link 1 (Upper Arm, between J2 Shoulder Lift and J3 Elbow Pitch).
   * - Base at y = 0: ROTATING CNC Aluminum Servo Horn (Palonnier J2) and idler pivot
   *   mechanically attached to Link 1, rotating with Link 1 around the X-axis.
   * - Upright channel: Twin green profiled side plates rising to y = 0.320.
   * - Internal actuator: Black digital servo firmly attached to Link 1, driving J3 Elbow Pitch.
   */
  private _buildLink1UpperArm(
    group: THREE.Group,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material
  ): void {
    const lengthY = 0.320;
    const widthX = 0.072;
    const halfX = widthX / 2; // 0.036
    const plateThickness = 0.004;
    const depthZ = 0.048;

    // =========================================================================
    // J2 Shoulder Pivot (y = 0 in Link 1): ROTATING HORN & IDLER ATTACHED TO LINK 1
    // =========================================================================
    // 1. Rotating CNC Aluminum Servo Horn (Palonnier J2) mechanically driven by J2 servo
    const j2Horn = new THREE.Mesh(
      new THREE.CylinderGeometry(0.014, 0.014, 0.004, 24).rotateZ(Math.PI / 2),
      hornMat
    );
    j2Horn.name = 'J2_RotatingHorn';
    j2Horn.position.set(0.030, 0, 0);
    j2Horn.castShadow = true;
    group.add(j2Horn);

    // 4 Silver M2.5 retaining screws bolting horn face to Link 1's right green plate
    const j2ScrewGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.007, 10).rotateZ(Math.PI / 2);
    for (let k = 0; k < 4; k++) {
      const angle = (k * Math.PI) / 2;
      const sy = Math.cos(angle) * 0.009;
      const sz = Math.sin(angle) * 0.009;
      const screw = new THREE.Mesh(j2ScrewGeo, hornMat);
      screw.position.set(0.035, sy, sz);
      group.add(screw);
    }

    // Central M3 socket cap screw clamping horn to J2 output spline
    const j2CenterScrew = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0035, 0.0035, 0.010, 12).rotateZ(Math.PI / 2),
      servoMat
    );
    j2CenterScrew.position.set(0.038, 0, 0);
    group.add(j2CenterScrew);

    // Passive side: Rotating idler stub pin and retaining washer
    const j2IdlerPin = new THREE.Mesh(
      new THREE.CylinderGeometry(0.007, 0.007, 0.012, 14).rotateZ(Math.PI / 2),
      servoMat
    );
    j2IdlerPin.position.set(-0.030, 0, 0);
    group.add(j2IdlerPin);

    const j2IdlerCap = new THREE.Mesh(
      new THREE.CylinderGeometry(0.013, 0.013, 0.003, 18).rotateZ(Math.PI / 2),
      hornMat
    );
    j2IdlerCap.position.set(-0.037, 0, 0);
    j2IdlerCap.castShadow = true;
    group.add(j2IdlerCap);

    // Exterior bearing cap on active side
    const j2ActiveCap = new THREE.Mesh(
      new THREE.CylinderGeometry(0.015, 0.015, 0.003, 20).rotateZ(Math.PI / 2),
      hornMat
    );
    j2ActiveCap.position.set(0.038, 0, 0);
    j2ActiveCap.castShadow = true;
    group.add(j2ActiveCap);

    // =========================================================================
    // Green U-Bracket Structure of Link 1
    // =========================================================================
    const plateGeo = this._createChannelPlateGeometry(lengthY, depthZ, plateThickness, true, true, 2, 0);
    for (const sx of [-halfX, halfX]) {
      const sideWall = new THREE.Mesh(plateGeo, greenMat);
      sideWall.position.set(sx, 0, 0);
      sideWall.castShadow = true;
      sideWall.receiveShadow = true;
      group.add(sideWall);
    }

    // Lower and upper cross-bridge stiffeners
    const bridgeW = widthX - plateThickness * 2;
    const botBridge = new THREE.Mesh(new THREE.BoxGeometry(bridgeW, 0.014, depthZ * 0.72), greenMat);
    botBridge.position.set(0, 0.035, 0);
    group.add(botBridge);

    const topBridge = new THREE.Mesh(new THREE.BoxGeometry(bridgeW, 0.014, depthZ * 0.72), greenMat);
    topBridge.position.set(0, 0.220, 0);
    group.add(topBridge);

    // =========================================================================
    // Internal J3 Elbow Pitch Digital Servo Actuator (Powers Joint 3)
    // Firmly attached to Link 1 (stays fixed to Link 1 when J3 rotates)
    // =========================================================================
    const servoW = Math.min(bridgeW - 0.004, 0.044);
    const servoH = 0.075;
    const servoPosY = 0.270;

    const j3Servo = new THREE.Mesh(new THREE.BoxGeometry(servoW, servoH, depthZ * 0.78), servoMat);
    j3Servo.name = 'J3_ServoHousing';
    j3Servo.position.set(0, servoPosY, 0);
    j3Servo.castShadow = true;
    group.add(j3Servo);

    // Mounting ears clamping J3 servo to Link 1 side plates
    const earGeo = new THREE.BoxGeometry(widthX - 0.002, 0.004, depthZ * 0.60);
    const earTop = new THREE.Mesh(earGeo, servoMat);
    earTop.position.set(0, servoPosY + servoH / 2 - 0.003, 0);
    earTop.castShadow = true;
    group.add(earTop);

    const earBot = new THREE.Mesh(earGeo, servoMat);
    earBot.position.set(0, servoPosY - servoH / 2 + 0.003, 0);
    earBot.castShadow = true;
    group.add(earBot);

    // 4 Silver M3 screws clamping J3 servo ears into Link 1 side walls
    const earScrewGeo = new THREE.CylinderGeometry(0.0025, 0.0025, 0.008, 10).rotateZ(Math.PI / 2);
    for (const sx of [-halfX * 1.02, halfX * 1.02]) {
      for (const ey of [servoPosY + servoH / 2 - 0.003, servoPosY - servoH / 2 + 0.003]) {
        const screw = new THREE.Mesh(earScrewGeo, hornMat);
        screw.position.set(sx, ey, 0);
        group.add(screw);
      }
    }

    // Stationary spline drive neck on J3 servo casing at J3 axis (y = 0.320)
    const j3StatorBushing = new THREE.Mesh(
      new THREE.CylinderGeometry(0.009, 0.009, 0.012, 16).rotateZ(Math.PI / 2),
      servoMat
    );
    j3StatorBushing.position.set(0.022, lengthY, 0);
    j3StatorBushing.castShadow = true;
    group.add(j3StatorBushing);

    // Passive side stationary idler bearing socket on J3 servo casing at J3 axis (y = 0.320)
    const j3IdlerSocket = new THREE.Mesh(
      new THREE.CylinderGeometry(0.009, 0.009, 0.012, 14).rotateZ(Math.PI / 2),
      servoMat
    );
    j3IdlerSocket.position.set(-0.022, lengthY, 0);
    group.add(j3IdlerSocket);
  }

  private _buildLink2Forearm(
    group: THREE.Group,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material
  ): void {
    const lengthY = 0.150;
    const widthX = 0.066;
    const halfX = widthX / 2; // 0.033
    const plateThickness = 0.004;
    const depthZ = 0.048;

    // =========================================================================
    // J3 Elbow Pivot (y = 0 in Link 2): ROTATING HORN & IDLER ATTACHED TO LINK 2
    // =========================================================================
    // 1. Rotating CNC Aluminum Servo Horn (Palonnier J3) mechanically driven by J3 servo
    const j3Horn = new THREE.Mesh(
      new THREE.CylinderGeometry(0.013, 0.013, 0.004, 24).rotateZ(Math.PI / 2),
      hornMat
    );
    j3Horn.name = 'J3_RotatingHorn';
    j3Horn.position.set(0.028, 0, 0);
    j3Horn.castShadow = true;
    group.add(j3Horn);

    // 4 Silver M2.5 retaining screws bolting horn face to Link 2's right green plate
    const j3ScrewGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.007, 10).rotateZ(Math.PI / 2);
    for (let k = 0; k < 4; k++) {
      const angle = (k * Math.PI) / 2;
      const sy = Math.cos(angle) * 0.008;
      const sz = Math.sin(angle) * 0.008;
      const screw = new THREE.Mesh(j3ScrewGeo, hornMat);
      screw.position.set(0.033, sy, sz);
      group.add(screw);
    }

    // Central M3 socket cap screw clamping horn to J3 output spline
    const j3CenterScrew = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0035, 0.0035, 0.010, 12).rotateZ(Math.PI / 2),
      servoMat
    );
    j3CenterScrew.position.set(0.035, 0, 0);
    group.add(j3CenterScrew);

    // Passive side: Rotating idler stub pin and retaining washer
    const j3IdlerPin = new THREE.Mesh(
      new THREE.CylinderGeometry(0.007, 0.007, 0.012, 14).rotateZ(Math.PI / 2),
      servoMat
    );
    j3IdlerPin.position.set(-0.028, 0, 0);
    group.add(j3IdlerPin);

    const j3IdlerCap = new THREE.Mesh(
      new THREE.CylinderGeometry(0.013, 0.013, 0.003, 18).rotateZ(Math.PI / 2),
      hornMat
    );
    j3IdlerCap.position.set(-0.034, 0, 0);
    j3IdlerCap.castShadow = true;
    group.add(j3IdlerCap);

    // Exterior bearing cap on active side
    const j3ActiveCap = new THREE.Mesh(
      new THREE.CylinderGeometry(0.015, 0.015, 0.003, 20).rotateZ(Math.PI / 2),
      hornMat
    );
    j3ActiveCap.position.set(0.034, 0, 0);
    j3ActiveCap.castShadow = true;
    group.add(j3ActiveCap);

    // Side plates: hasBottomEar = true (for J3 elbow pitch), hasTopEar = false (terminates flat at J4 collar)
    const plateGeo = this._createChannelPlateGeometry(
      lengthY,
      depthZ,
      plateThickness,
      true,
      false,
      1,
      0
    );

    for (const sx of [-halfX, halfX]) {
      const sideWall = new THREE.Mesh(plateGeo, greenMat);
      sideWall.position.set(sx, 0, 0);
      sideWall.castShadow = true;
      sideWall.receiveShadow = true;
      group.add(sideWall);
    }

    // Lower green cross-bridge stiffener plate near J3
    const bridgeW = widthX - plateThickness * 2;
    const bridgeGeo = new THREE.BoxGeometry(bridgeW, 0.012, depthZ * 0.72);
    const bottomBridge = new THREE.Mesh(bridgeGeo, greenMat);
    bottomBridge.position.set(0, 0.035, 0);
    group.add(bottomBridge);

    // Internal black digital servo actuator (driving J4 axial roll)
    // Firmly attached to Link 2 parent link (remains fixed when J4 rotates)
    const servo = new THREE.Mesh(
      new THREE.BoxGeometry(0.042, 0.058, depthZ * 0.78),
      servoMat
    );
    servo.name = 'J4_ServoHousing';
    servo.position.set(0, 0.095, 0);
    servo.castShadow = true;
    group.add(servo);

    // Servo mounting tabs / ears bolting it rigidly to the green side plates
    const link2EarGeo = new THREE.BoxGeometry(widthX - 0.004, 0.004, depthZ * 0.64);
    const link2EarTop = new THREE.Mesh(link2EarGeo, servoMat);
    link2EarTop.position.set(0, 0.118, 0);
    link2EarTop.castShadow = true;
    group.add(link2EarTop);

    const link2EarBot = new THREE.Mesh(link2EarGeo, servoMat);
    link2EarBot.position.set(0, 0.072, 0);
    link2EarBot.castShadow = true;
    group.add(link2EarBot);

    // 4 Silver M2.5 screws bolting servo ears through side walls
    const link2ScrewGeo = new THREE.CylinderGeometry(0.0022, 0.0022, 0.008, 10).rotateZ(Math.PI / 2);
    for (const sx of [-halfX * 1.02, halfX * 1.02]) {
      for (const ey of [0.072, 0.118]) {
        const screw = new THREE.Mesh(link2ScrewGeo, hornMat);
        screw.position.set(sx, ey, 0);
        group.add(screw);
      }
    }

    // Cylindrical drive shaft coupling boss extending along +Y into the J4 stator socket
    const j4DriveShaft = new THREE.Mesh(
      new THREE.CylinderGeometry(0.008, 0.008, 0.024, 20),
      hornMat
    );
    j4DriveShaft.position.set(0, 0.136, 0);
    j4DriveShaft.castShadow = true;
    group.add(j4DriveShaft);

    // Silver drive collar with clamping set screws
    const j4CouplerRing = new THREE.Mesh(
      new THREE.CylinderGeometry(0.012, 0.012, 0.006, 20),
      hornMat
    );
    j4CouplerRing.position.set(0, 0.142, 0);
    j4CouplerRing.castShadow = true;
    group.add(j4CouplerRing);

    // =========================================================================
    // J4 Distal Axial Torsion Stator Collar Assembly (XZ plane, normal to Y)
    // =========================================================================
    // 1. Transverse green circular stator mounting flange at distal end
    const statorFlange = new THREE.Mesh(
      new THREE.CylinderGeometry(0.028, 0.028, 0.005, 32),
      greenMat
    );
    statorFlange.position.set(0, lengthY - 0.0025, 0);
    statorFlange.castShadow = true;
    group.add(statorFlange);

    // 2. Silver stator bearing collar / sleeve ring
    const statorCollar = new THREE.Mesh(
      new THREE.CylinderGeometry(0.022, 0.022, 0.003, 28),
      hornMat
    );
    statorCollar.position.set(0, lengthY - 0.001, 0);
    statorCollar.castShadow = true;
    group.add(statorCollar);

    // 3. Central drive socket / inner bearing bushing
    const innerSocket = new THREE.Mesh(
      new THREE.CylinderGeometry(0.014, 0.014, 0.005, 24),
      servoMat
    );
    innerSocket.position.set(0, lengthY - 0.001, 0);
    group.add(innerSocket);

    // 4. Perimeter stator assembly bolts
    const boltGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.004, 10);
    for (let k = 0; k < 4; k++) {
      const angle = (k * Math.PI) / 2 + Math.PI / 4;
      const bx = Math.cos(angle) * 0.024;
      const bz = Math.sin(angle) * 0.024;
      const bolt = new THREE.Mesh(boltGeo, hornMat);
      bolt.position.set(bx, lengthY - 0.001, bz);
      group.add(bolt);
    }
  }

  /**
   * Builds Link 3 (Wrist Bracket / Cradle, between J4 Forearm Roll and J5 Wrist Pitch).
   * - Base at y = 0: Transverse circular rotor turntable in the XZ plane with silver horn,
   *   bearing thrust ring, and perimeter fasteners, rotating around Y for J4 axial torsion.
   * - Upright structure: Twin green side plates rising from the turntable to y = 0.130.
   * - Top at y = 0.130: Concentric rounded ears with silver bearing caps along X for J5 Wrist Pitch.
   * - Internal actuator: Black digital servo driving J5 wrist pitch.
   */
  private _buildLink3WristCradle(
    group: THREE.Group,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material
  ): void {
    const lengthY = 0.130;
    const widthX = 0.054;
    const halfX = widthX / 2;
    const plateThickness = 0.004;
    const depthZ = 0.044;
    const turntableH = 0.007;

    // =========================================================================
    // J4 Proximal Axial Torsion Rotor Turntable Assembly (XZ plane, normal to Y)
    // =========================================================================
    // 1. Silver servo horn / thrust washer ring mating with Link 2's stator collar
    const rotorHorn = new THREE.Mesh(
      new THREE.CylinderGeometry(0.022, 0.022, 0.0025, 28),
      hornMat
    );
    rotorHorn.position.set(0, 0.00125, 0);
    rotorHorn.castShadow = true;
    group.add(rotorHorn);

    // 2. Rotating green circular turntable rotor disc at y = 0.002..0.007
    const rotorTurntable = new THREE.Mesh(
      new THREE.CylinderGeometry(0.027, 0.027, turntableH - 0.002, 32),
      greenMat
    );
    rotorTurntable.name = 'J4_RotatingTurntable';
    rotorTurntable.position.set(0, (turntableH + 0.002) / 2, 0);
    rotorTurntable.castShadow = true;
    group.add(rotorTurntable);

    // 3. Central drive spindle / hub
    const rotorSpindle = new THREE.Mesh(
      new THREE.CylinderGeometry(0.010, 0.010, 0.006, 20),
      servoMat
    );
    rotorSpindle.position.set(0, 0.003, 0);
    group.add(rotorSpindle);

    // 4. Perimeter rotor retaining screws
    const screwYGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.004, 10);
    for (let k = 0; k < 6; k++) {
      const angle = (k * Math.PI) / 3;
      const rx = Math.cos(angle) * 0.021;
      const rz = Math.sin(angle) * 0.021;
      const screw = new THREE.Mesh(screwYGeo, hornMat);
      screw.position.set(rx, turntableH + 0.001, rz);
      group.add(screw);
    }

    // =========================================================================
    // Upright Green Side Plates (rising from y = turntableH to y = lengthY)
    // =========================================================================
    // hasBottomEar = false (starts flat on turntable), hasTopEar = true (for J5 wrist pitch)
    const plateGeo = this._createChannelPlateGeometry(
      lengthY,
      depthZ,
      plateThickness,
      false,
      true,
      1,
      turntableH
    );

    const screwXGeo = new THREE.CylinderGeometry(0.0035, 0.0035, 0.008, 12).rotateZ(Math.PI / 2);

    for (const sx of [-halfX, halfX]) {
      const sideWall = new THREE.Mesh(plateGeo, greenMat);
      sideWall.position.set(sx, 0, 0);
      sideWall.castShadow = true;
      sideWall.receiveShadow = true;
      group.add(sideWall);

      // Concentric silver pivot bearing cap & retaining pin at top pivot (y = 0.130, J5 pitch)
      const horn = new THREE.Mesh(
        new THREE.CylinderGeometry(0.015, 0.015, 0.003, 20).rotateZ(Math.PI / 2),
        hornMat
      );
      horn.position.set(sx * 1.05, lengthY, 0);
      horn.castShadow = true;
      group.add(horn);

      const centerPin = new THREE.Mesh(screwXGeo, servoMat);
      centerPin.position.set(sx * 1.09, lengthY, 0);
      group.add(centerPin);
    }

    // Cross-bridge stiffener plate
    const bridgeW = widthX - plateThickness * 2;
    const bridgeGeo = new THREE.BoxGeometry(bridgeW, 0.012, depthZ * 0.70);
    const midBridge = new THREE.Mesh(bridgeGeo, greenMat);
    midBridge.position.set(0, 0.035, 0);
    group.add(midBridge);

    // Internal black digital servo actuator (driving J5 wrist pitch)
    // Firmly attached to Link 3 parent link (remains fixed when J5 rotates)
    const servo = new THREE.Mesh(
      new THREE.BoxGeometry(0.040, 0.055, depthZ * 0.76),
      servoMat
    );
    servo.name = 'J5_ServoHousing';
    servo.position.set(0, 0.085, 0);
    servo.castShadow = true;
    group.add(servo);

    // Servo mounting tabs / ears bolting it rigidly to the side walls
    const link3EarGeo = new THREE.BoxGeometry(widthX - 0.003, 0.004, depthZ * 0.62);
    const link3EarTop = new THREE.Mesh(link3EarGeo, servoMat);
    link3EarTop.position.set(0, 0.108, 0);
    link3EarTop.castShadow = true;
    group.add(link3EarTop);

    const link3EarBot = new THREE.Mesh(link3EarGeo, servoMat);
    link3EarBot.position.set(0, 0.062, 0);
    link3EarBot.castShadow = true;
    group.add(link3EarBot);

    // 4 Silver screws bolting servo ears through side walls
    const link3ScrewGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.008, 10).rotateZ(Math.PI / 2);
    for (const sx of [-halfX * 1.02, halfX * 1.02]) {
      for (const ey of [0.062, 0.108]) {
        const screw = new THREE.Mesh(link3ScrewGeo, hornMat);
        screw.position.set(sx, ey, 0);
        group.add(screw);
      }
    }

    // Stationary spline drive neck on J5 servo casing at J5 axis (y = 0.130)
    const j5StatorBushing = new THREE.Mesh(
      new THREE.CylinderGeometry(0.008, 0.008, 0.012, 16).rotateZ(Math.PI / 2),
      servoMat
    );
    j5StatorBushing.position.set(halfX * 0.60, lengthY, 0);
    j5StatorBushing.castShadow = true;
    group.add(j5StatorBushing);

    // Passive side stationary idler bearing socket on J5 servo casing at J5 axis (y = 0.130)
    const j5IdlerSocket = new THREE.Mesh(
      new THREE.CylinderGeometry(0.008, 0.008, 0.012, 14).rotateZ(Math.PI / 2),
      servoMat
    );
    j5IdlerSocket.position.set(-halfX * 0.60, lengthY, 0);
    group.add(j5IdlerSocket);
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

    // 6 Perimeter bolts on rotating turntable
    const boltGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.005, 10);
    for (let k = 0; k < 6; k++) {
      const angle = (k * Math.PI) / 3;
      const bx = Math.cos(angle) * 0.048;
      const bz = Math.sin(angle) * 0.048;
      const bolt = new THREE.Mesh(boltGeo, hornMat);
      bolt.position.set(bx, -0.004, bz);
      group.add(bolt);
    }

    // Green shoulder U-bracket cradle + black digital servo (driving J2 shoulder lift)
    // Firmly attached to Link 0 parent link (remains fixed when J2 rotates)
    const servoBody = new THREE.Mesh(
      new THREE.BoxGeometry(0.046, 0.088, 0.044),
      servoMat
    );
    servoBody.name = 'J2_ServoHousing';
    servoBody.position.set(0, 0.055, 0);
    servoBody.castShadow = true;
    group.add(servoBody);

    // Servo casing mounting tabs / ears bolting it firmly to the green shoulder side plates
    const turretEarGeo = new THREE.BoxGeometry(0.060, 0.004, 0.040);
    const turretEarTop = new THREE.Mesh(turretEarGeo, servoMat);
    turretEarTop.position.set(0, 0.090, 0);
    turretEarTop.castShadow = true;
    group.add(turretEarTop);

    const turretEarBottom = new THREE.Mesh(turretEarGeo, servoMat);
    turretEarBottom.position.set(0, 0.020, 0);
    turretEarBottom.castShadow = true;
    group.add(turretEarBottom);

    // 4 Silver M3 mounting screws clamping turret servo ears into the side plates
    const turretScrewGeo = new THREE.CylinderGeometry(0.0025, 0.0025, 0.008, 10).rotateZ(Math.PI / 2);
    for (const sx of [-0.029, 0.029]) {
      for (const sy of [0.020, 0.090]) {
        const screw = new THREE.Mesh(turretScrewGeo, hornMat);
        screw.position.set(sx, sy, 0);
        group.add(screw);
      }
    }

    // Stationary spline drive neck / stator collar on servo casing at J2 axis (y = 0.120)
    const j2StatorBushing = new THREE.Mesh(
      new THREE.CylinderGeometry(0.009, 0.009, 0.012, 16).rotateZ(Math.PI / 2),
      servoMat
    );
    j2StatorBushing.position.set(0.022, 0.120, 0);
    j2StatorBushing.castShadow = true;
    group.add(j2StatorBushing);

    // Passive side stationary idler bearing socket on servo casing at J2 axis (y = 0.120)
    const j2IdlerSocket = new THREE.Mesh(
      new THREE.CylinderGeometry(0.009, 0.009, 0.012, 14).rotateZ(Math.PI / 2),
      servoMat
    );
    j2IdlerSocket.position.set(-0.022, 0.120, 0);
    group.add(j2IdlerSocket);

    // Green Turret side plates
    const plateGeo = this._createChannelPlateGeometry(0.120, 0.048, 0.004, false, true, 1);
    for (const sx of [-0.031, 0.031]) {
      const sidePlate = new THREE.Mesh(plateGeo, greenMat);
      sidePlate.position.set(sx, 0, 0);
      sidePlate.castShadow = true;
      group.add(sidePlate);
    }
  }

  private _buildWristAndCameraLink(
    group: THREE.Group,
    greenMat: THREE.Material,
    servoMat: THREE.Material,
    hornMat: THREE.Material,
    brassMat: THREE.Material
  ): void {
    const lengthY = 0.08;
    const widthX = 0.046;
    const halfX = widthX / 2; // 0.023
    const plateThickness = 0.004;
    const depthZ = 0.048;

    // =========================================================================
    // J5 Wrist Pitch Pivot (y = 0 in Link 4): ROTATING HORN & IDLER ATTACHED TO LINK 4
    // =========================================================================
    // 1. Rotating CNC Aluminum Servo Horn (Palonnier J5) mechanically driven by J5 servo
    const j5Horn = new THREE.Mesh(
      new THREE.CylinderGeometry(0.011, 0.011, 0.003, 20).rotateZ(Math.PI / 2),
      hornMat
    );
    j5Horn.name = 'J5_RotatingHorn';
    j5Horn.position.set(0.019, 0, 0);
    j5Horn.castShadow = true;
    group.add(j5Horn);

    // 4 Silver M2.5 retaining screws bolting horn face to Link 4's right green plate
    const j5ScrewGeo = new THREE.CylinderGeometry(0.0018, 0.0018, 0.006, 10).rotateZ(Math.PI / 2);
    for (let k = 0; k < 4; k++) {
      const angle = (k * Math.PI) / 2;
      const sy = Math.cos(angle) * 0.007;
      const sz = Math.sin(angle) * 0.007;
      const screw = new THREE.Mesh(j5ScrewGeo, hornMat);
      screw.position.set(0.023, sy, sz);
      group.add(screw);
    }

    // Central M2.5 socket cap screw clamping horn to J5 output spline
    const j5CenterScrew = new THREE.Mesh(
      new THREE.CylinderGeometry(0.003, 0.003, 0.008, 12).rotateZ(Math.PI / 2),
      servoMat
    );
    j5CenterScrew.position.set(0.025, 0, 0);
    group.add(j5CenterScrew);

    // Passive side: Rotating idler stub pin and retaining washer
    const j5IdlerPin = new THREE.Mesh(
      new THREE.CylinderGeometry(0.006, 0.006, 0.010, 14).rotateZ(Math.PI / 2),
      servoMat
    );
    j5IdlerPin.position.set(-0.019, 0, 0);
    group.add(j5IdlerPin);

    const j5IdlerCap = new THREE.Mesh(
      new THREE.CylinderGeometry(0.012, 0.012, 0.003, 18).rotateZ(Math.PI / 2),
      hornMat
    );
    j5IdlerCap.position.set(-0.024, 0, 0);
    j5IdlerCap.castShadow = true;
    group.add(j5IdlerCap);

    // Exterior bearing cap on active side
    const j5ActiveCap = new THREE.Mesh(
      new THREE.CylinderGeometry(0.013, 0.013, 0.003, 20).rotateZ(Math.PI / 2),
      hornMat
    );
    j5ActiveCap.position.set(0.024, 0, 0);
    j5ActiveCap.castShadow = true;
    group.add(j5ActiveCap);

    // Green profiled side walls of Link 4
    const plateGeo = this._createChannelPlateGeometry(lengthY, depthZ, plateThickness, true, false, 0, 0);
    for (const sx of [-halfX, halfX]) {
      const sideWall = new THREE.Mesh(plateGeo, greenMat);
      sideWall.position.set(sx, 0, 0);
      sideWall.castShadow = true;
      sideWall.receiveShadow = true;
      group.add(sideWall);
    }

    // Internal J6 Tool Roll Digital Servo Actuator (Powers Joint 6)
    // Firmly attached to Link 4 parent link (remains fixed when J6 rotates)
    const j6Servo = new THREE.Mesh(
      new THREE.BoxGeometry(0.038, 0.040, depthZ * 0.72),
      servoMat
    );
    j6Servo.name = 'J6_ServoHousing';
    j6Servo.position.set(0, 0.044, 0);
    j6Servo.castShadow = true;
    group.add(j6Servo);

    // Mounting ears clamping J6 servo to Link 4 side walls
    const j6EarGeo = new THREE.BoxGeometry(widthX - 0.002, 0.004, depthZ * 0.60);
    const j6EarTop = new THREE.Mesh(j6EarGeo, servoMat);
    j6EarTop.position.set(0, 0.060, 0);
    j6EarTop.castShadow = true;
    group.add(j6EarTop);

    const j6EarBot = new THREE.Mesh(j6EarGeo, servoMat);
    j6EarBot.position.set(0, 0.028, 0);
    j6EarBot.castShadow = true;
    group.add(j6EarBot);

    // 4 Silver screws clamping J6 servo ears to side plates
    const j6ScrewGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.008, 10).rotateZ(Math.PI / 2);
    for (const sx of [-halfX * 1.02, halfX * 1.02]) {
      for (const ey of [0.028, 0.060]) {
        const screw = new THREE.Mesh(j6ScrewGeo, hornMat);
        screw.position.set(sx, ey, 0);
        group.add(screw);
      }
    }

    // Output drive spindle pointing along +Y into the J6 tool flange
    const j6DriveSpindle = new THREE.Mesh(
      new THREE.CylinderGeometry(0.006, 0.006, 0.014, 18),
      hornMat
    );
    j6DriveSpindle.position.set(0, 0.072, 0);
    j6DriveSpindle.castShadow = true;
    group.add(j6DriveSpindle);

    // =========================================================================
    // J6 Distal Tool Roll Stator Bearing Assembly (Zero Gap Interface to Gripper)
    // =========================================================================
    // 1. Green distal end-closure cap plate spanning between side walls (closing wrist channel)
    const capPlateGeo = new THREE.BoxGeometry(0.046, 0.005, 0.048);
    const capPlate = new THREE.Mesh(capPlateGeo, greenMat);
    capPlate.position.set(0, lengthY - 0.0025, 0);
    capPlate.castShadow = true;
    group.add(capPlate);

    // 2. Silver stator bearing collar / sleeve ring at distal face (y = 0.080)
    const statorCollar = new THREE.Mesh(
      new THREE.CylinderGeometry(0.017, 0.017, 0.004, 28),
      hornMat
    );
    statorCollar.position.set(0, lengthY, 0);
    statorCollar.castShadow = true;
    group.add(statorCollar);

    // 3. Central drive bushing
    const innerSocket = new THREE.Mesh(
      new THREE.CylinderGeometry(0.009, 0.009, 0.006, 20),
      servoMat
    );
    innerSocket.position.set(0, lengthY, 0);
    group.add(innerSocket);

    // 4. Perimeter stator mounting bolts (4 silver screws)
    const boltGeo = new THREE.CylinderGeometry(0.0016, 0.0016, 0.004, 10);
    for (let k = 0; k < 4; k++) {
      const angle = (k * Math.PI) / 2 + Math.PI / 4;
      const bx = Math.cos(angle) * 0.013;
      const bz = Math.sin(angle) * 0.013;
      const bolt = new THREE.Mesh(boltGeo, hornMat);
      bolt.position.set(bx, lengthY + 0.001, bz);
      group.add(bolt);
    }

    // 5. Silver side fastening screws bolting end cap plate through green side walls
    const capScrewGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.006, 10).rotateZ(Math.PI / 2);
    for (const sx of [-0.023, 0.023]) {
      for (const sz of [-0.014, 0.014]) {
        const screw = new THREE.Mesh(capScrewGeo, hornMat);
        screw.position.set(sx, lengthY - 0.0025, sz);
        group.add(screw);
      }
    }

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
