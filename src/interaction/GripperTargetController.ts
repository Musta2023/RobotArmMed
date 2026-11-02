/**
 * @file GripperTargetController.ts
 * Direct 3D Mouse Control of the Robot Gripper (End-Effector Target Gizmo)
 * with 6-DOF Translation (X, Y, Z) and Orientation (Roll, Pitch, Yaw / Quaternion)
 * via Damped Least Squares Inverse Kinematics and Physical Joint Motors (Sections 3 & 4).
 *
 * Rules enforced:
 * - Never teleports the robot arm or its individual links.
 * - All movements are solved via IK and executed by the physical joint PD motors.
 * - Detects unreachable targets, clamps to the closest valid reachable workspace pose,
 *   displays a visual warning, and never forces joints beyond their limits.
 */

import * as THREE from 'three';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { RobotModel } from '../robot/RobotModel.js';
import { MotionController } from '../control/MotionController.js';
import {
  solveInverseKinematics,
  IKSolverResult,
  createTopDownQuaternion
} from '../control/InverseKinematics.js';
import { CameraManager } from '../scene/CameraManager.js';
import { DEG2RAD, RAD2DEG } from '../robot/RobotJoints.js';

export type GizmoTransformMode = 'translate' | 'rotate';

export class GripperTargetController {
  public robotModel: RobotModel;
  public motionController: MotionController;
  public scene: THREE.Scene;
  public cameraManager: CameraManager;

  public targetAnchor: THREE.Group;
  public ringMesh!: THREE.Mesh;
  public transformControls: TransformControls | null = null;
  public transformControlsHelper: THREE.Object3D | null = null;
  public gizmoMode: GizmoTransformMode = 'translate';
  public isGizmoActive: boolean = false;
  public isDraggingGizmo: boolean = false;

  public isTargetReachable: boolean = true;
  public warningMessage: string = '';
  public lastIKResult: IKSolverResult | null = null;

  constructor(
    robotModel: RobotModel,
    motionController: MotionController,
    scene: THREE.Scene,
    cameraManager: CameraManager,
    rendererDomElement?: HTMLElement | null
  ) {
    this.robotModel = robotModel;
    this.motionController = motionController;
    this.scene = scene;
    this.cameraManager = cameraManager;

    this.targetAnchor = new THREE.Group();
    this.targetAnchor.name = 'GripperTargetGizmoAnchor';
    this._buildTargetIndicatorMesh();
    this.scene.add(this.targetAnchor);

    // Initialize anchor at current robot TCP pose
    this.syncToCurrentTCP();

    if (typeof window !== 'undefined' && rendererDomElement) {
      this._initTransformControls(rendererDomElement);
    }
  }

  private _buildTargetIndicatorMesh(): void {
    const ringGeo = new THREE.TorusGeometry(0.038, 0.0035, 12, 32);
    ringGeo.rotateX(Math.PI / 2);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x10b981,
      transparent: true,
      opacity: 0.85
    });
    this.ringMesh = new THREE.Mesh(ringGeo, ringMat);
    this.ringMesh.name = 'GripperTargetRing';
    this.targetAnchor.add(this.ringMesh);

    // Orientation direction cone pointing along gripper local +Y (jaw forward axis)
    const coneGeo = new THREE.ConeGeometry(0.010, 0.028, 12);
    const coneMat = new THREE.MeshBasicMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.8
    });
    const coneMesh = new THREE.Mesh(coneGeo, coneMat);
    coneMesh.position.set(0, 0.024, 0);
    this.targetAnchor.add(coneMesh);

    const axes = new THREE.AxesHelper(0.048);
    this.targetAnchor.add(axes);
  }

  private _initTransformControls(domElement: HTMLElement): void {
    this.transformControls = new TransformControls(
      this.cameraManager.mainCamera,
      domElement
    );
    this.transformControls.setSize(0.75);
    this.transformControls.setMode(this.gizmoMode);
    this.transformControls.attach(this.targetAnchor);
    this.transformControls.enabled = false;

    const tcAny = this.transformControls as unknown as {
      getHelper?: () => THREE.Object3D;
    };
    this.transformControlsHelper =
      typeof tcAny.getHelper === 'function'
        ? tcAny.getHelper()
        : (this.transformControls as unknown as THREE.Object3D);
    this.transformControlsHelper.visible = false;

    this.transformControls.addEventListener('dragging-changed', (event: { value: unknown }) => {
      const dragging = Boolean(event.value);
      this.isDraggingGizmo = dragging;
      if (this.cameraManager.orbitControls) {
        this.cameraManager.orbitControls.enabled = !dragging;
      }
    });

    this.transformControls.addEventListener('change', () => {
      if (this.isDraggingGizmo && this.isGizmoActive) {
        this._applyAnchorPoseToRobot();
      }
    });

    this.scene.add(this.transformControlsHelper);
  }

  /**
   * Enables or disables the 3D gripper target gizmo in the viewport.
   */
  public setGizmoEnabled(enabled: boolean): void {
    this.isGizmoActive = enabled;
    this.targetAnchor.visible = true;
    if (enabled) {
      this.syncToCurrentTCP();
    }
    if (this.transformControls) {
      this.transformControls.enabled = enabled;
      if (this.transformControlsHelper) {
        this.transformControlsHelper.visible = enabled;
      }
      if (enabled) {
        this.transformControls.attach(this.targetAnchor);
      } else {
        this.transformControls.detach();
      }
    }
  }

  /**
   * Switches the 3D gizmo between 'translate' (X/Y/Z position) and 'rotate' (X/Y/Z orientation).
   */
  public setGizmoMode(mode: GizmoTransformMode): void {
    this.gizmoMode = mode;
    if (this.transformControls) {
      this.transformControls.setMode(mode);
    }
  }

  /**
   * Synchronizes the 3D target gizmo anchor to the robot's live End-Effector (TCP) pose.
   */
  public syncToCurrentTCP(): void {
    const tcp = this.robotModel.gripper.getGraspFrameWorldPose();
    this.targetAnchor.position.copy(tcp.position);
    this.targetAnchor.quaternion.copy(tcp.quaternion);
    this.targetAnchor.updateMatrixWorld(true);
    this.isTargetReachable = true;
    this.warningMessage = '';
    this._updateRingColor(true);
  }

  private _updateRingColor(reachable: boolean): void {
    const mat = this.ringMesh.material as THREE.MeshBasicMaterial;
    if (mat) {
      mat.color.setHex(reachable ? 0x10b981 : 0xef4444);
    }
  }

  /**
   * Returns the current 3D target position `[x, y, z]` and quaternion `[qx, qy, qz, qw]`.
   */
  public getTargetPose(): {
    position: [number, number, number];
    quaternion: [number, number, number, number];
    eulerDeg: [number, number, number];
  } {
    const p = this.targetAnchor.position;
    const q = this.targetAnchor.quaternion;
    const e = new THREE.Euler().setFromQuaternion(q, 'XYZ');
    return {
      position: [p.x, p.y, p.z],
      quaternion: [q.x, q.y, q.z, q.w],
      eulerDeg: [e.x * RAD2DEG, e.y * RAD2DEG, e.z * RAD2DEG]
    };
  }

  /**
   * Moves the gripper target to `[x, y, z]` while preserving the current target orientation,
   * and commands the robot via IK and joint motors.
   */
  public setTargetPosition(position: [number, number, number]): IKSolverResult {
    this.targetAnchor.position.set(position[0], position[1], position[2]);
    this.targetAnchor.updateMatrixWorld(true);
    return this._applyAnchorPoseToRobot();
  }

  /**
   * Translates the gripper target by `[dx, dy, dz]` meters.
   */
  public translateTarget(delta: [number, number, number]): IKSolverResult {
    this.targetAnchor.position.x += delta[0];
    this.targetAnchor.position.y += delta[1];
    this.targetAnchor.position.z += delta[2];
    this.targetAnchor.updateMatrixWorld(true);
    return this._applyAnchorPoseToRobot();
  }

  /**
   * Sets the gripper target orientation from Euler angles `[rxDeg, ryDeg, rzDeg]` (in degrees)
   * relative to the top-down grasp reference or world frame, and solves 6-DOF IK.
   */
  public setTargetOrientationEulerDeg(
    eulerDeg: [number, number, number],
    relativeToTopDown: boolean = true
  ): IKSolverResult {
    const qDelta = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(
        eulerDeg[0] * DEG2RAD,
        eulerDeg[1] * DEG2RAD,
        eulerDeg[2] * DEG2RAD,
        'XYZ'
      )
    );
    if (relativeToTopDown) {
      const baseDown = createTopDownQuaternion(0);
      this.targetAnchor.quaternion.copy(baseDown).multiply(qDelta).normalize();
    } else {
      this.targetAnchor.quaternion.copy(qDelta).normalize();
    }
    this.targetAnchor.updateMatrixWorld(true);
    return this._applyAnchorPoseToRobot();
  }

  /**
   * Rotates the current gripper target orientation by `[dRxDeg, dRyDeg, dRzDeg]`.
   */
  public rotateTargetEulerDeg(deltaEulerDeg: [number, number, number]): IKSolverResult {
    const qDelta = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(
        deltaEulerDeg[0] * DEG2RAD,
        deltaEulerDeg[1] * DEG2RAD,
        deltaEulerDeg[2] * DEG2RAD,
        'XYZ'
      )
    );
    this.targetAnchor.quaternion.multiply(qDelta).normalize();
    this.targetAnchor.updateMatrixWorld(true);
    return this._applyAnchorPoseToRobot();
  }

  /**
   * Sets both target position and target orientation quaternion and commands the robot via IK.
   */
  public setTargetPose(
    position: [number, number, number],
    quaternion?: [number, number, number, number] | THREE.Quaternion
  ): IKSolverResult {
    this.targetAnchor.position.set(position[0], position[1], position[2]);
    if (quaternion instanceof THREE.Quaternion) {
      this.targetAnchor.quaternion.copy(quaternion).normalize();
    } else if (Array.isArray(quaternion)) {
      this.targetAnchor.quaternion
        .set(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        .normalize();
    }
    this.targetAnchor.updateMatrixWorld(true);
    return this._applyAnchorPoseToRobot();
  }

  /**
   * Solves Inverse Kinematics for `targetAnchor` and drives the robot's physical joint motors.
   * If `targetAnchor` is outside the reachable workspace, projects to the closest valid
   * reachable position, commands the motors to that closest valid pose, and sets a clear warning.
   */
  private _applyAnchorPoseToRobot(): IKSolverResult {
    const desiredPos = this.targetAnchor.position.clone();
    const desiredQuat = this.targetAnchor.quaternion.clone().normalize();
    const currentAngles = this.robotModel.robotJoints.getCurrentAnglesDeg();

    const shoulderOrigin = new THREE.Vector3(0, 0.26, 0);
    const vecFromShoulder = new THREE.Vector3().subVectors(desiredPos, shoulderOrigin);
    const distFromShoulder = vecFromShoulder.length();
    const maxSafeReach = 0.72;
    const minSafeReach = 0.10;
    const minSafeY = 0.022;

    const isOutOfBounds =
      distFromShoulder > maxSafeReach ||
      distFromShoulder < minSafeReach ||
      desiredPos.y < minSafeY;

    const rawIk = solveInverseKinematics(
      {
        targetPosition: desiredPos,
        targetQuaternion: desiredQuat,
        currentAnglesDeg: currentAngles
      },
      this.robotModel.config
    );

    if (!isOutOfBounds && rawIk.isReachable) {
      this.isTargetReachable = true;
      this.warningMessage = '';
      this._updateRingColor(true);
      this.lastIKResult = rawIk;
      this.motionController.commandJointAnglesDeg(rawIk.targetAnglesDeg);
      return rawIk;
    }

    // Target is unreachable -> compute closest valid reachable position in workspace
    const clampedPos = desiredPos.clone();
    const fromShoulder = new THREE.Vector3().subVectors(clampedPos, shoulderOrigin);
    const len = fromShoulder.length();
    if (len > maxSafeReach) {
      fromShoulder.normalize().multiplyScalar(maxSafeReach);
      clampedPos.copy(shoulderOrigin).add(fromShoulder);
    } else if (len < minSafeReach) {
      if (len < 1e-5) fromShoulder.set(0, 0, minSafeReach);
      else fromShoulder.normalize().multiplyScalar(minSafeReach);
      clampedPos.copy(shoulderOrigin).add(fromShoulder);
    }
    if (clampedPos.y < minSafeY) {
      clampedPos.y = minSafeY;
    }

    const closestIk = solveInverseKinematics(
      {
        targetPosition: clampedPos,
        targetQuaternion: desiredQuat,
        currentAnglesDeg: currentAngles
      },
      this.robotModel.config
    );

    // Command the physical joint motors to the closest valid reachable pose
    this.motionController.commandJointAnglesDeg(closestIk.targetAnglesDeg);

    this.isTargetReachable = false;
    this.warningMessage = `WARNING / ATTENTION : Cible [${desiredPos.x.toFixed(2)}, ${desiredPos.y.toFixed(2)}, ${desiredPos.z.toFixed(2)}] m inaccessible (d=${distFromShoulder.toFixed(2)} m). Arrêt à la posture atteignable la plus proche.`;
    this._updateRingColor(false);

    const unreachableResult: IKSolverResult = {
      ...closestIk,
      success: false,
      status: 'UNREACHABLE',
      isReachable: false,
      message: this.warningMessage
    };
    this.lastIKResult = unreachableResult;
    return unreachableResult;
  }
}
