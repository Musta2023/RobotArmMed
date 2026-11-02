/**
 * @file CameraManager.ts
 * Manages the primary OrbitControls viewport camera AND the robot-mounted
 * End-Effector Wrist Camera (with field-of-view visualization, Picture-in-Picture
 * preview rendering, and camera-frame object pose / RGB-D interface).
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RobotModel } from '../robot/RobotModel.js';
import { InstrumentFactory } from '../objects/InstrumentFactory.js';

export interface CameraSensorObservation {
  cameraWorldPosition: [number, number, number];
  cameraWorldQuaternion: [number, number, number, number];
  fovDeg: number;
  visibleObjectsInCameraFrame: Array<{
    id: string;
    name: string;
    positionInCameraFrame: [number, number, number];
    distanceMeters: number;
  }>;
}

export class CameraManager {
  public mainCamera: THREE.PerspectiveCamera;
  public orbitControls: OrbitControls | null = null;
  public robotModel: RobotModel;
  public instrumentFactory: InstrumentFactory;
  public pipRenderer: THREE.WebGLRenderer | null = null;
  public pipEnabled: boolean = true;

  constructor(
    robotModel: RobotModel,
    instrumentFactory: InstrumentFactory,
    mainDomElement?: HTMLElement,
    pipContainer?: HTMLElement | null
  ) {
    this.robotModel = robotModel;
    this.instrumentFactory = instrumentFactory;

    this.mainCamera = new THREE.PerspectiveCamera(44, 16 / 9, 0.01, 30);
    this.resetMainCamera();

    if (mainDomElement) {
      this.orbitControls = new OrbitControls(this.mainCamera, mainDomElement);
      this.orbitControls.enableDamping = true;
      this.orbitControls.dampingFactor = 0.06;
      this.orbitControls.maxPolarAngle = Math.PI * 0.49;
      this.orbitControls.minDistance = 0.25;
      this.orbitControls.maxDistance = 3.5;
      this.orbitControls.target.set(0, 0.20, 0.12);
      this.orbitControls.update();
    }

    if (typeof window !== 'undefined' && pipContainer) {
      this.pipRenderer = new THREE.WebGLRenderer({ antialias: true });
      this.pipRenderer.setSize(240, 150);
      this.pipRenderer.setPixelRatio(1);
      pipContainer.innerHTML = '';
      pipContainer.appendChild(this.pipRenderer.domElement);
    }
  }

  public resetMainCamera(): void {
    this.mainCamera.position.set(0.55, 0.68, 0.92);
    if (this.orbitControls) {
      this.orbitControls.target.set(0, 0.20, 0.12);
      this.orbitControls.update();
    } else {
      this.mainCamera.lookAt(0, 0.20, 0.12);
    }
  }

  /**
   * Toggles the 3D field-of-view frustum wireframe for the end-effector wrist camera.
   */
  public setWristCameraFOVVisible(visible: boolean): void {
    this.robotModel.wristCameraHelper.visible = visible;
    if (visible) {
      this.robotModel.wristCameraHelper.update();
    }
  }

  /**
   * Renders the End-Effector Wrist Camera feed into the Picture-in-Picture preview canvas.
   */
  public renderWristCameraPreview(scene: THREE.Scene): void {
    if (!this.pipRenderer || !this.pipEnabled) return;
    const prevHelperVis = this.robotModel.wristCameraHelper.visible;
    this.robotModel.wristCameraHelper.visible = false;
    this.pipRenderer.render(scene, this.robotModel.wristCamera);
    this.robotModel.wristCameraHelper.visible = prevHelperVis;
  }

  /**
   * Captures object positions and orientations transformed into the End-Effector
   * Wrist Camera's local coordinate frame (preparing for RGB-D / VLA vision policies).
   */
  public captureCameraObservation(): CameraSensorObservation {
    const cam = this.robotModel.wristCamera;
    cam.updateMatrixWorld(true);

    const camPos = new THREE.Vector3();
    const camQuat = new THREE.Quaternion();
    cam.matrixWorld.decompose(camPos, camQuat, new THREE.Vector3());

    const invCamMatrix = cam.matrixWorld.clone().invert();
    const visibleObjectsInCameraFrame: CameraSensorObservation['visibleObjectsInCameraFrame'] = [];

    for (const [, inst] of this.instrumentFactory.instruments) {
      const t = inst.managedBody.body.translation();
      const worldPos = new THREE.Vector3(t.x, t.y, t.z);
      const localCamPos = worldPos.clone().applyMatrix4(invCamMatrix);
      const dist = worldPos.distanceTo(camPos);

      visibleObjectsInCameraFrame.push({
        id: inst.config.id,
        name: inst.config.name,
        positionInCameraFrame: [localCamPos.x, localCamPos.y, localCamPos.z],
        distanceMeters: dist
      });
    }

    return {
      cameraWorldPosition: [camPos.x, camPos.y, camPos.z],
      cameraWorldQuaternion: [camQuat.x, camQuat.y, camQuat.z, camQuat.w],
      fovDeg: cam.fov,
      visibleObjectsInCameraFrame
    };
  }
}
