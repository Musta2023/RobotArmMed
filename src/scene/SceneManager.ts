/**
 * @file SceneManager.ts
 * Manages the Three.js Scene, WebGLRenderer, Lighting, and CameraManager
 * for the Surgical Action AI Digital Twin.
 */

import * as THREE from 'three';
import { Lighting } from './Lighting.js';
import { CameraManager } from './CameraManager.js';
import { RobotModel } from '../robot/RobotModel.js';
import { InstrumentFactory } from '../objects/InstrumentFactory.js';

export class SceneManager {
  public container: HTMLElement | null;
  public scene: THREE.Scene;
  public renderer: THREE.WebGLRenderer | null = null;
  public lighting: Lighting;
  public cameraManager: CameraManager;

  constructor(
    container: HTMLElement | null,
    pipContainer: HTMLElement | null,
    robotModel: RobotModel,
    instrumentFactory: InstrumentFactory
  ) {
    this.container = container;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b0d12);
    this.scene.fog = new THREE.FogExp2(0x0b0d12, 0.22);

    this.lighting = new Lighting(this.scene);

    let mainCanvasElement: HTMLElement | undefined;

    if (typeof window !== 'undefined' && container) {
      const w = Math.max(container.clientWidth, 640);
      const h = Math.max(container.clientHeight, 480);

      this.renderer = new THREE.WebGLRenderer({
        antialias: true,
        powerPreference: 'high-performance'
      });
      this.renderer.setSize(w, h);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.renderer.toneMappingExposure = 1.1;

      container.innerHTML = '';
      container.appendChild(this.renderer.domElement);
      mainCanvasElement = this.renderer.domElement;
    }

    this.cameraManager = new CameraManager(
      robotModel,
      instrumentFactory,
      mainCanvasElement,
      pipContainer
    );

    this.scene.add(robotModel.wristCameraHelper);

    if (typeof window !== 'undefined' && container) {
      this.handleResize();
      window.addEventListener('resize', () => this.handleResize());
    }
  }

  public handleResize(): void {
    if (!this.container || !this.renderer) return;
    const w = Math.max(this.container.clientWidth, 200);
    const h = Math.max(this.container.clientHeight, 200);
    this.cameraManager.mainCamera.aspect = w / h;
    this.cameraManager.mainCamera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  public render(): void {
    if (this.cameraManager.orbitControls) {
      this.cameraManager.orbitControls.update();
    }
    if (this.renderer) {
      this.renderer.render(this.scene, this.cameraManager.mainCamera);
    }
    this.cameraManager.renderWristCameraPreview(this.scene);
  }
}
