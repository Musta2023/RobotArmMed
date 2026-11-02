/**
 * @file Lighting.ts
 * High-contrast NVIDIA Isaac Sim-inspired lighting setup for the Three.js Digital Twin.
 */

import * as THREE from 'three';

export class Lighting {
  public scene: THREE.Scene;
  public keyLight!: THREE.DirectionalLight;

  constructor(scene: THREE.Scene) {
    this.scene = scene;
    this._setup();
  }

  private _setup(): void {
    const hemi = new THREE.HemisphereLight(0xf8fafc, 0x1e293b, 0.9);
    hemi.position.set(0, 3.0, 0);
    this.scene.add(hemi);

    this.keyLight = new THREE.DirectionalLight(0xffffff, 1.55);
    this.keyLight.position.set(0.95, 2.2, 1.45);
    this.keyLight.castShadow = true;
    this.keyLight.shadow.mapSize.width = 2048;
    this.keyLight.shadow.mapSize.height = 2048;
    this.keyLight.shadow.camera.near = 0.2;
    this.keyLight.shadow.camera.far = 5.0;
    const d = 0.95;
    this.keyLight.shadow.camera.left = -d;
    this.keyLight.shadow.camera.right = d;
    this.keyLight.shadow.camera.top = d;
    this.keyLight.shadow.camera.bottom = -d;
    this.keyLight.shadow.bias = -0.0004;
    this.scene.add(this.keyLight);

    const rimLight = new THREE.DirectionalLight(0x34d399, 0.45);
    rimLight.position.set(-1.2, 1.5, -1.0);
    this.scene.add(rimLight);

    const warmFloorBounce = new THREE.PointLight(0xf59e0b, 0.35, 1.8);
    warmFloorBounce.position.set(0, 0.45, 0.35);
    this.scene.add(warmFloorBounce);
  }
}
