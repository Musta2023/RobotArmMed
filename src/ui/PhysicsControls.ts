/**
 * @file PhysicsControls.ts
 * Interactive Physics Parameters (Gravity, Mass, Friction, Restitution, Motor Drive)
 * and Surgical Instrument Spawning/Force Injection Panel.
 */

import { PhysicsWorld } from '../physics/PhysicsWorld.js';
import { JointManager } from '../physics/JointManager.js';
import { PhysicsDebugger } from '../physics/PhysicsDebugger.js';
import { InstrumentFactory } from '../objects/InstrumentFactory.js';
import { InstrumentPresetType, PICKUP_STATION_POS } from '../objects/ObjectProperties.js';
import { Environment } from '../scene/Environment.js';
import { GraspManager } from '../manipulation/GraspManager.js';

export class PhysicsControls {
  public container: HTMLElement | null;
  public physicsWorld: PhysicsWorld;
  public jointManager: JointManager;
  public instrumentFactory: InstrumentFactory;
  public environment: Environment;
  public graspManager: GraspManager;
  public physicsDebugger: PhysicsDebugger | null;

  constructor(
    container: HTMLElement | null,
    physicsWorld: PhysicsWorld,
    jointManager: JointManager,
    instrumentFactory: InstrumentFactory,
    environment: Environment,
    graspManager: GraspManager,
    physicsDebugger: PhysicsDebugger | null = null
  ) {
    this.container = container;
    this.physicsWorld = physicsWorld;
    this.jointManager = jointManager;
    this.instrumentFactory = instrumentFactory;
    this.environment = environment;
    this.graspManager = graspManager;
    this.physicsDebugger = physicsDebugger;

    if (this.container) {
      this._render();
      this.refresh();
    }
  }

  private _render(): void {
    if (!this.container) return;
    this.container.innerHTML = '';

    const activeInst = this.instrumentFactory.getActiveInstrument();
    const initMass = activeInst ? activeInst.config.massKg : 0.15;
    const initFriction = activeInst ? activeInst.config.friction : 0.85;
    const initRestitution = activeInst ? activeInst.config.restitution : 0.05;

    // 1. Physics World & Motor Drive Parameters (Collapsible to save vertical space)
    const physSec = document.createElement('section');
    physSec.className = 'panel-section';
    physSec.innerHTML = `
      <details class="phys-params-collapsible">
        <summary class="section-header" style="cursor: pointer; list-style: none;">
          <h2>Paramètres Physiques</h2>
          <span class="badge badge-info">SI</span>
        </summary>
        <div style="margin-top: 6px; display: flex; flex-direction: column; gap: 6px;">
          <div class="param-slider-group">
            <div class="param-label-row">
              <span>Gravité (g)</span>
              <strong id="val-gravity">${this.physicsWorld.gravityY.toFixed(2)} m/s²</strong>
            </div>
            <input type="range" id="slider-gravity" min="-20.0" max="0.0" step="0.1" value="${this.physicsWorld.gravityY}" />
          </div>

          <div class="param-slider-group">
            <div class="param-label-row">
              <span>Masse Objet (m)</span>
              <strong id="val-mass">${initMass.toFixed(2)} kg</strong>
            </div>
            <input type="range" id="slider-mass" min="0.02" max="2.50" step="0.02" value="${initMass}" />
          </div>

          <div class="param-slider-group">
            <div class="param-label-row">
              <span>Friction (μ)</span>
              <strong id="val-friction">${initFriction.toFixed(2)}</strong>
            </div>
            <input type="range" id="slider-friction" min="0.02" max="1.50" step="0.02" value="${initFriction}" />
          </div>

          <div class="param-slider-group">
            <div class="param-label-row">
              <span>Rebond / Restitution (e)</span>
              <strong id="val-restitution">${initRestitution.toFixed(2)}</strong>
            </div>
            <input type="range" id="slider-restitution" min="0.00" max="0.90" step="0.05" value="${initRestitution}" />
          </div>

          <div class="param-slider-group">
            <div class="param-label-row">
              <span>Échelle Couple Moteurs</span>
              <strong id="val-torque-scale">1.00×</strong>
            </div>
            <input type="range" id="slider-torque-scale" min="0.10" max="2.50" step="0.05" value="1.00" />
          </div>

          <div class="param-slider-group">
            <div class="param-label-row">
              <span>Kp / Kd Moteurs</span>
              <strong id="val-stiffness-damping">1.0× / 1.0×</strong>
            </div>
            <div class="dual-slider-row">
              <input type="range" id="slider-stiffness" min="0.2" max="2.5" step="0.1" value="1.0" title="Kp" />
              <input type="range" id="slider-damping" min="0.2" max="2.5" step="0.1" value="1.0" title="Kd" />
            </div>
          </div>

          <div class="param-slider-group">
            <div class="param-label-row">
              <span>Vitesse Simulation</span>
              <strong id="val-sim-speed">${this.physicsWorld.simulationSpeed.toFixed(2)}×</strong>
            </div>
            <input type="range" id="slider-sim-speed" min="0.25" max="2.50" step="0.25" value="${this.physicsWorld.simulationSpeed}" />
          </div>
        </div>
      </details>
    `;

    const gravSlider = physSec.querySelector('#slider-gravity') as HTMLInputElement;
    const gravVal = physSec.querySelector('#val-gravity') as HTMLElement;
    gravSlider.addEventListener('input', () => {
      const g = parseFloat(gravSlider.value);
      this.physicsWorld.setGravity(g);
      gravVal.textContent = `${g.toFixed(2)} m/s²`;
    });

    const massSlider = physSec.querySelector('#slider-mass') as HTMLInputElement;
    const massVal = physSec.querySelector('#val-mass') as HTMLElement;
    massSlider.addEventListener('input', () => {
      const m = parseFloat(massSlider.value);
      const inst = this.instrumentFactory.getActiveInstrument();
      if (inst) {
        this.instrumentFactory.setInstrumentMass(inst.config.id, m);
      }
      massVal.textContent = `${m.toFixed(2)} kg`;
    });

    const fricSlider = physSec.querySelector('#slider-friction') as HTMLInputElement;
    const fricVal = physSec.querySelector('#val-friction') as HTMLElement;
    const restSlider = physSec.querySelector('#slider-restitution') as HTMLInputElement;
    const restVal = physSec.querySelector('#val-restitution') as HTMLElement;

    const updateMaterial = () => {
      const mu = parseFloat(fricSlider.value);
      const e = parseFloat(restSlider.value);
      const inst = this.instrumentFactory.getActiveInstrument();
      if (inst) {
        this.instrumentFactory.setInstrumentMaterial(inst.config.id, mu, e);
      }
      fricVal.textContent = mu.toFixed(2);
      restVal.textContent = e.toFixed(2);
    };

    fricSlider.addEventListener('input', updateMaterial);
    restSlider.addEventListener('input', updateMaterial);

    const trqSlider = physSec.querySelector('#slider-torque-scale') as HTMLInputElement;
    const trqVal = physSec.querySelector('#val-torque-scale') as HTMLElement;
    const kpSlider = physSec.querySelector('#slider-stiffness') as HTMLInputElement;
    const kdSlider = physSec.querySelector('#slider-damping') as HTMLInputElement;
    const kpKdVal = physSec.querySelector('#val-stiffness-damping') as HTMLElement;

    const updateDriveScales = () => {
      const trq = parseFloat(trqSlider.value);
      const kp = parseFloat(kpSlider.value);
      const kd = parseFloat(kdSlider.value);
      this.jointManager.globalMaxTorqueScale = trq;
      this.jointManager.globalStiffnessScale = kp;
      this.jointManager.globalDampingScale = kd;
      trqVal.textContent = `${trq.toFixed(2)}×`;
      kpKdVal.textContent = `${kp.toFixed(1)}× / ${kd.toFixed(1)}×`;
    };

    trqSlider.addEventListener('input', updateDriveScales);
    kpSlider.addEventListener('input', updateDriveScales);
    kdSlider.addEventListener('input', updateDriveScales);

    const speedSlider = physSec.querySelector('#slider-sim-speed') as HTMLInputElement;
    const speedVal = physSec.querySelector('#val-sim-speed') as HTMLElement;
    speedSlider.addEventListener('input', () => {
      const s = parseFloat(speedSlider.value);
      this.physicsWorld.simulationSpeed = s;
      speedVal.textContent = `${s.toFixed(2)}×`;
    });

    // 2. Surgical Object Spawning, Forces & Obstacle Management
    const objSec = document.createElement('section');
    objSec.className = 'panel-section';
    objSec.innerHTML = `
      <div class="section-header">
        <h2>Objets & Obstacles</h2>
        <span class="badge badge-info" id="obj-count-badge">${this.instrumentFactory.instruments.size} ACTIF</span>
      </div>

      <div class="select-row">
        <select id="select-instrument-preset" class="control-select">
          <option value="rectangular_instrument">Cube Inox (0.15 kg)</option>
          <option value="cylindrical_instrument">Trocart (0.18 kg)</option>
          <option value="surgical_tool">Pince chirurgicale (0.22 kg)</option>
          <option value="small_metallic_object">Micro-clip Titane (0.10 kg)</option>
        </select>
        <button type="button" class="btn btn-primary" id="btn-spawn-inst">Spawn</button>
      </div>

      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-drop-gravity">Drop Test</button>
        <button type="button" class="btn btn-secondary" id="btn-push-impulse">Impulsion (+1.5N)</button>
      </div>

      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-toggle-obstacle">Obstacle On/Off</button>
        <button type="button" class="btn btn-secondary" id="btn-reset-objects">Reset Objets</button>
      </div>

      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-toggle-phys-debug">Debug Collisions</button>
        <span class="badge badge-info" id="phys-debug-badge" style="display:flex;align-items:center;justify-content:center;">OFF</span>
      </div>
    `;

    const presetSelect = objSec.querySelector('#select-instrument-preset') as HTMLSelectElement;

    objSec.querySelector('#btn-spawn-inst')?.addEventListener('click', () => {
      const preset = presetSelect.value as InstrumentPresetType;
      this.graspManager.releaseGrasp();
      for (const id of Array.from(this.instrumentFactory.instruments.keys())) {
        this.instrumentFactory.removeInstrument(id);
      }
      const spawned = this.instrumentFactory.spawnInstrument(preset);
      this.instrumentFactory.activeInstrumentId = spawned.config.id;
      massSlider.value = String(spawned.config.massKg);
      massVal.textContent = `${spawned.config.massKg.toFixed(2)} kg`;
      fricSlider.value = String(spawned.config.friction);
      fricVal.textContent = spawned.config.friction.toFixed(2);
      restSlider.value = String(spawned.config.restitution);
      restVal.textContent = spawned.config.restitution.toFixed(2);
      this.refresh();
    });

    objSec.querySelector('#btn-drop-gravity')?.addEventListener('click', () => {
      this.graspManager.releaseGrasp();
      const inst = this.instrumentFactory.getActiveInstrument();
      if (inst) {
        this.instrumentFactory.setInstrumentPose(inst.config.id, [
          PICKUP_STATION_POS[0],
          0.28,
          PICKUP_STATION_POS[2]
        ]);
      }
    });

    objSec.querySelector('#btn-push-impulse')?.addEventListener('click', () => {
      const inst = this.instrumentFactory.getActiveInstrument();
      if (inst) {
        this.instrumentFactory.bodyManager.applyExternalImpulse(inst.config.id, [0.06, 0.02, 0.0]);
      }
    });

    objSec.querySelector('#btn-toggle-obstacle')?.addEventListener('click', () => {
      const next = !this.environment.obstacleEnabled;
      this.environment.setObstacleEnabled(next);
    });

    objSec.querySelector('#btn-reset-objects')?.addEventListener('click', () => {
      this.graspManager.releaseGrasp();
      this.instrumentFactory.resetAllInstruments();
    });

    objSec.querySelector('#btn-toggle-phys-debug')?.addEventListener('click', () => {
      if (!this.physicsDebugger) return;
      const next = !this.physicsDebugger.enabled;
      this.physicsDebugger.setEnabled(next, true);
      const badge = objSec.querySelector('#phys-debug-badge');
      if (badge) {
        badge.textContent = next ? 'DEBUG: ON' : 'DEBUG: OFF';
      }
    });

    this.container.appendChild(physSec);
    this.container.appendChild(objSec);
  }

  public refresh(): void {
    if (!this.container) return;
    const countBadge = this.container.querySelector('#obj-count-badge');
    if (countBadge) {
      countBadge.textContent = `${this.instrumentFactory.instruments.size} ACTIVE`;
    }
  }
}
