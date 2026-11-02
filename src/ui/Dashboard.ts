/**
 * @file Dashboard.ts
 * Coordonnateur maître de la télémétrie et de l'interface utilisateur pour le jumeau numérique Surgical Action AI.
 * Synchronise la barre supérieure de télémétrie, les panneaux latéraux d'ingénierie et les sous-composants.
 */

import { PhysicsWorld } from '../physics/PhysicsWorld.js';
import { CollisionManager } from '../physics/CollisionManager.js';
import { RobotModel } from '../robot/RobotModel.js';
import { InstrumentFactory } from '../objects/InstrumentFactory.js';
import { PickAndPlace } from '../manipulation/PickAndPlace.js';
import { RobotControls } from './RobotControls.js';
import { PhysicsControls } from './PhysicsControls.js';
import { SimulationControls } from './SimulationControls.js';

export interface DashboardConfig {
  physicsWorld: PhysicsWorld;
  collisionManager: CollisionManager;
  robotModel: RobotModel;
  instrumentFactory: InstrumentFactory;
  pickAndPlace: PickAndPlace;
  robotControls: RobotControls;
  physicsControls: PhysicsControls;
  simulationControls: SimulationControls;
}

export class Dashboard {
  public config: DashboardConfig;
  public fps: number = 60.0;
  private lastFrameTimeMs: number = 0;

  constructor(config: DashboardConfig) {
    this.config = config;
  }

  /**
   * Met à jour tous les indicateurs télémétriques en temps réel et rafraîchit les sous-panneaux.
   */
  public update(nowMs: number = performance.now()): void {
    if (this.lastFrameTimeMs > 0) {
      const dtMs = Math.max(1, nowMs - this.lastFrameTimeMs);
      const instFps = 1000 / dtMs;
      this.fps = this.fps * 0.9 + instFps * 0.1;
    }
    this.lastFrameTimeMs = nowMs;

    this.config.robotControls.refresh();
    this.config.physicsControls.refresh();
    this.config.simulationControls.refresh();

    if (typeof document === 'undefined') return;

    const snap = this.config.robotModel.state.latestSnapshot;
    const tcpPos = snap.endEffector.position;
    const tcpEuler = snap.endEffector.eulerDeg;

    const simTimeEl = document.getElementById('telemetry-sim-time');
    const dtEl = document.getElementById('telemetry-timestep');
    const fpsEl = document.getElementById('telemetry-fps');
    const tcpEl = document.getElementById('telemetry-tcp-pose');
    const objPoseEl = document.getElementById('telemetry-obj-pose');
    const colEl = document.getElementById('telemetry-collision');
    const stateEl = document.getElementById('telemetry-task-state');

    if (simTimeEl) {
      simTimeEl.textContent = `${this.config.physicsWorld.simulationTime.toFixed(2)} s`;
    }
    if (dtEl) {
      dtEl.textContent = `${(this.config.physicsWorld.fixedTimestep * 1000).toFixed(2)} ms (60 Hz)`;
    }
    if (fpsEl) {
      fpsEl.textContent = `${this.fps.toFixed(1)} FPS`;
      if (this.fps >= 50) {
        fpsEl.style.color = '#34d399';
      } else if (this.fps >= 30) {
        fpsEl.style.color = '#fbbf24';
      } else {
        fpsEl.style.color = '#f87171';
      }
    }
    if (tcpEl) {
      tcpEl.innerHTML = `<span class="val-xyz">[${tcpPos[0].toFixed(3)}, ${tcpPos[1].toFixed(3)}, ${tcpPos[2].toFixed(3)}]</span> <span class="val-unit">m</span> <span class="val-sep">·</span> <span class="val-rpy">RPY [${tcpEuler[0].toFixed(0)}°, ${tcpEuler[1].toFixed(0)}°, ${tcpEuler[2].toFixed(0)}°]</span>`;
      tcpEl.title = `Pose TCP Monde : [${tcpPos[0].toFixed(4)}, ${tcpPos[1].toFixed(4)}, ${tcpPos[2].toFixed(4)}] m | RPY : [${tcpEuler[0].toFixed(1)}°, ${tcpEuler[1].toFixed(1)}°, ${tcpEuler[2].toFixed(1)}°]`;
    }

    const activeInst = this.config.instrumentFactory.getActiveInstrument();
    if (objPoseEl && activeInst) {
      const pose = this.config.instrumentFactory.getInstrumentWorldPose(activeInst.config.id);
      const cleanName = activeInst.config.name
        .replace('Surgical Calibration Cube ', '')
        .replace('Surgical Instrument ', '');
      objPoseEl.textContent = `${cleanName} (${(activeInst.config.massKg * 1000).toFixed(0)}g)`;
      if (pose) {
        objPoseEl.title = `${activeInst.config.name} (${activeInst.config.massKg.toFixed(2)} kg) @ [${pose.position.x.toFixed(3)}, ${pose.position.y.toFixed(3)}, ${pose.position.z.toFixed(3)}] m`;
      }
    }

    const prohibited = this.config.collisionManager.prohibitedCollisions;
    const contacts = this.config.collisionManager.activeContacts;
    if (colEl) {
      if (prohibited.length > 0) {
        colEl.textContent = 'COLLISION';
        colEl.className = 'telemetry-pill pill-danger';
        colEl.title = `Collision interdite détectée : ${prohibited[0].bodyIdA} ↔ ${prohibited[0].bodyIdB}`;
      } else {
        colEl.textContent = contacts.length > 0 ? `AUCUNE (${contacts.length})` : 'AUCUNE';
        colEl.className = 'telemetry-pill pill-valid';
        colEl.title = `Sécurité collision : ${contacts.length} contacts physiques actifs`;
      }
    }

    if (stateEl) {
      const stateMapFr: Record<string, string> = {
        IDLE: 'ATTENTE',
        APPROACH: 'APPROCHE',
        GRASP: 'PRISE',
        GRASP_VERIFICATION: 'VÉRIF. PRISE',
        LIFT: 'LEVAGE',
        TRANSPORT: 'TRANSPORT',
        PLACE: 'DÉPOSE',
        VERIFY_PLACEMENT: 'VÉRIF. DÉPOSE',
        COMPLETE: 'TERMINÉ',
        FAILED: 'ÉCHEC'
      };
      const curState = this.config.pickAndPlace.state;
      stateEl.textContent = stateMapFr[curState] ?? curState;
      stateEl.title = `État Pick & Place : ${curState}`;
    }
  }
}
