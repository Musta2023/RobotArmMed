/**
 * @file Dashboard.ts
 * Master Telemetry & UI Coordinator for the Surgical Action AI Digital Twin.
 * Synchronizes top-bar telemetry, left/right engineering docks, and sub-panels.
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
   * Updates all real-time telemetry indicators and refreshes sub-panels.
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
      dtEl.textContent = `${(this.config.physicsWorld.fixedTimestep * 1000).toFixed(2)} ms (${Math.round(1 / this.config.physicsWorld.fixedTimestep)} Hz)`;
    }
    if (fpsEl) {
      fpsEl.textContent = `${this.fps.toFixed(1)} FPS`;
    }
    if (tcpEl) {
      tcpEl.textContent = `[${tcpPos[0].toFixed(3)}, ${tcpPos[1].toFixed(3)}, ${tcpPos[2].toFixed(3)}] m | RPY [${tcpEuler[0].toFixed(0)}°, ${tcpEuler[1].toFixed(0)}°, ${tcpEuler[2].toFixed(0)}°]`;
    }

    const activeInst = this.config.instrumentFactory.getActiveInstrument();
    if (objPoseEl && activeInst) {
      const pose = this.config.instrumentFactory.getInstrumentWorldPose(activeInst.config.id);
      if (pose) {
        objPoseEl.textContent = `${activeInst.config.name} (${activeInst.config.massKg.toFixed(2)} kg) @ [${pose.position.x.toFixed(3)}, ${pose.position.y.toFixed(3)}, ${pose.position.z.toFixed(3)}] m`;
      }
    }

    const prohibited = this.config.collisionManager.prohibitedCollisions;
    const contacts = this.config.collisionManager.activeContacts;
    if (colEl) {
      if (prohibited.length > 0) {
        colEl.textContent = `COLLISION (${prohibited[0].bodyIdA} ↔ ${prohibited[0].bodyIdB})`;
        colEl.className = 'telemetry-pill pill-danger';
      } else {
        colEl.textContent = `CLEAR (${contacts.length} contacts)`;
        colEl.className = 'telemetry-pill pill-valid';
      }
    }

    if (stateEl) {
      stateEl.textContent = this.config.pickAndPlace.state;
    }
  }
}
