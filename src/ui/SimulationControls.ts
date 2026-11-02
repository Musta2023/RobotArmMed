/**
 * @file SimulationControls.ts
 * Simulation Transport Controls (Play/Pause/Step/Reset), 9-State Pick-and-Place Task
 * Execution, Failure Detection & Recovery JSON Inspector, and AI Dataset Recorder.
 */

import { PickAndPlace, PickAndPlaceState } from '../manipulation/PickAndPlace.js';
import { EpisodeRecorder } from '../ai/EpisodeRecorder.js';
import { RobotModel } from '../robot/RobotModel.js';

export interface SimulationTransportCallbacks {
  isPlaying: () => boolean;
  setPlaying: (playing: boolean) => void;
  stepOnce: () => void;
  resetAll: () => void;
  resetRobotOnly: () => void;
  resetObjectsOnly: () => void;
}

const ORDERED_TASK_STATES: PickAndPlaceState[] = [
  'IDLE',
  'APPROACH',
  'GRASP',
  'GRASP_VERIFICATION',
  'LIFT',
  'TRANSPORT',
  'PLACE',
  'VERIFY_PLACEMENT',
  'COMPLETE'
];

export class SimulationControls {
  public container: HTMLElement | null;
  public pickAndPlace: PickAndPlace;
  public episodeRecorder: EpisodeRecorder;
  public robotModel: RobotModel;
  public callbacks: SimulationTransportCallbacks;

  private stateChipRefs: Map<PickAndPlaceState, HTMLElement> = new Map();

  constructor(
    container: HTMLElement | null,
    pickAndPlace: PickAndPlace,
    episodeRecorder: EpisodeRecorder,
    robotModel: RobotModel,
    callbacks: SimulationTransportCallbacks
  ) {
    this.container = container;
    this.pickAndPlace = pickAndPlace;
    this.episodeRecorder = episodeRecorder;
    this.robotModel = robotModel;
    this.callbacks = callbacks;

    if (this.container) {
      this._render();
      this.refresh();
    }
  }

  private _render(): void {
    if (!this.container) return;
    this.container.innerHTML = '';

    // 1. Simulation Transport & Reset Controls
    const simSec = document.createElement('section');
    simSec.className = 'panel-section';
    simSec.innerHTML = `
      <div class="section-header">
        <h2>Simulation</h2>
        <span class="badge badge-valid" id="sim-run-badge">RUNNING</span>
      </div>
      <div class="button-grid-3">
        <button type="button" class="btn btn-primary" id="btn-sim-play-pause">Pause</button>
        <button type="button" class="btn btn-secondary" id="btn-sim-step">Pas (1/60s)</button>
        <button type="button" class="btn btn-secondary" id="btn-sim-reset-all">Reset All</button>
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-sim-reset-robot">Reset Robot</button>
        <button type="button" class="btn btn-secondary" id="btn-toggle-cam-frustum">FOV Cam</button>
      </div>
    `;

    simSec.querySelector('#btn-sim-play-pause')?.addEventListener('click', () => {
      this.callbacks.setPlaying(!this.callbacks.isPlaying());
      this.refresh();
    });

    simSec.querySelector('#btn-sim-step')?.addEventListener('click', () => {
      this.callbacks.setPlaying(false);
      this.callbacks.stepOnce();
      this.refresh();
    });

    simSec.querySelector('#btn-sim-reset-all')?.addEventListener('click', () => {
      this.callbacks.resetAll();
      this.refresh();
    });

    simSec.querySelector('#btn-sim-reset-robot')?.addEventListener('click', () => {
      this.callbacks.resetRobotOnly();
      this.refresh();
    });

    simSec.querySelector('#btn-toggle-cam-frustum')?.addEventListener('click', () => {
      this.robotModel.wristCameraHelper.visible = !this.robotModel.wristCameraHelper.visible;
    });

    // 2. 9-State Pick and Place Manipulation State Machine
    const taskSec = document.createElement('section');
    taskSec.className = 'panel-section';
    taskSec.innerHTML = `
      <div class="section-header">
        <h2>Tâche Pick & Place</h2>
        <span class="badge badge-info" id="pnp-state-badge">IDLE</span>
      </div>
      <div class="state-pipeline-grid" id="pnp-pipeline-grid"></div>
      <div class="feedback-banner status-info" id="pnp-status-message" style="font-size: 10px; padding: 4px 6px;">
        Prêt. Cliquez sur Démarrer.
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-primary" id="btn-pnp-start">Démarrer</button>
        <button type="button" class="btn btn-warning" id="btn-pnp-fail-test">Tester Échec</button>
      </div>
      <button type="button" class="btn btn-accent btn-full" id="btn-pnp-recover" style="display:none; margin-top: 4px;">
        Autoriser Récupération
      </button>
      <div class="json-failure-box" id="json-failure-container" style="display:none; margin-top: 4px;">
        <div class="json-box-header">Rapport d'échec</div>
        <pre id="json-failure-pre" style="max-height: 80px; overflow-y: auto;"></pre>
      </div>
    `;

    const pipelineGrid = taskSec.querySelector('#pnp-pipeline-grid') as HTMLElement;
    this.stateChipRefs.clear();

    ORDERED_TASK_STATES.forEach((st, idx) => {
      const chip = document.createElement('div');
      chip.className = 'state-chip';
      chip.textContent = `${idx + 1}. ${st}`;
      this.stateChipRefs.set(st, chip);
      pipelineGrid.appendChild(chip);
    });

    taskSec.querySelector('#btn-pnp-start')?.addEventListener('click', () => {
      this.callbacks.setPlaying(true);
      if (!this.episodeRecorder.isRecording) {
        this.episodeRecorder.startRecording('surgical_pick_and_place');
      }
      this.pickAndPlace.start();
      this.refresh();
    });

    taskSec.querySelector('#btn-pnp-fail-test')?.addEventListener('click', () => {
      this.callbacks.setPlaying(true);
      this.pickAndPlace.injectIntentionalGraspFailure(true);
      this.pickAndPlace.start();
      this.refresh();
    });

    taskSec.querySelector('#btn-pnp-recover')?.addEventListener('click', () => {
      this.callbacks.setPlaying(true);
      this.pickAndPlace.authorizeAndRecover();
      this.refresh();
    });

    // 3. AI & Imitation Learning Dataset Recorder
    const aiSec = document.createElement('section');
    aiSec.className = 'panel-section';
    aiSec.innerHTML = `
      <div class="section-header">
        <h2>Enregistreur IA</h2>
        <span class="badge badge-info" id="ai-rec-badge">STANDBY</span>
      </div>
      <div class="telemetry-kv-grid">
        <div><span>Épisode</span><strong id="ai-ep-id">ep_001</strong></div>
        <div><span>Échantillons</span><strong id="ai-step-count">0</strong></div>
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-ai-toggle-rec">Enregistrer</button>
        <button type="button" class="btn btn-warning" id="btn-ai-mark-failed">Stop (Échec)</button>
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-accent" id="btn-ai-export-json">Export JSON</button>
        <button type="button" class="btn btn-secondary" id="btn-ai-export-csv">Export CSV</button>
      </div>
    `;

    aiSec.querySelector('#btn-ai-toggle-rec')?.addEventListener('click', () => {
      if (this.episodeRecorder.isRecording) {
        this.episodeRecorder.stopRecording('success');
      } else {
        this.episodeRecorder.startRecording('surgical_demo');
      }
      this.refresh();
    });

    aiSec.querySelector('#btn-ai-mark-failed')?.addEventListener('click', () => {
      this.episodeRecorder.stopRecording('failure');
      this.refresh();
    });

    aiSec.querySelector('#btn-ai-export-json')?.addEventListener('click', () => {
      if (this.episodeRecorder.isRecording) {
        this.episodeRecorder.stopRecording();
      }
      const jsonStr = this.episodeRecorder.exportJSON();
      if (typeof Blob !== 'undefined' && typeof URL !== 'undefined') {
        const blob = new Blob([jsonStr], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `surgical_action_ai_dataset_${Date.now()}.json`;
        a.click();
        URL.revokeObjectURL(url);
      }
      this.refresh();
    });

    aiSec.querySelector('#btn-ai-export-csv')?.addEventListener('click', () => {
      if (this.episodeRecorder.isRecording) {
        this.episodeRecorder.stopRecording();
      }
      const csvStr = this.episodeRecorder.exportCSV();
      if (typeof Blob !== 'undefined' && typeof URL !== 'undefined') {
        const blob = new Blob([csvStr], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `surgical_action_ai_dataset_${Date.now()}.csv`;
        a.click();
        URL.revokeObjectURL(url);
      }
      this.refresh();
    });

    this.container.appendChild(simSec);
    this.container.appendChild(taskSec);
    this.container.appendChild(aiSec);
  }

  public refresh(): void {
    if (!this.container) return;

    const playing = this.callbacks.isPlaying();
    const runBadge = this.container.querySelector('#sim-run-badge');
    const playPauseBtn = this.container.querySelector('#btn-sim-play-pause');
    if (runBadge) {
      runBadge.textContent = playing ? 'RUNNING' : 'PAUSED';
      runBadge.className = playing ? 'badge badge-valid' : 'badge badge-warning';
    }
    if (playPauseBtn) {
      playPauseBtn.textContent = playing ? 'Pause Sim' : 'Play Sim';
    }

    const curState = this.pickAndPlace.state;
    const stateBadge = this.container.querySelector('#pnp-state-badge');
    const statusMsg = this.container.querySelector('#pnp-status-message');
    const recoverBtn = this.container.querySelector('#btn-pnp-recover') as HTMLElement | null;
    const jsonBox = this.container.querySelector('#json-failure-container') as HTMLElement | null;
    const jsonPre = this.container.querySelector('#json-failure-pre');

    if (stateBadge) {
      stateBadge.textContent = curState;
      stateBadge.className =
        curState === 'COMPLETE'
          ? 'badge badge-valid'
          : curState === 'FAILED'
          ? 'badge badge-danger'
          : 'badge badge-info';
    }

    if (statusMsg) {
      statusMsg.textContent = this.pickAndPlace.statusMessage;
      statusMsg.className =
        curState === 'COMPLETE'
          ? 'feedback-banner status-valid'
          : curState === 'FAILED'
          ? 'feedback-banner status-danger'
          : 'feedback-banner status-info';
    }

    const curIdx = ORDERED_TASK_STATES.indexOf(curState);
    ORDERED_TASK_STATES.forEach((st, idx) => {
      const chip = this.stateChipRefs.get(st);
      if (!chip) return;
      chip.className = 'state-chip';
      if (st === curState) {
        chip.classList.add('active');
      } else if (curIdx > idx || curState === 'COMPLETE') {
        chip.classList.add('completed');
      }
    });

    const failureReport =
      this.pickAndPlace.failureDetector.activeFailure ??
      this.pickAndPlace.lastOrchestratorResult;

    if (curState === 'FAILED' && failureReport) {
      if (recoverBtn) recoverBtn.style.display = 'block';
      if (jsonBox && jsonPre) {
        jsonBox.style.display = 'block';
        jsonPre.textContent = JSON.stringify(failureReport, null, 2);
      }
    } else {
      if (recoverBtn) recoverBtn.style.display = 'none';
      if (jsonBox) jsonBox.style.display = 'none';
    }

    const recBadge = this.container.querySelector('#ai-rec-badge');
    const epIdEl = this.container.querySelector('#ai-ep-id');
    const stepCountEl = this.container.querySelector('#ai-step-count');
    const recBtn = this.container.querySelector('#btn-ai-toggle-rec');

    if (recBadge) {
      recBadge.textContent = this.episodeRecorder.isRecording ? 'RECORDING' : 'STANDBY';
      recBadge.className = this.episodeRecorder.isRecording ? 'badge badge-danger' : 'badge badge-info';
    }
    if (epIdEl) {
      epIdEl.textContent = this.episodeRecorder.episodeId;
    }
    if (stepCountEl) {
      stepCountEl.textContent = `${this.episodeRecorder.frames.length} samples`;
    }
    if (recBtn) {
      recBtn.textContent = this.episodeRecorder.isRecording ? 'Stop Recording' : 'Start Recording';
    }
  }
}
