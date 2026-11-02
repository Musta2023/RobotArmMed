/**
 * @file SimulationControls.ts
 * Contrôles de transport de la simulation (Lecture/Pause/Pas/Réinitialisation),
 * Machine à états de manipulation Pick-and-Place en 9 étapes,
 * Inspecteur JSON d'échec et de récupération, et Enregistreur de jeux de données IA.
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

export const TASK_STATE_LABELS_FR: Record<PickAndPlaceState, string> = {
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

    // 1. Contrôles de Transport et Réinitialisation de la Simulation
    const simSec = document.createElement('section');
    simSec.className = 'panel-section';
    simSec.innerHTML = `
      <div class="section-header">
        <h2>Simulation</h2>
        <span class="badge badge-valid" id="sim-run-badge">ACTIF</span>
      </div>
      <div class="button-grid-3">
        <button type="button" class="btn btn-primary" id="btn-sim-play-pause">Pause</button>
        <button type="button" class="btn btn-secondary" id="btn-sim-step">Pas (1/60s)</button>
        <button type="button" class="btn btn-secondary" id="btn-sim-reset-all">Tout Réinitialiser</button>
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-sim-reset-robot">Réinit. Robot</button>
        <button type="button" class="btn btn-secondary" id="btn-toggle-cam-frustum">Champ Caméra</button>
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

    // 2. Machine à États de Manipulation Pick-and-Place (9 étapes)
    const taskSec = document.createElement('section');
    taskSec.className = 'panel-section';
    taskSec.innerHTML = `
      <div class="section-header">
        <h2>Tâche Pick & Place</h2>
        <span class="badge badge-info" id="pnp-state-badge">ATTENTE</span>
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

      <!-- Tableau de bord des Zones de Travail (Zone A -> Zone B) -->
      <div class="zone-dashboard-card" style="margin-top: 6px; padding: 6px 8px; background: rgba(15, 23, 42, 0.7); border: 1px solid #1e293b; border-radius: 6px;">
        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 4px;">
          <span style="font-weight: 700; font-size: 10px; color: #94a3b8; letter-spacing: 0.05em;">ZONE A ➔ ZONE B (6 OBJETS)</span>
          <span class="badge badge-valid" id="zone-progress-badge" style="font-size: 9px;">0 / 6 (0%)</span>
        </div>
        
        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 4px; font-size: 10px; margin-bottom: 6px;">
          <div style="background: rgba(29, 78, 216, 0.15); border: 1px solid rgba(59, 130, 246, 0.3); padding: 3px 6px; border-radius: 4px;">
            <div style="color: #60a5fa; font-size: 8.5px; font-weight: 600;">ZONE A (DÉPART)</div>
            <strong id="zone-a-count" style="color: #f8fafc; font-size: 11px;">6 restants</strong>
          </div>
          <div style="background: rgba(5, 150, 105, 0.15); border: 1px solid rgba(16, 185, 129, 0.3); padding: 3px 6px; border-radius: 4px;">
            <div style="color: #34d399; font-size: 8.5px; font-weight: 600;">ZONE B (CIBLE)</div>
            <strong id="zone-b-count" style="color: #f8fafc; font-size: 11px;">0 placés</strong>
          </div>
        </div>

        <div style="font-size: 9.5px; color: #cbd5e1; margin-bottom: 4px; display: flex; justify-content: space-between;">
          <span>Objet en cours : <strong id="zone-active-obj-name" style="color: #38bdf8;">Object_01</strong></span>
          <span id="zone-task-substatus" style="color: #94a3b8;">En attente</span>
        </div>

        <div style="width: 100%; height: 5px; background: #0f172a; border-radius: 3px; overflow: hidden; margin-bottom: 6px; border: 1px solid #1e293b;">
          <div id="zone-progress-fill" style="width: 0%; height: 100%; background: linear-gradient(90deg, #38bdf8 0%, #34d399 100%); transition: width 0.3s ease;"></div>
        </div>

        <div class="button-grid-2">
          <button type="button" class="btn btn-primary" id="btn-start-multi-zone" style="font-size: 10px; padding: 4px 6px;">
            Démarrer (6 Objets)
          </button>
          <button type="button" class="btn btn-secondary" id="btn-randomize-zone-a" style="font-size: 10px; padding: 4px 6px;">
            Randomiser Zone A
          </button>
        </div>
        <button type="button" class="btn btn-secondary btn-full" id="btn-reset-zone-a" style="margin-top: 4px; font-size: 10px; padding: 3px 6px;">
          Réinitialiser vers Zone A
        </button>
      </div>
    `;

    const pipelineGrid = taskSec.querySelector('#pnp-pipeline-grid') as HTMLElement;
    this.stateChipRefs.clear();

    ORDERED_TASK_STATES.forEach((st, idx) => {
      const chip = document.createElement('div');
      chip.className = 'state-chip';
      chip.textContent = `${idx + 1}. ${TASK_STATE_LABELS_FR[st] ?? st}`;
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

    taskSec.querySelector('#btn-start-multi-zone')?.addEventListener('click', () => {
      this.callbacks.setPlaying(true);
      if (!this.episodeRecorder.isRecording) {
        this.episodeRecorder.startRecording('dual_zone_transfer_6_objects');
      }
      if (this.pickAndPlace.isMultiObjectSequence && this.pickAndPlace.state !== 'IDLE' && this.pickAndPlace.state !== 'COMPLETE') {
        this.pickAndPlace.reset();
      } else {
        this.pickAndPlace.startMultiObjectSequence();
      }
      this.refresh();
    });

    taskSec.querySelector('#btn-randomize-zone-a')?.addEventListener('click', () => {
      if (this.pickAndPlace.workZoneManager) {
        this.pickAndPlace.workZoneManager.randomizeZoneAPositions(this.pickAndPlace.instrumentFactory);
      }
      this.pickAndPlace.reset();
      this.refresh();
    });

    taskSec.querySelector('#btn-reset-zone-a')?.addEventListener('click', () => {
      this.pickAndPlace.instrumentFactory.resetAllInstrumentsToZoneA();
      this.pickAndPlace.reset();
      this.refresh();
    });

    // 3. Enregistreur de Données pour IA & Apprentissage par Imitation
    const aiSec = document.createElement('section');
    aiSec.className = 'panel-section';
    aiSec.innerHTML = `
      <div class="section-header">
        <h2>Enregistreur IA</h2>
        <span class="badge badge-info" id="ai-rec-badge">EN ATTENTE</span>
      </div>
      <div class="telemetry-kv-grid">
        <div><span>Épisode</span><strong id="ai-ep-id">ep_001</strong></div>
        <div><span>Échantillons</span><strong id="ai-step-count">0</strong></div>
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-ai-toggle-rec">Démarrer Enregistrement</button>
        <button type="button" class="btn btn-warning" id="btn-ai-mark-failed">Arrêter (Échec)</button>
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
      runBadge.textContent = playing ? 'ACTIF' : 'EN PAUSE';
      runBadge.className = playing ? 'badge badge-valid' : 'badge badge-warning';
    }
    if (playPauseBtn) {
      playPauseBtn.textContent = playing ? 'Pause' : 'Reprendre';
    }

    const curState = this.pickAndPlace.state;
    const stateBadge = this.container.querySelector('#pnp-state-badge');
    const statusMsg = this.container.querySelector('#pnp-status-message');
    const recoverBtn = this.container.querySelector('#btn-pnp-recover') as HTMLElement | null;
    const jsonBox = this.container.querySelector('#json-failure-container') as HTMLElement | null;
    const jsonPre = this.container.querySelector('#json-failure-pre');

    if (stateBadge) {
      stateBadge.textContent = TASK_STATE_LABELS_FR[curState] ?? curState;
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
      recBadge.textContent = this.episodeRecorder.isRecording ? 'ENREGISTREMENT' : 'EN ATTENTE';
      recBadge.className = this.episodeRecorder.isRecording ? 'badge badge-danger' : 'badge badge-info';
    }
    if (epIdEl) {
      epIdEl.textContent = this.episodeRecorder.episodeId;
    }
    if (stepCountEl) {
      stepCountEl.textContent = `${this.episodeRecorder.frames.length} échantillons`;
    }
    if (recBtn) {
      recBtn.textContent = this.episodeRecorder.isRecording ? 'Arrêter Enregistrement' : 'Démarrer Enregistrement';
    }

    // Tableau de bord des Zones de Travail (Progression)
    const progress = this.pickAndPlace.getMultiObjectProgress();
    const zoneProgressBadge = this.container.querySelector('#zone-progress-badge');
    const zoneACount = this.container.querySelector('#zone-a-count');
    const zoneBCount = this.container.querySelector('#zone-b-count');
    const zoneActiveObj = this.container.querySelector('#zone-active-obj-name');
    const zoneSubstatus = this.container.querySelector('#zone-task-substatus');
    const zoneProgressFill = this.container.querySelector('#zone-progress-fill') as HTMLElement | null;
    const multiBtn = this.container.querySelector('#btn-start-multi-zone');

    if (zoneProgressBadge) {
      zoneProgressBadge.textContent = `${progress.placed} / ${progress.total} (${progress.progressPercent}%)`;
      zoneProgressBadge.className = progress.placed >= progress.total ? 'badge badge-valid' : 'badge badge-info';
    }

    if (this.pickAndPlace.workZoneManager) {
      const telem = this.pickAndPlace.workZoneManager.getTelemetry(this.pickAndPlace.instrumentFactory);
      if (zoneACount) zoneACount.textContent = `${telem.remainingInZoneA} restants`;
      if (zoneBCount) zoneBCount.textContent = `${telem.placedInZoneB} placés`;
    }

    if (zoneActiveObj) {
      zoneActiveObj.textContent = progress.currentObject ?? (this.pickAndPlace.instrumentFactory.activeInstrumentId ?? 'Object_01');
    }

    if (zoneSubstatus) {
      zoneSubstatus.textContent = progress.isSequenceRunning
        ? `En cours (${progress.currentIndex + 1}/6)`
        : progress.placed >= progress.total
        ? 'Terminé avec succès'
        : 'Prêt';
    }

    if (zoneProgressFill) {
      zoneProgressFill.style.width = `${progress.progressPercent}%`;
    }

    if (multiBtn) {
      multiBtn.textContent = progress.isSequenceRunning ? 'Arrêter Séquence' : 'Démarrer (6 Objets)';
    }
  }
}
