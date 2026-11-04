/**
 * @file SimulationControls.ts
 * Contrôles de transport de la simulation (Lecture/Pause/Pas/Réinitialisation),
 * Machine à états de manipulation Pick-and-Place en 9 étapes,
 * Inspecteur JSON d'échec et de récupération, et Enregistreur de jeux de données IA.
 */

import { PickAndPlace, PickAndPlaceState } from '../manipulation/PickAndPlace.js';
import { EpisodeRecorder } from '../ai/EpisodeRecorder.js';
import { RobotModel } from '../robot/RobotModel.js';
import { ScenarioGenerator, ScenarioDifficultyLevel } from '../objects/ScenarioGenerator.js';
import { DataQualityValidator } from '../ai/DataQualityValidator.js';
import { NeuralPolicyController } from '../ai/NeuralPolicyController.js';
import {
  FailureScenarioType,
  FAILURE_SCENARIO_DEFINITIONS
} from '../manipulation/FailureScenarioController.js';

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
  public scenarioGenerator?: ScenarioGenerator;
  public neuralPolicyController?: NeuralPolicyController;
  public dataQualityValidator: DataQualityValidator = new DataQualityValidator();

  private stateChipRefs: Map<PickAndPlaceState, HTMLElement> = new Map();

  constructor(
    container: HTMLElement | null,
    pickAndPlace: PickAndPlace,
    episodeRecorder: EpisodeRecorder,
    robotModel: RobotModel,
    callbacks: SimulationTransportCallbacks,
    scenarioGenerator?: ScenarioGenerator,
    neuralPolicyController?: NeuralPolicyController
  ) {
    this.container = container;
    this.pickAndPlace = pickAndPlace;
    this.episodeRecorder = episodeRecorder;
    this.robotModel = robotModel;
    this.callbacks = callbacks;
    this.scenarioGenerator = scenarioGenerator;
    this.neuralPolicyController = neuralPolicyController;

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

    // 3. Enregistreur de Données pour IA (Behavior Cloning & Diffusion Policy)
    const aiSec = document.createElement('section');
    aiSec.className = 'panel-section';
    aiSec.innerHTML = `
      <div class="section-header">
        <h2>Enregistreur IA (20 Hz)</h2>
        <span class="badge badge-info" id="ai-rec-badge">EN ATTENTE</span>
      </div>
      <div class="telemetry-kv-grid">
        <div><span>Épisode en cours</span><strong id="ai-ep-id">ep_001</strong></div>
        <div><span>Épisodes archivés</span><strong id="ai-archived-count">0</strong></div>
        <div><span>Échantillons (20 Hz)</span><strong id="ai-step-count">0</strong></div>
        <div><span>Conformité Dataset</span><strong id="ai-quality-score">100%</strong></div>
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-ai-toggle-rec">Démarrer Enregistrement</button>
        <button type="button" class="btn btn-warning" id="btn-ai-mark-failed">Arrêter (Échec)</button>
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-accent" id="btn-ai-export-json">Export Dataset JSON</button>
        <button type="button" class="btn btn-secondary" id="btn-ai-export-csv">Export CSV Consolidé</button>
      </div>
      <div style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-ai-clear" style="width: 100%; opacity: 0.85; font-size: 11px;">Réinitialiser Données & Compteurs</button>
      </div>
    `;

    aiSec.querySelector('#btn-ai-clear')?.addEventListener('click', () => {
      this.episodeRecorder.clear();
      this.refresh();
    });

    aiSec.querySelector('#btn-ai-toggle-rec')?.addEventListener('click', () => {
      if (this.episodeRecorder.isRecording) {
        this.episodeRecorder.stopRecording('success');
      } else {
        this.episodeRecorder.startRecording(
          `surgical_ep_${String(this.episodeRecorder.episodeCounter).padStart(3, '0')}`,
          this.scenarioGenerator?.currentScenario ?? undefined
        );
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
      const jsonStr =
        this.episodeRecorder.completedEpisodes.length > 1
          ? this.episodeRecorder.exportConsolidatedJSON()
          : this.episodeRecorder.exportJSON();
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
      const csvStr =
        this.episodeRecorder.completedEpisodes.length > 1
          ? this.episodeRecorder.exportConsolidatedCSV()
          : this.episodeRecorder.exportCSV();
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

    // 4. Générateur de Scénarios ML Déterministe
    const scenSec = document.createElement('section');
    scenSec.className = 'panel-section';
    scenSec.innerHTML = `
      <div class="section-header">
        <h2>Générateur de Scénarios ML</h2>
        <span class="badge badge-info" id="scen-active-id">DÉFAUT</span>
      </div>
      <div style="margin-bottom: 6px;">
        <label style="font-size: 11px; opacity: 0.8; display: block; margin-bottom: 2px;">Niveau de Difficulté :</label>
        <select id="scen-level-select" style="width: 100%; background: #1a1e24; color: #fff; border: 1px solid #333d4b; padding: 4px; border-radius: 4px; font-size: 11px;">
          <option value="1">Niveau 1 : 1 objet - Position fixe canonique</option>
          <option value="2" selected>Niveau 2 : 1 objet - Position & lacet aléatoires</option>
          <option value="3">Niveau 3 : 1 objet - Limites espace de travail</option>
          <option value="4">Niveau 4 : 6 objets - Grille multi-objets</option>
          <option value="5">Niveau 5 : Perturbé - Randomisation physique</option>
        </select>
      </div>
      <div class="button-grid-2">
        <button type="button" class="btn btn-secondary" id="btn-scen-generate">Nouveau Scénario</button>
        <button type="button" class="btn btn-primary" id="btn-scen-apply-start">Appliquer & Démarrer</button>
      </div>
    `;

    scenSec.querySelector('#btn-scen-generate')?.addEventListener('click', () => {
      const select = scenSec.querySelector('#scen-level-select') as HTMLSelectElement | null;
      const level = (parseInt(select?.value ?? '2', 10) || 2) as ScenarioDifficultyLevel;
      if (this.scenarioGenerator) {
        const scen = this.scenarioGenerator.generateScenario(level, Date.now());
        this.scenarioGenerator.applyScenario(scen);
        const scenBadge = scenSec.querySelector('#scen-active-id');
        if (scenBadge) scenBadge.textContent = `LVL ${level}`;
      }
      this.pickAndPlace.reset();
      this.refresh();
    });

    scenSec.querySelector('#btn-scen-apply-start')?.addEventListener('click', () => {
      const select = scenSec.querySelector('#scen-level-select') as HTMLSelectElement | null;
      const level = (parseInt(select?.value ?? '2', 10) || 2) as ScenarioDifficultyLevel;
      if (this.scenarioGenerator) {
        const scen = this.scenarioGenerator.generateScenario(level, Date.now());
        this.scenarioGenerator.applyScenario(scen);
      }
      this.callbacks.setPlaying(true);
      if (level === 4) {
        this.pickAndPlace.startMultiObjectSequence();
      } else {
        this.pickAndPlace.start();
      }
      this.refresh();
    });

    // 5. Simulateur d'Échecs ML (Failure Dataset)
    const failSec = document.createElement('section');
    failSec.className = 'panel-section';
    failSec.innerHTML = `
      <div class="section-header">
        <h2>Simulateur d'Échecs ML (Dataset)</h2>
        <span class="badge badge-info" id="fail-active-badge">NOMINAL</span>
      </div>
      <div style="margin-bottom: 6px;">
        <label style="font-size: 11px; opacity: 0.8; display: block; margin-bottom: 2px;">Type de Défaillance :</label>
        <select id="fail-type-select" style="width: 100%; background: #1a1e24; color: #fff; border: 1px solid #333d4b; padding: 4px; border-radius: 4px; font-size: 11px;">
          <option value="MISSED_APPROACH">1. Approche Décalée (Prise Manquée)</option>
          <option value="SLIP_LOW_FRICTION">2. Glissement par Force Insuffisante</option>
          <option value="DROP_IN_TRANSIT">3. Chute en Cours de Transport</option>
          <option value="COLLISION_OBSTACLE">4. Collision avec Obstacle</option>
          <option value="JOINT_LIMIT_REACH">5. Dépassement de Butée Articulaire</option>
          <option value="EXCESSIVE_TORQUE">6. Surcharge de Couple Moteur</option>
          <option value="TIPPED_PLACEMENT">7. Dépose Instable & Basculement</option>
          <option value="KNOCK_ON_APPROACH">8. Percussion / Collision à l'Approche</option>
          <option value="ORIENTATION_MISALIGNMENT">9. Défaut d'Orientation (+45°)</option>
          <option value="DYNAMIC_JERK_SLIP">10. Décrochage par À-Coup Dynamique</option>
          <option value="INCOMPLETE_RELEASE">11. Accrochage au Relâchement</option>
          <option value="RANDOM_FAILURE">🎲 Scénario Aléatoire</option>
        </select>
      </div>
      <div class="telemetry-kv-grid" style="margin-bottom: 6px;">
        <div><span>Défaillance Armée</span><strong id="fail-armed-status">Non</strong></div>
        <div><span>Déclenchée</span><strong id="fail-triggered-status">Non</strong></div>
      </div>
      <p id="fail-desc-p" style="font-size: 10px; color: #94a3b8; margin: 4px 0 6px; line-height: 1.3;">
        Sélectionnez un scénario d'échec pour collecter des démonstrations négatives labellisées pour l'IA.
      </p>
      <div class="button-grid-2">
        <button type="button" class="btn btn-warning" id="btn-fail-start">Collecter Défaillance</button>
        <button type="button" class="btn btn-secondary" id="btn-fail-disarm">Réinitialiser</button>
      </div>
    `;

    failSec.querySelector('#fail-type-select')?.addEventListener('change', (e) => {
      const select = e.target as HTMLSelectElement;
      const type = (select.value ?? 'MISSED_APPROACH') as FailureScenarioType;
      const def = FAILURE_SCENARIO_DEFINITIONS[type];
      const descP = failSec.querySelector('#fail-desc-p');
      if (descP && def) {
        descP.textContent = def.description;
      }
    });

    failSec.querySelector('#btn-fail-start')?.addEventListener('click', () => {
      const select = failSec.querySelector('#fail-type-select') as HTMLSelectElement | null;
      const type = (select?.value ?? 'MISSED_APPROACH') as FailureScenarioType;
      this.callbacks.setPlaying(true);
      if (!this.episodeRecorder.isRecording) {
        this.episodeRecorder.startRecording(
          `failure_${type.toLowerCase()}_${Date.now()}`,
          this.scenarioGenerator?.currentScenario ?? undefined,
          type
        );
      }
      this.pickAndPlace.armFailureScenario(type);
      this.pickAndPlace.start();
      this.refresh();
    });

    failSec.querySelector('#btn-fail-disarm')?.addEventListener('click', () => {
      this.pickAndPlace.failureController.disarm();
      this.pickAndPlace.reset();
      this.refresh();
    });

    // 6. Pilote IA Autonome (ONNX Policy Inference)
    const policySec = document.createElement('section');
    policySec.className = 'panel-section';
    policySec.innerHTML = `
      <div class="section-header">
        <h2>Pilote IA Autonome (ONNX)</h2>
        <span class="badge badge-info" id="ai-policy-badge">NON CHARGÉ</span>
      </div>
      <div class="telemetry-kv-grid" style="margin-bottom: 6px;">
        <div><span>Statut IA</span><strong id="ai-policy-status">Inactif</strong></div>
        <div><span>Inférence</span><strong id="ai-policy-latency">0 ms</strong></div>
      </div>
      <p style="font-size: 10px; color: #94a3b8; margin: 4px 0 6px; line-height: 1.3;">
        Exécute le réseau de neurones entraîné (robot_policy.onnx) en boucle fermée à 20 Hz directement dans Three.js.
      </p>
      <div class="button-grid-2">
        <button type="button" class="btn btn-secondary" id="btn-load-ai-policy">Charger ONNX</button>
        <button type="button" class="btn btn-primary" id="btn-toggle-ai-policy">Démarrer IA</button>
      </div>
    `;

    policySec.querySelector('#btn-load-ai-policy')?.addEventListener('click', async () => {
      const badge = policySec.querySelector('#ai-policy-badge');
      const loadBtn = policySec.querySelector('#btn-load-ai-policy') as HTMLButtonElement | null;
      if (badge) badge.textContent = 'CHARGEMENT...';
      if (loadBtn) loadBtn.disabled = true;
      if (this.neuralPolicyController) {
        const ok = await this.neuralPolicyController.loadModel();
        const telem = this.neuralPolicyController.getTelemetry();
        if (badge) {
          const backendLabel = telem.backend === 'onnx-wasm' ? 'ONNX' : (telem.backend === 'native-js' ? 'Natif JS' : '');
          badge.textContent = ok ? `PRÊT ${backendLabel ? `(${backendLabel})` : ''}` : 'ERREUR';
          badge.className = ok ? 'badge badge-valid' : 'badge badge-danger';
        }
      }
      if (loadBtn) loadBtn.disabled = false;
      this.refresh();
    });

    policySec.querySelector('#btn-toggle-ai-policy')?.addEventListener('click', async () => {
      if (!this.neuralPolicyController) return;
      const toggleBtn = policySec.querySelector('#btn-toggle-ai-policy') as HTMLButtonElement | null;
      if (!this.neuralPolicyController.isLoaded) {
        if (toggleBtn) toggleBtn.textContent = 'Chargement...';
        await this.neuralPolicyController.loadModel();
      }
      this.callbacks.setPlaying(true);
      if (this.neuralPolicyController.isRunning) {
        this.neuralPolicyController.stop();
      } else {
        this.neuralPolicyController.start();
      }
      this.refresh();
    });

    this.container.appendChild(simSec);
    this.container.appendChild(taskSec);
    this.container.appendChild(aiSec);
    this.container.appendChild(scenSec);
    this.container.appendChild(failSec);
    this.container.appendChild(policySec);
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
    const archivedEl = this.container.querySelector('#ai-archived-count');
    const stepCountEl = this.container.querySelector('#ai-step-count');
    const qualityScoreEl = this.container.querySelector('#ai-quality-score');
    const recBtn = this.container.querySelector('#btn-ai-toggle-rec');

    if (recBadge) {
      recBadge.textContent = this.episodeRecorder.isRecording ? 'ENREGISTREMENT' : 'EN ATTENTE';
      recBadge.className = this.episodeRecorder.isRecording ? 'badge badge-danger' : 'badge badge-info';
    }
    if (epIdEl) {
      epIdEl.textContent = this.episodeRecorder.episodeId;
    }
    if (archivedEl) {
      archivedEl.textContent = `${this.episodeRecorder.completedEpisodes.length} épisodes`;
    }
    if (stepCountEl) {
      stepCountEl.textContent = `${this.episodeRecorder.totalRecordedSamples} (20 Hz)`;
    }
    if (qualityScoreEl) {
      const epCount = this.episodeRecorder.completedEpisodes.length;
      if (epCount > 0) {
        const rep = this.dataQualityValidator.generateDatasetQualityReport(this.episodeRecorder.completedEpisodes);
        qualityScoreEl.textContent = `${rep.compliance_rate_percent}% (${rep.compliant_episodes_count}/${epCount})`;
      } else {
        qualityScoreEl.textContent = '100% (Prêt)';
      }
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

    // Contrôleur de Défaillances & Échecs ML
    const failStatus = this.pickAndPlace.failureController.getStatus();
    const failBadge = this.container.querySelector('#fail-active-badge');
    const failArmed = this.container.querySelector('#fail-armed-status');
    const failTriggered = this.container.querySelector('#fail-triggered-status');
    if (failBadge) {
      failBadge.textContent = failStatus.isInjecting ? failStatus.resolvedType : 'NOMINAL';
      failBadge.className = failStatus.isInjecting ? 'badge badge-warning' : 'badge badge-info';
    }
    if (failArmed) {
      failArmed.textContent = failStatus.isInjecting ? 'Oui' : 'Non';
      failArmed.style.color = failStatus.isInjecting ? '#f59e0b' : '#94a3b8';
    }
    if (failTriggered) {
      failTriggered.textContent = failStatus.faultTriggered ? 'Oui (Détecté)' : 'En attente';
      failTriggered.style.color = failStatus.faultTriggered ? '#ef4444' : '#94a3b8';
    }

    // Télémétrie Politique Neuronale ONNX
    if (this.neuralPolicyController) {
      const telem = this.neuralPolicyController.getTelemetry();
      const policyBadge = this.container.querySelector('#ai-policy-badge');
      const policyStatus = this.container.querySelector('#ai-policy-status');
      const policyLatency = this.container.querySelector('#ai-policy-latency');
      const toggleBtn = this.container.querySelector('#btn-toggle-ai-policy');

      if (policyBadge && telem.isLoaded) {
        const backendLabel = telem.backend === 'onnx-wasm' ? 'ONNX' : (telem.backend === 'native-js' ? 'Natif JS' : '');
        policyBadge.textContent = telem.isRunning ? `EN COURS (${backendLabel})` : `PRÊT (${backendLabel})`;
        policyBadge.className = telem.isRunning ? 'badge badge-valid' : 'badge badge-info';
      }
      if (policyStatus) {
        const phaseInfo = telem.currentPhase
          ? `${telem.phaseIndex}/9 ${telem.currentPhase} (${Math.round((telem.confidenceScore ?? 1) * 100)}%)`
          : 'Contrôle Actif (20 Hz)';
        policyStatus.textContent = telem.isRunning
          ? phaseInfo
          : telem.isLoaded
          ? `Prêt (${telem.backend})`
          : 'Non chargé';
        (policyStatus as HTMLElement).style.color = telem.isRunning ? '#34d399' : '#94a3b8';
      }
      if (policyLatency) {
        policyLatency.textContent = `${telem.inferenceTimeMs.toFixed(1)} ms`;
      }
      if (toggleBtn) {
        toggleBtn.textContent = telem.isRunning ? 'Arrêter IA' : 'Démarrer IA';
        toggleBtn.className = telem.isRunning ? 'btn btn-warning' : 'btn btn-primary';
      }
    }
  }
}
