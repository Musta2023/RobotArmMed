/**
 * @file RobotControls.ts
 * Interactive 3D Manipulation Modes (Object Mode / Robot Mode / AI Mode),
 * 3D Gizmo Translation & Rotation Controls, Mechanical Gripper Control,
 * and Collapsible Advanced 6-DOF Joint Motor Panel (Section 9).
 */

import { RobotModel } from '../robot/RobotModel.js';
import { MotionController } from '../control/MotionController.js';
import { InteractionModeManager } from '../interaction/InteractionModeManager.js';
import { DragPlaneMode } from '../interaction/ObjectManipulator.js';

export class RobotControls {
  public container: HTMLElement | null;
  public robotModel: RobotModel;
  public motionController: MotionController;
  public interactionModeManager: InteractionModeManager | null;

  private jointDomRefs: Array<{
    slider: HTMLInputElement;
    numInput: HTMLInputElement;
    curReadout: HTMLElement;
    velReadout: HTMLElement;
    torqueReadout: HTMLElement;
    row: HTMLElement;
  }> = [];

  constructor(
    container: HTMLElement | null,
    robotModel: RobotModel,
    motionController: MotionController,
    interactionModeManager: InteractionModeManager | null = null
  ) {
    this.container = container;
    this.robotModel = robotModel;
    this.motionController = motionController;
    this.interactionModeManager = interactionModeManager;

    if (this.container) {
      this._render();
      this.refresh();
    }
  }

  private _render(): void {
    if (!this.container) return;
    this.container.innerHTML = '';

    // 1. 3D Mouse Interaction Modes & Gizmo Section (Primary Workflow)
    const modeSec = document.createElement('section');
    modeSec.className = 'panel-section';
    modeSec.innerHTML = `
      <div class="section-header">
        <h2>3D Interaction</h2>
        <span class="badge badge-valid" id="active-mode-badge">OBJECT</span>
      </div>
      <div class="button-grid-3">
        <button type="button" class="btn btn-primary" id="btn-mode-object" title="Shortcut: 1">Object (1)</button>
        <button type="button" class="btn btn-secondary" id="btn-mode-robot" title="Shortcut: 2">Robot (2)</button>
        <button type="button" class="btn btn-secondary" id="btn-mode-ai" title="Shortcut: 3">AI (3)</button>
      </div>
      <div class="button-grid-2" style="margin-top: 4px;">
        <button type="button" class="btn btn-secondary" id="btn-gizmo-translate" title="Shortcut: W">Move (W)</button>
        <button type="button" class="btn btn-secondary" id="btn-gizmo-rotate" title="Shortcut: E">Rotate (E)</button>
      </div>
      <div class="select-row" style="margin-top: 4px;">
        <select id="select-drag-plane" class="control-select" title="Object 3D Drag Plane">
          <option value="horizontal">Plan: Horizontal (XZ)</option>
          <option value="vertical">Plan: Vertical (XY)</option>
          <option value="camera">Plan: Vue Caméra</option>
        </select>
        <button type="button" class="btn btn-secondary" id="btn-reset-camera">Reset Cam</button>
      </div>
      <button type="button" class="btn btn-accent btn-full" id="btn-approach-object" style="margin-top: 4px;">
        Aligner Pince sur Objet
      </button>
      <div class="feedback-banner status-info" id="interaction-status-banner" style="margin-top: 4px;">
        Mode Objet : Glisser-déposer l'objet à la souris.
      </div>
    `;

    modeSec.querySelector('#btn-mode-object')?.addEventListener('click', () => {
      this.interactionModeManager?.setMode('object');
      this.refresh();
    });
    modeSec.querySelector('#btn-mode-robot')?.addEventListener('click', () => {
      this.interactionModeManager?.setMode('robot');
      this.refresh();
    });
    modeSec.querySelector('#btn-mode-ai')?.addEventListener('click', () => {
      this.interactionModeManager?.setMode('ai');
      this.refresh();
    });

    modeSec.querySelector('#btn-gizmo-translate')?.addEventListener('click', () => {
      this.interactionModeManager?.setGizmoTransformMode('translate');
      this.refresh();
    });
    modeSec.querySelector('#btn-gizmo-rotate')?.addEventListener('click', () => {
      this.interactionModeManager?.setGizmoTransformMode('rotate');
      this.refresh();
    });

    const planeSelect = modeSec.querySelector('#select-drag-plane') as HTMLSelectElement;
    planeSelect?.addEventListener('change', () => {
      this.interactionModeManager?.setDragPlaneMode(planeSelect.value as DragPlaneMode);
    });

    modeSec.querySelector('#btn-reset-camera')?.addEventListener('click', () => {
      this.interactionModeManager?.cameraManager.resetMainCamera();
    });

    modeSec.querySelector('#btn-approach-object')?.addEventListener('click', () => {
      if (this.interactionModeManager) {
        this.interactionModeManager.setMode('robot');
        this.interactionModeManager.commandRobotToApproachObject();
        this.refresh();
      }
    });

    // 2. Mechanical Gripper Control Section
    const gripSec = document.createElement('section');
    gripSec.className = 'panel-section';
    const gCfg = this.robotModel.config.gripper;

    gripSec.innerHTML = `
      <div class="section-header">
        <h2>Pince Mécanique</h2>
        <span class="badge badge-info" id="grip-status-badge">OPEN</span>
      </div>
      <div class="button-grid-2">
        <button type="button" class="btn btn-secondary" id="btn-grip-open">Ouvrir (G)</button>
        <button type="button" class="btn btn-primary" id="btn-grip-close">Fermer (G)</button>
      </div>
      <div class="gripper-opening-row" style="margin-top: 4px;">
        <div class="gripper-readout-header">
          <span>Ouverture</span>
          <strong id="grip-open-mm">${(gCfg.defaultOpeningMeters * 1000).toFixed(1)} mm</strong>
        </div>
        <input
          type="range"
          id="grip-open-slider"
          min="${(gCfg.minOpeningMeters * 1000).toFixed(0)}"
          max="${(gCfg.maxOpeningMeters * 1000).toFixed(0)}"
          step="1"
          value="${(gCfg.defaultOpeningMeters * 1000).toFixed(0)}"
        />
      </div>
      <div class="gripper-opening-row">
        <div class="gripper-readout-header">
          <span>Force serrage</span>
          <strong id="grip-force-n">${gCfg.defaultGrippingForceNewtons.toFixed(1)} N</strong>
        </div>
        <input
          type="range"
          id="grip-force-slider"
          min="0.5"
          max="${gCfg.maxGrippingForceNewtons}"
          step="0.5"
          value="${gCfg.defaultGrippingForceNewtons}"
        />
      </div>
      <div class="feedback-banner status-info" id="grip-contact-banner" style="font-size: 10px; padding: 4px 6px;">
        En attente de contact physique.
      </div>
    `;

    const openSlider = gripSec.querySelector('#grip-open-slider') as HTMLInputElement;
    const forceSlider = gripSec.querySelector('#grip-force-slider') as HTMLInputElement;

    openSlider.addEventListener('input', () => {
      const m = parseFloat(openSlider.value) / 1000;
      this.robotModel.gripper.setTargetOpeningMeters(m, false);
    });

    forceSlider.addEventListener('input', () => {
      const f = parseFloat(forceSlider.value);
      this.robotModel.gripper.setGrippingForceNewtons(f);
    });

    gripSec.querySelector('#btn-grip-open')?.addEventListener('click', () => {
      this.robotModel.gripper.open(false);
    });

    gripSec.querySelector('#btn-grip-close')?.addEventListener('click', () => {
      this.robotModel.gripper.close(false);
    });

    // 3. Inverse Kinematics Target Pose (Position XYZ + Orientation RPY)
    const ikSec = document.createElement('section');
    ikSec.className = 'panel-section';
    ikSec.innerHTML = `
      <details class="ik-collapsible">
        <summary class="section-header" style="cursor: pointer; list-style: none;">
          <h2>Cible 6D / IK Numérique</h2>
          <span class="badge badge-info" id="ik-status-badge">READY</span>
        </summary>
        <div style="margin-top: 6px;">
          <div class="ik-input-grid">
            <label>X (m)<input type="number" id="ik-target-x" step="0.02" value="-0.30" class="joint-number-input" /></label>
            <label>Y (m)<input type="number" id="ik-target-y" step="0.02" value="0.18" class="joint-number-input" /></label>
            <label>Z (m)<input type="number" id="ik-target-z" step="0.02" value="0.24" class="joint-number-input" /></label>
          </div>
          <div class="ik-input-grid" style="margin-top: 4px;">
            <label>Rx (°)<input type="number" id="ik-target-rx" step="5" value="0" class="joint-number-input" /></label>
            <label>Ry (°)<input type="number" id="ik-target-ry" step="5" value="0" class="joint-number-input" /></label>
            <label>Rz (°)<input type="number" id="ik-target-rz" step="5" value="0" class="joint-number-input" /></label>
          </div>
          <div class="button-grid-2" style="margin-top: 6px;">
            <button type="button" class="btn btn-accent" id="btn-solve-ik">Appliquer IK</button>
            <button type="button" class="btn btn-secondary" id="btn-sync-ik-tcp">Lire TCP</button>
          </div>
        </div>
      </details>
    `;

    const xInput = ikSec.querySelector('#ik-target-x') as HTMLInputElement;
    const yInput = ikSec.querySelector('#ik-target-y') as HTMLInputElement;
    const zInput = ikSec.querySelector('#ik-target-z') as HTMLInputElement;
    const rxInput = ikSec.querySelector('#ik-target-rx') as HTMLInputElement;
    const ryInput = ikSec.querySelector('#ik-target-ry') as HTMLInputElement;
    const rzInput = ikSec.querySelector('#ik-target-rz') as HTMLInputElement;
    const ikBadge = ikSec.querySelector('#ik-status-badge') as HTMLElement;

    ikSec.querySelector('#btn-solve-ik')?.addEventListener('click', () => {
      const tx = parseFloat(xInput.value) || 0;
      const ty = parseFloat(yInput.value) || 0.2;
      const tz = parseFloat(zInput.value) || 0.2;
      const rx = parseFloat(rxInput.value) || 0;
      const ry = parseFloat(ryInput.value) || 0;
      const rz = parseFloat(rzInput.value) || 0;

      if (this.interactionModeManager) {
        const gCtrl = this.interactionModeManager.gripperTargetController;
        gCtrl.setTargetPosition([tx, ty, tz]);
        const res = gCtrl.setTargetOrientationEulerDeg([rx, ry, rz], true);
        ikBadge.textContent = res.status;
        ikBadge.className = res.isReachable ? 'badge badge-valid' : 'badge badge-danger';
      } else {
        const res = this.motionController.commandCartesianPose([tx, ty, tz]);
        ikBadge.textContent = res.status;
        ikBadge.className = res.success ? 'badge badge-valid' : 'badge badge-danger';
      }
    });

    ikSec.querySelector('#btn-sync-ik-tcp')?.addEventListener('click', () => {
      const pos = this.robotModel.state.latestSnapshot.endEffector.position;
      xInput.value = pos[0].toFixed(3);
      yInput.value = pos[1].toFixed(3);
      zInput.value = pos[2].toFixed(3);
      this.interactionModeManager?.gripperTargetController.syncToCurrentTCP();
    });

    // 4. Advanced Collapsible 6-DOF Joint Motor Control Section
    const jointSec = document.createElement('section');
    jointSec.className = 'panel-section';

    jointSec.innerHTML = `
      <details class="advanced-joints-details">
        <summary class="section-header" style="cursor: pointer; list-style: none;">
          <h2>Advanced: 6-DOF Joint Motors</h2>
          <button type="button" class="btn-mini" id="btn-robot-home">Home Pose</button>
        </summary>
        <div class="joints-list" id="joints-motor-list" style="margin-top: 8px;"></div>
      </details>
    `;

    const listEl = jointSec.querySelector('#joints-motor-list') as HTMLElement;
    this.jointDomRefs = [];

    this.robotModel.config.joints.forEach((cfg, idx) => {
      const row = document.createElement('div');
      row.className = 'joint-row';
      const initDeg = cfg.homeAngleDeg.toFixed(1);

      row.innerHTML = `
        <div class="joint-row-top">
          <div class="joint-title-group">
            <span class="joint-id-badge">${cfg.id}</span>
            <span class="joint-name">${cfg.shortName}</span>
            <span class="joint-axis-tag">[${cfg.minAngleDeg}°..+${cfg.maxAngleDeg}°]</span>
          </div>
          <span class="joint-angle-readout" id="j-cur-${cfg.id}">${initDeg}°</span>
        </div>
        <div class="joint-row-controls">
          <input
            type="range"
            class="joint-slider"
            min="${cfg.minAngleDeg}"
            max="${cfg.maxAngleDeg}"
            step="0.5"
            value="${initDeg}"
          />
          <input
            type="number"
            class="joint-number-input"
            min="${cfg.minAngleDeg}"
            max="${cfg.maxAngleDeg}"
            step="1"
            value="${initDeg}"
          />
        </div>
        <div class="joint-telemetry-subrow">
          <span id="j-vel-${cfg.id}">ω: 0.00 rad/s</span>
          <span id="j-trq-${cfg.id}">τ: 0.00 / ${cfg.maxMotorTorqueNm.toFixed(0)} N·m</span>
        </div>
      `;

      const slider = row.querySelector('.joint-slider') as HTMLInputElement;
      const numInput = row.querySelector('.joint-number-input') as HTMLInputElement;
      const curReadout = row.querySelector(`#j-cur-${cfg.id}`) as HTMLElement;
      const velReadout = row.querySelector(`#j-vel-${cfg.id}`) as HTMLElement;
      const torqueReadout = row.querySelector(`#j-trq-${cfg.id}`) as HTMLElement;

      const onTargetChange = (val: number) => {
        const clamped = this.motionController.jointController.setJointTargetPositionDeg(
          idx,
          val
        );
        slider.value = String(clamped);
        numInput.value = clamped.toFixed(1);
      };

      slider.addEventListener('input', () => onTargetChange(parseFloat(slider.value)));
      numInput.addEventListener('change', () => onTargetChange(parseFloat(numInput.value)));

      this.jointDomRefs.push({
        slider,
        numInput,
        curReadout,
        velReadout,
        torqueReadout,
        row
      });
      listEl.appendChild(row);
    });

    jointSec.querySelector('#btn-robot-home')?.addEventListener('click', (e) => {
      e.stopPropagation();
      this.motionController.moveToHomePose(false);
    });

    this.container.appendChild(modeSec);
    this.container.appendChild(gripSec);
    this.container.appendChild(ikSec);
    this.container.appendChild(jointSec);
  }

  public refresh(): void {
    if (!this.container) return;
    const snap = this.robotModel.state.latestSnapshot;

    // Refresh Mode & Reachability UI
    if (this.interactionModeManager) {
      const m = this.interactionModeManager.mode;
      const modeBadge = this.container.querySelector('#active-mode-badge');
      const btnObj = this.container.querySelector('#btn-mode-object');
      const btnRob = this.container.querySelector('#btn-mode-robot');
      const btnAi = this.container.querySelector('#btn-mode-ai');
      const statusBanner = this.container.querySelector('#interaction-status-banner');
      const gCtrl = this.interactionModeManager.gripperTargetController;

      if (modeBadge) {
        modeBadge.textContent =
          m === 'object' ? 'OBJECT MODE' : m === 'robot' ? 'ROBOT MODE' : 'AI MODE';
      }
      if (btnObj && btnRob && btnAi) {
        btnObj.className = m === 'object' ? 'btn btn-primary' : 'btn btn-secondary';
        btnRob.className = m === 'robot' ? 'btn btn-primary' : 'btn btn-secondary';
        btnAi.className = m === 'ai' ? 'btn btn-primary' : 'btn btn-secondary';
      }
      if (statusBanner) {
        if (!gCtrl.isTargetReachable && gCtrl.warningMessage) {
          statusBanner.className = 'feedback-banner status-danger';
          statusBanner.textContent = gCtrl.warningMessage;
        } else {
          statusBanner.className = 'feedback-banner status-info';
          statusBanner.textContent = this.interactionModeManager.statusText;
        }
      }
    }

    snap.joints.forEach((jt, idx) => {
      const ref = this.jointDomRefs[idx];
      if (!ref) return;

      ref.curReadout.textContent = `${jt.currentAngleDeg.toFixed(1)}° (→${jt.targetAngleDeg.toFixed(0)}°)`;
      ref.velReadout.textContent = `ω: ${jt.angularVelocityRadPerSec.toFixed(2)} rad/s`;
      ref.torqueReadout.textContent = `τ: ${Math.abs(jt.motorTorqueNm).toFixed(2)} / ${jt.maxMotorTorqueNm.toFixed(1)} N·m`;

      if (typeof document !== 'undefined' && document.activeElement !== ref.slider) {
        ref.slider.value = jt.targetAngleDeg.toFixed(1);
      }
      if (typeof document !== 'undefined' && document.activeElement !== ref.numInput) {
        ref.numInput.value = jt.targetAngleDeg.toFixed(1);
      }

      ref.row.classList.toggle('joint-at-limit', jt.isAtLimit || jt.isTorqueSaturated);
    });

    const grip = snap.gripper;
    const openMmEl = this.container.querySelector('#grip-open-mm');
    const forceNEl = this.container.querySelector('#grip-force-n');
    const gripBadge = this.container.querySelector('#grip-status-badge');
    const contactBanner = this.container.querySelector('#grip-contact-banner');

    if (openMmEl) {
      openMmEl.textContent = `${grip.openingMillimeters.toFixed(1)} mm`;
    }
    if (forceNEl) {
      forceNEl.textContent = `${grip.grippingForceNewtons.toFixed(1)} N (Fn: ${grip.appliedNormalForceNewtons.toFixed(1)} N)`;
    }
    if (gripBadge && contactBanner) {
      if (grip.isGrasping) {
        gripBadge.textContent = 'GRASP SECURED';
        gripBadge.className = 'badge badge-valid';
        contactBanner.className = 'feedback-banner status-valid';
        contactBanner.textContent = `Bilateral friction grasp active (Ff_max=${grip.frictionCapacityNewtons.toFixed(1)} N, obj=${grip.graspedObjectId}).`;
      } else if (grip.isSlipping) {
        gripBadge.textContent = 'SLIPPING!';
        gripBadge.className = 'badge badge-danger';
        contactBanner.className = 'feedback-banner status-danger';
        contactBanner.textContent = `Insufficient friction (${grip.frictionCapacityNewtons.toFixed(2)} N) — object slipping under gravity!`;
      } else {
        gripBadge.textContent =
          grip.leftFingerContact || grip.rightFingerContact ? 'CONTACT' : 'UNGRASPED';
        gripBadge.className = 'badge badge-info';
        contactBanner.className = 'feedback-banner status-info';
        contactBanner.textContent = `Contacts: L=${grip.leftFingerContact ? 'YES' : 'NO'} | R=${grip.rightFingerContact ? 'YES' : 'NO'}`;
      }
    }
  }
}
