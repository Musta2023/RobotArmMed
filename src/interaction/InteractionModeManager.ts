/**
 * @file InteractionModeManager.ts
 * Coordinates the Three Interaction Modes (Section 7):
 * - MODE A: 'object' (OBJECT MODE) — Select, drag, position, and rotate objects in 3D with mouse; release into physics.
 * - MODE B: 'robot'  (ROBOT MODE)  — Control the gripper 3D target position & orientation via 3D gizmo, IK, and joint motors.
 * - MODE C: 'ai'     (AI MODE)     — Disable manual manipulation; let AI policy / autonomous PickAndPlace control the robot.
 */

import * as THREE from 'three';
import { ObjectManipulator, DragPlaneMode } from './ObjectManipulator.js';
import {
  GripperTargetController,
  GizmoTransformMode
} from './GripperTargetController.js';
import { RobotModel } from '../robot/RobotModel.js';
import { InstrumentFactory } from '../objects/InstrumentFactory.js';
import { PickAndPlace } from '../manipulation/PickAndPlace.js';
import { CameraManager } from '../scene/CameraManager.js';
import { createTopDownQuaternion } from '../control/InverseKinematics.js';

export type InteractionMode = 'object' | 'robot' | 'ai';

export class InteractionModeManager {
  public mode: InteractionMode = 'object';
  public objectManipulator: ObjectManipulator;
  public gripperTargetController: GripperTargetController;
  public robotModel: RobotModel;
  public instrumentFactory: InstrumentFactory;
  public pickAndPlace: PickAndPlace;
  public cameraManager: CameraManager;
  public domElement: HTMLElement | null;

  public statusText: string = 'MODE OBJET : Survoler et cliquer sur un objet pour le déplacer en 3D.';
  private raycaster: THREE.Raycaster = new THREE.Raycaster();

  constructor(params: {
    objectManipulator: ObjectManipulator;
    gripperTargetController: GripperTargetController;
    robotModel: RobotModel;
    instrumentFactory: InstrumentFactory;
    pickAndPlace: PickAndPlace;
    cameraManager: CameraManager;
    domElement?: HTMLElement | null;
  }) {
    this.objectManipulator = params.objectManipulator;
    this.gripperTargetController = params.gripperTargetController;
    this.robotModel = params.robotModel;
    this.instrumentFactory = params.instrumentFactory;
    this.pickAndPlace = params.pickAndPlace;
    this.cameraManager = params.cameraManager;
    this.domElement = params.domElement ?? null;

    this.setMode('object');

    if (typeof window !== 'undefined' && this.domElement) {
      this._bindViewportEvents(this.domElement);
    }
  }

  /**
   * Switches between 'object' (Mode A), 'robot' (Mode B), and 'ai' (Mode C)
   * without breaking or pausing the physics simulation.
   */
  public setMode(nextMode: InteractionMode): void {
    // Always cleanly release any active object drag when switching modes
    if (this.objectManipulator.isDragging) {
      this.objectManipulator.releaseObject();
    }
    this.objectManipulator.setHoveredObject(null);

    this.mode = nextMode;

    if (nextMode === 'object') {
      this.gripperTargetController.setGizmoEnabled(false);
      if (this.cameraManager.orbitControls) {
        this.cameraManager.orbitControls.enabled = true;
      }
      this.statusText =
        'MODE OBJET : Clic gauche pour déplacer en 3D (Shift = Vertical, Molette = Hauteur/Profondeur).';
    } else if (nextMode === 'robot') {
      this.pickAndPlace.reset();
      this.gripperTargetController.setGizmoEnabled(true);
      this.statusText =
        'MODE ROBOT : Manipuler le Gizmo 3D de la pince (W = Déplacer, E = Pivoter, Touche G = Pince).';
    } else if (nextMode === 'ai') {
      this.gripperTargetController.setGizmoEnabled(false);
      if (this.cameraManager.orbitControls) {
        this.cameraManager.orbitControls.enabled = true;
      }
      this.statusText =
        'MODE IA : Manipulation manuelle désactivée. Contrôle autonome par politique IA actif.';
    }
  }

  public setGizmoTransformMode(transformMode: GizmoTransformMode): void {
    this.gripperTargetController.setGizmoMode(transformMode);
    if (this.mode !== 'robot') {
      this.setMode('robot');
    }
  }

  public setDragPlaneMode(planeMode: DragPlaneMode): void {
    this.objectManipulator.setDragPlaneMode(
      planeMode,
      this.cameraManager.mainCamera
    );
  }

  /**
   * Checks if a normalized device coordinate ray hits the robot's gripper mesh.
   */
  public raycastGripper(ndcX: number, ndcY: number): boolean {
    this.raycaster.setFromCamera(
      new THREE.Vector2(ndcX, ndcY),
      this.cameraManager.mainCamera
    );
    this.robotModel.gripper.root.updateMatrixWorld(true);
    const hits = this.raycaster.intersectObject(this.robotModel.gripper.root, true);
    return hits.length > 0;
  }

  private _computeNDC(event: PointerEvent | WheelEvent, dom: HTMLElement): [number, number] {
    const rect = dom.getBoundingClientRect();
    const x = ((event.clientX - rect.left) / Math.max(rect.width, 1)) * 2 - 1;
    const y = -((event.clientY - rect.top) / Math.max(rect.height, 1)) * 2 + 1;
    return [x, y];
  }

  private _bindViewportEvents(dom: HTMLElement): void {
    let isRightClickRotating = false;

    // Prevent default context menu on 3D canvas so right-click is fully usable for rotation
    dom.addEventListener('contextmenu', (e: MouseEvent) => {
      e.preventDefault();
    });

    dom.addEventListener('pointermove', (e: PointerEvent) => {
      if (this.mode === 'ai') return;
      const [ndcX, ndcY] = this._computeNDC(e, dom);

      if (this.mode === 'object') {
        if (isRightClickRotating) {
          const targetId =
            this.objectManipulator.grabbedObjectId ??
            this.objectManipulator.selectedObjectId ??
            this.instrumentFactory.activeInstrumentId;
          if (targetId) {
            this.objectManipulator.rotateWithMouseDelta(
              e.movementX,
              e.movementY,
              targetId,
              this.cameraManager.mainCamera
            );
            this.statusText = `Rotation de [${targetId}] en 3D (Lacet/Tangage)`;
          }
        } else if (this.objectManipulator.isDragging) {
          const desiredMode: DragPlaneMode = e.shiftKey
            ? 'vertical'
            : this.objectManipulator.dragPlaneMode;
          if (desiredMode !== this.objectManipulator.dragPlaneMode) {
            this.objectManipulator.setDragPlaneMode(
              desiredMode,
              this.cameraManager.mainCamera
            );
          }
          const pos = this.objectManipulator.dragByRay(
            ndcX,
            ndcY,
            this.cameraManager.mainCamera
          );
          if (pos) {
            this.statusText = `Déplacement de [${this.objectManipulator.grabbedObjectId}] @ [${pos[0].toFixed(3)}, ${pos[1].toFixed(3)}, ${pos[2].toFixed(3)}] m`;
          }
        } else {
          const hovered = this.objectManipulator.updateHoverByRay(
            ndcX,
            ndcY,
            this.cameraManager.mainCamera
          );
          dom.style.cursor = hovered ? 'grab' : 'default';
        }
      }
    });

    dom.addEventListener('pointerdown', (e: PointerEvent) => {
      if (this.mode === 'ai') return;
      const [ndcX, ndcY] = this._computeNDC(e, dom);

      if (this.mode === 'object') {
        if (e.button === 0) {
          // Left-click: grab object for translation
          const grabbed = this.objectManipulator.grabObjectByRay(
            ndcX,
            ndcY,
            this.cameraManager.mainCamera
          );
          if (grabbed) {
            dom.style.cursor = 'grabbing';
            if (this.cameraManager.orbitControls) {
              this.cameraManager.orbitControls.enabled = false;
            }
            const p = grabbed.getPosition();
            this.statusText = `Objet saisi : ${grabbed.config.name} (${grabbed.config.massKg.toFixed(2)} kg) @ [${p[0].toFixed(3)}, ${p[1].toFixed(3)}, ${p[2].toFixed(3)}] m`;
            return;
          }

          // If user clicked directly on the robot gripper while in Object Mode, switch to Robot Mode
          if (this.raycastGripper(ndcX, ndcY)) {
            this.setMode('robot');
          }
        } else if (e.button === 2) {
          // Right-click: direct 3D interactive rotation
          e.preventDefault();
          const hit = this.objectManipulator.raycastObject(
            ndcX,
            ndcY,
            this.cameraManager.mainCamera
          );
          const targetId =
            hit?.instrument.id ??
            this.objectManipulator.grabbedObjectId ??
            this.objectManipulator.selectedObjectId ??
            this.instrumentFactory.activeInstrumentId;

          if (targetId) {
            this.objectManipulator.selectedObjectId = targetId;
            this.instrumentFactory.activeInstrumentId = targetId;
            isRightClickRotating = true;
            dom.style.cursor = 'crosshair';
            if (this.cameraManager.orbitControls) {
              this.cameraManager.orbitControls.enabled = false;
            }
            this.statusText = `Rotation de [${targetId}] par glisser-droit (Ctrl+Molette, touches R/T, ou Gizmo E)...`;
          }
        }
      }
    });

    window.addEventListener('pointerup', (e: PointerEvent) => {
      if (isRightClickRotating) {
        isRightClickRotating = false;
        if (dom) dom.style.cursor = 'default';
        if (this.cameraManager.orbitControls) {
          this.cameraManager.orbitControls.enabled = true;
        }
      }
      if (this.objectManipulator.isDragging) {
        const released = this.objectManipulator.releaseObject();
        if (dom) dom.style.cursor = 'default';
        if (this.cameraManager.orbitControls) {
          this.cameraManager.orbitControls.enabled = true;
        }
        if (released) {
          this.statusText = `Objet relâché : ${released.config.name} — chute naturelle sous gravité.`;
        }
      }
    });

    dom.addEventListener(
      'wheel',
      (e: WheelEvent) => {
        if (this.mode === 'object') {
          if (e.ctrlKey || e.altKey) {
            e.preventDefault();
            const deltaYaw = e.deltaY < 0 ? 15 : -15;
            this.objectManipulator.rotateSelectedYaw(deltaYaw);
            this.statusText = `Rotation Lacet : ${deltaYaw > 0 ? '+' : ''}${deltaYaw}° (Ctrl/Alt + Molette)`;
            return;
          }
          if (this.objectManipulator.isDragging) {
            e.preventDefault();
            const stepMeters = e.deltaY < 0 ? 0.018 : -0.018;
            if (e.shiftKey) {
              this.objectManipulator.adjustGrabbedDepth(stepMeters);
            } else {
              this.objectManipulator.adjustGrabbedHeight(stepMeters);
            }
          }
        }
      },
      { passive: false }
    );

    window.addEventListener('keydown', (e: KeyboardEvent) => {
      if (
        document.activeElement &&
        (document.activeElement.tagName === 'INPUT' ||
          document.activeElement.tagName === 'SELECT')
      ) {
        return;
      }
      if (e.key === '1') this.setMode('object');
      else if (e.key === '2') this.setMode('robot');
      else if (e.key === '3') this.setMode('ai');
      else if (e.key.toLowerCase() === 'w') {
        this.setGizmoTransformMode('translate');
      } else if (e.key.toLowerCase() === 'e') {
        this.setGizmoTransformMode('rotate');
      } else if (this.mode === 'object' && e.key.toLowerCase() === 'r') {
        const deltaYaw = e.shiftKey ? -15 : 15;
        this.objectManipulator.rotateSelectedYaw(deltaYaw);
        this.statusText = `Rotation Lacet : ${deltaYaw > 0 ? '+' : ''}${deltaYaw}° (Touche R)`;
      } else if (this.mode === 'object' && e.key.toLowerCase() === 't') {
        const deltaPitch = e.shiftKey ? -15 : 15;
        this.objectManipulator.rotateSelectedPitch(deltaPitch);
        this.statusText = `Rotation Tangage : ${deltaPitch > 0 ? '+' : ''}${deltaPitch}° (Touche T)`;
      } else if (e.key.toLowerCase() === 'g') {
        if (this.robotModel.gripper.isClosingCommandActive) {
          this.robotModel.gripper.open(false);
        } else {
          this.robotModel.gripper.close(false);
        }
      }
    });
  }

  /**
   * Generic, object-independent helper that commands the robot to approach any
   * compatible object in the workspace based on its live 3D position and orientation.
   */
  public commandRobotToApproachObject(objectId?: string): {
    accepted: boolean;
    message: string;
    targetPosition: [number, number, number];
  } {
    const inst = objectId
      ? this.instrumentFactory.instruments.get(objectId)
      : this.instrumentFactory.getActiveInstrument();

    if (!inst) {
      return {
        accepted: false,
        message: 'No target object found in workspace.',
        targetPosition: [0, 0, 0]
      };
    }

    const graspInfo = inst.getGraspabilityInfo(
      this.robotModel.config.gripper.maxOpeningMeters,
      this.robotModel.config.gripper.maxGrippingForceNewtons
    );
    const livePos = inst.getPosition();
    const liveQuat = inst.getOrientation();

    if (!graspInfo.isGraspable) {
      return {
        accepted: false,
        message: graspInfo.reason,
        targetPosition: livePos
      };
    }

    // Extract object yaw around Y so the gripper aligns its jaws with the object's orientation
    const euler = new THREE.Euler().setFromQuaternion(
      new THREE.Quaternion(liveQuat[0], liveQuat[1], liveQuat[2], liveQuat[3]),
      'YXZ'
    );
    const alignedQuat = createTopDownQuaternion(euler.y);

    this.robotModel.gripper.open(false);
    const ikRes = this.gripperTargetController.setTargetPose(livePos, alignedQuat);

    return {
      accepted: ikRes.isReachable,
      message: ikRes.message,
      targetPosition: livePos
    };
  }
}
