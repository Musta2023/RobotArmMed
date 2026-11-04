/**
 * @file FailureScenarioController.ts
 * Specialized robotic failure injection controller designed to systematically simulate,
 * trigger, and collect labeled physical failure demonstrations for machine learning
 * (anomaly detection, recovery policy learning, and safety-constrained offline RL).
 *
 * Supported Failure Scenarios:
 * 1. MISSED_APPROACH    - Spatial offset during approach, jaws close on empty air (reach/grasp fail)
 * 2. SLIP_LOW_FRICTION  - Grip force insufficient against gravity, object slips during lift
 * 3. DROP_IN_TRANSIT    - Premature gripper opening mid-transit at safe hover height
 * 4. COLLISION_OBSTACLE - Low-altitude transit trajectory colliding with table obstacles
 * 5. JOINT_LIMIT_REACH  - Demands kinematics beyond joint safety limits
 * 6. EXCESSIVE_TORQUE   - Injects overloaded payload mass triggering motor torque saturation
 * 7. TIPPED_PLACEMENT   - High-elevation / tilted release causing object to tumble off target
 * 8. RANDOM_FAILURE     - Pseudo-randomly selects one of the deterministic failure modes
 */

import * as THREE from 'three';
import { Gripper } from '../robot/Gripper.js';
import { MotionController } from '../control/MotionController.js';
import { FailureDetector, SurgicalErrorCode } from '../recovery/FailureDetector.js';
import { InstrumentFactory, ActiveSurgicalInstrument } from '../objects/InstrumentFactory.js';
import { EpisodeRecorder } from '../ai/EpisodeRecorder.js';

export type FailureScenarioType =
  | 'NONE'
  | 'MISSED_APPROACH'
  | 'SLIP_LOW_FRICTION'
  | 'DROP_IN_TRANSIT'
  | 'COLLISION_OBSTACLE'
  | 'JOINT_LIMIT_REACH'
  | 'EXCESSIVE_TORQUE'
  | 'TIPPED_PLACEMENT'
  | 'KNOCK_ON_APPROACH'
  | 'ORIENTATION_MISALIGNMENT'
  | 'DYNAMIC_JERK_SLIP'
  | 'INCOMPLETE_RELEASE'
  | 'RANDOM_FAILURE';

export interface FailureScenarioConfig {
  type: FailureScenarioType;
  labelFr: string;
  expectedErrorCode: SurgicalErrorCode;
  description: string;
}

export const FAILURE_SCENARIO_DEFINITIONS: Record<FailureScenarioType, FailureScenarioConfig> = {
  NONE: {
    type: 'NONE',
    labelFr: 'Aucun (Nominal)',
    expectedErrorCode: 'grasp_failed',
    description: 'Trajectoire nominale sans injection de défaillance.'
  },
  MISSED_APPROACH: {
    type: 'MISSED_APPROACH',
    labelFr: '1. Approche Décalée (Prise Manquée)',
    expectedErrorCode: 'grasp_failed',
    description: 'Le robot descend avec un décalage latéral de 9.5 cm et ferme la pince dans le vide.'
  },
  SLIP_LOW_FRICTION: {
    type: 'SLIP_LOW_FRICTION',
    labelFr: '2. Glissement par Force Insuffisante',
    expectedErrorCode: 'object_slipping',
    description: 'La pince applique une force de serrage minime (0.4 N). L\'objet glisse et chute sous gravité lors du levage.'
  },
  DROP_IN_TRANSIT: {
    type: 'DROP_IN_TRANSIT',
    labelFr: '3. Chute en Cours de Transport',
    expectedErrorCode: 'object_falling',
    description: 'La pince s\'ouvre prématurément à mi-parcours à 24 cm de hauteur, provoquant la chute libre de l\'objet.'
  },
  COLLISION_OBSTACLE: {
    type: 'COLLISION_OBSTACLE',
    labelFr: '4. Collision avec Obstacle',
    expectedErrorCode: 'collision_detected',
    description: 'Trajectoire de transit à basse altitude (5 cm) heurtant le diviseur ou les objets environnants.'
  },
  JOINT_LIMIT_REACH: {
    type: 'JOINT_LIMIT_REACH',
    labelFr: '5. Dépassement de Butée Articulaire',
    expectedErrorCode: 'joint_limit_violation',
    description: 'Commande une cible cartésienne hors portée physique, déclenchant le bridage de sécurité articulaire.'
  },
  EXCESSIVE_TORQUE: {
    type: 'EXCESSIVE_TORQUE',
    labelFr: '6. Dépassement de Couple Moteur (Surcharge)',
    expectedErrorCode: 'excessive_motor_torque',
    description: 'Simule une surcharge mécanique (3.5 kg) excédant la capacité de couple maximal des servomoteurs J2/J3.'
  },
  TIPPED_PLACEMENT: {
    type: 'TIPPED_PLACEMENT',
    labelFr: '7. Dépose Instable & Basculement',
    expectedErrorCode: 'placement_failed',
    description: 'Relâchement de l\'objet à 12 cm de hauteur avec un angle de 35°, provoquant le basculement hors du plateau.'
  },
  KNOCK_ON_APPROACH: {
    type: 'KNOCK_ON_APPROACH',
    labelFr: '8. Percussion / Collision à l\'Approche',
    expectedErrorCode: 'collision_detected',
    description: 'Descente trop basse (-4.5 cm) heurtant violemment l\'instrument avant la saisie et le renversant.'
  },
  ORIENTATION_MISALIGNMENT: {
    type: 'ORIENTATION_MISALIGNMENT',
    labelFr: '9. Défaut d\'Orientation des Mors (+45°)',
    expectedErrorCode: 'unstable_manipulation',
    description: 'Désalignement en lacet/roulis de 45° provoquant une prise asymétrique instable de l\'instrument.'
  },
  DYNAMIC_JERK_SLIP: {
    type: 'DYNAMIC_JERK_SLIP',
    labelFr: '10. Décrochage par À-Coup Dynamique (Jerk)',
    expectedErrorCode: 'object_falling',
    description: 'Accélération brusque en cours de transit surpassant le frottement statique et éjectant l\'objet.'
  },
  INCOMPLETE_RELEASE: {
    type: 'INCOMPLETE_RELEASE',
    labelFr: '11. Accrochage au Relâchement (Pince Coincée)',
    expectedErrorCode: 'placement_failed',
    description: 'Ouverture partielle des mors (1.2 cm) bloquant l\'objet et l\'arrachant du plateau lors du retour Home.'
  },
  RANDOM_FAILURE: {
    type: 'RANDOM_FAILURE',
    labelFr: '🎲 Scénario Aléatoire',
    expectedErrorCode: 'grasp_failed',
    description: 'Sélectionne aléatoirement l\'un des 7 scénarios de défaillance physique.'
  }
};

export class FailureScenarioController {
  public activeType: FailureScenarioType = 'NONE';
  public resolvedType: FailureScenarioType = 'NONE';
  public isInjecting: boolean = false;
  public faultTriggered: boolean = false;
  public transportElapsedSeconds: number = 0;

  private gripper: Gripper;
  private motionController: MotionController;
  private failureDetector: FailureDetector;
  private instrumentFactory: InstrumentFactory;
  private episodeRecorder?: EpisodeRecorder;

  constructor(params: {
    gripper: Gripper;
    motionController: MotionController;
    failureDetector: FailureDetector;
    instrumentFactory: InstrumentFactory;
    episodeRecorder?: EpisodeRecorder;
  }) {
    this.gripper = params.gripper;
    this.motionController = params.motionController;
    this.failureDetector = params.failureDetector;
    this.instrumentFactory = params.instrumentFactory;
    this.episodeRecorder = params.episodeRecorder;
  }

  /**
   * Sets the desired failure scenario mode for the next execution.
   */
  public setScenarioType(type: FailureScenarioType): void {
    this.activeType = type;
    if (type === 'RANDOM_FAILURE') {
      const candidates: FailureScenarioType[] = [
        'MISSED_APPROACH',
        'SLIP_LOW_FRICTION',
        'DROP_IN_TRANSIT',
        'COLLISION_OBSTACLE',
        'JOINT_LIMIT_REACH',
        'EXCESSIVE_TORQUE',
        'TIPPED_PLACEMENT',
        'KNOCK_ON_APPROACH',
        'ORIENTATION_MISALIGNMENT',
        'DYNAMIC_JERK_SLIP',
        'INCOMPLETE_RELEASE'
      ];
      this.resolvedType = candidates[Math.floor(Math.random() * candidates.length)];
    } else {
      this.resolvedType = type;
    }
  }

  /**
   * Arms and starts the failure scenario.
   */
  public armScenario(type?: FailureScenarioType): void {
    if (type) {
      this.setScenarioType(type);
    }
    this.isInjecting = this.resolvedType !== 'NONE';
    this.faultTriggered = false;
    this.transportElapsedSeconds = 0;
  }

  public disarm(): void {
    this.activeType = 'NONE';
    this.resolvedType = 'NONE';
    this.isInjecting = false;
    this.faultTriggered = false;
    this.transportElapsedSeconds = 0;
    // Restore default normal gripping force
    this.gripper.grippingForceNewtons = this.gripper.config.defaultGrippingForceNewtons;
  }

  /**
   * Modifies the approach hover/descent target position if MISSED_APPROACH, KNOCK_ON_APPROACH, or JOINT_LIMIT_REACH is active.
   */
  public getModifiedApproachTarget(
    nominalTarget: [number, number, number]
  ): [number, number, number] {
    if (!this.isInjecting) return [...nominalTarget];

    if (this.resolvedType === 'MISSED_APPROACH') {
      // Shift 9.5 cm along X and 3 cm along Z so jaws miss the object
      return [
        nominalTarget[0] + 0.095,
        nominalTarget[1],
        nominalTarget[2] + 0.030
      ];
    }

    if (this.resolvedType === 'KNOCK_ON_APPROACH') {
      this.faultTriggered = true;
      // Force descent 4.5 cm below nominal, impacting table / object directly
      return [
        nominalTarget[0],
        Math.max(0.005, nominalTarget[1] - 0.045),
        nominalTarget[2]
      ];
    }

    if (this.resolvedType === 'JOINT_LIMIT_REACH') {
      // Command an extreme unreachable point beyond the 0.79 m maximum physical envelope
      return [
        nominalTarget[0] + 0.55,
        nominalTarget[1] + 0.40,
        nominalTarget[2] + 0.45
      ];
    }

    return [...nominalTarget];
  }

  /**
   * Modifies the grasp orientation if ORIENTATION_MISALIGNMENT is active.
   */
  public getModifiedGraspOrientation(nominalQuat: THREE.Quaternion): THREE.Quaternion {
    if (!this.isInjecting || this.resolvedType !== 'ORIENTATION_MISALIGNMENT') {
      return nominalQuat.clone();
    }
    this.faultTriggered = true;
    // 45-degree (0.785 rad) misalignment on yaw/roll
    const errQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 4);
    return nominalQuat.clone().multiply(errQuat);
  }

  /**
   * Hook called immediately prior to closing the gripper in state 'GRASP'.
   */
  public onGraspEnter(): void {
    if (!this.isInjecting) return;

    if (this.resolvedType === 'SLIP_LOW_FRICTION') {
      // Drastically reduce normal gripping force to 0.35 N (well below weightLoad / (2 * mu))
      this.gripper.grippingForceNewtons = 0.35;
      this.faultTriggered = true;
    }
  }

  /**
   * Hook called on each fixed timestep during state 'TRANSPORT'.
   * Handles mid-transit premature drops, dynamic jerks, or low-altitude collisions.
   */
  public onTransportStep(dt: number, currentTcpY: number): void {
    if (!this.isInjecting) return;

    this.transportElapsedSeconds += dt;

    if (this.resolvedType === 'DROP_IN_TRANSIT' && !this.faultTriggered) {
      // Trigger premature jaw opening 0.6 seconds into the transport phase
      if (this.transportElapsedSeconds >= 0.60) {
        this.faultTriggered = true;
        this.gripper.open(true);
      }
    }

    if (this.resolvedType === 'DYNAMIC_JERK_SLIP' && !this.faultTriggered) {
      // Sudden dynamic jerk mid-turn: sharp loss of friction
      if (this.transportElapsedSeconds >= 0.35) {
        this.faultTriggered = true;
        this.gripper.grippingForceNewtons = 0.10;
        this.gripper.open(true);
      }
    }
  }

  /**
   * Modifies the transport trajectory altitude if COLLISION_OBSTACLE is active.
   */
  public getModifiedTransportAltitude(nominalSafeHoverY: number): number {
    if (!this.isInjecting) return nominalSafeHoverY;

    if (this.resolvedType === 'COLLISION_OBSTACLE') {
      // Dangerously low transit altitude (0.045 m) that collides with the central boundary
      this.faultTriggered = true;
      return 0.045;
    }

    return nominalSafeHoverY;
  }

  /**
   * Modifies placement release pose and altitude if TIPPED_PLACEMENT is active.
   */
  public getModifiedPlacementPose(
    nominalTargetSlot: [number, number, number],
    nominalQuat: THREE.Quaternion
  ): {
    targetPosition: [number, number, number];
    targetOrientation: THREE.Quaternion;
  } {
    if (!this.isInjecting || this.resolvedType !== 'TIPPED_PLACEMENT') {
      return {
        targetPosition: [...nominalTargetSlot],
        targetOrientation: nominalQuat.clone()
      };
    }

    this.faultTriggered = true;
    // Release from high altitude (12 cm above tray) with a 38-degree tilt angle
    const tiltEuler = new THREE.Euler(0.66, 0, 0.45);
    const tiltQuat = new THREE.Quaternion().setFromEuler(tiltEuler).multiply(nominalQuat);

    return {
      targetPosition: [
        nominalTargetSlot[0] + 0.025,
        nominalTargetSlot[1] + 0.120,
        nominalTargetSlot[2] + 0.020
      ],
      targetOrientation: tiltQuat
    };
  }

  /**
   * Determines whether gripper should remain jammed / partially closed during PLACE release.
   */
  public shouldPreventFullRelease(): boolean {
    if (!this.isInjecting || this.resolvedType !== 'INCOMPLETE_RELEASE') {
      return false;
    }
    this.faultTriggered = true;
    return true;
  }

  /**
   * Returns current active payload mass override (used for EXCESSIVE_TORQUE simulation).
   */
  public getSimulatedPayloadMassKg(nominalMassKg: number): number {
    if (!this.isInjecting) return nominalMassKg;

    if (this.resolvedType === 'EXCESSIVE_TORQUE') {
      this.faultTriggered = true;
      // Injects a 3.8 kg excessive load, causing motor torque saturation on J2/J3
      return 3.8;
    }

    return nominalMassKg;
  }

  /**
   * Returns current state summary for UI telemetry.
   */
  public getStatus(): {
    activeType: FailureScenarioType;
    resolvedType: FailureScenarioType;
    isInjecting: boolean;
    faultTriggered: boolean;
    labelFr: string;
    description: string;
  } {
    const def = FAILURE_SCENARIO_DEFINITIONS[this.resolvedType] ?? FAILURE_SCENARIO_DEFINITIONS.NONE;
    return {
      activeType: this.activeType,
      resolvedType: this.resolvedType,
      isInjecting: this.isInjecting,
      faultTriggered: this.faultTriggered,
      labelFr: def.labelFr,
      description: def.description
    };
  }
}
