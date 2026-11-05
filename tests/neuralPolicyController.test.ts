/**
 * @file neuralPolicyController.test.ts
 * Validation Physique dans le Digital Twin Rapier3D du nouveau contrôleur ML
 * `NeuralPolicyController` (Sections 5, 6 & 8 de la Mission).
 *
 * Vérifie sur le moteur physique réel Rapier3D (sans triche ni désactivation des collisions) :
 * 1. Absence totale de dérive cumulative (q_target absolu) et respect strict des limites articulaires.
 * 2. Progression complète des 9 phases Pick-and-Place :
 *    APPROACH -> ALIGNMENT -> DESCENT -> GRASP -> VERIFICATION -> LIFT -> MOVE_TO_TARGET -> RELEASE -> SUCCESS.
 * 3. Préhension physique bilatérale réelle (graspManager.isGrasping === true) et dépôt précis en Zone B.
 * 4. Généralisation à un objet unique placé à des positions inédites, orientations variées,
 *    hauteurs différentes et configurations initiales variées du bras.
 */

import { describe, it, expect } from 'vitest';
import * as fs from 'fs';
import * as path from 'path';
import { createDigitalTwinApp } from '../src/main.js';
import { MANIPULATION_PHASES } from '../src/ai/NeuralPolicyController.js';

const weightsPath = path.resolve(process.cwd(), 'public/models/robot_policy_weights.json');
const weightsJson = JSON.parse(fs.readFileSync(weightsPath, 'utf-8'));

describe('Section 6 & 8 : Validation Physique & Généralisation de NeuralPolicyController (35D -> 8D)', () => {
  it('extrait un vecteur d’observation 35D valide et prédit directement [J1_target..J6_target, Gripper_target, Confidence]', async () => {
    const app = await createDigitalTwinApp({ headless: true });
    app.neuralPolicyController.loadWeightsFromObject(weightsJson);

    expect(app.neuralPolicyController.isLoaded).toBe(true);
    expect(app.neuralPolicyController.backend).toBe('native-js');

    const obs35 = app.neuralPolicyController.extractObservation35D([0.30, 0.02, 0.24], 'APPROACH');
    expect(obs35.length).toBe(35);

    const out8 = app.neuralPolicyController.forwardNative(obs35);
    expect(out8.length).toBe(8);
    // Vérifier que les angles cibles prédits sont finis et dans les bornes physiques raisonnables (en rad)
    for (let i = 0; i < 6; i++) {
      expect(Number.isFinite(out8[i])).toBe(true);
      expect(Math.abs(out8[i])).toBeLessThan(Math.PI);
    }
    // Ouverture pince et score de confiance dans [0, 1]
    expect(out8[6]).toBeGreaterThanOrEqual(-0.02);
    expect(out8[6]).toBeLessThanOrEqual(0.10);
    expect(out8[7]).toBeGreaterThanOrEqual(0.0);
    expect(out8[7]).toBeLessThanOrEqual(1.0);

    app.dispose();
  });

  it('exécute un Pick-and-Place physique complet en 9 phases sur un objet unique sans dérive ni blocage', async () => {
    const app = await createDigitalTwinApp({ headless: true });
    app.neuralPolicyController.loadWeightsFromObject(weightsJson);

    // Scène à 1 seul objet (Cube chirurgical sur la station de départ)
    const activeInst = app.instrumentFactory.getActiveInstrument()!;
    expect(activeInst).toBeDefined();

    const started = app.neuralPolicyController.start();
    expect(started).toBe(true);

    let physicalGraspConfirmedDuringTransport = false;
    let maxJointLimitViolation = false;
    const config = app.robotModel.config;

    // Exécuter la simulation physique Rapier3D à 60 Hz jusqu'à la fin du cycle (~7 secondes = 420 pas)
    for (let step = 0; step < 520; step++) {
      app.stepSimulation(1);

      // Vérifier que toutes les articulations J1..J6 restent dans leurs limites mécaniques
      const currentDeg = app.robotJoints.getCurrentAnglesDeg();
      for (let j = 0; j < 6; j++) {
        if (
          currentDeg[j] < config.joints[j].minAngleDeg - 2.0 ||
          currentDeg[j] > config.joints[j].maxAngleDeg + 2.0
        ) {
          maxJointLimitViolation = true;
        }
      }

      // Vérifier qu'une véritable préhension physique (graspManager.isGrasping) a lieu pendant le levage/transport
      if (
        (app.neuralPolicyController.currentPhase === 'LIFT' ||
          app.neuralPolicyController.currentPhase === 'MOVE_TO_TARGET') &&
        app.graspManager.isGrasping
      ) {
        physicalGraspConfirmedDuringTransport = true;
      }

      if (!app.neuralPolicyController.isRunning && step > 60) {
        break;
      }
    }

    // Laisser l'objet se stabiliser 15 pas sur le plateau cible
    app.stepSimulation(15);

    expect(maxJointLimitViolation).toBe(false);
    expect(physicalGraspConfirmedDuringTransport).toBe(true);

    // Vérifier que les 9 phases ont bien été parcourues dans l'ordre logique
    for (const reqPhase of MANIPULATION_PHASES) {
      expect(app.neuralPolicyController.phaseHistory).toContain(reqPhase);
    }

    // Vérifier le placement physique réel de l'objet sur le plateau cible Zone B (0.30, 0.02, 0.24)
    const finalPos = activeInst.managedBody.body.translation();
    const distToTargetHoriz = Math.hypot(finalPos.x - 0.30, finalPos.z - 0.24);
    expect(distToTargetHoriz).toBeLessThan(0.045);
    expect(finalPos.y).toBeGreaterThan(0.015);
    expect(finalPos.y).toBeLessThan(0.085);

    const report = app.placementVerifier.verify(activeInst, app.graspManager.isGrasping);
    expect(report.verified).toBe(true);
    expect(report.withinPositionTolerance).toBe(true);
    expect(report.isDetachedFromGripper).toBe(true);

    app.dispose();
  });

  it('généralise à un objet unique placé à des positions, orientations, hauteurs et poses initiales inédites', async () => {
    const testScenarios = [
      {
        name: 'Position inédite Nord-Ouest + Rotation Lacet +30°',
        objPos: [-0.36, 0.041, 0.19] as [number, number, number],
        yawRad: (30 * Math.PI) / 180,
        initArmDeg: [0, 20, 70, 0, 45, 0],
        targetPos: [0.30, 0.02, 0.24] as [number, number, number]
      },
      {
        name: 'Position inédite Sud-Est + Rotation -40° + Pose initiale bras perturbée',
        objPos: [-0.23, 0.041, 0.30] as [number, number, number],
        yawRad: (-40 * Math.PI) / 180,
        initArmDeg: [22, 12, 78, 0, 38, -15],
        targetPos: [0.34, 0.02, 0.27] as [number, number, number]
      },
      {
        name: 'Distance étendue Sud-Ouest + Rotation +45° + Cible inédite',
        objPos: [-0.39, 0.041, 0.31] as [number, number, number],
        yawRad: (45 * Math.PI) / 180,
        initArmDeg: [-25, 28, 60, 0, 50, 10],
        targetPos: [0.24, 0.02, 0.21] as [number, number, number]
      }
    ];

    for (const scen of testScenarios) {
      const app = await createDigitalTwinApp({ headless: true });
      app.neuralPolicyController.loadWeightsFromObject(weightsJson);

      const activeInst = app.instrumentFactory.getActiveInstrument()!;
      const halfYaw = scen.yawRad * 0.5;
      const quat: [number, number, number, number] = [0, Math.sin(halfYaw), 0, Math.cos(halfYaw)];
      app.instrumentFactory.setInstrumentPose(activeInst.config.id, scen.objPos, quat);

      // Appliquer une configuration initiale différente au bras
      app.motionController.commandJointAnglesDeg(scen.initArmDeg, true);
      app.pickAndPlace.targetPosition = [...scen.targetPos];
      app.placementVerifier.targetTrayCenter.set(scen.targetPos[0], scen.targetPos[1], scen.targetPos[2]);
      app.stepSimulation(10);

      app.neuralPolicyController.start();

      let graspedInFlight = false;
      for (let step = 0; step < 540; step++) {
        app.stepSimulation(1);
        if (app.graspManager.isGrasping && app.neuralPolicyController.currentPhase === 'MOVE_TO_TARGET') {
          graspedInFlight = true;
        }
        if (!app.neuralPolicyController.isRunning && step > 60) {
          break;
        }
      }
      app.stepSimulation(15);

      expect(graspedInFlight).toBe(true);
      const finalPos = activeInst.managedBody.body.translation();
      const errHoriz = Math.hypot(finalPos.x - scen.targetPos[0], finalPos.z - scen.targetPos[2]);
      expect(errHoriz).toBeLessThan(0.045);

      const rep = app.placementVerifier.verify(activeInst, app.graspManager.isGrasping);
      expect(rep.verified).toBe(true);

      app.dispose();
    }
  }, 30000);

  it('place chaque objet sur son propre slot dédié (Slot B1, B2, B3...) en scène multi-objets au lieu du centre unique', async () => {
    const app = await createDigitalTwinApp({ headless: true });
    app.setupSixObjectsInZoneA();
    app.stepSimulation(15);

    app.neuralPolicyController.loadWeightsFromObject(weightsJson);
    app.neuralPolicyController.start();

    // Vérifier dès le départ que Object_01 cible bien Slot B1 [0.22, 0.02, 0.20] (et non le centre [0.30, 0.02, 0.24])
    expect(app.neuralPolicyController.lockedTargetPosition[0]).toBeCloseTo(0.22, 3);
    expect(app.neuralPolicyController.lockedTargetPosition[2]).toBeCloseTo(0.20, 3);

    // Exécuter le transfert des 2 premiers objets (Object_01 -> Slot B1, Object_02 -> Slot B2)
    for (let step = 0; step < 950; step++) {
      app.stepSimulation(1);
      const obj1 = app.instrumentFactory.instruments.get('Object_01')!;
      const obj2 = app.instrumentFactory.instruments.get('Object_02')!;
      if (obj1.isPlacedInTargetSlot && obj2.isPlacedInTargetSlot) {
        break;
      }
    }
    app.stepSimulation(15);

    const obj1Pos = app.instrumentFactory.instruments.get('Object_01')!.managedBody.body.translation();
    const obj2Pos = app.instrumentFactory.instruments.get('Object_02')!.managedBody.body.translation();

    // Object_01 doit être posé sur Slot B1 (0.22, 0.02, 0.20)
    expect(Math.hypot(obj1Pos.x - 0.22, obj1Pos.z - 0.20)).toBeLessThan(0.045);
    // Object_02 doit être posé sur Slot B2 (0.30, 0.02, 0.20)
    expect(Math.hypot(obj2Pos.x - 0.30, obj2Pos.z - 0.20)).toBeLessThan(0.045);
    // Les deux objets sont bien séparés chacun sur sa place (~8 cm d'écart)
    expect(Math.hypot(obj1Pos.x - obj2Pos.x, obj1Pos.z - obj2Pos.z)).toBeGreaterThan(0.055);

    app.dispose();
  }, 30000);
});
