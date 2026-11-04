/**
 * @file collectMixedDataset.ts
 * Generates a balanced, production-ready dataset combining both:
 * 1. Nominal Expert Demonstrations (successes across multiple difficulty levels)
 * 2. Physical & Kinematic Failure Scenarios (the 11 labeled failure modes)
 *
 * Usage:
 *   npx vite-node scripts/collectMixedDataset.ts
 */

import * as fs from 'fs';
import * as path from 'path';
import { createDigitalTwinApp } from '../src/main.js';
import {
  FailureScenarioType,
  FAILURE_SCENARIO_DEFINITIONS
} from '../src/manipulation/FailureScenarioController.js';
import { ScenarioDifficultyLevel } from '../objects/ScenarioGenerator.js';

interface MixedCollectionConfig {
  nominalEpisodesCount: number;
  failureEpisodesPerType: number;
  outputDir: string;
}

const CONFIG: MixedCollectionConfig = {
  nominalEpisodesCount: 30, // 30 épisodes réussis (niveaux 1, 2, 3)
  failureEpisodesPerType: 2, // 2 répétitions x 11 pannes = 22 échecs
  outputDir: path.resolve(process.cwd(), 'datasets')
};

const FAILURE_SCENARIOS: FailureScenarioType[] = [
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

async function runMixedDatasetCollection() {
  console.log('===============================================================');
  console.log('🤖 ROBOT-TO-ACTION: COLLECTEUR DE DATASET MIXTE (SUCCÈS + ÉCHECS)');
  console.log('===============================================================');
  console.log(`Épisodes nominaux (succès) prévus : ${CONFIG.nominalEpisodesCount}`);
  console.log(`Épisodes d'échecs prévus          : ${FAILURE_SCENARIOS.length * CONFIG.failureEpisodesPerType}`);
  console.log('---------------------------------------------------------------');

  const app = await createDigitalTwinApp({ headless: true });
  const recorder = app.episodeRecorder;
  const failureController = app.pickAndPlace.failureController;
  const scenGen = app.pickAndPlace.scenarioGenerator;

  recorder.clear();

  // PARTIE 1 : Collecte des démonstrations nominales (Succès)
  console.log('\n[PHASE 1/2] Collecte des Démonstrations Nominales d\'Experts...');
  for (let i = 1; i <= CONFIG.nominalEpisodesCount; i++) {
    const episodeId = `ep_success_nom_${String(i).padStart(3, '0')}`;
    app.resetAll();
    failureController.disarm();

    // Alternance de niveaux de difficulté (1: fixe, 2: lacet aléatoire, 3: limites)
    const level = ((i % 3) + 1) as ScenarioDifficultyLevel;
    if (scenGen) {
      const scen = scenGen.generateScenario(level, 1000 + i);
      scenGen.applyScenario(scen);
    }

    recorder.startRecording(episodeId);
    app.pickAndPlace.start();

    // Avancement de la simulation jusqu'à la fin de la tâche ou timeout
    let steps = 0;
    while (steps < 250) {
      steps++;
      app.stepSimulation(1, 1 / 20);
      if (app.pickAndPlace.state === 'COMPLETE') {
        break;
      }
    }

    recorder.stopRecording('success');
    console.log(`  ✓ [${episodeId}] Niveau ${level} : Succès enregistré (${steps} frames)`);
  }

  // PARTIE 2 : Collecte des démonstrations d'échecs (Failures)
  console.log('\n[PHASE 2/2] Collecte des Démonstrations d\'Échecs Contrôlés...');
  let failIdx = 0;
  for (const scenType of FAILURE_SCENARIOS) {
    for (let rep = 1; rep <= CONFIG.failureEpisodesPerType; rep++) {
      failIdx++;
      const episodeId = `ep_fail_${scenType.toLowerCase()}_${String(rep).padStart(3, '0')}`;
      app.resetAll();
      failureController.armScenario(scenType);

      recorder.startRecording(episodeId, undefined, scenType);
      app.pickAndPlace.start();

      let steps = 0;
      while (steps < 180) {
        steps++;
        app.stepSimulation(1, 1 / 20);
        if (
          failureController.faultTriggered ||
          app.failureDetector.hasActiveFailure ||
          app.pickAndPlace.state === 'RECOVERY' ||
          app.pickAndPlace.state === 'ERROR'
        ) {
          // 8 pas supplémentaires pour enregistrer les conséquences
          for (let p = 0; p < 8; p++) app.stepSimulation(1, 1 / 20);
          break;
        }
      }

      recorder.stopRecording('failure');
      failureController.disarm();
      console.log(`  ✓ [${episodeId}] Défaillance: ${scenType} enregistrée`);
    }
  }

  // Exportation des datasets consolidés
  if (!fs.existsSync(CONFIG.outputDir)) {
    fs.mkdirSync(CONFIG.outputDir, { recursive: true });
  }

  const timestamp = Date.now();
  const csvPath = path.join(CONFIG.outputDir, `mixed_dataset_${timestamp}.csv`);
  const jsonPath = path.join(CONFIG.outputDir, `mixed_dataset_${timestamp}.json`);

  fs.writeFileSync(csvPath, recorder.exportConsolidatedCSV(), 'utf-8');
  fs.writeFileSync(jsonPath, recorder.exportConsolidatedJSON(), 'utf-8');

  console.log('\n===============================================================');
  console.log('✅ DATASET MIXTE COMPLET GÉNÉRÉ AVEC SUCCÈS !');
  console.log(`📊 Total épisodes : ${recorder.completedEpisodes.length}`);
  console.log(`📁 CSV  : ${csvPath}`);
  console.log(`📁 JSON : ${jsonPath}`);
  console.log('===============================================================');

  app.dispose();
  process.exit(0);
}

runMixedDatasetCollection().catch((err) => {
  console.error('Erreur :', err);
  process.exit(1);
});
