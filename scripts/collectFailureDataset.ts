/**
 * @file collectFailureDataset.ts
 * Automated Batch Script for Systematic Robotic Failure Dataset Collection.
 * Runs headless physical simulations across all defined failure modes
 * and exports consolidated CSV and JSON datasets for ML training.
 *
 * Usage:
 *   npx vite-node scripts/collectFailureDataset.ts
 */

import * as fs from 'fs';
import * as path from 'path';
import { createDigitalTwinApp } from '../src/main.js';
import {
  FailureScenarioType,
  FAILURE_SCENARIO_DEFINITIONS
} from '../src/manipulation/FailureScenarioController.js';

interface BatchConfig {
  episodesPerScenario: number;
  outputDir: string;
  maxStepsPerEpisode: number;
}

const CONFIG: BatchConfig = {
  episodesPerScenario: 3, // Nombre d'épisodes par type de défaillance
  outputDir: path.resolve(process.cwd(), 'datasets'),
  maxStepsPerEpisode: 200 // Limite de pas pour éviter les boucles infinies (à 20 Hz, ~10s)
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

async function runFailureBatchCollection() {
  console.log('===============================================================');
  console.log('🤖 ROBOT-TO-ACTION: COLLECTEUR AUTOMATISÉ DE DATASETS D\'ÉCHEC');
  console.log('===============================================================');
  console.log(`Scénarios à exécuter : ${FAILURE_SCENARIOS.length}`);
  console.log(`Répétitions par scénario : ${CONFIG.episodesPerScenario}`);
  console.log(`Total épisodes attendus : ${FAILURE_SCENARIOS.length * CONFIG.episodesPerScenario}`);
  console.log('---------------------------------------------------------------');

  // Initialisation du simulateur headless
  const app = await createDigitalTwinApp({ headless: true });
  const failureController = app.pickAndPlace.failureController;
  const recorder = app.episodeRecorder;

  recorder.clear();

  let globalEpisodeIndex = 0;

  for (const scenarioType of FAILURE_SCENARIOS) {
    const scenarioDef = FAILURE_SCENARIO_DEFINITIONS[scenarioType];
    console.log(`\n▶ [Scénario: ${scenarioType}] - ${scenarioDef.labelFr}`);
    console.log(`  Description: ${scenarioDef.description}`);

    for (let ep = 1; ep <= CONFIG.episodesPerScenario; ep++) {
      globalEpisodeIndex++;
      const episodeId = `ep_fail_${scenarioType.toLowerCase()}_${String(ep).padStart(3, '0')}`;

      // 1. Réinitialiser la scène et le bras
      app.resetAll();

      // 2. Armer le scénario d'échec
      failureController.armScenario(scenarioType);

      // 3. Démarrer l'enregistrement de l'épisode
      recorder.startRecording(episodeId, undefined, scenarioType);

      // 4. Lancer la tâche de manipulation
      app.pickAndPlace.start();

      // 5. Simuler pas à pas
      let stepCount = 0;
      let failureOccurred = false;

      while (stepCount < CONFIG.maxStepsPerEpisode) {
        stepCount++;
        app.stepSimulation(1, 1 / 20);

        // Vérifier si un échec ou une défaillance s'est produite
        if (
          failureController.faultTriggered ||
          app.failureDetector.hasActiveFailure ||
          app.pickAndPlace.state === 'RECOVERY' ||
          app.pickAndPlace.state === 'ERROR'
        ) {
          failureOccurred = true;
          // Simuler 10 pas supplémentaires pour enregistrer les conséquences physiques (chute, rebond, etc.)
          for (let post = 0; post < 10; post++) {
            app.stepSimulation(1, 1 / 20);
          }
          break;
        }
      }

      // 6. Clôturer l'enregistrement avec le statut 'failure'
      recorder.stopRecording('failure');
      failureController.disarm();

      const lastEpisode = recorder.completedEpisodes[recorder.completedEpisodes.length - 1];
      const frameCount = lastEpisode ? lastEpisode.frames.length : 0;
      console.log(
        `  ✓ Épisode ${globalEpisodeIndex} [${episodeId}] enregistré : ${frameCount} frames collectées (Défaillance: ${failureOccurred ? 'OUI' : 'TIMEOUT'})`
      );
    }
  }

  // 7. Exportation des datasets consolidés
  if (!fs.existsSync(CONFIG.outputDir)) {
    fs.mkdirSync(CONFIG.outputDir, { recursive: true });
  }

  const timestamp = Date.now();
  const csvPath = path.join(CONFIG.outputDir, `failure_dataset_${timestamp}.csv`);
  const jsonPath = path.join(CONFIG.outputDir, `failure_dataset_${timestamp}.json`);

  const csvContent = recorder.exportConsolidatedCSV();
  const jsonContent = recorder.exportConsolidatedJSON();

  fs.writeFileSync(csvPath, csvContent, 'utf-8');
  fs.writeFileSync(jsonPath, jsonContent, 'utf-8');

  console.log('\n===============================================================');
  console.log('✅ COLLECTE TERMINÉE AVEC SUCCÈS !');
  console.log(`📁 Fichiers générés :`);
  console.log(`   - CSV  : ${csvPath} (${Buffer.byteLength(csvContent)} octets)`);
  console.log(`   - JSON : ${jsonPath} (${Buffer.byteLength(jsonContent)} octets)`);
  console.log(`📊 Total épisodes enregistrés : ${recorder.completedEpisodes.length}`);
  console.log('===============================================================');

  app.dispose();
  process.exit(0);
}

runFailureBatchCollection().catch((err) => {
  console.error('❌ Erreur lors de la collecte du dataset d\'échec :', err);
  process.exit(1);
});
