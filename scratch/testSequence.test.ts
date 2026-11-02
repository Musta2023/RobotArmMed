import { beforeEach, describe, it, expect } from 'vitest';
import { setupVirtualApplication, VirtualApplication } from '../tests/testHelpers.js';

describe('Sequential 6-object test', () => {
  it('runs all 6 objects and logs any failure', () => {
    const app = setupVirtualApplication({ sixObjects: true });
    expect(app.instrumentFactory.instruments.size).toBe(6);

    app.pickAndPlace.startMultiObjectSequence();
    console.log('Started multi-object sequence');

    for (let step = 0; step < 3500; step++) {
      app.stepSimulation(1);
      if (app.pickAndPlace.state === 'FAILED') {
        console.log('FAILURE OCCURRED at step', step, 'object:', app.instrumentFactory.activeInstrumentId);
        console.log('Result:', JSON.stringify(app.pickAndPlace.lastOrchestratorResult, null, 2));
        console.log('Prohibited collisions:', app.motionController.collisionManager.prohibitedCollisions);
        break;
      }
      if (app.pickAndPlace.state === 'COMPLETE') {
        console.log('ALL 6 COMPLETED at step', step);
        break;
      }
    }
  });
});
