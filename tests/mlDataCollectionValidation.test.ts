/**
 * @file mlDataCollectionValidation.test.ts
 * Comprehensive Automated Test Suite for ML Data Collection, Scenario Generation,
 * 7-DOF Normalized Action Interface, 7-Stage Episode Validation, Data Quality Checks,
 * and Continuous Multi-Episode Dataset Export (Phase K).
 */

import { describe, it, expect, beforeEach } from 'vitest';
import * as THREE from 'three';
import { createDigitalTwinApp } from '../src/main.js';
import { ScenarioGenerator } from '../src/objects/ScenarioGenerator.js';
import { DigitalTwinObservation } from '../src/ai/ObservationBuilder.js';
import { EpisodeValidator } from '../src/ai/EpisodeValidator.js';
import { DataQualityValidator } from '../src/ai/DataQualityValidator.js';
import { RecordedEpisodeDataset } from '../src/ai/EpisodeRecorder.js';

describe('ML-Ready Robotic Digital Twin & Data Collection Suite (Phase K)', () => {
  let app: Awaited<ReturnType<typeof createDigitalTwinApp>>;
  let scenarioGenerator: ScenarioGenerator;
  let episodeValidator: EpisodeValidator;
  let dataQualityValidator: DataQualityValidator;

  beforeEach(async () => {
    app = await createDigitalTwinApp({ headless: true });
    scenarioGenerator = new ScenarioGenerator({
      instrumentFactory: app.instrumentFactory,
      workZoneManager: app.workZoneManager,
      robotModel: app.robotModel
    });
    episodeValidator = new EpisodeValidator();
    dataQualityValidator = new DataQualityValidator();
  });

  // TEST 1: Physical Simulation Reliability (60 Hz fixed timestep & Coulomb friction)
  it('TEST 1 — Physics Reliability: fixed 60 Hz stepping, friction grasp force condition', () => {
    expect(app.physicsWorld.fixedTimestep).toBeCloseTo(1 / 60, 4);
    expect(app.gripper.grippingForceNewtons).toBeGreaterThanOrEqual(1.0);

    const inst = app.instrumentFactory.getActiveInstrument();
    expect(inst).toBeDefined();
    if (inst) {
      const mass = inst.config.massKg ?? 0.22;
      const weight = mass * 9.81;
      const normalForce = app.gripper.grippingForceNewtons;
      const muEff = 0.95;
      const frictionCapacity = 2 * muEff * normalForce;
      expect(frictionCapacity).toBeGreaterThanOrEqual(1.05 * weight);
    }
  });

  // TEST 2: ScenarioGenerator Deterministic Reproducibility
  it('TEST 2 — ScenarioGenerator: deterministic seeding produces identical object placements', () => {
    const seed = 12345;
    const scen1 = scenarioGenerator.generateScenario(2, seed);
    const scen2 = scenarioGenerator.generateScenario(2, seed);

    expect(scen1.objects.length).toBe(scen2.objects.length);
    expect(scen1.objects[0].position[0]).toBeCloseTo(scen2.objects[0].position[0], 5);
    expect(scen1.objects[0].position[2]).toBeCloseTo(scen2.objects[0].position[2], 5);
    expect(scen1.objects[0].quaternion[1]).toBeCloseTo(scen2.objects[0].quaternion[1], 5);

    // Different seed produces different placement
    const scenDiff = scenarioGenerator.generateScenario(2, 99999);
    const dist = Math.hypot(
      scen1.objects[0].position[0] - scenDiff.objects[0].position[0],
      scen1.objects[0].position[2] - scenDiff.objects[0].position[2]
    );
    expect(dist).toBeGreaterThan(0.001);
  });

  // TEST 3: Multi-Level Scenarios & Workspace Constraints
  it('TEST 3 — ScenarioGenerator Levels 1–5: validates object counts, reachability & clearance', () => {
    const lvl1 = scenarioGenerator.generateScenario(1, 42);
    expect(lvl1.objects.length).toBe(1);
    expect(lvl1.difficulty_level).toBe(1);

    const lvl4 = scenarioGenerator.generateScenario(4, 42);
    expect(lvl4.objects.length).toBe(6);
    expect(lvl4.difficulty_level).toBe(4);

    // Check no two objects overlap in Level 4
    for (let i = 0; i < lvl4.objects.length; i++) {
      for (let j = i + 1; j < lvl4.objects.length; j++) {
        const p1 = lvl4.objects[i].position;
        const p2 = lvl4.objects[j].position;
        const d = Math.hypot(p1[0] - p2[0], p1[2] - p2[2]);
        expect(d).toBeGreaterThanOrEqual(0.06);
      }
      const r = Math.hypot(lvl4.objects[i].position[0], lvl4.objects[i].position[2]);
      expect(r).toBeGreaterThanOrEqual(0.23);
      expect(r).toBeLessThanOrEqual(0.48);
    }

    const lvl5 = scenarioGenerator.generateScenario(5, 77);
    expect(lvl5.is_perturbation_scenario).toBe(true);
  });

  // TEST 4: Global Observation System (Proprioception, Multi-Object Ground Truth & Policy Vector)
  it('TEST 4 — ObservationBuilder: generates 30D policy vector and multi-object ground truth', () => {
    app.robotModel.captureStateSnapshot(0.15, true, false, 'Object_01', 0.22);
    const obs = app.observationBuilder.buildObservation([0.30, 0.02, 0.24]);

    expect(obs.joint_positions_rad.length).toBe(6);
    expect(obs.joint_velocities_rad_s.length).toBe(6);
    expect(obs.end_effector_position.length).toBe(3);
    expect(obs.end_effector_orientation.length).toBe(4);
    expect(obs.end_effector_base_frame_position?.length).toBe(3);
    expect(obs.gripper_state.opening_meters).toBeGreaterThan(0);
    expect(obs.gripper_state.is_grasping).toBe(true);

    // Multi-object ground truth list
    expect(obs.ground_truth_objects).toBeDefined();
    expect(obs.ground_truth_objects!.length).toBeGreaterThanOrEqual(1);

    // Policy Observation Vector
    expect(obs.policy_observation_vector).toBeDefined();
    expect(obs.policy_observation_vector!.length).toBe(30);
    obs.policy_observation_vector!.forEach((v) => {
      expect(Number.isFinite(v)).toBe(true);
    });
  });

  // TEST 5: 7-DOF Normalized Action Interface & Joint Limit Clamping
  it('TEST 5 — ActionInterface: maps 7D actions [-1, 1]^7 to safe velocity-clipped motor targets', () => {
    // Command small positive step on all joints and close gripper
    const action7D = [0.5, 0.5, -0.5, 0.0, 0.2, 0.1, 1.0];
    const res = app.actionInterface.applyNormalizedAction(action7D);

    expect(res.accepted).toBe(true);
    expect(res.appliedAction.gripper_command).toBe('close');
    expect(res.commandedDeltaDeg.length).toBe(6);

    // Check that delta per tick does not exceed maxJointVelocity * dt
    const maxDeltaDeg = (app.actionInterface.maxJointVelocityRadPerSec * app.actionInterface.controlTimestepSec * 180) / Math.PI;
    res.commandedDeltaDeg.forEach((d) => {
      expect(Math.abs(d)).toBeLessThanOrEqual(maxDeltaDeg + 0.01);
    });

    // Test extreme out-of-bound action clipping
    const extremeAction = [10.0, -20.0, 50.0, 0, 0, 0, -1.0];
    const extremeRes = app.actionInterface.applyNormalizedAction(extremeAction);
    expect(extremeRes.wasClipped).toBe(true);
    expect(extremeRes.appliedAction.gripper_command).toBe('open');
  });

  // TEST 6: EpisodeValidator 7-Stage Evaluation for Successful Manipulation
  it('TEST 6 — EpisodeValidator: correctly validates all 7 stages in a successful trajectory', () => {
    const observations: DigitalTwinObservation[] = [];
    const targetPos: [number, number, number] = [0.30, 0.02, 0.24];
    const initialObjPos: [number, number, number] = [-0.30, 0.041, 0.24];

    // Build synthetic 40-step trajectory covering 7 stages
    for (let step = 0; step < 40; step++) {
      const t = step * 0.05;
      let tcp: [number, number, number] = [-0.30, 0.20, 0.24];
      let obj: [number, number, number] = [...initialObjPos];
      let isGrasping = false;
      let opening = 0.06;
      let linVel: [number, number, number] = [0, 0, 0];

      if (step < 5) {
        // Approach
        tcp = [-0.30, 0.10 - step * 0.012, 0.24];
      } else if (step < 10) {
        // Grasp
        tcp = [-0.30, 0.041, 0.24];
        isGrasping = true;
        opening = 0.042;
      } else if (step < 18) {
        // Lift
        const h = 0.041 + (step - 10) * 0.015;
        tcp = [-0.30, h, 0.24];
        obj = [-0.30, h, 0.24];
        isGrasping = true;
        opening = 0.042;
      } else if (step < 28) {
        // Transport
        const alpha = (step - 18) / 10;
        const x = -0.30 + alpha * 0.60;
        tcp = [x, 0.16, 0.24];
        obj = [x, 0.16, 0.24];
        isGrasping = true;
        opening = 0.042;
      } else if (step < 33) {
        // Placement
        tcp = [0.30, 0.041, 0.24];
        obj = [0.30, 0.041, 0.24];
        isGrasping = true;
        opening = 0.042;
      } else {
        // Release & Stasis
        tcp = [0.30, 0.20, 0.24];
        obj = [0.30, 0.041, 0.24];
        isGrasping = false;
        opening = 0.06;
        linVel = [0.002, 0.001, 0.001];
      }

      observations.push({
        timestamp: t,
        joint_positions_rad: [0, 0, 0, 0, 0, 0],
        joint_positions_deg: [0, 0, 0, 0, 0, 0],
        joint_velocities_rad_s: [0, 0, 0, 0, 0, 0],
        joint_torques_nm: [0, 0, 0, 0, 0, 0],
        end_effector_position: tcp,
        end_effector_orientation: [0, 0, 0, 1],
        object_position: obj,
        object_orientation: [0, 0, 0, 1],
        object_linear_velocity: linVel,
        gripper_state: {
          opening_meters: opening,
          gripping_force_n: isGrasping ? 12 : 0,
          is_grasping: isGrasping,
          is_slipping: false,
          grasped_object_id: isGrasping ? 'Object_01' : null
        },
        target_position: targetPos,
        collision_state: { has_collision: false, active_contact_count: 2 },
        camera_metadata: { fov_deg: 65, resolution: [640, 480], camera_world_position: [0, 0.2, 0] },
        simulation_version: 'surgical_action_ai_v2.0'
      });
    }

    const report = episodeValidator.evaluateEpisode('ep_valid_001', observations, 0);
    expect(report.is_valid).toBe(true);
    expect(report.outcome).toBe('success');
    expect(report.failure_reason).toBeNull();
    expect(report.stages.reach.reached).toBe(true);
    expect(report.stages.grasp.reached).toBe(true);
    expect(report.stages.lift.reached).toBe(true);
    expect(report.stages.transport.reached).toBe(true);
    expect(report.stages.placement.reached).toBe(true);
    expect(report.stages.release.reached).toBe(true);
    expect(report.stages.stability.reached).toBe(true);
    expect(report.metrics.final_position_error_m).toBeLessThan(0.01);
  });

  // TEST 7: EpisodeValidator Detection of Failure Modes
  it('TEST 7 — EpisodeValidator: detects intentional grasp failure and prohibited collisions', () => {
    const failedObs: DigitalTwinObservation[] = [];
    for (let i = 0; i < 20; i++) {
      failedObs.push({
        timestamp: i * 0.05,
        joint_positions_rad: [0, 0, 0, 0, 0, 0],
        joint_positions_deg: [0, 0, 0, 0, 0, 0],
        joint_velocities_rad_s: [0, 0, 0, 0, 0, 0],
        joint_torques_nm: [0, 0, 0, 0, 0, 0],
        end_effector_position: [-0.30, 0.20, 0.24],
        end_effector_orientation: [0, 0, 0, 1],
        object_position: [-0.30, 0.041, 0.24],
        object_orientation: [0, 0, 0, 1],
        gripper_state: {
          opening_meters: 0.06,
          gripping_force_n: 0,
          is_grasping: false,
          is_slipping: false,
          grasped_object_id: null
        },
        target_position: [0.30, 0.02, 0.24],
        collision_state: { has_collision: false, active_contact_count: 0 },
        camera_metadata: { fov_deg: 65, resolution: [640, 480], camera_world_position: [0, 0.2, 0] },
        simulation_version: 'surgical_action_ai_v2.0'
      });
    }

    const graspFailReport = episodeValidator.evaluateEpisode('ep_fail_grasp', failedObs, 0);
    expect(graspFailReport.is_valid).toBe(false);
    expect(graspFailReport.outcome).toBe('failure');
    expect(graspFailReport.failure_reason).toContain('reach_failed');

    const collReport = episodeValidator.evaluateEpisode('ep_fail_coll', failedObs, 3);
    expect(collReport.is_valid).toBe(false);
    expect(collReport.failure_reason).toContain('prohibited_collision');
  });

  // TEST 8: DataQualityValidator Anomaly & Integrity Inspection
  it('TEST 8 — DataQualityValidator: identifies NaNs, joint violations and discontinuous jumps', () => {
    const ep = app.episodeRecorder.exportDataset();

    const corruptedEp: RecordedEpisodeDataset = {
      ...ep,
      episode_id: 'ep_corrupted',
      total_steps: 12,
      frames: Array.from({ length: 12 }, (_, i) => ({
        step_index: i,
        timestamp: i * 0.05,
        observation: {
          ...app.observationBuilder.buildObservation(),
          timestamp: i * 0.05,
          joint_positions_rad: i === 5 ? [NaN, 0, 0, 0, 0, 0] : [0, 0, 0, 0, 0, 0],
          end_effector_position: i === 6 ? [5.0, 5.0, 5.0] : [-0.30, 0.20, 0.24]
        },
        action: {},
        joint_state: { positions_rad: [0, 0, 0, 0, 0, 0], velocities_rad_s: [0, 0, 0, 0, 0, 0], torques_nm: [0, 0, 0, 0, 0, 0] },
        end_effector_pose: {
          position: i === 6 ? [5.0, 5.0, 5.0] : [-0.30, 0.20, 0.24],
          orientation: [0, 0, 0, 1]
        },
        gripper_target_pose: { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
        object_state: { position: [0, 0, 0], orientation: [0, 0, 0, 1], is_grasped: false, is_slipping: false },
        object_metadata: { id: 'obj_1', name: 'Instrument', dimensions: { width: 0.04, height: 0.04, depth: 0.04 }, mass_kg: 0.2 },
        gripper_state: { opening_meters: 0.05, gripping_force_n: 0 },
        physical_contacts: [],
        grasp_succeeded: false,
        task_state: 'IDLE',
        status: 'running'
      }))
    };

    const anomaly = dataQualityValidator.inspectEpisode(corruptedEp);
    expect(anomaly.has_nan_or_inf).toBe(true);
    expect(anomaly.has_discontinuous_jumps).toBe(true);
    expect(anomaly.is_dataset_compliant).toBe(false);

    const batchReport = dataQualityValidator.generateDatasetQualityReport([corruptedEp]);
    expect(batchReport.total_episodes_inspected).toBe(1);
    expect(batchReport.non_compliant_episodes_count).toBe(1);
    expect(batchReport.compliance_rate_percent).toBe(0);
  });

  // TEST 9: Decoupled 20 Hz Recording & Continuous Multi-Episode Collection
  it('TEST 9 — EpisodeRecorder: records at 20 Hz from 60 Hz physics, chains multi-episode collection', () => {
    app.episodeRecorder.recordingRateHz = 20; // Explicitly set to 20 Hz for this test
    app.episodeRecorder.startRecording('surgical_chain_001');
    app.episodeRecorder.continuousMode = true;

    // Simulate 6 physical steps (at 60 Hz). Decimation factor = 3 -> exactly 2 recorded frames!
    for (let simStep = 0; simStep < 6; simStep++) {
      const obs = app.observationBuilder.buildObservation();
      app.episodeRecorder.recordStep({
        observation: obs,
        action: { delta_joint_actions_normalized: [0, 0, 0, 0, 0, 0, 0] },
        contacts: [],
        taskState: simStep === 5 ? 'COMPLETE' : 'APPROACH'
      });
    }

    expect(app.episodeRecorder.completedEpisodes.length).toBe(1);
    expect(app.episodeRecorder.completedEpisodes[0].episode_id).toBe('surgical_chain_001');
    expect(app.episodeRecorder.completedEpisodes[0].total_steps).toBe(2);

    for (let simStep = 0; simStep < 6; simStep++) {
      const obs = app.observationBuilder.buildObservation();
      app.episodeRecorder.recordStep({
        observation: obs,
        action: { delta_joint_actions_normalized: [0, 0, 0, 0, 0, 0, 0] },
        contacts: [],
        taskState: simStep === 5 ? 'COMPLETE' : 'APPROACH'
      });
    }

    expect(app.episodeRecorder.completedEpisodes.length).toBe(2);
    expect(app.episodeRecorder.totalRecordedSamples).toBe(4);
  });

  // TEST 10: Complete Dataset Export & Framework Compatibility
  it('TEST 10 — Dataset Export: verifies consolidated JSON bundle and stacked multi-episode CSV', () => {
    app.episodeRecorder.startRecording('export_test_001');
    for (let i = 0; i < 9; i++) {
      app.episodeRecorder.recordStep({
        observation: app.observationBuilder.buildObservation(),
        action: { delta_joint_actions_normalized: [0.1, 0, 0, 0, 0, 0, -1] },
        contacts: [],
        taskState: i === 8 ? 'COMPLETE' : 'APPROACH'
      });
    }

    const jsonStr = app.episodeRecorder.exportJSON();
    const ep = JSON.parse(jsonStr);
    expect(ep.schema_version).toBe('surgical_action_ai_v2.0');
    expect(ep.total_steps).toBe(9);

    const bundleJson = app.episodeRecorder.exportConsolidatedJSON();
    const bundle = JSON.parse(bundleJson);
    expect(bundle.dataset_name).toBe('surgical_action_ai_demonstrations');
    expect(bundle.action_dimension).toBe(7);
    expect(bundle.observation_dimension).toBe(30);

    const csvStr = app.episodeRecorder.exportCSV();
    const lines = csvStr.trim().split('\n');
    expect(lines.length).toBeGreaterThanOrEqual(2);
    expect(lines[0]).toContain('episode_id,step_index,timestamp,outcome');
  });
});
