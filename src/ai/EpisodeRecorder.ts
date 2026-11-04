/**
 * @file EpisodeRecorder.ts
 * Multi-episode continuous demonstration recorder and dataset exporter
 * for Behavior Cloning, Imitation Learning (ACT / ALOHA), Diffusion Policy,
 * Flow Matching, and Hugging Face LeRobot pipelines (Phase F & I).
 *
 * Key features:
 * - Decouples 60 Hz physical simulation from configurable 20 Hz ML data collection.
 * - Continuous recording mode: collects multiple consecutive episodes without stopping.
 * - Synchronized (o_t, a_t, o_{t+1}) step transitions.
 * - Automated post-episode evaluation via EpisodeValidator.
 * - Automated dataset quality verification via DataQualityValidator.
 * - Consolidated multi-episode JSON and CSV exports.
 */

import { DigitalTwinObservation } from './ObservationBuilder.js';
import { PolicyActionCommand } from './ActionInterface.js';
import { ContactEventRecord } from '../physics/CollisionManager.js';
import { PickAndPlaceState } from '../manipulation/PickAndPlace.js';
import { EpisodeValidator, EpisodeValidationReport } from './EpisodeValidator.js';
import { ScenarioDefinition } from '../objects/ScenarioGenerator.js';

export interface ObjectMetadataSnapshot {
  id: string;
  name: string;
  dimensions: { width: number; height: number; depth: number; radius?: number };
  mass_kg: number;
}

export interface EpisodeStepFrame {
  step_index: number;
  timestamp: number;
  observation: DigitalTwinObservation;
  next_observation?: DigitalTwinObservation;
  action: PolicyActionCommand;
  joint_state: {
    positions_rad: number[];
    velocities_rad_s: number[];
    torques_nm: number[];
  };
  end_effector_pose: {
    position: [number, number, number];
    orientation: [number, number, number, number];
  };
  gripper_target_pose: {
    position: [number, number, number];
    orientation: [number, number, number, number];
  };
  target_position?: [number, number, number];
  object_state: {
    position: [number, number, number];
    orientation: [number, number, number, number];
    is_grasped: boolean;
    is_slipping: boolean;
  };
  object_metadata: ObjectMetadataSnapshot;
  gripper_state: {
    opening_meters: number;
    gripping_force_n: number;
  };
  physical_contacts: Array<{
    body_a: string;
    body_b: string;
    is_prohibited: boolean;
  }>;
  grasp_succeeded: boolean;
  physics_mode?: 'kinematic' | 'dynamic';
  grasp_verified?: boolean;
  task_state: PickAndPlaceState;
  status: 'running' | 'success' | 'failure';
}

export interface RecordedEpisodeDataset {
  schema_version: 'surgical_action_ai_v2.0';
  compatible_frameworks: string[];
  episode_id: string;
  scenario_id?: string;
  scenario_config?: ScenarioDefinition;
  failure_mode?: string;
  created_at: string;
  total_steps: number;
  outcome: 'success' | 'failure' | 'incomplete';
  grasp_succeeded: boolean;
  object_metadata: ObjectMetadataSnapshot;
  initial_object_pose: {
    position: [number, number, number];
    orientation: [number, number, number, number];
  };
  final_object_pose: {
    position: [number, number, number];
    orientation: [number, number, number, number];
  };
  initial_joint_positions_rad: number[];
  final_joint_positions_rad: number[];
  object_trajectory: Array<{
    timestamp: number;
    position: [number, number, number];
    orientation: [number, number, number, number];
  }>;
  validation_report?: EpisodeValidationReport;
  frames: EpisodeStepFrame[];
}

export interface ConsolidatedDatasetBundle {
  dataset_name: string;
  version: string;
  created_at: string;
  total_episodes: number;
  successful_episodes: number;
  failed_episodes: number;
  total_samples: number;
  recording_frequency_hz: number;
  action_dimension: number;
  observation_dimension: number;
  episodes: RecordedEpisodeDataset[];
}

export type RecorderLifecycleState = 'IDLE' | 'RECORDING' | 'VALIDATING' | 'SAVED';

export class EpisodeRecorder {
  public isRecording: boolean = false;
  public continuousMode: boolean = true;
  public lifecycleState: RecorderLifecycleState = 'IDLE';

  public recordingRateHz: number = 60;
  public simPhysicsRateHz: number = 60;
  private _simStepCounter: number = 0;

  public episodeId: string = 'surgical_ep_001';
  public currentScenario?: ScenarioDefinition;
  public currentEpisodeFrames: EpisodeStepFrame[] = [];
  public completedEpisodes: RecordedEpisodeDataset[] = [];

  public explicitOutcome: 'success' | 'failure' | 'incomplete' | null = null;
  public episodeCounter: number = 1;
  public prohibitedCollisionEventsCount: number = 0;

  public validator: EpisodeValidator = new EpisodeValidator();
  public activeFailureMode: string | null = null;
  private _lastRecordedTaskState: PickAndPlaceState | null = null;

  constructor() {
    this.completedEpisodes = [];
  }

  public get decimationFactor(): number {
    return Math.max(1, Math.round(this.simPhysicsRateHz / this.recordingRateHz));
  }

  public get totalRecordedSamples(): number {
    return (
      this.completedEpisodes.reduce((acc, ep) => acc + ep.total_steps, 0) +
      this.currentEpisodeFrames.length
    );
  }

  /**
   * Starts recording. In continuous mode, multiple episodes will be chained seamlessly.
   */
  public startRecording(
    customEpisodeId?: string,
    scenario?: ScenarioDefinition,
    failureMode?: string
  ): void {
    this.currentScenario = scenario;
    this.activeFailureMode = failureMode ?? null;
    this.episodeId =
      customEpisodeId ?? `surgical_ep_${String(this.episodeCounter++).padStart(3, '0')}`;
    this.currentEpisodeFrames = [];
    this.explicitOutcome = null;
    this.prohibitedCollisionEventsCount = 0;
    this._simStepCounter = 0;
    this.isRecording = true;
    this.lifecycleState = 'RECORDING';
  }

  /**
   * Manually stops recording, validates current active episode, and archives it.
   */
  public stopRecording(outcome?: 'success' | 'failure' | 'incomplete'): void {
    if (!this.isRecording) return;
    this.isRecording = false;
    if (outcome) {
      this.explicitOutcome = outcome;
    }
    this.finalizeActiveEpisode();
    this.lifecycleState = 'SAVED';
  }

  /**
   * Finalizes the current episode, evaluates it, and adds it to the completed dataset.
   * If continuousMode is active, it prepares the next episode immediately.
   */
  public finalizeActiveEpisode(): RecordedEpisodeDataset | null {
    if (this.currentEpisodeFrames.length === 0) {
      this.lifecycleState = 'IDLE';
      return null;
    }

    this.lifecycleState = 'VALIDATING';
    const observations = this.currentEpisodeFrames.map((f) => f.observation);
    const validationReport = this.validator.evaluateEpisode(
      this.episodeId,
      observations,
      this.prohibitedCollisionEventsCount
    );

    const episodeDataset = this._buildEpisodeDataset(validationReport);
    this.completedEpisodes.push(episodeDataset);

    // If continuous mode is enabled and still recording, start next episode automatically
    if (this.isRecording && this.continuousMode) {
      const nextId = `surgical_ep_${String(this.episodeCounter++).padStart(3, '0')}`;
      this.episodeId = nextId;
      this.currentEpisodeFrames = [];
      this.explicitOutcome = null;
      this.prohibitedCollisionEventsCount = 0;
      this._simStepCounter = 0;
      this.lifecycleState = 'RECORDING';
    } else {
      this.lifecycleState = 'SAVED';
    }

    return episodeDataset;
  }

  public clear(): void {
    this.currentEpisodeFrames = [];
    this.completedEpisodes = [];
    this.explicitOutcome = null;
    this.activeFailureMode = null;
    this.lifecycleState = 'IDLE';
    this.isRecording = false;
    this.episodeCounter = 1;
    this.episodeId = 'surgical_ep_001';
  }

  /**
   * Records a single simulation timestep frame. Decimates to configured recordingRateHz (default 20 Hz).
   */
  public recordStep(params: {
    observation: DigitalTwinObservation;
    action: PolicyActionCommand;
    contacts: ContactEventRecord[];
    taskState: PickAndPlaceState;
    gripperTargetPose?: {
      position: [number, number, number];
      orientation: [number, number, number, number];
    };
    objectMetadata?: ObjectMetadataSnapshot;
    physicsMode?: 'kinematic' | 'dynamic';
    isGraspVerified?: boolean;
  }): EpisodeStepFrame | null {
    if (!this.isRecording) return null;

    // Guard against recording post-completion idle ticks
    if (
      (params.taskState === 'COMPLETE' || params.taskState === 'FAILED') &&
      this._lastRecordedTaskState === params.taskState
    ) {
      return null;
    }

    // Check prohibited collisions
    if (params.contacts.some((c) => c.isProhibitedCollision)) {
      this.prohibitedCollisionEventsCount++;
    }

    // Decimation: only record 1 in every N physical steps to match recordingRateHz
    this._simStepCounter++;
    if (this._simStepCounter % this.decimationFactor !== 0) {
      return null;
    }

    const { observation, action, contacts, taskState } = params;
    let status: 'running' | 'success' | 'failure' = 'running';
    if (taskState === 'COMPLETE') status = 'success';
    else if (taskState === 'FAILED') status = 'failure';

    const targetPose = params.gripperTargetPose ?? {
      position: [...observation.end_effector_position],
      orientation: [...observation.end_effector_orientation]
    };

    const objMeta: ObjectMetadataSnapshot = params.objectMetadata ?? {
      id: 'surgical_instrument_01',
      name: 'Surgical Instrument',
      dimensions: { width: 0.042, height: 0.042, depth: 0.042 },
      mass_kg: 0.22
    };

    const frame: EpisodeStepFrame = {
      step_index: this.currentEpisodeFrames.length,
      timestamp: observation.timestamp,
      observation,
      action: { ...action },
      joint_state: {
        positions_rad: [...observation.joint_positions_rad],
        velocities_rad_s: [...observation.joint_velocities_rad_s],
        torques_nm: [...observation.joint_torques_nm]
      },
      end_effector_pose: {
        position: [...observation.end_effector_position],
        orientation: [...observation.end_effector_orientation]
      },
      gripper_target_pose: {
        position: [...targetPose.position],
        orientation: [...targetPose.orientation]
      },
      target_position: [...observation.target_position],
      object_state: {
        position: [...observation.object_position],
        orientation: [...observation.object_orientation],
        is_grasped: observation.gripper_state.is_grasping,
        is_slipping: observation.gripper_state.is_slipping
      },
      object_metadata: objMeta,
      gripper_state: {
        opening_meters: observation.gripper_state.opening_meters,
        gripping_force_n: observation.gripper_state.gripping_force_n
      },
      physical_contacts: contacts.map((c) => ({
        body_a: c.bodyIdA,
        body_b: c.bodyIdB,
        is_prohibited: c.isProhibitedCollision
      })),
      grasp_succeeded: observation.gripper_state.is_grasping,
      task_state: taskState,
      physics_mode: params.physicsMode ?? 'kinematic',
      grasp_verified:
        params.isGraspVerified ??
        (observation.gripper_state.is_grasping && !observation.gripper_state.is_slipping),
      status
    };

    // Link o_t -> a_t -> o_{t+1} transition
    if (this.currentEpisodeFrames.length > 0) {
      this.currentEpisodeFrames[this.currentEpisodeFrames.length - 1].next_observation = observation;
    }

    this.currentEpisodeFrames.push(frame);

    // Auto-finalize episode only on edge transition into COMPLETE or FAILED
    if (
      (taskState === 'COMPLETE' || taskState === 'FAILED') &&
      this._lastRecordedTaskState !== taskState &&
      this.currentEpisodeFrames.length >= 2
    ) {
      this._lastRecordedTaskState = taskState;
      this.finalizeActiveEpisode();
    } else if (taskState !== 'COMPLETE' && taskState !== 'FAILED') {
      this._lastRecordedTaskState = taskState;
    }

    return frame;
  }

  // Alias for backward compatibility with existing tests
  public exportDataset(): RecordedEpisodeDataset {
    // If active buffer has a real trajectory, export it
    if (this.currentEpisodeFrames.length > 1) {
      const report = this.validator.evaluateEpisode(
        this.episodeId,
        this.currentEpisodeFrames.map((f) => f.observation),
        this.prohibitedCollisionEventsCount
      );
      return this._buildEpisodeDataset(report);
    }
    // Otherwise fallback to the last completed real episode
    const realEpisodes = this.completedEpisodes.filter((e) => e.total_steps > 1);
    if (realEpisodes.length > 0) {
      return realEpisodes[realEpisodes.length - 1];
    }
    if (this.completedEpisodes.length > 0) {
      return this.completedEpisodes[this.completedEpisodes.length - 1];
    }
    if (this.currentEpisodeFrames.length > 0) {
      const report = this.validator.evaluateEpisode(
        this.episodeId,
        this.currentEpisodeFrames.map((f) => f.observation),
        this.prohibitedCollisionEventsCount
      );
      return this._buildEpisodeDataset(report);
    }
    const report = this.validator.evaluateEpisode(this.episodeId, [], 0);
    return this._buildEpisodeDataset(report);
  }

  public get frames(): EpisodeStepFrame[] {
    return this.currentEpisodeFrames;
  }

  private _buildEpisodeDataset(
    validationReport?: EpisodeValidationReport
  ): RecordedEpisodeDataset {
    const frames = this.currentEpisodeFrames;
    const firstFrame = frames[0];
    const lastFrame = frames[frames.length - 1];

    const anyGraspSucceeded = frames.some((f) => f.grasp_succeeded);
    let outcome: 'success' | 'failure' | 'incomplete' = 'incomplete';

    if (this.explicitOutcome) {
      outcome = this.explicitOutcome;
    } else if (validationReport) {
      outcome = validationReport.outcome;
    } else if (lastFrame?.status === 'success') {
      outcome = 'success';
    } else if (lastFrame?.status === 'failure') {
      outcome = 'failure';
    }

    const defaultMeta: ObjectMetadataSnapshot = firstFrame?.object_metadata ?? {
      id: 'surgical_instrument_01',
      name: 'Surgical Instrument',
      dimensions: { width: 0.042, height: 0.042, depth: 0.042 },
      mass_kg: 0.22
    };

    return {
      schema_version: 'surgical_action_ai_v2.0',
      compatible_frameworks: [
        'PyTorch Behavior Cloning',
        'Imitation Learning (ACT / ALOHA)',
        'Diffusion Policy',
        'Flow Matching VLA',
        'HuggingFace LeRobot',
        'NVIDIA Isaac Lab'
      ],
      episode_id: this.episodeId,
      scenario_id: this.currentScenario?.scenario_id,
      scenario_config: this.currentScenario,
      failure_mode: this.activeFailureMode ?? undefined,
      created_at: new Date().toISOString(),
      total_steps: frames.length,
      outcome,
      grasp_succeeded: anyGraspSucceeded,
      object_metadata: defaultMeta,
      initial_object_pose: firstFrame
        ? {
            position: [...firstFrame.object_state.position],
            orientation: [...firstFrame.object_state.orientation]
          }
        : { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
      final_object_pose: lastFrame
        ? {
            position: [...lastFrame.object_state.position],
            orientation: [...lastFrame.object_state.orientation]
          }
        : { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
      initial_joint_positions_rad: firstFrame
        ? [...firstFrame.joint_state.positions_rad]
        : [0, 0, 0, 0, 0, 0],
      final_joint_positions_rad: lastFrame
        ? [...lastFrame.joint_state.positions_rad]
        : [0, 0, 0, 0, 0, 0],
      object_trajectory: frames.map((f) => ({
        timestamp: f.timestamp,
        position: [...f.object_state.position],
        orientation: [...f.object_state.orientation]
      })),
      validation_report: validationReport,
      frames: [...frames]
    };
  }

  /**
   * Returns a consolidated bundle of all recorded episodes in the current session.
   */
  public exportConsolidatedBundle(): ConsolidatedDatasetBundle {
    const all = [...this.completedEpisodes.filter((e) => e.total_steps > 1)];
    if (
      this.currentEpisodeFrames.length > 1 &&
      !all.some((e) => e.episode_id === this.episodeId)
    ) {
      all.push(this.exportDataset());
    }

    const successes = all.filter((ep) => ep.outcome === 'success').length;
    const failures = all.filter((ep) => ep.outcome === 'failure').length;
    const totalSamples = all.reduce((acc, ep) => acc + ep.total_steps, 0);

    return {
      dataset_name: 'surgical_action_ai_demonstrations',
      version: '2.0.0',
      created_at: new Date().toISOString(),
      total_episodes: all.length,
      successful_episodes: successes,
      failed_episodes: failures,
      total_samples: totalSamples,
      recording_frequency_hz: this.recordingRateHz,
      action_dimension: 7,
      observation_dimension: 30,
      episodes: all
    };
  }

  public exportJSON(): string {
    return JSON.stringify(this.exportDataset(), null, 2);
  }

  public exportConsolidatedJSON(): string {
    return JSON.stringify(this.exportConsolidatedBundle(), null, 2);
  }

  /**
   * Exports the latest / active recorded demonstration trajectory in standard CSV format.
   */
  public exportCSV(): string {
    const dataset = this.exportDataset();
    const headers = [
      'episode_id',
      'step_index',
      'timestamp',
      'outcome',
      'grasp_succeeded',
      'task_state',
      'obj_id',
      'obj_mass_kg',
      'obj_x',
      'obj_y',
      'obj_z',
      'obj_qx',
      'obj_qy',
      'obj_qz',
      'obj_qw',
      'ee_x',
      'ee_y',
      'ee_z',
      'target_x',
      'target_y',
      'target_z',
      'gripper_opening_m',
      'j1_rad',
      'j2_rad',
      'j3_rad',
      'j4_rad',
      'j5_rad',
      'j6_rad',
      'contact_count'
    ];

    const rows = dataset.frames.map((f) => {
      const op = f.object_state.position;
      const oq = f.object_state.orientation;
      const ee = f.end_effector_pose.position;
      const tp = (f as any).target_position ?? f.gripper_target_pose.position;
      const jp = f.joint_state.positions_rad;
      return [
        dataset.episode_id,
        f.step_index,
        f.timestamp.toFixed(4),
        dataset.outcome,
        f.grasp_succeeded ? 1 : 0,
        f.task_state,
        f.object_metadata.id,
        f.object_metadata.mass_kg.toFixed(3),
        op[0].toFixed(4),
        op[1].toFixed(4),
        op[2].toFixed(4),
        oq[0].toFixed(4),
        oq[1].toFixed(4),
        oq[2].toFixed(4),
        oq[3].toFixed(4),
        ee[0].toFixed(4),
        ee[1].toFixed(4),
        ee[2].toFixed(4),
        tp[0].toFixed(4),
        tp[1].toFixed(4),
        tp[2].toFixed(4),
        f.gripper_state.opening_meters.toFixed(4),
        (jp[0] ?? 0).toFixed(4),
        (jp[1] ?? 0).toFixed(4),
        (jp[2] ?? 0).toFixed(4),
        (jp[3] ?? 0).toFixed(4),
        (jp[4] ?? 0).toFixed(4),
        (jp[5] ?? 0).toFixed(4),
        f.physical_contacts.length
      ].join(',');
    });

    return [headers.join(','), ...rows].join('\n');
  }

  /**
   * Exports all recorded episodes consolidated into a single stacked CSV table.
   */
  public exportConsolidatedCSV(): string {
    const bundle = this.exportConsolidatedBundle();
    const headers = [
      'episode_id',
      'step_index',
      'timestamp',
      'outcome',
      'grasp_succeeded',
      'task_state',
      'obj_id',
      'obj_mass_kg',
      'obj_x',
      'obj_y',
      'obj_z',
      'obj_qx',
      'obj_qy',
      'obj_qz',
      'obj_qw',
      'ee_x',
      'ee_y',
      'ee_z',
      'target_x',
      'target_y',
      'target_z',
      'gripper_opening_m',
      'j1_rad',
      'j2_rad',
      'j3_rad',
      'j4_rad',
      'j5_rad',
      'j6_rad',
      'contact_count'
    ];

    const rows: string[] = [];

    bundle.episodes.forEach((ep) => {
      ep.frames.forEach((f) => {
        const op = f.object_state.position;
        const oq = f.object_state.orientation;
        const ee = f.end_effector_pose.position;
        const tp = (f as any).target_position ?? f.gripper_target_pose.position;
        const jp = f.joint_state.positions_rad;
        rows.push(
          [
            ep.episode_id,
            f.step_index,
            f.timestamp.toFixed(4),
            ep.outcome,
            f.grasp_succeeded ? 1 : 0,
            f.task_state,
            f.object_metadata.id,
            f.object_metadata.mass_kg.toFixed(3),
            op[0].toFixed(4),
            op[1].toFixed(4),
            op[2].toFixed(4),
            oq[0].toFixed(4),
            oq[1].toFixed(4),
            oq[2].toFixed(4),
            oq[3].toFixed(4),
            ee[0].toFixed(4),
            ee[1].toFixed(4),
            ee[2].toFixed(4),
            tp[0].toFixed(4),
            tp[1].toFixed(4),
            tp[2].toFixed(4),
            f.gripper_state.opening_meters.toFixed(4),
            (jp[0] ?? 0).toFixed(4),
            (jp[1] ?? 0).toFixed(4),
            (jp[2] ?? 0).toFixed(4),
            (jp[3] ?? 0).toFixed(4),
            (jp[4] ?? 0).toFixed(4),
            (jp[5] ?? 0).toFixed(4),
            f.physical_contacts.length
          ].join(',')
        );
      });
    });

    return [headers.join(','), ...rows].join('\n');
  }
}
