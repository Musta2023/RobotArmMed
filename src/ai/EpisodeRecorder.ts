/**
 * @file EpisodeRecorder.ts
 * Demonstration trajectory recorder for Behavior Cloning, Imitation Learning,
 * Diffusion Policy, and Flow Matching training in PyTorch / NVIDIA Isaac Lab (Section 8).
 */

import { DigitalTwinObservation } from './ObservationBuilder.js';
import { PolicyActionCommand } from './ActionInterface.js';
import { ContactEventRecord } from '../physics/CollisionManager.js';
import { PickAndPlaceState } from '../manipulation/PickAndPlace.js';

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
  frames: EpisodeStepFrame[];
}

export class EpisodeRecorder {
  public isRecording: boolean = false;
  public episodeId: string = 'episode_001';
  public frames: EpisodeStepFrame[] = [];
  public explicitOutcome: 'success' | 'failure' | 'incomplete' | null = null;
  private episodeCounter: number = 1;

  public startRecording(customEpisodeId?: string): void {
    this.episodeId =
      customEpisodeId ?? `surgical_ep_${String(this.episodeCounter++).padStart(3, '0')}`;
    this.frames = [];
    this.explicitOutcome = null;
    this.isRecording = true;
  }

  public stopRecording(outcome?: 'success' | 'failure' | 'incomplete'): void {
    this.isRecording = false;
    if (outcome) {
      this.explicitOutcome = outcome;
    }
  }

  public clear(): void {
    this.frames = [];
    this.explicitOutcome = null;
  }

  /**
   * Records a single simulation timestep frame (manual or AI-driven).
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
      step_index: this.frames.length,
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
      grasp_verified: params.isGraspVerified ?? (observation.gripper_state.is_grasping && !observation.gripper_state.is_slipping),
      status
    };

    this.frames.push(frame);
    return frame;
  }

  /**
   * Exports the recorded trajectory dataset as a structured object ready for
   * PyTorch Imitation Learning / Diffusion Policy / Flow Matching pipelines.
   */
  public exportDataset(): RecordedEpisodeDataset {
    const firstFrame = this.frames[0];
    const lastFrame = this.frames[this.frames.length - 1];

    const anyGraspSucceeded = this.frames.some((f) => f.grasp_succeeded);
    let outcome: 'success' | 'failure' | 'incomplete' = 'incomplete';
    if (this.explicitOutcome) {
      outcome = this.explicitOutcome;
    } else if (lastFrame?.status === 'success') {
      outcome = 'success';
    } else if (lastFrame?.status === 'failure') {
      outcome = 'failure';
    } else if (anyGraspSucceeded && this.frames.length > 5) {
      // If a manual demonstration grasped, moved, and released an object cleanly
      const startPos = firstFrame.object_state.position;
      const endPos = lastFrame.object_state.position;
      const movedDist = Math.hypot(
        endPos[0] - startPos[0],
        endPos[1] - startPos[1],
        endPos[2] - startPos[2]
      );
      outcome = movedDist > 0.02 ? 'success' : 'incomplete';
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
        'NVIDIA Isaac Lab'
      ],
      episode_id: this.episodeId,
      created_at: new Date().toISOString(),
      total_steps: this.frames.length,
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
      object_trajectory: this.frames.map((f) => ({
        timestamp: f.timestamp,
        position: [...f.object_state.position],
        orientation: [...f.object_state.orientation]
      })),
      frames: this.frames
    };
  }

  public exportJSON(): string {
    return JSON.stringify(this.exportDataset(), null, 2);
  }

  /**
   * Exports the recorded demonstration trajectory in CSV format (Section 8).
   */
  public exportCSV(): string {
    const dataset = this.exportDataset();
    const headers = [
      'episode_id',
      'step_index',
      'timestamp',
      'outcome',
      'grasp_succeeded',
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
      const tp = f.gripper_target_pose.position;
      const jp = f.joint_state.positions_rad;
      return [
        dataset.episode_id,
        f.step_index,
        f.timestamp.toFixed(4),
        dataset.outcome,
        f.grasp_succeeded ? 1 : 0,
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
}
