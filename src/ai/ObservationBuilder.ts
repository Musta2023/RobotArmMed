/**
 * @file ObservationBuilder.ts
 * Global multimodal observation and ground-truth telemetry system for
 * Behavior Cloning, Imitation Learning (ACT / ALOHA), Diffusion Policy,
 * Flow Matching, and NVIDIA Isaac Lab parity.
 *
 * Implements Phase C:
 * - Full robot proprioception (joint positions, velocities, torques, TCP world/base poses)
 * - Gripper physical state (opening, force, bilateral contact, slip detection)
 * - Multi-object ground truth tracking (all active objects with 6-DOF poses and 6-DOF velocities)
 * - Target zone and slot telemetry
 * - Clean separation between simulator ground truth and policy observation vector
 */

import * as THREE from 'three';
import { RobotModel } from '../robot/RobotModel.js';
import { InstrumentFactory, ActiveSurgicalInstrument } from '../objects/InstrumentFactory.js';
import { CollisionManager } from '../physics/CollisionManager.js';
import { TARGET_TRAY_POS } from '../objects/ObjectProperties.js';

export interface ObjectGroundTruthState {
  id: string;
  name: string;
  type: string;
  position: [number, number, number];
  orientation: [number, number, number, number];
  linear_velocity: [number, number, number];
  angular_velocity: [number, number, number];
  is_grasped: boolean;
  is_placed: boolean;
  mass_kg: number;
}

export interface DigitalTwinObservation {
  timestamp: number;
  step_id?: number;
  episode_id?: string;
  scenario_id?: string;
  simulation_version: string;

  // Proprioception
  joint_positions_rad: number[];
  joint_positions_deg: number[];
  joint_velocities_rad_s: number[];
  joint_torques_nm: number[];
  end_effector_position: [number, number, number];
  end_effector_orientation: [number, number, number, number];
  end_effector_base_frame_position?: [number, number, number];
  end_effector_base_frame_orientation?: [number, number, number, number];

  // Gripper
  gripper_state: {
    opening_meters: number;
    gripping_force_n: number;
    is_grasping: boolean;
    is_slipping: boolean;
    grasped_object_id: string | null;
  };

  // Primary active object (for backward compatibility and focused manipulation)
  object_position: [number, number, number];
  object_orientation: [number, number, number, number];
  object_linear_velocity?: [number, number, number];
  object_angular_velocity?: [number, number, number];

  // Target
  target_position: [number, number, number];
  target_orientation?: [number, number, number, number];
  target_tolerance_m?: number;

  // Collision
  collision_state: {
    has_collision: boolean;
    active_contact_count: number;
  };

  // Camera
  camera_metadata: {
    fov_deg: number;
    resolution: [number, number];
    camera_world_position: [number, number, number];
  };

  // Complete multi-object ground truth (all objects in scene)
  ground_truth_objects?: ObjectGroundTruthState[];

  // Flat normalized numerical policy input vector (ready for PyTorch / Diffusion Policy)
  policy_observation_vector?: number[];
}

export class ObservationBuilder {
  public robotModel: RobotModel;
  public instrumentFactory: InstrumentFactory;
  public collisionManager: CollisionManager;

  public activeEpisodeId: string = 'episode_001';
  public activeScenarioId: string = 'default_scenario';
  public currentStepId: number = 0;
  public readonly simulationVersion: string = 'surgical_action_ai_v2.0';

  constructor(
    robotModel: RobotModel,
    instrumentFactory: InstrumentFactory,
    collisionManager: CollisionManager
  ) {
    this.robotModel = robotModel;
    this.instrumentFactory = instrumentFactory;
    this.collisionManager = collisionManager;
  }

  public setContext(episodeId: string, scenarioId: string, stepId: number): void {
    this.activeEpisodeId = episodeId;
    this.activeScenarioId = scenarioId;
    this.currentStepId = stepId;
  }

  /**
   * Builds the complete structured observation vector at the current simulation timestep.
   */
  public buildObservation(
    targetPosition: [number, number, number] = TARGET_TRAY_POS,
    targetOrientation: [number, number, number, number] = [0, 0, 0, 1]
  ): DigitalTwinObservation {
    const snap = this.robotModel.state.latestSnapshot;
    const activeInst = this.instrumentFactory.getActiveInstrument();

    // 1. Multi-object ground truth inspection
    const groundTruthObjects: ObjectGroundTruthState[] = [];
    let primaryObjPos: [number, number, number] = [0, 0, 0];
    let primaryObjQuat: [number, number, number, number] = [0, 0, 0, 1];
    let primaryObjLinVel: [number, number, number] = [0, 0, 0];
    let primaryObjAngVel: [number, number, number] = [0, 0, 0];

    this.instrumentFactory.instruments.forEach((inst, id) => {
      const body = inst.managedBody.body;
      const t = body.translation();
      const r = body.rotation();
      const lv = body.linvel();
      const av = body.angvel();

      const isCurrentActive = activeInst?.config.id === id;
      const isGrasped = isCurrentActive && snap.gripper.isGrasping;

      const objState: ObjectGroundTruthState = {
        id,
        name: inst.config.name,
        type: inst.config.type,
        position: [t.x, t.y, t.z],
        orientation: [r.x, r.y, r.z, r.w],
        linear_velocity: [lv.x, lv.y, lv.z],
        angular_velocity: [av.x, av.y, av.z],
        is_grasped: isGrasped,
        is_placed: inst.isPlacedInTargetSlot ?? false,
        mass_kg: inst.config.mass_kg
      };

      groundTruthObjects.push(objState);

      if (isCurrentActive) {
        primaryObjPos = [t.x, t.y, t.z];
        primaryObjQuat = [r.x, r.y, r.z, r.w];
        primaryObjLinVel = [lv.x, lv.y, lv.z];
        primaryObjAngVel = [av.x, av.y, av.z];
      }
    });

    // 2. Camera pose
    const camWorldPos = this.robotModel.wristCamera.getWorldPosition(
      this.robotModel.wristCamera.position.clone()
    );

    // 3. Robot base frame transforms
    const baseInv = this.robotModel.root.matrixWorld.clone().invert();
    const tcpWorld = new THREE.Vector3(...snap.endEffector.position);
    const tcpBase = tcpWorld.clone().applyMatrix4(baseInv);

    const qWorld = new THREE.Quaternion(...snap.endEffector.quaternion);
    const baseQuat = new THREE.Quaternion();
    this.robotModel.root.getWorldQuaternion(baseQuat);
    const qBase = baseQuat.clone().invert().multiply(qWorld);

    // 4. Policy Observation Vector (concatenated proprioception + active object + target)
    // Dimension = 6 (qpos) + 6 (qvel) + 3 (tcp) + 4 (tcp_quat) + 1 (gripper) + 3 (obj) + 4 (obj_quat) + 3 (target) = 30 floats
    const policyVector: number[] = [
      ...snap.joints.map((j) => j.currentAngleRad),
      ...snap.joints.map((j) => j.angularVelocityRadPerSec),
      tcpBase.x,
      tcpBase.y,
      tcpBase.z,
      qBase.x,
      qBase.y,
      qBase.z,
      qBase.w,
      snap.gripper.openingMeters,
      primaryObjPos[0],
      primaryObjPos[1],
      primaryObjPos[2],
      primaryObjQuat[0],
      primaryObjQuat[1],
      primaryObjQuat[2],
      primaryObjQuat[3],
      targetPosition[0],
      targetPosition[1],
      targetPosition[2]
    ];

    return {
      timestamp: snap.timestamp,
      step_id: this.currentStepId++,
      episode_id: this.activeEpisodeId,
      scenario_id: this.activeScenarioId,
      simulation_version: this.simulationVersion,

      joint_positions_rad: snap.joints.map((j) => j.currentAngleRad),
      joint_positions_deg: snap.joints.map((j) => j.currentAngleDeg),
      joint_velocities_rad_s: snap.joints.map((j) => j.angularVelocityRadPerSec),
      joint_torques_nm: snap.joints.map((j) => j.motorTorqueNm),

      end_effector_position: [...snap.endEffector.position],
      end_effector_orientation: [...snap.endEffector.quaternion],
      end_effector_base_frame_position: [tcpBase.x, tcpBase.y, tcpBase.z],
      end_effector_base_frame_orientation: [qBase.x, qBase.y, qBase.z, qBase.w],

      gripper_state: {
        opening_meters: snap.gripper.openingMeters,
        gripping_force_n: snap.gripper.grippingForceNewtons,
        is_grasping: snap.gripper.isGrasping,
        is_slipping: snap.gripper.isSlipping,
        grasped_object_id: snap.gripper.isGrasping && activeInst ? activeInst.config.id : null
      },

      object_position: primaryObjPos,
      object_orientation: primaryObjQuat,
      object_linear_velocity: primaryObjLinVel,
      object_angular_velocity: primaryObjAngVel,

      target_position: [...targetPosition],
      target_orientation: [...targetOrientation],
      target_tolerance_m: 0.045,

      collision_state: {
        has_collision: this.collisionManager.prohibitedCollisions.length > 0,
        active_contact_count: this.collisionManager.activeContacts.length
      },

      camera_metadata: {
        fov_deg: this.robotModel.config.wristCamera.fovDeg,
        resolution: [...this.robotModel.config.wristCamera.resolution],
        camera_world_position: [camWorldPos.x, camWorldPos.y, camWorldPos.z]
      },

      ground_truth_objects: groundTruthObjects,
      policy_observation_vector: policyVector
    };
  }
}
