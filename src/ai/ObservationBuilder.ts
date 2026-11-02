/**
 * @file ObservationBuilder.ts
 * Constructs standardized multimodal observations for Imitation Learning,
 * Behavior Cloning, Diffusion Policy, Flow Matching, and NVIDIA Isaac Sim parity.
 */

import { RobotModel } from '../robot/RobotModel.js';
import { InstrumentFactory } from '../objects/InstrumentFactory.js';
import { CollisionManager } from '../physics/CollisionManager.js';
import { TARGET_TRAY_POS } from '../objects/ObjectProperties.js';

export interface DigitalTwinObservation {
  timestamp: number;
  joint_positions_rad: number[];
  joint_positions_deg: number[];
  joint_velocities_rad_s: number[];
  joint_torques_nm: number[];
  end_effector_position: [number, number, number];
  end_effector_orientation: [number, number, number, number];
  object_position: [number, number, number];
  object_orientation: [number, number, number, number];
  gripper_state: {
    opening_meters: number;
    gripping_force_n: number;
    is_grasping: boolean;
    is_slipping: boolean;
  };
  target_position: [number, number, number];
  collision_state: {
    has_collision: boolean;
    active_contact_count: number;
  };
  camera_metadata: {
    fov_deg: number;
    resolution: [number, number];
    camera_world_position: [number, number, number];
  };
}

export class ObservationBuilder {
  public robotModel: RobotModel;
  public instrumentFactory: InstrumentFactory;
  public collisionManager: CollisionManager;

  constructor(
    robotModel: RobotModel,
    instrumentFactory: InstrumentFactory,
    collisionManager: CollisionManager
  ) {
    this.robotModel = robotModel;
    this.instrumentFactory = instrumentFactory;
    this.collisionManager = collisionManager;
  }

  /**
   * Builds the complete structured observation vector at the current simulation timestep.
   */
  public buildObservation(
    targetPosition: [number, number, number] = TARGET_TRAY_POS
  ): DigitalTwinObservation {
    const snap = this.robotModel.state.latestSnapshot;
    const inst = this.instrumentFactory.getActiveInstrument();

    let objPos: [number, number, number] = [0, 0, 0];
    let objQuat: [number, number, number, number] = [0, 0, 0, 1];

    if (inst) {
      const t = inst.managedBody.body.translation();
      const r = inst.managedBody.body.rotation();
      objPos = [t.x, t.y, t.z];
      objQuat = [r.x, r.y, r.z, r.w];
    }

    const camWorldPos = this.robotModel.wristCamera.getWorldPosition(
      this.robotModel.wristCamera.position.clone()
    );

    return {
      timestamp: snap.timestamp,
      joint_positions_rad: snap.joints.map((j) => j.currentAngleRad),
      joint_positions_deg: snap.joints.map((j) => j.currentAngleDeg),
      joint_velocities_rad_s: snap.joints.map((j) => j.angularVelocityRadPerSec),
      joint_torques_nm: snap.joints.map((j) => j.motorTorqueNm),
      end_effector_position: [...snap.endEffector.position],
      end_effector_orientation: [...snap.endEffector.quaternion],
      object_position: objPos,
      object_orientation: objQuat,
      gripper_state: {
        opening_meters: snap.gripper.openingMeters,
        gripping_force_n: snap.gripper.grippingForceNewtons,
        is_grasping: snap.gripper.isGrasping,
        is_slipping: snap.gripper.isSlipping
      },
      target_position: [...targetPosition],
      collision_state: {
        has_collision: this.collisionManager.prohibitedCollisions.length > 0,
        active_contact_count: this.collisionManager.activeContacts.length
      },
      camera_metadata: {
        fov_deg: this.robotModel.config.wristCamera.fovDeg,
        resolution: [...this.robotModel.config.wristCamera.resolution],
        camera_world_position: [camWorldPos.x, camWorldPos.y, camWorldPos.z]
      }
    };
  }
}
