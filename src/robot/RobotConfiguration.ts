/**
 * @file RobotConfiguration.ts
 * Configurable physical and kinematic parameters for the DofBot-inspired
 * 6-DOF Green Articulated Robotic Manipulator and Mechanical Linkage Gripper.
 */

export interface JointConfigEntry {
  id: string;
  name: string;
  shortName: string;
  index: number;
  pivotOffset: [number, number, number];
  axis: [number, number, number];
  axisLabel: 'X' | 'Y' | 'Z';
  minAngleDeg: number;
  maxAngleDeg: number;
  homeAngleDeg: number;
  maxVelocityRadPerSec: number;
  maxAccelerationRadPerSec2: number;
  maxMotorTorqueNm: number;
  stiffness: number;
  damping: number;
  linkMassKg: number;
  linkLengthMeters: number;
  centerOfMassLocal: [number, number, number];
  colliderDimensions: [number, number, number];
}

export interface GripperConfig {
  minOpeningMeters: number;
  maxOpeningMeters: number;
  defaultOpeningMeters: number;
  fingerMassKg: number;
  fingerFriction: number;
  fingerRestitution: number;
  maxGrippingForceNewtons: number;
  defaultGrippingForceNewtons: number;
  motorSpeedMetersPerSec: number;
  graspFrameOffset: [number, number, number];
  fingerDimensions: [number, number, number];
}

export interface WristCameraConfig {
  mountOffset: [number, number, number];
  lookAtOffset: [number, number, number];
  fovDeg: number;
  near: number;
  far: number;
  resolution: [number, number];
}

export interface RobotSystemConfiguration {
  name: string;
  basePosition: [number, number, number];
  baseMassKg: number;
  baseHeightMeters: number;
  joints: JointConfigEntry[];
  gripper: GripperConfig;
  wristCamera: WristCameraConfig;
}

export const DEFAULT_ROBOT_CONFIGURATION: RobotSystemConfiguration = {
  name: 'DOFBOT-6X SURGICAL DIGITAL TWIN',
  basePosition: [0, 0, 0],
  baseMassKg: 2.4,
  baseHeightMeters: 0.14,

  joints: [
    {
      id: 'J1',
      name: 'Base Rotation (Shoulder Pan)',
      shortName: 'J1 Base Pan',
      index: 0,
      pivotOffset: [0, 0.14, 0],
      axis: [0, 1, 0],
      axisLabel: 'Y',
      minAngleDeg: -170,
      maxAngleDeg: 170,
      homeAngleDeg: 0,
      maxVelocityRadPerSec: 4.5,
      maxAccelerationRadPerSec2: 45.0,
      maxMotorTorqueNm: 18.0,
      stiffness: 160.0,
      damping: 18.0,
      linkMassKg: 0.65,
      linkLengthMeters: 0.12,
      centerOfMassLocal: [0, 0.06, 0],
      colliderDimensions: [0.11, 0.11, 0.11]
    },
    {
      id: 'J2',
      name: 'Shoulder Rotation (Shoulder Lift)',
      shortName: 'J2 Shoulder Lift',
      index: 1,
      pivotOffset: [0, 0.12, 0],
      axis: [1, 0, 0],
      axisLabel: 'X',
      minAngleDeg: -95,
      maxAngleDeg: 95,
      homeAngleDeg: 20,
      maxVelocityRadPerSec: 4.2,
      maxAccelerationRadPerSec2: 42.0,
      maxMotorTorqueNm: 24.0,
      stiffness: 180.0,
      damping: 20.0,
      linkMassKg: 0.85,
      linkLengthMeters: 0.32,
      centerOfMassLocal: [0, 0.16, 0],
      colliderDimensions: [0.072, 0.32, 0.048]
    },
    {
      id: 'J3',
      name: 'Elbow Rotation (Elbow Pitch)',
      shortName: 'J3 Elbow Pitch',
      index: 2,
      pivotOffset: [0, 0.32, 0],
      axis: [1, 0, 0],
      axisLabel: 'X',
      minAngleDeg: -135,
      maxAngleDeg: 135,
      homeAngleDeg: 70,
      maxVelocityRadPerSec: 4.5,
      maxAccelerationRadPerSec2: 45.0,
      maxMotorTorqueNm: 18.0,
      stiffness: 165.0,
      damping: 18.0,
      linkMassKg: 0.60,
      linkLengthMeters: 0.15,
      centerOfMassLocal: [0, 0.075, 0],
      colliderDimensions: [0.075, 0.14, 0.07]
    },
    {
      id: 'J4',
      name: 'Forearm Rotation (Forearm Roll)',
      shortName: 'J4 Forearm Roll',
      index: 3,
      pivotOffset: [0, 0.15, 0],
      axis: [0, 1, 0],
      axisLabel: 'Y',
      minAngleDeg: -180,
      maxAngleDeg: 180,
      homeAngleDeg: 0,
      maxVelocityRadPerSec: 5.0,
      maxAccelerationRadPerSec2: 50.0,
      maxMotorTorqueNm: 12.0,
      stiffness: 140.0,
      damping: 14.0,
      linkMassKg: 0.45,
      linkLengthMeters: 0.13,
      centerOfMassLocal: [0, 0.065, 0],
      colliderDimensions: [0.07, 0.12, 0.065]
    },
    {
      id: 'J5',
      name: 'Wrist Orientation (Wrist Lift/Pitch)',
      shortName: 'J5 Wrist Pitch',
      index: 4,
      pivotOffset: [0, 0.13, 0],
      axis: [1, 0, 0],
      axisLabel: 'X',
      minAngleDeg: -165,
      maxAngleDeg: 165,
      homeAngleDeg: 45,
      maxVelocityRadPerSec: 4.8,
      maxAccelerationRadPerSec2: 48.0,
      maxMotorTorqueNm: 10.0,
      stiffness: 135.0,
      damping: 13.0,
      linkMassKg: 0.38,
      linkLengthMeters: 0.08,
      centerOfMassLocal: [0, 0.04, 0],
      colliderDimensions: [0.065, 0.075, 0.065]
    },
    {
      id: 'J6',
      name: 'Tool Orientation Axis (Tool Roll)',
      shortName: 'J6 Tool Roll',
      index: 5,
      pivotOffset: [0, 0.08, 0],
      axis: [0, 1, 0],
      axisLabel: 'Y',
      minAngleDeg: -180,
      maxAngleDeg: 180,
      homeAngleDeg: 0,
      maxVelocityRadPerSec: 5.5,
      maxAccelerationRadPerSec2: 55.0,
      maxMotorTorqueNm: 8.0,
      stiffness: 125.0,
      damping: 12.0,
      linkMassKg: 0.32,
      linkLengthMeters: 0.11,
      centerOfMassLocal: [0, 0.028, 0],
      colliderDimensions: [0.065, 0.045, 0.050]
    }
  ],

  gripper: {
    minOpeningMeters: 0.002,
    maxOpeningMeters: 0.090,
    defaultOpeningMeters: 0.078,
    fingerMassKg: 0.08,
    fingerFriction: 0.90,
    fingerRestitution: 0.05,
    maxGrippingForceNewtons: 35.0,
    defaultGrippingForceNewtons: 16.0,
    motorSpeedMetersPerSec: 0.12,
    graspFrameOffset: [0, 0.11, 0],
    fingerDimensions: [0.012, 0.068, 0.024]
  },

  wristCamera: {
    mountOffset: [0, 0.035, 0.058],
    lookAtOffset: [0, 0.14, 0.01],
    fovDeg: 58,
    near: 0.02,
    far: 2.5,
    resolution: [320, 200]
  }
};
