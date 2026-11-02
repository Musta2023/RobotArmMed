/**
 * @file RobotState.ts
 * Structured runtime state container for the 6-DOF Robotic Digital Twin.
 */

export interface JointTelemetry {
  id: string;
  name: string;
  index: number;
  currentAngleDeg: number;
  currentAngleRad: number;
  targetAngleDeg: number;
  targetAngleRad: number;
  angularVelocityRadPerSec: number;
  motorTorqueNm: number;
  gravityLoadTorqueNm: number;
  minAngleDeg: number;
  maxAngleDeg: number;
  maxMotorTorqueNm: number;
  isAtLimit: boolean;
  isTorqueSaturated: boolean;
}

export interface GripperTelemetry {
  openingMeters: number;
  openingMillimeters: number;
  targetOpeningMeters: number;
  grippingForceNewtons: number;
  appliedNormalForceNewtons: number;
  frictionCapacityNewtons: number;
  isGrasping: boolean;
  isSlipping: boolean;
  graspedObjectId: string | null;
  leftFingerContact: boolean;
  rightFingerContact: boolean;
}

export interface EndEffectorPose {
  position: [number, number, number];
  quaternion: [number, number, number, number];
  eulerDeg: [number, number, number];
}

export interface RobotStateSnapshot {
  timestamp: number;
  joints: JointTelemetry[];
  endEffector: EndEffectorPose;
  gripper: GripperTelemetry;
  totalPayloadMassKg: number;
  anyJointAtLimit: boolean;
  anyTorqueExceeded: boolean;
}

export class RobotState {
  public latestSnapshot!: RobotStateSnapshot;

  constructor(initialSnapshot: RobotStateSnapshot) {
    this.latestSnapshot = initialSnapshot;
  }

  public update(snapshot: RobotStateSnapshot): void {
    this.latestSnapshot = snapshot;
  }

  public getJointAnglesDeg(): number[] {
    return this.latestSnapshot.joints.map((j) => j.currentAngleDeg);
  }

  public getJointAnglesRad(): number[] {
    return this.latestSnapshot.joints.map((j) => j.currentAngleRad);
  }

  public getTargetAnglesDeg(): number[] {
    return this.latestSnapshot.joints.map((j) => j.targetAngleDeg);
  }

  public getJointVelocitiesRadPerSec(): number[] {
    return this.latestSnapshot.joints.map((j) => j.angularVelocityRadPerSec);
  }

  public getJointTorquesNm(): number[] {
    return this.latestSnapshot.joints.map((j) => j.motorTorqueNm);
  }
}
