/**
 * @file ForwardKinematics.ts
 * Forward Kinematics (FK) and analytical Geometric Jacobian computation for
 * the 6-DOF DofBot Surgical Digital Twin.
 */

import * as THREE from 'three';
import {
  RobotSystemConfiguration,
  DEFAULT_ROBOT_CONFIGURATION
} from '../robot/RobotConfiguration.js';
import { DEG2RAD, RAD2DEG } from '../robot/RobotJoints.js';

export interface JointTransformResult {
  id: string;
  index: number;
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
  worldAxis: THREE.Vector3;
  matrix: THREE.Matrix4;
}

export interface ForwardKinematicsResult {
  anglesDeg: number[];
  anglesRad: number[];
  jointTransforms: JointTransformResult[];
  tcpPosition: THREE.Vector3;
  tcpQuaternion: THREE.Quaternion;
  tcpEulerDeg: [number, number, number];
  tcpMatrix: THREE.Matrix4;
}

/**
 * Clamps a joint angle array (in degrees) to the robot configuration's joint limits.
 */
export function clampAnglesDegToConfig(
  anglesDeg: number[],
  config: RobotSystemConfiguration = DEFAULT_ROBOT_CONFIGURATION
): number[] {
  return config.joints.map((jCfg, idx) => {
    const raw = anglesDeg[idx];
    const val = Number.isFinite(raw) ? raw : jCfg.homeAngleDeg;
    return Math.min(Math.max(val, jCfg.minAngleDeg), jCfg.maxAngleDeg);
  });
}

/**
 * Computes the complete Forward Kinematics chain from joint angles in degrees.
 */
export function computeForwardKinematics(
  anglesDeg: number[],
  config: RobotSystemConfiguration = DEFAULT_ROBOT_CONFIGURATION
): ForwardKinematicsResult {
  const clampedDeg = clampAnglesDegToConfig(anglesDeg, config);
  const anglesRad = clampedDeg.map((d) => d * DEG2RAD);

  const currentMatrix = new THREE.Matrix4().makeTranslation(
    config.basePosition[0],
    config.basePosition[1],
    config.basePosition[2]
  );

  const jointTransforms: JointTransformResult[] = [];

  for (let i = 0; i < config.joints.length; i++) {
    const jCfg = config.joints[i];
    const thetaRad = anglesRad[i];

    const transMat = new THREE.Matrix4().makeTranslation(
      jCfg.pivotOffset[0],
      jCfg.pivotOffset[1],
      jCfg.pivotOffset[2]
    );
    const localAxis = new THREE.Vector3(
      jCfg.axis[0],
      jCfg.axis[1],
      jCfg.axis[2]
    ).normalize();
    const rotMat = new THREE.Matrix4().makeRotationAxis(localAxis, thetaRad);

    currentMatrix.multiply(transMat).multiply(rotMat);

    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    currentMatrix.decompose(position, quaternion, new THREE.Vector3());

    const worldAxis = localAxis.clone().applyQuaternion(quaternion).normalize();

    jointTransforms.push({
      id: jCfg.id,
      index: i,
      position,
      quaternion,
      worldAxis,
      matrix: currentMatrix.clone()
    });
  }

  // Apply Gripper GraspFrame (TCP) offset from J6
  const tcpOffset = config.gripper.graspFrameOffset;
  const tcpMatrix = currentMatrix
    .clone()
    .multiply(new THREE.Matrix4().makeTranslation(tcpOffset[0], tcpOffset[1], tcpOffset[2]));

  const tcpPosition = new THREE.Vector3();
  const tcpQuaternion = new THREE.Quaternion();
  tcpMatrix.decompose(tcpPosition, tcpQuaternion, new THREE.Vector3());

  const euler = new THREE.Euler().setFromQuaternion(tcpQuaternion, 'XYZ');

  return {
    anglesDeg: clampedDeg,
    anglesRad,
    jointTransforms,
    tcpPosition,
    tcpQuaternion,
    tcpEulerDeg: [euler.x * RAD2DEG, euler.y * RAD2DEG, euler.z * RAD2DEG],
    tcpMatrix
  };
}

/**
 * Computes the 6xN analytical Geometric Jacobian J(theta) in world coordinates.
 */
export function computeGeometricJacobian(
  anglesDeg: number[],
  config: RobotSystemConfiguration = DEFAULT_ROBOT_CONFIGURATION
): { jacobian: number[][]; fk: ForwardKinematicsResult } {
  const fk = computeForwardKinematics(anglesDeg, config);
  const n = config.joints.length;
  const J: number[][] = Array.from({ length: 6 }, () => new Array(n).fill(0));

  for (let i = 0; i < n; i++) {
    const jt = fk.jointTransforms[i];
    const r = new THREE.Vector3().subVectors(fk.tcpPosition, jt.position);
    const linVel = new THREE.Vector3().crossVectors(jt.worldAxis, r);

    J[0][i] = linVel.x;
    J[1][i] = linVel.y;
    J[2][i] = linVel.z;
    J[3][i] = jt.worldAxis.x;
    J[4][i] = jt.worldAxis.y;
    J[5][i] = jt.worldAxis.z;
  }

  return { jacobian: J, fk };
}
