/**
 * @file InverseKinematics.ts
 * Damped Least Squares (DLS) Jacobian Inverse Kinematics solver with analytical
 * geometric seeding for the 6-DOF DofBot Digital Twin.
 *
 * NOTE (Section 5 requirement):
 * The IK solver computes feasible joint target angles; the physics engine executes
 * all movements through motorized joint PD controllers rather than directly teleporting meshes.
 */

import * as THREE from 'three';
import {
  RobotSystemConfiguration,
  DEFAULT_ROBOT_CONFIGURATION
} from '../robot/RobotConfiguration.js';
import {
  computeForwardKinematics,
  computeGeometricJacobian,
  clampAnglesDegToConfig
} from './ForwardKinematics.js';
import { RAD2DEG } from '../robot/RobotJoints.js';

export interface IKSolverRequest {
  targetPosition: [number, number, number] | THREE.Vector3;
  targetQuaternion?: [number, number, number, number] | THREE.Quaternion | null;
  currentAnglesDeg?: number[];
  positionToleranceMeters?: number;
  orientationToleranceRad?: number;
  maxIterations?: number;
}

export interface IKSolverResult {
  success: boolean;
  status: 'SUCCESS' | 'PARTIAL' | 'UNREACHABLE' | 'FAILED';
  targetAnglesDeg: number[];
  iterations: number;
  positionErrorMeters: number;
  orientationErrorRad: number;
  isReachable: boolean;
  message: string;
}

/**
 * Creates a top-down tool orientation quaternion where the gripper's local +Y axis
 * points straight down along world -Y (0, -1, 0), with optional yaw around world Y.
 */
export function createTopDownQuaternion(yawRad: number = 0): THREE.Quaternion {
  const qDown = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(1, 0, 0),
    Math.PI
  );
  const qYaw = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 1, 0),
    yawRad
  );
  return qYaw.multiply(qDown);
}

/**
 * Solves J^T * (J * J^T + lambda^2 * I)^(-1) * errorVec for 6xN Jacobian J.
 */
function solveDLS6xN(J: number[][], errorVec: number[], lambda: number): number[] {
  const m = 6;
  const n = J[0].length;
  const lSq = lambda * lambda;

  const A: number[][] = Array.from({ length: m }, (_, i) => {
    const row = new Array(m + 1).fill(0);
    for (let j = 0; j < m; j++) {
      let s = 0;
      for (let k = 0; k < n; k++) {
        s += J[i][k] * J[j][k];
      }
      if (i === j) s += lSq;
      row[j] = s;
    }
    row[m] = errorVec[i];
    return row;
  });

  // Gaussian elimination with partial pivoting
  for (let col = 0; col < m; col++) {
    let pivotRow = col;
    let maxVal = Math.abs(A[col][col]);
    for (let row = col + 1; row < m; row++) {
      const v = Math.abs(A[row][col]);
      if (v > maxVal) {
        maxVal = v;
        pivotRow = row;
      }
    }
    if (maxVal < 1e-10) continue;
    if (pivotRow !== col) {
      const tmp = A[col];
      A[col] = A[pivotRow];
      A[pivotRow] = tmp;
    }
    const piv = A[col][col];
    for (let row = col + 1; row < m; row++) {
      const factor = A[row][col] / piv;
      for (let k = col; k <= m; k++) {
        A[row][k] -= factor * A[col][k];
      }
    }
  }

  const y = new Array(m).fill(0);
  for (let i = m - 1; i >= 0; i--) {
    let sum = A[i][m];
    for (let j = i + 1; j < m; j++) {
      sum -= A[i][j] * y[j];
    }
    y[i] = Math.abs(A[i][i]) > 1e-12 ? sum / A[i][i] : 0;
  }

  const deltaTheta = new Array(n).fill(0);
  for (let k = 0; k < n; k++) {
    let sum = 0;
    for (let i = 0; i < m; i++) {
      sum += J[i][k] * y[i];
    }
    deltaTheta[k] = Number.isFinite(sum) ? sum : 0;
  }
  return deltaTheta;
}

/**
 * Generates analytical geometric seed candidates for the 6-DOF DofBot arm.
 */
function generateAnalyticalSeeds(
  targetPos: THREE.Vector3,
  targetQuat: THREE.Quaternion,
  config: RobotSystemConfiguration
): number[][] {
  const shoulderY =
    config.joints[0].pivotOffset[1] + config.joints[1].pivotOffset[1]; // 0.14 + 0.12 = 0.26 m
  const L2 = config.joints[2].pivotOffset[1]; // 0.32 m
  const L3 =
    config.joints[3].pivotOffset[1] + config.joints[4].pivotOffset[1]; // 0.15 + 0.13 = 0.28 m
  const Ltool =
    config.joints[5].pivotOffset[1] + config.gripper.graspFrameOffset[1]; // 0.08 + 0.11 = 0.19 m

  const toolYAxis = new THREE.Vector3(0, 1, 0).applyQuaternion(targetQuat).normalize();
  const toolXAxis = new THREE.Vector3(1, 0, 0).applyQuaternion(targetQuat).normalize();

  const pWrist = targetPos.clone().addScaledVector(toolYAxis, -Ltool);
  const horiz = Math.sqrt(pWrist.x * pWrist.x + pWrist.z * pWrist.z);
  const theta1Rad = horiz > 1e-5 ? Math.atan2(pWrist.x, pWrist.z) : 0;
  const theta1Deg = theta1Rad * RAD2DEG;

  const dy = pWrist.y - shoulderY;
  const distSq = horiz * horiz + dy * dy;
  const cosTheta3 = Math.min(
    Math.max((distSq - L2 * L2 - L3 * L3) / (2 * L2 * L3), -1),
    1
  );
  const alpha = Math.atan2(horiz, dy);

  const seeds: number[][] = [];
  for (const sign of [1, -1]) {
    const theta3Rad = sign * Math.acos(cosTheta3);
    const beta = Math.atan2(
      L3 * Math.sin(theta3Rad),
      L2 + L3 * Math.cos(theta3Rad)
    );
    const theta2Rad = alpha - beta;

    const theta2Deg = theta2Rad * RAD2DEG;
    const theta3Deg = theta3Rad * RAD2DEG;

    const radialHoriz =
      toolYAxis.x * Math.sin(theta1Rad) + toolYAxis.z * Math.cos(theta1Rad);
    const desiredPitch =
      Math.abs(radialHoriz) < 1e-7 && toolYAxis.y < 0
        ? Math.PI
        : Math.atan2(radialHoriz, toolYAxis.y);
    const theta5Deg = desiredPitch * RAD2DEG - (theta2Deg + theta3Deg);

    const c1 = Math.cos(theta1Rad);
    const s1 = Math.sin(theta1Rad);
    const lx = toolXAxis.x * c1 - toolXAxis.z * s1;
    const lz = toolXAxis.x * s1 + toolXAxis.z * c1;
    const theta6Deg = Math.atan2(lz, lx) * RAD2DEG;

    seeds.push(
      clampAnglesDegToConfig(
        [theta1Deg, theta2Deg, theta3Deg, 0, theta5Deg, theta6Deg],
        config
      )
    );
  }

  return seeds;
}

/**
 * Solves Inverse Kinematics for a desired target end-effector pose.
 */
export function solveInverseKinematics(
  request: IKSolverRequest,
  config: RobotSystemConfiguration = DEFAULT_ROBOT_CONFIGURATION
): IKSolverResult {
  const homePose = config.joints.map((j) => j.homeAngleDeg);
  const currentAngles = clampAnglesDegToConfig(
    request.currentAnglesDeg ?? homePose,
    config
  );

  const targetPos =
    request.targetPosition instanceof THREE.Vector3
      ? request.targetPosition.clone()
      : new THREE.Vector3(
          request.targetPosition[0],
          request.targetPosition[1],
          request.targetPosition[2]
        );

  if (
    !Number.isFinite(targetPos.x) ||
    !Number.isFinite(targetPos.y) ||
    !Number.isFinite(targetPos.z)
  ) {
    return {
      success: false,
      status: 'FAILED',
      targetAnglesDeg: currentAngles,
      iterations: 0,
      positionErrorMeters: Infinity,
      orientationErrorRad: Infinity,
      isReachable: false,
      message: 'Non-finite target coordinates supplied to IK solver.'
    };
  }

  let targetQuat: THREE.Quaternion;
  if (request.targetQuaternion instanceof THREE.Quaternion) {
    targetQuat = request.targetQuaternion.clone().normalize();
  } else if (Array.isArray(request.targetQuaternion)) {
    targetQuat = new THREE.Quaternion(
      request.targetQuaternion[0],
      request.targetQuaternion[1],
      request.targetQuaternion[2],
      request.targetQuaternion[3]
    ).normalize();
  } else {
    targetQuat = createTopDownQuaternion(0);
  }

  const posTol = request.positionToleranceMeters ?? 0.0025;
  const oriTol = request.orientationToleranceRad ?? 0.035;
  const maxIter = request.maxIterations ?? 110;

  // Geometric reachability check from shoulder pivot (0, 0.26, 0)
  const shoulderOrigin = new THREE.Vector3(0, 0.26, 0);
  const distFromShoulder = targetPos.distanceTo(shoulderOrigin);
  const maxReach = 0.32 + 0.28 + 0.19; // 0.79 m
  const isGeometricallyReachable =
    distFromShoulder <= maxReach + 0.02 && targetPos.y >= -0.02;

  const candidateSeeds = [
    currentAngles,
    ...generateAnalyticalSeeds(targetPos, targetQuat, config),
    homePose
  ];

  let bestSolution = currentAngles;
  let bestPosErr = Infinity;
  let bestOriErr = Infinity;
  let bestIters = 0;

  for (const seed of candidateSeeds) {
    let angles = [...seed];
    let iterCount = 0;

    for (let iter = 0; iter < maxIter; iter++) {
      iterCount = iter + 1;
      const { jacobian, fk } = computeGeometricJacobian(angles, config);

      const posErrVec = new THREE.Vector3().subVectors(targetPos, fk.tcpPosition);
      const posErr = posErrVec.length();

      const qCurInv = fk.tcpQuaternion.clone().invert();
      const qDiff = new THREE.Quaternion().multiplyQuaternions(targetQuat, qCurInv);
      if (qDiff.w < 0) {
        qDiff.x = -qDiff.x;
        qDiff.y = -qDiff.y;
        qDiff.z = -qDiff.z;
        qDiff.w = -qDiff.w;
      }

      const sinHalf = Math.sqrt(
        qDiff.x * qDiff.x + qDiff.y * qDiff.y + qDiff.z * qDiff.z
      );
      let oriErrRad = 0;
      const oriErrVec = new THREE.Vector3();
      if (sinHalf > 1e-7) {
        oriErrRad = 2 * Math.atan2(sinHalf, Math.min(Math.max(qDiff.w, -1), 1));
        const s = oriErrRad / sinHalf;
        oriErrVec.set(qDiff.x * s, qDiff.y * s, qDiff.z * s);
      }

      if (posErr + 0.1 * oriErrRad < bestPosErr + 0.1 * bestOriErr) {
        bestPosErr = posErr;
        bestOriErr = oriErrRad;
        bestSolution = [...angles];
        bestIters = iterCount;
      }

      if (posErr <= posTol && oriErrRad <= oriTol) {
        return {
          success: true,
          status: 'SUCCESS',
          targetAnglesDeg: clampAnglesDegToConfig(angles, config),
          iterations: iterCount,
          positionErrorMeters: posErr,
          orientationErrorRad: oriErrRad,
          isReachable: true,
          message: `IK converged in ${iterCount} iterations (err=${(posErr * 1000).toFixed(2)} mm).`
        };
      }

      const stepClip = posErr > 0.08 ? 0.08 / posErr : 1.0;
      const err6D = [
        posErrVec.x * stepClip,
        posErrVec.y * stepClip,
        posErrVec.z * stepClip,
        oriErrVec.x * 0.65,
        oriErrVec.y * 0.65,
        oriErrVec.z * 0.65
      ];

      const deltaRad = solveDLS6xN(jacobian, err6D, 0.065);
      for (let k = 0; k < angles.length; k++) {
        const dDeg = Math.min(Math.max(deltaRad[k] * RAD2DEG, -14), 14);
        angles[k] += dDeg;
      }
      angles = clampAnglesDegToConfig(angles, config);
    }
  }

  if (!isGeometricallyReachable || bestPosErr > 0.025) {
    let failureDetail = `Target unreachable (distance=${distFromShoulder.toFixed(3)} m, maxReach=${maxReach.toFixed(2)} m).`;

    if (distFromShoulder > maxReach + 0.02) {
      failureDetail = `Target beyond outer workspace reach: distance=${distFromShoulder.toFixed(3)} m exceeds maxReach=${maxReach.toFixed(2)} m.`;
    } else if (targetPos.y < -0.02) {
      failureDetail = `Target below ground plane: y=${targetPos.y.toFixed(3)} m < -0.02 m.`;
    } else {
      const horizRadius = Math.hypot(targetPos.x, targetPos.z);
      if (horizRadius < 0.23 && targetPos.y < 0.15) {
        failureDetail = `Target in base inner dead-zone (radius=${horizRadius.toFixed(3)} m < minReach=0.23 m): too close to robot base for downward reach.`;
      } else {
        failureDetail = `Target unreachable: joint angle limits / singular posture (posErr=${(bestPosErr * 1000).toFixed(1)} mm, radius=${horizRadius.toFixed(3)} m).`;
      }
    }

    return {
      success: false,
      status: 'UNREACHABLE',
      targetAnglesDeg: bestSolution,
      iterations: bestIters,
      positionErrorMeters: bestPosErr,
      orientationErrorRad: bestOriErr,
      isReachable: false,
      message: failureDetail
    };
  }

  return {
    success: bestPosErr <= posTol * 2.5,
    status: bestPosErr <= posTol * 2.5 ? 'SUCCESS' : 'PARTIAL',
    targetAnglesDeg: bestSolution,
    iterations: bestIters,
    positionErrorMeters: bestPosErr,
    orientationErrorRad: bestOriErr,
    isReachable: true,
    message: `Partial IK convergence (posErr=${(bestPosErr * 1000).toFixed(2)} mm).`
  };
}
