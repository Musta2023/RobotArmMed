import * as THREE from 'three';
import { solveInverseKinematics, createTopDownQuaternion } from '../src/control/InverseKinematics.js';
import { DEFAULT_ROBOT_CONFIGURATION } from '../src/robot/RobotConfiguration.js';
import { SIX_OBJECT_DEFINITIONS } from '../src/objects/ObjectProperties.js';

console.log('--- Testing Object_01 IK ---');
const def1 = SIX_OBJECT_DEFINITIONS[0];
console.log('Object_01 initial position:', def1.initialPosition);

// In APPROACH subphase 0:
const targetPos = [def1.initialPosition[0], 0.24, def1.initialPosition[2]];
const shoulderOrigin = new THREE.Vector3(0, 0.26, 0);
const dist = new THREE.Vector3(...targetPos).distanceTo(shoulderOrigin);
console.log('Target pos:', targetPos, 'Dist from shoulder:', dist);

const res = solveInverseKinematics(
  {
    targetPosition: targetPos,
    targetQuaternion: createTopDownQuaternion(0),
    currentAnglesDeg: [0, 45, 45, 0, 45, 0]
  },
  DEFAULT_ROBOT_CONFIGURATION
);

console.log('Result for Object_01 at initial slot:', res);
