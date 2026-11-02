/**
 * @file CollisionGroups.ts
 * Logical Rapier3D collision and solver group definitions (Phase 4).
 *
 * Uses 16-bit membership masks and 16-bit filter masks packed into 32-bit integers:
 *   (membership << 16) | filter
 *
 * Rules enforced:
 * - Dynamic objects collide with floor/platforms, other dynamic objects, obstacles,
 *   all robot links, and gripper fingers.
 * - Adjacent robot links do not self-interfere, while non-adjacent distal links
 *   (wrist/gripper) DO collide with the robot base.
 * - Gripper fingers collide with dynamic objects, obstacles, floor, and base,
 *   without jamming against the wrist flange or each other when fully closed.
 * - Sensors/detection zones are distinguished from physical colliders.
 */

export const COLLISION_GROUP = {
  GROUND_ENV: 0x0001,       // Floor slab, Pickup Station Pad, Target Surgical Tray
  DYNAMIC_OBJECT: 0x0002,   // Surgical instruments, stacked boxes, pushed objects
  OBSTACLE: 0x0004,         // Workspace safety obstacles
  ROBOT_BASE_LOWER: 0x0008, // robot_base, robot_link_0 (J1), robot_link_1 (J2)
  ROBOT_UPPER_ARM: 0x0010,  // robot_link_2 (J3), robot_link_3 (J4)
  ROBOT_WRIST: 0x0020,      // robot_link_4 (J5), robot_link_5 (J6)
  GRIPPER_FINGER: 0x0040,   // gripper_left_finger, gripper_right_finger
  SENSOR_ZONE: 0x0080       // Non-physical detection sensors
} as const;

/**
 * Packs a 16-bit membership mask and 16-bit filter mask into Rapier's 32-bit interaction group format.
 */
export function encodeCollisionGroups(membership: number, filter: number): number {
  return ((membership & 0xffff) << 16) | (filter & 0xffff);
}

/**
 * Decodes a 32-bit Rapier interaction group integer into its membership and filter masks.
 */
export function decodeCollisionGroups(encoded: number): {
  membership: number;
  filter: number;
} {
  return {
    membership: (encoded >>> 16) & 0xffff,
    filter: encoded & 0xffff
  };
}

/**
 * Returns true if two Rapier collision groups (either 32-bit encoded integers or
 * `{ membership, filter }` preset objects) are permitted to interact.
 */
export function canGroupsInteract(
  groupA: number | { membership: number; filter: number },
  groupB: number | { membership: number; filter: number }
): boolean {
  const a = typeof groupA === 'number' ? decodeCollisionGroups(groupA) : groupA;
  const b = typeof groupB === 'number' ? decodeCollisionGroups(groupB) : groupB;
  return (a.membership & b.filter) !== 0 && (b.membership & a.filter) !== 0;
}

/**
 * Precomputed membership & filter presets for each physical category in the Digital Twin.
 */
export const COLLISION_PRESETS = {
  GROUND_AND_PLATFORMS: {
    membership: COLLISION_GROUP.GROUND_ENV,
    filter:
      COLLISION_GROUP.DYNAMIC_OBJECT |
      COLLISION_GROUP.ROBOT_UPPER_ARM |
      COLLISION_GROUP.ROBOT_WRIST |
      COLLISION_GROUP.GRIPPER_FINGER
  },

  DYNAMIC_OBJECT: {
    membership: COLLISION_GROUP.DYNAMIC_OBJECT,
    filter:
      COLLISION_GROUP.GROUND_ENV |
      COLLISION_GROUP.DYNAMIC_OBJECT |
      COLLISION_GROUP.OBSTACLE |
      COLLISION_GROUP.ROBOT_BASE_LOWER |
      COLLISION_GROUP.ROBOT_UPPER_ARM |
      COLLISION_GROUP.ROBOT_WRIST |
      COLLISION_GROUP.GRIPPER_FINGER |
      COLLISION_GROUP.SENSOR_ZONE
  },

  OBSTACLE: {
    membership: COLLISION_GROUP.OBSTACLE,
    filter:
      COLLISION_GROUP.DYNAMIC_OBJECT |
      COLLISION_GROUP.ROBOT_BASE_LOWER |
      COLLISION_GROUP.ROBOT_UPPER_ARM |
      COLLISION_GROUP.ROBOT_WRIST |
      COLLISION_GROUP.GRIPPER_FINGER
  },

  ROBOT_BASE_LOWER: {
    membership: COLLISION_GROUP.ROBOT_BASE_LOWER,
    filter:
      COLLISION_GROUP.DYNAMIC_OBJECT |
      COLLISION_GROUP.OBSTACLE |
      COLLISION_GROUP.ROBOT_WRIST |
      COLLISION_GROUP.GRIPPER_FINGER
  },

  ROBOT_UPPER_ARM: {
    membership: COLLISION_GROUP.ROBOT_UPPER_ARM,
    filter:
      COLLISION_GROUP.GROUND_ENV |
      COLLISION_GROUP.DYNAMIC_OBJECT |
      COLLISION_GROUP.OBSTACLE
  },

  ROBOT_WRIST: {
    membership: COLLISION_GROUP.ROBOT_WRIST,
    filter:
      COLLISION_GROUP.GROUND_ENV |
      COLLISION_GROUP.DYNAMIC_OBJECT |
      COLLISION_GROUP.OBSTACLE |
      COLLISION_GROUP.ROBOT_BASE_LOWER
  },

  GRIPPER_FINGER: {
    membership: COLLISION_GROUP.GRIPPER_FINGER,
    filter:
      COLLISION_GROUP.GROUND_ENV |
      COLLISION_GROUP.DYNAMIC_OBJECT |
      COLLISION_GROUP.OBSTACLE |
      COLLISION_GROUP.ROBOT_BASE_LOWER
  },

  SENSOR: {
    membership: COLLISION_GROUP.SENSOR_ZONE,
    filter: COLLISION_GROUP.DYNAMIC_OBJECT | COLLISION_GROUP.GRIPPER_FINGER
  }
} as const;
