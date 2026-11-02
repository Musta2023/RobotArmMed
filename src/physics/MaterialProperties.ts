/**
 * @file MaterialProperties.ts
 * Physical material properties (friction, restitution, density) and surface interaction
 * laws for the Surgical Action AI Robotic Digital Twin.
 */

export type MaterialType = 'metal' | 'rubber' | 'plastic' | 'surgical_steel' | 'anodized_aluminum';

export interface PhysicalMaterialConfig {
  id: MaterialType;
  name: string;
  friction: number;
  restitution: number;
  densityKgPerM3: number;
  colorHex: number;
  metalness: number;
  roughness: number;
}

export const MATERIAL_LIBRARY: Record<MaterialType, PhysicalMaterialConfig> = {
  rubber: {
    id: 'rubber',
    name: 'High-Friction Elastomer (Gripper Pad)',
    friction: 0.92,
    restitution: 0.08,
    densityKgPerM3: 1200,
    colorHex: 0x1e293b,
    metalness: 0.1,
    roughness: 0.88
  },
  surgical_steel: {
    id: 'surgical_steel',
    name: 'Surgical Stainless Steel (316L)',
    friction: 0.58,
    restitution: 0.18,
    densityKgPerM3: 7900,
    colorHex: 0x94a3b8,
    metalness: 0.78,
    roughness: 0.25
  },
  metal: {
    id: 'metal',
    name: 'Machined Industrial Metal',
    friction: 0.45,
    restitution: 0.22,
    densityKgPerM3: 7800,
    colorHex: 0x64748b,
    metalness: 0.75,
    roughness: 0.32
  },
  plastic: {
    id: 'plastic',
    name: 'Medical-Grade Polymer (PEEK/ABS)',
    friction: 0.38,
    restitution: 0.25,
    densityKgPerM3: 1320,
    colorHex: 0x38bdf8,
    metalness: 0.15,
    roughness: 0.48
  },
  anodized_aluminum: {
    id: 'anodized_aluminum',
    name: 'Green Anodized Aluminum (DofBot Bracket)',
    friction: 0.48,
    restitution: 0.20,
    densityKgPerM3: 2700,
    colorHex: 0x00c853,
    metalness: 0.55,
    roughness: 0.30
  }
};

/**
 * Pairwise empirical friction lookup table for specific material combinations,
 * falling back to geometric mean sqrt(mu1 * mu2) when customized by user sliders.
 */
export const PAIRWISE_FRICTION_TABLE: Record<string, number> = {
  'rubber:surgical_steel': 0.85,
  'rubber:metal': 0.82,
  'rubber:plastic': 0.72,
  'metal:metal': 0.26,
  'metal:surgical_steel': 0.28,
  'plastic:metal': 0.34,
  'plastic:surgical_steel': 0.35
};

/**
 * Computes the effective Coulomb friction coefficient between two contacting materials
 * or two explicit friction scalars.
 */
export function computeEffectiveFriction(
  frictionA: number,
  frictionB: number,
  matA?: MaterialType,
  matB?: MaterialType
): number {
  const safeA = Math.max(0, Number.isFinite(frictionA) ? frictionA : 0.5);
  const safeB = Math.max(0, Number.isFinite(frictionB) ? frictionB : 0.5);

  if (matA && matB) {
    const key1 = `${matA}:${matB}`;
    const key2 = `${matB}:${matA}`;
    const tableVal = PAIRWISE_FRICTION_TABLE[key1] ?? PAIRWISE_FRICTION_TABLE[key2];
    if (tableVal !== undefined) {
      // Scale table value proportionally if user adjusted friction sliders away from default
      const defA = MATERIAL_LIBRARY[matA]?.friction || 0.5;
      const defB = MATERIAL_LIBRARY[matB]?.friction || 0.5;
      const scale = Math.sqrt((safeA * safeB) / Math.max(defA * defB, 1e-4));
      return Math.max(0, tableVal * scale);
    }
  }

  return Math.sqrt(safeA * safeB);
}

/**
 * Computes the effective restitution (bounciness) between two contacting bodies.
 */
export function computeEffectiveRestitution(restitutionA: number, restitutionB: number): number {
  const a = Math.min(Math.max(restitutionA, 0), 1);
  const b = Math.min(Math.max(restitutionB, 0), 1);
  return Math.max(a, b);
}
