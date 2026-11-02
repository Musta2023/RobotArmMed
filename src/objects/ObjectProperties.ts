/**
 * @file ObjectProperties.ts
 * Physical and geometric definitions for dummy surgical instruments and obstacles
 * in the Surgical Action AI Digital Twin.
 */

import { MaterialType } from '../physics/MaterialProperties.js';
import { ColliderShapeType } from '../physics/RigidBodyManager.js';

export type InstrumentPresetType =
  | 'rectangular_instrument'
  | 'cylindrical_instrument'
  | 'surgical_tool'
  | 'small_metallic_object';

export interface SurgicalInstrumentConfig {
  id: string;
  presetType: InstrumentPresetType;
  name: string;
  shape: ColliderShapeType;
  dimensions: {
    width: number;
    height: number;
    depth: number;
    radius?: number;
  };
  graspWidthMeters: number;
  massKg: number;
  friction: number;
  restitution: number;
  materialType: MaterialType;
  colorHex: number;
  initialPosition: [number, number, number];
  initialQuaternion: [number, number, number, number];
}

export const PICKUP_STATION_POS: [number, number, number] = [-0.30, 0.02, 0.24];
export const TARGET_TRAY_POS: [number, number, number] = [0.30, 0.02, 0.24];
export const PLACEMENT_TRAY_POS: [number, number, number] = TARGET_TRAY_POS;

export const INSTRUMENT_PRESETS: Record<InstrumentPresetType, Omit<SurgicalInstrumentConfig, 'id'>> = {
  rectangular_instrument: {
    presetType: 'rectangular_instrument',
    name: 'Surgical Calibration Cube (Steel Block)',
    shape: 'box',
    dimensions: {
      width: 0.042,
      height: 0.042,
      depth: 0.042
    },
    graspWidthMeters: 0.042,
    massKg: 0.22,
    friction: 0.65,
    restitution: 0.04,
    materialType: 'surgical_steel',
    colorHex: 0x94a3b8,
    initialPosition: [-0.30, 0.041, 0.24],
    initialQuaternion: [0, 0, 0, 1]
  },
  cylindrical_instrument: {
    presetType: 'cylindrical_instrument',
    name: 'Canule Trocart Cylindrique',
    shape: 'cylinder',
    dimensions: {
      width: 0.038,
      height: 0.056,
      depth: 0.038,
      radius: 0.019
    },
    graspWidthMeters: 0.038,
    massKg: 0.18,
    friction: 0.62,
    restitution: 0.04,
    materialType: 'surgical_steel',
    colorHex: 0x38bdf8,
    initialPosition: [-0.30, 0.048, 0.24],
    initialQuaternion: [0, 0, 0, 1]
  },
  surgical_tool: {
    presetType: 'surgical_tool',
    name: 'Endoscopic Forceps Handle Block',
    shape: 'box',
    dimensions: {
      width: 0.036,
      height: 0.050,
      depth: 0.058
    },
    graspWidthMeters: 0.036,
    massKg: 0.26,
    friction: 0.70,
    restitution: 0.03,
    materialType: 'plastic',
    colorHex: 0x10b981,
    initialPosition: [-0.30, 0.045, 0.24],
    initialQuaternion: [0, 0, 0, 1]
  },
  small_metallic_object: {
    presetType: 'small_metallic_object',
    name: 'Titanium Micro-Vascular Clip Box',
    shape: 'box',
    dimensions: {
      width: 0.032,
      height: 0.036,
      depth: 0.032
    },
    graspWidthMeters: 0.032,
    massKg: 0.10,
    friction: 0.55,
    restitution: 0.04,
    materialType: 'metal',
    colorHex: 0xf59e0b,
    initialPosition: [-0.30, 0.038, 0.24],
    initialQuaternion: [0, 0, 0, 1]
  }
};


export interface WorkZoneConfig {
  id: 'ZONE_A' | 'ZONE_B';
  name: string;
  role: 'START' | 'TARGET';
  colorHex: number;
  emissiveHex: number;
  center: [number, number, number];
  dimensions: {
    width: number;
    height: number;
    depth: number;
  };
  bounds: {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
    minZ: number;
    maxZ: number;
  };
  slots: [number, number, number][];
}

export const ZONE_A_CONFIG: WorkZoneConfig = {
  id: 'ZONE_A',
  name: 'ZONE A — DÉPART',
  role: 'START',
  colorHex: 0x1d4ed8, // Vibrant Cobalt Blue
  emissiveHex: 0x1e3a8a,
  center: [-0.30, 0.01, 0.245],
  dimensions: { width: 0.32, height: 0.02, depth: 0.24 },
  bounds: {
    minX: -0.46,
    maxX: -0.14,
    minY: 0.0,
    maxY: 0.25,
    minZ: 0.125,
    maxZ: 0.365
  },
  slots: [
    [-0.38, 0.02, 0.20], // Slot A1
    [-0.30, 0.02, 0.20], // Slot A2
    [-0.22, 0.02, 0.20], // Slot A3
    [-0.38, 0.02, 0.29], // Slot A4
    [-0.30, 0.02, 0.29], // Slot A5
    [-0.22, 0.02, 0.29]  // Slot A6
  ]
};

export const ZONE_B_CONFIG: WorkZoneConfig = {
  id: 'ZONE_B',
  name: 'ZONE B — CIBLE',
  role: 'TARGET',
  colorHex: 0x059669, // Vibrant Emerald Green
  emissiveHex: 0x064e3b,
  center: [0.30, 0.01, 0.245],
  dimensions: { width: 0.32, height: 0.02, depth: 0.24 },
  bounds: {
    minX: 0.14,
    maxX: 0.46,
    minY: 0.0,
    maxY: 0.25,
    minZ: 0.125,
    maxZ: 0.365
  },
  slots: [
    [0.22, 0.02, 0.20],  // Slot B1
    [0.30, 0.02, 0.20],  // Slot B2
    [0.38, 0.02, 0.20],  // Slot B3
    [0.22, 0.02, 0.29],  // Slot B4
    [0.30, 0.02, 0.29],  // Slot B5
    [0.38, 0.02, 0.29]   // Slot B6
  ]
};

export const SIX_OBJECT_DEFINITIONS: SurgicalInstrumentConfig[] = [
  {
    id: 'Object_01',
    presetType: 'rectangular_instrument',
    name: 'Cube d\'Étalonnage Titane',
    shape: 'box',
    dimensions: { width: 0.038, height: 0.038, depth: 0.038 },
    graspWidthMeters: 0.038,
    massKg: 0.18,
    friction: 0.65,
    restitution: 0.04,
    materialType: 'surgical_steel',
    colorHex: 0x94a3b8, // Metallic Silver
    initialPosition: [-0.38, 0.039, 0.20],
    initialQuaternion: [0, 0, 0, 1]
  },
  {
    id: 'Object_02',
    presetType: 'cylindrical_instrument',
    name: 'Cylindrical Trocar Cannula',
    shape: 'cylinder',
    dimensions: { width: 0.036, height: 0.046, depth: 0.036, radius: 0.018 },
    graspWidthMeters: 0.036,
    massKg: 0.16,
    friction: 0.62,
    restitution: 0.04,
    materialType: 'surgical_steel',
    colorHex: 0x06b6d4, // Surgical Cyan
    initialPosition: [-0.30, 0.043, 0.20],
    initialQuaternion: [0, 0, 0, 1]
  },
  {
    id: 'Object_03',
    presetType: 'surgical_tool',
    name: 'Prisme Pince Endoscopique',
    shape: 'box',
    dimensions: { width: 0.034, height: 0.044, depth: 0.044 },
    graspWidthMeters: 0.034,
    massKg: 0.20,
    friction: 0.70,
    restitution: 0.03,
    materialType: 'plastic',
    colorHex: 0x10b981, // Emerald Green
    initialPosition: [-0.22, 0.042, 0.20],
    initialQuaternion: [0, 0, 0, 1]
  },
  {
    id: 'Object_04',
    presetType: 'small_metallic_object',
    name: 'Boîte Micro-Clip Vasculaire',
    shape: 'box',
    dimensions: { width: 0.032, height: 0.036, depth: 0.032 },
    graspWidthMeters: 0.032,
    massKg: 0.12,
    friction: 0.55,
    restitution: 0.04,
    materialType: 'metal',
    colorHex: 0xf59e0b, // Amber Gold
    initialPosition: [-0.38, 0.038, 0.29],
    initialQuaternion: [0, 0, 0, 1]
  },
  {
    id: 'Object_05',
    presetType: 'cylindrical_instrument',
    name: 'Capsule Spécimen Polymère',
    shape: 'cylinder',
    dimensions: { width: 0.032, height: 0.044, depth: 0.032, radius: 0.016 },
    graspWidthMeters: 0.032,
    massKg: 0.14,
    friction: 0.60,
    restitution: 0.04,
    materialType: 'plastic',
    colorHex: 0x8b5cf6, // Violet Purple
    initialPosition: [-0.30, 0.042, 0.29],
    initialQuaternion: [0, 0, 0, 1]
  },
  {
    id: 'Object_06',
    presetType: 'rectangular_instrument',
    name: 'Titanium Marker Block',
    shape: 'box',
    dimensions: { width: 0.036, height: 0.040, depth: 0.036 },
    graspWidthMeters: 0.036,
    massKg: 0.15,
    friction: 0.65,
    restitution: 0.04,
    materialType: 'surgical_steel',
    colorHex: 0xef4444, // Crimson Red
    initialPosition: [-0.22, 0.040, 0.29],
    initialQuaternion: [0, 0, 0, 1]
  }
];
