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
    name: 'Cylindrical Trocar Cannula',
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
