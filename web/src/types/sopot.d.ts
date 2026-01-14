/**
 * TypeScript type definitions for SOPOT WebAssembly module
 */

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  q1: number;
  q2: number;
  q3: number;
  q4: number;
}

export interface SimulationState {
  time: number;
  position: Vector3;
  velocity: Vector3;
  quaternion: Quaternion;
  altitude: number;
  speed: number;
  mass: number;
}

export interface RocketSimulator {
  // Configuration
  setLauncher(elevation: number, azimuth: number): void;
  setDiameter(diameter: number): void;
  setTimestep(dt: number): void;

  // Demo data (embedded, no external files needed)
  loadDemoData(): void;

  // Data loading (Phase 1 - file paths)
  loadMassDataFromPath(path: string): void;
  loadEngineDataFromPath(path: string): void;
  loadAeroDataFromPath(path: string): void;
  loadDampingDataFromPath(path: string): void;

  // Data loading (Phase 2 - arrays, ✅ implemented)
  loadMassData(time: number[], mass: number[]): void;
  loadEngineData(time: number[], thrust: number[]): void;

  // Initialization
  setup(): void;
  reset(): void;

  // Simulation control
  step(): boolean;
  stepWithDt(dt: number): boolean;

  // Scalar state queries
  getTime(): number;
  getAltitude(): number;
  getSpeed(): number;
  getMass(): number;
  getBurnTime(): number;

  // Vector/quaternion state queries
  getPosition(): Vector3;
  getVelocity(): Vector3;
  getQuaternion(): Quaternion;
  getFullState(): SimulationState;

  // Utility
  getStateDimension(): number;
  isInitialized(): boolean;

  // Debug
  getStateAt(index: number): number;
  getDebugInfo(): string;
}

export interface SopotModule {
  RocketSimulator: new () => RocketSimulator;
}

export type CreateSopotModule = () => Promise<SopotModule>;

declare global {
  interface Window {
    createSopotModule?: CreateSopotModule;
  }
}
