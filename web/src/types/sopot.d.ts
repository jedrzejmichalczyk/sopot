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

export interface TimeSeriesData {
  time: number[];
  kinematics: {
    altitude: number[];
    speed: number[];
    pos_x: number[];
    pos_y: number[];
    pos_z: number[];
    vel_x: number[];
    vel_y: number[];
    vel_z: number[];
  };
  dynamics: {
    mass: number[];
    accel_x: number[];
    accel_y: number[];
    accel_z: number[];
  };
  forces: {
    thrust: number[];
    gravity: number[];
  };
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

  // Time-series data retrieval
  getTimeSeries(): TimeSeriesData;
  getHistorySize(): number;
  setRecordHistory(enable: boolean): void;
  clearHistory(): void;

  // Utility
  getStateDimension(): number;
  isInitialized(): boolean;

  // Debug
  getStateAt(index: number): number;
  getDebugInfo(): string;
}

/**
 * 2D Grid simulation state
 */
export interface Grid2DWasmState {
  time: number;
  rows: number;
  cols: number;
  positions: number[];  // Flat array: [x0, y0, x1, y1, ...]
  velocities: number[]; // Flat array: [vx0, vy0, vx1, vy1, ...]
}

/**
 * 2D Grid Simulator - WASM wrapper for C++ physics
 */
export interface Grid2DSimulator {
  // Configuration
  setGridSize(rows: number, cols: number): void;
  setMass(mass: number): void;
  setSpacing(spacing: number): void;
  setStiffness(stiffness: number): void;
  setDamping(damping: number): void;
  setTimestep(dt: number): void;

  // Initialization
  initialize(): void;
  reset(): void;
  perturbMass(row: number, col: number, dx: number, dy: number): void;
  perturbCenter(dx: number, dy: number): void;

  // Simulation
  step(): void;
  stepWithDt(dt: number): void;

  // State queries
  getTime(): number;
  getRows(): number;
  getCols(): number;
  isInitialized(): boolean;
  getPositions(): number[];
  getVelocities(): number[];
  getState(): Grid2DWasmState;
  getMassPosition(row: number, col: number): { x: number; y: number };
  getKineticEnergy(): number;
  getPotentialEnergy(): number;
  getTotalEnergy(): number;
  getCenterOfMass(): { x: number; y: number };
}

/**
 * Inverted Double Pendulum Simulator
 */
export interface InvertedPendulumSimulator {
  // Configuration
  setParameters(cartMass: number, m1: number, m2: number, L1: number, L2: number, g: number): void;
  setInitialState(x: number, theta1: number, theta2: number, xdot: number, omega1: number, omega2: number): void;
  setTimestep(dt: number): void;
  setMaxForce(maxForce: number): void;
  configureLQR(qDiag: number[], r: number): void;
  setupDefault(): void;
  reset(): void;

  // Simulation control
  step(): boolean;
  stepWithDt(dt: number): boolean;
  setControllerEnabled(enabled: boolean): void;
  applyCartImpulse(impulse: number): void;
  applyLink1Impulse(impulse: number): void;
  applyLink2Impulse(impulse: number): void;

  // State queries
  getTime(): number;
  getCartPosition(): number;
  getTheta1(): number;
  getTheta2(): number;
  getCartVelocity(): number;
  getOmega1(): number;
  getOmega2(): number;
  getControlForce(): number;
  getFullState(): {
    time: number;
    x: number;
    theta1: number;
    theta2: number;
    xdot: number;
    omega1: number;
    omega2: number;
    controlForce: number;
    link1Tip: { x: number; y: number };
    link2Tip: { x: number; y: number };
  };
  getVisualizationData(): {
    cart: { x: number; y: number };
    joint1: { x: number; y: number };
    joint2: { x: number; y: number };
    tip: { x: number; y: number };
    theta1: number;
    theta2: number;
    controlForce: number;
  };

  // History
  setRecordHistory(record: boolean): void;
  getHistorySize(): number;
  clearHistory(): void;
  getHistory(): {
    time: number[];
    x: number[];
    theta1: number[];
    theta2: number[];
    xdot: number[];
    omega1: number[];
    omega2: number[];
    controlForce: number[];
  };

  // Parameters
  getCartMass(): number;
  getMass1(): number;
  getMass2(): number;
  getLength1(): number;
  getLength2(): number;
  getGravity(): number;
  getMaxForce(): number;
  isInitialized(): boolean;
  isControllerEnabled(): boolean;
  getLQRGain(): number[];

  delete(): void;
}

export interface SopotModule {
  RocketSimulator: new () => RocketSimulator;
  Grid2DSimulator: new () => Grid2DSimulator;
  InvertedPendulumSimulator?: new () => InvertedPendulumSimulator;
}

export type CreateSopotModule = () => Promise<SopotModule>;

declare global {
  interface Window {
    createSopotModule?: CreateSopotModule;
  }
}
