import { useState, useEffect, useRef, useCallback } from 'react';
import type { SopotModule, Grid2DSimulator, GridTopology, IntegratorType } from '../types/sopot';
import type { Grid2DState } from '../components/Grid2DVisualization';
import { loadSopotWasmModule } from '../utils/wasmLoader';

// Supported grid sizes (must match WASM implementation)
export type GridSize = 5 | 10 | 20 | 50 | 100;
export const SUPPORTED_GRID_SIZES: GridSize[] = [5, 10, 20, 50, 100];

// Supported integrators
export const SUPPORTED_INTEGRATORS: IntegratorType[] = ['rk4', 'symplectic', 'verlet'];
export const INTEGRATOR_LABELS: Record<IntegratorType, string> = {
  'rk4': 'RK4 (4th order)',
  'symplectic': 'Symplectic Euler',
  'verlet': 'Velocity Verlet',
};

// Physics tuning constants for different grid sizes
const TIMESTEP_LARGE_GRID = 0.0005;  // For grids >= 50x50 (stability)
const TIMESTEP_SMALL_GRID = 0.001;   // For grids < 50x50
const SPACING_LARGE_GRID = 0.2;      // For grids > 20x20 (fit in view)
const SPACING_MEDIUM_GRID = 0.3;     // For grids 11-20
const SPACING_SMALL_GRID = 0.5;      // For grids <= 10

/** Get appropriate timestep for grid size (smaller for stability on large grids) */
function getTimestepForGridSize(size: GridSize): number {
  return size >= 50 ? TIMESTEP_LARGE_GRID : TIMESTEP_SMALL_GRID;
}

/** Get appropriate spacing for grid size (smaller for large grids to fit in view) */
function getSpacingForGridSize(size: GridSize): number {
  if (size > 20) return SPACING_LARGE_GRID;
  if (size > 10) return SPACING_MEDIUM_GRID;
  return SPACING_SMALL_GRID;
}

/**
 * Hook for 2D Grid simulation using WASM
 *
 * All physics computations run in C++ via WebAssembly.
 * The frontend only handles visualization.
 *
 * Supports grids up to 100x100 (10,000 masses) using the
 * Unified Graph Architecture with O(K) template instantiations.
 *
 * Grid types:
 * - 'quad': Horizontal + vertical springs only (standard grid)
 * - 'triangle': H + V + diagonal springs (X pattern, more stable, cloth-like)
 */
export function useGrid2DSimulation(defaultGridSize: GridSize = 10) {
  const [isReady, setIsReady] = useState(false);
  const [isInitialized, setIsInitialized] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [currentState, setCurrentState] = useState<Grid2DState | null>(null);
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0);

  const moduleRef = useRef<SopotModule | null>(null);
  const simulatorRef = useRef<Grid2DSimulator | null>(null);
  const animationFrameRef = useRef<number | null>(null);
  const lastTimeRef = useRef<number>(0);

  // Grid configuration - now variable size
  const [gridSize, setGridSize] = useState<GridSize>(defaultGridSize);

  // Physics parameters
  const [mass, setMass] = useState(1.0);
  const [stiffness, setStiffness] = useState(100.0);
  const [damping, setDamping] = useState(1.0);
  const [gridType, setGridType] = useState<GridTopology>('quad');
  const [integrator, setIntegrator] = useState<IntegratorType>('rk4');

  // Repulsion parameters (collision avoidance)
  const [repulsionEnabled, setRepulsionEnabled] = useState(false);
  const [minDistance, setMinDistance] = useState(0.05);  // Default: 5cm collision radius

  // Load WASM module (same approach as useRocketSimulation)
  useEffect(() => {
    let mounted = true;

    const loadModule = async () => {
      try {
        console.log('[Grid2D] Loading WASM module...');

        // Load the SOPOT WebAssembly module
        const module = await loadSopotWasmModule();

        if (!mounted) return;

        moduleRef.current = module;

        // Check if Grid2DSimulator is available
        if (module.Grid2DSimulator) {
          console.log('[Grid2D] WASM module loaded with Grid2DSimulator');
          setIsReady(true);
        } else {
          console.warn('[Grid2D] Grid2DSimulator not found in WASM module');
          setError('Grid2DSimulator not available. Rebuild WASM module.');
        }
      } catch (err) {
        if (!mounted) return;
        console.error('[Grid2D] Failed to load WASM module:', err);
        setError(`Failed to load physics engine: ${err instanceof Error ? err.message : String(err)}`);
      }
    };

    loadModule();

    return () => {
      mounted = false;
    };
  }, []);

  // Convert WASM state to visualization state
  const wasmToVizState = useCallback((simulator: Grid2DSimulator): Grid2DState => {
    const wasmState = simulator.getState();
    const positions: Array<{ x: number; y: number }> = [];
    const velocities: Array<{ vx: number; vy: number }> = [];

    const numMasses = wasmState.rows * wasmState.cols;
    for (let i = 0; i < numMasses; i++) {
      positions.push({
        x: wasmState.positions[i * 2],
        y: wasmState.positions[i * 2 + 1],
      });
      velocities.push({
        vx: wasmState.velocities[i * 2],
        vy: wasmState.velocities[i * 2 + 1],
      });
    }

    // Get angular velocities for rotational visualization
    const angularVelocitiesRaw = simulator.getAngularVelocities();
    const angularVelocities = Array.from(angularVelocitiesRaw);

    // Get center of mass and energy values
    const centerOfMass = simulator.getCenterOfMass();
    const kineticEnergy = simulator.getKineticEnergy();
    // potentialEnergy and totalEnergy may not exist in new implementation
    const potentialEnergy = simulator.getPotentialEnergy?.() ?? 0;
    const totalEnergy = simulator.getTotalEnergy?.() ?? kineticEnergy;

    return {
      time: wasmState.time,
      rows: wasmState.rows,
      cols: wasmState.cols,
      positions,
      velocities,
      angularVelocities,
      centerOfMass,
      kineticEnergy,
      potentialEnergy,
      totalEnergy,
    };
  }, []);

  // Initialize simulation
  const initialize = useCallback(async () => {
    if (!moduleRef.current || !moduleRef.current.Grid2DSimulator) {
      setError('WASM module not ready');
      return;
    }

    try {
      const numMasses = gridSize * gridSize;
      const numSprings = 2 * gridSize * (gridSize - 1);
      console.log(`[Grid2D] Initializing ${gridSize}x${gridSize} grid (${numMasses} masses, ${numSprings} springs)`);

      // Create new simulator
      const simulator = new moduleRef.current.Grid2DSimulator();

      // Configure grid - using new unified graph architecture
      simulator.setGridSize(gridSize, gridSize);
      simulator.setGridType(gridType);
      simulator.setIntegrator(integrator);
      simulator.setMass(mass);
      simulator.setSpacing(getSpacingForGridSize(gridSize));
      simulator.setStiffness(stiffness);
      simulator.setDamping(damping);
      simulator.setTimestep(getTimestepForGridSize(gridSize));

      // Configure repulsion (collision avoidance)
      // Second param -1 means use default stiffness
      if (repulsionEnabled) {
        simulator.setRepulsion(minDistance, -1);
      } else {
        simulator.setRepulsion(0, -1);  // Disabled
      }

      // Initialize
      simulator.initialize();

      // Add initial perturbation to center
      const perturbAmount = gridSize > 20 ? 0.1 : 0.3;
      simulator.perturbCenter(0.0, perturbAmount);

      simulatorRef.current = simulator;
      setCurrentState(wasmToVizState(simulator));
      setIsInitialized(true);
      setError(null);

      // Log system info
      if (simulator.getSystemInfo) {
        const info = simulator.getSystemInfo();
        console.log(`[Grid2D] System: ${info.numMasses} masses, ${info.numSprings} springs, ${info.stateSize} state vars`);
        console.log(`[Grid2D] Architecture: ${info.architecture}`);
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to initialize';
      console.error('[Grid2D] Initialization error:', err);
      setError(message);
    }
  }, [gridSize, gridType, integrator, mass, stiffness, damping, repulsionEnabled, minDistance, wasmToVizState]);

  // Reset simulation
  const reset = useCallback(() => {
    if (simulatorRef.current) {
      setIsRunning(false);
      simulatorRef.current = null;
    }
    setCurrentState(null);
    setIsInitialized(false);
  }, []);

  // Start simulation
  const start = useCallback(() => {
    if (!isInitialized) return;
    console.log('[Grid2D] Starting simulation (C++ physics)');
    setIsRunning(true);
    lastTimeRef.current = performance.now();
  }, [isInitialized]);

  // Pause simulation
  const pause = useCallback(() => {
    console.log('[Grid2D] Pausing simulation');
    setIsRunning(false);
  }, []);

  // Single step
  const step = useCallback(() => {
    if (!simulatorRef.current || isRunning) return;

    simulatorRef.current.step();
    setCurrentState(wasmToVizState(simulatorRef.current));
  }, [isRunning, wasmToVizState]);

  // Perturb a specific mass
  const perturbMass = useCallback((row: number, col: number, dx: number, dy: number) => {
    if (!simulatorRef.current) return;

    simulatorRef.current.perturbMass(row, col, dx, dy);
    setCurrentState(wasmToVizState(simulatorRef.current));
  }, [wasmToVizState]);

  // Animation loop
  useEffect(() => {
    if (!isRunning || !simulatorRef.current) return;

    let frameCount = 0;
    let initialCoM: { x: number; y: number } | null = null;

    const animate = (currentTime: number) => {
      const deltaTime = (currentTime - lastTimeRef.current) / 1000;
      lastTimeRef.current = currentTime;

      if (deltaTime > 0 && deltaTime < 0.1 && simulatorRef.current) {
        // Use consistent timestep calculation
        const dt = getTimestepForGridSize(gridSize);
        // More steps per frame for large grids to compensate for smaller timestep
        const baseStepsPerFrame = gridSize >= 50 ? 10 : 5;

        // Run multiple physics steps for stability
        const numSteps = Math.max(1, Math.floor(deltaTime * playbackSpeed / dt));
        const actualSteps = Math.min(numSteps, baseStepsPerFrame * 4);

        // Use stepMultiple for efficiency on large grids
        if (simulatorRef.current.stepMultiple && actualSteps > 1) {
          simulatorRef.current.stepMultiple(actualSteps);
        } else {
          for (let i = 0; i < actualSteps; i++) {
            simulatorRef.current.step();
          }
        }
        setCurrentState(wasmToVizState(simulatorRef.current));

        // Log diagnostics every 100 frames (less frequently for large grids)
        frameCount++;
        const logInterval = gridSize >= 50 ? 200 : 100;
        if (frameCount % logInterval === 0 && simulatorRef.current) {
          const com = simulatorRef.current.getCenterOfMass();
          const momentum = simulatorRef.current.getTotalMomentum();
          const time = simulatorRef.current.getTime();

          if (!initialCoM) {
            initialCoM = { ...com };
          }

          const comDrift = Math.sqrt(
            Math.pow(com.x - initialCoM.x, 2) + Math.pow(com.y - initialCoM.y, 2)
          );
          const momentumMag = Math.sqrt(momentum.px * momentum.px + momentum.py * momentum.py);

          console.log(
            `[Grid2D ${gridSize}x${gridSize}] t=${time.toFixed(2)}s | CoM drift=${comDrift.toFixed(6)} | |p|=${momentumMag.toFixed(6)}`
          );
        }
      }

      animationFrameRef.current = requestAnimationFrame(animate);
    };

    animationFrameRef.current = requestAnimationFrame(animate);

    return () => {
      if (animationFrameRef.current !== null) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [isRunning, playbackSpeed, gridSize, wasmToVizState]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      simulatorRef.current = null;
    };
  }, []);

  return {
    isReady,
    isInitialized,
    isRunning,
    error,
    currentState,
    playbackSpeed,
    gridSize,
    mass,
    stiffness,
    damping,
    gridType,
    integrator,
    repulsionEnabled,
    minDistance,
    initialize,
    start,
    pause,
    reset,
    step,
    setPlaybackSpeed,
    setGridSize,
    setMass,
    setStiffness,
    setDamping,
    setGridType,
    setIntegrator,
    setRepulsionEnabled,
    setMinDistance,
    perturbMass,
  };
}
