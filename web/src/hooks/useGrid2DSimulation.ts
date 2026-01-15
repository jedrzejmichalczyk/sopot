import { useState, useEffect, useRef, useCallback } from 'react';
import type { SopotModule, Grid2DSimulator } from '../types/sopot';
import type { Grid2DState } from '../components/Grid2DVisualization';

/**
 * Hook for 2D Grid simulation using WASM
 *
 * All physics computations run in C++ via WebAssembly.
 * The frontend only handles visualization.
 */
export function useGrid2DSimulation(defaultRows = 5, defaultCols = 5) {
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

  // Grid configuration
  const [rows] = useState(defaultRows);
  const [cols] = useState(defaultCols);

  // Load WASM module (same approach as useRocketSimulation)
  useEffect(() => {
    let mounted = true;

    const loadModule = async () => {
      try {
        console.log('[Grid2D] Loading WASM module...');

        // Use base URL to handle GitHub Pages deployment path
        const basePath = import.meta.env.BASE_URL || '/';
        const moduleUrl = `${basePath}sopot.js`;

        console.log(`[Grid2D] Loading from: ${moduleUrl}`);

        // @ts-ignore - Dynamic import of WebAssembly
        const createSopotModule = await import(/* @vite-ignore */ moduleUrl);

        if (!mounted) return;

        const module = await createSopotModule.default();

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

    return {
      time: wasmState.time,
      rows: wasmState.rows,
      cols: wasmState.cols,
      positions,
      velocities,
    };
  }, []);

  // Initialize simulation
  const initialize = useCallback(async () => {
    if (!moduleRef.current || !moduleRef.current.Grid2DSimulator) {
      setError('WASM module not ready');
      return;
    }

    try {
      console.log(`[Grid2D] Initializing ${rows}x${cols} grid (C++ physics via WASM)`);

      // Create new simulator
      const simulator = new moduleRef.current.Grid2DSimulator();

      // Configure grid
      simulator.setGridSize(rows, cols);
      simulator.setMass(1.0);
      simulator.setSpacing(0.5);
      simulator.setStiffness(50.0);
      simulator.setDamping(0.5);
      simulator.setTimestep(0.005);

      // Initialize
      simulator.initialize();

      // Add initial perturbation to center
      simulator.perturbCenter(0.0, 0.3);

      simulatorRef.current = simulator;
      setCurrentState(wasmToVizState(simulator));
      setIsInitialized(true);
      setError(null);

      console.log('[Grid2D] Initialized successfully');
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to initialize';
      console.error('[Grid2D] Initialization error:', err);
      setError(message);
    }
  }, [rows, cols, wasmToVizState]);

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

  // Animation loop
  useEffect(() => {
    if (!isRunning || !simulatorRef.current) return;

    const animate = (currentTime: number) => {
      const deltaTime = (currentTime - lastTimeRef.current) / 1000;
      lastTimeRef.current = currentTime;

      if (deltaTime > 0 && deltaTime < 0.1 && simulatorRef.current) {
        // Run multiple physics steps for stability
        const numSteps = Math.max(1, Math.floor(deltaTime * playbackSpeed / 0.005));
        for (let i = 0; i < Math.min(numSteps, 20); i++) {
          simulatorRef.current.step();
        }
        setCurrentState(wasmToVizState(simulatorRef.current));
      }

      animationFrameRef.current = requestAnimationFrame(animate);
    };

    animationFrameRef.current = requestAnimationFrame(animate);

    return () => {
      if (animationFrameRef.current !== null) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [isRunning, playbackSpeed, wasmToVizState]);

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
    initialize,
    start,
    pause,
    reset,
    step,
    setPlaybackSpeed,
  };
}
