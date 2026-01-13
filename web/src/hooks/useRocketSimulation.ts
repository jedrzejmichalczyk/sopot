import { useState, useEffect, useCallback, useRef } from 'react';
import type { SopotModule, RocketSimulator, SimulationState } from '../types/sopot';

export interface SimulationConfig {
  elevation: number;
  azimuth: number;
  diameter: number;
  timestep: number;
  dataPath?: string;
}

export interface UseRocketSimulationReturn {
  // State
  isReady: boolean;
  isInitialized: boolean;
  isRunning: boolean;
  currentState: SimulationState | null;
  error: string | null;

  // Actions
  initialize: (config: SimulationConfig) => Promise<void>;
  start: () => void;
  pause: () => void;
  reset: () => void;
  step: () => void;

  // Computed values
  playbackSpeed: number;
  setPlaybackSpeed: (speed: number) => void;
}

export function useRocketSimulation(): UseRocketSimulationReturn {
  const [module, setModule] = useState<SopotModule | null>(null);
  const [simulator, setSimulator] = useState<RocketSimulator | null>(null);
  const [isRunning, setIsRunning] = useState(false);
  const [currentState, setCurrentState] = useState<SimulationState | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0);

  const animationFrameRef = useRef<number | null>(null);
  const lastFrameTimeRef = useRef<number>(0);

  // Load WebAssembly module on mount
  useEffect(() => {
    let mounted = true;

    const loadModule = async () => {
      try {
        // Import the SOPOT WebAssembly module
        // @ts-ignore - Dynamic import of WebAssembly
        const createSopotModule = await import('/sopot.js');

        if (!mounted) return;

        const moduleInstance = await createSopotModule.default();

        if (mounted) {
          setModule(moduleInstance);
          console.log('SOPOT WebAssembly module loaded successfully');
        }
      } catch (err) {
        if (mounted) {
          const errorMsg = err instanceof Error ? err.message : String(err);
          setError(`Failed to load WebAssembly module: ${errorMsg}`);
          console.error('Module loading error:', err);
        }
      }
    };

    loadModule();

    return () => {
      mounted = false;
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, []);

  // Initialize simulator with configuration
  const initialize = useCallback(async (config: SimulationConfig) => {
    if (!module) {
      throw new Error('WebAssembly module not loaded yet');
    }

    try {
      console.log('Initializing simulator with config:', config);

      const sim = new module.RocketSimulator();

      // Configure
      sim.setLauncher(config.elevation, config.azimuth);
      sim.setDiameter(config.diameter);
      sim.setTimestep(config.timestep);

      // Load data if path provided
      // Note: For demo, we'll skip file loading
      // In production, you'd load from public/ directory or embedded data

      // Initialize the simulation system
      sim.setup();

      console.log('Simulator initialized, state dimension:', sim.getStateDimension());

      setSimulator(sim);
      setCurrentState(sim.getFullState());
      setError(null);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : String(err);
      setError(`Initialization failed: ${errorMsg}`);
      throw err;
    }
  }, [module]);

  // Animation loop
  const animate = useCallback((currentTime: number) => {
    if (!simulator || !isRunning) return;

    const deltaTime = (currentTime - lastFrameTimeRef.current) / 1000;
    lastFrameTimeRef.current = currentTime;

    try {
      // Run multiple steps based on playback speed
      const stepsToRun = Math.max(1, Math.floor(playbackSpeed));

      let shouldContinue = true;
      for (let i = 0; i < stepsToRun && shouldContinue; i++) {
        shouldContinue = simulator.step();

        if (!shouldContinue) {
          // Simulation ended (rocket landed)
          setIsRunning(false);
          console.log('Simulation complete at t =', simulator.getTime());
          break;
        }
      }

      // Update state for visualization
      setCurrentState(simulator.getFullState());

      if (shouldContinue) {
        animationFrameRef.current = requestAnimationFrame(animate);
      }
    } catch (err) {
      console.error('Animation error:', err);
      setIsRunning(false);
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [simulator, isRunning, playbackSpeed]);

  // Start simulation
  const start = useCallback(() => {
    if (!simulator || isRunning) return;

    setIsRunning(true);
    lastFrameTimeRef.current = performance.now();
    animationFrameRef.current = requestAnimationFrame(animate);
  }, [simulator, isRunning, animate]);

  // Pause simulation
  const pause = useCallback(() => {
    setIsRunning(false);
    if (animationFrameRef.current) {
      cancelAnimationFrame(animationFrameRef.current);
      animationFrameRef.current = null;
    }
  }, []);

  // Reset simulation
  const reset = useCallback(() => {
    if (!simulator) return;

    pause();

    try {
      simulator.reset();
      setCurrentState(simulator.getFullState());
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [simulator, pause]);

  // Single step
  const step = useCallback(() => {
    if (!simulator || isRunning) return;

    try {
      const shouldContinue = simulator.step();
      setCurrentState(simulator.getFullState());

      if (!shouldContinue) {
        console.log('Simulation complete');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [simulator, isRunning]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, []);

  return {
    isReady: !!module,
    isInitialized: !!simulator && simulator.isInitialized(),
    isRunning,
    currentState,
    error,
    initialize,
    start,
    pause,
    reset,
    step,
    playbackSpeed,
    setPlaybackSpeed,
  };
}
