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
  simulator: RocketSimulator | null;

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
  const isRunningRef = useRef(false);

  // Load WebAssembly module on mount
  useEffect(() => {
    let mounted = true;

    const loadModule = async () => {
      try {
        const startTime = performance.now();

        // Import the SOPOT WebAssembly module
        // Use base URL to handle GitHub Pages deployment path
        const basePath = import.meta.env.BASE_URL || '/';
        const moduleUrl = `${basePath}sopot.js`;

        console.log(`[WASM] Loading module from: ${moduleUrl}`);

        // Fetch the module and create a blob URL to bypass Vite's transform
        const response = await fetch(moduleUrl);
        if (!response.ok) {
          throw new Error(`Failed to fetch ${moduleUrl}: ${response.status}`);
        }
        const moduleText = await response.text();
        const blob = new Blob([moduleText], { type: 'application/javascript' });
        const blobUrl = URL.createObjectURL(blob);

        // @ts-ignore - Dynamic import of WebAssembly
        const createSopotModule = await import(/* @vite-ignore */ blobUrl);
        URL.revokeObjectURL(blobUrl);

        if (!mounted) return;

        const loaderLoadTime = performance.now() - startTime;
        console.log(`[WASM] Loader script loaded in ${loaderLoadTime.toFixed(2)}ms`);

        const instantiateStartTime = performance.now();
        const moduleInstance = await createSopotModule.default({
          locateFile: (path: string) => {
            // WASM file should be loaded from the public directory
            if (path.endsWith('.wasm')) {
              return `${basePath}${path}`;
            }
            return path;
          }
        });
        const instantiateTime = performance.now() - instantiateStartTime;

        const totalLoadTime = performance.now() - startTime;

        if (mounted) {
          setModule(moduleInstance);
          console.log(`[WASM] Module instantiated in ${instantiateTime.toFixed(2)}ms`);
          console.log(`[WASM] Total load time: ${totalLoadTime.toFixed(2)}ms`);

          // Log metrics for monitoring
          if (typeof window !== 'undefined' && (window as any).gtag) {
            (window as any).gtag('event', 'wasm_load_complete', {
              load_time_ms: Math.round(totalLoadTime),
              loader_time_ms: Math.round(loaderLoadTime),
              instantiate_time_ms: Math.round(instantiateTime),
            });
          }
        }
      } catch (err) {
        if (mounted) {
          const errorMsg = err instanceof Error ? err.message : String(err);
          setError(`Failed to load WebAssembly module: ${errorMsg}`);
          console.error('[WASM] Module loading error:', err);
          console.error('[WASM] Stack trace:', err instanceof Error ? err.stack : 'N/A');

          // Log error metrics
          if (typeof window !== 'undefined' && (window as any).gtag) {
            (window as any).gtag('event', 'wasm_load_error', {
              error_message: errorMsg,
            });
          }
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

      // Load demo data (embedded in WASM, no external files needed)
      sim.loadDemoData();

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

  // Animation loop - uses ref to avoid stale closure
  const animate = useCallback((currentTime: number) => {
    if (!simulator || !isRunningRef.current) return;

    lastFrameTimeRef.current = currentTime;

    try {
      // Run multiple steps based on playback speed
      const stepsToRun = Math.max(1, Math.floor(playbackSpeed));

      let shouldContinue = true;
      for (let i = 0; i < stepsToRun && shouldContinue; i++) {
        // Debug: log state before step
        if (simulator.getTime() < 0.1) {
          console.log('Before step:', simulator.getDebugInfo());
        }

        shouldContinue = simulator.step();

        // Debug: log state after step
        if (simulator.getTime() < 0.1) {
          console.log('After step:', simulator.getDebugInfo(), 'continue:', shouldContinue);
        }

        if (!shouldContinue) {
          // Simulation ended (rocket landed)
          isRunningRef.current = false;
          setIsRunning(false);
          console.log('Simulation ended at t =', simulator.getTime(), simulator.getDebugInfo());
          break;
        }
      }

      // Update state for visualization
      setCurrentState(simulator.getFullState());

      if (shouldContinue && isRunningRef.current) {
        animationFrameRef.current = requestAnimationFrame(animate);
      }
    } catch (err) {
      console.error('Animation error:', err);
      isRunningRef.current = false;
      setIsRunning(false);
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [simulator, playbackSpeed]);

  // Start simulation
  const start = useCallback(() => {
    if (!simulator || isRunningRef.current) return;

    isRunningRef.current = true;
    setIsRunning(true);
    lastFrameTimeRef.current = performance.now();
    animationFrameRef.current = requestAnimationFrame(animate);
  }, [simulator, animate]);

  // Pause simulation
  const pause = useCallback(() => {
    isRunningRef.current = false;
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
    simulator,
    initialize,
    start,
    pause,
    reset,
    step,
    playbackSpeed,
    setPlaybackSpeed,
  };
}
