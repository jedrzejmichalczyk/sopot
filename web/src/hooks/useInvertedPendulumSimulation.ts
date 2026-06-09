import { useState, useEffect, useRef, useCallback } from 'react';
import type { SopotModule, InvertedPendulumSimulator } from '../types/sopot';
import type { PendulumState, PendulumVisualizationData } from '../components/InvertedPendulumVisualization';
import { loadSopotWasmModule } from '../utils/wasmLoader';

/**
 * Hook for the cart-N-pendulum simulation (six inverted links) using WASM.
 *
 * All physics and LQR control run in C++ via WebAssembly. The frontend
 * handles visualization and user interaction.
 */
export function useInvertedPendulumSimulation() {
  const [isReady, setIsReady] = useState(false);
  const [isInitialized, setIsInitialized] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [currentState, setCurrentState] = useState<PendulumState | null>(null);
  const [visualizationData, setVisualizationData] = useState<PendulumVisualizationData | null>(null);
  const [playbackSpeed, setPlaybackSpeedState] = useState(1.0);
  const [controllerEnabled, setControllerEnabledState] = useState(true);
  const [simulationFailed, setSimulationFailed] = useState(false);
  const [numLinks, setNumLinks] = useState(6);

  const moduleRef = useRef<SopotModule | null>(null);
  const simulatorRef = useRef<InvertedPendulumSimulator | null>(null);
  const animationFrameRef = useRef<number | null>(null);
  const lastTimeRef = useRef<number>(0);

  // Load WASM module
  useEffect(() => {
    let mounted = true;

    const loadModule = async () => {
      try {
        console.log('[Pendulum] Loading WASM module...');
        const module = await loadSopotWasmModule();
        if (!mounted) return;
        moduleRef.current = module;

        if (module.InvertedPendulumSimulator) {
          console.log('[Pendulum] WASM module loaded with InvertedPendulumSimulator');
          setIsReady(true);
        } else {
          console.warn('[Pendulum] InvertedPendulumSimulator not found in WASM module');
          setError('InvertedPendulumSimulator not available. Rebuild WASM module with pendulum support.');
        }
      } catch (err) {
        if (!mounted) return;
        console.error('[Pendulum] Failed to load WASM module:', err);
        setError(`Failed to load physics engine: ${err instanceof Error ? err.message : String(err)}`);
      }
    };

    loadModule();
    return () => { mounted = false; };
  }, []);

  // Update state from simulator
  const updateState = useCallback(() => {
    const simulator = simulatorRef.current;
    if (!simulator) return;

    const fullState = simulator.getFullState();
    setCurrentState({
      time: fullState.time,
      x: fullState.x,
      xdot: fullState.xdot,
      angles: fullState.angles,
      omegas: fullState.omegas,
      controlForce: fullState.controlForce,
      numLinks: fullState.numLinks,
    });

    const vizData = simulator.getVisualizationData();
    setVisualizationData(vizData);
  }, []);

  // Initialize simulation
  const initialize = useCallback((
    initialAngleRad?: number,
    cartMass?: number,
    linkMass?: number,
    linkLength?: number,
  ) => {
    const module = moduleRef.current;
    if (!module || !module.InvertedPendulumSimulator) {
      setError('WASM module not ready');
      return;
    }

    try {
      if (simulatorRef.current) {
        simulatorRef.current.delete();
      }

      const SimulatorClass = module.InvertedPendulumSimulator;
      simulatorRef.current = new SimulatorClass();
      const simulator = simulatorRef.current;

      simulator.setupDefault();

      if (cartMass !== undefined || linkMass !== undefined || linkLength !== undefined) {
        simulator.setUniformParameters(
          cartMass ?? 2.0,
          linkMass ?? 0.1,
          linkLength ?? 0.3,
          9.81,
        );
        // Re-arm controller for the new parameters.
        simulator.setupDefault();
      }

      if (initialAngleRad !== undefined) {
        simulator.setInitialAnglesUniform(initialAngleRad);
      }

      simulator.reset();
      setNumLinks(simulator.getNumLinks());
      updateState();

      setIsInitialized(true);
      setSimulationFailed(false);
      setError(null);

      console.log('[Pendulum] Simulation initialized with', simulator.getNumLinks(), 'links');
      console.log('[Pendulum] LQR Gains:', simulator.getLQRGain());
    } catch (err) {
      console.error('[Pendulum] Failed to initialize:', err);
      setError(`Failed to initialize: ${err instanceof Error ? err.message : String(err)}`);
    }
  }, [updateState]);

  // Animation loop
  const animate = useCallback((timestamp: number) => {
    if (!isRunning || !simulatorRef.current) return;

    const simulator = simulatorRef.current;

    if (lastTimeRef.current === 0) {
      lastTimeRef.current = timestamp;
    }

    const realDelta = (timestamp - lastTimeRef.current) / 1000;
    lastTimeRef.current = timestamp;

    const simDelta = realDelta * playbackSpeed;
    const dt = 0.002; // 2ms fixed physics timestep (stiff six-link system)
    const steps = Math.min(Math.floor(simDelta / dt), 40);

    for (let i = 0; i < steps; i++) {
      const stillRunning = simulator.step();
      if (!stillRunning) {
        setIsRunning(false);
        setSimulationFailed(true);
        console.log('[Pendulum] Simulation stopped: a pendulum fell');
        break;
      }
    }

    updateState();

    if (isRunning && !simulationFailed) {
      animationFrameRef.current = requestAnimationFrame(animate);
    }
  }, [isRunning, playbackSpeed, simulationFailed, updateState]);

  useEffect(() => {
    if (isRunning && simulatorRef.current) {
      lastTimeRef.current = 0;
      animationFrameRef.current = requestAnimationFrame(animate);
    } else if (animationFrameRef.current) {
      cancelAnimationFrame(animationFrameRef.current);
      animationFrameRef.current = null;
    }

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [isRunning, animate]);

  const start = useCallback(() => {
    if (isInitialized && !simulationFailed) {
      setIsRunning(true);
    }
  }, [isInitialized, simulationFailed]);

  const pause = useCallback(() => {
    setIsRunning(false);
  }, []);

  const reset = useCallback(() => {
    setIsRunning(false);
    if (simulatorRef.current) {
      simulatorRef.current.reset();
      updateState();
      setSimulationFailed(false);
    }
  }, [updateState]);

  const setPlaybackSpeed = useCallback((speed: number) => {
    setPlaybackSpeedState(Math.max(0.1, Math.min(10, speed)));
  }, []);

  const setControllerEnabled = useCallback((enabled: boolean) => {
    if (simulatorRef.current) {
      simulatorRef.current.setControllerEnabled(enabled);
      setControllerEnabledState(enabled);
    }
  }, []);

  const applyDisturbance = useCallback((type: 'cart' | 'link', index: number, impulse: number) => {
    if (!simulatorRef.current) return;
    if (type === 'cart') {
      simulatorRef.current.applyCartImpulse(impulse);
    } else {
      simulatorRef.current.applyLinkImpulse(index, impulse);
    }
    updateState();
  }, [updateState]);

  // Cleanup
  useEffect(() => {
    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      if (simulatorRef.current) {
        simulatorRef.current.delete();
      }
    };
  }, []);

  const getHistory = useCallback(() => {
    if (!simulatorRef.current) return null;
    return simulatorRef.current.getHistory();
  }, []);

  const getLQRGains = useCallback(() => {
    if (!simulatorRef.current) return null;
    return simulatorRef.current.getLQRGain();
  }, []);

  return {
    // State
    isReady,
    isInitialized,
    isRunning,
    error,
    currentState,
    visualizationData,
    playbackSpeed,
    controllerEnabled,
    simulationFailed,
    numLinks,

    // Actions
    initialize,
    start,
    pause,
    reset,
    setPlaybackSpeed,
    setControllerEnabled,
    applyDisturbance,
    getHistory,
    getLQRGains,
  };
}

export default useInvertedPendulumSimulation;
