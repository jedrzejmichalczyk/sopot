import { useState, useEffect, useRef, useCallback } from 'react';
import type { SopotModule } from '../types/sopot';
import type { PendulumState, PendulumVisualizationData } from '../components/InvertedPendulumVisualization';
import { loadSopotWasmModule } from '../utils/wasmLoader';

/**
 * Interface for the InvertedPendulumSimulator from WASM
 */
interface InvertedPendulumSimulator {
  setupDefault(): void;
  reset(): void;
  step(): boolean;
  stepWithDt(dt: number): boolean;
  setControllerEnabled(enabled: boolean): void;
  applyCartImpulse(impulse: number): void;
  applyLink1Impulse(impulse: number): void;
  applyLink2Impulse(impulse: number): void;
  setParameters(cartMass: number, m1: number, m2: number, L1: number, L2: number, g: number): void;
  setInitialState(x: number, theta1: number, theta2: number, xdot: number, omega1: number, omega2: number): void;
  setTimestep(dt: number): void;
  setMaxForce(maxForce: number): void;
  configureLQR(qDiag: number[], r: number): void;

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

  delete(): void;
}

/**
 * Hook for inverted double pendulum simulation using WASM
 *
 * All physics and control computations run in C++ via WebAssembly.
 * The frontend handles visualization and user interaction.
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

        // Load the SOPOT WebAssembly module
        const module = await loadSopotWasmModule();

        if (!mounted) return;

        moduleRef.current = module;

        // Check if InvertedPendulumSimulator is available
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

    return () => {
      mounted = false;
    };
  }, []);

  // Update state from simulator
  const updateState = useCallback(() => {
    const simulator = simulatorRef.current;
    if (!simulator) return;

    const fullState = simulator.getFullState();
    setCurrentState({
      time: fullState.time,
      x: fullState.x,
      theta1: fullState.theta1,
      theta2: fullState.theta2,
      xdot: fullState.xdot,
      omega1: fullState.omega1,
      omega2: fullState.omega2,
      controlForce: fullState.controlForce,
    });

    const vizData = simulator.getVisualizationData();
    setVisualizationData(vizData);
  }, []);

  // Initialize simulation
  const initialize = useCallback((
    cartMass?: number,
    m1?: number,
    m2?: number,
    L1?: number,
    L2?: number,
    initialTheta1?: number,
    initialTheta2?: number,
  ) => {
    const module = moduleRef.current;
    if (!module || !module.InvertedPendulumSimulator) {
      setError('WASM module not ready');
      return;
    }

    try {
      // Clean up existing simulator
      if (simulatorRef.current) {
        simulatorRef.current.delete();
      }

      // Create new simulator - we've verified InvertedPendulumSimulator exists above
      const SimulatorClass = module.InvertedPendulumSimulator!;
      simulatorRef.current = new SimulatorClass();
      const simulator = simulatorRef.current!;

      // Set up with default or custom parameters
      simulator.setupDefault();

      // Override with custom parameters if provided
      if (cartMass !== undefined || m1 !== undefined) {
        simulator.setParameters(
          cartMass ?? 1.0,
          m1 ?? 0.5,
          m2 ?? 0.5,
          L1 ?? 0.7,
          L2 ?? 0.7,
          9.81
        );
      }

      if (initialTheta1 !== undefined || initialTheta2 !== undefined) {
        simulator.setInitialState(
          0,
          initialTheta1 ?? 0.1,
          initialTheta2 ?? 0.05,
          0, 0, 0
        );
      }

      simulator.reset();
      updateState();

      setIsInitialized(true);
      setSimulationFailed(false);
      setError(null);

      console.log('[Pendulum] Simulation initialized');
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

    // Calculate elapsed real time
    if (lastTimeRef.current === 0) {
      lastTimeRef.current = timestamp;
    }

    const realDelta = (timestamp - lastTimeRef.current) / 1000; // seconds
    lastTimeRef.current = timestamp;

    // Calculate simulation steps based on playback speed
    const simDelta = realDelta * playbackSpeed;
    const dt = 0.005; // 5ms fixed physics timestep
    const steps = Math.min(Math.floor(simDelta / dt), 20); // Cap at 20 steps per frame

    // Run physics steps
    for (let i = 0; i < steps; i++) {
      const stillRunning = simulator.step();
      if (!stillRunning) {
        // Pendulum has fallen
        setIsRunning(false);
        setSimulationFailed(true);
        console.log('[Pendulum] Simulation stopped: pendulum fell');
        break;
      }
    }

    // Update visualization state
    updateState();

    // Continue animation loop
    if (isRunning && !simulationFailed) {
      animationFrameRef.current = requestAnimationFrame(animate);
    }
  }, [isRunning, playbackSpeed, simulationFailed, updateState]);

  // Start/stop animation when isRunning changes
  useEffect(() => {
    if (isRunning && simulatorRef.current) {
      lastTimeRef.current = 0;
      animationFrameRef.current = requestAnimationFrame(animate);
    } else {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
        animationFrameRef.current = null;
      }
    }

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [isRunning, animate]);

  // Control functions
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

  const applyDisturbance = useCallback((type: 'cart' | 'link1' | 'link2', impulse: number) => {
    if (!simulatorRef.current) return;

    switch (type) {
      case 'cart':
        simulatorRef.current.applyCartImpulse(impulse);
        break;
      case 'link1':
        simulatorRef.current.applyLink1Impulse(impulse);
        break;
      case 'link2':
        simulatorRef.current.applyLink2Impulse(impulse);
        break;
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

  // Get history for plotting
  const getHistory = useCallback(() => {
    if (!simulatorRef.current) return null;
    return simulatorRef.current.getHistory();
  }, []);

  // Get LQR gains
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
