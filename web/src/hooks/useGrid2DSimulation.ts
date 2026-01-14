import { useState, useEffect, useRef, useCallback } from 'react';
import type { Grid2DState } from '../components/Grid2DVisualization';

// Mock simulation for now - will be replaced with actual WASM implementation
interface Grid2DSimulator {
  step: (dt: number) => void;
  getState: () => Grid2DState;
  reset: () => void;
  destroy: () => void;
}

function createMockGrid2DSimulator(rows: number, cols: number): Grid2DSimulator {
  let time = 0;
  const numMasses = rows * cols;
  const spacing = 0.5;

  // Initialize positions in a grid
  const positions: Array<{ x: number; y: number }> = [];
  const velocities: Array<{ vx: number; vy: number }> = [];

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      positions.push({
        x: c * spacing,
        y: r * spacing,
      });
      velocities.push({ vx: 0, vy: 0 });
    }
  }

  // Add initial perturbation to center
  const centerIdx = Math.floor(rows / 2) * cols + Math.floor(cols / 2);
  if (centerIdx < numMasses) {
    positions[centerIdx].y += 0.3;
  }

  // Simple spring-mass dynamics
  const mass = 0.5;
  const stiffness = 30.0;
  const damping = 0.8;
  const restLength = spacing;

  return {
    step(dt: number) {
      // Compute forces
      const forces: Array<{ fx: number; fy: number }> = Array(numMasses)
        .fill(null)
        .map(() => ({ fx: 0, fy: 0 }));

      // Horizontal springs
      for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols - 1; c++) {
          const idx1 = r * cols + c;
          const idx2 = r * cols + c + 1;

          const dx = positions[idx2].x - positions[idx1].x;
          const dy = positions[idx2].y - positions[idx1].y;
          const length = Math.sqrt(dx * dx + dy * dy);

          // Guard against zero-length springs to prevent NaN propagation
          if (length < 1e-10) continue;

          const extension = length - restLength;

          const fx = (stiffness * extension * dx) / length;
          const fy = (stiffness * extension * dy) / length;

          // Damping
          const dvx = velocities[idx2].vx - velocities[idx1].vx;
          const dvy = velocities[idx2].vy - velocities[idx1].vy;
          const relVel = (dvx * dx + dvy * dy) / length;
          const dampingFx = (damping * relVel * dx) / length;
          const dampingFy = (damping * relVel * dy) / length;

          forces[idx1].fx += fx + dampingFx;
          forces[idx1].fy += fy + dampingFy;
          forces[idx2].fx -= fx + dampingFx;
          forces[idx2].fy -= fy + dampingFy;
        }
      }

      // Vertical springs
      for (let r = 0; r < rows - 1; r++) {
        for (let c = 0; c < cols; c++) {
          const idx1 = r * cols + c;
          const idx2 = (r + 1) * cols + c;

          const dx = positions[idx2].x - positions[idx1].x;
          const dy = positions[idx2].y - positions[idx1].y;
          const length = Math.sqrt(dx * dx + dy * dy);

          // Guard against zero-length springs to prevent NaN propagation
          if (length < 1e-10) continue;

          const extension = length - restLength;

          const fx = (stiffness * extension * dx) / length;
          const fy = (stiffness * extension * dy) / length;

          // Damping
          const dvx = velocities[idx2].vx - velocities[idx1].vx;
          const dvy = velocities[idx2].vy - velocities[idx1].vy;
          const relVel = (dvx * dx + dvy * dy) / length;
          const dampingFx = (damping * relVel * dx) / length;
          const dampingFy = (damping * relVel * dy) / length;

          forces[idx1].fx += fx + dampingFx;
          forces[idx1].fy += fy + dampingFy;
          forces[idx2].fx -= fx + dampingFx;
          forces[idx2].fy -= fy + dampingFy;
        }
      }

      // Integrate
      for (let i = 0; i < numMasses; i++) {
        const ax = forces[i].fx / mass;
        const ay = forces[i].fy / mass;

        velocities[i].vx += ax * dt;
        velocities[i].vy += ay * dt;

        positions[i].x += velocities[i].vx * dt;
        positions[i].y += velocities[i].vy * dt;
      }

      time += dt;
    },

    getState(): Grid2DState {
      return {
        time,
        rows,
        cols,
        positions: [...positions],
        velocities: [...velocities],
      };
    },

    reset() {
      time = 0;
      for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
          const idx = r * cols + c;
          positions[idx] = { x: c * spacing, y: r * spacing };
          velocities[idx] = { vx: 0, vy: 0 };
        }
      }
      // Re-add initial perturbation
      if (centerIdx < numMasses) {
        positions[centerIdx].y += 0.3;
      }
    },

    destroy() {
      // Cleanup if needed
    },
  };
}

export function useGrid2DSimulation(rows = 5, cols = 5) {
  const [isReady, setIsReady] = useState(false);
  const [isInitialized, setIsInitialized] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [currentState, setCurrentState] = useState<Grid2DState | null>(null);
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0);

  const simulatorRef = useRef<Grid2DSimulator | null>(null);
  const animationFrameRef = useRef<number | null>(null);
  const lastTimeRef = useRef<number>(0);

  // Mark as ready immediately (or after WASM loads)
  useEffect(() => {
    setIsReady(true);
  }, []);

  const initialize = useCallback(() => {
    try {
      console.log(`[Grid2D] Initializing ${rows}×${cols} grid simulation`);
      simulatorRef.current = createMockGrid2DSimulator(rows, cols);
      setCurrentState(simulatorRef.current.getState());
      setIsInitialized(true);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to initialize');
      console.error('[Grid2D] Initialization error:', err);
    }
  }, [rows, cols]);

  const reset = useCallback(() => {
    if (!simulatorRef.current) return;

    console.log('[Grid2D] Resetting simulation');
    setIsRunning(false);

    // Destroy and recreate to properly clean up resources
    simulatorRef.current.destroy();
    simulatorRef.current = null;
    setCurrentState(null);
    setIsInitialized(false);
  }, []);

  const start = useCallback(() => {
    if (!isInitialized) return;
    console.log('[Grid2D] Starting simulation');
    setIsRunning(true);
    lastTimeRef.current = performance.now();
  }, [isInitialized]);

  const pause = useCallback(() => {
    console.log('[Grid2D] Pausing simulation');
    setIsRunning(false);
  }, []);

  const step = useCallback(() => {
    if (!simulatorRef.current || isRunning) return;

    const dt = 0.01; // Fixed timestep
    simulatorRef.current.step(dt);
    setCurrentState(simulatorRef.current.getState());
  }, [isRunning]);

  // Animation loop
  useEffect(() => {
    if (!isRunning || !simulatorRef.current) return;

    const animate = (currentTime: number) => {
      const deltaTime = (currentTime - lastTimeRef.current) / 1000; // Convert to seconds
      lastTimeRef.current = currentTime;

      if (deltaTime > 0 && deltaTime < 0.1) {
        // Limit max dt to avoid instability
        const dt = Math.min(deltaTime * playbackSpeed, 0.05);
        simulatorRef.current!.step(dt);
        setCurrentState(simulatorRef.current!.getState());
      }

      animationFrameRef.current = requestAnimationFrame(animate);
    };

    animationFrameRef.current = requestAnimationFrame(animate);

    return () => {
      if (animationFrameRef.current !== null) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [isRunning, playbackSpeed]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (simulatorRef.current) {
        simulatorRef.current.destroy();
      }
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
