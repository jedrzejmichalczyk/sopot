import { useState, useEffect } from 'react';
import * as THREE from 'three';
import { RocketVisualization3D } from './components/RocketVisualization3D';
import { TelemetryPanel } from './components/TelemetryPanel';
import { ControlPanel } from './components/ControlPanel';
import { useRocketSimulation } from './hooks/useRocketSimulation';

function App() {
  const simulation = useRocketSimulation();
  const [trajectoryHistory, setTrajectoryHistory] = useState<
    Array<{ position: THREE.Vector3; time: number }>
  >([]);

  // Track trajectory history
  useEffect(() => {
    if (!simulation.currentState) return;

    const { position, time } = simulation.currentState;

    // Convert ENU to Three.js coordinates
    const threePosition = new THREE.Vector3(
      position.x,
      position.z, // Z is up in SOPOT
      -position.y // Y is North in SOPOT, -Z in Three.js
    );

    setTrajectoryHistory((prev) => {
      // Limit history to last 1000 points to prevent memory issues
      const maxPoints = 1000;
      const newHistory = [...prev, { position: threePosition, time }];

      if (newHistory.length > maxPoints) {
        return newHistory.slice(-maxPoints);
      }

      return newHistory;
    });
  }, [simulation.currentState]);

  // Reset trajectory when simulation resets
  useEffect(() => {
    if (
      simulation.currentState &&
      simulation.currentState.time < 0.1 &&
      trajectoryHistory.length > 10
    ) {
      setTrajectoryHistory([]);
    }
  }, [simulation.currentState, trajectoryHistory.length]);

  return (
    <div style={styles.container}>
      {/* Left Panel: Controls */}
      <div style={styles.leftPanel}>
        <ControlPanel
          isReady={simulation.isReady}
          isInitialized={simulation.isInitialized}
          isRunning={simulation.isRunning}
          error={simulation.error}
          playbackSpeed={simulation.playbackSpeed}
          onInitialize={simulation.initialize}
          onStart={simulation.start}
          onPause={simulation.pause}
          onReset={simulation.reset}
          onStep={simulation.step}
          onPlaybackSpeedChange={simulation.setPlaybackSpeed}
        />
      </div>

      {/* Center Panel: 3D Visualization */}
      <div style={styles.centerPanel}>
        {simulation.isInitialized ? (
          <RocketVisualization3D
            state={simulation.currentState}
            trajectoryHistory={trajectoryHistory}
          />
        ) : (
          <div style={styles.placeholder}>
            <div style={styles.placeholderContent}>
              <h1 style={styles.placeholderTitle}>
                🚀 SOPOT Rocket Simulation
              </h1>
              <p style={styles.placeholderText}>
                C++20 Physics Simulation compiled to WebAssembly
              </p>
              <p style={styles.placeholderText}>
                Initialize the simulation to begin
              </p>
              {!simulation.isReady && (
                <div style={styles.loadingIndicator}>
                  <div style={styles.spinner} />
                  <p style={styles.loadingText}>Loading WebAssembly module...</p>
                </div>
              )}
            </div>
          </div>
        )}
      </div>

      {/* Right Panel: Telemetry */}
      <div style={styles.rightPanel}>
        <TelemetryPanel
          state={simulation.currentState}
          isRunning={simulation.isRunning}
        />
      </div>
    </div>
  );
}

const styles = {
  container: {
    display: 'flex',
    width: '100vw',
    height: '100vh',
    backgroundColor: '#1a1a1a',
  },
  leftPanel: {
    width: '320px',
    height: '100%',
    borderRight: '2px solid #34495e',
    overflow: 'hidden',
  },
  centerPanel: {
    flex: 1,
    height: '100%',
    position: 'relative' as const,
  },
  rightPanel: {
    width: '320px',
    height: '100%',
    borderLeft: '2px solid #34495e',
    overflow: 'hidden',
  },
  placeholder: {
    width: '100%',
    height: '100%',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
  },
  placeholderContent: {
    textAlign: 'center' as const,
    color: '#fff',
    padding: '40px',
  },
  placeholderTitle: {
    fontSize: '48px',
    marginBottom: '20px',
    fontWeight: 'bold' as const,
  },
  placeholderText: {
    fontSize: '20px',
    marginBottom: '10px',
    opacity: 0.9,
  },
  loadingIndicator: {
    marginTop: '40px',
    display: 'flex',
    flexDirection: 'column' as const,
    alignItems: 'center',
    gap: '20px',
  },
  spinner: {
    width: '50px',
    height: '50px',
    border: '5px solid rgba(255, 255, 255, 0.3)',
    borderTop: '5px solid #fff',
    borderRadius: '50%',
    animation: 'spin 1s linear infinite',
  },
  loadingText: {
    fontSize: '16px',
    opacity: 0.9,
  },
};

// Add keyframe animation for spinner
const styleSheet = document.createElement('style');
styleSheet.textContent = `
  @keyframes spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }
`;
document.head.appendChild(styleSheet);

export default App;
