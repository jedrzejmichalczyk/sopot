import { useState, useEffect } from 'react';
import * as THREE from 'three';
import { SimulationSelector, SimulationType } from './components/SimulationSelector';
import { RocketVisualization3D } from './components/RocketVisualization3D';
import { Grid2DVisualization } from './components/Grid2DVisualization';
import { TelemetryPanel } from './components/TelemetryPanel';
import { ControlPanel } from './components/ControlPanel';
import { Grid2DControlPanel } from './components/Grid2DControlPanel';
import { PlotPanel } from './components/PlotPanel';
import { useRocketSimulation } from './hooks/useRocketSimulation';
import { useGrid2DSimulation } from './hooks/useGrid2DSimulation';
import type { TimeSeriesData } from './types/sopot';

function App() {
  const [simulationType, setSimulationType] = useState<SimulationType>('rocket');
  const [showVelocities, setShowVelocities] = useState(false);
  const [showGrid, setShowGrid] = useState(true);

  const rocketSim = useRocketSimulation();
  const gridSim = useGrid2DSimulation(5, 5);

  const [trajectoryHistory, setTrajectoryHistory] = useState<
    Array<{ position: THREE.Vector3; time: number }>
  >([]);
  const [timeSeries, setTimeSeries] = useState<TimeSeriesData | null>(null);

  // Get the active simulation based on type
  const activeSim = simulationType === 'rocket' ? rocketSim : gridSim;

  // Track trajectory history (rocket only)
  useEffect(() => {
    if (simulationType !== 'rocket' || !rocketSim.currentState) return;

    const { position, time } = rocketSim.currentState;

    // Convert ENU to Three.js coordinates
    const threePosition = new THREE.Vector3(
      position.x,
      position.z, // Z is up in SOPOT
      -position.y // Y is North in SOPOT, -Z in Three.js
    );

    setTrajectoryHistory((prev) => {
      const maxPoints = 1000;
      const newHistory = [...prev, { position: threePosition, time }];

      if (newHistory.length > maxPoints) {
        return newHistory.slice(-maxPoints);
      }

      return newHistory;
    });
  }, [simulationType, rocketSim.currentState]);

  // Reset trajectory when simulation resets
  useEffect(() => {
    if (
      simulationType === 'rocket' &&
      rocketSim.currentState &&
      rocketSim.currentState.time < 0.1 &&
      trajectoryHistory.length > 10
    ) {
      setTrajectoryHistory([]);
    }
  }, [simulationType, rocketSim.currentState, trajectoryHistory.length]);

  // Fetch time-series data periodically when rocket simulation is running
  useEffect(() => {
    if (simulationType !== 'rocket' || !rocketSim.isRunning || !rocketSim.simulator) return;

    const interval = setInterval(() => {
      try {
        const data = rocketSim.simulator!.getTimeSeries();
        console.log('[PlotPanel] Time series data points:', data?.time?.length || 0);
        setTimeSeries(data);
      } catch (error) {
        console.error('Error fetching time series:', error);
      }
    }, 500);

    return () => clearInterval(interval);
  }, [simulationType, rocketSim.isRunning, rocketSim.simulator]);

  // Get final time-series data when rocket simulation pauses
  useEffect(() => {
    if (
      simulationType === 'rocket' &&
      !rocketSim.isRunning &&
      rocketSim.simulator &&
      rocketSim.isInitialized
    ) {
      try {
        const data = rocketSim.simulator.getTimeSeries();
        setTimeSeries(data);
      } catch (error) {
        console.error('Error fetching time series:', error);
      }
    }
  }, [simulationType, rocketSim.isRunning, rocketSim.simulator, rocketSim.isInitialized]);

  // Handle simulation type change - reset both simulations
  const handleSimulationTypeChange = (type: SimulationType) => {
    console.log(`[App] Switching simulation type to: ${type}`);

    // Pause current simulation
    if (rocketSim.isRunning) rocketSim.pause();
    if (gridSim.isRunning) gridSim.pause();

    setSimulationType(type);
    setTrajectoryHistory([]);
    setTimeSeries(null);
  };

  // Render center panel based on simulation type
  const renderVisualization = () => {
    if (simulationType === 'rocket') {
      if (rocketSim.isInitialized) {
        return (
          <RocketVisualization3D
            state={rocketSim.currentState}
            trajectoryHistory={trajectoryHistory}
          />
        );
      }
      return renderPlaceholder('🚀 SOPOT Rocket Simulation', rocketSim.isReady);
    } else {
      if (gridSim.isInitialized) {
        return (
          <Grid2DVisualization
            state={gridSim.currentState}
            showVelocities={showVelocities}
            showGrid={showGrid}
          />
        );
      }
      return renderPlaceholder('🎨 SOPOT 2D Grid Simulation', gridSim.isReady);
    }
  };

  const renderPlaceholder = (title: string, isReady: boolean) => (
    <div style={styles.placeholder}>
      <div style={styles.placeholderContent}>
        <h1 style={styles.placeholderTitle}>{title}</h1>
        <p style={styles.placeholderText}>
          C++20 Physics Simulation compiled to WebAssembly
        </p>
        <p style={styles.placeholderText}>Initialize the simulation to begin</p>
        {!isReady && (
          <div style={styles.loadingIndicator}>
            <div style={styles.spinner} />
            <p style={styles.loadingText}>Loading WebAssembly module...</p>
          </div>
        )}
      </div>
    </div>
  );

  return (
    <div style={styles.container}>
      {/* Top section: 3-column layout */}
      <div style={styles.topSection}>
        {/* Left Panel: Controls */}
        <div style={styles.leftPanel}>
          <SimulationSelector
            currentSimulation={simulationType}
            onSimulationChange={handleSimulationTypeChange}
            disabled={activeSim.isRunning}
          />

          {simulationType === 'rocket' ? (
            <ControlPanel
              isReady={rocketSim.isReady}
              isInitialized={rocketSim.isInitialized}
              isRunning={rocketSim.isRunning}
              error={rocketSim.error}
              playbackSpeed={rocketSim.playbackSpeed}
              onInitialize={rocketSim.initialize}
              onStart={rocketSim.start}
              onPause={rocketSim.pause}
              onReset={rocketSim.reset}
              onStep={rocketSim.step}
              onPlaybackSpeedChange={rocketSim.setPlaybackSpeed}
            />
          ) : (
            <Grid2DControlPanel
              isReady={gridSim.isReady}
              isInitialized={gridSim.isInitialized}
              isRunning={gridSim.isRunning}
              error={gridSim.error}
              playbackSpeed={gridSim.playbackSpeed}
              onInitialize={gridSim.initialize}
              onStart={gridSim.start}
              onPause={gridSim.pause}
              onReset={gridSim.reset}
              onStep={gridSim.step}
              onPlaybackSpeedChange={gridSim.setPlaybackSpeed}
              showVelocities={showVelocities}
              showGrid={showGrid}
              onShowVelocitiesChange={setShowVelocities}
              onShowGridChange={setShowGrid}
            />
          )}
        </div>

        {/* Center Panel: Visualization */}
        <div style={styles.centerPanel}>{renderVisualization()}</div>

        {/* Right Panel: Telemetry (rocket only for now) */}
        <div style={styles.rightPanel}>
          {simulationType === 'rocket' ? (
            <TelemetryPanel
              state={rocketSim.currentState}
              isRunning={rocketSim.isRunning}
            />
          ) : (
            <div style={styles.telemetryPlaceholder}>
              <h3 style={styles.telemetryTitle}>Grid Info</h3>
              {gridSim.currentState && (
                <div style={styles.gridInfo}>
                  <div style={styles.infoRow}>
                    <span>Time:</span>
                    <span>{gridSim.currentState.time.toFixed(3)}s</span>
                  </div>
                  <div style={styles.infoRow}>
                    <span>Grid Size:</span>
                    <span>{gridSim.currentState.rows}×{gridSim.currentState.cols}</span>
                  </div>
                  <div style={styles.infoRow}>
                    <span>Masses:</span>
                    <span>{gridSim.currentState.positions.length}</span>
                  </div>
                </div>
              )}
            </div>
          )}
        </div>
      </div>

      {/* Bottom section: Plotting panel (rocket only for now) */}
      {simulationType === 'rocket' && (
        <div style={styles.bottomSection}>
          <PlotPanel timeSeries={timeSeries} />
        </div>
      )}
    </div>
  );
}

const styles = {
  container: {
    display: 'flex',
    flexDirection: 'column' as const,
    width: '100vw',
    height: '100vh',
    backgroundColor: '#1a1a1a',
  },
  topSection: {
    display: 'flex',
    flex: 1,
    minHeight: 0,
  },
  bottomSection: {
    height: '300px',
    minHeight: '200px',
    maxHeight: '400px',
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
  telemetryPlaceholder: {
    height: '100%',
    backgroundColor: '#2c3e50',
    color: '#ecf0f1',
    padding: '20px',
  },
  telemetryTitle: {
    margin: '0 0 20px 0',
    fontSize: '20px',
    fontWeight: 'bold' as const,
  },
  gridInfo: {
    display: 'flex',
    flexDirection: 'column' as const,
    gap: '15px',
  },
  infoRow: {
    display: 'flex',
    justifyContent: 'space-between',
    fontSize: '14px',
    padding: '10px',
    backgroundColor: '#34495e',
    borderRadius: '4px',
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
