import { useState, useEffect } from 'react';
import * as THREE from 'three';
import { SimulationSelector, SimulationType } from './components/SimulationSelector';
import { RocketVisualization3D } from './components/RocketVisualization3D';
import { Grid2DVisualization } from './components/Grid2DVisualization';
import { TelemetryPanel } from './components/TelemetryPanel';
import { ControlPanel } from './components/ControlPanel';
import { Grid2DControlPanel } from './components/Grid2DControlPanel';
import { PlotPanel } from './components/PlotPanel';
import { FloatingActionButton } from './components/FloatingActionButton';
import { BottomSheet } from './components/BottomSheet';
import { useRocketSimulation } from './hooks/useRocketSimulation';
import { useGrid2DSimulation } from './hooks/useGrid2DSimulation';
import type { TimeSeriesData } from './types/sopot';
import './styles/responsive.css';

type MobilePanel = 'controls' | 'telemetry' | 'plots' | null;

function App() {
  const [simulationType, setSimulationType] = useState<SimulationType>('rocket');
  const [showVelocities, setShowVelocities] = useState(false);
  const [showGrid, setShowGrid] = useState(true);
  const [mobilePanel, setMobilePanel] = useState<MobilePanel>(null);
  const [isMobile, setIsMobile] = useState(window.innerWidth < 768);

  // Note: Both hooks are instantiated to maintain React hook call order,
  // but inactive simulations are reset when switching types to free resources.
  // The rocket WASM module loads on mount for the primary simulation type.
  const rocketSim = useRocketSimulation();
  const gridSim = useGrid2DSimulation(5, 5);

  const [trajectoryHistory, setTrajectoryHistory] = useState<
    Array<{ position: THREE.Vector3; time: number }>
  >([]);
  const [timeSeries, setTimeSeries] = useState<TimeSeriesData | null>(null);

  // Get the active simulation based on type
  const activeSim = simulationType === 'rocket' ? rocketSim : gridSim;

  // Handle window resize for responsive layout
  useEffect(() => {
    const handleResize = () => {
      const mobile = window.innerWidth < 768;
      setIsMobile(mobile);
      // Close mobile panel when switching to desktop
      if (!mobile) {
        setMobilePanel(null);
      }
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

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

    // Reset the simulation we're switching away from to free resources
    if (simulationType === 'rocket' && rocketSim.isInitialized) {
      rocketSim.reset();
    } else if (simulationType === 'grid2d' && gridSim.isInitialized) {
      gridSim.reset();
    }

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

  const renderPlaceholder = (title: string, isReady: boolean) => {
    // Use accurate description based on simulation type
    const description = simulationType === 'rocket'
      ? 'C++20 Physics Simulation compiled to WebAssembly'
      : 'High-performance physics simulation engine';

    const loadingText = simulationType === 'rocket'
      ? 'Loading WebAssembly module...'
      : 'Loading simulation engine...';

    return (
      <div style={styles.placeholder}>
        <div style={styles.placeholderContent}>
          <h1 style={styles.placeholderTitle}>{title}</h1>
          <p style={styles.placeholderText}>{description}</p>
          <p style={styles.placeholderText}>Initialize the simulation to begin</p>
          {!isReady && (
            <div style={styles.loadingIndicator}>
              <div style={styles.spinner} />
              <p style={styles.loadingText}>{loadingText}</p>
            </div>
          )}
        </div>
      </div>
    );
  };

  // Handle play/pause for FAB
  const handlePlayPause = () => {
    if (activeSim.isRunning) {
      activeSim.pause();
    } else {
      activeSim.start();
    }
  };

  return (
    <div className="app-container">{/* Floating Action Button - Mobile Only */}
      {isMobile && (
        <FloatingActionButton
          isRunning={activeSim.isRunning}
          isInitialized={activeSim.isInitialized}
          simulationType={simulationType}
          onPlayPause={handlePlayPause}
          onReset={activeSim.reset}
          onOpenPanel={(panel) => setMobilePanel(panel)}
        />
      )}

      {/* Main Layout */}
      <div className="app-layout">
        {/* Top section: 3-column layout (desktop/tablet) */}
        <div className="top-section">
          {/* Left Panel: Controls (desktop/tablet) */}
          <div className="left-panel desktop-only">
            <SimulationSelector
              currentSimulation={simulationType}
              onSimulationChange={handleSimulationTypeChange}
              disabled={activeSim.isRunning}
            />

            <div style={styles.controlPanelWrapper}>
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
          </div>

          {/* Center Panel: Visualization */}
          <div className="center-panel">
            {renderVisualization()}
          </div>

          {/* Right Panel: Telemetry (desktop only) */}
          <div className="right-panel desktop-only">
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

        {/* Bottom section: Plotting panel (desktop/tablet, rocket only) */}
        {simulationType === 'rocket' && (
          <div className="bottom-section desktop-only">
            <PlotPanel timeSeries={timeSeries} />
          </div>
        )}
      </div>

      {/* Modern Bottom Sheets - Mobile Only */}
      {isMobile && (
        <>
          {/* Controls Bottom Sheet */}
          <BottomSheet
            isOpen={mobilePanel === 'controls'}
            onClose={() => setMobilePanel(null)}
            title="Controls"
            initialSnapPoint="half"
          >
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
          </BottomSheet>

          {/* Telemetry Bottom Sheet */}
          {simulationType === 'rocket' && (
            <BottomSheet
              isOpen={mobilePanel === 'telemetry'}
              onClose={() => setMobilePanel(null)}
              title="Telemetry"
              initialSnapPoint="half"
            >
              <TelemetryPanel
                state={rocketSim.currentState}
                isRunning={rocketSim.isRunning}
              />
            </BottomSheet>
          )}

          {/* Plots Bottom Sheet */}
          {simulationType === 'rocket' && (
            <BottomSheet
              isOpen={mobilePanel === 'plots'}
              onClose={() => setMobilePanel(null)}
              title="Plots"
              initialSnapPoint="expanded"
            >
              <PlotPanel timeSeries={timeSeries} />
            </BottomSheet>
          )}
        </>
      )}
    </div>
  );
}

const styles = {
  controlPanelWrapper: {
    flex: 1,
    minHeight: 0,
    overflow: 'auto',
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

export default App;
