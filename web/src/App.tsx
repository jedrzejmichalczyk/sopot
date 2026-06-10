import { useState, useEffect } from 'react';
import * as THREE from 'three';
import { SimulationSelector, SimulationType } from './components/SimulationSelector';
import { RocketVisualization3D } from './components/RocketVisualization3D';
import { Grid2DVisualization } from './components/Grid2DVisualization';
import { InvertedPendulumVisualization } from './components/InvertedPendulumVisualization';
import { TelemetryPanel } from './components/TelemetryPanel';
import { ControlPanel } from './components/ControlPanel';
import { Grid2DControlPanel } from './components/Grid2DControlPanel';
import { InvertedPendulumControlPanel } from './components/InvertedPendulumControlPanel';
import { PlotPanel } from './components/PlotPanel';
import { FloatingActionButton } from './components/FloatingActionButton';
import { BottomSheet } from './components/BottomSheet';
import { useRocketSimulation } from './hooks/useRocketSimulation';
import { useGrid2DSimulation } from './hooks/useGrid2DSimulation';
import { useInvertedPendulumSimulation } from './hooks/useInvertedPendulumSimulation';
import type { TimeSeriesData } from './types/sopot';
import './styles/responsive.css';

type MobilePanel = 'controls' | 'telemetry' | 'plots' | null;

function App() {
  const [simulationType, setSimulationType] = useState<SimulationType>('rocket');
  const [showVelocities, setShowVelocities] = useState(false);
  const [showGrid, setShowGrid] = useState(true);
  const [cameraTracking, setCameraTracking] = useState(false);
  // Start with all panels closed: the visualization placeholder has its own
  // "Start" CTA, so we don't cover the screen with a sheet on load.
  const [mobilePanel, setMobilePanel] = useState<MobilePanel>(null);
  const [isMobile, setIsMobile] = useState(window.innerWidth < 768);
  const [pendingAutoStart, setPendingAutoStart] = useState(false);

  // Note: All hooks are instantiated to maintain React hook call order,
  // but inactive simulations are reset when switching types to free resources.
  // The WASM module loads on mount for the selected simulation type.
  const rocketSim = useRocketSimulation();
  const gridSim = useGrid2DSimulation(10);  // Default 10x10 grid
  const pendulumSim = useInvertedPendulumSimulation();

  const [trajectoryHistory, setTrajectoryHistory] = useState<
    Array<{ position: THREE.Vector3; time: number }>
  >([]);
  const [timeSeries, setTimeSeries] = useState<TimeSeriesData | null>(null);

  // Get the active simulation based on type
  const activeSim = simulationType === 'rocket'
    ? rocketSim
    : simulationType === 'grid2d'
      ? gridSim
      : pendulumSim;

  // Run the simulation as soon as a quick-start initialization lands
  useEffect(() => {
    if (pendingAutoStart && activeSim.isInitialized) {
      activeSim.start();
      setPendingAutoStart(false);
    }
  }, [pendingAutoStart, activeSim]);

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

  // Handle simulation type change - reset all simulations
  const handleSimulationTypeChange = (type: SimulationType) => {
    console.log(`[App] Switching simulation type to: ${type}`);

    // Pause all simulations
    if (rocketSim.isRunning) rocketSim.pause();
    if (gridSim.isRunning) gridSim.pause();
    if (pendulumSim.isRunning) pendulumSim.pause();

    // Reset the simulation we're switching away from to free resources
    if (simulationType === 'rocket' && rocketSim.isInitialized) {
      rocketSim.reset();
    } else if (simulationType === 'grid2d' && gridSim.isInitialized) {
      gridSim.reset();
    } else if (simulationType === 'pendulum' && pendulumSim.isInitialized) {
      pendulumSim.reset();
    }

    setSimulationType(type);
    setTrajectoryHistory([]);
    setTimeSeries(null);
    setPendingAutoStart(false);
  };

  // One-tap start with sensible defaults, used by the placeholder CTA.
  // Critical on mobile, where the control panels live behind the FAB menu.
  const handleQuickStart = () => {
    if (simulationType === 'rocket') {
      Promise.resolve(
        rocketSim.initialize({ elevation: 85, azimuth: 0, diameter: 0.16, timestep: 0.01 })
      ).catch((err) => console.error('Quick start failed:', err));
    } else if (simulationType === 'grid2d') {
      Promise.resolve(gridSim.initialize()).catch((err) =>
        console.error('Quick start failed:', err)
      );
    } else {
      pendulumSim.initialize((1.7 * Math.PI) / 180);
    }
    setPendingAutoStart(true);
  };

  // Render center panel based on simulation type
  const renderVisualization = () => {
    if (simulationType === 'rocket') {
      if (rocketSim.isInitialized) {
        return (
          <RocketVisualization3D
            state={rocketSim.currentState}
            trajectoryHistory={trajectoryHistory}
            cameraTracking={cameraTracking}
          />
        );
      }
      return renderPlaceholder('🚀 SOPOT Rocket Simulation', rocketSim.isReady);
    } else if (simulationType === 'grid2d') {
      if (gridSim.isInitialized) {
        return (
          <Grid2DVisualization
            state={gridSim.currentState}
            showVelocities={showVelocities}
            showGrid={showGrid}
            gridType={gridSim.gridType}
            onMassPerturb={gridSim.perturbMass}
          />
        );
      }
      return renderPlaceholder('🎨 SOPOT 2D Grid Simulation', gridSim.isReady);
    } else {
      // Pendulum simulation
      if (pendulumSim.isInitialized) {
        return (
          <div style={styles.visualizationWrapper}>
            <InvertedPendulumVisualization
              state={pendulumSim.currentState}
              visualizationData={pendulumSim.visualizationData}
              numLinks={pendulumSim.numLinks}
              showTelemetry={true}
              showForceArrow={true}
              onApplyImpulse={pendulumSim.applyDisturbance}
            />
            {pendulumSim.simulationFailed ? (
              <div style={styles.vizOverlayCenter}>
                <div style={styles.failedCard}>
                  <p style={styles.failedTitle}>A pendulum fell!</p>
                  <p style={styles.failedText}>A link exceeded 45°. Reset to try again.</p>
                  <button
                    className="touch-button btn-danger"
                    onClick={pendulumSim.reset}
                    style={{ width: '100%' }}
                  >
                    Reset
                  </button>
                </div>
              </div>
            ) : (
              !pendulumSim.isRunning && (
                <button
                  className="touch-button btn-success"
                  style={styles.resumeChip}
                  onClick={pendulumSim.start}
                >
                  ▶ {pendulumSim.currentState && pendulumSim.currentState.time > 0 ? 'Resume' : 'Run simulation'}
                </button>
              )
            )}
          </div>
        );
      }
      return renderPlaceholder('⚖️ Inverted 6-Pendulum on Cart', pendulumSim.isReady);
    }
  };

  const renderPlaceholder = (title: string, isReady: boolean) => {
    // Use accurate description based on simulation type
    const description = simulationType === 'rocket'
      ? 'C++20 Physics Simulation compiled to WebAssembly'
      : simulationType === 'pendulum'
        ? 'LQR-controlled inverted six-pendulum chain with real-time stabilization'
        : 'High-performance physics simulation engine';

    const loadingText = 'Loading WebAssembly module...';

    return (
      <div style={styles.placeholder}>
        <div style={styles.placeholderContent}>
          <h1 style={styles.placeholderTitle}>{title}</h1>
          <p style={styles.placeholderText}>{description}</p>
          {activeSim.error && (
            <p style={styles.placeholderError}>{activeSim.error}</p>
          )}
          {isReady ? (
            <>
              <button
                className="touch-button btn-success"
                style={styles.quickStartButton}
                onClick={handleQuickStart}
              >
                ▶ Start Simulation
              </button>
              <p style={styles.placeholderHint}>
                {isMobile
                  ? 'Tune parameters via the menu button below'
                  : 'Or tune parameters in the panel on the left'}
              </p>
            </>
          ) : !activeSim.error && (
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
                  cameraTracking={cameraTracking}
                  onInitialize={rocketSim.initialize}
                  onStart={rocketSim.start}
                  onPause={rocketSim.pause}
                  onReset={rocketSim.reset}
                  onStep={rocketSim.step}
                  onPlaybackSpeedChange={rocketSim.setPlaybackSpeed}
                  onCameraTrackingChange={setCameraTracking}
                />
              ) : simulationType === 'grid2d' ? (
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
                  gridSize={gridSim.gridSize}
                  onGridSizeChange={gridSim.setGridSize}
                  mass={gridSim.mass}
                  stiffness={gridSim.stiffness}
                  damping={gridSim.damping}
                  gridType={gridSim.gridType}
                  integrator={gridSim.integrator}
                  repulsionEnabled={gridSim.repulsionEnabled}
                  minDistance={gridSim.minDistance}
                  onMassChange={gridSim.setMass}
                  onStiffnessChange={gridSim.setStiffness}
                  onDampingChange={gridSim.setDamping}
                  onGridTypeChange={gridSim.setGridType}
                  onIntegratorChange={gridSim.setIntegrator}
                  onRepulsionEnabledChange={gridSim.setRepulsionEnabled}
                  onMinDistanceChange={gridSim.setMinDistance}
                />
              ) : (
                <InvertedPendulumControlPanel
                  state={pendulumSim.currentState}
                  isReady={pendulumSim.isReady}
                  isRunning={pendulumSim.isRunning}
                  isInitialized={pendulumSim.isInitialized}
                  error={pendulumSim.error}
                  simulationFailed={pendulumSim.simulationFailed}
                  controllerEnabled={pendulumSim.controllerEnabled}
                  playbackSpeed={pendulumSim.playbackSpeed}
                  lqrGains={pendulumSim.getLQRGains()}
                  numLinks={pendulumSim.numLinks}
                  onInitialize={pendulumSim.initialize}
                  onStart={pendulumSim.start}
                  onPause={pendulumSim.pause}
                  onReset={pendulumSim.reset}
                  onSetPlaybackSpeed={pendulumSim.setPlaybackSpeed}
                  onSetControllerEnabled={pendulumSim.setControllerEnabled}
                  onApplyDisturbance={pendulumSim.applyDisturbance}
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
            ) : simulationType === 'grid2d' ? (
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
            ) : (
              <div style={styles.telemetryPlaceholder}>
                <h3 style={styles.telemetryTitle}>Pendulum Info</h3>
                {pendulumSim.currentState && (
                  <div style={styles.gridInfo}>
                    <div style={styles.infoRow}>
                      <span>Time:</span>
                      <span>{pendulumSim.currentState.time.toFixed(3)}s</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Cart Position:</span>
                      <span>{pendulumSim.currentState.x.toFixed(3)}m</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Links:</span>
                      <span>{pendulumSim.currentState.numLinks}</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>max |θ|:</span>
                      <span>{(pendulumSim.currentState.angles.reduce((m, a) => Math.max(m, Math.abs(a)), 0) * 180 / Math.PI).toFixed(1)}°</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Control Force:</span>
                      <span>{pendulumSim.currentState.controlForce.toFixed(1)}N</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Controller:</span>
                      <span style={{ color: pendulumSim.controllerEnabled ? '#4ade80' : '#f87171' }}>
                        {pendulumSim.controllerEnabled ? 'ON' : 'OFF'}
                      </span>
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
                cameraTracking={cameraTracking}
                onInitialize={rocketSim.initialize}
                onStart={rocketSim.start}
                onPause={rocketSim.pause}
                onReset={rocketSim.reset}
                onStep={rocketSim.step}
                onPlaybackSpeedChange={rocketSim.setPlaybackSpeed}
                onCameraTrackingChange={setCameraTracking}
              />
            ) : simulationType === 'grid2d' ? (
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
                gridSize={gridSim.gridSize}
                onGridSizeChange={gridSim.setGridSize}
                mass={gridSim.mass}
                stiffness={gridSim.stiffness}
                damping={gridSim.damping}
                gridType={gridSim.gridType}
                integrator={gridSim.integrator}
                repulsionEnabled={gridSim.repulsionEnabled}
                minDistance={gridSim.minDistance}
                onMassChange={gridSim.setMass}
                onStiffnessChange={gridSim.setStiffness}
                onDampingChange={gridSim.setDamping}
                onGridTypeChange={gridSim.setGridType}
                onIntegratorChange={gridSim.setIntegrator}
                onRepulsionEnabledChange={gridSim.setRepulsionEnabled}
                onMinDistanceChange={gridSim.setMinDistance}
              />
            ) : (
              <InvertedPendulumControlPanel
                state={pendulumSim.currentState}
                isReady={pendulumSim.isReady}
                isRunning={pendulumSim.isRunning}
                isInitialized={pendulumSim.isInitialized}
                error={pendulumSim.error}
                simulationFailed={pendulumSim.simulationFailed}
                controllerEnabled={pendulumSim.controllerEnabled}
                playbackSpeed={pendulumSim.playbackSpeed}
                lqrGains={pendulumSim.getLQRGains()}
                numLinks={pendulumSim.numLinks}
                onInitialize={pendulumSim.initialize}
                onStart={pendulumSim.start}
                onPause={pendulumSim.pause}
                onReset={pendulumSim.reset}
                onSetPlaybackSpeed={pendulumSim.setPlaybackSpeed}
                onSetControllerEnabled={pendulumSim.setControllerEnabled}
                onApplyDisturbance={pendulumSim.applyDisturbance}
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

          {simulationType === 'pendulum' && (
            <BottomSheet
              isOpen={mobilePanel === 'telemetry'}
              onClose={() => setMobilePanel(null)}
              title="Pendulum Telemetry"
              initialSnapPoint="half"
            >
              <div style={styles.telemetryPlaceholder}>
                <h3 style={styles.telemetryTitle}>Pendulum State</h3>
                {pendulumSim.currentState && (
                  <div style={styles.gridInfo}>
                    <div style={styles.infoRow}>
                      <span>Time:</span>
                      <span>{pendulumSim.currentState.time.toFixed(3)}s</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Cart Position:</span>
                      <span>{pendulumSim.currentState.x.toFixed(3)}m</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Cart Velocity:</span>
                      <span>{pendulumSim.currentState.xdot.toFixed(3)}m/s</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Links:</span>
                      <span>{pendulumSim.currentState.numLinks}</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>max |θ|:</span>
                      <span>{(pendulumSim.currentState.angles.reduce((m, a) => Math.max(m, Math.abs(a)), 0) * 180 / Math.PI).toFixed(2)}°</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>max |ω|:</span>
                      <span>{pendulumSim.currentState.omegas.reduce((m, w) => Math.max(m, Math.abs(w)), 0).toFixed(2)} rad/s</span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Control Force:</span>
                      <span style={{ color: pendulumSim.currentState.controlForce > 0 ? '#4ade80' : '#f87171' }}>
                        {pendulumSim.currentState.controlForce.toFixed(1)}N
                      </span>
                    </div>
                    <div style={styles.infoRow}>
                      <span>Controller:</span>
                      <span style={{ color: pendulumSim.controllerEnabled ? '#4ade80' : '#f87171' }}>
                        {pendulumSim.controllerEnabled ? 'LQR ACTIVE' : 'DISABLED'}
                      </span>
                    </div>
                  </div>
                )}
              </div>
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
    padding: '24px',
    maxWidth: '520px',
  },
  placeholderTitle: {
    fontSize: 'clamp(26px, 6vw, 48px)',
    marginBottom: '20px',
    fontWeight: 'bold' as const,
  },
  placeholderText: {
    fontSize: 'clamp(15px, 4vw, 20px)',
    marginBottom: '10px',
    opacity: 0.9,
  },
  placeholderError: {
    fontSize: '14px',
    margin: '16px 0',
    padding: '12px',
    borderRadius: '8px',
    backgroundColor: 'rgba(0, 0, 0, 0.35)',
    color: '#ffb4b4',
    wordBreak: 'break-word' as const,
  },
  placeholderHint: {
    fontSize: '13px',
    marginTop: '12px',
    opacity: 0.7,
  },
  quickStartButton: {
    marginTop: '24px',
    width: '100%',
    maxWidth: '320px',
    fontSize: '18px',
  },
  visualizationWrapper: {
    position: 'relative' as const,
    width: '100%',
    height: '100%',
  },
  vizOverlayCenter: {
    position: 'absolute' as const,
    inset: 0,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: 'rgba(10, 14, 20, 0.55)',
    padding: '24px',
  },
  failedCard: {
    backgroundColor: 'var(--bg-secondary)',
    border: '1px solid var(--accent-red)',
    borderRadius: '12px',
    padding: '24px',
    maxWidth: '320px',
    width: '100%',
    textAlign: 'center' as const,
    boxShadow: '0 8px 32px rgba(0, 0, 0, 0.5)',
  },
  failedTitle: {
    margin: '0 0 8px 0',
    fontSize: '18px',
    fontWeight: 'bold' as const,
    color: 'var(--accent-red)',
  },
  failedText: {
    margin: '0 0 16px 0',
    fontSize: '14px',
    color: 'var(--text-secondary)',
  },
  resumeChip: {
    position: 'absolute' as const,
    bottom: 'calc(24px + env(safe-area-inset-bottom, 0px))',
    left: '50%',
    transform: 'translateX(-50%)',
    padding: '12px 28px',
    fontSize: '16px',
    whiteSpace: 'nowrap' as const,
    boxShadow: '0 4px 16px rgba(0, 0, 0, 0.5)',
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
