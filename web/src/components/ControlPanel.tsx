import { useState } from 'react';
import type { SimulationConfig } from '../hooks/useRocketSimulation';

interface ControlPanelProps {
  isReady: boolean;
  isInitialized: boolean;
  isRunning: boolean;
  error: string | null;
  playbackSpeed: number;
  onInitialize: (config: SimulationConfig) => Promise<void>;
  onStart: () => void;
  onPause: () => void;
  onReset: () => void;
  onStep: () => void;
  onPlaybackSpeedChange: (speed: number) => void;
}

export function ControlPanel({
  isReady,
  isInitialized,
  isRunning,
  error,
  playbackSpeed,
  onInitialize,
  onStart,
  onPause,
  onReset,
  onStep,
  onPlaybackSpeedChange,
}: ControlPanelProps) {
  const [config, setConfig] = useState<SimulationConfig>({
    elevation: 85,
    azimuth: 0,
    diameter: 0.16,
    timestep: 0.01,
  });

  const [isInitializing, setIsInitializing] = useState(false);

  const handleInitialize = async () => {
    setIsInitializing(true);
    try {
      await onInitialize(config);
    } catch (err) {
      console.error('Initialization error:', err);
    } finally {
      setIsInitializing(false);
    }
  };

  return (
    <div style={styles.container}>
      <h2 style={styles.title}>🚀 SOPOT Rocket Simulation</h2>

      {/* Error display */}
      {error && (
        <div style={styles.errorBox}>
          <strong>Error:</strong> {error}
        </div>
      )}

      {/* Configuration Section */}
      {!isInitialized && (
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>Configuration</h3>

          <div style={styles.inputGroup}>
            <label style={styles.label}>
              Launch Elevation (°)
              <input
                type="number"
                value={config.elevation}
                onChange={(e) =>
                  setConfig({ ...config, elevation: parseFloat(e.target.value) })
                }
                min={0}
                max={90}
                step={1}
                className="touch-input"
                style={styles.input}
                disabled={!isReady || isInitializing}
              />
            </label>
          </div>

          <div style={styles.inputGroup}>
            <label style={styles.label}>
              Launch Azimuth (°)
              <input
                type="number"
                value={config.azimuth}
                onChange={(e) =>
                  setConfig({ ...config, azimuth: parseFloat(e.target.value) })
                }
                min={0}
                max={360}
                step={1}
                className="touch-input"
                style={styles.input}
                disabled={!isReady || isInitializing}
              />
            </label>
          </div>

          <div style={styles.inputGroup}>
            <label style={styles.label}>
              Rocket Diameter (m)
              <input
                type="number"
                value={config.diameter}
                onChange={(e) =>
                  setConfig({ ...config, diameter: parseFloat(e.target.value) })
                }
                min={0.01}
                max={1}
                step={0.01}
                className="touch-input"
                style={styles.input}
                disabled={!isReady || isInitializing}
              />
            </label>
          </div>

          <div style={styles.inputGroup}>
            <label style={styles.label}>
              Timestep (s)
              <input
                type="number"
                value={config.timestep}
                onChange={(e) =>
                  setConfig({ ...config, timestep: parseFloat(e.target.value) })
                }
                min={0.001}
                max={0.1}
                step={0.001}
                className="touch-input"
                style={styles.input}
                disabled={!isReady || isInitializing}
              />
            </label>
          </div>

          <button
            onClick={handleInitialize}
            disabled={!isReady || isInitializing}
            className="touch-button"
            style={{
              ...styles.button,
              ...styles.buttonPrimary,
              width: '100%',
            }}
          >
            {isInitializing ? 'Initializing...' : 'Initialize Simulation'}
          </button>

          {!isReady && (
            <div style={styles.infoBox}>
              Loading WebAssembly module...
            </div>
          )}
        </div>
      )}

      {/* Control Section */}
      {isInitialized && (
        <>
          <div style={styles.section}>
            <h3 style={styles.sectionTitle}>Controls</h3>

            <div style={styles.buttonGroup}>
              {!isRunning ? (
                <button
                  onClick={onStart}
                  className="touch-button"
                  style={{ ...styles.button, ...styles.buttonSuccess }}
                >
                  ▶ Start
                </button>
              ) : (
                <button
                  onClick={onPause}
                  className="touch-button"
                  style={{ ...styles.button, ...styles.buttonWarning }}
                >
                  ⏸ Pause
                </button>
              )}

              <button
                onClick={onStep}
                disabled={isRunning}
                className="touch-button"
                style={{
                  ...styles.button,
                  ...styles.buttonInfo,
                  ...(isRunning ? styles.buttonDisabled : {}),
                }}
              >
                ⏭ Step
              </button>

              <button
                onClick={onReset}
                className="touch-button"
                style={{ ...styles.button, ...styles.buttonDanger }}
              >
                ⟲ Reset
              </button>
            </div>
          </div>

          <div style={styles.section}>
            <h3 style={styles.sectionTitle}>Playback Speed</h3>

            <div style={styles.sliderContainer}>
              <input
                type="range"
                min={0.1}
                max={10}
                step={0.1}
                value={playbackSpeed}
                onChange={(e) =>
                  onPlaybackSpeedChange(parseFloat(e.target.value))
                }
                className="touch-slider"
                style={styles.slider}
                disabled={!isInitialized}
              />
              <div style={styles.sliderValue}>{playbackSpeed.toFixed(1)}x</div>
            </div>
          </div>

          <div style={styles.section}>
            <h3 style={styles.sectionTitle}>Camera Controls</h3>
            <div style={styles.infoBox}>
              <div>🖱️ Left click + drag: Rotate</div>
              <div>🖱️ Right click + drag: Pan</div>
              <div>🖱️ Scroll: Zoom</div>
            </div>
          </div>
        </>
      )}

      {/* Info Section */}
      <div style={styles.footer}>
        <div style={styles.footerText}>
          SOPOT WebAssembly Demo v0.1
        </div>
        <div style={styles.footerText}>
          C++20 Physics Simulation in Browser
        </div>
      </div>
    </div>
  );
}

const styles = {
  container: {
    padding: '20px',
    backgroundColor: '#2c3e50',
    color: '#ecf0f1',
    height: '100%',
    overflowY: 'auto' as const,
    display: 'flex',
    flexDirection: 'column' as const,
  },
  title: {
    margin: '0 0 20px 0',
    fontSize: '24px',
    fontWeight: 'bold' as const,
    color: '#fff',
    textAlign: 'center' as const,
  },
  section: {
    marginBottom: '20px',
    padding: '15px',
    backgroundColor: '#34495e',
    borderRadius: '8px',
  },
  sectionTitle: {
    margin: '0 0 15px 0',
    fontSize: '16px',
    fontWeight: 'bold' as const,
    color: '#3498db',
    textTransform: 'uppercase' as const,
    letterSpacing: '0.5px',
  },
  inputGroup: {
    marginBottom: '15px',
  },
  label: {
    display: 'block',
    fontSize: '14px',
    marginBottom: '5px',
    color: '#bdc3c7',
  },
  input: {
    width: '100%',
    padding: '10px',
    marginTop: '5px',
    backgroundColor: '#2c3e50',
    border: '1px solid #7f8c8d',
    borderRadius: '4px',
    color: '#ecf0f1',
    fontSize: '14px',
  },
  buttonGroup: {
    display: 'grid',
    gridTemplateColumns: 'repeat(3, 1fr)',
    gap: '10px',
  },
  button: {
    padding: '12px 20px',
    border: 'none',
    borderRadius: '5px',
    fontSize: '14px',
    fontWeight: 'bold' as const,
    cursor: 'pointer',
    transition: 'all 0.2s',
  },
  buttonPrimary: {
    backgroundColor: '#3498db',
    color: '#fff',
  },
  buttonSuccess: {
    backgroundColor: '#2ecc71',
    color: '#fff',
  },
  buttonWarning: {
    backgroundColor: '#f39c12',
    color: '#fff',
  },
  buttonDanger: {
    backgroundColor: '#e74c3c',
    color: '#fff',
  },
  buttonInfo: {
    backgroundColor: '#9b59b6',
    color: '#fff',
  },
  buttonDisabled: {
    opacity: 0.5,
    cursor: 'not-allowed',
  },
  sliderContainer: {
    display: 'flex',
    alignItems: 'center',
    gap: '15px',
  },
  slider: {
    flex: 1,
    height: '8px',
    borderRadius: '4px',
    outline: 'none',
    opacity: 0.8,
    transition: 'opacity 0.2s',
  },
  sliderValue: {
    minWidth: '50px',
    textAlign: 'center' as const,
    fontWeight: 'bold' as const,
    fontSize: '16px',
  },
  errorBox: {
    padding: '15px',
    marginBottom: '20px',
    backgroundColor: '#c0392b',
    borderRadius: '5px',
    border: '1px solid #e74c3c',
    fontSize: '14px',
  },
  infoBox: {
    padding: '10px',
    marginTop: '15px',
    backgroundColor: '#34495e',
    borderRadius: '5px',
    border: '1px solid #7f8c8d',
    fontSize: '13px',
    color: '#bdc3c7',
  },
  footer: {
    marginTop: 'auto',
    paddingTop: '20px',
    borderTop: '1px solid #7f8c8d',
    textAlign: 'center' as const,
  },
  footerText: {
    fontSize: '12px',
    color: '#95a5a6',
    margin: '5px 0',
  },
};
