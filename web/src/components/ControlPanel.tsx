import { useState } from 'react';
import type { SimulationConfig } from '../hooks/useRocketSimulation';

interface ControlPanelProps {
  isReady: boolean;
  isInitialized: boolean;
  isRunning: boolean;
  error: string | null;
  playbackSpeed: number;
  cameraTracking?: boolean;
  onInitialize: (config: SimulationConfig) => Promise<void>;
  onStart: () => void;
  onPause: () => void;
  onReset: () => void;
  onStep: () => void;
  onPlaybackSpeedChange: (speed: number) => void;
  onCameraTrackingChange?: (enabled: boolean) => void;
}

export function ControlPanel({
  isReady,
  isInitialized,
  isRunning,
  error,
  playbackSpeed,
  cameraTracking = false,
  onInitialize,
  onStart,
  onPause,
  onReset,
  onStep,
  onPlaybackSpeedChange,
  onCameraTrackingChange,
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
      <div style={styles.titleContainer}>
        <div className="technical-label" style={styles.systemLabel}>SYS-SOPOT</div>
        <h2 style={styles.title}>MISSION CONTROL</h2>
        <div style={styles.subtitle}>6-DOF Trajectory Simulation</div>
      </div>

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
                  className="touch-button btn-success"
                  style={styles.button}
                >
                  START
                </button>
              ) : (
                <button
                  onClick={onPause}
                  className="touch-button btn-warning"
                  style={styles.button}
                >
                  PAUSE
                </button>
              )}

              <button
                onClick={onStep}
                disabled={isRunning}
                className="touch-button btn-primary"
                style={{
                  ...styles.button,
                  ...(isRunning ? styles.buttonDisabled : {}),
                }}
              >
                STEP
              </button>

              <button
                onClick={onReset}
                className="touch-button btn-danger"
                style={styles.button}
              >
                RESET
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

            {onCameraTrackingChange && (
              <div style={styles.inputGroup}>
                <label style={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={cameraTracking}
                    onChange={(e) => onCameraTrackingChange(e.target.checked)}
                    className="touch-checkbox"
                  />
                  <span style={styles.checkboxText}>Track Rocket</span>
                </label>
                <div style={styles.helpText}>
                  Camera follows rocket position (keeps rocket in view)
                </div>
              </div>
            )}

            <div style={styles.infoBox}>
              <div style={styles.controlItem}>
                <span className="technical-label" style={styles.controlLabel}>LMB+DRAG</span>
                <span>Rotate view</span>
              </div>
              <div style={styles.controlItem}>
                <span className="technical-label" style={styles.controlLabel}>RMB+DRAG</span>
                <span>Pan view</span>
              </div>
              <div style={styles.controlItem}>
                <span className="technical-label" style={styles.controlLabel}>SCROLL</span>
                <span>Zoom</span>
              </div>
            </div>
          </div>
        </>
      )}

      {/* System Info */}
      <div style={styles.footer}>
        <div className="section-divider"></div>
        <div className="technical-label" style={styles.footerLabel}>SYSTEM STATUS</div>
        <div style={styles.footerText}>SOPOT-WASM v0.1.0</div>
        <div style={styles.footerText}>C++20 Physics Engine</div>
      </div>
    </div>
  );
}

const styles = {
  container: {
    padding: '20px',
    background: 'linear-gradient(180deg, var(--bg-secondary) 0%, var(--bg-primary) 100%)',
    color: 'var(--text-primary)',
    height: '100%',
    overflowY: 'auto' as const,
    display: 'flex',
    flexDirection: 'column' as const,
  },
  titleContainer: {
    marginBottom: '24px',
    textAlign: 'center' as const,
    borderBottom: '2px solid var(--border-color)',
    paddingBottom: '16px',
  },
  systemLabel: {
    marginBottom: '8px',
    color: 'var(--accent-cyan)',
    background: 'rgba(0, 212, 255, 0.1)',
    border: '1px solid rgba(0, 212, 255, 0.3)',
    padding: '4px 12px',
    borderRadius: '4px',
    display: 'inline-block',
  },
  title: {
    margin: '8px 0',
    fontSize: '22px',
    fontWeight: 700,
    color: 'var(--text-primary)',
    letterSpacing: '2px',
  },
  subtitle: {
    fontSize: '11px',
    color: 'var(--text-secondary)',
    textTransform: 'uppercase' as const,
    letterSpacing: '1.5px',
    marginTop: '4px',
  },
  section: {
    marginBottom: '16px',
    padding: '16px',
    background: 'var(--bg-tertiary)',
    borderRadius: '8px',
    border: '1px solid var(--border-color)',
    boxShadow: 'inset 0 1px 0 rgba(0, 212, 255, 0.05)',
  },
  sectionTitle: {
    margin: '0 0 12px 0',
    fontSize: '13px',
    fontWeight: 700,
    color: 'var(--accent-cyan)',
    textTransform: 'uppercase' as const,
    letterSpacing: '1.5px',
  },
  inputGroup: {
    marginBottom: '12px',
  },
  label: {
    display: 'block',
    fontSize: '12px',
    marginBottom: '6px',
    color: 'var(--text-secondary)',
    textTransform: 'uppercase' as const,
    letterSpacing: '0.5px',
    fontWeight: 600,
  },
  input: {
    width: '100%',
    padding: '10px',
    marginTop: '6px',
    backgroundColor: 'var(--bg-secondary)',
    border: '1px solid var(--border-color)',
    borderRadius: '4px',
    color: 'var(--text-primary)',
    fontSize: '14px',
    fontFamily: 'var(--font-mono)',
  },
  buttonGroup: {
    display: 'grid',
    gridTemplateColumns: 'repeat(3, 1fr)',
    gap: '10px',
  },
  button: {
    padding: '12px 16px',
    fontSize: '12px',
    fontWeight: 700,
    letterSpacing: '1px',
    cursor: 'pointer',
    transition: 'all 0.2s',
  },
  buttonDisabled: {
    opacity: 0.3,
    cursor: 'not-allowed',
  },
  buttonPrimary: {
    background: 'linear-gradient(135deg, #0066cc 0%, #004499 100%)',
    border: '1px solid rgba(0, 212, 255, 0.3)',
    boxShadow: '0 0 12px rgba(0, 102, 204, 0.3)',
  },
  sliderContainer: {
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
  },
  slider: {
    flex: 1,
  },
  sliderValue: {
    minWidth: '50px',
    textAlign: 'center' as const,
    fontWeight: 600,
    fontSize: '16px',
    fontFamily: 'var(--font-mono)',
    color: 'var(--accent-cyan)',
  },
  errorBox: {
    padding: '12px 16px',
    marginBottom: '16px',
    background: 'linear-gradient(135deg, rgba(255, 59, 59, 0.2) 0%, rgba(153, 34, 34, 0.2) 100%)',
    borderRadius: '6px',
    border: '1px solid var(--accent-red)',
    fontSize: '13px',
    color: 'var(--text-primary)',
    fontFamily: 'var(--font-mono)',
  },
  infoBox: {
    padding: '12px',
    backgroundColor: 'var(--bg-secondary)',
    borderRadius: '6px',
    border: '1px solid var(--border-color)',
    fontSize: '12px',
    color: 'var(--text-secondary)',
  },
  controlItem: {
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
    marginBottom: '8px',
  },
  controlLabel: {
    fontSize: '9px',
    padding: '2px 6px',
    background: 'rgba(0, 212, 255, 0.1)',
    border: '1px solid rgba(0, 212, 255, 0.3)',
    borderRadius: '3px',
    minWidth: '80px',
    textAlign: 'center' as const,
  },
  checkboxLabel: {
    display: 'flex',
    alignItems: 'center',
    gap: '10px',
    cursor: 'pointer',
    padding: '8px 0',
  },
  checkboxText: {
    fontSize: '13px',
    color: 'var(--text-primary)',
    fontWeight: 500,
  },
  helpText: {
    fontSize: '11px',
    color: 'var(--text-secondary)',
    marginLeft: '28px',
    marginTop: '-4px',
  },
  footer: {
    marginTop: 'auto',
    paddingTop: '16px',
    textAlign: 'center' as const,
  },
  footerLabel: {
    fontSize: '9px',
    marginBottom: '8px',
    color: 'var(--text-secondary)',
  },
  footerText: {
    fontSize: '11px',
    color: 'var(--text-secondary)',
    margin: '4px 0',
    fontFamily: 'var(--font-mono)',
  },
};
