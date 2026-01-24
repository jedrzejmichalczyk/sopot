import type { GridTopology } from '../types/sopot';
import type { GridSize } from '../hooks/useGrid2DSimulation';
import { SUPPORTED_GRID_SIZES } from '../hooks/useGrid2DSimulation';

interface Grid2DControlPanelProps {
  isReady: boolean;
  isInitialized: boolean;
  isRunning: boolean;
  error: string | null;
  playbackSpeed: number;
  onInitialize: () => void;
  onStart: () => void;
  onPause: () => void;
  onReset: () => void;
  onStep: () => void;
  onPlaybackSpeedChange: (speed: number) => void;
  // Grid-specific controls
  showVelocities?: boolean;
  showGrid?: boolean;
  onShowVelocitiesChange?: (show: boolean) => void;
  onShowGridChange?: (show: boolean) => void;
  // Grid size
  gridSize?: GridSize;
  onGridSizeChange?: (size: GridSize) => void;
  // Physics parameters
  mass?: number;
  stiffness?: number;
  damping?: number;
  gridType?: GridTopology;
  onMassChange?: (mass: number) => void;
  onStiffnessChange?: (stiffness: number) => void;
  onDampingChange?: (damping: number) => void;
  onGridTypeChange?: (gridType: GridTopology) => void;
}

export function Grid2DControlPanel({
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
  showVelocities = false,
  showGrid = true,
  onShowVelocitiesChange,
  onShowGridChange,
  gridSize = 10,
  onGridSizeChange,
  mass = 1.0,
  stiffness = 100.0,
  damping = 1.0,
  gridType = 'quad',
  onMassChange,
  onStiffnessChange,
  onDampingChange,
  onGridTypeChange,
}: Grid2DControlPanelProps) {
  const playbackSpeeds = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0];

  return (
    <div style={styles.container}>
      {/* Header */}
      <div style={styles.header}>
        <h2 style={styles.title}>2D Grid Controls</h2>
        <div style={styles.statusBadge}>
          <div
            style={{
              ...styles.statusDot,
              backgroundColor: isRunning
                ? 'var(--accent-green)'
                : isInitialized
                ? 'var(--accent-amber)'
                : 'var(--text-secondary)',
            }}
          />
          <span style={styles.statusText}>
            {isRunning ? 'Running' : isInitialized ? 'Ready' : 'Stopped'}
          </span>
        </div>
      </div>

      {/* Error Display */}
      {error && (
        <div style={styles.errorBox}>
          <strong>Error:</strong> {error}
        </div>
      )}

      {/* Main Controls */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Simulation</h3>

        {!isInitialized ? (
          <button
            onClick={onInitialize}
            disabled={!isReady}
            className="touch-button"
            style={{
              ...styles.button,
              ...styles.buttonPrimary,
              ...((!isReady) ? styles.buttonDisabled : {}),
            }}
          >
            {isReady ? 'Initialize Grid' : 'Loading...'}
          </button>
        ) : (
          <div style={styles.buttonGroup}>
            <button
              onClick={isRunning ? onPause : onStart}
              className="touch-button btn-success"
              style={{
                ...styles.button,
              }}
            >
              {isRunning ? 'PAUSE' : 'START'}
            </button>
            <button
              onClick={onReset}
              className="touch-button btn-danger"
              style={{
                ...styles.button,
              }}
            >
              RESET
            </button>
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
          </div>
        )}
      </div>

      {/* Playback Speed */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Playback Speed</h3>
        <div style={styles.speedGrid}>
          {playbackSpeeds.map((speed) => (
            <button
              key={speed}
              onClick={() => onPlaybackSpeedChange(speed)}
              disabled={!isInitialized}
              className="touch-button"
              style={{
                ...styles.speedButton,
                ...(playbackSpeed === speed ? styles.speedButtonActive : {}),
                ...((!isInitialized) ? styles.buttonDisabled : {}),
              }}
            >
              {speed}×
            </button>
          ))}
        </div>
      </div>

      {/* Physics Parameters */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Physics Parameters</h3>
        <p style={styles.warningText}>
          Change these before initialization
        </p>

        <div style={styles.parameterControl}>
          <label style={styles.parameterLabel}>
            <span style={styles.parameterName}>Grid Size</span>
            <span style={styles.parameterValue}>
              {gridSize}×{gridSize} ({gridSize * gridSize} masses)
            </span>
          </label>
          <div style={styles.gridSizeGrid}>
            {SUPPORTED_GRID_SIZES.map((size) => (
              <button
                key={size}
                onClick={() => onGridSizeChange?.(size)}
                disabled={isInitialized}
                className="touch-button"
                style={{
                  ...styles.gridSizeButton,
                  ...(gridSize === size ? styles.gridSizeButtonActive : {}),
                  ...(isInitialized ? styles.buttonDisabled : {}),
                }}
              >
                {size}×{size}
              </button>
            ))}
          </div>
          {gridSize >= 50 && (
            <p style={styles.warningText}>
              Large grids ({gridSize * gridSize} masses) may run slower
            </p>
          )}
        </div>

        <div style={styles.parameterControl}>
          <label style={styles.parameterLabel}>
            <span style={styles.parameterName}>Mass (kg)</span>
            <span style={styles.parameterValue}>{mass.toFixed(2)}</span>
          </label>
          <input
            type="range"
            min="0.1"
            max="5.0"
            step="0.1"
            value={mass}
            onChange={(e) => onMassChange?.(parseFloat(e.target.value))}
            disabled={isInitialized}
            style={styles.slider}
          />
        </div>

        <div style={styles.parameterControl}>
          <label style={styles.parameterLabel}>
            <span style={styles.parameterName}>Stiffness (N/m)</span>
            <span style={styles.parameterValue}>{stiffness.toFixed(1)}</span>
          </label>
          <input
            type="range"
            min="10"
            max="200"
            step="5"
            value={stiffness}
            onChange={(e) => onStiffnessChange?.(parseFloat(e.target.value))}
            disabled={isInitialized}
            style={styles.slider}
          />
        </div>

        <div style={styles.parameterControl}>
          <label style={styles.parameterLabel}>
            <span style={styles.parameterName}>Damping (N·s/m)</span>
            <span style={styles.parameterValue}>{damping.toFixed(2)}</span>
          </label>
          <input
            type="range"
            min="0.0"
            max="2.0"
            step="0.05"
            value={damping}
            onChange={(e) => onDampingChange?.(parseFloat(e.target.value))}
            disabled={isInitialized}
            style={styles.slider}
          />
        </div>

        <div style={styles.parameterControl}>
          <label style={styles.parameterLabel}>
            <span style={styles.parameterName}>Grid Type</span>
            <span style={styles.parameterValue}>{gridType}</span>
          </label>
          <div style={styles.gridTypeGrid}>
            <button
              onClick={() => onGridTypeChange?.('quad')}
              disabled={isInitialized}
              className="touch-button"
              style={{
                ...styles.gridTypeButton,
                ...(gridType === 'quad' ? styles.gridTypeButtonActive : {}),
                ...(isInitialized ? styles.buttonDisabled : {}),
              }}
            >
              Quad
            </button>
            <button
              onClick={() => onGridTypeChange?.('triangle')}
              disabled={isInitialized}
              className="touch-button"
              style={{
                ...styles.gridTypeButton,
                ...(gridType === 'triangle' ? styles.gridTypeButtonActive : {}),
                ...(isInitialized ? styles.buttonDisabled : {}),
              }}
            >
              Triangle
            </button>
          </div>
          <p style={styles.infoTextSmall}>
            {gridType === 'quad'
              ? 'Horizontal + vertical springs'
              : 'H + V + diagonal springs (more stable)'}
          </p>
        </div>
      </div>

      {/* Visualization Options */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Visualization</h3>

        <label style={styles.checkboxLabel}>
          <input
            type="checkbox"
            checked={showGrid}
            onChange={(e) => onShowGridChange?.(e.target.checked)}
            style={styles.checkbox}
          />
          <span style={styles.checkboxText}>Show Grid (Springs)</span>
        </label>

        <label style={styles.checkboxLabel}>
          <input
            type="checkbox"
            checked={showVelocities}
            onChange={(e) => onShowVelocitiesChange?.(e.target.checked)}
            style={styles.checkbox}
          />
          <span style={styles.checkboxText}>Show Velocities</span>
        </label>
      </div>

      {/* Info Section */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>About</h3>
        <p style={styles.infoText}>
          2D mass-spring grid simulation using Hooke's law with damping.
          Supports grids from 5×5 (25 masses) up to 100×100 (10,000 masses).
        </p>
        <p style={styles.infoText}>
          Uses the <strong>Unified Graph Architecture</strong> with O(K) template
          instantiations and compile-time validated state function resolution.
        </p>
        <p style={styles.infoText}>
          All physics runs in C++ via WebAssembly with zero runtime overhead.
        </p>
      </div>
    </div>
  );
}

const styles = {
  container: {
    height: '100%',
    backgroundColor: 'var(--bg-secondary)',
    color: 'var(--text-primary)',
    overflowY: 'auto' as const,
    display: 'flex',
    flexDirection: 'column' as const,
  },
  header: {
    padding: '20px',
    borderBottom: '2px solid var(--bg-tertiary)',
  },
  title: {
    margin: '0 0 10px 0',
    fontSize: '20px',
    fontWeight: 'bold' as const,
  },
  statusBadge: {
    display: 'flex',
    alignItems: 'center',
    gap: '8px',
  },
  statusDot: {
    width: '10px',
    height: '10px',
    borderRadius: '50%',
  },
  statusText: {
    fontSize: '14px',
    color: 'var(--text-secondary)',
  },
  errorBox: {
    margin: '10px 20px',
    padding: '10px',
    backgroundColor: '#c0392b',
    border: '1px solid var(--accent-red)',
    borderRadius: '4px',
    fontSize: '14px',
  },
  section: {
    padding: '20px',
    borderBottom: '1px solid var(--bg-tertiary)',
  },
  sectionTitle: {
    margin: '0 0 15px 0',
    fontSize: '14px',
    fontWeight: 600,
    color: 'var(--text-secondary)',
    textTransform: 'uppercase' as const,
    letterSpacing: '1px',
  },
  buttonGroup: {
    display: 'flex',
    flexDirection: 'column' as const,
    gap: '8px',
  },
  button: {
    padding: '12px 20px',
    fontSize: '14px',
    fontWeight: 'bold' as const,
    border: 'none',
    borderRadius: '6px',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
  },
  buttonPrimary: {
    backgroundColor: 'var(--accent-cyan)',
    color: '#fff',
  },
  buttonSecondary: {
    backgroundColor: 'var(--bg-tertiary)',
    color: 'var(--text-primary)',
  },
  buttonDisabled: {
    opacity: 0.5,
    cursor: 'not-allowed',
  },
  speedGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(4, 1fr)',
    gap: '6px',
  },
  speedButton: {
    padding: '8px',
    fontSize: '12px',
    fontWeight: 'bold' as const,
    border: '2px solid var(--bg-tertiary)',
    borderRadius: '4px',
    backgroundColor: 'var(--bg-secondary)',
    color: 'var(--text-primary)',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
  },
  speedButtonActive: {
    borderColor: 'var(--accent-cyan)',
    backgroundColor: 'var(--accent-cyan)',
    color: '#fff',
  },
  checkboxLabel: {
    display: 'flex',
    alignItems: 'center',
    gap: '10px',
    marginBottom: '10px',
    cursor: 'pointer',
  },
  checkbox: {
    width: '18px',
    height: '18px',
    cursor: 'pointer',
  },
  checkboxText: {
    fontSize: '14px',
    color: 'var(--text-primary)',
  },
  infoText: {
    fontSize: '13px',
    color: 'var(--text-secondary)',
    lineHeight: '1.6',
    marginBottom: '10px',
  },
  warningText: {
    fontSize: '12px',
    color: 'var(--accent-amber)',
    marginBottom: '15px',
    fontStyle: 'italic' as const,
  },
  parameterControl: {
    marginBottom: '15px',
  },
  parameterLabel: {
    display: 'flex',
    justifyContent: 'space-between',
    marginBottom: '5px',
  },
  parameterName: {
    fontSize: '13px',
    color: 'var(--text-primary)',
  },
  parameterValue: {
    fontSize: '13px',
    color: 'var(--accent-cyan)',
    fontWeight: 'bold' as const,
  },
  slider: {
    width: '100%',
    cursor: 'pointer',
  },
  gridTypeGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(2, 1fr)',
    gap: '8px',
    marginTop: '8px',
  },
  gridTypeButton: {
    padding: '10px 16px',
    fontSize: '13px',
    fontWeight: 'bold' as const,
    border: '2px solid var(--bg-tertiary)',
    borderRadius: '4px',
    backgroundColor: 'var(--bg-secondary)',
    color: 'var(--text-primary)',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
  },
  gridTypeButtonActive: {
    borderColor: 'var(--accent-cyan)',
    backgroundColor: 'var(--accent-cyan)',
    color: '#fff',
  },
  infoTextSmall: {
    fontSize: '11px',
    color: 'var(--text-secondary)',
    marginTop: '6px',
    fontStyle: 'italic' as const,
  },
  gridSizeGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(5, 1fr)',
    gap: '6px',
    marginTop: '8px',
  },
  gridSizeButton: {
    padding: '10px 8px',
    fontSize: '12px',
    fontWeight: 'bold' as const,
    border: '2px solid var(--bg-tertiary)',
    borderRadius: '4px',
    backgroundColor: 'var(--bg-secondary)',
    color: 'var(--text-primary)',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
  },
  gridSizeButtonActive: {
    borderColor: 'var(--accent-cyan)',
    backgroundColor: 'var(--accent-cyan)',
    color: '#fff',
  },
};
