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
                ? '#27ae60'
                : isInitialized
                ? '#f39c12'
                : '#95a5a6',
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
              style={{
                ...styles.button,
                ...styles.buttonPrimary,
              }}
            >
              {isRunning ? '⏸ Pause' : '▶ Start'}
            </button>
            <button
              onClick={onReset}
              style={{
                ...styles.button,
                ...styles.buttonSecondary,
              }}
            >
              🔄 Reset
            </button>
            <button
              onClick={onStep}
              disabled={isRunning}
              style={{
                ...styles.button,
                ...styles.buttonSecondary,
                ...(isRunning ? styles.buttonDisabled : {}),
              }}
            >
              ⏭ Step
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
          Each mass has 4 states: (x, y, vx, vy).
        </p>
        <p style={styles.infoText}>
          The grid simulates cloth-like behavior with springs connecting
          adjacent masses.
        </p>
      </div>
    </div>
  );
}

const styles = {
  container: {
    height: '100%',
    backgroundColor: '#2c3e50',
    color: '#ecf0f1',
    overflowY: 'auto' as const,
    display: 'flex',
    flexDirection: 'column' as const,
  },
  header: {
    padding: '20px',
    borderBottom: '2px solid #34495e',
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
    color: '#95a5a6',
  },
  errorBox: {
    margin: '10px 20px',
    padding: '10px',
    backgroundColor: '#c0392b',
    border: '1px solid #e74c3c',
    borderRadius: '4px',
    fontSize: '14px',
  },
  section: {
    padding: '20px',
    borderBottom: '1px solid #34495e',
  },
  sectionTitle: {
    margin: '0 0 15px 0',
    fontSize: '14px',
    fontWeight: 600,
    color: '#95a5a6',
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
    backgroundColor: '#3498db',
    color: '#fff',
  },
  buttonSecondary: {
    backgroundColor: '#34495e',
    color: '#ecf0f1',
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
    border: '2px solid #34495e',
    borderRadius: '4px',
    backgroundColor: '#2c3e50',
    color: '#ecf0f1',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
  },
  speedButtonActive: {
    borderColor: '#3498db',
    backgroundColor: '#3498db',
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
    color: '#ecf0f1',
  },
  infoText: {
    fontSize: '13px',
    color: '#95a5a6',
    lineHeight: '1.6',
    marginBottom: '10px',
  },
};
