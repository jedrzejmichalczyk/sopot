import type { SimulationState } from '../types/sopot';

interface TelemetryPanelProps {
  state: SimulationState | null;
  isRunning: boolean;
}

export function TelemetryPanel({ state, isRunning }: TelemetryPanelProps) {
  if (!state) {
    return (
      <div style={styles.container}>
        <div style={styles.noData}>No simulation data</div>
      </div>
    );
  }

  const telemetryItems = [
    {
      label: 'Time',
      value: state.time.toFixed(2),
      unit: 's',
      color: '#3498db',
    },
    {
      label: 'Altitude',
      value: state.altitude.toFixed(1),
      unit: 'm',
      color: '#e74c3c',
    },
    {
      label: 'Speed',
      value: state.speed.toFixed(1),
      unit: 'm/s',
      color: '#2ecc71',
    },
    {
      label: 'Mass',
      value: state.mass.toFixed(2),
      unit: 'kg',
      color: '#f39c12',
    },
  ];

  const positionItems = [
    { label: 'East', value: state.position.x.toFixed(1), unit: 'm' },
    { label: 'North', value: state.position.y.toFixed(1), unit: 'm' },
    { label: 'Up', value: state.position.z.toFixed(1), unit: 'm' },
  ];

  const velocityItems = [
    { label: 'Vx', value: state.velocity.x.toFixed(2), unit: 'm/s' },
    { label: 'Vy', value: state.velocity.y.toFixed(2), unit: 'm/s' },
    { label: 'Vz', value: state.velocity.z.toFixed(2), unit: 'm/s' },
  ];

  return (
    <div style={styles.container} className="scrollable">
      {/* Status indicator */}
      <div style={styles.statusBar} className="card">
        <div
          style={{
            ...styles.statusIndicator,
            backgroundColor: isRunning ? '#2ecc71' : '#95a5a6',
          }}
        />
        <span style={styles.statusText}>
          {isRunning ? 'RUNNING' : 'PAUSED'}
        </span>
      </div>

      {/* Main telemetry */}
      <div style={styles.mainTelemetry} className="grid-2-col">
        {telemetryItems.map((item) => (
          <div key={item.label} style={styles.telemetryCard} className="card fade-in">
            <div style={styles.telemetryLabel} className="section-title">{item.label}</div>
            <div style={{ ...styles.telemetryValue, color: item.color }}>
              {item.value}
              <span style={styles.telemetryUnit}>{item.unit}</span>
            </div>
          </div>
        ))}
      </div>

      {/* Position vector */}
      <div style={styles.section} className="card">
        <h3 style={styles.sectionTitle} className="section-title">Position (ENU)</h3>
        <div style={styles.vectorGrid}>
          {positionItems.map((item) => (
            <div key={item.label} style={styles.vectorItem}>
              <span style={styles.vectorLabel}>{item.label}:</span>
              <span style={styles.vectorValue}>
                {item.value} {item.unit}
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* Velocity vector */}
      <div style={styles.section} className="card">
        <h3 style={styles.sectionTitle} className="section-title">Velocity (ENU)</h3>
        <div style={styles.vectorGrid}>
          {velocityItems.map((item) => (
            <div key={item.label} style={styles.vectorItem}>
              <span style={styles.vectorLabel}>{item.label}:</span>
              <span style={styles.vectorValue}>
                {item.value} {item.unit}
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* Quaternion */}
      <div style={styles.section} className="card">
        <h3 style={styles.sectionTitle} className="section-title">Attitude</h3>
        <div style={styles.quaternionGrid}>
          <div style={styles.quaternionItem}>
            <span style={styles.quaternionLabel}>q1:</span>
            <span style={styles.quaternionValue}>
              {state.quaternion.q1.toFixed(3)}
            </span>
          </div>
          <div style={styles.quaternionItem}>
            <span style={styles.quaternionLabel}>q2:</span>
            <span style={styles.quaternionValue}>
              {state.quaternion.q2.toFixed(3)}
            </span>
          </div>
          <div style={styles.quaternionItem}>
            <span style={styles.quaternionLabel}>q3:</span>
            <span style={styles.quaternionValue}>
              {state.quaternion.q3.toFixed(3)}
            </span>
          </div>
          <div style={styles.quaternionItem}>
            <span style={styles.quaternionLabel}>q4:</span>
            <span style={styles.quaternionValue}>
              {state.quaternion.q4.toFixed(3)}
            </span>
          </div>
        </div>
      </div>
    </div>
  );
}

const styles = {
  container: {
    padding: '20px',
    backgroundColor: '#1e1e1e',
    color: '#fff',
    height: '100%',
    overflowY: 'auto' as const,
    fontFamily: 'monospace',
  },
  noData: {
    textAlign: 'center' as const,
    color: '#95a5a6',
    padding: '40px 20px',
  },
  statusBar: {
    display: 'flex',
    alignItems: 'center',
    marginBottom: '20px',
    padding: '10px',
    backgroundColor: '#2c2c2c',
    borderRadius: '5px',
  },
  statusIndicator: {
    width: '12px',
    height: '12px',
    borderRadius: '50%',
    marginRight: '10px',
  },
  statusText: {
    fontSize: '14px',
    fontWeight: 'bold' as const,
    letterSpacing: '1px',
  },
  mainTelemetry: {
    display: 'grid',
    gridTemplateColumns: 'repeat(2, 1fr)',
    gap: '15px',
    marginBottom: '20px',
  },
  telemetryCard: {
    backgroundColor: '#2c2c2c',
    padding: '15px',
    borderRadius: '8px',
    border: '1px solid #3c3c3c',
  },
  telemetryLabel: {
    fontSize: '12px',
    color: '#95a5a6',
    marginBottom: '5px',
    textTransform: 'uppercase' as const,
    letterSpacing: '0.5px',
  },
  telemetryValue: {
    fontSize: '28px',
    fontWeight: 'bold' as const,
    fontFamily: 'monospace',
  },
  telemetryUnit: {
    fontSize: '14px',
    color: '#95a5a6',
    marginLeft: '5px',
    fontWeight: 'normal' as const,
  },
  section: {
    marginBottom: '20px',
    padding: '15px',
    backgroundColor: '#2c2c2c',
    borderRadius: '8px',
    border: '1px solid #3c3c3c',
  },
  sectionTitle: {
    fontSize: '14px',
    color: '#95a5a6',
    marginBottom: '10px',
    textTransform: 'uppercase' as const,
    letterSpacing: '0.5px',
  },
  vectorGrid: {
    display: 'grid',
    gridTemplateColumns: '1fr',
    gap: '8px',
  },
  vectorItem: {
    display: 'flex',
    justifyContent: 'space-between',
    fontSize: '14px',
  },
  vectorLabel: {
    color: '#95a5a6',
  },
  vectorValue: {
    color: '#fff',
    fontWeight: 'bold' as const,
  },
  quaternionGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(2, 1fr)',
    gap: '10px',
  },
  quaternionItem: {
    display: 'flex',
    justifyContent: 'space-between',
    fontSize: '14px',
  },
  quaternionLabel: {
    color: '#95a5a6',
  },
  quaternionValue: {
    color: '#fff',
    fontWeight: 'bold' as const,
  },
};
