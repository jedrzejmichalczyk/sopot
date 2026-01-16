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
      label: 'Mission Time',
      shortLabel: 'MET',
      value: state.time.toFixed(2),
      unit: 's',
    },
    {
      label: 'Altitude ASL',
      shortLabel: 'ALT',
      value: state.altitude.toFixed(1),
      unit: 'm',
    },
    {
      label: 'Ground Speed',
      shortLabel: 'SPD',
      value: state.speed.toFixed(1),
      unit: 'm/s',
    },
    {
      label: 'Total Mass',
      shortLabel: 'MASS',
      value: state.mass.toFixed(2),
      unit: 'kg',
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
      {/* Header */}
      <div style={styles.header}>
        <div className="technical-label" style={styles.headerLabel}>TELEMETRY</div>
        <div
          className={`status-indicator ${isRunning ? 'active' : 'inactive'}`}
          style={styles.statusDot}
        />
        <span style={styles.statusText}>
          {isRunning ? 'ACTIVE' : 'STANDBY'}
        </span>
      </div>

      <div className="section-divider" style={{ margin: '16px 0' }}></div>

      {/* Main telemetry */}
      <div style={styles.mainTelemetry} className="grid-2-col">
        {telemetryItems.map((item) => (
          <div key={item.label} style={styles.telemetryCard} className="mission-panel corner-accent fade-in">
            <div className="technical-label" style={styles.telemetryLabel}>{item.shortLabel}</div>
            <div className="telemetry-value" style={styles.telemetryValue}>
              {item.value}
            </div>
            <div style={styles.telemetryUnit}>{item.unit}</div>
          </div>
        ))}
      </div>

      {/* Position vector */}
      <div style={styles.section} className="mission-panel">
        <h3 style={styles.sectionTitle}>
          <span className="technical-label" style={styles.sectionLabel}>POSITION</span>
          <span style={styles.frameLabel}>ENU FRAME</span>
        </h3>
        <div style={styles.vectorGrid}>
          {positionItems.map((item) => (
            <div key={item.label} style={styles.vectorItem}>
              <span style={styles.vectorLabel}>{item.label}</span>
              <span className="data-readout" style={styles.vectorValue}>
                {item.value} <span style={styles.unitLabel}>{item.unit}</span>
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* Velocity vector */}
      <div style={styles.section} className="mission-panel">
        <h3 style={styles.sectionTitle}>
          <span className="technical-label" style={styles.sectionLabel}>VELOCITY</span>
          <span style={styles.frameLabel}>ENU FRAME</span>
        </h3>
        <div style={styles.vectorGrid}>
          {velocityItems.map((item) => (
            <div key={item.label} style={styles.vectorItem}>
              <span style={styles.vectorLabel}>{item.label}</span>
              <span className="data-readout" style={styles.vectorValue}>
                {item.value} <span style={styles.unitLabel}>{item.unit}</span>
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* Quaternion */}
      <div style={styles.section} className="mission-panel">
        <h3 style={styles.sectionTitle}>
          <span className="technical-label" style={styles.sectionLabel}>ATTITUDE</span>
          <span style={styles.frameLabel}>QUATERNION</span>
        </h3>
        <div style={styles.quaternionGrid}>
          <div style={styles.quaternionItem}>
            <span style={styles.quaternionLabel}>q₁</span>
            <span className="data-readout" style={styles.quaternionValue}>
              {state.quaternion.q1.toFixed(4)}
            </span>
          </div>
          <div style={styles.quaternionItem}>
            <span style={styles.quaternionLabel}>q₂</span>
            <span className="data-readout" style={styles.quaternionValue}>
              {state.quaternion.q2.toFixed(4)}
            </span>
          </div>
          <div style={styles.quaternionItem}>
            <span style={styles.quaternionLabel}>q₃</span>
            <span className="data-readout" style={styles.quaternionValue}>
              {state.quaternion.q3.toFixed(4)}
            </span>
          </div>
          <div style={styles.quaternionItem}>
            <span style={styles.quaternionLabel}>q₄</span>
            <span className="data-readout" style={styles.quaternionValue}>
              {state.quaternion.q4.toFixed(4)}
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
    background: 'linear-gradient(180deg, var(--bg-secondary) 0%, var(--bg-primary) 100%)',
    color: 'var(--text-primary)',
    height: '100%',
    overflowY: 'auto' as const,
    fontFamily: 'var(--font-mono)',
  },
  noData: {
    textAlign: 'center' as const,
    color: 'var(--text-secondary)',
    padding: '40px 20px',
    fontFamily: 'var(--font-mono)',
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
    marginBottom: '16px',
    padding: '12px 16px',
    background: 'var(--bg-tertiary)',
    borderRadius: '6px',
    border: '1px solid var(--border-color)',
  },
  headerLabel: {
    flex: 1,
    fontSize: '11px',
    background: 'rgba(0, 212, 255, 0.1)',
    border: '1px solid rgba(0, 212, 255, 0.3)',
    padding: '4px 10px',
    borderRadius: '4px',
  },
  statusDot: {
    marginRight: '0',
  },
  statusText: {
    fontSize: '12px',
    fontWeight: 700,
    letterSpacing: '1.5px',
    color: 'var(--text-primary)',
    fontFamily: 'var(--font-mono)',
  },
  mainTelemetry: {
    marginBottom: '20px',
  },
  telemetryCard: {
    padding: '16px',
    display: 'flex',
    flexDirection: 'column' as const,
    gap: '8px',
  },
  telemetryLabel: {
    fontSize: '10px',
    background: 'rgba(0, 212, 255, 0.1)',
    border: '1px solid rgba(0, 212, 255, 0.3)',
    padding: '3px 8px',
    borderRadius: '3px',
    display: 'inline-block',
    width: 'fit-content',
  },
  telemetryValue: {
    fontSize: '32px',
    fontWeight: 600,
  },
  telemetryUnit: {
    fontSize: '12px',
    color: 'var(--text-secondary)',
    marginTop: '4px',
    fontFamily: 'var(--font-mono)',
  },
  section: {
    marginBottom: '16px',
    padding: '16px',
  },
  sectionTitle: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: '12px',
  },
  sectionLabel: {
    fontSize: '10px',
    background: 'rgba(0, 212, 255, 0.1)',
    border: '1px solid rgba(0, 212, 255, 0.3)',
    padding: '3px 8px',
    borderRadius: '3px',
  },
  frameLabel: {
    fontSize: '9px',
    color: 'var(--text-secondary)',
    letterSpacing: '1px',
    fontWeight: 600,
  },
  vectorGrid: {
    display: 'grid',
    gridTemplateColumns: '1fr',
    gap: '10px',
  },
  vectorItem: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '8px 12px',
    background: 'var(--bg-secondary)',
    borderRadius: '4px',
    border: '1px solid var(--border-color)',
  },
  vectorLabel: {
    fontSize: '11px',
    color: 'var(--text-secondary)',
    fontWeight: 600,
    textTransform: 'uppercase' as const,
    letterSpacing: '0.5px',
  },
  vectorValue: {
    fontSize: '15px',
    fontWeight: 600,
  },
  unitLabel: {
    fontSize: '11px',
    color: 'var(--text-secondary)',
    marginLeft: '6px',
    opacity: 0.7,
  },
  quaternionGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(2, 1fr)',
    gap: '10px',
  },
  quaternionItem: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '8px 12px',
    background: 'var(--bg-secondary)',
    borderRadius: '4px',
    border: '1px solid var(--border-color)',
  },
  quaternionLabel: {
    fontSize: '12px',
    color: 'var(--text-secondary)',
    fontWeight: 600,
  },
  quaternionValue: {
    fontSize: '14px',
    fontWeight: 600,
  },
};
