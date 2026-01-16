import { useState } from 'react';

export type SimulationType = 'rocket' | 'grid2d' | 'pendulum';

interface SimulationSelectorProps {
  currentSimulation: SimulationType;
  onSimulationChange: (type: SimulationType) => void;
  disabled?: boolean;
}

export function SimulationSelector({
  currentSimulation,
  onSimulationChange,
  disabled = false,
}: SimulationSelectorProps) {
  const [hoveredType, setHoveredType] = useState<SimulationType | null>(null);

  const simulations = [
    {
      type: 'rocket' as SimulationType,
      label: 'RKTFLT',
      title: 'Rocket Flight',
      description: '6-DOF trajectory simulation with aerodynamics',
      subsystem: 'PROPULSION',
    },
    {
      type: 'grid2d' as SimulationType,
      label: 'GRID2D',
      title: '2D Mass-Spring Grid',
      description: 'Deformable membrane physics simulation',
      subsystem: 'STRUCTURAL',
    },
    {
      type: 'pendulum' as SimulationType,
      label: 'INVPND',
      title: 'Inverted Double Pendulum',
      description: 'LQR-controlled balancing with real-time stabilization',
      subsystem: 'CONTROL',
    },
  ];

  return (
    <div style={styles.container}>
      <h3 style={styles.header} className="section-title">Simulation Type</h3>
      <div style={styles.cardContainer}>
        {simulations.map((sim) => {
          const isSelected = currentSimulation === sim.type;
          const isHovered = hoveredType === sim.type;

          return (
            <button
              key={sim.type}
              onClick={() => !disabled && onSimulationChange(sim.type)}
              onMouseEnter={() => setHoveredType(sim.type)}
              onMouseLeave={() => setHoveredType(null)}
              disabled={disabled}
              className="touch-button mission-panel corner-accent"
              style={{
                ...styles.card,
                ...(isSelected ? styles.cardSelected : {}),
                ...(isHovered && !disabled ? styles.cardHovered : {}),
                ...(disabled ? styles.cardDisabled : {}),
              }}
            >
              <div style={styles.labelContainer}>
                <div className="technical-label" style={styles.label}>{sim.label}</div>
                <div style={styles.subsystem}>{sim.subsystem}</div>
              </div>
              <div style={styles.cardTitle}>{sim.title}</div>
              <div style={styles.cardDescription}>{sim.description}</div>
              {isSelected && (
                <div className="status-indicator active" style={styles.statusDot}></div>
              )}
            </button>
          );
        })}
      </div>
    </div>
  );
}

const styles = {
  container: {
    padding: '20px',
    borderBottom: '1px solid var(--border-color)',
    background: 'linear-gradient(180deg, var(--bg-primary) 0%, var(--bg-secondary) 100%)',
  },
  header: {
    margin: '0 0 15px 0',
    fontSize: '14px',
    fontWeight: 600,
    color: 'var(--text-secondary)',
    textTransform: 'uppercase' as const,
    letterSpacing: '2px',
  },
  cardContainer: {
    display: 'flex',
    gap: '12px',
    flexDirection: 'column' as const,
  },
  card: {
    padding: '16px',
    cursor: 'pointer',
    transition: 'all 0.25s ease',
    textAlign: 'left' as const,
    position: 'relative' as const,
  },
  cardSelected: {
    borderColor: 'var(--accent-cyan)',
    boxShadow: '0 0 16px rgba(0, 212, 255, 0.4), inset 0 1px 0 rgba(0, 212, 255, 0.2)',
  },
  cardHovered: {
    transform: 'translateY(-2px)',
    boxShadow: '0 6px 16px rgba(0, 0, 0, 0.4)',
  },
  cardDisabled: {
    opacity: 0.4,
    cursor: 'not-allowed',
  },
  labelContainer: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: '12px',
  },
  label: {
    fontSize: '14px',
    color: 'var(--accent-cyan)',
    padding: '4px 8px',
    background: 'rgba(0, 212, 255, 0.1)',
    border: '1px solid rgba(0, 212, 255, 0.3)',
    borderRadius: '4px',
  },
  subsystem: {
    fontSize: '9px',
    color: 'var(--text-secondary)',
    letterSpacing: '1.5px',
    fontWeight: 600,
    textTransform: 'uppercase' as const,
  },
  cardTitle: {
    fontSize: '16px',
    fontWeight: 600,
    color: 'var(--text-primary)',
    marginBottom: '6px',
    letterSpacing: '0.3px',
  },
  cardDescription: {
    fontSize: '12px',
    color: 'var(--text-secondary)',
    lineHeight: '1.5',
  },
  statusDot: {
    position: 'absolute' as const,
    top: '12px',
    right: '12px',
  },
};
