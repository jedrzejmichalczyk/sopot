import { useState } from 'react';

export type SimulationType = 'rocket' | 'grid2d';

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
      emoji: '🚀',
      title: 'Rocket Flight',
      description: '6-DOF rocket simulation with aerodynamics',
    },
    {
      type: 'grid2d' as SimulationType,
      emoji: '🎨',
      title: '2D Grid',
      description: 'Mass-spring cloth/membrane simulation',
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
              className="touch-button card"
              style={{
                ...styles.card,
                ...(isSelected ? styles.cardSelected : {}),
                ...(isHovered && !disabled ? styles.cardHovered : {}),
                ...(disabled ? styles.cardDisabled : {}),
              }}
            >
              <div style={styles.emoji}>{sim.emoji}</div>
              <div style={styles.cardTitle}>{sim.title}</div>
              <div style={styles.cardDescription}>{sim.description}</div>
              {isSelected && (
                <div style={styles.selectedIndicator}>
                  <span style={styles.checkmark}>✓</span> Active
                </div>
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
    borderBottom: '1px solid #34495e',
  },
  header: {
    margin: '0 0 15px 0',
    fontSize: '16px',
    fontWeight: 600,
    color: '#ecf0f1',
    textTransform: 'uppercase' as const,
    letterSpacing: '1px',
  },
  cardContainer: {
    display: 'flex',
    gap: '10px',
    flexDirection: 'column' as const,
  },
  card: {
    padding: '15px',
    backgroundColor: '#2c3e50',
    border: '2px solid #34495e',
    borderRadius: '8px',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
    textAlign: 'left' as const,
    position: 'relative' as const,
  },
  cardSelected: {
    backgroundColor: '#34495e',
    borderColor: '#3498db',
    boxShadow: '0 0 10px rgba(52, 152, 219, 0.3)',
  },
  cardHovered: {
    transform: 'translateY(-2px)',
    boxShadow: '0 4px 8px rgba(0, 0, 0, 0.2)',
  },
  cardDisabled: {
    opacity: 0.5,
    cursor: 'not-allowed',
  },
  emoji: {
    fontSize: '32px',
    marginBottom: '8px',
  },
  cardTitle: {
    fontSize: '16px',
    fontWeight: 'bold' as const,
    color: '#ecf0f1',
    marginBottom: '4px',
  },
  cardDescription: {
    fontSize: '12px',
    color: '#95a5a6',
    lineHeight: '1.4',
  },
  selectedIndicator: {
    position: 'absolute' as const,
    top: '10px',
    right: '10px',
    backgroundColor: '#27ae60',
    color: '#fff',
    fontSize: '11px',
    padding: '4px 8px',
    borderRadius: '12px',
    fontWeight: 'bold' as const,
    display: 'flex',
    alignItems: 'center',
    gap: '4px',
  },
  checkmark: {
    fontSize: '12px',
  },
};
