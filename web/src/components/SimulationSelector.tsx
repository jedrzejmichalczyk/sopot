export type SimulationType = 'rocket' | 'grid2d' | 'pendulum';

export interface SimulationInfo {
  type: SimulationType;
  label: string;
  shortTitle: string;
  title: string;
  description: string;
  subsystem: string;
}

export const SIMULATIONS: SimulationInfo[] = [
  {
    type: 'rocket',
    label: 'RKTFLT',
    shortTitle: 'Rocket Flight',
    title: 'Rocket Flight',
    description: '6-DOF trajectory simulation with aerodynamics',
    subsystem: 'PROPULSION',
  },
  {
    type: 'grid2d',
    label: 'GRID2D',
    shortTitle: 'Mass-Spring Grid',
    title: '2D Mass-Spring Grid',
    description: 'Deformable membrane physics simulation',
    subsystem: 'STRUCTURAL',
  },
  {
    type: 'pendulum',
    label: 'INVPND',
    shortTitle: 'Inverted Pendulum',
    title: 'Inverted 6-Pendulum on Cart',
    description: 'LQR-controlled balancing with real-time stabilization',
    subsystem: 'CONTROL',
  },
];

interface SimulationSelectorProps {
  currentSimulation: SimulationType;
  onSimulationChange: (type: SimulationType) => void;
}

/**
 * Top-level problem selector, rendered as an always-visible tab bar.
 *
 * Selection is intentionally never disabled: switching mid-run is safe
 * because the App-level handler pauses and resets the active simulation.
 */
export function SimulationSelector({
  currentSimulation,
  onSimulationChange,
}: SimulationSelectorProps) {
  return (
    <header className="sim-header">
      <div className="sim-header-brand">
        <span className="sim-header-logo">SOPOT</span>
        <span className="sim-header-tagline">Physics Lab</span>
      </div>
      <nav className="sim-tabs" role="tablist" aria-label="Simulation problem">
        {SIMULATIONS.map((sim) => {
          const isSelected = currentSimulation === sim.type;
          return (
            <button
              key={sim.type}
              role="tab"
              aria-selected={isSelected}
              className={`sim-tab${isSelected ? ' sim-tab-active' : ''}`}
              onClick={() => onSimulationChange(sim.type)}
              title={`${sim.title} — ${sim.description}`}
            >
              <span className="sim-tab-code">{sim.label}</span>
              <span className="sim-tab-title">{sim.shortTitle}</span>
            </button>
          );
        })}
        <a
          href={`${import.meta.env.BASE_URL}winding.html`}
          className="sim-tab sim-tab-link"
          title="Filament Winding CAD/CAM — Oxidizer tank winding machine simulation with G-code output (opens separate tool)"
        >
          <span className="sim-tab-code">FILWND</span>
          <span className="sim-tab-title">Filament Winding ↗</span>
        </a>
      </nav>
    </header>
  );
}

/**
 * Compact summary of the currently selected problem, shown above the
 * control panel (left panel on desktop, controls sheet on mobile).
 */
export function ActiveSimulationBanner({
  currentSimulation,
}: {
  currentSimulation: SimulationType;
}) {
  const sim = SIMULATIONS.find((s) => s.type === currentSimulation);
  if (!sim) return null;

  return (
    <div className="sim-active-banner mission-panel corner-accent">
      <div className="sim-active-banner-row">
        <span className="sim-active-banner-code">{sim.label}</span>
        <span className="sim-active-banner-subsystem">{sim.subsystem}</span>
      </div>
      <div className="sim-active-banner-title">{sim.title}</div>
      <div className="sim-active-banner-desc">{sim.description}</div>
    </div>
  );
}
