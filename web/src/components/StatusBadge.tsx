interface StatusBadgeProps {
  isReady: boolean;
  isInitialized: boolean;
  isRunning: boolean;
  /** Optional override label/state (e.g. "Failed" for the pendulum). */
  failed?: boolean;
}

/**
 * Shared run-state indicator used at the top of every sub-problem control
 * panel so the three simulations read consistently.
 */
export function StatusBadge({ isReady, isInitialized, isRunning, failed }: StatusBadgeProps) {
  const { label, className } = failed
    ? { label: 'Failed', className: 'inactive' }
    : isRunning
      ? { label: 'Running', className: 'active' }
      : isInitialized
        ? { label: 'Ready', className: 'standby' }
        : isReady
          ? { label: 'Standby', className: 'standby' }
          : { label: 'Loading…', className: 'inactive' };

  return (
    <div className="ctl-status">
      <span className={`status-indicator ${className}`} />
      <span className="ctl-status-text">{label}</span>
    </div>
  );
}
