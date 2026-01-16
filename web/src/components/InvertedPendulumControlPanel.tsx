import { useState } from 'react';
import type { PendulumState } from './InvertedPendulumVisualization';

interface InvertedPendulumControlPanelProps {
  state: PendulumState | null;
  isRunning: boolean;
  isInitialized: boolean;
  simulationFailed: boolean;
  controllerEnabled: boolean;
  playbackSpeed: number;
  lqrGains: number[] | null;
  onInitialize: (theta1?: number, theta2?: number) => void;
  onStart: () => void;
  onPause: () => void;
  onReset: () => void;
  onSetPlaybackSpeed: (speed: number) => void;
  onSetControllerEnabled: (enabled: boolean) => void;
  onApplyDisturbance: (type: 'cart' | 'link1' | 'link2', impulse: number) => void;
}

export function InvertedPendulumControlPanel({
  state,
  isRunning,
  isInitialized,
  simulationFailed,
  controllerEnabled,
  playbackSpeed,
  lqrGains,
  onInitialize,
  onStart,
  onPause,
  onReset,
  onSetPlaybackSpeed,
  onSetControllerEnabled,
  onApplyDisturbance,
}: InvertedPendulumControlPanelProps) {
  const [initialTheta1, setInitialTheta1] = useState(5.7); // degrees
  const [initialTheta2, setInitialTheta2] = useState(2.9); // degrees
  const [disturbanceStrength, setDisturbanceStrength] = useState(5);

  const handleInitialize = () => {
    const theta1Rad = (initialTheta1 * Math.PI) / 180;
    const theta2Rad = (initialTheta2 * Math.PI) / 180;
    onInitialize(theta1Rad, theta2Rad);
  };

  const formatAngle = (rad: number) => {
    return ((rad * 180) / Math.PI).toFixed(1);
  };

  return (
    <div className="control-panel" style={{
      padding: '16px',
      backgroundColor: 'var(--bg-secondary)',
      borderRadius: '8px',
      maxWidth: '400px',
    }}>
      <h3 style={{ marginTop: 0, marginBottom: '16px', color: 'var(--text-primary)' }}>
        Inverted Pendulum Control
      </h3>

      {/* Initialization Section */}
      {!isInitialized && (
        <div style={{ marginBottom: '16px' }}>
          <h4 style={{ margin: '0 0 8px 0', color: 'var(--text-secondary)' }}>Initial Angles</h4>
          <div style={{ display: 'flex', gap: '16px', marginBottom: '8px' }}>
            <label style={{ flex: 1 }}>
              <span style={{ fontSize: '12px', color: 'var(--text-tertiary)' }}>θ₁ (degrees)</span>
              <input
                type="number"
                value={initialTheta1}
                onChange={(e) => setInitialTheta1(parseFloat(e.target.value) || 0)}
                style={{
                  width: '100%',
                  padding: '8px',
                  marginTop: '4px',
                  backgroundColor: 'var(--bg-tertiary)',
                  border: '1px solid var(--border-color)',
                  borderRadius: '4px',
                  color: 'var(--text-primary)',
                }}
              />
            </label>
            <label style={{ flex: 1 }}>
              <span style={{ fontSize: '12px', color: 'var(--text-tertiary)' }}>θ₂ (degrees)</span>
              <input
                type="number"
                value={initialTheta2}
                onChange={(e) => setInitialTheta2(parseFloat(e.target.value) || 0)}
                style={{
                  width: '100%',
                  padding: '8px',
                  marginTop: '4px',
                  backgroundColor: 'var(--bg-tertiary)',
                  border: '1px solid var(--border-color)',
                  borderRadius: '4px',
                  color: 'var(--text-primary)',
                }}
              />
            </label>
          </div>
          <button
            onClick={handleInitialize}
            style={{
              width: '100%',
              padding: '12px',
              backgroundColor: 'var(--accent-primary)',
              color: 'white',
              border: 'none',
              borderRadius: '6px',
              cursor: 'pointer',
              fontWeight: 'bold',
            }}
          >
            Initialize Simulation
          </button>
        </div>
      )}

      {/* Playback Controls */}
      {isInitialized && (
        <>
          <div style={{ marginBottom: '16px' }}>
            <div style={{ display: 'flex', gap: '8px', marginBottom: '8px' }}>
              {!isRunning ? (
                <button
                  onClick={onStart}
                  disabled={simulationFailed}
                  style={{
                    flex: 1,
                    padding: '10px',
                    backgroundColor: simulationFailed ? 'var(--bg-tertiary)' : 'var(--accent-primary)',
                    color: simulationFailed ? 'var(--text-tertiary)' : 'white',
                    border: 'none',
                    borderRadius: '6px',
                    cursor: simulationFailed ? 'not-allowed' : 'pointer',
                    fontWeight: 'bold',
                  }}
                >
                  ▶ Start
                </button>
              ) : (
                <button
                  onClick={onPause}
                  style={{
                    flex: 1,
                    padding: '10px',
                    backgroundColor: 'var(--accent-secondary)',
                    color: 'white',
                    border: 'none',
                    borderRadius: '6px',
                    cursor: 'pointer',
                    fontWeight: 'bold',
                  }}
                >
                  ⏸ Pause
                </button>
              )}
              <button
                onClick={onReset}
                style={{
                  flex: 1,
                  padding: '10px',
                  backgroundColor: 'var(--bg-tertiary)',
                  color: 'var(--text-primary)',
                  border: '1px solid var(--border-color)',
                  borderRadius: '6px',
                  cursor: 'pointer',
                }}
              >
                ↺ Reset
              </button>
            </div>

            {/* Playback Speed */}
            <label style={{ display: 'block', marginBottom: '8px' }}>
              <span style={{ fontSize: '12px', color: 'var(--text-tertiary)' }}>
                Playback Speed: {playbackSpeed.toFixed(1)}x
              </span>
              <input
                type="range"
                min="0.1"
                max="5"
                step="0.1"
                value={playbackSpeed}
                onChange={(e) => onSetPlaybackSpeed(parseFloat(e.target.value))}
                style={{ width: '100%', marginTop: '4px' }}
              />
            </label>
          </div>

          {/* Controller Toggle */}
          <div style={{ marginBottom: '16px' }}>
            <label style={{ display: 'flex', alignItems: 'center', gap: '8px', cursor: 'pointer' }}>
              <input
                type="checkbox"
                checked={controllerEnabled}
                onChange={(e) => onSetControllerEnabled(e.target.checked)}
                style={{ width: '18px', height: '18px' }}
              />
              <span style={{ color: 'var(--text-primary)' }}>LQR Controller Enabled</span>
            </label>
            {!controllerEnabled && (
              <p style={{ margin: '4px 0 0 26px', fontSize: '12px', color: 'var(--accent-danger)' }}>
                Warning: Pendulum will fall without controller!
              </p>
            )}
          </div>

          {/* Disturbances */}
          <div style={{ marginBottom: '16px' }}>
            <h4 style={{ margin: '0 0 8px 0', color: 'var(--text-secondary)' }}>Apply Disturbance</h4>
            <label style={{ display: 'block', marginBottom: '8px' }}>
              <span style={{ fontSize: '12px', color: 'var(--text-tertiary)' }}>
                Impulse Strength: {disturbanceStrength} N·s
              </span>
              <input
                type="range"
                min="1"
                max="20"
                step="1"
                value={disturbanceStrength}
                onChange={(e) => setDisturbanceStrength(parseInt(e.target.value))}
                style={{ width: '100%', marginTop: '4px' }}
              />
            </label>
            <div style={{ display: 'flex', gap: '8px' }}>
              <button
                onClick={() => onApplyDisturbance('cart', disturbanceStrength)}
                style={{
                  flex: 1,
                  padding: '8px',
                  backgroundColor: 'var(--bg-tertiary)',
                  color: 'var(--text-primary)',
                  border: '1px solid var(--border-color)',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  fontSize: '12px',
                }}
              >
                Push Cart →
              </button>
              <button
                onClick={() => onApplyDisturbance('link1', disturbanceStrength * 0.1)}
                style={{
                  flex: 1,
                  padding: '8px',
                  backgroundColor: 'var(--bg-tertiary)',
                  color: 'var(--text-primary)',
                  border: '1px solid var(--border-color)',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  fontSize: '12px',
                }}
              >
                Tap Link 1
              </button>
              <button
                onClick={() => onApplyDisturbance('link2', disturbanceStrength * 0.05)}
                style={{
                  flex: 1,
                  padding: '8px',
                  backgroundColor: 'var(--bg-tertiary)',
                  color: 'var(--text-primary)',
                  border: '1px solid var(--border-color)',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  fontSize: '12px',
                }}
              >
                Tap Link 2
              </button>
            </div>
          </div>

          {/* Status */}
          {simulationFailed && (
            <div style={{
              padding: '12px',
              backgroundColor: 'rgba(248, 113, 113, 0.1)',
              border: '1px solid var(--accent-danger)',
              borderRadius: '6px',
              marginBottom: '16px',
            }}>
              <p style={{ margin: 0, color: 'var(--accent-danger)', fontWeight: 'bold' }}>
                Pendulum Fell!
              </p>
              <p style={{ margin: '4px 0 0 0', fontSize: '12px', color: 'var(--text-secondary)' }}>
                The pendulum angles exceeded 45°. Click Reset to try again.
              </p>
            </div>
          )}

          {/* Telemetry */}
          {state && (
            <div style={{
              backgroundColor: 'var(--bg-tertiary)',
              padding: '12px',
              borderRadius: '6px',
              fontFamily: 'monospace',
              fontSize: '12px',
            }}>
              <h4 style={{ margin: '0 0 8px 0', color: 'var(--text-secondary)' }}>State</h4>
              <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>
                <span style={{ color: 'var(--text-tertiary)' }}>Time:</span>
                <span style={{ color: 'var(--text-primary)' }}>{state.time.toFixed(2)} s</span>

                <span style={{ color: 'var(--text-tertiary)' }}>Cart x:</span>
                <span style={{ color: 'var(--text-primary)' }}>{state.x.toFixed(3)} m</span>

                <span style={{ color: 'var(--text-tertiary)' }}>θ₁:</span>
                <span style={{ color: 'var(--text-primary)' }}>{formatAngle(state.theta1)}°</span>

                <span style={{ color: 'var(--text-tertiary)' }}>θ₂:</span>
                <span style={{ color: 'var(--text-primary)' }}>{formatAngle(state.theta2)}°</span>

                <span style={{ color: 'var(--text-tertiary)' }}>Control:</span>
                <span style={{ color: state.controlForce > 0 ? '#4ade80' : '#f87171' }}>
                  {state.controlForce.toFixed(1)} N
                </span>
              </div>
            </div>
          )}

          {/* LQR Gains */}
          {lqrGains && (
            <div style={{
              marginTop: '12px',
              backgroundColor: 'var(--bg-tertiary)',
              padding: '12px',
              borderRadius: '6px',
              fontSize: '11px',
            }}>
              <h4 style={{ margin: '0 0 4px 0', color: 'var(--text-secondary)' }}>LQR Gains</h4>
              <div style={{ fontFamily: 'monospace', color: 'var(--text-tertiary)' }}>
                K = [{lqrGains.map(k => k.toFixed(1)).join(', ')}]
              </div>
            </div>
          )}
        </>
      )}
    </div>
  );
}

export default InvertedPendulumControlPanel;
