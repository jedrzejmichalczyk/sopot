import { useState } from 'react';
import type { PendulumState } from './InvertedPendulumVisualization';

interface InvertedPendulumControlPanelProps {
  state: PendulumState | null;
  isReady: boolean;
  isInitialized: boolean;
  isRunning: boolean;
  error: string | null;
  simulationFailed: boolean;
  controllerEnabled: boolean;
  playbackSpeed: number;
  lqrGains: number[] | null;
  numLinks: number;
  onInitialize: (initialAngleRad?: number) => void;
  onStart: () => void;
  onPause: () => void;
  onReset: () => void;
  onSetPlaybackSpeed: (speed: number) => void;
  onSetControllerEnabled: (enabled: boolean) => void;
  onApplyDisturbance: (type: 'cart' | 'link', index: number, impulse: number) => void;
}

export function InvertedPendulumControlPanel({
  state,
  isReady,
  isRunning,
  isInitialized,
  error,
  simulationFailed,
  controllerEnabled,
  playbackSpeed,
  lqrGains,
  numLinks,
  onInitialize,
  onStart,
  onPause,
  onReset,
  onSetPlaybackSpeed,
  onSetControllerEnabled,
  onApplyDisturbance,
}: InvertedPendulumControlPanelProps) {
  const [initialTilt, setInitialTilt] = useState(1.7); // degrees, applied to every link
  const [disturbanceStrength, setDisturbanceStrength] = useState(5);

  const handleInitialize = () => {
    onInitialize((initialTilt * Math.PI) / 180);
  };

  const formatAngle = (rad: number) => ((rad * 180) / Math.PI).toFixed(1);

  return (
    <div className="control-panel" style={{
      padding: '16px',
      backgroundColor: 'var(--bg-secondary)',
      borderRadius: '8px',
    }}>
      <h3 style={{ marginTop: 0, marginBottom: '16px', color: 'var(--text-primary)' }}>
        Inverted {numLinks}-Pendulum Control
      </h3>

      {/* Error display */}
      {error && (
        <div style={{
          padding: '12px',
          marginBottom: '16px',
          backgroundColor: 'rgba(248, 113, 113, 0.1)',
          border: '1px solid var(--accent-danger)',
          borderRadius: '6px',
          color: 'var(--accent-danger)',
          fontSize: '13px',
        }}>
          <strong>Error:</strong> {error}
        </div>
      )}

      {/* Initialization Section */}
      {!isInitialized && (
        <div style={{ marginBottom: '16px' }}>
          <h4 style={{ margin: '0 0 8px 0', color: 'var(--text-secondary)' }}>Initial Tilt</h4>
          <label style={{ display: 'block', marginBottom: '8px' }}>
            <span style={{ fontSize: '12px', color: 'var(--text-tertiary)' }}>
              Angle per link (degrees)
            </span>
            <input
              type="number"
              inputMode="decimal"
              value={initialTilt}
              step="0.1"
              disabled={!isReady}
              onChange={(e) => setInitialTilt(parseFloat(e.target.value) || 0)}
              style={{
                width: '100%',
                minHeight: '44px',
                padding: '8px',
                marginTop: '4px',
                backgroundColor: 'var(--bg-tertiary)',
                border: '1px solid var(--border-color)',
                borderRadius: '4px',
                color: 'var(--text-primary)',
                fontSize: '16px', // Prevent zoom on iOS
              }}
            />
          </label>
          <button
            onClick={handleInitialize}
            disabled={!isReady}
            className="touch-button btn-primary"
            style={{ width: '100%', opacity: isReady ? 1 : 0.5, cursor: isReady ? 'pointer' : 'not-allowed' }}
          >
            Initialize Simulation
          </button>
          {!isReady && !error && (
            <p style={{ margin: '8px 0 0 0', fontSize: '12px', color: 'var(--text-tertiary)' }}>
              Loading WebAssembly module…
            </p>
          )}
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
                  className="touch-button btn-success"
                  style={{ flex: 1 }}
                >
                  START
                </button>
              ) : (
                <button
                  onClick={onPause}
                  className="touch-button btn-warning"
                  style={{ flex: 1 }}
                >
                  PAUSE
                </button>
              )}
              <button
                onClick={onReset}
                className="touch-button btn-danger"
                style={{ flex: 1 }}
              >
                RESET
              </button>
            </div>

            {/* Playback Speed */}
            <label style={{ display: 'block', marginBottom: '8px' }}>
              <span style={{ fontSize: '12px', color: 'var(--text-tertiary)' }}>
                Playback Speed: {playbackSpeed.toFixed(1)}x
              </span>
              <input
                type="range"
                className="touch-slider"
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
                Warning: the pendulums will fall without the controller!
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
                className="touch-slider"
                min="1"
                max="20"
                step="1"
                value={disturbanceStrength}
                onChange={(e) => setDisturbanceStrength(parseInt(e.target.value))}
                style={{ width: '100%', marginTop: '4px' }}
              />
            </label>
            <div style={{ display: 'flex', gap: '8px', flexWrap: 'wrap' }}>
              <button
                onClick={() => onApplyDisturbance('cart', 0, -disturbanceStrength)}
                style={disturbanceButtonStyle}
              >
                ← Push Cart
              </button>
              <button
                onClick={() => onApplyDisturbance('cart', 0, disturbanceStrength)}
                style={disturbanceButtonStyle}
              >
                Push Cart →
              </button>
              <button
                onClick={() => onApplyDisturbance('link', 0, disturbanceStrength * 0.1)}
                style={disturbanceButtonStyle}
              >
                Tap Bottom Link
              </button>
              <button
                onClick={() => onApplyDisturbance('link', numLinks - 1, disturbanceStrength * 0.05)}
                style={disturbanceButtonStyle}
              >
                Tap Top Link
              </button>
            </div>
            <p style={{ margin: '6px 0 0 0', fontSize: '11px', color: 'var(--text-tertiary)' }}>
              Tip: tap any mass to nudge that link, or tap beside the cart to push it toward that side.
            </p>
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
                A Pendulum Fell!
              </p>
              <p style={{ margin: '4px 0 0 0', fontSize: '12px', color: 'var(--text-secondary)' }}>
                A link exceeded 45°. Click Reset to try again.
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

                <span style={{ color: 'var(--text-tertiary)' }}>max |θ|:</span>
                <span style={{ color: 'var(--text-primary)' }}>
                  {formatAngle(state.angles.reduce((m, a) => Math.max(m, Math.abs(a)), 0))}°
                </span>

                <span style={{ color: 'var(--text-tertiary)' }}>Control:</span>
                <span style={{ color: state.controlForce > 0 ? '#4ade80' : '#f87171' }}>
                  {state.controlForce.toFixed(1)} N
                </span>
              </div>
              {state.angles.length > 0 && (
                <div style={{ marginTop: '8px', color: 'var(--text-tertiary)' }}>
                  θ = [{state.angles.map((a) => formatAngle(a)).join(', ')}]°
                </div>
              )}
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
              <h4 style={{ margin: '0 0 4px 0', color: 'var(--text-secondary)' }}>
                LQR Gains ({lqrGains.length} states)
              </h4>
              <div style={{ fontFamily: 'monospace', color: 'var(--text-tertiary)', wordBreak: 'break-all' }}>
                K = [{lqrGains.map((k) => k.toFixed(1)).join(', ')}]
              </div>
            </div>
          )}
        </>
      )}
    </div>
  );
}

const disturbanceButtonStyle: React.CSSProperties = {
  flex: 1,
  minWidth: '110px',
  minHeight: '44px', // Accessible touch target
  padding: '8px 10px',
  backgroundColor: 'var(--bg-tertiary)',
  color: 'var(--text-primary)',
  border: '1px solid var(--border-color)',
  borderRadius: '6px',
  cursor: 'pointer',
  fontSize: '13px',
};

export default InvertedPendulumControlPanel;
