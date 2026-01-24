import React, { useState } from 'react';
import { PlayIcon, PauseIcon, MenuIcon, SettingsIcon, DataIcon, ChartIcon, RefreshIcon, CloseIcon } from './icons/Icons';

interface FABProps {
  isRunning: boolean;
  isInitialized: boolean;
  simulationType: string;
  onPlayPause: () => void;
  onReset: () => void;
  onOpenPanel: (panel: 'controls' | 'telemetry' | 'plots') => void;
}

export const FloatingActionButton: React.FC<FABProps> = ({
  isRunning,
  isInitialized,
  simulationType,
  onPlayPause,
  onReset,
  onOpenPanel,
}) => {
  const [isMenuOpen, setIsMenuOpen] = useState(false);

  const toggleMenu = () => setIsMenuOpen(!isMenuOpen);

  return (
    <div className="fab-container">
      {/* Secondary action buttons (appear when menu is open) */}
      {isMenuOpen && (
        <>
          <div className="fab-backdrop" onClick={() => setIsMenuOpen(false)} />
          <div className="fab-menu">
            <button
              className="fab-menu-item"
              onClick={() => {
                onOpenPanel('controls');
                setIsMenuOpen(false);
              }}
              title="Controls"
            >
              <SettingsIcon size={20} />
              <span>Controls</span>
            </button>
            {simulationType === 'rocket' && (
              <>
                <button
                  className="fab-menu-item"
                  onClick={() => {
                    onOpenPanel('telemetry');
                    setIsMenuOpen(false);
                  }}
                  title="Telemetry"
                >
                  <DataIcon size={20} />
                  <span>Data</span>
                </button>
                <button
                  className="fab-menu-item"
                  onClick={() => {
                    onOpenPanel('plots');
                    setIsMenuOpen(false);
                  }}
                  title="Plots"
                >
                  <ChartIcon size={20} />
                  <span>Plots</span>
                </button>
              </>
            )}
            {isInitialized && (
              <button
                className="fab-menu-item fab-menu-item-danger"
                onClick={() => {
                  onReset();
                  setIsMenuOpen(false);
                }}
                title="Reset"
              >
                <RefreshIcon size={20} />
                <span>Reset</span>
              </button>
            )}
          </div>
        </>
      )}

      {/* Primary play/pause button */}
      {isInitialized && (
        <button
          className={`fab-button fab-button-play ${isRunning ? 'fab-button-pause' : ''}`}
          onClick={onPlayPause}
          title={isRunning ? 'Pause' : 'Play'}
        >
          {isRunning ? <PauseIcon size={24} /> : <PlayIcon size={24} />}
        </button>
      )}

      {/* Menu toggle button */}
      <button
        className={`fab-button fab-button-menu ${isMenuOpen ? 'fab-button-active' : ''}`}
        onClick={toggleMenu}
        title={isMenuOpen ? 'Close menu' : 'Open menu'}
      >
        {isMenuOpen ? <CloseIcon size={24} /> : <MenuIcon size={24} />}
      </button>
    </div>
  );
};
