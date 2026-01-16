import React, { useRef, useEffect } from 'react';
import { useBottomSheet, SnapPoint } from '../hooks/useBottomSheet';
import { ChevronDownIcon, ChevronUpIcon } from './icons/Icons';

interface BottomSheetProps {
  isOpen: boolean;
  onClose: () => void;
  children: React.ReactNode;
  title?: string;
  initialSnapPoint?: SnapPoint;
}

export const BottomSheet: React.FC<BottomSheetProps> = ({
  isOpen,
  onClose,
  children,
  title,
  initialSnapPoint = 'half',
}) => {
  const {
    snapPoint,
    setSnapPoint,
    isDragging,
    translateY,
    handleTouchStart,
    handleTouchMove,
    handleTouchEnd,
  } = useBottomSheet({
    initialSnapPoint: isOpen ? initialSnapPoint : 'hidden',
    onSnapPointChange: (point) => {
      if (point === 'hidden') {
        onClose();
      }
    },
  });

  const handleRef = useRef<HTMLDivElement>(null);
  const sheetRef = useRef<HTMLDivElement>(null);

  // Update snap point when isOpen changes
  useEffect(() => {
    if (isOpen && snapPoint === 'hidden') {
      setSnapPoint(initialSnapPoint);
    } else if (!isOpen && snapPoint !== 'hidden') {
      setSnapPoint('hidden');
    }
  }, [isOpen, snapPoint, setSnapPoint, initialSnapPoint]);

  // Add touch event listeners
  useEffect(() => {
    const handle = handleRef.current;
    if (!handle) return;

    handle.addEventListener('touchstart', handleTouchStart);
    handle.addEventListener('touchmove', handleTouchMove);
    handle.addEventListener('touchend', handleTouchEnd);

    return () => {
      handle.removeEventListener('touchstart', handleTouchStart);
      handle.removeEventListener('touchmove', handleTouchMove);
      handle.removeEventListener('touchend', handleTouchEnd);
    };
  }, [handleTouchStart, handleTouchMove, handleTouchEnd]);

  const getSheetHeight = () => {
    switch (snapPoint) {
      case 'hidden':
        return '0';
      case 'collapsed':
        return '120px';
      case 'half':
        return '50vh';
      case 'expanded':
        return '85vh';
      default:
        return '0';
    }
  };

  const handleExpandCollapse = () => {
    if (snapPoint === 'expanded') {
      setSnapPoint('half');
    } else if (snapPoint === 'half') {
      setSnapPoint('expanded');
    } else if (snapPoint === 'collapsed') {
      setSnapPoint('half');
    }
  };

  return (
    <>
      {/* Backdrop */}
      {snapPoint !== 'hidden' && snapPoint !== 'collapsed' && (
        <div
          className="bottom-sheet-backdrop"
          onClick={() => setSnapPoint('hidden')}
        />
      )}

      {/* Bottom Sheet */}
      <div
        ref={sheetRef}
        className={`bottom-sheet ${isDragging ? 'dragging' : ''}`}
        style={{
          height: getSheetHeight(),
          transform: `translateY(${translateY}px)`,
        }}
      >
        {/* Drag Handle */}
        <div ref={handleRef} className="bottom-sheet-handle-container">
          <div className="bottom-sheet-handle" />
          {title && snapPoint !== 'hidden' && (
            <div className="bottom-sheet-header">
              <h3 className="bottom-sheet-title">{title}</h3>
              <button
                className="bottom-sheet-expand-button"
                onClick={handleExpandCollapse}
                aria-label={snapPoint === 'expanded' ? 'Collapse' : 'Expand'}
              >
                {snapPoint === 'expanded' ? (
                  <ChevronDownIcon size={20} />
                ) : (
                  <ChevronUpIcon size={20} />
                )}
              </button>
            </div>
          )}
        </div>

        {/* Content */}
        <div className="bottom-sheet-content">
          {children}
        </div>
      </div>
    </>
  );
};
