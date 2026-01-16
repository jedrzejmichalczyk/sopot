import { useState, useCallback, useRef } from 'react';

export type SnapPoint = 'hidden' | 'collapsed' | 'half' | 'expanded';

interface UseBottomSheetOptions {
  initialSnapPoint?: SnapPoint;
  onSnapPointChange?: (snapPoint: SnapPoint) => void;
}

export function useBottomSheet(options: UseBottomSheetOptions = {}) {
  const { initialSnapPoint = 'hidden', onSnapPointChange } = options;
  const [snapPoint, setSnapPointInternal] = useState<SnapPoint>(initialSnapPoint);
  const [isDragging, setIsDragging] = useState(false);
  const [translateY, setTranslateY] = useState(0);
  const startYRef = useRef(0);
  const currentYRef = useRef(0);

  const setSnapPoint = useCallback((newSnapPoint: SnapPoint) => {
    setSnapPointInternal(newSnapPoint);
    setTranslateY(0);
    onSnapPointChange?.(newSnapPoint);
  }, [onSnapPointChange]);

  const handleTouchStart = useCallback((e: TouchEvent) => {
    startYRef.current = e.touches[0].clientY;
    currentYRef.current = e.touches[0].clientY;
    setIsDragging(true);
  }, []);

  const handleTouchMove = useCallback((e: TouchEvent) => {
    if (!isDragging) return;

    const currentY = e.touches[0].clientY;
    const deltaY = currentY - startYRef.current;
    currentYRef.current = currentY;

    // Only allow dragging down or up based on snap point
    if (snapPoint === 'expanded' && deltaY < 0) return; // Can't drag up from expanded
    if (snapPoint === 'hidden' && deltaY > 0) return; // Can't drag down from hidden

    setTranslateY(deltaY);
  }, [isDragging, snapPoint]);

  const handleTouchEnd = useCallback(() => {
    if (!isDragging) return;
    setIsDragging(false);

    const deltaY = translateY;
    const threshold = 50; // pixels

    // Determine next snap point based on drag direction and distance
    if (Math.abs(deltaY) < threshold) {
      // Snap back to current position
      setTranslateY(0);
      return;
    }

    if (deltaY > threshold) {
      // Dragging down
      if (snapPoint === 'expanded') {
        setSnapPoint('half');
      } else if (snapPoint === 'half') {
        setSnapPoint('collapsed');
      } else if (snapPoint === 'collapsed') {
        setSnapPoint('hidden');
      }
    } else if (deltaY < -threshold) {
      // Dragging up
      if (snapPoint === 'hidden') {
        setSnapPoint('collapsed');
      } else if (snapPoint === 'collapsed') {
        setSnapPoint('half');
      } else if (snapPoint === 'half') {
        setSnapPoint('expanded');
      }
    }

    setTranslateY(0);
  }, [isDragging, translateY, snapPoint, setSnapPoint]);

  return {
    snapPoint,
    setSnapPoint,
    isDragging,
    translateY,
    handleTouchStart,
    handleTouchMove,
    handleTouchEnd,
  };
}
