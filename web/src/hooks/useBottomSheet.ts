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

  // Use refs to avoid stale closures
  const startYRef = useRef(0);
  const currentYRef = useRef(0);
  const snapPointRef = useRef(snapPoint);
  const isDraggingRef = useRef(isDragging);
  const translateYRef = useRef(translateY);

  // Keep refs in sync with state
  snapPointRef.current = snapPoint;
  isDraggingRef.current = isDragging;
  translateYRef.current = translateY;

  const setSnapPoint = useCallback((newSnapPoint: SnapPoint) => {
    setSnapPointInternal(newSnapPoint);
    setTranslateY(0);
    onSnapPointChange?.(newSnapPoint);
  }, [onSnapPointChange]);

  const handleTouchStart = useCallback((e: TouchEvent) => {
    e.preventDefault();
    startYRef.current = e.touches[0].clientY;
    currentYRef.current = e.touches[0].clientY;
    setIsDragging(true);
  }, []);

  const handleTouchMove = useCallback((e: TouchEvent) => {
    // Use ref to avoid stale closure
    if (!isDraggingRef.current) return;

    e.preventDefault();
    const currentY = e.touches[0].clientY;
    const deltaY = currentY - startYRef.current;
    currentYRef.current = currentY;

    // Get current snap point from ref
    const currentSnapPoint = snapPointRef.current;

    // Only allow dragging down from expanded, or up from hidden
    if (currentSnapPoint === 'expanded' && deltaY < 0) return; // Can't drag up from expanded
    if (currentSnapPoint === 'hidden' && deltaY > 0) return; // Can't drag down from hidden

    setTranslateY(deltaY);
  }, []); // No dependencies - use refs instead

  const handleTouchEnd = useCallback(() => {
    // Use ref to avoid stale closure
    if (!isDraggingRef.current) return;
    setIsDragging(false);

    const deltaY = translateYRef.current;
    const currentSnapPoint = snapPointRef.current;
    const threshold = 50; // pixels

    // Determine next snap point based on drag direction and distance
    if (Math.abs(deltaY) < threshold) {
      // Snap back to current position
      setTranslateY(0);
      return;
    }

    if (deltaY > threshold) {
      // Dragging down - collapse sheet
      if (currentSnapPoint === 'expanded') {
        setSnapPoint('half');
      } else if (currentSnapPoint === 'half') {
        setSnapPoint('collapsed');
      } else if (currentSnapPoint === 'collapsed') {
        setSnapPoint('hidden');
      }
    } else if (deltaY < -threshold) {
      // Dragging up - expand sheet
      if (currentSnapPoint === 'hidden') {
        setSnapPoint('collapsed');
      } else if (currentSnapPoint === 'collapsed') {
        setSnapPoint('half');
      } else if (currentSnapPoint === 'half') {
        setSnapPoint('expanded');
      }
    }

    setTranslateY(0);
  }, [setSnapPoint]); // Only depends on setSnapPoint

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
