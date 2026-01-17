import { useEffect, useRef, useState, useCallback } from 'react';

// Constants for touch interaction and visualization
const TOUCH_RADIUS_PX = 30; // pixels - hit detection radius for touch targets
const TOUCH_FEEDBACK_DURATION_MS = 200; // milliseconds - visual feedback duration
const PERTURB_STRENGTH_M = 0.2; // meters - vertical perturbation strength
const CANVAS_PADDING_PX = 50; // pixels - padding around visualization area
const MASS_RADIUS_NORMAL_PX = 4; // pixels - normal mass render radius
const MASS_RADIUS_TOUCHED_PX = 6; // pixels - touched mass render radius
const MASS_GLOW_RADIUS_NORMAL_PX = 8; // pixels - normal mass glow radius
const MASS_GLOW_RADIUS_TOUCHED_PX = 12; // pixels - touched mass glow radius
const MASS_GLOW_OPACITY_NORMAL = 0.3; // opacity for normal glow
const MASS_GLOW_OPACITY_TOUCHED = 0.6; // opacity for touched glow

export interface Grid2DState {
  time: number;
  rows: number;
  cols: number;
  positions: Array<{ x: number; y: number }>; // Position of each mass
  velocities: Array<{ vx: number; vy: number }>; // Velocity of each mass
  centerOfMass?: { x: number; y: number }; // Center of mass
  kineticEnergy?: number; // Kinetic energy
  potentialEnergy?: number; // Potential energy
  totalEnergy?: number; // Total energy
}

interface Grid2DVisualizationProps {
  state: Grid2DState | null;
  showVelocities?: boolean;
  showGrid?: boolean;
  onMassPerturb?: (row: number, col: number, dx: number, dy: number) => void;
}

/**
 * Helper function to resolve CSS variables for Canvas 2D API
 * Canvas context cannot parse var() syntax, so we need to resolve it
 */
function getCSSVariable(variableName: string): string {
  return getComputedStyle(document.documentElement)
    .getPropertyValue(variableName)
    .trim();
}

export function Grid2DVisualization({
  state,
  showVelocities = false,
  showGrid = true,
  onMassPerturb,
}: Grid2DVisualizationProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [dimensions, setDimensions] = useState({ width: 800, height: 600 });
  const [touchedMass, setTouchedMass] = useState<number | null>(null);

  // Handle canvas resizing
  useEffect(() => {
    const updateSize = () => {
      if (containerRef.current) {
        const { width, height } = containerRef.current.getBoundingClientRect();
        setDimensions({ width, height });
      }
    };

    updateSize();
    window.addEventListener('resize', updateSize);
    return () => window.removeEventListener('resize', updateSize);
  }, []);

  // Handle touch/click interaction with masses
  const handleMassInteraction = useCallback((clientX: number, clientY: number) => {
    if (!canvasRef.current || !state || !onMassPerturb) return;

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const canvasX = clientX - rect.left;
    const canvasY = clientY - rect.top;

    // Calculate scaling parameters (same as in rendering)
    const positions = state.positions;
    if (positions.length === 0) return;

    const xs = positions.map((p) => p.x);
    const ys = positions.map((p) => p.y);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);

    // Center of bounding box (for transformation)
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;

    const padding = CANVAS_PADDING_PX;
    const estimatedSpacing = Math.max(
      (maxX - minX) / (state.cols - 1 || 1),
      (maxY - minY) / (state.rows - 1 || 1)
    ) || 0.5;

    const rangeX = Math.max(maxX - minX, estimatedSpacing);
    const rangeY = Math.max(maxY - minY, estimatedSpacing);
    const scaleX = (dimensions.width - 2 * padding) / rangeX;
    const scaleY = (dimensions.height - 2 * padding) / rangeY;
    const scale = Math.min(scaleX, scaleY);

    // Transform functions - center the view on the bounding box center
    const canvasCenterX = dimensions.width / 2;
    const canvasCenterY = dimensions.height / 2;
    const toCanvasX = (x: number) => canvasCenterX + (x - centerX) * scale;
    const toCanvasY = (y: number) => canvasCenterY - (y - centerY) * scale; // Flip Y axis

    // Find closest mass
    let closestIdx = -1;
    let closestDist = Infinity;
    const touchRadius = TOUCH_RADIUS_PX;

    positions.forEach((pos, idx) => {
      const massCanvasX = toCanvasX(pos.x);
      const massCanvasY = toCanvasY(pos.y);
      const dist = Math.sqrt(
        Math.pow(canvasX - massCanvasX, 2) + Math.pow(canvasY - massCanvasY, 2)
      );
      if (dist < closestDist && dist < touchRadius) {
        closestDist = dist;
        closestIdx = idx;
      }
    });

    if (closestIdx !== -1) {
      // Convert index to row/col
      const row = Math.floor(closestIdx / state.cols);
      const col = closestIdx % state.cols;

      // Apply upward perturbation
      onMassPerturb(row, col, 0, PERTURB_STRENGTH_M);

      // Visual feedback
      setTouchedMass(closestIdx);
      setTimeout(() => setTouchedMass(null), TOUCH_FEEDBACK_DURATION_MS);
    }
  }, [state, dimensions, onMassPerturb]);

  // Touch event handlers
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !state) return;

    const handleTouch = (e: TouchEvent) => {
      e.preventDefault();
      const touch = e.touches[0] || e.changedTouches[0];
      if (touch) {
        handleMassInteraction(touch.clientX, touch.clientY);
      }
    };

    const handleClick = (e: MouseEvent) => {
      handleMassInteraction(e.clientX, e.clientY);
    };

    canvas.addEventListener('touchstart', handleTouch, { passive: false });
    canvas.addEventListener('click', handleClick);

    return () => {
      canvas.removeEventListener('touchstart', handleTouch);
      canvas.removeEventListener('click', handleClick);
    };
  }, [handleMassInteraction, state]);

  // Render the grid
  useEffect(() => {
    if (!canvasRef.current || !state) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.fillStyle = getCSSVariable('--bg-primary');
    ctx.fillRect(0, 0, dimensions.width, dimensions.height);

    // Calculate bounds for scaling
    const positions = state.positions;
    if (positions.length === 0) return;

    const xs = positions.map((p) => p.x);
    const ys = positions.map((p) => p.y);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);

    // Use center of mass as the view center (NOT bounding box center)
    // This keeps the CoM visually stationary on screen, as it should be in physics
    const centerX = state.centerOfMass?.x ?? (minX + maxX) / 2;
    const centerY = state.centerOfMass?.y ?? (minY + maxY) / 2;

    // Add padding
    const padding = CANVAS_PADDING_PX;

    // Use grid-based fallback for better scaling when points are collinear
    // Estimate grid spacing from number of rows/cols
    const estimatedSpacing = Math.max(
      (maxX - minX) / (state.cols - 1 || 1),
      (maxY - minY) / (state.rows - 1 || 1)
    ) || 0.5; // Default to 0.5 if spacing can't be estimated

    const rangeX = Math.max(maxX - minX, estimatedSpacing);
    const rangeY = Math.max(maxY - minY, estimatedSpacing);
    const scaleX = (dimensions.width - 2 * padding) / rangeX;
    const scaleY = (dimensions.height - 2 * padding) / rangeY;
    const scale = Math.min(scaleX, scaleY);

    // Transform from simulation coordinates to canvas coordinates
    // Center the view on the CENTER OF MASS to keep it visually stationary
    const canvasCenterX = dimensions.width / 2;
    const canvasCenterY = dimensions.height / 2;
    const toCanvasX = (x: number) => canvasCenterX + (x - centerX) * scale;
    const toCanvasY = (y: number) => canvasCenterY - (y - centerY) * scale; // Flip Y axis

    // Draw grid edges (springs)
    if (showGrid) {
      ctx.strokeStyle = getCSSVariable('--bg-tertiary');
      ctx.lineWidth = 1;

      const { rows, cols } = state;

      // Horizontal edges
      for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols - 1; c++) {
          const idx1 = r * cols + c;
          const idx2 = r * cols + c + 1;
          const p1 = positions[idx1];
          const p2 = positions[idx2];

          ctx.beginPath();
          ctx.moveTo(toCanvasX(p1.x), toCanvasY(p1.y));
          ctx.lineTo(toCanvasX(p2.x), toCanvasY(p2.y));
          ctx.stroke();
        }
      }

      // Vertical edges
      for (let r = 0; r < rows - 1; r++) {
        for (let c = 0; c < cols; c++) {
          const idx1 = r * cols + c;
          const idx2 = (r + 1) * cols + c;
          const p1 = positions[idx1];
          const p2 = positions[idx2];

          ctx.beginPath();
          ctx.moveTo(toCanvasX(p1.x), toCanvasY(p1.y));
          ctx.lineTo(toCanvasX(p2.x), toCanvasY(p2.y));
          ctx.stroke();
        }
      }
    }

    // Draw velocity vectors
    if (showVelocities && state.velocities) {
      ctx.strokeStyle = getCSSVariable('--accent-cyan');
      ctx.lineWidth = 2;

      const velocityScale = 0.1; // Scale factor for velocity visualization

      positions.forEach((pos, idx) => {
        const vel = state.velocities[idx];
        if (!vel) return;

        const x1 = toCanvasX(pos.x);
        const y1 = toCanvasY(pos.y);
        const x2 = x1 + vel.vx * velocityScale * scale;
        const y2 = y1 - vel.vy * velocityScale * scale; // Flip Y

        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();

        // Arrow head
        const angle = Math.atan2(y2 - y1, x2 - x1);
        const headLength = 8;
        ctx.beginPath();
        ctx.moveTo(x2, y2);
        ctx.lineTo(
          x2 - headLength * Math.cos(angle - Math.PI / 6),
          y2 - headLength * Math.sin(angle - Math.PI / 6)
        );
        ctx.moveTo(x2, y2);
        ctx.lineTo(
          x2 - headLength * Math.cos(angle + Math.PI / 6),
          y2 - headLength * Math.sin(angle + Math.PI / 6)
        );
        ctx.stroke();
      });
    }

    // Draw masses as circles
    positions.forEach((pos, idx) => {
      const x = toCanvasX(pos.x);
      const y = toCanvasY(pos.y);
      const isTouched = idx === touchedMass;

      // Glow effect (larger if touched)
      const redColor = getCSSVariable('--accent-red');
      const rgb = redColor.match(/^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i);
      const opacity = isTouched ? MASS_GLOW_OPACITY_TOUCHED : MASS_GLOW_OPACITY_NORMAL;
      if (rgb) {
        ctx.fillStyle = `rgba(${parseInt(rgb[1], 16)}, ${parseInt(rgb[2], 16)}, ${parseInt(rgb[3], 16)}, ${opacity})`;
      } else {
        ctx.fillStyle = `rgba(255, 59, 59, ${opacity})`;
      }
      ctx.beginPath();
      const glowRadius = isTouched ? MASS_GLOW_RADIUS_TOUCHED_PX : MASS_GLOW_RADIUS_NORMAL_PX;
      ctx.arc(x, y, glowRadius, 0, 2 * Math.PI);
      ctx.fill();

      // Mass point
      ctx.fillStyle = getCSSVariable('--accent-red');
      ctx.beginPath();
      const massRadius = isTouched ? MASS_RADIUS_TOUCHED_PX : MASS_RADIUS_NORMAL_PX;
      ctx.arc(x, y, massRadius, 0, 2 * Math.PI);
      ctx.fill();
    });

    // Draw center of mass with different color
    if (state.centerOfMass) {
      const comX = toCanvasX(state.centerOfMass.x);
      const comY = toCanvasY(state.centerOfMass.y);

      // Glow effect for center of mass (cyan color)
      const cyanColor = getCSSVariable('--accent-cyan');
      const cyanRgb = cyanColor.match(/^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i);
      if (cyanRgb) {
        ctx.fillStyle = `rgba(${parseInt(cyanRgb[1], 16)}, ${parseInt(cyanRgb[2], 16)}, ${parseInt(cyanRgb[3], 16)}, 0.4)`;
      } else {
        ctx.fillStyle = 'rgba(59, 174, 218, 0.4)';
      }
      ctx.beginPath();
      ctx.arc(comX, comY, 12, 0, 2 * Math.PI);
      ctx.fill();

      // Center of mass point
      ctx.fillStyle = getCSSVariable('--accent-cyan');
      ctx.beginPath();
      ctx.arc(comX, comY, 6, 0, 2 * Math.PI);
      ctx.fill();

      // Add label
      ctx.fillStyle = getCSSVariable('--text-primary');
      ctx.font = '12px monospace';
      ctx.fillText('CoM', comX + 10, comY - 10);
    }

    // Draw time display
    ctx.fillStyle = getCSSVariable('--text-primary');
    ctx.font = '14px monospace';
    ctx.fillText(`t = ${state.time.toFixed(3)}s`, 10, 20);

    // Draw grid info
    ctx.fillText(`Grid: ${state.rows}×${state.cols}`, 10, 38);

    // Draw energy info
    if (state.totalEnergy !== undefined) {
      const ke = state.kineticEnergy ?? 0;
      const pe = state.potentialEnergy ?? 0;
      const total = state.totalEnergy;

      ctx.fillStyle = getCSSVariable('--accent-cyan');
      ctx.fillText(`KE: ${ke.toFixed(3)} J`, 10, 56);

      ctx.fillStyle = getCSSVariable('--accent-amber');
      ctx.fillText(`PE: ${pe.toFixed(3)} J`, 10, 74);

      ctx.fillStyle = getCSSVariable('--accent-green');
      ctx.fillText(`E:  ${total.toFixed(3)} J`, 10, 92);
    }

    // Draw center of mass coordinates (should be constant with no external forces)
    if (state.centerOfMass) {
      ctx.fillStyle = getCSSVariable('--accent-cyan');
      ctx.fillText(`CoM: (${state.centerOfMass.x.toFixed(3)}, ${state.centerOfMass.y.toFixed(3)})`, 10, 110);
    }
  }, [state, dimensions, showVelocities, showGrid, touchedMass]);

  return (
    <div ref={containerRef} style={styles.container}>
      <canvas
        ref={canvasRef}
        width={dimensions.width}
        height={dimensions.height}
        style={styles.canvas}
      />
      {!state && (
        <div style={styles.placeholder}>
          <div style={styles.placeholderText}>
            Initialize simulation to begin
          </div>
        </div>
      )}
    </div>
  );
}

const styles = {
  container: {
    position: 'relative' as const,
    width: '100%',
    height: '100%',
    backgroundColor: 'var(--bg-primary)',
    overflow: 'hidden',
  },
  canvas: {
    display: 'block',
    width: '100%',
    height: '100%',
  },
  placeholder: {
    position: 'absolute' as const,
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    pointerEvents: 'none' as const,
  },
  placeholderText: {
    color: 'var(--text-secondary)',
    fontSize: '18px',
    fontWeight: 'bold' as const,
  },
};
