import { useEffect, useRef, useState } from 'react';

export interface Grid2DState {
  time: number;
  rows: number;
  cols: number;
  positions: Array<{ x: number; y: number }>; // Position of each mass
  velocities: Array<{ vx: number; vy: number }>; // Velocity of each mass
}

interface Grid2DVisualizationProps {
  state: Grid2DState | null;
  showVelocities?: boolean;
  showGrid?: boolean;
}

export function Grid2DVisualization({
  state,
  showVelocities = false,
  showGrid = true,
}: Grid2DVisualizationProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [dimensions, setDimensions] = useState({ width: 800, height: 600 });

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

  // Render the grid
  useEffect(() => {
    if (!canvasRef.current || !state) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.fillStyle = 'var(--bg-primary)';
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

    // Add padding
    const padding = 50;

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
    const toCanvasX = (x: number) =>
      padding + (x - minX) * scale;
    const toCanvasY = (y: number) =>
      dimensions.height - (padding + (y - minY) * scale); // Flip Y axis

    // Draw grid edges (springs)
    if (showGrid) {
      ctx.strokeStyle = 'var(--bg-tertiary)';
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
      ctx.strokeStyle = 'var(--accent-cyan)';
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
    positions.forEach((pos) => {
      const x = toCanvasX(pos.x);
      const y = toCanvasY(pos.y);

      // Mass point
      ctx.fillStyle = 'var(--accent-red)';
      ctx.beginPath();
      ctx.arc(x, y, 4, 0, 2 * Math.PI);
      ctx.fill();

      // Glow effect
      ctx.fillStyle = 'rgba(231, 76, 60, 0.3)';
      ctx.beginPath();
      ctx.arc(x, y, 8, 0, 2 * Math.PI);
      ctx.fill();
    });

    // Draw time display
    ctx.fillStyle = 'var(--text-primary)';
    ctx.font = '16px monospace';
    ctx.fillText(`t = ${state.time.toFixed(3)}s`, 10, 25);

    // Draw grid info
    ctx.fillText(`Grid: ${state.rows}×${state.cols}`, 10, 45);
  }, [state, dimensions, showVelocities, showGrid]);

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
