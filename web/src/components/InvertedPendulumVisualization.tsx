import { useEffect, useRef, useState, useCallback } from 'react';

// Constants for touch interaction and visualization
const CANVAS_PADDING_PX = 60; // pixels - padding around visualization area
const TOUCH_FEEDBACK_DURATION_MS = 200; // milliseconds - visual feedback duration
const HIT_AREA_EXPANSION_PX = 10; // pixels - expansion of hit detection areas
const IMPULSE_CART_NS = 5; // N·s - impulse strength for cart
const IMPULSE_LINK_NS = 0.5; // N·s - impulse strength for links
const MASS_BASE_RADIUS_PX = 6; // pixels - base radius for mass rendering
const MASS_RADIUS_SCALE_FACTOR = 12; // pixels per kg - scaling factor for mass size
const CART_WIDTH_M = 0.3; // meters - cart width in simulation units
const CART_HEIGHT_M = 0.15; // meters - cart height in simulation units
const GROUND_Y_FACTOR = 0.75; // fraction - ground position relative to canvas height
const SCALE_FACTOR = 0.85; // scaling factor for coordinate transforms
const VIEW_HEIGHT_FACTOR = 1.3; // factor for view height calculation
const HIGHLIGHT_GLOW_RADIUS_PX = 20; // pixels - glow radius when element is touched

// Distinct hues for the link chain (cycled if there are more links than colors).
const LINK_COLORS = ['#6366f1', '#8b5cf6', '#a855f7', '#d946ef', '#ec4899', '#f43f5e'];

export interface PendulumState {
  time: number;
  x: number;            // Cart position
  xdot: number;         // Cart velocity
  angles: number[];     // Link angles from vertical
  omegas: number[];     // Link angular velocities
  controlForce: number;
  numLinks: number;
}

export interface PendulumVisualizationData {
  cart: { x: number; y: number };
  joints: Array<{ x: number; y: number }>; // [pivot, tip₁, …, tip_N]
  angles: number[];
  controlForce: number;
  numLinks: number;
}

interface InvertedPendulumVisualizationProps {
  state: PendulumState | null;
  visualizationData?: PendulumVisualizationData | null;
  cartMass?: number;
  linkMass?: number;
  linkLength?: number;
  numLinks?: number;
  showTelemetry?: boolean;
  showForceArrow?: boolean;
  trackWidth?: number;  // Total track width for cart travel
  onApplyImpulse?: (type: 'cart' | 'link', index: number, impulse: number) => void;
}

/**
 * Helper function to resolve CSS variables for Canvas 2D API
 */
function getCSSVariable(variableName: string): string {
  return getComputedStyle(document.documentElement)
    .getPropertyValue(variableName)
    .trim();
}

export function InvertedPendulumVisualization({
  state,
  visualizationData,
  cartMass: _cartMass = 2.0,
  linkMass = 0.1,
  linkLength = 0.3,
  numLinks = 6,
  showTelemetry = true,
  showForceArrow = true,
  trackWidth = 3.0,
  onApplyImpulse,
}: InvertedPendulumVisualizationProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [dimensions, setDimensions] = useState({ width: 800, height: 600 });
  // Highlighted element: 'cart' or a link index.
  const [highlighted, setHighlighted] = useState<{ type: 'cart' | 'link'; index: number } | null>(null);

  // Resolve the chain of joint positions (pivot + tips) from data or state.
  const resolveJoints = useCallback((): Array<{ x: number; y: number }> => {
    if (visualizationData?.joints && visualizationData.joints.length > 0) {
      return visualizationData.joints;
    }
    if (state?.angles && state.angles.length > 0) {
      const joints: Array<{ x: number; y: number }> = [{ x: state.x, y: 0 }];
      let px = state.x;
      let py = 0;
      for (let i = 0; i < state.angles.length; i++) {
        px += linkLength * Math.sin(state.angles[i]);
        py += linkLength * Math.cos(state.angles[i]);
        joints.push({ x: px, y: py });
      }
      return joints;
    }
    // Default: upright chain at origin.
    const joints: Array<{ x: number; y: number }> = [{ x: 0, y: 0 }];
    for (let i = 0; i < numLinks; i++) {
      joints.push({ x: 0, y: (i + 1) * linkLength });
    }
    return joints;
  }, [visualizationData, state, linkLength, numLinks]);

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

  // Compute the simulation->canvas transform (shared by render and hit testing).
  const computeTransform = useCallback(() => {
    const totalHeight = numLinks * linkLength;
    const padding = CANVAS_PADDING_PX;
    const viewWidth = Math.max(trackWidth, 2 * totalHeight);
    const viewHeight = totalHeight * VIEW_HEIGHT_FACTOR;

    const scaleX = (dimensions.width - 2 * padding) / viewWidth;
    const scaleY = (dimensions.height - 2 * padding) / viewHeight;
    const scale = Math.min(scaleX, scaleY) * SCALE_FACTOR;

    const centerX = dimensions.width / 2;
    const groundY = dimensions.height * GROUND_Y_FACTOR;

    return {
      scale,
      toCanvasX: (x: number) => centerX + x * scale,
      toCanvasY: (y: number) => groundY - y * scale,
      groundY,
      centerX,
    };
  }, [dimensions, numLinks, linkLength, trackWidth]);

  // Handle touch/click interaction
  const handleInteraction = useCallback((clientX: number, clientY: number) => {
    if (!canvasRef.current || !onApplyImpulse) return;

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const canvasX = clientX - rect.left;
    const canvasY = clientY - rect.top;

    const joints = resolveJoints();
    if (joints.length === 0) return;

    const { scale, toCanvasX, toCanvasY, groundY } = computeTransform();
    const cartWidth = CART_WIDTH_M * scale;
    const cartHeight = CART_HEIGHT_M * scale;
    const massRadius = MASS_BASE_RADIUS_PX + linkMass * MASS_RADIUS_SCALE_FACTOR;

    // Check link masses (tips), nearest first.
    for (let i = joints.length - 1; i >= 1; i--) {
      const jx = toCanvasX(joints[i].x);
      const jy = toCanvasY(joints[i].y);
      const dist = Math.hypot(canvasX - jx, canvasY - jy);
      if (dist < massRadius + HIT_AREA_EXPANSION_PX) {
        onApplyImpulse('link', i - 1, IMPULSE_LINK_NS);
        setHighlighted({ type: 'link', index: i - 1 });
        setTimeout(() => setHighlighted(null), TOUCH_FEEDBACK_DURATION_MS);
        return;
      }
    }

    // Check cart.
    const cartX = toCanvasX(joints[0].x);
    if (
      canvasX >= cartX - cartWidth / 2 - HIT_AREA_EXPANSION_PX &&
      canvasX <= cartX + cartWidth / 2 + HIT_AREA_EXPANSION_PX &&
      canvasY >= groundY - cartHeight - HIT_AREA_EXPANSION_PX &&
      canvasY <= groundY + HIT_AREA_EXPANSION_PX
    ) {
      onApplyImpulse('cart', 0, IMPULSE_CART_NS);
      setHighlighted({ type: 'cart', index: 0 });
      setTimeout(() => setHighlighted(null), TOUCH_FEEDBACK_DURATION_MS);
    }
  }, [resolveJoints, computeTransform, linkMass, onApplyImpulse]);

  // Touch event handlers
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const handleTouch = (e: TouchEvent) => {
      e.preventDefault();
      const touch = e.touches[0] || e.changedTouches[0];
      if (touch) {
        handleInteraction(touch.clientX, touch.clientY);
      }
    };

    const handleClick = (e: MouseEvent) => {
      handleInteraction(e.clientX, e.clientY);
    };

    canvas.addEventListener('touchstart', handleTouch, { passive: false });
    canvas.addEventListener('click', handleClick);

    return () => {
      canvas.removeEventListener('touchstart', handleTouch);
      canvas.removeEventListener('click', handleClick);
    };
  }, [handleInteraction]);

  // Render the pendulum
  useEffect(() => {
    if (!canvasRef.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas with background
    ctx.fillStyle = getCSSVariable('--bg-primary') || '#1a1a2e';
    ctx.fillRect(0, 0, dimensions.width, dimensions.height);

    const joints = resolveJoints();
    const controlForce = visualizationData?.controlForce ?? state?.controlForce ?? 0;

    const { scale, toCanvasX, toCanvasY, groundY, centerX } = computeTransform();

    // Draw track
    const trackHalfWidth = (trackWidth / 2) * scale;
    ctx.strokeStyle = getCSSVariable('--text-tertiary') || '#666';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX - trackHalfWidth, groundY);
    ctx.lineTo(centerX + trackHalfWidth, groundY);
    ctx.stroke();

    // Draw track markers
    ctx.fillStyle = getCSSVariable('--text-tertiary') || '#666';
    for (let i = -5; i <= 5; i++) {
      const markerX = centerX + ((i * trackWidth) / 10) * scale;
      ctx.beginPath();
      ctx.arc(markerX, groundY + 5, 2, 0, Math.PI * 2);
      ctx.fill();
    }

    // Draw cart
    const cartWidth = CART_WIDTH_M * scale;
    const cartHeight = CART_HEIGHT_M * scale;
    const cartX = toCanvasX(joints[0].x);
    const cartY = groundY;

    if (highlighted?.type === 'cart') {
      ctx.fillStyle = '#6dd5ff';
      ctx.shadowBlur = HIGHLIGHT_GLOW_RADIUS_PX;
      ctx.shadowColor = '#6dd5ff';
    } else {
      ctx.fillStyle = getCSSVariable('--accent-secondary') || '#4a9eff';
    }
    ctx.fillRect(cartX - cartWidth / 2, cartY - cartHeight, cartWidth, cartHeight);
    ctx.shadowBlur = 0;

    // Cart wheels
    const wheelRadius = cartHeight * 0.3;
    ctx.fillStyle = getCSSVariable('--text-secondary') || '#888';
    ctx.beginPath();
    ctx.arc(cartX - cartWidth / 3, cartY, wheelRadius, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.arc(cartX + cartWidth / 3, cartY, wheelRadius, 0, Math.PI * 2);
    ctx.fill();

    // Draw force arrow
    if (showForceArrow && Math.abs(controlForce) > 0.1) {
      const maxForceDisplay = 200; // N
      const maxArrowLength = 80; // pixels
      const arrowLength = (controlForce / maxForceDisplay) * maxArrowLength;

      ctx.strokeStyle = controlForce > 0 ? '#4ade80' : '#f87171';
      ctx.fillStyle = controlForce > 0 ? '#4ade80' : '#f87171';
      ctx.lineWidth = 3;

      const arrowY = cartY - cartHeight / 2;
      const arrowStartX = cartX;
      const arrowEndX = cartX + arrowLength;

      ctx.beginPath();
      ctx.moveTo(arrowStartX, arrowY);
      ctx.lineTo(arrowEndX, arrowY);
      ctx.stroke();

      const headSize = 8;
      const dir = controlForce > 0 ? 1 : -1;
      ctx.beginPath();
      ctx.moveTo(arrowEndX, arrowY);
      ctx.lineTo(arrowEndX - dir * headSize, arrowY - headSize / 2);
      ctx.lineTo(arrowEndX - dir * headSize, arrowY + headSize / 2);
      ctx.closePath();
      ctx.fill();
    }

    // Draw the link chain.
    ctx.lineCap = 'round';
    const pivotX = toCanvasX(joints[0].x);
    const pivotY = toCanvasY(joints[0].y) - cartHeight; // pivot mounted on top of cart

    let prevX = pivotX;
    let prevY = pivotY;
    for (let i = 1; i < joints.length; i++) {
      const jx = toCanvasX(joints[i].x);
      const jy = toCanvasY(joints[i].y);

      ctx.strokeStyle = LINK_COLORS[(i - 1) % LINK_COLORS.length];
      ctx.lineWidth = Math.max(3, 6 - (i - 1) * 0.4);
      ctx.beginPath();
      ctx.moveTo(prevX, prevY);
      ctx.lineTo(jx, jy);
      ctx.stroke();

      prevX = jx;
      prevY = jy;
    }

    // Pivot joint marker.
    ctx.fillStyle = getCSSVariable('--text-primary') || '#fff';
    ctx.beginPath();
    ctx.arc(pivotX, pivotY, 5, 0, Math.PI * 2);
    ctx.fill();

    // Draw masses at each link tip.
    const massRadius = MASS_BASE_RADIUS_PX + linkMass * MASS_RADIUS_SCALE_FACTOR;
    for (let i = 1; i < joints.length; i++) {
      const jx = toCanvasX(joints[i].x);
      const jy = toCanvasY(joints[i].y);
      const color = LINK_COLORS[(i - 1) % LINK_COLORS.length];

      if (highlighted?.type === 'link' && highlighted.index === i - 1) {
        ctx.fillStyle = '#ffffff';
        ctx.shadowBlur = HIGHLIGHT_GLOW_RADIUS_PX;
        ctx.shadowColor = color;
      } else {
        ctx.fillStyle = color;
      }
      ctx.beginPath();
      ctx.arc(jx, jy, massRadius, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.shadowBlur = 0;
    }

    // Telemetry overlay
    if (showTelemetry && state) {
      ctx.fillStyle = getCSSVariable('--text-primary') || '#fff';
      ctx.font = '14px monospace';

      const tX = 20;
      let tY = 30;
      const lh = 18;

      ctx.fillText(`t = ${state.time.toFixed(2)} s`, tX, tY); tY += lh;
      ctx.fillText(`x = ${state.x.toFixed(3)} m`, tX, tY); tY += lh;
      const maxAng = state.angles.reduce((m, a) => Math.max(m, Math.abs(a)), 0);
      ctx.fillText(`max|θ| = ${((maxAng * 180) / Math.PI).toFixed(1)}°`, tX, tY); tY += lh;
      ctx.fillText(`F = ${state.controlForce.toFixed(1)} N`, tX, tY);
    }

    // Title
    ctx.fillStyle = getCSSVariable('--text-secondary') || '#888';
    ctx.font = 'bold 16px sans-serif';
    ctx.textAlign = 'center';
    const nLinks = joints.length - 1;
    ctx.fillText(`Inverted ${nLinks}-Pendulum on Cart with LQR Control`, dimensions.width / 2, 25);
    ctx.textAlign = 'left';
  }, [state, visualizationData, dimensions, linkMass, numLinks, linkLength, showTelemetry, showForceArrow, trackWidth, highlighted, resolveJoints, computeTransform]);

  return (
    <div
      ref={containerRef}
      style={{
        width: '100%',
        height: '100%',
        minHeight: '400px',
        position: 'relative',
      }}
    >
      <canvas
        ref={canvasRef}
        width={dimensions.width}
        height={dimensions.height}
        style={{
          display: 'block',
          width: '100%',
          height: '100%',
        }}
      />
    </div>
  );
}

export default InvertedPendulumVisualization;
