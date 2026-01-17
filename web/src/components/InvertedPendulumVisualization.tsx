import { useEffect, useRef, useState, useCallback } from 'react';

export interface PendulumState {
  time: number;
  x: number;       // Cart position
  theta1: number;  // Link 1 angle from vertical
  theta2: number;  // Link 2 angle from vertical
  xdot: number;    // Cart velocity
  omega1: number;  // Link 1 angular velocity
  omega2: number;  // Link 2 angular velocity
  controlForce: number;
}

export interface PendulumVisualizationData {
  cart: { x: number; y: number };
  joint1: { x: number; y: number };
  joint2: { x: number; y: number };
  tip: { x: number; y: number };
  theta1: number;
  theta2: number;
  controlForce: number;
}

interface InvertedPendulumVisualizationProps {
  state: PendulumState | null;
  visualizationData?: PendulumVisualizationData | null;
  cartMass?: number;
  mass1?: number;
  mass2?: number;
  length1?: number;
  length2?: number;
  showTelemetry?: boolean;
  showForceArrow?: boolean;
  trackWidth?: number;  // Total track width for cart travel
  onApplyImpulse?: (type: 'cart' | 'link1' | 'link2', impulse: number) => void;
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
  cartMass: _cartMass = 1.0,
  mass1 = 0.5,
  mass2 = 0.5,
  length1 = 0.7,
  length2 = 0.7,
  showTelemetry = true,
  showForceArrow = true,
  trackWidth = 3.0,
  onApplyImpulse,
}: InvertedPendulumVisualizationProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [dimensions, setDimensions] = useState({ width: 800, height: 600 });
  const [highlightedElement, setHighlightedElement] = useState<'cart' | 'link1' | 'link2' | null>(null);

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

  // Handle touch/click interaction
  const handleInteraction = useCallback((clientX: number, clientY: number) => {
    if (!canvasRef.current || !state || !onApplyImpulse) return;

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const canvasX = clientX - rect.left;
    const canvasY = clientY - rect.top;

    // Get current visualization positions
    let cart: { x: number; y: number };
    let joint2: { x: number; y: number };
    let tip: { x: number; y: number };

    if (visualizationData) {
      cart = visualizationData.cart;
      joint2 = visualizationData.joint2;
      tip = visualizationData.tip;
    } else {
      const x = state.x;
      const theta1 = state.theta1;
      const theta2 = state.theta2;
      cart = { x, y: 0 };
      joint2 = {
        x: x + length1 * Math.sin(theta1),
        y: length1 * Math.cos(theta1)
      };
      tip = {
        x: joint2.x + length2 * Math.sin(theta2),
        y: joint2.y + length2 * Math.cos(theta2)
      };
    }

    // Transform to canvas coordinates (same as rendering)
    const totalHeight = length1 + length2;
    const padding = 60;
    const viewWidth = Math.max(trackWidth, 2 * totalHeight);
    const viewHeight = totalHeight * 1.5;

    const scaleX = (dimensions.width - 2 * padding) / viewWidth;
    const scaleY = (dimensions.height - 2 * padding) / viewHeight;
    const scale = Math.min(scaleX, scaleY) * 0.8;

    const centerX = dimensions.width / 2;
    const groundY = dimensions.height * 0.7;

    const toCanvasX = (x: number) => centerX + x * scale;
    const toCanvasY = (y: number) => groundY - y * scale;

    const cartWidth = 0.3 * scale;
    const cartHeight = 0.15 * scale;
    const cartX = toCanvasX(cart.x);
    const cartY = groundY;
    const j2x = toCanvasX(joint2.x);
    const j2y = toCanvasY(joint2.y);
    const tipX = toCanvasX(tip.x);
    const tipY = toCanvasY(tip.y);

    const mass1Radius = 8 + mass1 * 10;
    const mass2Radius = 8 + mass2 * 10;

    // Check what was clicked
    let clickedElement: 'cart' | 'link1' | 'link2' | null = null;

    // Check mass 2 (tip)
    const distTip = Math.sqrt(Math.pow(canvasX - tipX, 2) + Math.pow(canvasY - tipY, 2));
    if (distTip < mass2Radius + 10) {
      clickedElement = 'link2';
    }

    // Check mass 1 (joint2)
    if (!clickedElement) {
      const distJoint2 = Math.sqrt(Math.pow(canvasX - j2x, 2) + Math.pow(canvasY - j2y, 2));
      if (distJoint2 < mass1Radius + 10) {
        clickedElement = 'link1';
      }
    }

    // Check cart
    if (!clickedElement) {
      if (canvasX >= cartX - cartWidth / 2 - 10 && canvasX <= cartX + cartWidth / 2 + 10 &&
          canvasY >= cartY - cartHeight - 10 && canvasY <= cartY + 10) {
        clickedElement = 'cart';
      }
    }

    if (clickedElement) {
      // Apply impulse based on clicked element
      const impulseStrength = clickedElement === 'cart' ? 5 : 0.5;
      onApplyImpulse(clickedElement, impulseStrength);

      // Visual feedback
      setHighlightedElement(clickedElement);
      setTimeout(() => setHighlightedElement(null), 200);
    }
  }, [state, visualizationData, dimensions, length1, length2, mass1, mass2, trackWidth, onApplyImpulse]);

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

    // Get visualization data or compute from state
    let cart: { x: number; y: number };
    let joint1: { x: number; y: number };
    let joint2: { x: number; y: number };
    let tip: { x: number; y: number };
    let theta1: number;
    let theta2: number;
    let controlForce: number;

    if (visualizationData) {
      cart = visualizationData.cart;
      joint1 = visualizationData.joint1;
      joint2 = visualizationData.joint2;
      tip = visualizationData.tip;
      theta1 = visualizationData.theta1;
      theta2 = visualizationData.theta2;
      controlForce = visualizationData.controlForce;
    } else if (state) {
      const x = state.x;
      theta1 = state.theta1;
      theta2 = state.theta2;
      controlForce = state.controlForce;

      cart = { x, y: 0 };
      joint1 = { x, y: 0 };
      joint2 = {
        x: x + length1 * Math.sin(theta1),
        y: length1 * Math.cos(theta1)
      };
      tip = {
        x: joint2.x + length2 * Math.sin(theta2),
        y: joint2.y + length2 * Math.cos(theta2)
      };
    } else {
      // Default state: upright at origin
      cart = { x: 0, y: 0 };
      joint1 = { x: 0, y: 0 };
      joint2 = { x: 0, y: length1 };
      tip = { x: 0, y: length1 + length2 };
      theta1 = 0;
      theta2 = 0;
      controlForce = 0;
    }

    // Scale and center the visualization
    const totalHeight = length1 + length2;
    const padding = 60;
    const viewWidth = Math.max(trackWidth, 2 * totalHeight);
    const viewHeight = totalHeight * 1.5;

    const scaleX = (dimensions.width - 2 * padding) / viewWidth;
    const scaleY = (dimensions.height - 2 * padding) / viewHeight;
    const scale = Math.min(scaleX, scaleY) * 0.8;

    // Center point on canvas (cart track is at vertical center-bottom area)
    const centerX = dimensions.width / 2;
    const groundY = dimensions.height * 0.7;

    // Transform from simulation to canvas coordinates
    const toCanvasX = (x: number) => centerX + x * scale;
    const toCanvasY = (y: number) => groundY - y * scale; // Flip Y axis

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
      const markerX = centerX + (i * trackWidth / 10) * scale;
      ctx.beginPath();
      ctx.arc(markerX, groundY + 5, 2, 0, Math.PI * 2);
      ctx.fill();
    }

    // Draw cart
    const cartWidth = 0.3 * scale;
    const cartHeight = 0.15 * scale;
    const cartX = toCanvasX(cart.x);
    const cartY = groundY;

    // Cart body (highlight if touched)
    if (highlightedElement === 'cart') {
      ctx.fillStyle = '#6dd5ff';
      ctx.shadowBlur = 15;
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
      const maxForceDisplay = 100; // N
      const maxArrowLength = 80; // pixels
      const arrowLength = (controlForce / maxForceDisplay) * maxArrowLength;

      ctx.strokeStyle = controlForce > 0 ? '#4ade80' : '#f87171';
      ctx.fillStyle = controlForce > 0 ? '#4ade80' : '#f87171';
      ctx.lineWidth = 3;

      const arrowY = cartY - cartHeight / 2;
      const arrowStartX = cartX;
      const arrowEndX = cartX + arrowLength;

      // Arrow line
      ctx.beginPath();
      ctx.moveTo(arrowStartX, arrowY);
      ctx.lineTo(arrowEndX, arrowY);
      ctx.stroke();

      // Arrow head
      const headSize = 8;
      const dir = controlForce > 0 ? 1 : -1;
      ctx.beginPath();
      ctx.moveTo(arrowEndX, arrowY);
      ctx.lineTo(arrowEndX - dir * headSize, arrowY - headSize / 2);
      ctx.lineTo(arrowEndX - dir * headSize, arrowY + headSize / 2);
      ctx.closePath();
      ctx.fill();
    }

    // Draw pendulum links
    const j1x = toCanvasX(joint1.x);
    const j1y = toCanvasY(joint1.y);
    const j2x = toCanvasX(joint2.x);
    const j2y = toCanvasY(joint2.y);
    const tipX = toCanvasX(tip.x);
    const tipY = toCanvasY(tip.y);

    // Link 1
    ctx.strokeStyle = getCSSVariable('--accent-primary') || '#6366f1';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(j1x, j1y - cartHeight);
    ctx.lineTo(j2x, j2y);
    ctx.stroke();

    // Link 2
    ctx.strokeStyle = getCSSVariable('--accent-tertiary') || '#8b5cf6';
    ctx.beginPath();
    ctx.moveTo(j2x, j2y);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();

    // Draw joints and masses
    // Joint 1 (cart pivot)
    ctx.fillStyle = getCSSVariable('--text-primary') || '#fff';
    ctx.beginPath();
    ctx.arc(j1x, j1y - cartHeight, 6, 0, Math.PI * 2);
    ctx.fill();

    // Mass 1 (joint 2) - highlight if touched
    const mass1Radius = 8 + mass1 * 10;
    if (highlightedElement === 'link1') {
      ctx.fillStyle = '#8b87ff';
      ctx.shadowBlur = 20;
      ctx.shadowColor = '#8b87ff';
    } else {
      ctx.fillStyle = getCSSVariable('--accent-primary') || '#6366f1';
    }
    ctx.beginPath();
    ctx.arc(j2x, j2y, mass1Radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Mass 2 (tip) - highlight if touched
    const mass2Radius = 8 + mass2 * 10;
    if (highlightedElement === 'link2') {
      ctx.fillStyle = '#b490ff';
      ctx.shadowBlur = 20;
      ctx.shadowColor = '#b490ff';
    } else {
      ctx.fillStyle = getCSSVariable('--accent-tertiary') || '#8b5cf6';
    }
    ctx.beginPath();
    ctx.arc(tipX, tipY, mass2Radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Draw telemetry overlay
    if (showTelemetry && state) {
      ctx.fillStyle = getCSSVariable('--text-primary') || '#fff';
      ctx.font = '14px monospace';

      const telemetryX = 20;
      const telemetryY = 30;
      const lineHeight = 20;

      ctx.fillText(`t = ${state.time.toFixed(2)} s`, telemetryX, telemetryY);
      ctx.fillText(`x = ${state.x.toFixed(3)} m`, telemetryX, telemetryY + lineHeight);
      ctx.fillText(`θ₁ = ${(state.theta1 * 180 / Math.PI).toFixed(1)}°`, telemetryX, telemetryY + 2 * lineHeight);
      ctx.fillText(`θ₂ = ${(state.theta2 * 180 / Math.PI).toFixed(1)}°`, telemetryX, telemetryY + 3 * lineHeight);
      ctx.fillText(`F = ${state.controlForce.toFixed(1)} N`, telemetryX, telemetryY + 4 * lineHeight);
    }

    // Draw title
    ctx.fillStyle = getCSSVariable('--text-secondary') || '#888';
    ctx.font = 'bold 16px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Inverted Double Pendulum with LQR Control', dimensions.width / 2, 25);
    ctx.textAlign = 'left';

  }, [state, visualizationData, dimensions, length1, length2, mass1, mass2, showTelemetry, showForceArrow, trackWidth, highlightedElement]);

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
