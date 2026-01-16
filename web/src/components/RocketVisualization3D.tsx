import { useRef, useMemo, useEffect } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Line, Cone, Cylinder } from '@react-three/drei';
import * as THREE from 'three';
import type { SimulationState } from '../types/sopot';

interface RocketVisualization3DProps {
  state: SimulationState | null;
  trajectoryHistory: Array<{ position: THREE.Vector3; time: number }>;
  cameraTracking?: boolean;
}

/**
 * Rocket mesh component with proper orientation - moves as one cohesive piece
 */
function RocketMesh({ state }: { state: SimulationState | null }) {
  const groupRef = useRef<THREE.Group>(null);
  const velocityArrowRef = useRef<THREE.ArrowHelper | null>(null);

  // Create arrow helper once using useEffect to prevent memory leaks
  useEffect(() => {
    if (!velocityArrowRef.current) {
      velocityArrowRef.current = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0), // Initial direction
        new THREE.Vector3(0, 0, 0), // Origin
        1, // Initial length
        0x00ff00, // Green color
        0.3, // Head length
        0.15 // Head width
      );
      velocityArrowRef.current.visible = false;
    }

    // Cleanup arrow on unmount
    return () => {
      if (velocityArrowRef.current) {
        velocityArrowRef.current.dispose?.();
        velocityArrowRef.current = null;
      }
    };
  }, []);

  useFrame(() => {
    if (!groupRef.current || !state) return;

    // Update position (ENU to Three.js: x=East, y=Up, z=-North)
    groupRef.current.position.set(
      state.position.x,
      state.position.z, // Z is up in SOPOT
      -state.position.y // Y is North in SOPOT, -Z in Three.js
    );

    // Update orientation from quaternion
    // SOPOT quaternion convention: (q1, q2, q3, q4) where q4 is scalar, q1-q3 are vector
    // SOPOT: body X points forward (thrust), quaternion transforms body->ENU
    // Mesh: built to point along +Y (up by default)
    //
    // Coordinate transformation ENU -> Three.js:
    //   Three.js X = ENU X (East)
    //   Three.js Y = ENU Z (Up)
    //   Three.js Z = -ENU Y (South)
    //
    // Steps:
    // 1. Transform SOPOT quaternion from ENU to Three.js coords
    // 2. Compose with rotation that maps mesh +Y to body +X
    const q = state.quaternion;

    // Transform quaternion components from ENU to Three.js frame
    const q_threejs = new THREE.Quaternion(
      q.q1,   // x component stays (East axis)
      q.q3,   // y component = old z (Up axis)
      -q.q2,  // z component = -old y (South axis)
      q.q4    // w (scalar) stays
    );

    // Pre-rotation: mesh +Y to body +X (rotate -90° around Z axis)
    // This accounts for mesh pointing along +Y while body X is forward
    const meshToBody = new THREE.Quaternion();
    meshToBody.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);

    // Compose: first rotate mesh to align with body, then apply physics orientation
    groupRef.current.quaternion.copy(q_threejs).multiply(meshToBody);

    // Update velocity arrow direction and length
    if (velocityArrowRef.current && state.speed > 0.1) {
      // Velocity in ENU frame, convert to Three.js frame
      const velDirection = new THREE.Vector3(
        state.velocity.x,
        state.velocity.z,
        -state.velocity.y
      ).normalize();

      const arrowLength = Math.min(state.speed * 0.05, 5);
      velocityArrowRef.current.setDirection(velDirection);
      velocityArrowRef.current.setLength(arrowLength, 0.3, 0.15);
      velocityArrowRef.current.visible = true;
    } else if (velocityArrowRef.current) {
      velocityArrowRef.current.visible = false;
    }
  });

  return (
    <group ref={groupRef}>
      {/* Rocket mesh built to point along +Y by default (up) */}
      {/* Body X in SOPOT = forward/thrust direction */}
      {/* When vertical: body X points ENU up (+Z) which becomes Three.js +Y */}

      {/* Rocket body - metallic red cylinder pointing up */}
      <Cylinder args={[0.08, 0.08, 1.0, 16]}>
        <meshStandardMaterial color="#cc0000" metalness={0.7} roughness={0.3} />
      </Cylinder>

      {/* Nose cone - white tip at +Y */}
      <Cone args={[0.08, 0.25, 16]} position={[0, 0.625, 0]}>
        <meshStandardMaterial color="#ffffff" metalness={0.5} roughness={0.5} />
      </Cone>

      {/* Fins at -Y (base of rocket), positioned around the body */}
      {[0, 90, 180, 270].map((angle, i) => {
        const rad = (angle * Math.PI) / 180;
        return (
          <mesh
            key={i}
            position={[Math.cos(rad) * 0.12, -0.4, Math.sin(rad) * 0.12]}
            rotation={[0, 0, rad]}
          >
            <boxGeometry args={[0.15, 0.15, 0.02]} />
            <meshStandardMaterial color="#333333" metalness={0.8} roughness={0.2} />
          </mesh>
        );
      })}

      {/* Velocity vector (green arrow) - positioned at rocket center */}
      {velocityArrowRef.current && <primitive object={velocityArrowRef.current} />}
    </group>
  );
}

/**
 * Camera controller for tracking rocket
 */
function CameraController({
  target,
  enabled,
}: {
  target: THREE.Vector3 | null;
  enabled: boolean;
}) {
  const controlsRef = useRef<any>(null);

  useFrame(() => {
    if (controlsRef.current && enabled && target) {
      // Smoothly move camera target to rocket position
      controlsRef.current.target.lerp(target, 0.1);
      controlsRef.current.update();
    }
  });

  return (
    <OrbitControls
      ref={controlsRef}
      enablePan={true}
      enableZoom={true}
      enableRotate={true}
      minDistance={5}
      maxDistance={500}
      maxPolarAngle={Math.PI / 2}
      enableDamping={true}
      dampingFactor={0.05}
      rotateSpeed={0.8}
      zoomSpeed={1.0}
      panSpeed={0.8}
      touches={{
        ONE: 0, // TOUCH.ROTATE
        TWO: 2, // TOUCH.DOLLY_PAN
      }}
    />
  );
}

/**
 * Trajectory line showing path history
 */
function TrajectoryLine({
  trajectoryHistory,
}: {
  trajectoryHistory: Array<{ position: THREE.Vector3 }>;
}) {
  const points = useMemo(() => {
    return trajectoryHistory.map((p) => p.position);
  }, [trajectoryHistory]);

  if (points.length < 2) return null;

  return <Line points={points} color="cyan" lineWidth={2} />;
}

/**
 * Ground plane with grid
 */
function GroundPlane() {
  return (
    <>
      {/* Ground mesh */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
        <planeGeometry args={[1000, 1000]} />
        <meshStandardMaterial color="#2d5016" opacity={0.8} transparent />
      </mesh>

      {/* Grid helper */}
      <Grid
        args={[100, 100]}
        cellSize={5}
        cellThickness={0.5}
        cellColor="#6f6f6f"
        sectionSize={25}
        sectionThickness={1}
        sectionColor="#8f8f8f"
        fadeDistance={400}
        fadeStrength={1}
        followCamera={false}
        infiniteGrid
      />
    </>
  );
}

/**
 * Launch pad marker
 */
function LaunchPad() {
  return (
    <group position={[0, 0.1, 0]}>
      {/* Platform */}
      <Cylinder args={[2, 2, 0.2, 32]} position={[0, 0, 0]}>
        <meshStandardMaterial color="#555555" metalness={0.9} roughness={0.1} />
      </Cylinder>

      {/* Support structure */}
      <Cylinder args={[0.1, 0.1, 3, 8]} position={[0, 1.5, 0]}>
        <meshStandardMaterial color="#ff6600" metalness={0.8} roughness={0.3} />
      </Cylinder>
    </group>
  );
}

/**
 * Scene lighting
 */
function SceneLighting() {
  return (
    <>
      {/* Ambient light */}
      <ambientLight intensity={0.4} />

      {/* Main directional light (sun) */}
      <directionalLight
        position={[50, 100, 50]}
        intensity={1.0}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-camera-left={-50}
        shadow-camera-right={50}
        shadow-camera-top={50}
        shadow-camera-bottom={-50}
      />

      {/* Fill light */}
      <directionalLight position={[-30, 20, -30]} intensity={0.3} />

      {/* Hemisphere light for sky */}
      <hemisphereLight args={['#87CEEB', '#2d5016', 0.5]} />
    </>
  );
}

/**
 * Coordinate axes helper
 */
function CoordinateAxes() {
  return (
    <group>
      {/* X axis (East) - Red */}
      <Line points={[[0, 0, 0], [10, 0, 0]]} color="red" lineWidth={2} />
      {/* Y axis (Up) - Green */}
      <Line points={[[0, 0, 0], [0, 10, 0]]} color="green" lineWidth={2} />
      {/* Z axis (North) - Blue */}
      <Line points={[[0, 0, 0], [0, 0, 10]]} color="blue" lineWidth={2} />
    </group>
  );
}

/**
 * Main 3D visualization component
 */
export function RocketVisualization3D({
  state,
  trajectoryHistory,
  cameraTracking = false,
}: RocketVisualization3DProps) {
  // Compute camera target from rocket position
  const cameraTarget = useMemo(() => {
    if (!state) return null;
    return new THREE.Vector3(
      state.position.x,
      state.position.z,
      -state.position.y
    );
  }, [state]);

  return (
    <div style={{ width: '100%', height: '100%', background: '#87CEEB' }}>
      <Canvas
        camera={{ position: [20, 15, 20], fov: 50 }}
        shadows
        gl={{ antialias: true }}
        className="visualization-canvas"
      >
        {/* Lighting */}
        <SceneLighting />

        {/* Scene elements */}
        <GroundPlane />
        <LaunchPad />
        <CoordinateAxes />

        {/* Rocket and trajectory */}
        <RocketMesh state={state} />
        <TrajectoryLine trajectoryHistory={trajectoryHistory} />

        {/* Camera controls with optional tracking */}
        <CameraController target={cameraTarget} enabled={cameraTracking} />
      </Canvas>
    </div>
  );
}
