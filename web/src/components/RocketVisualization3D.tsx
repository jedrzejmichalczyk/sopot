import { useRef, useMemo, useEffect } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Line, Cone, Cylinder } from '@react-three/drei';
import * as THREE from 'three';
import type { SimulationState } from '../types/sopot';

interface RocketVisualization3DProps {
  state: SimulationState | null;
  trajectoryHistory: Array<{ position: THREE.Vector3; time: number }>;
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
    // SOPOT quaternion is body to ENU, Three.js expects x=right, y=up, z=back
    const q = state.quaternion;
    groupRef.current.quaternion.set(q.q2, q.q4, -q.q3, q.q1);

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
      {/* Rocket body - metallic red cylinder */}
      <Cylinder args={[0.08, 0.08, 1.0, 16]} rotation={[Math.PI / 2, 0, 0]}>
        <meshStandardMaterial color="#cc0000" metalness={0.7} roughness={0.3} />
      </Cylinder>

      {/* Nose cone - white tip */}
      <Cone
        args={[0.08, 0.25, 16]}
        position={[0.625, 0, 0]}
        rotation={[0, 0, -Math.PI / 2]}
      >
        <meshStandardMaterial color="#ffffff" metalness={0.5} roughness={0.5} />
      </Cone>

      {/* Fins (4 fins at 90° intervals) */}
      {[0, 90, 180, 270].map((angle, i) => {
        const rad = (angle * Math.PI) / 180;
        return (
          <mesh
            key={i}
            position={[-0.4, Math.cos(rad) * 0.12, Math.sin(rad) * 0.12]}
            rotation={[0, rad, 0]}
          >
            <boxGeometry args={[0.15, 0.02, 0.15]} />
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
}: RocketVisualization3DProps) {
  return (
    <div style={{ width: '100%', height: '100%', background: '#87CEEB' }}>
      <Canvas
        camera={{ position: [20, 15, 20], fov: 50 }}
        shadows
        gl={{ antialias: true }}
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

        {/* Camera controls */}
        <OrbitControls
          enablePan={true}
          enableZoom={true}
          enableRotate={true}
          minDistance={5}
          maxDistance={500}
          maxPolarAngle={Math.PI / 2}
        />
      </Canvas>
    </div>
  );
}
