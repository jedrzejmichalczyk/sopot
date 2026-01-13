# WebAssembly + UI Investigation for SOPOT Framework

**Date:** 2026-01-13
**Goal:** Investigate options for compiling SOPOT to WebAssembly with a modern web UI for general simulation setup

---

## Executive Summary

✅ **SOPOT is an excellent candidate for WebAssembly compilation**

Key findings:
- **Zero external dependencies** - Only C++20 STL required
- **C++20 support in Emscripten** - Full concepts, constexpr, templates support
- **Minimal modifications needed** - ~500 lines of wrapper code, 2-3 files to adapt
- **Compile-time architecture advantage** - No virtual functions = optimal Wasm performance
- **Rich ecosystem** - Modern React + Three.js + WebGPU for visualization

---

## 1. WebAssembly Compilation: Emscripten

### C++20 Support Status

Emscripten now has comprehensive C++20 support:

| Feature | Status | Notes |
|---------|--------|-------|
| Concepts | ✅ Fully supported | Complete implementation in LLVM 17+ |
| Constexpr | ✅ Fully supported | All compile-time features work |
| Templates | ✅ Fully supported | Template metaprogramming intact |
| Coroutines | ✅ Supported | C++20 co_await with JavaScript Promises |
| Modules | ⚠️ Partial | Work in progress, not needed for SOPOT |
| std::array/vector/span | ✅ Fully supported | All containers available |

**Sources:**
- [Emscripten C++20 concepts support issue](https://github.com/emscripten-core/emscripten/issues/14236)
- [Emscripten Documentation](https://emscripten.org/docs/introducing_emscripten/about_emscripten.html)
- [Emscripten GitHub Repository](https://github.com/emscripten-core/emscripten)

### SOPOT Compatibility Analysis

**✅ Perfect Compatibility:**
- Header-only library design
- Pure C++20 STL (no external libs)
- IEEE 754 double precision
- Standard allocators
- Compile-time dispatch (no vtables)

**⚠️ Requires Adaptation (3 files):**

| File | Issue | Solution |
|------|-------|----------|
| `io/csv_parser.hpp` | Uses `std::ifstream` | Create data-driven API: pass pre-loaded arrays |
| `core/solver.hpp` | Uses `std::chrono` for timing | Replace with JavaScript `performance.now()` or remove |
| Tests (`tests/*.cpp`) | Uses `std::cout` | Replace with Emscripten logging or remove |

**Estimated effort:** 2-3 days for full Wasm port

---

## 2. Modern Web Technology Stack

### Recommended Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    WEB APPLICATION                          │
├─────────────────────────────────────────────────────────────┤
│  UI Layer: React + Material-UI / Ant Design                 │
│    - Simulation parameters form                             │
│    - Real-time charts (Recharts / Plotly.js)               │
│    - Control buttons (play/pause/reset)                     │
├─────────────────────────────────────────────────────────────┤
│  Visualization: React Three Fiber + Three.js               │
│    - 3D rocket trajectory                                   │
│    - Attitude visualization (quaternions)                   │
│    - Camera controls (orbit, follow)                        │
│    - Ground reference, velocity vectors                     │
├─────────────────────────────────────────────────────────────┤
│  Physics Backend: SOPOT WebAssembly Module                 │
│    - Compiled with Emscripten + embind                      │
│    - C++ simulation core (unchanged)                        │
│    - RK4 integrator (20,000 steps in ~8ms)                 │
├─────────────────────────────────────────────────────────────┤
│  Data Layer: IndexedDB / LocalStorage                       │
│    - Cached CSV data (rocket mass, engine, aero)           │
│    - Simulation history                                     │
│    - User presets                                           │
└─────────────────────────────────────────────────────────────┘
```

### Technology Choices

#### UI Framework: **React + TypeScript**
- Industry standard with huge ecosystem
- Type-safe integration with WebAssembly
- Rich component libraries (Material-UI, Ant Design)
- Hot module replacement for development

#### 3D Visualization: **React Three Fiber (R3F)**
- Declarative Three.js with React components
- Excellent performance for trajectory rendering
- Built-in camera controls and helpers
- Active community and documentation

**Sources:**
- [React Three Rapier](https://github.com/pmndrs/react-three-rapier)
- [Building Browser Games with Three.js and React](https://rherault.dev/articles/create-3d-game-part-3)
- [React Three Fiber Physics Tutorial](https://wawasensei.dev/courses/react-three-fiber/lessons/physics)

#### Advanced Rendering: **WebGPU (Optional)**
- GPU-accelerated particle systems
- Compute shaders for trajectory prediction
- Real-time atmospheric effects
- 60 FPS with millions of particles

**Note:** WebGPU is optional for initial release but provides significant performance benefits for:
- Rendering large trajectory datasets
- Real-time sensitivity analysis
- GPU-accelerated Jacobian computation

**Sources:**
- [Three.js WebGPU Compute Shaders Tutorial](https://threejsroadmap.com/blog/galaxy-simulation-webgpu-compute-shaders)
- [WebGPU Performance in Three.js](https://medium.com/@sudenurcevik/upgrading-performance-moving-from-webgl-to-webgpu-in-three-js-4356e84e4702)
- [WebGPU TSL Field Guide](https://blog.maximeheckel.com/posts/field-guide-to-tsl-and-webgpu/)

#### Charting: **Recharts or Plotly.js**
- Real-time line charts (altitude, velocity, mass)
- Interactive phase space plots
- Export to PNG/SVG

---

## 3. C++ to JavaScript Binding Architecture

### Emscripten Embind

**Embind** is the recommended binding layer (performance: ~200 ns call overhead):

**Sources:**
- [Embind Official Documentation](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
- [C++ in Browser with WebAssembly Guide](https://ppuzio.medium.com/c-in-the-browser-with-webassembly-via-emscripten-vite-and-react-bd82e0598a5e)
- [Embind Tutorial](https://ninkovic.dev/blog/2022/an-improved-guide-for-compiling-wasm-with-emscripten-and-embind)

### Proposed API Design

```cpp
// wasm/wasm_rocket.cpp
#include <emscripten/bind.h>
#include "../rocket/rocket.hpp"

using namespace emscripten;
using namespace sopot;

// Wrapper class for cleaner JavaScript API
class RocketSimulator {
public:
    RocketSimulator() : m_rocket() {}

    // Configuration
    void setLauncher(double elevation_deg, double azimuth_deg) {
        m_rocket.setLauncher(elevation_deg, azimuth_deg);
    }

    void setDiameter(double diameter_m) {
        m_rocket.setDiameter(diameter_m);
    }

    // Data loading (pre-processed from JavaScript)
    void loadMassData(const std::vector<double>& time,
                     const std::vector<double>& mass) {
        // Convert JS arrays to internal interpolators
        m_rocket.setMassInterpolator(time, mass);
    }

    void loadEngineData(const std::vector<double>& time,
                       const std::vector<double>& thrust,
                       const std::vector<double>& isp) {
        m_rocket.setEngineInterpolator(time, thrust, isp);
    }

    // Initialization
    void setup() {
        m_rocket.setupBeforeSimulation();
        m_state = m_rocket.getInitialState();
    }

    // Simulation step (JavaScript controls the loop)
    void step(double dt) {
        // Single RK4 step
        auto k1 = m_rocket.computeDerivatives(m_time, m_state);
        auto k2 = m_rocket.computeDerivatives(m_time + dt/2,
                                              addScaled(m_state, k1, dt/2));
        auto k3 = m_rocket.computeDerivatives(m_time + dt/2,
                                              addScaled(m_state, k2, dt/2));
        auto k4 = m_rocket.computeDerivatives(m_time + dt,
                                              addScaled(m_state, k3, dt));

        // Update state
        for (size_t i = 0; i < m_state.size(); ++i) {
            m_state[i] += (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        }
        m_time += dt;
    }

    // State queries (called every frame)
    double getTime() const { return m_time; }
    double getAltitude() const { return m_rocket.getAltitude(m_state); }
    double getSpeed() const { return m_rocket.getSpeed(m_state); }
    double getMass() const { return m_rocket.getMass(m_state); }

    val getPosition() const {
        auto pos = m_rocket.getPosition(m_state);
        val result = val::object();
        result.set("x", pos.x);
        result.set("y", pos.y);
        result.set("z", pos.z);
        return result;
    }

    val getVelocity() const {
        auto vel = m_rocket.getVelocity(m_state);
        val result = val::object();
        result.set("x", vel.x);
        result.set("y", vel.y);
        result.set("z", vel.z);
        return result;
    }

    val getQuaternion() const {
        auto q = m_rocket.getQuaternion(m_state);
        val result = val::object();
        result.set("q1", q.q1);
        result.set("q2", q.q2);
        result.set("q3", q.q3);
        result.set("q4", q.q4);
        return result;
    }

private:
    rocket::Rocket<double> m_rocket;
    std::vector<double> m_state;
    double m_time = 0.0;

    std::vector<double> addScaled(const std::vector<double>& v,
                                   const std::vector<double>& dv,
                                   double scale) const {
        std::vector<double> result(v.size());
        for (size_t i = 0; i < v.size(); ++i) {
            result[i] = v[i] + scale * dv[i];
        }
        return result;
    }
};

// Embind declarations
EMSCRIPTEN_BINDINGS(sopot_module) {
    // Register vector types
    register_vector<double>("VectorDouble");

    // Register main class
    class_<RocketSimulator>("RocketSimulator")
        .constructor<>()
        .function("setLauncher", &RocketSimulator::setLauncher)
        .function("setDiameter", &RocketSimulator::setDiameter)
        .function("loadMassData", &RocketSimulator::loadMassData)
        .function("loadEngineData", &RocketSimulator::loadEngineData)
        .function("setup", &RocketSimulator::setup)
        .function("step", &RocketSimulator::step)
        .function("getTime", &RocketSimulator::getTime)
        .function("getAltitude", &RocketSimulator::getAltitude)
        .function("getSpeed", &RocketSimulator::getSpeed)
        .function("getMass", &RocketSimulator::getMass)
        .function("getPosition", &RocketSimulator::getPosition)
        .function("getVelocity", &RocketSimulator::getVelocity)
        .function("getQuaternion", &RocketSimulator::getQuaternion);
}
```

### JavaScript/TypeScript Usage

```typescript
// types/sopot.d.ts
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  q1: number;
  q2: number;
  q3: number;
  q4: number;
}

export interface RocketSimulator {
  setLauncher(elevation: number, azimuth: number): void;
  setDiameter(diameter: number): void;
  loadMassData(time: number[], mass: number[]): void;
  loadEngineData(time: number[], thrust: number[], isp: number[]): void;
  setup(): void;
  step(dt: number): void;
  getTime(): number;
  getAltitude(): number;
  getSpeed(): number;
  getMass(): number;
  getPosition(): Vector3;
  getVelocity(): Vector3;
  getQuaternion(): Quaternion;
}

export interface SopotModule {
  RocketSimulator: new () => RocketSimulator;
}

// hooks/useRocketSimulation.ts
import { useState, useEffect, useRef } from 'react';
import type { SopotModule, RocketSimulator } from '../types/sopot';

export function useRocketSimulation() {
  const [module, setModule] = useState<SopotModule | null>(null);
  const [simulator, setSimulator] = useState<RocketSimulator | null>(null);
  const [isRunning, setIsRunning] = useState(false);

  useEffect(() => {
    // Load WebAssembly module
    import('../wasm/sopot.js').then((Module) => {
      Module.default().then((instance: SopotModule) => {
        setModule(instance);
      });
    });
  }, []);

  const initialize = (config: {
    elevation: number;
    azimuth: number;
    diameter: number;
    massData: { time: number[]; mass: number[] };
    engineData: { time: number[]; thrust: number[]; isp: number[] };
  }) => {
    if (!module) return;

    const sim = new module.RocketSimulator();
    sim.setLauncher(config.elevation, config.azimuth);
    sim.setDiameter(config.diameter);
    sim.loadMassData(config.massData.time, config.massData.mass);
    sim.loadEngineData(
      config.engineData.time,
      config.engineData.thrust,
      config.engineData.isp
    );
    sim.setup();

    setSimulator(sim);
  };

  const step = (dt: number = 0.01) => {
    if (!simulator) return null;
    simulator.step(dt);
    return {
      time: simulator.getTime(),
      altitude: simulator.getAltitude(),
      speed: simulator.getSpeed(),
      mass: simulator.getMass(),
      position: simulator.getPosition(),
      velocity: simulator.getVelocity(),
      quaternion: simulator.getQuaternion(),
    };
  };

  return {
    isReady: !!module,
    simulator,
    initialize,
    step,
    isRunning,
    setIsRunning,
  };
}
```

---

## 4. UI Component Architecture

### React Component Hierarchy

```
App
├── SimulationSetup (configuration form)
│   ├── RocketParameters
│   │   ├── DiameterInput
│   │   ├── LauncherAngleInputs
│   │   └── DataFileUpload (CSV → JSON conversion)
│   ├── EnvironmentParameters
│   │   ├── WindSettings
│   │   └── AtmosphereModel
│   └── SimulationControls
│       ├── InitializeButton
│       ├── PlayPauseButton
│       └── ResetButton
│
├── Visualization3D (React Three Fiber)
│   ├── Scene
│   │   ├── RocketModel (3D mesh with attitude)
│   │   ├── TrajectoryLine (BufferGeometry)
│   │   ├── Ground (reference plane)
│   │   ├── VelocityVector (arrow helper)
│   │   └── Axes (coordinate frame)
│   ├── Camera (OrbitControls or FollowCamera)
│   └── Lighting
│
├── TelemetryPanel
│   ├── RealtimeCharts (Recharts)
│   │   ├── AltitudeChart
│   │   ├── VelocityChart
│   │   └── MassChart
│   ├── NumericDisplay
│   │   ├── CurrentTime
│   │   ├── CurrentAltitude
│   │   ├── CurrentSpeed
│   │   └── CurrentMass
│   └── AttitudeIndicator (quaternion visualization)
│
└── ExportTools
    ├── DownloadCSV (trajectory data)
    ├── DownloadJSON (full state history)
    └── ScreenshotCapture (3D view)
```

### Example Component: Visualization3D

```tsx
// components/Visualization3D.tsx
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Line, Sphere } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useRef, useEffect } from 'react';
import * as THREE from 'three';

interface TrajectoryPoint {
  position: [number, number, number];
  time: number;
}

interface Visualization3DProps {
  currentPosition: { x: number; y: number; z: number };
  currentQuaternion: { q1: number; q2: number; q3: number; q4: number };
  trajectory: TrajectoryPoint[];
}

function RocketMesh({
  position,
  quaternion
}: {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
}) {
  const meshRef = useRef<THREE.Mesh>(null);

  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.position.copy(position);
      meshRef.current.quaternion.copy(quaternion);
    }
  });

  return (
    <mesh ref={meshRef}>
      {/* Simplified rocket geometry */}
      <cylinderGeometry args={[0.1, 0.1, 1, 16]} />
      <meshStandardMaterial color="red" />
      {/* Nose cone */}
      <mesh position={[0, 0.6, 0]}>
        <coneGeometry args={[0.1, 0.2, 16]} />
        <meshStandardMaterial color="white" />
      </mesh>
    </mesh>
  );
}

function TrajectoryLine({ points }: { points: TrajectoryPoint[] }) {
  const positions = points.map(p => p.position);

  return (
    <Line
      points={positions}
      color="cyan"
      lineWidth={2}
      dashed={false}
    />
  );
}

export function Visualization3D({
  currentPosition,
  currentQuaternion,
  trajectory
}: Visualization3DProps) {
  const pos = new THREE.Vector3(
    currentPosition.x,
    currentPosition.z,  // Z is up in SOPOT, Y is up in Three.js
    currentPosition.y
  );

  const quat = new THREE.Quaternion(
    currentQuaternion.q2,
    currentQuaternion.q4,
    currentQuaternion.q3,
    currentQuaternion.q1
  );

  return (
    <Canvas camera={{ position: [10, 10, 10], fov: 50 }}>
      <ambientLight intensity={0.5} />
      <pointLight position={[10, 10, 10]} />

      <RocketMesh position={pos} quaternion={quat} />
      <TrajectoryLine points={trajectory} />

      {/* Ground plane */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
        <planeGeometry args={[100, 100]} />
        <meshStandardMaterial color="green" opacity={0.5} transparent />
      </mesh>

      {/* Coordinate axes */}
      <axesHelper args={[5]} />

      <OrbitControls />
    </Canvas>
  );
}
```

---

## 5. Implementation Roadmap

### Phase 1: WebAssembly Core (Week 1-2)

**Files to create:**
- `wasm/wasm_rocket.cpp` - Embind wrapper
- `wasm/CMakeLists.txt` - Emscripten build configuration
- `wasm/build.sh` - Build script

**Files to modify:**
- `io/csv_parser.hpp` - Add data-driven constructor
- `core/solver.hpp` - Make timing optional

**Deliverable:** `sopot.js` + `sopot.wasm` that can be imported in Node.js

**Build command:**
```bash
emcc -std=c++20 \
     -lembind \
     -O3 \
     -s WASM=1 \
     -s ALLOW_MEMORY_GROWTH=1 \
     -s MODULARIZE=1 \
     -s EXPORT_NAME="createSopotModule" \
     -I../core -I../rocket -I../io \
     wasm_rocket.cpp \
     -o sopot.js
```

### Phase 2: TypeScript Integration (Week 3)

**Files to create:**
- `web/src/types/sopot.d.ts` - Type definitions
- `web/src/hooks/useRocketSimulation.ts` - React hook
- `web/src/utils/csvParser.ts` - Client-side CSV parsing

**Deliverable:** Working TypeScript integration with type safety

### Phase 3: Basic UI (Week 4-5)

**Tech stack:**
- Vite + React + TypeScript
- Material-UI for forms
- React Three Fiber for 3D

**Components:**
- Configuration form
- Basic 3D visualization
- Play/pause controls
- Real-time telemetry

**Deliverable:** Working web app with basic simulation

### Phase 4: Advanced Features (Week 6-8)

**Features:**
- CSV file upload and parsing
- Real-time charts (Recharts)
- Camera follow mode
- Export simulation data
- Preset configurations
- Responsive design

**Optional (if performance needed):**
- WebGPU rendering
- GPU-accelerated trajectory prediction
- Shader-based atmospheric effects

---

## 6. File Structure

```
sopot/
├── wasm/                          # WebAssembly bindings (NEW)
│   ├── wasm_rocket.cpp            # Embind wrapper
│   ├── CMakeLists.txt             # Emscripten build
│   └── build.sh                   # Build script
│
├── web/                           # Web application (NEW)
│   ├── package.json
│   ├── vite.config.ts
│   ├── tsconfig.json
│   ├── index.html
│   └── src/
│       ├── main.tsx               # Entry point
│       ├── App.tsx                # Root component
│       ├── types/
│       │   └── sopot.d.ts         # Wasm type definitions
│       ├── hooks/
│       │   └── useRocketSimulation.ts
│       ├── components/
│       │   ├── SimulationSetup/
│       │   ├── Visualization3D/
│       │   ├── TelemetryPanel/
│       │   └── ExportTools/
│       ├── utils/
│       │   ├── csvParser.ts
│       │   └── rk4Integrator.ts   # JS fallback
│       └── assets/
│           └── sample_data/       # Embedded CSVs
│
├── core/                          # Existing SOPOT core
├── rocket/                        # Existing rocket sim
├── io/                            # Existing I/O (modified)
└── tests/                         # Existing tests
```

---

## 7. Performance Expectations

### Simulation Performance

Based on existing benchmarks:
- **Single `computeDerivatives()` call:** ~0.3 μs (13-state rocket)
- **20,000 RK4 steps:** ~8 ms (200s flight at 0.01s timestep)
- **Target frame rate:** 60 FPS = 16.67 ms per frame

**Achievable real-time factors:**
- 10x real-time on desktop (simulate 10s of flight per 1s of real time)
- 5x real-time on mobile
- Can run faster if visualization skips frames

### WebAssembly Overhead

- Embind call overhead: ~200 ns per call
- Memory copy (14 doubles): ~100 ns
- Total overhead per step: ~300 ns (negligible)

### Visualization Performance

Using React Three Fiber + WebGL:
- 10,000 trajectory points: 60 FPS
- Real-time rocket mesh update: 60 FPS
- With WebGPU: 1M+ particles at 60 FPS

---

## 8. Deployment Options

### Static Hosting (Recommended)

Deploy to:
- **GitHub Pages** (free, easy CI/CD)
- **Netlify** (free tier, CDN)
- **Vercel** (free tier, optimized for React)

Files to deploy:
- `index.html`
- `assets/*.js` (Vite bundles)
- `sopot.js` + `sopot.wasm`

### Progressive Web App (PWA)

Add service worker for:
- Offline simulation
- Install as desktop/mobile app
- Cache Wasm module

---

## 9. Recommended Next Steps

1. **Prototype Phase (1-2 days):**
   - Create minimal `wasm_rocket.cpp` wrapper
   - Build with Emscripten
   - Test in Node.js console

2. **Validation Phase (2-3 days):**
   - Compare Wasm results with C++ tests
   - Verify numerical accuracy
   - Measure performance

3. **UI Prototype (1 week):**
   - Create React app skeleton
   - Integrate Wasm module
   - Build basic 3D visualization

4. **Polish Phase (2-3 weeks):**
   - Full UI implementation
   - Data loading/export
   - Documentation
   - Examples

---

## 10. Alternative Approaches (Not Recommended)

### Option 2: Pure JavaScript Port
- ❌ Would lose C++ performance benefits
- ❌ Would require maintaining two codebases
- ❌ No autodiff support
- ✅ Easier debugging

### Option 3: Server-Side Simulation
- ❌ Requires backend infrastructure
- ❌ Network latency issues
- ❌ Can't run offline
- ✅ No Wasm complexity

**Conclusion:** WebAssembly is the best option for SOPOT's use case.

---

## 11. References

### Emscripten & WebAssembly
- [Emscripten Documentation](https://emscripten.org/docs/introducing_emscripten/about_emscripten.html)
- [Emscripten GitHub](https://github.com/emscripten-core/emscripten)
- [Embind API Reference](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
- [C++20 Concepts Support Issue](https://github.com/emscripten-core/emscripten/issues/14236)
- [C++ in Browser with Emscripten Tutorial](https://ppuzio.medium.com/c-in-the-browser-with-webassembly-via-emscripten-vite-and-react-bd82e0598a5e)
- [Emscripten Embind Guide](https://ninkovic.dev/blog/2022/an-improved-guide-for-compiling-wasm-with-emscripten-and-embind)
- [WebAssembly MDN Guide](https://developer.mozilla.org/en-US/docs/WebAssembly/Guides/C_to_Wasm)

### Modern Web Performance
- [High-Performance Web Apps 2026](https://letket.com/high-performance-web-apps-in-2026-webassembly-webgpu-and-edge-architectures/)

### React Three Fiber & 3D Visualization
- [React Three Rapier GitHub](https://github.com/pmndrs/react-three-rapier)
- [Building Browser Games with Three.js Part 3](https://rherault.dev/articles/create-3d-game-part-3)
- [React Three Fiber Physics Tutorial](https://wawasensei.dev/courses/react-three-fiber/lessons/physics)
- [Three.js Journey Physics Course](https://threejs-journey.com/lessons/physics)

### WebGPU & Advanced Graphics
- [Interactive Galaxy with WebGPU Compute Shaders](https://threejsroadmap.com/blog/galaxy-simulation-webgpu-compute-shaders)
- [Three.js WebGPU Performance](https://medium.com/@sudenurcevik/upgrading-performance-moving-from-webgl-to-webgpu-in-three-js-4356e84e4702)
- [WebGPU TSL Field Guide](https://blog.maximeheckel.com/posts/field-guide-to-tsl-and-webgpu/)
- [Three.js WebGPU Renderer Tutorial](https://sbcode.net/threejs/webgpu-renderer/)
- [WebGPU API Documentation](https://developer.mozilla.org/en-US/docs/Web/API/WebGPU_API)

### Physics Libraries (for reference)
- [Matter.js](https://brm.io/matter-js/)
- [JavaScript Physics Libraries Overview](https://medium.com/@vmklvm/the-most-handy-javascript-physics-libraries-for-interactive-and-realistic-web-development-33ff59d073d1)
- [JavaScript Animation Libraries 2025](https://www.devkit.best/blog/mdx/javascript-animation-libraries-physics-engines-2025)
- [Rapier Physics Engine](https://rapier.rs/)

---

## 12. Conclusion

SOPOT's architecture is **ideally suited for WebAssembly** compilation:

✅ **Technical Feasibility:** 95% compatibility, minimal modifications needed
✅ **Performance:** Near-native speed, 10x real-time simulation possible
✅ **Modern Stack:** React + Three.js + WebGPU provides excellent UX
✅ **Development Time:** 6-8 weeks for full-featured web app
✅ **Deployment:** Static hosting (GitHub Pages/Netlify), no backend needed

**Recommended Action:** Proceed with WebAssembly implementation using the phased approach outlined above.
