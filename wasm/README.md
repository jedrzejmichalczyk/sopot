# SOPOT WebAssembly Module

This directory contains the WebAssembly build of the SOPOT rocket simulation framework, enabling C++20 physics simulation to run in web browsers.

## 🎯 Features

- **Zero runtime overhead**: Compile-time C++20 design compiles to optimal WebAssembly
- **Full simulation API**: Access to all SOPOT rocket simulation features
- **JavaScript-friendly**: Clean embind bindings with object-based API
- **Modern ES6**: Exports ES6 module compatible with React, Vue, etc.
- **Type-safe**: Easy TypeScript integration

## 📋 Prerequisites

### Emscripten Installation

Install the Emscripten SDK:

```bash
# Clone the emsdk repository
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# Download and install the latest SDK tools
./emsdk install latest

# Activate the SDK for current user
./emsdk activate latest

# Activate PATH for current terminal
source ./emsdk_env.sh
```

Verify installation:

```bash
emcc --version
# Should show: emscripten X.Y.Z (with LLVM X.Y.Z)
```

## 🔨 Building

### Quick Build (Recommended)

```bash
cd wasm
./build.sh
```

This produces:
- `sopot.js` - JavaScript loader/glue code
- `sopot.wasm` - WebAssembly binary module

### Build Options

```bash
# Release build (optimized, default)
./build.sh Release

# Debug build (with assertions and symbols)
./build.sh Debug

# Use CMake instead of direct emcc
./build.sh Release cmake
```

### Manual Build

```bash
emcc -std=c++20 \
    -O3 \
    -lembind \
    -s WASM=1 \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s MODULARIZE=1 \
    -s 'EXPORT_NAME="createSopotModule"' \
    -s EXPORT_ES6=1 \
    -s ENVIRONMENT=web,worker \
    -s NO_DISABLE_EXCEPTION_CATCHING \
    -fexceptions \
    -I.. \
    wasm_rocket.cpp \
    -o sopot.js
```

## 🚀 Usage

### JavaScript/ES6

```javascript
import createSopotModule from './sopot.js';

// Initialize the WebAssembly module
const Module = await createSopotModule();

// Create simulator instance
const sim = new Module.RocketSimulator();

// Configure
sim.setLauncher(85, 0);        // elevation=85°, azimuth=0°
sim.setDiameter(0.16);         // 16cm diameter
sim.setTimestep(0.01);         // 10ms timestep

// Load data (prototype - requires file serving)
sim.loadMassDataFromPath('../data/');
sim.loadEngineDataFromPath('../data/');
sim.loadAeroDataFromPath('../data/');

// Initialize simulation system
sim.setup();

// Run simulation
while (sim.step()) {
    console.log(`t=${sim.getTime().toFixed(2)}s, alt=${sim.getAltitude().toFixed(1)}m`);
}

console.log('Simulation complete!');
```

### TypeScript

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
  // Configuration
  setLauncher(elevation: number, azimuth: number): void;
  setDiameter(diameter: number): void;
  setTimestep(dt: number): void;

  // Data loading
  loadMassDataFromPath(path: string): void;
  loadEngineDataFromPath(path: string): void;
  loadAeroDataFromPath(path: string): void;
  loadDampingDataFromPath(path: string): void;

  // Initialization
  setup(): void;
  reset(): void;

  // Simulation
  step(): boolean;
  stepWithDt(dt: number): boolean;

  // Scalar queries
  getTime(): number;
  getAltitude(): number;
  getSpeed(): number;
  getMass(): number;
  getBurnTime(): number;

  // Vector queries
  getPosition(): Vector3;
  getVelocity(): Vector3;
  getQuaternion(): Quaternion;
  getFullState(): any;

  // Utility
  getStateDimension(): number;
  isInitialized(): boolean;
}

export interface SopotModule {
  RocketSimulator: new () => RocketSimulator;
}

// Usage
import createSopotModule from './sopot.js';

const Module: SopotModule = await createSopotModule();
const sim = new Module.RocketSimulator();
```

### React Hook Example

```typescript
// hooks/useRocketSimulation.ts
import { useState, useEffect, useCallback } from 'react';
import type { SopotModule, RocketSimulator } from '../types/sopot';

export function useRocketSimulation() {
  const [module, setModule] = useState<SopotModule | null>(null);
  const [simulator, setSimulator] = useState<RocketSimulator | null>(null);
  const [isRunning, setIsRunning] = useState(false);

  // Load WebAssembly module
  useEffect(() => {
    import('../wasm/sopot.js').then((createModule) => {
      createModule.default().then((instance: SopotModule) => {
        setModule(instance);
      });
    });
  }, []);

  const initialize = useCallback((config: {
    elevation: number;
    azimuth: number;
    diameter: number;
    dataPath: string;
  }) => {
    if (!module) return;

    const sim = new module.RocketSimulator();
    sim.setLauncher(config.elevation, config.azimuth);
    sim.setDiameter(config.diameter);
    sim.loadMassDataFromPath(config.dataPath);
    sim.loadEngineDataFromPath(config.dataPath);
    sim.loadAeroDataFromPath(config.dataPath);
    sim.setup();

    setSimulator(sim);
  }, [module]);

  const step = useCallback(() => {
    if (!simulator) return null;

    const shouldContinue = simulator.step();

    return {
      time: simulator.getTime(),
      altitude: simulator.getAltitude(),
      speed: simulator.getSpeed(),
      mass: simulator.getMass(),
      position: simulator.getPosition(),
      velocity: simulator.getVelocity(),
      quaternion: simulator.getQuaternion(),
      shouldContinue
    };
  }, [simulator]);

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

## 📊 API Reference

### RocketSimulator Class

#### Configuration Methods

```cpp
void setLauncher(double elevation_deg, double azimuth_deg)
```
Set launcher angles. Elevation: 0=horizontal, 90=vertical. Azimuth: 0=North, 90=East.

```cpp
void setDiameter(double diameter_m)
```
Set rocket reference diameter in meters.

```cpp
void setTimestep(double dt)
```
Set integration timestep in seconds (default: 0.01).

#### Data Loading (Prototype)

```cpp
void loadMassDataFromPath(const std::string& path)
void loadEngineDataFromPath(const std::string& path)
void loadAeroDataFromPath(const std::string& path)
void loadDampingDataFromPath(const std::string& path)
```

Load simulation data from CSV files. Path should end with `/`.

**Note**: These methods require files to be served via an HTTP server, as browsers restrict local file access via the `file://` protocol for security reasons. The WebAssembly module can only load files that are accessible through HTTP/HTTPS. In Phase 2, we'll add JavaScript array-based loading that doesn't require file I/O.

#### Initialization

```cpp
void setup()
```
Initialize the simulation system. Must be called after configuration.

```cpp
void reset()
```
Reset simulation to initial conditions.

#### Simulation Control

```cpp
bool step()
```
Advance simulation by one timestep. Returns `false` when rocket lands (altitude < 0).

```cpp
bool stepWithDt(double dt)
```
Advance simulation with custom timestep.

#### State Queries

**Scalars:**
```cpp
double getTime()        // Current simulation time [s]
double getAltitude()    // Altitude above ground [m]
double getSpeed()       // Total velocity magnitude [m/s]
double getMass()        // Current rocket mass [kg]
double getBurnTime()    // Engine burn duration [s]
```

**Vectors (return JavaScript objects):**
```cpp
{x, y, z} getPosition()    // Position in ENU frame [m]
{x, y, z} getVelocity()    // Velocity in ENU frame [m/s]
{q1, q2, q3, q4} getQuaternion()  // Attitude quaternion (body→ENU)
```

**Full State:**
```cpp
object getFullState()   // All telemetry in one object
```

Returns:
```javascript
{
  time: number,
  position: {x, y, z},
  velocity: {x, y, z},
  quaternion: {q1, q2, q3, q4},
  altitude: number,
  speed: number,
  mass: number
}
```

#### Utility

```cpp
size_t getStateDimension()  // Get state vector size (14)
bool isInitialized()        // Check if setup() was called
```

## 🧪 Testing

### Browser Test

1. Start a local HTTP server:
```bash
# Python 3
python3 -m http.server 8000

# Node.js
npx http-server -p 8000
```

2. Open `http://localhost:8000/wasm/test.html` in your browser

3. Configure and run the simulation

### Console Test (Node.js)

```bash
node --experimental-modules test.js
```

## 📁 File Structure

```
wasm/
├── wasm_rocket.cpp      # C++ embind wrapper
├── CMakeLists.txt       # CMake build configuration
├── build.sh             # Build script
├── test.html            # Browser test application
├── README.md            # This file
├── sopot.js             # Generated: JS glue code
└── sopot.wasm           # Generated: WebAssembly binary
```

## 🔧 Troubleshooting

### Build Errors

**"emcc: command not found"**
- Solution: Install Emscripten and run `source /path/to/emsdk/emsdk_env.sh`

**"cannot open file: ../rocket/rocket.hpp"**
- Solution: Ensure you're building from the `wasm/` directory

**"undefined symbol: std::__throw_bad_function_call"**
- Solution: Add `-fexceptions` and `-s NO_DISABLE_EXCEPTION_CATCHING` flags

### Runtime Errors

**"Cannot read file: ../data/sim_mass.csv"**
- Solution: Start an HTTP server. Browsers can't access local files directly.
- Alternative: Use the upcoming array-based data loading API (Phase 2)

**Memory errors**
- Solution: Increase Wasm memory with `-s INITIAL_MEMORY=64MB` flag

**Performance issues**
- Solution: Build with `-O3` optimization flag (Release mode)

## 🎯 Current Limitations (Phase 1 Prototype)

1. **File-based data loading**: Requires HTTP server. Phase 2 will add JavaScript array-based API.
2. **No CSV parsing in browser**: Need to pre-convert CSV to JSON for full web deployment.
3. **Timing uses std::chrono**: Not critical for Wasm, but could be replaced with JavaScript timing.

## 🚀 Next Steps (Phase 2)

- [ ] Add array-based data loading API (no file I/O)
- [ ] Create TypeScript npm package
- [ ] Add React Three Fiber visualization example
- [ ] Implement CSV to JSON converter
- [ ] Add WebGPU trajectory rendering
- [ ] Create full web application

## 📚 Resources

- [Emscripten Documentation](https://emscripten.org/docs/)
- [Embind Reference](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
- [WebAssembly.org](https://webassembly.org/)
- [SOPOT Documentation](../CLAUDE.md)

## 📝 Performance Notes

Based on native benchmarks:
- Single `computeDerivatives()` call: ~0.3 μs
- 20,000 RK4 steps: ~8 ms
- Embind call overhead: ~200 ns

Expected Wasm performance:
- **70-80% of native speed** for compute-intensive code
- **Target**: 10x real-time simulation (simulate 10s per 1s wall time)
- **Achievable**: 60 FPS visualization with real-time physics

## 📄 License

Same as SOPOT framework.

---

**Status**: Phase 1 Prototype Complete ✅

Built with ❤️ using C++20, Emscripten, and embind.
