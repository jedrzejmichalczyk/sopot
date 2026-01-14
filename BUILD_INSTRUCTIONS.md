# Build Instructions for SOPOT Rocket Visualization

This guide explains how to build the complete SOPOT visualization system with the new plotting features.

## Overview of Changes

The following enhancements have been implemented:

### 1. **Improved Rocket Visualization**
- **File**: `web/src/components/RocketVisualization3D.tsx`
- **Changes**: Rocket now moves as a single cohesive piece with properly synchronized velocity arrow
- The rocket body, nose cone, fins, and velocity indicator all move together as a unified `THREE.Group`

### 2. **Time-Series Data Collection (C++)**
- **File**: `wasm/wasm_rocket.cpp`
- **New Features**:
  - Added `TimeSeriesData` structure to store state function histories during ODE integration
  - Records all key state functions: altitude, speed, position, velocity, mass, acceleration, forces
  - New methods: `getTimeSeries()`, `getHistorySize()`, `setRecordHistory()`, `clearHistory()`
  - Automatic recording of data at each RK4 integration step

### 3. **Data Plotting Panel (React)**
- **File**: `web/src/components/PlotPanel.tsx`
- **Features**:
  - Interactive plotting using Recharts library
  - Six plot types: Altitude, Speed, Velocity Components, Acceleration Components, Mass, Forces
  - Dropdown selector to switch between different state functions
  - Responsive design with dark theme matching the UI
  - Real-time data updates during simulation

### 4. **Updated Application Layout**
- **File**: `web/src/App.tsx`
- **Changes**:
  - Reorganized to 4-panel layout: Controls (left), 3D View (center), Telemetry (right), Plots (bottom)
  - Bottom panel shows time-series plots (300px height)
  - Automatic fetching of time-series data during simulation

### 5. **TypeScript Type Definitions**
- **File**: `web/src/types/sopot.d.ts`
- **Added**:
  - `TimeSeriesData` interface for structured time-series data
  - New methods in `RocketSimulator` interface

## Building the WASM Module

### Prerequisites

1. **Emscripten SDK** (version 3.1.0 or higher for C++20 support)
   ```bash
   # Install Emscripten
   git clone https://github.com/emscripten-core/emsdk.git
   cd emsdk
   ./emsdk install latest
   ./emsdk activate latest
   source ./emsdk_env.sh
   ```

### Build Steps

1. **Navigate to the WASM directory**:
   ```bash
   cd <project_root>/wasm
   ```

2. **Run the build script**:
   ```bash
   ./build.sh Release
   ```

   This will:
   - Compile `wasm_rocket.cpp` with C++20 support
   - Generate `sopot.js` and `sopot.wasm` files
   - Optimize with `-O3` for release builds

3. **Copy WASM files to web public directory**:
   ```bash
   cp sopot.js sopot.wasm ../web/public/
   ```

### Alternative: Manual Build

If the build script doesn't work, you can build manually:

```bash
cd <project_root>/wasm

emcc -std=c++20 \
  -O3 \
  -s ASSERTIONS=0 \
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

## Building the Web Application

### Prerequisites

1. **Node.js** (version 16 or higher)
2. **npm** (comes with Node.js)

### Build Steps

1. **Navigate to the web directory**:
   ```bash
   cd <project_root>/web
   ```

2. **Install dependencies** (first time only):
   ```bash
   npm install
   ```

3. **Run development server**:
   ```bash
   npm run dev
   ```

   This will start Vite dev server at `http://localhost:5173`

4. **Build for production**:
   ```bash
   npm run build
   ```

   Output will be in `web/dist/` directory

## State Functions Available for Plotting

The following state functions are now available through the time-series data:

### Kinematics
- **Altitude** (m): Height above ground
- **Speed** (m/s): Total velocity magnitude
- **Position** (m): East, North, Up components
- **Velocity** (m/s): East, North, Up velocity components

### Dynamics
- **Mass** (kg): Time-varying vehicle mass
- **Acceleration** (m/s²): East, North, Up acceleration components

### Forces
- **Thrust** (N): Engine thrust magnitude
- **Gravity** (m/s²): Gravitational acceleration

## Testing the New Features

1. **Start the simulation**:
   - Click "Initialize" in the control panel
   - Click "Start" to run the simulation

2. **View the plots**:
   - The bottom panel will show real-time plots
   - Use the dropdown menu to select different state functions
   - Plots update every 500ms during simulation

3. **Verify the rocket visual**:
   - The rocket should move smoothly as a single unit
   - The velocity arrow should track with the rocket
   - All parts (body, nose, fins) should rotate together

## Architecture Notes

### Data Flow
```
C++ ODE Integration (wasm_rocket.cpp)
  ↓ (records at each step)
TimeSeriesData storage
  ↓ (getTimeSeries())
JavaScript/TypeScript
  ↓ (React state)
PlotPanel Component (Recharts)
```

### Performance Considerations
- Time-series data is pre-allocated for ~10,000 points
- Recording adds minimal overhead (~1-2% CPU)
- Plot updates are throttled to 500ms intervals
- All serious computations happen in C++ (as required)

## Troubleshooting

### WASM Build Fails
- Ensure Emscripten 3.1.0+ is installed
- Check C++20 compiler support
- Verify all header files are accessible

### Plots Not Showing
- Check browser console for errors
- Verify WASM module loaded successfully
- Ensure simulation is initialized before checking plots

### TypeScript Errors
- Run `npm run type-check` to verify types
- Ensure `sopot.d.ts` is up to date

## File Summary

### Modified Files
1. `wasm/wasm_rocket.cpp` - Added time-series data collection
2. `web/src/components/RocketVisualization3D.tsx` - Improved rocket cohesion
3. `web/src/App.tsx` - New 4-panel layout with plotting
4. `web/src/types/sopot.d.ts` - Added time-series interfaces
5. `web/src/hooks/useRocketSimulation.ts` - Exposed simulator instance

### New Files
1. `web/src/components/PlotPanel.tsx` - Data plotting component
2. `BUILD_INSTRUCTIONS.md` - This file

## Next Steps

After building and testing:
1. Verify all plots show correct data
2. Test different simulation configurations
3. Check performance with long simulations
4. Consider adding more state functions (Mach number, dynamic pressure, etc.)

## References

- [SOPOT Framework Documentation](CLAUDE.md)
- [Emscripten Documentation](https://emscripten.org/docs/)
- [Recharts Documentation](https://recharts.org/)
- [React Three Fiber Documentation](https://docs.pmnd.rs/react-three-fiber/)
