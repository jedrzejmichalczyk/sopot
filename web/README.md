# SOPOT Web - Interactive 3D Rocket Simulation

**Real-time 6-DOF rocket flight simulation in your browser, powered by C++20 and WebAssembly.**

This is a React + TypeScript application that integrates the SOPOT C++20 physics simulation framework (compiled to WebAssembly) with React Three Fiber for stunning 3D visualization.

![SOPOT Web Demo](screenshot.png)

## ✨ Features

### 🎮 Interactive Controls
- **Configure** launch parameters (elevation, azimuth, diameter, timestep)
- **Start/Pause/Step** simulation with real-time control
- **Playback speed** adjustment (0.1x to 10x real-time)
- **Reset** to initial conditions

### 🌍 3D Visualization
- **Real-time rocket rendering** with proper 6-DOF dynamics
- **Trajectory path** showing flight history
- **Quaternion-based attitude** visualization
- **Launch pad** and ground reference
- **Orbit controls** (rotate, pan, zoom)

### 📊 Live Telemetry
- **Time, altitude, speed, mass** displays
- **Position vector** (East, North, Up)
- **Velocity vector** components
- **Attitude quaternion** display
- **Running/paused** status indicator

## 🚀 Quick Start

### Prerequisites

- Node.js 18+ (for npm)
- The SOPOT WebAssembly module (from `../wasm/`)

### Installation

```bash
# Navigate to web directory
cd web

# Install dependencies
npm install

# Copy WebAssembly files to public directory
cp ../wasm/sopot.js ../wasm/sopot.wasm public/
```

### Development

```bash
# Start development server
npm run dev

# Server will start at http://localhost:3000
```

The app will automatically open in your browser!

### Production Build

```bash
# Build for production
npm run build

# Preview production build
npm run preview
```

## 🎯 Usage

### 1. Initialize Simulation

When you first load the app:
1. Wait for the WebAssembly module to load (indicated in the center panel)
2. Configure launch parameters in the left panel:
   - **Elevation**: 0-90° (default: 85°)
   - **Azimuth**: 0-360° (default: 0°, North)
   - **Diameter**: 0.01-1m (default: 0.16m)
   - **Timestep**: 0.001-0.1s (default: 0.01s)
3. Click **"Initialize Simulation"**

### 2. Run Simulation

After initialization:
- **▶ Start**: Begin continuous simulation
- **⏸ Pause**: Pause the simulation
- **⏭ Step**: Advance by one timestep (when paused)
- **⟲ Reset**: Return to initial conditions

### 3. Adjust Playback Speed

Use the slider to control simulation speed:
- **0.1x**: Slow motion for detailed observation
- **1.0x**: Real-time
- **10x**: Fast-forward for long flights

### 4. Explore the 3D View

**Mouse Controls:**
- **Left click + drag**: Rotate camera around rocket
- **Right click + drag**: Pan camera
- **Scroll wheel**: Zoom in/out

## 📐 Coordinate Systems

**SOPOT (ENU Frame):**
- X: East
- Y: North
- Z: Up

**Three.js Visualization:**
- X: East (Red axis)
- Y: Up (Green axis)
- Z: South (Blue axis, -North)

Automatic conversion is handled in the code.

## 🏗️ Architecture

```
web/
├── src/
│   ├── components/
│   │   ├── RocketVisualization3D.tsx   # Three.js 3D scene
│   │   ├── TelemetryPanel.tsx          # Real-time data display
│   │   └── ControlPanel.tsx            # Configuration & controls
│   ├── hooks/
│   │   └── useRocketSimulation.ts      # WebAssembly integration hook
│   ├── types/
│   │   └── sopot.d.ts                  # TypeScript definitions
│   ├── App.tsx                         # Main application component
│   └── main.tsx                        # React entry point
├── public/
│   ├── sopot.js                        # WebAssembly loader (copied from ../wasm/)
│   └── sopot.wasm                      # Compiled simulation (copied from ../wasm/)
├── package.json
├── vite.config.ts
└── tsconfig.json
```

## 🛠️ Tech Stack

- **React 18** - UI framework
- **TypeScript** - Type safety
- **Vite** - Build tool and dev server
- **React Three Fiber** - React renderer for Three.js
- **@react-three/drei** - Three.js helpers
- **Three.js** - 3D graphics
- **SOPOT WebAssembly** - Physics simulation engine

## ⚡ Performance

**Expected Performance:**
- **60 FPS** visualization on modern hardware
- **10x real-time** simulation speed (simulate 10s in 1s)
- **<1 MB** WebAssembly module size
- **<100ms** initialization time

**Tested on:**
- Chrome 120+ ✅
- Firefox 120+ ✅
- Safari 17+ ✅
- Edge 120+ ✅

## 📊 Data Flow

```
User Input → ControlPanel
              ↓
         useRocketSimulation Hook
              ↓
         SOPOT WebAssembly Module
              ↓
         Simulation State
         ↙           ↘
RocketVisualization3D   TelemetryPanel
     (3D View)          (Numbers)
```

## 🎨 Customization

### Changing Rocket Appearance

Edit `src/components/RocketVisualization3D.tsx`:

```typescript
// Rocket body color
<meshStandardMaterial color="#cc0000" /> // Red

// Rocket size
<Cylinder args={[0.08, 0.08, 1.0, 16]} /> // radius, radius, height, segments
```

### Adding More Telemetry

Edit `src/components/TelemetryPanel.tsx`:

```typescript
const telemetryItems = [
  // Add new telemetry item
  {
    label: 'Mach Number',
    value: (state.speed / 340).toFixed(2), // Speed of sound
    unit: 'Ma',
    color: '#9b59b6',
  },
];
```

### Modifying Initial Configuration

Edit `src/components/ControlPanel.tsx`:

```typescript
const [config, setConfig] = useState<SimulationConfig>({
  elevation: 85,    // Change default elevation
  azimuth: 0,       // Change default azimuth
  diameter: 0.16,   // Change default diameter
  timestep: 0.01,   // Change default timestep
});
```

## 🐛 Troubleshooting

### "Failed to load WebAssembly module"

**Solution**: Ensure `sopot.js` and `sopot.wasm` are in the `public/` directory:
```bash
cp ../wasm/sopot.{js,wasm} public/
```

### Black screen or no 3D view

**Solution**: Check browser console for errors. Ensure WebGL is enabled:
- Visit `chrome://gpu` (Chrome) or `about:support` (Firefox)
- Check for WebGL support

### Simulation runs too fast/slow

**Solution**: Adjust playback speed slider or reduce timestep in configuration

### Poor performance

**Solutions:**
- Reduce trajectory history limit in `App.tsx` (currently 1000 points)
- Increase timestep (less frequent updates)
- Close other browser tabs
- Use Chrome for best performance

## 🔬 Development Notes

### WebAssembly Integration

The WebAssembly module is loaded dynamically:

```typescript
const createSopotModule = await import('/sopot.js');
const moduleInstance = await createSopotModule.default();
const sim = new moduleInstance.RocketSimulator();
```

### Animation Loop

Simulation runs in `requestAnimationFrame`:

```typescript
const animate = (currentTime: number) => {
  simulator.step();                    // Advance physics
  setCurrentState(sim.getFullState()); // Update React state
  requestAnimationFrame(animate);      // Next frame
};
```

### Trajectory Tracking

Position history is stored with coordinate conversion:

```typescript
const threePosition = new THREE.Vector3(
  position.x,    // East (same)
  position.z,    // Up (Z→Y)
  -position.y    // North (Y→-Z)
);
```

## 📈 Future Enhancements

### Phase 2 (Planned)
- [ ] Real-time charts (altitude vs time, speed vs time)
- [ ] CSV data loading from browser
- [ ] Export trajectory data
- [ ] Camera follow mode
- [ ] Multiple camera angles
- [ ] Atmospheric layers visualization

### Phase 3 (Advanced)
- [ ] WebGPU rendering for particle effects
- [ ] Wind field visualization
- [ ] Monte Carlo uncertainty visualization
- [ ] Parameter optimization UI
- [ ] Multi-stage rockets
- [ ] Compare multiple trajectories

## 📝 License

Same as SOPOT framework.

## 🤝 Contributing

This is a demonstration application. For production use:
1. Implement proper error handling
2. Add data persistence (localStorage)
3. Implement CSV file upload
4. Add unit tests
5. Optimize bundle size
6. Add accessibility features

## 📚 Resources

- [React Three Fiber Docs](https://docs.pmnd.rs/react-three-fiber)
- [Three.js Manual](https://threejs.org/manual/)
- [SOPOT Documentation](../CLAUDE.md)
- [WebAssembly MDN](https://developer.mozilla.org/en-US/docs/WebAssembly)

---

**Built with ❤️ using React, Three.js, and SOPOT WebAssembly**

*Phase 1 Complete - Interactive 3D visualization achieved!* 🚀✨
