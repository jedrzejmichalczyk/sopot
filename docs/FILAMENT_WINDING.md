# SOPOT FilWind — Web-Based Filament Winding Simulation

A mobile-friendly, browser-based filament winding CAD/CAM demo for designing and
simulating the manufacture of a composite rocket **oxidizer tank** (COPV-style
overwrap on a liner). It reproduces, in miniature, the workflow of commercial
packages such as **Cadfil**, **TaniqWind Pro**, **CADWIND**, and **ComposicaD**:

| Commercial workflow stage | FilWind equivalent |
|---|---|
| Mandrel definition | Cylinder + dome (hemispherical / elliptical) liner with polar bosses |
| Laminate design / netting analysis | Optimal winding angle, helical/hoop ply count for MEOP × SF |
| Fibre path generation | Geodesic paths via Clairaut's relation, turnaround dwell |
| Pattern / coverage | Circuit count from band width, coprime skip number *k* (diamond pattern) |
| Payout path & machine animation | Animated mandrel (A axis) + carriage (X axis) + payout eye |
| Post-processor | FANUC-style 2-axis G-code (X mm / A deg), download or copy |

## Where it lives

- Source: `web/public/winding.html` — a single self-contained page (three.js from CDN, no WASM needed).
- Deployed with the existing GitHub Pages workflow at **`https://<user>.github.io/sopot/winding.html`**.
- Linked from the main SOPOT demo app via the **FILWND** card in the simulation selector.
- Local preview: `cd web && npm run dev` then open `http://localhost:3000/winding.html`
  (or just `python3 -m http.server -d web/public` — the page has no build step).

Open it on a phone: everything is touch-first (orbit/pinch on the 3D view, bottom-sheet
parameter panel, large transport controls).

## Physics & algorithms

### Netting analysis (the "optimal solution")

For burst pressure `p = MEOP × SF` on a cylinder of radius `R`, with fiber-direction
laminate allowable `σ = σ_ult × η_translation × V_f`:

- Geodesic helical angle fixed by the polar opening (Clairaut): `α = asin((r₀ + b/2)/R)`
- Axial equilibrium → helical thickness: `t_h = pR / (2σ cos²α)`
- Hoop equilibrium → hoop thickness: `t_θ = pR (2 − tan²α) / (2σ)`
- Plies are rounded up to whole covers (helical cover = ±α pair) and achieved burst /
  margin is recomputed from the as-built laminate.
- Outputs: tank volume, oxidizer capacity (default N₂O at 745 kg/m³), composite shell mass.

### Geodesic path generation

The mandrel meridian is a polyline `(z, r)` parameterized by arc length `m`. A geodesic on
a surface of revolution obeys `r·sin α = c` (Clairaut), giving `dφ/dm = tan α / r`. The
path is marched pole-to-pole; at the turn radius (`sin α → 1`) the band wraps the polar
opening with a **dwell** rotation, then the meridian direction reverses.

The dwell is stretched so that the advance per circuit is `Δφ = 2π(M + k/n)` where `n` is
the circuit count needed for full coverage (`n = ⌈2πR cos α / b⌉`) and `k` is a skip
number coprime with `n` — this produces the uniform diamond coverage pattern familiar
from production winders.

Hoop layers are generated at ~90° with one band-width of carriage advance per revolution.

### Machine simulation & post-processing

Each path point maps to machine coordinates: `X = z` (carriage) and `A = φ` (accumulating
mandrel rotation). The 3D view animates the machine frame (mandrel rotates under a fixed
payout eye), draws the deposited band as a surface ribbon with true band width, and shows
live A/X/α/time telemetry. The post-processor emits decimated `G1 X.. A.. F..` blocks
with a program header carrying the design parameters.

## Limitations / next steps

- Netting analysis only (fibers carry all load) — no FEA, no dome thickness prediction
  beyond the standard `t·R·cosα_R/(r·cosα(r))` buildup note.
- Geodesic paths only; non-geodesic (friction-utilizing) winding as in TaniqWind is not implemented.
- 2-axis post-processor; real machines often use 4–6 axes (yaw/cross-feed for band flattening).
- The path generator is plain TypeScript-free JS by design; porting it to a SOPOT
  `TypedComponent` (states: `m, φ` with `dφ/dm` dynamics) and compiling via the existing
  WASM pipeline is a natural follow-up if autodiff-based pattern optimization is wanted.
