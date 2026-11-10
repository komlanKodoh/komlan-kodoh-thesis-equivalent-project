# Architecture

## Workspace map

This repository is a Rust workspace with four crates and a browser client:

```text
crates/
  quad-core/       Portable simulation data and Quad control layers
  quad-sim/        Simulation helpers built on quad-core
  wasm-interface/  wasm-bindgen adapter exposing quad-core to JavaScript
  bevy-ui/         Native Bevy app, Rapier physics, and browser build/server tools
web/               TypeScript browser app, Rapier.js physics, Three.js rendering
scripts/           Setup wizard and installer subscripts
```

`quad-core` owns portable world snapshots, vehicle state, and the QuadSensors,
QuadMissionPlanner, QuadController, and safety layers. Keep it independent of
Bevy, Rapier, rendering, and WebAssembly bindings. `quad-sim` provides reusable
simulation helpers over this core. `wasm-interface` adapts the core's controller
for WebAssembly. `bevy-ui` owns the native application and the commands that
build and serve the browser version. The `web` directory owns browser rendering,
input, and Rapier.js physics.

## Runtime and control flow

Both front ends gather vehicle state and nearby obstacles, call the shared Quad
controller, then apply its output in their own physics engine. Native builds use
Bevy with Rapier; browser builds call the same controller through WebAssembly
and use Rapier.js with Three.js. Browser code displays telemetry but does not
make flight-control decisions.

The operation planner refreshes its goal directive at 1 Hz. Actuation and
safety run at the 60 Hz physics update. The quad is modeled as a single rigid
body; rotor and propeller dynamics are outside this prototype.

The browser adapter accepts vehicle position and velocity, obstacle rows in
`[x, y, z, radius, height]` form, and elapsed time. It returns force, target
delta, and a safety-override flag. Keep this boundary small so the native and
browser front ends continue to share the same control behavior.

## Setup

Install Rust with `rustup` and Node.js 22.6 or newer. From the repository root,
run the setup wizard and choose **Install prerequisites**:

```sh
./scripts/wizard.sh
```

The installer adds the `wasm32-unknown-unknown` Rust target and
`wasm-bindgen-cli` needed for browser builds. It can install Rust through the
official rustup installer if Rust is missing. The browser build uses Node's
`--experimental-strip-types` flag; browser imports for Three.js and Rapier.js
are loaded from jsDelivr when the page runs.

## Run

From the workspace root:

```sh
./scripts/run.sh native   # launch the native desktop simulator
./scripts/run.sh web      # build WASM/browser assets and serve on 127.0.0.1:8080
```

The web build requires the WASM prerequisites installed by the wizard. Run
`./scripts/wizard.sh -i` for the interactive menu, or pass `install`, `native`,
or `web` as a direct command. With no arguments, the wizard prints help. For
manual web setup, run `cargo run -p bevy-ui --bin build-wasm`, then
`cargo run -p bevy-ui --bin serve`.
