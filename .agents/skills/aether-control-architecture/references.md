# Aether architecture references

## Rust crates

- `crates/drone-core/`: portable vectors, world snapshots, and `SmartUav` control layers.
- `crates/drone-core/src/interfaces/`: one file per perception, operation, actuation, and safety contract, with its default implementation beside it.
- `crates/wasm-interface/`: `wasm-bindgen` adapter around `drone-core`.
- `crates/bevy-ui/`: Bevy native app, Rapier physics, UI, browser build commands, and local static server.
- `web/`: TypeScript app, Rapier.js physics, Three.js scene, and browser UI.

## Control flow

The app reads body state and obstacles, calls `SmartUav::step`, then applies the returned force in its physics engine. The operation planner updates at 1 Hz; actuation and safety update each 60 Hz physics step. Browser code handles physics and rendering but makes no flight-control decisions.

The browser WASM adapter receives `[position xyz, velocity xyz]`, obstacle rows `[x, y, z, radius, height]`, and elapsed seconds. It returns `[force xyz, target delta xyz, safety override]`.

## Commands

Run from the workspace root:

```sh
./run.sh native
./run.sh web
```

The web command installs missing WASM prerequisites, builds the controller and browser assets, and serves the page at http://127.0.0.1:8080.
