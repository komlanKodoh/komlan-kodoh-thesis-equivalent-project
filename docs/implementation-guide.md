# Implementation guide

The Rust code is kept in three crates. `quad-core` defines world values and `Quad` layers; `wasm-interface` exposes the controller through `wasm-bindgen`; `bevy-ui` runs the native simulation and builds/serves the browser version. Browser JavaScript owns Rapier.js stepping and Three.js rendering.

Each physics step, the app sends vehicle state and nearby world objects through `Quad::step`. It applies the returned force to the physics body. Planning updates at 1 Hz, while QuadController and safety run at the physics rate.

Use `./scripts/run.sh native` or `./scripts/run.sh web` from the project root. The web option builds the WASM controller and browser assets before starting the server.
