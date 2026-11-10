# Aether Flight Systems

A 3D Quad control simulation with one Rust controller shared by the native and browser versions. The native app uses Bevy and Rapier; the browser uses the same controller as WebAssembly with Rapier.js physics and Three.js rendering.

## Architecture

UAV behavior is divided across four layers, each with a focused responsibility:

| Layer | Responsibility |
| --- | --- |
| Perception | Builds an understanding of the world from vehicle state and observed objects, including obstacles relevant to flight. |
| Planning | Uses the mission goal and perceived world to form the high-level plan and desired movement. This is where the core decision-making behavior lives. |
| Actuation | Translates the high-level plan into concrete UAV commands, including throttle, yaw, pitch, and roll. |
| Safety | Monitors the requested behavior and can override it when a collision is imminent. It has final authority to apply braking or other protective commands. |

Perception informs planning; planning provides intent to actuation; safety checks the resulting requested behavior and overrides it when needed. These layers are shared by the native and browser versions through `quad-core`; each version applies the final output through its own physics engine.

## Run native

```sh
./scripts/run.sh native
```

The native app opens fullscreen. Both versions use a 200 × 200 m stage, include mission-target controls, and let you add or remove buildings. Planning mode pauses the quad and lets you explore the stage: left-drag to orbit, right-drag to pan, and scroll to zoom. Start the simulation to switch to a camera that follows behind the quadcopter. In manual flight, W/S adjust throttle, A/D control yaw, I/K control pitch, and J/L control roll the retained throttle.

## Run in a browser

From the project directory, the launcher can set up the WebAssembly prerequisites and start the browser version:

```sh
./scripts/run.sh web
```

Run `./scripts/run.sh` without an option to choose between browser and native versions from a menu. The first browser launch installs the WebAssembly target and `wasm-bindgen-cli` if they are missing.

## Setup wizard

Install Rust and Node.js 22.6 or newer, then run `./scripts/wizard.sh install` to install prerequisites. The installer adds the Rust WebAssembly target and `wasm-bindgen-cli`; if Rust is missing, it installs it through the official rustup installer. Use `./scripts/wizard.sh -i` to open the interactive menu.

For manual setup, install the Rust WebAssembly target and `wasm-bindgen-cli` once, then run:

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --locked
cargo run -p bevy-ui --bin build-wasm
cargo run -p bevy-ui --bin serve
```

Open http://127.0.0.1:8080. The browser uses Rapier.js for physics and Three.js for rendering; those modules load from jsDelivr.

## Autonomous heading alignment

Autopilot turns the nose (body +Y) toward the requested horizontal velocity before
translating. It resumes movement within 12° of that direction, tapers speed between
12° and 20°, and brakes to a horizontal stop beyond 20° while turning. Once stopped
for alignment, it must return within 12° to resume. Vertical motion remains available.
Existing momentum takes time to brake; this constrains requested motion rather than
instantaneously changing velocity. Collision safety runs afterward and can brake in
any direction regardless of heading. Manual pilot channels retain direct control.
