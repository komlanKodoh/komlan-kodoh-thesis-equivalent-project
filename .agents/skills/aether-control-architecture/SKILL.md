---
name: aether-control-architecture
description: Use when changing Aether simulation boundaries, SmartUav control, native Bevy/Rapier integration, browser physics, WASM, or the project layout.
---

# Aether architecture

Keep `drone-core` independent of Bevy, Rapier, WASM bindings, and rendering. It owns portable simulation values and the `SmartUav` layers. `wasm-interface` adapts the core to WebAssembly. `bevy-ui` owns the native app, browser build tools, and server; TypeScript owns browser rendering and Rapier.js stepping.

For crate responsibilities, data flow, migration notes, and run commands, read [references.md](references.md) only when the task needs those details.

Keep UI and physics out of `drone-core`. Preserve the 1 Hz operation planner and live actuation/safety updates. Browser telemetry or visuals may format shared state but must not make control decisions.
