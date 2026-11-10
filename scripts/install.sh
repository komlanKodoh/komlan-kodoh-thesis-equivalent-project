#!/usr/bin/env bash
set -euo pipefail

install_rust() {
  if command -v rustup >/dev/null 2>&1; then
    return
  fi
  if ! command -v curl >/dev/null 2>&1; then
    echo "curl is required to install Rust. Install curl and rerun this script." >&2
    exit 1
  fi
  echo "Installing Rust with the official rustup installer..."
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
  export PATH="${CARGO_HOME:-$HOME/.cargo}/bin:$PATH"
}

install_wasm_tools() {
  rustup target add wasm32-unknown-unknown
  if command -v wasm-bindgen >/dev/null 2>&1; then
    echo "wasm-bindgen is already installed."
  else
    cargo install wasm-bindgen-cli --locked
  fi
}

install_rust
install_wasm_tools
echo "Setup complete. Launch with ./scripts/run.sh native or ./scripts/run.sh web."
