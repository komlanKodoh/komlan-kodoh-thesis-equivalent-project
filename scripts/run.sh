#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

usage() {
  cat <<'EOF'
Usage: ./scripts/run.sh [web|native|help]

  web      Build and start the browser version
  native   Start the native desktop version
  help     Show this help

Run ./scripts/run.sh with no option to choose from a menu.
EOF
}

run_web() {
  if ! rustup target list --installed | grep -qx 'wasm32-unknown-unknown'; then
    rustup target add wasm32-unknown-unknown
  fi

  if ! command -v wasm-bindgen >/dev/null 2>&1; then
    cargo install wasm-bindgen-cli --locked
  fi

  cargo run -p bevy-ui --bin build-wasm
  cargo run -p bevy-ui --bin serve
}

choice="${1:-}"
if [[ -z "$choice" ]]; then
  printf 'Aether Flight Systems\n\n1) Browser version\n2) Native desktop version\nq) Quit\n\nChoose an option: '
  read -r choice
  case "$choice" in
    1) choice=web ;;
    2) choice=native ;;
    q|Q) exit 0 ;;
  esac
fi

case "$choice" in
  web) run_web ;;
  native) cargo run -p bevy-ui --bin simulator ;;
  help|-h|--help) usage ;;
  *) usage >&2; exit 2 ;;
esac
