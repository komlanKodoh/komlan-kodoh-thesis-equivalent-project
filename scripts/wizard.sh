#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: ./scripts/wizard.sh [-i|install|native|web|help]

  install  Install Rust WebAssembly prerequisites
  native   Launch the native simulator
  web      Build and serve the browser version
  -i       Open the interactive menu
  help     Show this help
EOF
}

choose_action() {
  printf 'Aether Flight Systems setup wizard\n\n1) Install prerequisites\n2) Run native simulator\n3) Run browser version\nq) Quit\n\nChoose an option: '
  read -r action
  case "$action" in
    1) action=install ;;
    2) action=native ;;
    3) action=web ;;
    q|Q) exit 0 ;;
  esac
}

action="${1:-help}"
if [[ "$action" == "-i" ]]; then
  choose_action
fi

case "$action" in
  install) "$ROOT/scripts/install.sh" ;;
  native|web) "$ROOT/scripts/run.sh" "$action" ;;
  help|-h|--help) usage ;;
  *) usage >&2; exit 2 ;;
esac
