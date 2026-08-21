#!/usr/bin/env bash
# =============================================================================
#  One-time PlatformIO Core setup on a headless Raspberry Pi.
#
#      ./setup_pio.sh
#
#  Installs PlatformIO Core into its own virtualenv at ~/.platformio/penv so it
#  cannot collide with the system Python or with anything ROS installed.
#  Nothing here touches apt's python3 packages.
# =============================================================================
set -euo pipefail

say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
die() { printf '\n\033[1;31m!! %s\033[0m\n' "$*" >&2; exit 1; }

PENV_BIN="${HOME}/.platformio/penv/bin"

# --- 1. system packages ------------------------------------------------------
say "Installing build prerequisites"
sudo apt update
sudo apt install -y python3 python3-venv python3-pip git cmake curl unzip

# --- 2. PlatformIO Core ------------------------------------------------------
if [[ -x "${PENV_BIN}/pio" ]]; then
  say "PlatformIO Core already installed"
else
  say "Installing PlatformIO Core"
  tmp="$(mktemp -d)"
  curl -fsSL -o "${tmp}/get-platformio.py" \
    https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    || die "could not download the PlatformIO installer - check your network"
  python3 "${tmp}/get-platformio.py"
  rm -rf "${tmp}"
fi

[[ -x "${PENV_BIN}/pio" ]] || die "PlatformIO install did not produce ${PENV_BIN}/pio"

# --- 3. PATH -----------------------------------------------------------------
# Expose ONLY the pio commands, via symlinks in ~/.local/bin.
#
# Do NOT put ~/.platformio/penv/bin on PATH. That directory contains the
# virtualenv's own `python3`, which would shadow /usr/bin/python3 for every
# shell you open. A venv does not see /usr/lib/python3/dist-packages, so
# apt-installed modules - numpy in particular - vanish, and every rclpy node
# that touches geometry_msgs dies with ModuleNotFoundError. ROS's own packages
# keep importing (they come in via PYTHONPATH), which makes the failure look
# like a ROS problem rather than a PATH problem. Hours disappear that way.
say "Linking pio into ~/.local/bin"
mkdir -p "${HOME}/.local/bin"
ln -sf "${PENV_BIN}/pio"        "${HOME}/.local/bin/pio"
ln -sf "${PENV_BIN}/platformio" "${HOME}/.local/bin/platformio"

# Clean up the old, harmful PATH entry if a previous run of this script added it.
if grep -q 'platformio/penv/bin' "${HOME}/.bashrc" 2>/dev/null; then
  say "Removing the old ~/.platformio/penv/bin PATH entry from ~/.bashrc"
  sed -i '/platformio\/penv\/bin/d' "${HOME}/.bashrc"
  echo "    (it was shadowing the system python3)"
fi

if ! grep -q 'HOME/.local/bin' "${HOME}/.bashrc" 2>/dev/null; then
  say "Adding ~/.local/bin to PATH in ~/.bashrc"
  echo '' >> "${HOME}/.bashrc"
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> "${HOME}/.bashrc"
fi
export PATH="${HOME}/.local/bin:${PATH}"

# --- 4. serial access --------------------------------------------------------
if ! id -nG "$USER" | grep -qw dialout; then
  say "Adding $USER to the dialout group"
  sudo usermod -aG dialout "$USER"
  echo "    -> log out and back in for this to take effect"
else
  say "Already in the dialout group"
fi

# --- 5. done -----------------------------------------------------------------
say "PlatformIO $(pio --version 2>/dev/null || echo '?') is ready"
cat <<'EOF'

Next, in a NEW shell (or run: export PATH="$HOME/.platformio/penv/bin:$PATH"):

    cd ~/Arduino/bot_firmware_pio
    pio run -t upload

The first build downloads the ESP32 toolchain and compiles the entire
micro-ROS stack from source. On a Pi 4 that is 15-25 minutes. It is cached
afterwards, so later builds take about 20 seconds.

EOF
