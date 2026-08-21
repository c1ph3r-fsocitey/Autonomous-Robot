#!/usr/bin/env bash
# Installs the robot's udev rules and adds you to the dialout group.
# Run once:   sudo bash install_udev.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/99-bot-serial.rules"
RULES_DST="/etc/udev/rules.d/99-bot-serial.rules"

if [[ $EUID -ne 0 ]]; then
  echo "This needs root. Re-run with: sudo bash $0" >&2
  exit 1
fi

echo "Installing ${RULES_DST}"
install -m 0644 "${RULES_SRC}" "${RULES_DST}"

echo "Reloading udev"
udevadm control --reload-rules
udevadm trigger

# The user who invoked sudo, not root.
TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || echo "")}"
if [[ -n "${TARGET_USER}" ]] && ! id -nG "${TARGET_USER}" | grep -qw dialout; then
  echo "Adding ${TARGET_USER} to the dialout group"
  usermod -aG dialout "${TARGET_USER}"
  echo "  -> log out and back in for this to take effect"
fi

echo
echo "Done. Unplug and replug both USB devices, then check:"
echo "    ls -l /dev/esp32 /dev/rplidar"
echo
echo "If either symlink is missing, read the comments at the bottom of"
echo "99-bot-serial.rules - you almost certainly need to match on serial number."
