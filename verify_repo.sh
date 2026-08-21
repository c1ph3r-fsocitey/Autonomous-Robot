#!/usr/bin/env bash
# =============================================================================
#  Does this repo actually contain everything needed to rebuild the robot?
#
#      bash verify_repo.sh
#
#  Checks three things, in order of how badly they bite:
#
#    1. Is every required file present AND tracked by git? A file that exists
#       on this Pi but is untracked will not survive a fresh clone.
#    2. Did the calibrated constants make it in? A repo with a placeholder
#       TICKS_PER_REV builds fine and drives like a shopping trolley.
#    3. Is any generated junk tracked? build/ and install/ make the repo
#       enormous and permanently dirty.
#
#  Run it from the repo root (the directory holding this script).
# =============================================================================
set -uo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

GREEN='\033[1;32m'; RED='\033[1;31m'; YEL='\033[1;33m'; DIM='\033[2m'; OFF='\033[0m'
fails=0
warns=0

ok()   { printf "  ${GREEN}ok${OFF}    %s\n" "$1"; }
bad()  { printf "  ${RED}MISS${OFF}  %s\n" "$1"; fails=$((fails+1)); }
warn() { printf "  ${YEL}warn${OFF}  %s\n" "$1"; warns=$((warns+1)); }

if ! git rev-parse --git-dir >/dev/null 2>&1; then
  printf "${RED}Not a git repository.${OFF} Run 'git init' here first.\n"
  exit 1
fi

# Everything git currently tracks, one path per line.
TRACKED="$(git ls-files)"

tracked() { grep -qxF "$1" <<<"$TRACKED"; }

check() {
  local f="$1"
  if [[ ! -e "$f" ]]; then
    bad "$f  (file does not exist)"
  elif ! tracked "$f"; then
    bad "$f  (exists but NOT tracked - 'git add' it)"
  else
    ok "$f"
  fi
}

# =============================================================================
printf "\n${DIM}1. Required files${OFF}\n"
# =============================================================================

printf "\n  ${DIM}repo root${OFF}\n"
for f in README.md .gitignore; do check "$f"; done

printf "\n  ${DIM}bot_description${OFF}\n"
for f in \
  bot_description/package.xml \
  bot_description/CMakeLists.txt \
  bot_description/urdf/bot.urdf.xacro \
  bot_description/urdf/base.xacro \
  bot_description/urdf/wheels.xacro \
  bot_description/urdf/sensors.xacro \
  bot_description/urdf/materials.xacro \
  bot_description/urdf/inertial_macros.xacro \
  bot_description/urdf/ros2_control.xacro \
  bot_description/rviz/bot.rviz \
  bot_description/rviz/nav.rviz \
  bot_description/launch/display.launch.py
do check "$f"; done

printf "\n  ${DIM}bot_hardware${OFF}\n"
for f in \
  bot_hardware/package.xml \
  bot_hardware/CMakeLists.txt \
  bot_hardware/bot_hardware.xml \
  bot_hardware/include/bot_hardware/diffdrive_microros.hpp \
  bot_hardware/src/diffdrive_microros.cpp
do check "$f"; done

printf "\n  ${DIM}bot_bringup${OFF}\n"
for f in \
  bot_bringup/package.xml \
  bot_bringup/CMakeLists.txt \
  bot_bringup/README.md \
  bot_bringup/docs/CALIBRATION.md \
  bot_bringup/docs/TROUBLESHOOTING.md \
  bot_bringup/config/controllers.yaml \
  bot_bringup/config/ekf.yaml \
  bot_bringup/config/imu_filter.yaml \
  bot_bringup/config/joy_teleop.yaml \
  bot_bringup/config/rplidar.yaml \
  bot_bringup/config/slam_mapping.yaml \
  bot_bringup/config/slam_localization.yaml \
  bot_bringup/config/twist_mux.yaml \
  bot_bringup/launch/drive.launch.py \
  bot_bringup/launch/bringup.launch.py \
  bot_bringup/launch/rsp.launch.py \
  bot_bringup/launch/control.launch.py \
  bot_bringup/launch/sensors.launch.py \
  bot_bringup/launch/localization.launch.py \
  bot_bringup/launch/slam.launch.py \
  bot_bringup/launch/teleop.launch.py \
  bot_bringup/launch/teleop_race.launch.py \
  bot_bringup/launch/rviz.launch.py \
  bot_bringup/scripts/check_stack.py \
  bot_bringup/scripts/wheel_jog.py \
  bot_bringup/scripts/calibrate_encoders.py \
  bot_bringup/scripts/calibrate_odom.py \
  bot_bringup/scripts/joy_race_teleop.py \
  bot_bringup/udev/99-bot-serial.rules \
  bot_bringup/udev/install_udev.sh
do check "$f"; done

printf "\n  ${DIM}bot_navigation${OFF}\n"
for f in \
  bot_navigation/package.xml \
  bot_navigation/CMakeLists.txt \
  bot_navigation/config/nav2_params.yaml \
  bot_navigation/launch/navigation.launch.py \
  bot_navigation/launch/amcl.launch.py
do check "$f"; done

printf "\n  ${DIM}firmware${OFF}\n"
if [[ -d firmware ]]; then
  for f in \
    firmware/platformio.ini \
    firmware/src/main.cpp \
    firmware/src/config.h \
    firmware/README.md
  do check "$f"; done
else
  warn "firmware/ is not in the repo - the ESP32 source is NOT backed up"
  printf "        ${DIM}mv ~/Arduino/bot_firmware_pio firmware${OFF}\n"
  printf "        ${DIM}touch firmware/COLCON_IGNORE${OFF}\n"
fi

# =============================================================================
printf "\n${DIM}2. Calibrated values (are these the good ones?)${OFF}\n"
# =============================================================================

CFG=""
for c in firmware/src/config.h "$HOME/Arduino/bot_firmware_pio/src/config.h"; do
  [[ -f "$c" ]] && { CFG="$c"; break; }
done

if [[ -z "$CFG" ]]; then
  bad "config.h not found anywhere - cannot verify calibration"
else
  printf "  ${DIM}reading %s${OFF}\n" "$CFG"
  grep_val() { grep -E "^#define\s+$1\s" "$CFG" | awk '{print $3}'; }

  for name in TICKS_PER_REV_LEFT TICKS_PER_REV_RIGHT WHEEL_RADIUS_M MAX_WHEEL_RAD_S; do
    v="$(grep_val "$name")"
    if [[ -z "$v" ]]; then
      bad "$name is not defined"
    else
      ok "$name = $v"
    fi
  done

  if grep -qE '^#define\s+TICKS_PER_REV\s' "$CFG"; then
    warn "a single TICKS_PER_REV is still defined - the per-wheel split may have been reverted"
  fi
  if grep -qE '^#define\s+TICKS_PER_REV_LEFT\s+2800' "$CFG"; then
    bad "TICKS_PER_REV_LEFT is still the 2800 PLACEHOLDER - this is not calibrated"
  fi
fi

# controllers.yaml should agree with the measured ceiling
CY="bot_bringup/config/controllers.yaml"
if [[ -f "$CY" ]]; then
  mv_="$(grep -E '^\s+max_velocity:\s' "$CY" | head -1 | awk '{print $2}')"
  [[ -n "$mv_" ]] && ok "controllers.yaml linear max_velocity = $mv_" \
                  || warn "could not read max_velocity from controllers.yaml"
fi

# =============================================================================
printf "\n${DIM}3. Junk that should not be tracked${OFF}\n"
# =============================================================================

junk=0
while IFS= read -r p; do
  case "$p" in
    build/*|install/*|log/*|*/__pycache__/*|*.pyc|.pio/*|rplidar_ros/*)
      bad "tracked but should be ignored: $p"
      junk=$((junk+1))
      ;;
  esac
done <<<"$TRACKED"
[[ $junk -eq 0 ]] && ok "no generated files tracked"

n=$(wc -l <<<"$TRACKED")
size=$(du -sh .git 2>/dev/null | cut -f1)
printf "\n  ${DIM}%s files tracked, .git is %s${OFF}\n" "$n" "${size:-?}"
if [[ $n -gt 500 ]]; then
  warn "over 500 tracked files - something generated probably slipped in"
fi

# =============================================================================
printf "\n${DIM}4. Uncommitted work${OFF}\n"
# =============================================================================
if [[ -n "$(git status --porcelain)" ]]; then
  warn "working tree is dirty - commit before you call this backed up:"
  git status --short | sed 's/^/        /'
else
  ok "working tree clean"
fi

if git rev-parse --abbrev-ref '@{upstream}' >/dev/null 2>&1; then
  ahead=$(git rev-list --count '@{upstream}..HEAD' 2>/dev/null || echo 0)
  if [[ "$ahead" != "0" ]]; then
    warn "$ahead commit(s) not pushed yet - run 'git push'"
  else
    ok "everything pushed to the remote"
  fi
else
  warn "no upstream branch set - nothing is on GitHub yet"
  printf "        ${DIM}git push -u origin main${OFF}\n"
fi

# =============================================================================
printf "\n"
if [[ $fails -gt 0 ]]; then
  printf "${RED}%d problem(s)${OFF}, %d warning(s). A fresh clone would NOT rebuild this robot.\n\n" "$fails" "$warns"
  exit 1
elif [[ $warns -gt 0 ]]; then
  printf "${YEL}%d warning(s)${OFF}, no missing files.\n\n" "$warns"
  exit 0
else
  printf "${GREEN}Complete.${OFF} A fresh clone has everything needed to rebuild the robot.\n\n"
  exit 0
fi
