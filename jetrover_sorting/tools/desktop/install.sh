#!/usr/bin/env bash
# One-time setup on the robot HOST (run over SSH, it asks for the sudo password
# once). Afterwards two icons on the desktop start and stop the demo with a
# mouse click, no keyboard needed.
set -e
TOOLS="$(cd "$(dirname "$0")/.." && pwd)"
USER_NAME="$(id -un)"
chmod +x "$TOOLS"/start_demo.sh "$TOOLS"/stop_demo.sh "$TOOLS"/desktop/*.sh

# 1. let the start script run its two root commands without a password prompt
SYSTEMCTL="$(command -v systemctl)"
JETSON_CLOCKS="$(command -v jetson_clocks || echo /usr/bin/jetson_clocks)"
RULE="/etc/sudoers.d/jetrover-demo"
TMP="$(mktemp)"
echo "$USER_NAME ALL=(root) NOPASSWD: $SYSTEMCTL stop start_app_node.service, $JETSON_CLOCKS" > "$TMP"
sudo visudo -cf "$TMP"
sudo install -m 0440 -o root -g root "$TMP" "$RULE"
rm -f "$TMP"
echo "sudo rule installed: $RULE"

# 2. the desktop icons
DESKTOP="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$DESKTOP"
write_icon() {   # file name, label, script, icon
    cat > "$DESKTOP/$1" <<ICON
[Desktop Entry]
Type=Application
Name=$2
Exec=$TOOLS/desktop/run_in_window.sh $TOOLS/$3
Icon=$4
Terminal=true
ICON
    chmod +x "$DESKTOP/$1"
    gio set "$DESKTOP/$1" metadata::trusted true 2>/dev/null || true
}
write_icon jetrover-start.desktop 'Start robot demo' start_demo.sh media-playback-start
write_icon jetrover-stop.desktop 'Stop robot demo' stop_demo.sh media-playback-stop
echo "icons created in $DESKTOP"
