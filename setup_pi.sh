#!/bin/bash
# =============================================================================
# CamosunROV -- one-time Raspberry Pi setup (fresh Raspberry Pi OS Lite, Trixie)
#
# Run from the repo's top folder, as your normal user (NOT with sudo):
#     cd ~/CamosunROV_Pi
#     bash setup_pi.sh
#
# What it does (each step checks first and skips anything already done,
# so it's safe to run again, e.g. after pulling new code):
#   1. Installs the system packages (camera stack, PyAV, git, venv)
#   2. Sets up the UART for the STM32 (no serial console, Bluetooth off)
#   3. Adds you to the dialout (serial) and video (camera) groups
#   4. Gives eth0 the tether's static IP, 192.168.2.2/24 ("rov-tether")
#   5. Creates the ROV_venv Python environment (+ pyserial)
#   6. Makes main.py / Test_UART_DMA.py runnable
#   7. Turns off services the ROV doesn't need (faster boot)
#   8. Installs the "rov" service so main.py starts at every boot
# Then it tells you whether a reboot is needed.
#
# Uses sudo where needed (it will ask for your password once).
# Needs internet (Wi-Fi) for step 1.
# =============================================================================

set -euo pipefail

TETHER_IP="192.168.2.2/24"
TETHER_CON="rov-tether"
SERVICE="rov"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$REPO_DIR/ROV_venv"
RUN_USER="$(id -un)"
CONFIG_TXT="${CONFIG_TXT:-/boot/firmware/config.txt}"
CMDLINE_TXT="${CMDLINE_TXT:-/boot/firmware/cmdline.txt}"

REBOOT_NEEDED=0
step() { echo; echo "=== $* ==="; }
ok()   { echo "  [ok]   $*"; }
did()  { echo "  [done] $*"; }
warn() { echo "  [WARN] $*"; }

# ---------------------------------------------------------------- checks
if [[ $EUID -eq 0 ]]; then
    echo "Run this as your normal user, not with sudo:  bash setup_pi.sh"
    exit 1
fi
if [[ ! -f "$REPO_DIR/main.py" ]]; then
    echo "main.py not found next to this script -- run it from the CamosunROV_Pi folder."
    exit 1
fi
if [[ "$RUN_USER" != "pirov" || "$REPO_DIR" != "/home/pirov/CamosunROV_Pi" ]]; then
    warn "Expected user 'pirov' and folder /home/pirov/CamosunROV_Pi (the scripts' first lines use that path)."
    warn "Found user '$RUN_USER', folder $REPO_DIR. The boot service will still work; './main.py' may not."
fi
echo "CamosunROV Pi setup -- user $RUN_USER, repo $REPO_DIR"
sudo -v    # ask for the password once, up front

# ---------------------------------------------------------------- 1. packages
step "1. System packages"
if ! sudo apt-get update -q; then
    warn "apt-get update failed -- no internet? Connect Wi-Fi and run this script again."
fi
# One package at a time, so a single missing package can't stop the rest of the setup.
# --no-install-recommends keeps the desktop/GUI pieces off OS Lite.
for pkg in git python3-venv rpicam-apps-lite python3-picamera2 python3-libcamera python3-av; do
    if dpkg -s "$pkg" > /dev/null 2>&1; then
        ok "$pkg"
    elif sudo apt-get install -y -q --no-install-recommends "$pkg" > /dev/null; then
        did "installed $pkg"
    else
        warn "could not install $pkg"
    fi
done

# ---------------------------------------------------------------- 2. UART
step "2. UART for the STM32"
if [[ ! -f "$CONFIG_TXT" ]]; then
    warn "$CONFIG_TXT not found -- skipping UART setup (is this Raspberry Pi OS?)"
else
    if grep -qE 'console=(serial0|ttyAMA0|ttyS0)' "$CMDLINE_TXT" 2>/dev/null; then
        sudo raspi-config nonint do_serial_cons 1      # no Linux login console on the UART
        did "serial console turned off"
        REBOOT_NEEDED=1
    else
        ok "no serial console on the UART"
    fi
    if ! grep -q '^enable_uart=1' "$CONFIG_TXT"; then
        sudo raspi-config nonint do_serial_hw 0        # UART hardware on (enable_uart=1)
        did "UART hardware enabled"
        REBOOT_NEEDED=1
    else
        ok "UART hardware enabled"
    fi
    if ! grep -q '^dtoverlay=disable-bt' "$CONFIG_TXT"; then
        echo "dtoverlay=disable-bt" | sudo tee -a "$CONFIG_TXT" > /dev/null
        did "Bluetooth off, full PL011 UART on /dev/serial0 (dtoverlay=disable-bt)"
        REBOOT_NEEDED=1
    else
        ok "dtoverlay=disable-bt already set"
    fi
fi

# ---------------------------------------------------------------- 3. groups
step "3. Groups (serial port + camera access)"
for g in dialout video; do
    if id -nG "$RUN_USER" | tr ' ' '\n' | grep -qx "$g"; then
        ok "$RUN_USER is in $g"
    else
        sudo usermod -aG "$g" "$RUN_USER"
        did "added $RUN_USER to $g"
        REBOOT_NEEDED=1
    fi
done

# ---------------------------------------------------------------- 4. static IP
step "4. Tether static IP ($TETHER_IP on eth0)"
if ! command -v nmcli > /dev/null; then
    warn "nmcli (NetworkManager) not found -- set the static IP by hand"
elif nmcli -t -f NAME con show | grep -qx "$TETHER_CON"; then
    sudo nmcli con mod "$TETHER_CON" ipv4.method manual ipv4.addresses "$TETHER_IP" \
        ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 100
    ok "'$TETHER_CON' exists -- settings re-applied"
else
    sudo nmcli con add type ethernet ifname eth0 con-name "$TETHER_CON" \
        ipv4.method manual ipv4.addresses "$TETHER_IP" ipv6.method disabled \
        connection.autoconnect yes connection.autoconnect-priority 100 > /dev/null
    did "created '$TETHER_CON' (comes up whenever the tether is plugged in)"
fi
# Not activated here on purpose: switching eth0 now could drop an SSH session
# that runs over Ethernet. It activates by itself on the next boot / cable plug-in.

# ---------------------------------------------------------------- 5. venv
step "5. Python environment ($VENV)"
if [[ ! -x "$VENV/bin/python3" ]]; then
    # --system-site-packages: the venv uses the apt camera stack (picamera2,
    # libcamera, av), whose versions are matched to each other.
    python3 -m venv --system-site-packages "$VENV"
    did "created ROV_venv"
else
    ok "ROV_venv exists"
    if grep -q '^include-system-site-packages = false' "$VENV/pyvenv.cfg"; then
        sed -i 's/^include-system-site-packages = false/include-system-site-packages = true/' "$VENV/pyvenv.cfg"
        did "ROV_venv can now see the system camera packages (include-system-site-packages = true)"
    fi
fi
"$VENV/bin/pip" install -q --disable-pip-version-check pyserial==3.5
did "pyserial installed"
if "$VENV/bin/python3" -c "import serial, picamera2, libcamera, av" 2> /dev/null; then
    ok "venv can import serial, picamera2, libcamera, av"
else
    warn "venv import check failed -- run: $VENV/bin/python3 -c 'import serial, picamera2, libcamera, av'"
fi

# ---------------------------------------------------------------- 6. scripts
step "6. Program files"
for f in main.py Test_UART_DMA.py; do
    [[ -f "$REPO_DIR/$f" ]] || continue
    # A Windows line ending on the first line breaks "./main.py" ("bad interpreter").
    if head -n 1 "$REPO_DIR/$f" | grep -q $'\r'; then
        sed -i '1s/\r$//' "$REPO_DIR/$f"
        did "$f: removed Windows line ending from the first line"
    fi
    chmod +x "$REPO_DIR/$f"
done
ok "main.py and Test_UART_DMA.py are runnable"

# ---------------------------------------------------------------- 7. boot speed
step "7. Services the ROV doesn't need (faster boot)"
for s in hciuart bluetooth ModemManager NetworkManager-wait-online; do
    if systemctl is-enabled "$s" > /dev/null 2>&1; then
        sudo systemctl disable --now "$s" > /dev/null 2>&1 || true
        did "disabled $s"
    else
        ok "$s off / not installed"
    fi
done

# ---------------------------------------------------------------- 8. boot service
step "8. Boot service '$SERVICE' (runs main.py at power-on)"
UNIT="/etc/systemd/system/$SERVICE.service"
NEW_UNIT="$(cat <<EOF
# Installed by setup_pi.sh -- runs the CamosunROV software (main.py) at boot.
#   systemctl status $SERVICE        is it running?
#   journalctl -u $SERVICE -f        live output (same as ./main.py)
#   sudo systemctl stop $SERVICE     before running ./main.py or Test_UART_DMA.py by hand
[Unit]
Description=CamosunROV Pi software (bridge + camera)
After=network.target

[Service]
User=$RUN_USER
WorkingDirectory=$REPO_DIR
ExecStart=$VENV/bin/python3 -u main.py
Restart=always
RestartSec=2
KillSignal=SIGTERM
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF
)"
if [[ -f "$UNIT" ]] && [[ "$(cat "$UNIT")" == "$NEW_UNIT" ]]; then
    ok "$UNIT up to date"
else
    echo "$NEW_UNIT" | sudo tee "$UNIT" > /dev/null
    sudo systemctl daemon-reload
    did "wrote $UNIT"
fi
sudo systemctl enable "$SERVICE" > /dev/null 2>&1
ok "'$SERVICE' starts at every boot"

# ---------------------------------------------------------------- done
echo
echo "=============================================================================="
echo " Setup finished."
if [[ $REBOOT_NEEDED -eq 1 ]]; then
    echo " A REBOOT IS NEEDED for the UART / group changes to take effect."
fi
echo " After a reboot, main.py starts by itself at every power-on."
echo "   systemctl status $SERVICE      is it running?"
echo "   journalctl -u $SERVICE -f      live output (Ctrl+C stops watching)"
echo "   sudo systemctl stop $SERVICE   before running ./main.py or Test_UART_DMA.py by hand"
echo "=============================================================================="

# Ask until the answer is y or n. No terminal (e.g. piped input ran out) counts as "n".
while true; do
    if ! read -r -p "Reboot now? (y/n) " answer; then
        echo
        [[ -n "$answer" ]] || answer="n"      # input ended: use what was typed, else "n"
    fi
    case "${answer,,}" in
        y|yes) echo "Rebooting..."; sudo reboot; exit 0 ;;
        n|no)  break ;;
        *)     echo "Please type y or n." ;;
    esac
done

if [[ $REBOOT_NEEDED -eq 1 ]]; then
    echo "Not rebooting. Remember to reboot (sudo reboot) before using the ROV --"
    echo "the UART and group changes only apply after a reboot."
else
    sudo systemctl restart "$SERVICE"
    echo "Not rebooting. Started the '$SERVICE' service now -- main.py is running."
fi