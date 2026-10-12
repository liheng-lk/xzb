#!/bin/bash
set -e

DEV="${1:-/dev/ttyUSB0}"
RULE_FILE="/etc/udev/rules.d/99-linear-actuator-cp2102.rules"
SYMLINK="linear_actuator"

if [ ! -e "$DEV" ]; then
    echo "Device not found: $DEV"
    echo "Plug in the CP2102 and run: ls -l /dev/ttyUSB*"
    exit 1
fi

if ! command -v udevadm >/dev/null 2>&1; then
    echo "udevadm not found"
    exit 1
fi

PROPS="$(udevadm info --query=property --name="$DEV")"
VID="$(printf '%s\n' "$PROPS" | sed -n 's/^ID_VENDOR_ID=//p' | head -n1)"
PID="$(printf '%s\n' "$PROPS" | sed -n 's/^ID_MODEL_ID=//p' | head -n1)"
SERIAL="$(printf '%s\n' "$PROPS" | sed -n 's/^ID_SERIAL_SHORT=//p' | head -n1)"

if [ -z "$VID" ] || [ -z "$PID" ]; then
    echo "Cannot detect USB VID/PID from $DEV"
    exit 1
fi

echo "Detected: device=$DEV VID=$VID PID=$PID serial=${SERIAL:-<none>}"

if [ -n "$SERIAL" ]; then
    RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", ATTRS{serial}==\"$SERIAL\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"$SYMLINK\""
else
    echo "WARNING: no USB serial number was found. Rule will match all devices with this VID/PID."
    RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"$SYMLINK\""
fi

echo "$RULE" | sudo tee "$RULE_FILE" >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger

echo
cat "$RULE_FILE"
echo
echo "Rule installed. Unplug and reconnect the CP2102, then check:"
echo "  ls -l /dev/$SYMLINK"
