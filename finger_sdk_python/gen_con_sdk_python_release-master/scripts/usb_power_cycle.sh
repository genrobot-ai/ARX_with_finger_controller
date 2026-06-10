#!/bin/bash
# USB device reset helper — called with sudo by databus.py
# Usage: sudo usb_power_cycle.sh /sys/devices/.../1-1.3
set -e
USB_PATH="$1"
AUTH_FILE="$USB_PATH/authorized"

if [ ! -f "$AUTH_FILE" ]; then
    echo "ERROR: $AUTH_FILE not found"
    exit 1
fi

# Step 1: Find /dev/bus/usb device node from sysfs and do USBDEVFS_RESET
BUSNUM=$(cat "$USB_PATH/busnum" 2>/dev/null)
DEVNUM=$(cat "$USB_PATH/devnum" 2>/dev/null)
if [ -n "$BUSNUM" ] && [ -n "$DEVNUM" ]; then
    USB_DEV=$(printf "/dev/bus/usb/%03d/%03d" "$BUSNUM" "$DEVNUM")
    if [ -e "$USB_DEV" ]; then
        echo "Sending USB bus reset to $USB_DEV ..."
        usbreset "$USB_DEV" 2>&1 || true
        sleep 0.5
    fi
fi

# Step 2: Full deauthorize/reauthorize cycle (forces re-enumeration)
echo "Deauthorizing USB device..."
echo 0 > "$AUTH_FILE"
sleep 1.0
echo "Reauthorizing USB device..."
echo 1 > "$AUTH_FILE"
sleep 0.5

echo "OK: USB device reset complete"
