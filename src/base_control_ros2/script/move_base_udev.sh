#!/bin/bash
set -e
sudo tee /etc/udev/rules.d/99-move-base.rules >/dev/null <<'EOF'
# Bingda/Nano controller direct STM32 USB CDC. Uploaded firmware source uses PID 5801; some boards use 5802.
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5801", MODE:="0666", GROUP:="dialout", SYMLINK+="move_base"
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5802", MODE:="0666", GROUP:="dialout", SYMLINK+="move_base"
# CH340 USB-to-UART adapter.
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="move_base"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER" || true
echo "udev rule installed. Replug the controller, then run: ls -l /dev/move_base"
