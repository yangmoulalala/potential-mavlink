#!/bin/bash

RULE_FILE="/etc/udev/rules.d/99-stm32-serial.rules"

echo "=== Installing STM32 serial udev rule ==="

# 创建规则文件
sudo bash -c "cat > $RULE_FILE" <<EOF
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
    MODE="0777", SYMLINK+="c_board"
EOF

echo "[OK] Udev rule written to $RULE_FILE"

# 重载 udev
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[OK] Udev rules reloaded."

# 提示用户重新插拔设备
echo "=== Done! Please replug your STM32 USB cable. ==="
ls -l /dev/c_board 2>/dev/null