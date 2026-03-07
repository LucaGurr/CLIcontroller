#!/bin/bash
set -e

PINS=(17 27 22 23 24 25 5 6)
GPIO_PATH="/gpio"

echo "Setting up simulated GPIO sysfs structure..."

mkdir -p "$GPIO_PATH"
touch "$GPIO_PATH/export"
touch "$GPIO_PATH/unexport"

for pin in "${PINS[@]}"; do
    mkdir -p "$GPIO_PATH/gpio${pin}"
    echo "in" > "$GPIO_PATH/gpio${pin}/direction"
    echo "0"  > "$GPIO_PATH/gpio${pin}/value"
    echo "  Created gpio${pin} (direction=in, value=0)"
done

touch "$GPIO_PATH/activity.log"
echo "GPIO simulation ready. ${#PINS[@]} pins initialized."
echo ""

exec /monitor.sh
