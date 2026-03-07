#!/bin/bash

BLUE='\033[34m'
GREEN='\033[32m'
RED='\033[31m'
YELLOW='\033[33m'
CYAN='\033[36m'
BOLD='\033[1m'
RESET='\033[0m'

GPIO_PATH="/gpio"

echo -e "${BLUE}${BOLD}══════════════════════════════════════════════════════${RESET}"
echo -e "${BLUE}${BOLD}          GPIO PIN SIMULATION MONITOR                ${RESET}"
echo -e "${BLUE}${BOLD}══════════════════════════════════════════════════════${RESET}"
echo ""
echo -e "  ${CYAN}A4988 Driver -> Raspberry Pi GPIO Mapping:${RESET}"
echo "  ──────────────────────────────────────────────"
echo "  A4988 #1 STEP  ->  GPIO 17   (Motor 1: Base)"
echo "  A4988 #1 DIR   ->  GPIO 27   (Motor 1: Base)"
echo "  A4988 #2 STEP  ->  GPIO 22   (Motor 2: Shoulder)"
echo "  A4988 #2 DIR   ->  GPIO 23   (Motor 2: Shoulder)"
echo "  A4988 #3 STEP  ->  GPIO 24   (Motor 3: Elbow)"
echo "  A4988 #3 DIR   ->  GPIO 25   (Motor 3: Elbow)"
echo "  A4988 #4 STEP  ->  GPIO  5   (Motor 4: End Effector)"
echo "  A4988 #4 DIR   ->  GPIO  6   (Motor 4: End Effector)"
echo ""
echo -e "${BLUE}══════════════════════════════════════════════════════${RESET}"
echo -e "${YELLOW}  Waiting for driver activity...${RESET}"
echo ""

tail -f "$GPIO_PATH/activity.log" | while IFS= read -r line; do
    ts=$(date +%H:%M:%S.%3N)

    case "$line" in
        INIT:*)
            echo -e "  ${BLUE}[$ts]${RESET} ⚡ $line"
            ;;
        DRIVE:*)
            echo -e "  ${YELLOW}[$ts]${RESET} ⟳  $line"
            ;;
        DONE:*)
            echo -e "  ${GREEN}[$ts]${RESET} ✓  $line"
            ;;
        CLEANUP:*)
            echo -e "  ${RED}[$ts]${RESET} ■  $line"
            echo ""
            echo -e "  ${CYAN}Final Pin States:${RESET}"
            for pin in 17 27 22 23 24 25 5 6; do
                val=$(cat "$GPIO_PATH/gpio${pin}/value" 2>/dev/null || echo "?")
                dir_state=$(cat "$GPIO_PATH/gpio${pin}/direction" 2>/dev/null || echo "?")
                echo "    GPIO $pin: value=$val  direction=$dir_state"
            done
            echo ""
            echo -e "${BLUE}══════════════════════════════════════════════════════${RESET}"
            echo -e "${YELLOW}  Waiting for next activity...${RESET}"
            echo ""
            ;;
        *)
            echo "  [$ts] $line"
            ;;
    esac
done
