#!/usr/bin/env bash
set -euo pipefail

# ── Colors ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
RESET='\033[0m'

# ── Helpers ─────────────────────────────────────────────────────────────────
prompt_choice() {
    local prompt="$1"
    shift
    local options=("$@")
    local count=${#options[@]}

    printf "\n${CYAN}${prompt}${RESET}\n" >&2
    for i in "${!options[@]}"; do
        local num=$((i + 1))
        if [[ $i -eq 0 ]]; then
            printf "  ${BOLD}[%d] %s ${DIM}(default)${RESET}\n" "$num" "${options[$i]}" >&2
        else
            printf "  [%d] %s\n" "$num" "${options[$i]}" >&2
        fi
    done
    printf "${BOLD}> ${RESET}" >&2
    read -r choice
    choice=${choice:-1}

    if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > count )); then
        choice=1
    fi
    echo "$choice"
}

prompt_yn() {
    local prompt="$1"
    local default="${2:-y}"
    local hint="Y/n"
    [[ "$default" == "n" ]] && hint="y/N"

    printf "\n${CYAN}${prompt}${RESET} [${hint}] " >&2
    read -r answer
    answer=${answer:-$default}
    answer=$(echo "$answer" | tr '[:upper:]' '[:lower:]')
    [[ "$answer" == "y" ]]
}

# ── Banner ──────────────────────────────────────────────────────────────────
clear
printf "${BOLD}"
printf "  ╔══════════════════════════════════════╗\n"
printf "  ║         IMXRTNSY Build Config        ║\n"
printf "  ╚══════════════════════════════════════╝\n"
printf "${RESET}"
printf "  ${DIM}Teensy 4.1 (i.MX RT1062) firmware${RESET}\n"

# ── TFT Display ─────────────────────────────────────────────────────────────
tft_choice=$(prompt_choice "TFT Display (auto-detects ILI9341/ST7735):" \
    "Enabled" \
    "Disabled")

case "$tft_choice" in
    1) TFT=1 ;;
    2) TFT=0 ;;
esac

# ── Command Input ────────────────────────────────────────────────────────
# LPUART6 (pins 0/1) — command UART to host bridge.
NET=0
CMD_BAUD=115200

cmd_choice=$(prompt_choice "Command input:" \
    "Serial (pins 0/1, LPUART6)" \
    "Ethernet (KMBox Net UDP)")

case "$cmd_choice" in
    1)
        cmd_baud_choice=$(prompt_choice "Command UART baud rate (CP2102 max 921600):" \
            "115200" \
            "460800" \
            "921600" \
            "Custom")
        case "$cmd_baud_choice" in
            1) CMD_BAUD=115200 ;;
            2) CMD_BAUD=460800 ;;
            3) CMD_BAUD=921600 ;;
            4)
                printf "${BOLD}  Baud rate> ${RESET}"
                read -r CMD_BAUD
                CMD_BAUD=${CMD_BAUD:-115200}
                ;;
        esac
        ;;
    2)
        NET=1
        # NET mode requires TFT for IP/port/UUID display
        if [[ "$TFT" -eq 0 ]]; then
            printf "\n${YELLOW}  Note: Ethernet mode requires TFT — enabling display${RESET}\n"
            TFT=1
        fi
        ;;
esac

# ── Bluetooth ────────────────────────────────────────────────────────────
# HC-05 on LPUART7 (D28/D29), STA on D30
BT=0
BT_BAUD=115200

if prompt_yn "HC-05 Bluetooth input (D28/D29)?" "n"; then
    BT=1
    bt_baud_choice=$(prompt_choice "HC-05 baud rate (AT-configured at boot):" \
        "921600" \
        "460800" \
        "115200" \
        "Custom")
    case "$bt_baud_choice" in
        1) BT_BAUD=921600 ;;
        2) BT_BAUD=460800 ;;
        3) BT_BAUD=115200 ;;
        4)
            printf "${BOLD}  BT baud rate> ${RESET}"
            read -r BT_BAUD
            BT_BAUD=${BT_BAUD:-115200}
            ;;
    esac
fi

# ── Build Options ───────────────────────────────────────────────────────────
CLEAN=false
if prompt_yn "Clean before build?" "n"; then
    CLEAN=true
fi

FLASH=false
if prompt_yn "Flash after build?" "n"; then
    FLASH=true
fi

BUILD_BRIDGE=false

PARALLEL=$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)

# ── Summary ─────────────────────────────────────────────────────────────────
printf "\n${BOLD}  ┌─ Build Configuration ─────────────────┐${RESET}\n"

if [[ "$TFT" -eq 1 ]]; then
    printf "  │  Display:    ${GREEN}%-25s${RESET} │\n" "Auto-detect"
else
    printf "  │  Display:    ${DIM}%-25s${RESET} │\n" "Disabled"
fi

if [[ "$TFT" -eq 1 ]]; then
    printf "  │  Touch:      ${GREEN}%-25s${RESET} │\n" "Auto (ILI9341 only)"
else
    printf "  │  Touch:      ${DIM}%-25s${RESET} │\n" "Disabled"
fi

if [[ "$NET" -eq 1 ]]; then
    printf "  │  Cmd input:  ${GREEN}%-25s${RESET} │\n" "Ethernet (KMBox Net)"
else
    printf "  │  Cmd input:  ${GREEN}%-25s${RESET} │\n" "LPUART6 @ ${CMD_BAUD}"
fi

if [[ "$BT" -eq 1 ]]; then
    printf "  │  Bluetooth:  ${GREEN}%-25s${RESET} │\n" "HC-05 @ ${BT_BAUD}"
else
    printf "  │  Bluetooth:  ${DIM}%-25s${RESET} │\n" "Disabled"
fi

$CLEAN && cl="Yes" || cl="No"
$FLASH && fl="Yes" || fl="No"
$BUILD_BRIDGE && br="Yes" || br="No"
printf "  │  Clean:      %-25s │\n" "$cl"
printf "  │  Flash:      %-25s │\n" "$fl"
printf "  │  Bridge:     %-25s │\n" "$br"
printf "  │  Parallel:   %-25s │\n" "-j${PARALLEL}"
printf "  └──────────────────────────────────────┘\n"

if ! prompt_yn "Proceed?" "y"; then
    printf "${YELLOW}  Aborted.${RESET}\n"
    exit 0
fi

# ── Build ───────────────────────────────────────────────────────────────────
printf "\n"

MAKE_ARGS=(
    "TFT=${TFT}"
    "NET=${NET}"
    "CMD_BAUD=${CMD_BAUD}"
    "BT=${BT}"
    "BT_BAUD=${BT_BAUD}"
    "-j${PARALLEL}"
)

if $CLEAN; then
    printf "${YELLOW}Cleaning...${RESET}\n"
    make clean
    printf "${GREEN}Clean done.${RESET}\n\n"
fi

# Build firmware (without bridge target if not requested)
if $BUILD_BRIDGE; then
    printf "${CYAN}Building firmware + bridge...${RESET}\n"
    make "${MAKE_ARGS[@]}"
else
    printf "${CYAN}Building firmware...${RESET}\n"
    make "${MAKE_ARGS[@]}" firmware.hex
fi

printf "\n${GREEN}Build complete.${RESET}\n"

if $FLASH; then
    printf "\n${CYAN}Flashing Teensy 4.1...${RESET}\n"
    make flash
    printf "${GREEN}Flash complete.${RESET}\n"
fi

printf "\n${BOLD}Done.${RESET}\n"
