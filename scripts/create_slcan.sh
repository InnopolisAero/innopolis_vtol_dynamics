#!/bin/bash
# https://github.com/PonomarevDA/tools/blob/main/can/create_slcan.sh
# This software is distributed under the terms of the MIT License.
# Copyright (c) 2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

SCRIPT_NAME=$(basename $BASH_SOURCE)
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

HELP="usage: $SCRIPT_NAME [--help] [--device <dev_path>] [--interface <interface>] [--only-find].

The utility automatically finds a CAN-sniffer and creates SLCAN.

options:
    -d, --device DEV_PATH           Specify device path manually instead of trying to find it
                                    among known sniffers. Example: /dev/ttyACM0.
    -i, --interface INTERFACE       Specify custom interface. By default it attach it to slcan0.
    -o, --only-find                 Do not create the interface, only look for a sniffer.
    -h, --help                      Show this help message and exit."

INTERFACE=slcan0
DEV_PATH=""
ONLY_FIND="no"

function log_error() {
    lineno=($(caller))
    printf "$RED$SCRIPT_NAME ERROR on line ${lineno}: $1!$NC\n"
}

function log_warn() {
    lineno=($(caller))
    printf "$YELLOW$SCRIPT_NAME WARN on line ${lineno}: $1.$NC\n"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--device)
        DEV_PATH="$2"
        shift
        ;;

        -i|--interface)
        INTERFACE="$2"
        shift
        ;;

        -o|--only-find)
        ONLY_FIND="yes"
        ;;

        -h|--help)
        echo "$HELP"
        [[ "${BASH_SOURCE[0]}" -ef "$0" ]] && exit 0 || return 0
        ;;

        *)
        log_error "Unknown option: $1"
        echo "$HELP"
        [[ "${BASH_SOURCE[0]}" -ef "$0" ]] && exit 0 || return 1
        ;;
    esac
    shift
done

function find_sniffer() {
    KNOWN_SNIFFERS=(
        "ID_MODEL_ID=374b|ID_VENDOR_ID=0483" # RaccoonLab programmer-sniffer
        "ID_MODEL_ID=60c7|ID_VENDOR_ID=1d50" # Zubax Babel-Babel
    )

    EXPECTED_DEV_PATH="/dev/ttyACM*"
    EXPECTED_SYMLINK_PATH="/dev/serial/by-id/"
    for dev_path in $EXPECTED_DEV_PATH; do
        [ -e "$dev_path" ] || continue
        for sniffer in ${KNOWN_SNIFFERS[@]}; do
            is_sniffer=$(udevadm info $dev_path | grep -E "($sniffer)" -wc || true)
            if [ "$is_sniffer" == 2 ]; then
                DEV_PATH=$(find -L $EXPECTED_SYMLINK_PATH -samefile $dev_path)
                break
            fi
        done
    done
}

function create_slcan() {
    sudo slcand -o -c -f -s8 -t hw -S 1000000 $DEV_PATH $INTERFACE
    sleep 1 # without sleep next commands may fail with 'Cannot find device "slcan0"'
    sudo ip link set up $INTERFACE
    sudo tc qdisc add dev $INTERFACE root handle 1: pfifo_head_drop limit 1000
}


# Step 1. Define the device path
if [ -z $DEV_PATH ]; then
    find_sniffer

    if [ -z "$DEV_PATH" ]; then
        log_error "Can't find a sniffer"
        [[ "${BASH_SOURCE[0]}" -ef "$0" ]] && exit 0 || return 1
    fi

    if [[ $ONLY_FIND == "yes" ]]; then
        echo "$DEV_PATH"
        [[ "${BASH_SOURCE[0]}" -ef "$0" ]] && exit 0 || return 0
    fi
fi


# Step 2. Create SLCAN based on device path
echo "SLCAN creator settings:
- DEV_PATH=$DEV_PATH
- INTERFACE=$INTERFACE"

if [[ $(ip link show | grep $INTERFACE) ]]; then
    log_warn "specified interface already exist, skip"
    [[ "${BASH_SOURCE[0]}" -ef "$0" ]] && exit 0 || return 1
fi

create_slcan

if [[ -z $(ip link show | grep $INTERFACE) ]]; then
    log_error "Interface '$INTERFACE' has not been successfully created"
    [[ "${BASH_SOURCE[0]}" -ef "$0" ]] && exit 0 || return 1
fi
