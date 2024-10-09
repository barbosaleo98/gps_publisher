#!/bin/bash
set -e

UDP_PORT=5000
HOST_ADDR="172.16.1.1"
DEVICE_ADDR="172.16.2.1"

echo "Starting connection to UDP port $UDP_PORT ..."
ssh user@$DEVICE_ADDR "socat SYSTEM:\"gpspipe -r\" UDP:$HOST_ADDR:$UDP_PORT;"
