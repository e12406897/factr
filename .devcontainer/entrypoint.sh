#!/bin/bash
# Runs as root (the image's default runtime user — see Dockerfile, USER is no longer
# switched away from root before ENTRYPOINT). Do the one-time, root-only setup, then
# drop privileges to the dev user for everything else (postCreateCommand/postStartCommand
# in devcontainer.json run separately as `remoteUser`, unaffected by this).
set -e

# USB-serial latency timer must be 1 for the Dynamixel leader arm's control loop to hit
# 200-500 Hz; resets to default on every USB replug, hence set on every container start,
# not just once at image build (which couldn't reach real devices anyway).
for f in /sys/bus/usb-serial/devices/ttyUSB*/latency_timer; do
    if [ -e "$f" ]; then
        echo 1 > "$f" || echo "entrypoint.sh: could not set latency_timer for $f"
    fi
done

exec gosu "${USERNAME:-asl_team}" "$@"
