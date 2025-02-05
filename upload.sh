#!/bin/bash

sketch_path="$(pwd)/device/device.ino"

# ports
boards=(
    "/dev/cu.usbserial-11110"
    "/dev/cu.usbserial-11120"
    "/dev/cu.usbserial-11130"
)

# upload execution
for port in "${boards[@]}"; do
  echo "Uploading to $port..."
  arduino-cli upload -p $port --fqbn arduino:avr:uno "$sketch_path"
  if [ $? -eq 0 ]; then
    echo "Successfully uploaded to $port"
  else
    echo "Failed to upload to $port"
  fi
done
