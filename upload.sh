#!/bin/bash

sketch_path="$(pwd)/device/device.ino"

# ports
boards=(
  "/dev/cu.usbmodem14101"
  "/dev/cu.usbmodem14201"
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
