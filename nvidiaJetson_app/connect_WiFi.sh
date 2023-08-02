#!/bin/bash

# Connect to my WiFi Hotspot
network_name="P10_lite"
network_password="12345678"

while true; do
  nmcli device wifi rescan
  nmcli device wifi connect "$network_name" password "$network_password" && break
  echo "Failed to connect to $network_name. Retrying in 5 seconds..."
  sleep 5
done

echo "Connected to $network_name. Runing camera stream by RTP..."

sleep 5

# Run camera stream by RTP
# gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), format=NV12, width=1280, height=720' ! nvvidconv flip-method=0 ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=100.89.77.35 port=5001
