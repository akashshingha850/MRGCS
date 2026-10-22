# syntax=docker/dockerfile:1.6
# mavros-gcs/onboard — the drone-side ROS 2 stack: rosbridge (:9090) + ZED map relay + gcs_control + MAVROS
# (+ the brain / barcode nodes gcs_control starts on demand). Builds natively on arm64 (Jetson) and x86_64.
#
# Build (repo root):  docker compose -f docker/compose.yaml build
#
# The camera and the map come from the ZED SDK wrapper, which is NOT in this image (it needs CUDA and the
# ZED SDK — bisg_isaac's bisg/zed image, or Stereolabs' zed-ros2-wrapper); this stack only consumes its topics.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
        ros-jazzy-rosbridge-server \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-opencv python3-numpy python3-pip libzbar0 \
    && rm -rf /var/lib/apt/lists/*

# pyzbar is not packaged for Ubuntu 24.04
RUN pip3 install --break-system-packages --no-cache-dir pyzbar

# MAVROS aborts at start without the GeographicLib geoid / gravity / magnetic datasets
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

COPY raspberry/ /opt/gcs/raspberry/
COPY renderer/  /opt/gcs/renderer/
RUN chmod +x /opt/gcs/raspberry/*.sh /opt/gcs/raspberry/*.py

# Logs (~/gcs_logs) and the mission DB (~/brain_data.db) go to the /data volume
ENV HOME=/data
WORKDIR /data

# rosbridge + map relay + web app (GCS_WEB_PORT) + gcs_control; the dashboard starts the rest
ENV GCS_WEB_PORT=8090
CMD ["bash", "/opt/gcs/raspberry/rosbridge_boot.sh"]
