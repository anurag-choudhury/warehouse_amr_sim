#!/bin/bash
# Build the Docker image
IMAGE_NAME="ros2_humble_gz_harmonic"

docker build -t $IMAGE_NAME .
