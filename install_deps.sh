#!/bin/bash
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "installing dependencies:"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    git cmake g++ make \
    libmosquittopp-dev libssl-dev nlohmann-json3-dev

echo "dependencies installed successfully"