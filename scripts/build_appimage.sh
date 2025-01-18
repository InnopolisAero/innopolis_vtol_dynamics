#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
CMAKE_DIR=$REPO_DIR/src/Simulator
CREATE_APP_IMAGE_SCRIPT_PATH=$REPO_DIR/src/UavDynamics/scripts/build_appimage.sh
BUILD_DIR=$REPO_DIR/build

$CREATE_APP_IMAGE_SCRIPT_PATH --cmake-dir $CMAKE_DIR --build-dir $BUILD_DIR
