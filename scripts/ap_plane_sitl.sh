#!/bin/bash
echo "Hint:"
echo "cd ~/ardupilot"
echo "./Tools/autotest/sim_vehicle.py -v Plane --console --map -w --model JSON -l 55.75690,48.74115,-7,0"
sleep 1
echo ""

SCRIPT_DIR="$(dirname "$0")"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
CMAKE_DIR=$REPO_DIR/src/Simulator
CREATE_APP_IMAGE_SCRIPT_PATH=$REPO_DIR/src/UavDynamics/scripts/build_appimage.sh
BUILD_DIR=$REPO_DIR/build

rm $REPO_DIR/UavDynamics-x86_64.AppImage
$CREATE_APP_IMAGE_SCRIPT_PATH --cmake-dir $CMAKE_DIR --build-dir $BUILD_DIR

$REPO_DIR/UavDynamics-x86_64.AppImage --config $REPO_DIR/configs/ap_plane.yaml
