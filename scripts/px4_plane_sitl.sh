#!/bin/bash
echo "Hint:"
echo "cd ~/PX4-Autopilot"
echo "PX4_SIM_MODEL=gazebo-classic_plane ./build/px4_sitl_default/bin/px4 ./build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i 0"
sleep 1
echo ""

SCRIPT_DIR="$(dirname "$0")"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
CMAKE_DIR=$REPO_DIR/src/Simulator
CREATE_APP_IMAGE_SCRIPT_PATH=$REPO_DIR/src/UavDynamics/scripts/build_appimage.sh
BUILD_DIR=$REPO_DIR/build

rm $REPO_DIR/UavDynamics-x86_64.AppImage
$CREATE_APP_IMAGE_SCRIPT_PATH --cmake-dir $CMAKE_DIR --build-dir $BUILD_DIR

$REPO_DIR/UavDynamics-x86_64.AppImage --config $REPO_DIR/configs/px4_plane_5kg.yaml
