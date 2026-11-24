#!/bin/bash
# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Download the official OptiTrack NatNet SDK so that mocap4r2_optitrack_driver
# can be built with its "sdk" backend. The SDK is governed by the OptiTrack
# Plugins EULA and is not distributed with MOCAP4ROS2.
#
# usage: install_natnet_sdk.sh --accept-eula [--dir DIR] [--version 4.4]

set -euo pipefail

VERSION=4.4
DEST="${NATNET_SDK_DIR:-$HOME/.local/share/natnet_sdk}"
ACCEPT=0
EULA_URL="https://www.optitrack.com/about/legal/eula.html"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --accept-eula) ACCEPT=1 ;;
    --dir) DEST="$2"; shift ;;
    --version) VERSION="$2"; shift ;;
    -h|--help) sed -n '16,22p' "$0"; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 1 ;;
  esac
  shift
done

if [[ $ACCEPT -ne 1 ]]; then
  echo "The NatNet SDK is governed by the OptiTrack Plugins EULA: $EULA_URL"
  echo "Read it and run again with --accept-eula to download the SDK."
  exit 1
fi

case "$(uname -m)" in
  x86_64) URL="https://d2mzlempwep3hb.cloudfront.net/NatNetSDKLinux/ubuntu/NatNet_SDK_${VERSION}_ubuntu.tar" ;;
  aarch64|arm64) URL="https://d2mzlempwep3hb.cloudfront.net/NatNetSDKLinux/ubuntu_arm/NatNet_SDK_${VERSION}_ubuntu_ARM.tar" ;;
  *) echo "no NatNet SDK for $(uname -m): use backend 'open'" >&2; exit 1 ;;
esac

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
echo "Downloading $URL"
curl -fsSL --max-time 300 -o "$TMP/sdk.tar" "$URL"
mkdir -p "$TMP/x"
tar -xf "$TMP/sdk.tar" -C "$TMP/x"
ROOT=$(dirname "$(find "$TMP/x" -name NatNetClient.h -path '*/include/*' | head -1)")/..
if [[ ! -f "$ROOT/include/NatNetClient.h" || ! -f "$ROOT/lib/libNatNet.so" ]]; then
  echo "unexpected SDK layout in $URL" >&2
  exit 1
fi

mkdir -p "$DEST"
rm -rf "$DEST/include" "$DEST/lib"
cp -r "$ROOT/include" "$ROOT/lib" "$DEST/"
echo "$VERSION" > "$DEST/VERSION"
echo "NatNet SDK $VERSION installed in $DEST"
echo
echo "Now build mocap4r2_optitrack_driver from source in your workspace:"
if [[ "$DEST" != "$HOME/.local/share/natnet_sdk" ]]; then
  echo "  export NATNET_SDK_DIR=$DEST"
fi
echo "  colcon build --packages-select mocap4r2_optitrack_driver --cmake-clean-cache"
echo "and check with:  ros2 run mocap4r2_optitrack_driver optitrack_driver --ros-args -p backend:=sdk"
