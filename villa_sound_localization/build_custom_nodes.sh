#!/bin/bash

# Builds hark-villa hark nodes

# Run entire script as superuser
[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"

PARENT_DIR="$( dirname "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" )"

TEMP_DIR=`mktemp -d`
cd "$TEMP_DIR"

apt-get source harkfd
tar xf harkfd_2.3.0*.tar.gz

cd "$PARENT_DIR/hark-villa"
make clean
autoreconf --install
# MIC_CHANNELS should be number of microphones in microphone array and MIC_FRAME_LENGTH should be frame length Hark's audiostream from mic uses
./configure --prefix=/usr --with-hark-inc="$TEMP_DIR/hark-fd/include" --enable-ros MIC_CHANNELS=4 MIC_FRAME_LENGTH=512
make
sudo make install

rm -rf "$TEMP_DIR"
