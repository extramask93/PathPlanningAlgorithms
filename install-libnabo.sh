#!/bin/sh
set -ex
git clone --depth 1 https://github.com/ethz-asl/libnabo.git libnabo
cd libnabo
mkdir build
cd build
cmake .. && cmake --build . --config Release && sudo cmake --install

