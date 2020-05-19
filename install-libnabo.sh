#!/bin/sh
set -ex
git clone --depth 1 https://github.com/ethz-asl/libnabo.git libnabo
cmake . && sudo make install
