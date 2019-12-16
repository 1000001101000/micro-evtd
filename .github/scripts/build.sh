#!/bin/bash

version="7.5-2019.12"

wget "https://releases.linaro.org/components/toolchain/binaries/$version/arm-linux-gnueabihf/gcc-linaro-$version-x86_64_arm-linux-gnueabihf.tar.xz"
tar xf gcc-linaro-$version-x86_64_arm-linux-gnueabihf.tar.xz

mkdir bins
make 
if [ $? -ne 0 ]; then
  echo "build failed"
  exit 99
fi
cp bin/micro-evtd bins/micro-evtd-x86
make clean
prefix="gcc-linaro-$version-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-"
make CROSS_COMPILE="$prefix"
if [ $? -ne 0 ]; then
  echo "build failed"
  exit 99
fi
cp bin/micro-evtd bins/micro-evtd-armhf
find .
