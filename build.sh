#!/bin/sh
# Full FPGA flow for OrangeCrab (ECP5-25F, CSFBGA285)
# Usage: sh build.sh          -> build/eyeriss.bit, build/eyeriss.dfu
#        sh build.sh flash    -> also flash through the DFU bootloader
set -e
OSS="${OSS_CAD_SUITE:-/e/FPGA/oss-cad-suite}"
export PATH="$OSS/bin:$OSS/lib:$PATH"
mkdir -p build

yosys -q -l build/yosys.log build.ys
nextpnr-ecp5 --25k --package CSFBGA285 --lpf orangecrab.lpf \
    --json build/eyeriss.json --textcfg build/eyeriss.config \
    --freq 48 --timing-allow-fail -l build/nextpnr.log
# The OrangeCrab DFU bootloader rejects compressed / SPI-freq bitstreams
ecppack --input build/eyeriss.config --bit build/eyeriss.bit

cp build/eyeriss.bit build/eyeriss.dfu
dfu-suffix -v 1209 -p 5af0 -a build/eyeriss.dfu

grep -A 30 "Device utilisation" build/nextpnr.log | head -30
grep "Max frequency" build/nextpnr.log | tail -1

if [ "$1" = "flash" ]; then
    # Hold the button while plugging in the board to enter the bootloader
    dfu-util -D build/eyeriss.dfu
fi
