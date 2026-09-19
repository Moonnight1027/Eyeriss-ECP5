#!/bin/sh
# Run the SoC testbenches with Icarus Verilog (USB core replaced by sim/usb_stub.sv)
# Usage: sh sim/run_sim.sh   (from the project root)
#   tb_top   : legacy 1-channel protocol, golden model in SystemVerilog
#   tb_zlp   : USB bridge terminates full-packet replies with a ZLP
#   tb_layer : multi-channel layer protocol, vectors from sim/gen_vectors.py
set -e
export PATH="${OSS_CAD_SUITE:-/e/FPGA/oss-cad-suite}/bin:${OSS_CAD_SUITE:-/e/FPGA/oss-cad-suite}/lib:$PATH"
mkdir -p build
SRCS="define.svh
    PE_array/GIN/GIN_MulticastController.sv PE_array/GIN/GIN_Bus.sv PE_array/GIN/GIN.sv
    PE_array/GON/GON_MulticastController.sv PE_array/GON/GON_Bus.sv PE_array/GON/GON.sv
    PE_array/PE.sv PE_array/PE_array.sv
    PPU/PostQuant.sv PPU/Maxpool_Qint8.sv PPU/ReLU_Qint8.sv PPU/PPU.sv
    GLB_SRAM.sv sim/usb_stub.sv Eyeriss_SoC_Top.sv"

echo "=== tb_zlp"
iverilog -g2012 -s tb_zlp -o build/tb_zlp.vvp usb/usb_uart_bridge_ep.v sim/tb_zlp.sv
vvp -n build/tb_zlp.vvp | grep -v "finish called"

python sim/gen_vectors.py
for tb in tb_top tb_layer; do
    echo "=== $tb"
    iverilog -g2012 -I. -s $tb -o build/$tb.vvp $SRCS sim/$tb.sv 2>&1 | grep -v "sorry: constant selects" || true
    vvp -n build/$tb.vvp | grep -v "finish called"
done
