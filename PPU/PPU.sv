

module PPU (
    input clk,
    input rst,
    input [`DATA_BITS-1:0] data_in,
    input [5:0] scaling_factor,
    input maxpool_en,
    input maxpool_init,
    input relu_sel,
    input relu_en,
    output logic[7:0] data_out
);

logic [7:0] quant_out;
logic [7:0] maxpool_out;
logic [7:0] relu_in;

PostQuant u_postquant (
    .data_in(data_in),
    .scaling_factor(scaling_factor),
    .data_out(quant_out)
);

Maxpool_Qint8 u_maxpool (
    .clk(clk),
    .rst(rst),
    .en(maxpool_en),
    .init(maxpool_init),
    .data_in(quant_out),
    .data_out(maxpool_out)
);

// relu_sel 決定 ReLU 的輸入來源 
assign relu_in = relu_sel ? maxpool_out : quant_out;

ReLU_Qint8 u_relu (
    .en(relu_en),
    .data_in(relu_in),
    .data_out(data_out)
);

endmodule

// note
// PPU => Post-Processing Unit
// 1. Quantization
// 2. Max Pooling 
// 3. ReLU 
// 有些 layer 沒有 max pooling => Mux 決定是否跳過 