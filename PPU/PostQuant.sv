`include "define.svh"
module PostQuant (
    input [`DATA_BITS-1:0] data_in,
    input [5:0] scaling_factor,
    output logic [7:0] data_out
);

logic signed [31:0] shifted_data;
assign shifted_data = $signed(data_in) >>> scaling_factor;

// Saturation Arithmetic
logic [7:0] clamped_data;
always_comb begin
    if (shifted_data > 32'sd127) begin
        clamped_data = 8'h7F;  // 127
    end else if (shifted_data < -32'sd128) begin
        clamped_data = 8'h80;  // -128
    end else begin
        clamped_data = shifted_data[7:0];
    end
end

// Int8(-128~127) => UInt8(0~255)
assign data_out = { ~clamped_data[7], clamped_data[6:0] };

endmodule

// note
// dot_product 為 32-bit => 壓縮回 8-bit Feature Map => Quantization
// "有號數"除法用 Arithmetic Right Shift (>>>) => 右移後最高位補符號位 => 保持正負號不變
// 過大/過小的值直接硬切會產生精度損失 => Saturation Arithmetic 
// Saturation Arithmetic => 超過範圍的值固定在上下限
// sd : signed decimal
// Int8(-128~127) => UInt8(0~255) => 加上128 => 反轉符號位
