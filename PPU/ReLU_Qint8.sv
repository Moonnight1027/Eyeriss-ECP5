module ReLU_Qint8 (
    input en,
    input [7:0] data_in,
    output logic [7:0] data_out
);

// 數值小於 128 => 強制拉回 128，否則保持原值
assign data_out = en ? ((data_in > 8'd128) ? data_in : 8'd128) : data_in;

endmodule

// note
// ReLU => 負數全部變成 0
// UInt8 => 負數對應 0 ~ 127 , 0 對應128(Zero-Point) , 正數對應 129 ~ 255
// => 小於 128 的值全部變成 128