module Maxpool_Qint8 (
    input clk,
    input rst,
    input en,
    input init,
    input logic [7:0] data_in,
    output logic [7:0] data_out
);

logic [7:0] max_reg;

// 比大小
always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        max_reg <= 8'd0;
    end else if (init) begin
        max_reg <= data_in;
    end else if (en) begin
        if (data_in > max_reg) begin
            max_reg <= data_in;
        end
    end
end

assign data_out = en ? (init ? data_in : ((data_in > max_reg) ? data_in : max_reg)) : data_in;

endmodule

// note
// 兩次比大小
// always_ff => 記住上一次的最大值 => 與新的輸入比較 => 更新最大值 (會有 1 cycle latency) 
// assign 沒有記憶功能 => 直接比較輸入與 max_reg，輸出較大者 (達成0延遲)
