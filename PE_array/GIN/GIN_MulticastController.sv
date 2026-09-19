module GIN_MulticastController #(
    parameter ID_SIZE = `XID_BITS
    )(
    input clk,
    input rst,

    input set_id,
    input [ID_SIZE - 1:0] id_in,
    output logic [ID_SIZE - 1:0] id,

    input [ID_SIZE - 1:0] tag,

    input valid_in,
    output logic valid_out,
    input ready_in,
    output logic ready_out
);

// target：tag 跟自己的 id 一樣，或 tag 是廣播地址 (全為 1)
// Continuous assign (a declaration initializer would evaluate only once)
wire is_target;
assign is_target = (tag == id) || (tag == {ID_SIZE{1'b1}});

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        id <= 0;
    end else if (set_id) begin
        id <= id_in;
    end
end

// Ready-Valid Handshake
assign ready_out = is_target ? ready_in : 1'b1;
assign valid_out = is_target ? valid_in : 1'b0;

endmodule

// note
// master = GLB , slave = PE
// tag = 全為 1 => 廣播地址 => 所有 PE 都會收到這筆資料
// 若非 target => ready_out = 1 
// (因為整條 Bus 只要有其中一個 Ready = 0 就不會傳資料)
// (只要 valid_out = 0 就不會傳資料 , 即使 ready_out = 1)


