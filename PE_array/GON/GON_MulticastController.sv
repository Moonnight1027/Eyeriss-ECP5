module GON_MulticastController #(
    parameter ID_SIZE = `XID_BITS
)(
    input clk,
    input rst,

    // config id
    input set_id,
    input [ID_SIZE - 1:0] id_in,
    output logic [ID_SIZE - 1:0] id,

    // tag
    input [ID_SIZE - 1:0] tag,

    input valid_in,
    output logic valid_out,
    input ready_in,
    output logic ready_out
);

// 只有 tag 跟 id 一樣才會傳資料
// Continuous assign (a declaration initializer would evaluate only once)
wire is_target;
assign is_target = (tag == id);

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        id <= 0;
    end else if (set_id) begin
        id <= id_in;
    end
end

assign ready_out = is_target ? ready_in : 1'b0;
assign valid_out = is_target ? valid_in : 1'b0;

endmodule

// note
// 大致架構與 GIN 相同 but有幾處邏輯差異
// 只有 tag 跟 id 一樣才會傳資料
// 若非 target => ready_out = 0 
// 只有被 GLB 指定的 PE 才可以傳資料