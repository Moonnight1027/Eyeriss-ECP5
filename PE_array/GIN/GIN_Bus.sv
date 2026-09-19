 module GIN_Bus #(
    parameter NUMS_SLAVE = `NUMS_PE_COL,
    parameter ID_SIZE = `XID_BITS
) (
    input clk,
    input rst,

   // Master I/O
    input [ID_SIZE-1:0] tag,
    input master_valid,
    input [`DATA_BITS-1:0] master_data,
    output logic master_ready,

   // Slave I/O
    input [NUMS_SLAVE-1:0] slave_ready,
    output logic [NUMS_SLAVE-1:0] slave_valid,
    output logic [`DATA_BITS-1:0] slave_data,

    // Config
    input set_id,
    input [ID_SIZE-1:0] ID_scan_in,
    output logic [ID_SIZE-1:0] ID_scan_out
 );

assign slave_data = master_data;

logic [ID_SIZE-1:0] id_chain [0:NUMS_SLAVE];
assign id_chain[NUMS_SLAVE] = ID_scan_in;
assign ID_scan_out = id_chain[0];

// 全部 PE ready 才傳資料
logic [NUMS_SLAVE-1:0] ready_chain;
assign master_ready = &ready_chain; 

genvar x;
generate 
    for (x = 0; x < NUMS_SLAVE; x = x + 1) begin : gen_mc
        GIN_MulticastController #(.ID_SIZE(ID_SIZE)) GIN_MC (
            .clk(clk), 
            .rst(rst), 
            .set_id(set_id),
            .id_in(id_chain[x+1]),  
            .id(id_chain[x]),       
            .tag(tag), 
            .valid_in(master_valid), 
            .valid_out(slave_valid[x]),
            .ready_in(slave_ready[x]), 
            .ready_out(ready_chain[x])
        );
    end
endgenerate

endmodule

// note
// master = GLB (valid) -> bus (ready) , slave = bus (valid) -> PE (ready)
// 全部 PE ready 才傳資料
// generate for loop => 迴圈生成多個相同 module
// .id_in(id_chain[x+1])
// .id(id_chain[x])
// x=0 => .id_in(id_chain[1]), .id(id_chain[0])
// x=1 => .id_in(id_chain[2]), .id(id_chain[1])
// 第 1 顆 PE 的 id 傳給第 0 顆 PE 串聯 
// 並聯設計 : clk , rst ... 等 (大家共用完全一樣的訊號) => Synchronization Broadcast
// 串聯設計 : id 由後往前傳遞(每個 PE id 都不同) => 節省腳位 , 但要花費更多 clock cycles (but 僅需設定一次)
