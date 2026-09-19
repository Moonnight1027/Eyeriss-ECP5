module GON_Bus #(
    parameter NUMS_MASTER = `NUMS_PE_COL,
    parameter ID_SIZE = `XID_BITS
) (
    input clk,
    input rst,
    input [ID_SIZE - 1:0] tag,

    input [NUMS_MASTER - 1:0] master_valid,
    input [NUMS_MASTER * `DATA_BITS - 1:0] master_data,
    output logic [NUMS_MASTER - 1:0] master_ready,

    output logic slave_valid,
    input slave_ready,
    output logic [`DATA_BITS - 1:0] slave_data,

    // Config
    input set_id,
    input [ID_SIZE - 1:0] ID_scan_in,
    output logic [ID_SIZE - 1 :0] ID_scan_out
 );

logic [ID_SIZE-1:0] id_chain [0:NUMS_MASTER];
assign id_chain[NUMS_MASTER] = ID_scan_in;
assign ID_scan_out = id_chain[0];

logic [NUMS_MASTER-1:0] valid_chain;

genvar x;
generate
    for (x = 0; x < NUMS_MASTER; x = x + 1) begin : gen_mc
        GON_MulticastController #(.ID_SIZE(ID_SIZE)) GON_MC (
            .clk(clk), 
            .rst(rst), 
            .set_id(set_id),
            .id_in(id_chain[x+1]), 
            .id(id_chain[x]), 
            .tag(tag), 
            .valid_in(master_valid[x]), 
            .valid_out(valid_chain[x]),
            .ready_in(slave_ready), 
            .ready_out(master_ready[x])
        );
    end
endgenerate

always_comb begin
    slave_data = '0; 
    slave_valid = 1'b0;
    for (int i = 0; i < NUMS_MASTER; i = i + 1) begin
        if (valid_chain[i]) begin
            slave_data = master_data[i * `DATA_BITS +: `DATA_BITS];
            slave_valid = 1'b1;
        end
    end
end
endmodule

// note
// master = PE , slave = GLB
// 每顆 PE 都是 32 bits => 找出 GLB 指定 PE 的 data
// valid_chain 只有一個 bit 會是 1 (tag = id)
// ex : valid_chain[3] = 1 => slave_data = master_data[3 * 32 +: 32]
