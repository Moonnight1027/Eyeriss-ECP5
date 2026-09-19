
module GON (
    input clk, 
    input rst,
    output logic GON_valid, 
    input GON_ready, 
    output logic [`DATA_BITS-1:0] GON_data,
    input [`XID_BITS-1:0] tag_X, 
    input [`YID_BITS-1:0] tag_Y,
    input set_XID, 
    input [`XID_BITS - 1:0] XID_scan_in,
    input set_YID, 
    input [`YID_BITS - 1:0] YID_scan_in,
    input [`NUMS_PE_ROW * `NUMS_PE_COL - 1:0] PE_valid,
    output logic [`NUMS_PE_ROW * `NUMS_PE_COL - 1:0] PE_ready,
    input [`DATA_BITS * `NUMS_PE_ROW * `NUMS_PE_COL - 1:0] PE_data
);


logic [`YID_BITS-1:0] yid_chain [0:`NUMS_PE_ROW];
assign yid_chain[`NUMS_PE_ROW] = YID_scan_in;


logic [`XID_BITS-1:0] xid_chain [0:`NUMS_PE_ROW];
assign xid_chain[`NUMS_PE_ROW] = XID_scan_in;

logic [`NUMS_PE_ROW-1:0] row_valid, row_ready, y_valid_chain;
logic [`DATA_BITS-1:0]   row_data [`NUMS_PE_ROW];

genvar y;
generate
    for (y = 0; y < `NUMS_PE_ROW; y = y + 1) begin : gen_row
        GON_MulticastController #(.ID_SIZE(`YID_BITS)) Y_MC (
            .clk(clk), 
            .rst(rst), 
            .set_id(set_YID),
            .id_in(yid_chain[y+1]), 
            .id(yid_chain[y]), 
            .tag(tag_Y), 
            .valid_in(row_valid[y]), 
            .valid_out(y_valid_chain[y]),
            .ready_in(GON_ready), 
            .ready_out(row_ready[y])
        );

        GON_Bus #(.NUMS_MASTER(`NUMS_PE_COL), .ID_SIZE(`XID_BITS)) ROW_BUS (
            .clk(clk), 
            .rst(rst), 
            .tag(tag_X),
            .master_valid(PE_valid[y * `NUMS_PE_COL +: `NUMS_PE_COL]),
            .master_data(PE_data[y * `NUMS_PE_COL * `DATA_BITS +: `NUMS_PE_COL * `DATA_BITS]),
            .master_ready(PE_ready[y * `NUMS_PE_COL +: `NUMS_PE_COL]),
            .slave_valid(row_valid[y]), 
            .slave_ready(row_ready[y]), 
            .slave_data(row_data[y]),
            .set_id(set_XID),
            .ID_scan_in(xid_chain[y+1]),
            .ID_scan_out(xid_chain[y])
        );
    end
endgenerate

always_comb begin
    GON_data = '0; 
    GON_valid = 1'b0;
    for (int i = 0; i < `NUMS_PE_ROW; i = i + 1) begin
        if (y_valid_chain[i]) begin
            GON_data = row_data[i];
            GON_valid = 1'b1;
        end
    end
end

endmodule

// note
// master = PE , slave = GLB
// row_ready[y] , y_valid_chain[y]控制該 row 的 PE 是否可以傳資料給 GLB

