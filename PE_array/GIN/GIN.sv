
module GIN (
    input clk, 
    input rst,
    input GIN_valid, 
    output logic GIN_ready, 
    input [`DATA_BITS - 1:0] GIN_data,
    input [`XID_BITS - 1:0] tag_X, 
    input [`YID_BITS - 1:0] tag_Y,
    input set_XID, 
    input [`XID_BITS - 1:0] XID_scan_in,
    input set_YID, 
    input [`YID_BITS - 1:0] YID_scan_in,
    input [`NUMS_PE_ROW * `NUMS_PE_COL - 1:0] PE_ready,
    output logic [`NUMS_PE_ROW * `NUMS_PE_COL - 1:0] PE_valid,
    output logic [`DATA_BITS - 1:0] PE_data
);

assign PE_data = GIN_data;

logic [`YID_BITS-1:0] yid_chain[0:`NUMS_PE_ROW];
assign yid_chain[`NUMS_PE_ROW] = YID_scan_in;

logic [`XID_BITS-1:0] xid_chain[0:`NUMS_PE_ROW];
assign xid_chain[`NUMS_PE_ROW] = XID_scan_in;

logic [`NUMS_PE_ROW-1:0] row_valid, row_ready, y_ready_chain;
assign GIN_ready = &y_ready_chain;

genvar y;
generate
    for (y = 0; y < `NUMS_PE_ROW; y = y + 1) begin : gen_row
        GIN_MulticastController #(.ID_SIZE(`YID_BITS)) Y_MC (
            .clk(clk), 
            .rst(rst), 
            .set_id(set_YID),
            .id_in(yid_chain[y+1]), 
            .id(yid_chain[y]),
            .tag(tag_Y), 
            .valid_in(GIN_valid), 
            .valid_out(row_valid[y]),
            .ready_in(row_ready[y]), 
            .ready_out(y_ready_chain[y])
        );

        GIN_Bus #(.NUMS_SLAVE(`NUMS_PE_COL), .ID_SIZE(`XID_BITS)) ROW_BUS (
            .clk(clk), 
            .rst(rst), 
            .tag(tag_X),
            .master_valid(row_valid[y]), 
            .master_data(GIN_data), 
            .master_ready(row_ready[y]),
            .slave_ready(PE_ready[y*`NUMS_PE_COL+:`NUMS_PE_COL]),
            .slave_valid(PE_valid[y*`NUMS_PE_COL+:`NUMS_PE_COL]),
            .slave_data(), 
            .set_id(set_XID),
            .ID_scan_in(xid_chain[y+1]), 
            .ID_scan_out(xid_chain[y]) 
        );
    end
endgenerate

endmodule

// note
// row = GLB (valid) -> bus (ready), slave = bus (valid) -> PE (ready)
// 先看 tag_Y ，若不符合 => 整條 Bus 都不會傳資料
// .slave_ready(PE_ready[y*`NUMS_PE_COL+:`NUMS_PE_COL]),
// .slave_valid(PE_valid[y*`NUMS_PE_COL+:`NUMS_PE_COL]),
//  y = 2 , NUMS_PE_COL = 4 => PE_ready[2*4+:4] => PE_ready[8 +: 4] => PE_ready[11:8] 

/*
[ GLB ] --GIN_valid-->  [ Y_MC ] --row_valid[y]--> [ ROW_BUS ] --PE_valid--> [ 第 y 層的 8 顆 PE ]
                                 <--row_ready[y]--             <--PE_ready--
*/