

module PE_array #(
    parameter NUMS_PE_ROW = `NUMS_PE_ROW,
    parameter NUMS_PE_COL = `NUMS_PE_COL,
    parameter XID_BITS = `XID_BITS,
    parameter YID_BITS = `YID_BITS,
    parameter DATA_SIZE = `DATA_BITS,
    parameter CONFIG_SIZE = `CONFIG_SIZE
)(
    input clk, 
    input rst,

    input set_XID, 
    input [`XID_BITS-1:0] ifmap_XID_scan_in, 
    input [`XID_BITS-1:0] filter_XID_scan_in, 
    input [`XID_BITS-1:0] ipsum_XID_scan_in, 
    input [`XID_BITS-1:0] opsum_XID_scan_in,

    input set_YID, 
    input [`YID_BITS-1:0] ifmap_YID_scan_in, 
    input [`YID_BITS-1:0] filter_YID_scan_in, 
    input [`YID_BITS-1:0] ipsum_YID_scan_in, 
    input [`YID_BITS-1:0] opsum_YID_scan_in,

    input set_LN, 
    input [`NUMS_PE_ROW-2:0] LN_config_in,

    input [`NUMS_PE_ROW*`NUMS_PE_COL-1:0] PE_en, 
    input [`CONFIG_SIZE-1:0] PE_config,
    input [`XID_BITS-1:0] ifmap_tag_X, 
    input [`YID_BITS-1:0] ifmap_tag_Y, 
    input [`XID_BITS-1:0] filter_tag_X, 
    input [`YID_BITS-1:0] filter_tag_Y, 
    input [`XID_BITS-1:0] ipsum_tag_X, 
    input [`YID_BITS-1:0] ipsum_tag_Y, 
    input [`XID_BITS-1:0] opsum_tag_X, 
    input [`YID_BITS-1:0] opsum_tag_Y,

    input GLB_ifmap_valid, 
    output logic GLB_ifmap_ready, 
    input GLB_filter_valid, 
    output logic GLB_filter_ready, 
    input GLB_ipsum_valid, 
    output logic GLB_ipsum_ready, 
    input [DATA_SIZE-1:0] GLB_data_in,
    output logic GLB_opsum_valid, 
    input GLB_opsum_ready, 
    output logic [DATA_SIZE-1:0] GLB_data_out
);

localparam NUMS_PE = NUMS_PE_ROW * NUMS_PE_COL;

logic [NUMS_PE-1:0] PE_ifmap_valid, PE_ifmap_ready;
logic [NUMS_PE-1:0] PE_filter_valid, PE_filter_ready;
logic [NUMS_PE-1:0] PE_ipsum_valid, PE_ipsum_ready;
logic [DATA_SIZE-1:0] PE_ifmap_data, PE_filter_data, PE_ipsum_data;
logic [NUMS_PE-1:0] PE_opsum_valid, PE_opsum_ready;
logic [NUMS_PE*DATA_SIZE-1:0] PE_opsum_data;

GIN ifmap_GIN (.clk(clk), .rst(rst), .GIN_valid(GLB_ifmap_valid), .GIN_ready(GLB_ifmap_ready), .GIN_data(GLB_data_in), .tag_X(ifmap_tag_X), .tag_Y(ifmap_tag_Y), .set_XID(set_XID), .XID_scan_in(ifmap_XID_scan_in), .set_YID(set_YID), .YID_scan_in(ifmap_YID_scan_in), .PE_ready(PE_ifmap_ready), .PE_valid(PE_ifmap_valid), .PE_data(PE_ifmap_data));
GIN filter_GIN (.clk(clk), .rst(rst), .GIN_valid(GLB_filter_valid), .GIN_ready(GLB_filter_ready), .GIN_data(GLB_data_in), .tag_X(filter_tag_X), .tag_Y(filter_tag_Y), .set_XID(set_XID), .XID_scan_in(filter_XID_scan_in), .set_YID(set_YID), .YID_scan_in(filter_YID_scan_in), .PE_ready(PE_filter_ready), .PE_valid(PE_filter_valid), .PE_data(PE_filter_data));
GIN ipsum_GIN (.clk(clk), .rst(rst), .GIN_valid(GLB_ipsum_valid), .GIN_ready(GLB_ipsum_ready), .GIN_data(GLB_data_in), .tag_X(ipsum_tag_X), .tag_Y(ipsum_tag_Y), .set_XID(set_XID), .XID_scan_in(ipsum_XID_scan_in), .set_YID(set_YID), .YID_scan_in(ipsum_YID_scan_in), .PE_ready(PE_ipsum_ready), .PE_valid(PE_ipsum_valid), .PE_data(PE_ipsum_data));
GON opsum_GON (.clk(clk), .rst(rst), .GON_valid(GLB_opsum_valid), .GON_ready(GLB_opsum_ready), .GON_data(GLB_data_out), .tag_X(opsum_tag_X), .tag_Y(opsum_tag_Y), .set_XID(set_XID), .XID_scan_in(opsum_XID_scan_in), .set_YID(set_YID), .YID_scan_in(opsum_YID_scan_in), .PE_valid(PE_opsum_valid), .PE_ready(PE_opsum_ready), .PE_data(PE_opsum_data));

// 控制 6 row 之間的連結
logic [`NUMS_PE_ROW-2:0] ln_cfg;
always_ff @(posedge clk or posedge rst) begin
    if (rst) ln_cfg <= 0;
    else if (set_LN) ln_cfg <= LN_config_in;
end

logic [NUMS_PE-1:0] pe_out_valid;
logic [DATA_SIZE-1:0] pe_out_data [0:NUMS_PE-1];
logic [NUMS_PE-1:0] pe_in_ready;

genvar x, y;
generate
    for (y = 0; y < NUMS_PE_ROW; y = y + 1) begin : gen_pe_row
        for (x = 0; x < NUMS_PE_COL; x = x + 1) begin : gen_pe_col
            localparam int idx = y * NUMS_PE_COL + x;
            
            logic current_ipsum_valid;
            logic current_ipsum_ready;
            logic [DATA_SIZE-1:0] current_ipsum_data;
            logic current_opsum_ready;
            
            // Row 0 => ipsum 來自 GIN
            if (y == 0) begin
                assign current_ipsum_valid = PE_ipsum_valid[idx];
                assign current_ipsum_data  = PE_ipsum_data;
                assign PE_ipsum_ready[idx] = current_ipsum_ready;
            end else begin
            // ln_cfg[y] = 1 => 第 y row 的 PE 從 GIN 接收資料，但將結果傳給下一 row 的 PE，而非 GON
            // 同時，第 y+1 row 的 PE 會停止從 GIN 接收資料，改吃第 y row 下來的資料
                assign current_ipsum_valid = ln_cfg[y-1] ? pe_out_valid[(y-1)*NUMS_PE_COL + x] : PE_ipsum_valid[idx];
                assign current_ipsum_data  = ln_cfg[y-1] ? pe_out_data[(y-1)*NUMS_PE_COL + x]  : PE_ipsum_data;
                assign PE_ipsum_ready[idx] = ln_cfg[y-1] ? 1'b0 : current_ipsum_ready;
            end
            
            // Row 5 => opsum 送到 GON
            if (y == NUMS_PE_ROW - 1) begin
                assign PE_opsum_valid[idx] = pe_out_valid[idx];
                assign current_opsum_ready = PE_opsum_ready[idx];
            end else begin
                assign PE_opsum_valid[idx] = ln_cfg[y] ? 1'b0 : pe_out_valid[idx];
                assign current_opsum_ready = ln_cfg[y] ? pe_in_ready[(y+1)*NUMS_PE_COL + x] : PE_opsum_ready[idx];
            end
            
            assign PE_opsum_data[idx*DATA_SIZE +: DATA_SIZE] = pe_out_data[idx];
            assign pe_in_ready[idx] = current_ipsum_ready;

            PE pe_inst (
                .clk(clk), 
                .rst(rst), 
                .PE_en(PE_en[idx]), 
                .i_config(PE_config),
                .ifmap(PE_ifmap_data), 
                .filter(PE_filter_data), 
                .ipsum(current_ipsum_data),
                .ifmap_valid(PE_ifmap_valid[idx]), 
                .filter_valid(PE_filter_valid[idx]), 
                .ipsum_valid(current_ipsum_valid), 
                .opsum_ready(current_opsum_ready),
                .opsum(pe_out_data[idx]), 
                .ifmap_ready(PE_ifmap_ready[idx]), 
                .filter_ready(PE_filter_ready[idx]), 
                .ipsum_ready(current_ipsum_ready), 
                .opsum_valid(pe_out_valid[idx])
            );
        end
    end
endgenerate

endmodule

// note
// ln_cfg 控制 6 row 之間的連結
// 模式A : ln_cfg = 0 => PE 只能從 GIN 接收資料，將結果傳回 GON
// 模式B : ln_cfg[y] = 1 => 第 y row 的 PE 從 GIN 接收資料，但將結果傳給下一 row 的 PE，而非 GON
// 同時，第 y+1 row 的 PE 會停止從 GIN 接收資料，改吃第 y row 下來的資料
// 如果是模式B，則第 y row 的 PE_opsum_valid 永遠不會傳給 GON，而是給下一 row 的 PE_ipsum_valid