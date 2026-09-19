module PE (
    input clk,
    input rst,
    input PE_en,
    input [`CONFIG_SIZE-1:0] i_config,
    input [`DATA_BITS-1:0] ifmap,
    input [`DATA_BITS-1:0] filter,
    input [`DATA_BITS-1:0] ipsum,
    input ifmap_valid,
    input filter_valid,
    input ipsum_valid,
    input opsum_ready,
    output logic [`DATA_BITS-1:0] opsum,
    output logic ifmap_ready,
    output logic filter_ready,
    output logic ipsum_ready,
    output logic opsum_valid
);

// Configuration Registers
logic [2:0] ofmap_ch;   // p = # of output channels produced by a PE set
logic [4:0] ofmap_col;  // F = ofmap plane width
logic [1:0] i_ch;       // q = # of channels processed by a PE set

// SPAD (Scratchpad Memory)
logic [`DATA_BITS-1:0] ifmap_spad  [0:2];                  
logic [`DATA_BITS-1:0] filter_spad [0:`FILTER_SPAD_LEN-1]; 
logic [`DATA_BITS-1:0] ipsum_spad  [0:`OFMAP_SPAD_LEN-1];
logic [`DATA_BITS-1:0] psum_spad   [0:`OFMAP_SPAD_LEN-1];

// FSM State
typedef enum logic [2:0] {
    IDLE, 
    LOAD_FILT, 
    LOAD_IFMAP, 
    LOAD_IPSUM, 
    CALC, 
    OUT
} state_t;

state_t current_state, next_state;

// Pointers and Counters
logic [5:0] filter_wptr;
logic [1:0] ifmap_count;
logic [2:0] ipsum_wptr;
logic [5:0] calc_cnt;
logic [2:0] opsum_rptr;
logic [1:0] ifmap_idx;
logic [1:0] psum_idx;
logic [4:0] win_cnt;    // window index inside the current ofmap row

// ==========================================
// Pipeline Registers for MAC Operation
// ==========================================
logic signed [31:0] dot_product_reg;
logic [1:0] ifmap_idx_d1;
logic [1:0] psum_idx_d1;
logic calc_valid_d1;

always_ff @(posedge clk or posedge rst) begin
    if (rst) current_state <= IDLE;
    else current_state <= next_state;
end

// Ready-Valid Handshake
always_comb begin
    next_state = current_state;
    case (current_state)
        IDLE:       if (PE_en) next_state = LOAD_FILT;
        LOAD_FILT:  if (filter_valid && filter_ready && filter_wptr == 6'(ofmap_ch * 3 - 1)) next_state = LOAD_IFMAP;
        LOAD_IFMAP: if (ifmap_valid && ifmap_ready && ifmap_count == 2) next_state = LOAD_IPSUM;
        LOAD_IPSUM: if (ipsum_valid && ipsum_ready && ipsum_wptr == 3'(ofmap_ch - 1)) next_state = CALC;
        CALC:       if (calc_cnt == 6'(ofmap_ch * 3 - 1)) next_state = OUT;
        OUT:        if (opsum_valid && opsum_ready && opsum_rptr == 3'(ofmap_ch - 1)) next_state = LOAD_IFMAP; 
        default:    next_state = IDLE;
    endcase
end

assign filter_ready = (current_state == LOAD_FILT);
assign ifmap_ready  = (current_state == LOAD_IFMAP);
assign ipsum_ready  = (current_state == LOAD_IPSUM);
// Wait for pipeline stage 2 to add the last product before presenting psum
assign opsum_valid  = (current_state == OUT) && !calc_valid_d1;
assign opsum = psum_spad[opsum_rptr[1:0]];

logic [`DATA_BITS-1:0] ifmap_val;
logic [`DATA_BITS-1:0] filter_val;
assign ifmap_val = ifmap_spad[ifmap_idx];
assign filter_val = filter_spad[calc_cnt];

// Dot Product Combinational Logic (Stage 1)
logic signed [31:0] dot_product;
logic signed [7:0] ifmap_v;
logic signed [7:0] filt_v;

always_comb begin
    dot_product = '0; 
    for (int i = 0; i < 4; i++) begin
        ifmap_v = { ~ifmap_val[i*8+7], ifmap_val[i*8 +: 7] };
        filt_v  = $signed(filter_val[i*8 +: 8]);
        dot_product += (ifmap_v * filt_v);
    end
end

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        ofmap_ch    <= 0;
        ofmap_col   <= 0;
        i_ch        <= 0;
        filter_wptr <= 0;
        ifmap_count <= 0;
        ipsum_wptr  <= 0;
        calc_cnt    <= 0;
        opsum_rptr  <= 0;
        ifmap_idx   <= 0;
        psum_idx    <= 0;
        win_cnt     <= 0;

        // Reset Pipeline Registers
        dot_product_reg <= '0;
        ifmap_idx_d1    <= '0;
        psum_idx_d1     <= '0;
        calc_valid_d1   <= 1'b0;

        for (int i = 0; i < `OFMAP_SPAD_LEN; i++) psum_spad[i] <= 0;
        for (int i = 0; i < 3; i++) ifmap_spad[i] <= 0;
    end else begin
        // Read Config
        if (current_state == IDLE && PE_en) begin
            ofmap_ch  <= i_config[8:7] + 1; 
            ofmap_col <= i_config[6:2] + 1; 
            i_ch      <= i_config[1:0] + 1; 
            filter_wptr <= 0;
            ifmap_count <= 0;
            win_cnt     <= 0;
        end

        // Load Filter
        if (current_state == LOAD_FILT && filter_valid && filter_ready) begin
            filter_spad[filter_wptr] <= filter;
            filter_wptr <= filter_wptr + 1;
        end
        
        // Data Reuse via Shift Register
        if (current_state == LOAD_IFMAP && ifmap_valid && ifmap_ready) begin
            ifmap_spad[0] <= ifmap_spad[1];
            ifmap_spad[1] <= ifmap_spad[2];
            ifmap_spad[2] <= ifmap;
            ifmap_count <= ifmap_count + 1;
        end

        // Load Input Partial Sum
        if (current_state == LOAD_IPSUM && ipsum_valid && ipsum_ready) begin
            ipsum_spad[ipsum_wptr[1:0]] <= ipsum;
            ipsum_wptr <= ipsum_wptr + 1;
        end

        // ==========================================
        // Pipeline Stage 1: Multiplication & Indexing
        // ==========================================
        if (current_state == CALC) begin
            dot_product_reg <= dot_product;
            ifmap_idx_d1    <= ifmap_idx;
            psum_idx_d1     <= psum_idx;
            calc_valid_d1   <= 1'b1;

            calc_cnt <= calc_cnt + 1;
            if (ifmap_idx == 2) begin
                ifmap_idx <= 0;
                psum_idx <= psum_idx + 1;
            end else begin
                ifmap_idx <= ifmap_idx + 1;
            end
        end else begin
            calc_cnt      <= 0;
            ifmap_idx     <= 0;
            psum_idx      <= 0;
            calc_valid_d1 <= 1'b0;
        end

        // ==========================================
        // Pipeline Stage 2: Accumulation
        // ==========================================
        if (calc_valid_d1) begin
            if (ifmap_idx_d1 == 0)
                psum_spad[psum_idx_d1[1:0]] <= ipsum_spad[psum_idx_d1[1:0]] + dot_product_reg;
            else
                psum_spad[psum_idx_d1[1:0]] <= psum_spad[psum_idx_d1[1:0]] + dot_product_reg;
        end

        // Output Psum
        if (current_state == OUT && opsum_valid && opsum_ready) begin
            if (opsum_rptr == 3'(ofmap_ch - 1)) begin
                opsum_rptr <= 0;
                ipsum_wptr <= 0;
                // Last window of the row (F windows): next row needs 3 fresh
                // ifmaps, otherwise slide by one ifmap
                if (win_cnt == ofmap_col - 1) begin
                    win_cnt     <= 0;
                    ifmap_count <= 0;
                end else begin
                    win_cnt     <= win_cnt + 1;
                    ifmap_count <= 2;
                end
            end else begin
                opsum_rptr <= opsum_rptr + 1;
            end
        end
    end
end

endmodule