`include "define.svh"

// ============================================================================
// Eyeriss_SoC_Top
//   USB CDC  <->  controller  <->  GLB_SRAM  <->  PE_array (GIN/GON)  ->  PPU
//
// Multi-channel 3x3 convolution layer (stride 1, no padding), W x W ifmap.
//   * One SRAM word = 4 packed int8 channels (Eyeriss q = 4). Input channels
//     are processed in G groups of 4; each PE computes P (1..4) filters at once.
//   * PE k (k = 0,1,2) holds filter row k and is addressed through its
//     programmed (XID, YID) with the GIN/GON tags.
//   * For every output pixel the P psums travel through the GLB:
//       PE0(ifmap row y  , ipsum = bias or psum of previous group) -> carry
//       PE1(ifmap row y+1, ipsum = carry)                           -> carry
//       PE2(ifmap row y+2, ipsum = carry) -> psum (SRAM) / PPU (last group)
//     Between groups the PE array is reset and reloaded with the next
//     group's filters; the running psum stays in the GLB.
//
// Host protocol:
//   0x01 LEN_L LEN_H bytes...      : legacy, write bytes (1 byte / word) from address 0
//   0x02                           : run, reply P*F*F ofmap bytes (F = W - 2),
//                                    pixel row-major, channels 0..P-1 per pixel
//   0x03 W SHIFT                   : legacy config (G=1, P=1, ReLU on, no bias)
//   0x04 ADR_L ADR_H N_L N_H words : write N little-endian 32-bit words at ADR
//   0x05 ADR_L ADR_H N_L N_H       : read N words at ADR (4N bytes, little-endian)
//   0x06 W G P SHIFT FLAGS         : layer config, replies 0x06 (ok) / 0xEE (rejected)
//                                    W 3..32, G 1..16, P 1..4, SHIFT 0..31
//                                    FLAGS[0] = ReLU enable, FLAGS[1] = bias enable
//   other byte                     : echo (used by 0xFF PING)
//
// SRAM map (words):
//   0                 : ifmap,  G * W * W   (group, row, col)
//   filter_base       : filter, G * P * 9   (group, filter, row, tap)
//   bias_base         : bias,   P           (int32)
//   psum_base         : psum / ofmap, P * F * F
//   Total must fit in 2**ADDR_W words, otherwise 0x06 is rejected.
// ============================================================================
module Eyeriss_SoC_Top (
    input  logic clk,           // 48 MHz
    input  logic rst,           // board button (currently unused, see note at bottom)
    inout  logic usb_d_p,
    inout  logic usb_d_n,
    output logic usb_pullup,
    output logic led
);

    // ------------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------------
    localparam int KSIZE     = `FILT_R;   // 3, fixed by the PE ifmap shift register
    localparam int ADDR_W    = 13;
    localparam int MAX_W     = 32;        // ofmap_col is 5 bits
    localparam int MAX_G     = 16;
    localparam int MAX_P     = `OFMAP_SPAD_LEN;
    localparam int MEM_WORDS = 1 << ADDR_W;

    localparam logic [7:0] CMD_WRITE_SRAM = 8'h01;
    localparam logic [7:0] CMD_START_MAC  = 8'h02;
    localparam logic [7:0] CMD_CONFIG     = 8'h03;
    localparam logic [7:0] CMD_WRITE_WORD = 8'h04;
    localparam logic [7:0] CMD_READ_WORD  = 8'h05;
    localparam logic [7:0] CMD_LAYER_CFG  = 8'h06;
    localparam logic [7:0] RSP_ACK        = 8'h06;
    localparam logic [7:0] RSP_NACK       = 8'hEE;

    localparam int SOFT_RST_CYCLES = 4;
    localparam logic [25:0] MAC_TIMEOUT = 26'd48_000_000;   // 1 s @ 48 MHz

    // ------------------------------------------------------------------------
    // Controller state (declared first so every block can reference it)
    // ------------------------------------------------------------------------
    localparam logic [4:0]
        S_WAIT_CMD   = 5'd0,
        S_ECHO       = 5'd1,
        S_RECV_HDR   = 5'd2,    // address / length bytes of 0x01, 0x04, 0x05
        S_RECV_BURST = 5'd3,
        S_CFG_ARGS   = 5'd4,    // argument bytes of 0x03 / 0x06
        S_CFG_CALC   = 5'd5,    // derive layout with a serial multiplier
        S_CFG_COMMIT = 5'd6,
        S_READ_FETCH = 5'd7,
        S_READ_WAIT  = 5'd8,
        S_READ_TX    = 5'd9,
        // MAC run: states S_MAC_RESET .. S_GROUP_END are covered by the timeout
        S_MAC_RESET  = 5'd10,   // soft-reset PE array
        S_SET_ID     = 5'd11,   // shift XID/YID into the scan chains
        S_FETCH      = 5'd12,   // set SRAM read address / tag for current feed
        S_WAIT       = 5'd13,   // wait 1 cycle for synchronous SRAM read
        S_DRIVE      = 5'd14,   // hold valid until PE array accepts (ready)
        S_COLLECT    = 5'd15,   // wait for P opsums of the addressed PE
        S_WRITE_OUT  = 5'd16,   // P psums (or PPU results) -> SRAM
        S_GROUP_END  = 5'd17;

    // Feed phases
    localparam logic [1:0] PH_FILTER = 2'd0,
                           PH_IFMAP  = 2'd1,
                           PH_IPSUM  = 2'd2;

    logic [4:0] state;
    logic [1:0] phase;

    // ------------------------------------------------------------------------
    // 0. Power-on reset & heartbeat LED
    // ------------------------------------------------------------------------
    logic [7:0]  por_cnt   = '0;
    logic        sys_rst;
    logic [24:0] blink_cnt = '0;
    logic        err_flag;

    always_ff @(posedge clk) begin
        if (por_cnt != 8'hFF) por_cnt <= por_cnt + 1'b1;
        blink_cnt <= blink_cnt + 1'b1;
    end

    assign sys_rst    = (por_cnt != 8'hFF);
    assign usb_pullup = 1'b1;
    // Slow blink = alive, fast blink = last MAC run timed out
    assign led        = err_flag ? blink_cnt[21] : blink_cnt[24];

    // ------------------------------------------------------------------------
    // 1. USB CDC
    // ------------------------------------------------------------------------
    logic [7:0] rx_data, tx_data;
    logic       rx_valid, rx_ready, tx_start, tx_ready;

    // Back-pressure: only accept bytes in states that consume them, so nothing
    // is dropped while the controller is busy (the USB core NAKs the host)
    assign rx_ready = (state == S_WAIT_CMD)   || (state == S_RECV_HDR) ||
                      (state == S_RECV_BURST) || (state == S_CFG_ARGS);

    usb_uart_np u_usb_uart (
        .clk_48mhz      (clk),
        .reset          (sys_rst),
        .pin_usb_p      (usb_d_p),
        .pin_usb_n      (usb_d_n),
        .uart_out_data  (rx_data),
        .uart_out_valid (rx_valid),
        .uart_out_ready (rx_ready),
        .uart_in_data   (tx_data),
        .uart_in_valid  (tx_start),
        .uart_in_ready  (tx_ready),
        .debug          ()
    );

    // ------------------------------------------------------------------------
    // 2. Global buffer (SRAM, 1-cycle synchronous read)
    // ------------------------------------------------------------------------
    logic              glb_we;
    logic [ADDR_W-1:0] glb_waddr, glb_raddr;
    logic [31:0]       glb_wdata, glb_rdata;

    GLB_SRAM #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_W)) u_glb (
        .clk   (clk),
        .we    (glb_we),
        .waddr (glb_waddr),
        .raddr (glb_raddr),
        .wdata (glb_wdata),
        .rdata (glb_rdata)
    );

    // ------------------------------------------------------------------------
    // 3. Configuration (committed values)
    // ------------------------------------------------------------------------
    logic [5:0]        cfg_w;          // ifmap width W
    logic [4:0]        cfg_g;          // input channel groups G
    logic [2:0]        cfg_p;          // filters per run P
    logic [4:0]        cfg_shift;      // PPU scaling factor
    logic              cfg_relu;
    logic              cfg_bias;
    logic [ADDR_W-1:0] cfg_ww;         // W*W
    logic [ADDR_W-1:0] cfg_w2;         // 2*W
    logic [5:0]        cfg_p9;         // 9*P
    logic [4:0]        cfg_f_m1;       // F - 1 = W - 3
    logic [ADDR_W-1:0] filter_base;
    logic [ADDR_W-1:0] bias_base;
    logic [ADDR_W-1:0] psum_base;
    logic [ADDR_W:0]   ofmap_words;    // P*F*F

    // PE config: [8:7]=p-1 (filters), [6:2]=F-1 (ofmap width), [1:0]=q-1 (channels)
    logic [`CONFIG_SIZE-1:0] pe_config;
    assign pe_config = {1'b0, 2'(cfg_p - 3'd1), cfg_f_m1, 2'd3};

    // Candidate values while a config command is being evaluated
    logic [5:0]  n_w;
    logic [4:0]  n_g;
    logic [2:0]  n_p;
    logic [4:0]  n_shift;
    logic        n_relu, n_bias, n_ok, n_ack;
    logic [15:0] n_ww, n_ifw, n_ff, n_ofw, n_gp9;
    logic [15:0] n_total;
    logic [5:0]  n_p9;
    logic [5:0]  n_f;
    logic [2:0]  arg_idx;
    logic [2:0]  calc_step;
    logic [15:0] mul_a, mul_acc;
    logic [5:0]  mul_b;

    assign n_p9    = {n_p, 3'd0} + 6'(n_p);
    assign n_f     = n_w - 6'd2;
    assign n_total = n_ifw + n_gp9 + 16'(n_p) + n_ofw;

    // ------------------------------------------------------------------------
    // 4. PE array
    //    PE k -> (XID, YID): k=0 -> (0,0), k=1 -> (1,0), k=2 -> (0,1)
    //    Scan order (both chains shift simultaneously):
    //      XID: scan_in -> (y1,x1) -> (y1,x0) -> (y0,x1) -> (y0,x0)
    //      YID: scan_in -> row1 -> row0
    //    Shifting 0,1,0,1 (X) and 0,1 (Y) yields XID = x, YID = y.
    // ------------------------------------------------------------------------
    localparam int NUMS_PE = `NUMS_PE_ROW * `NUMS_PE_COL;

    logic [1:0]  k_cnt;         // PE / filter row index
    logic [1:0]  c_cnt;         // filter (output channel) index
    logic [1:0]  j_cnt;         // tap index inside a row
    logic [4:0]  g_cnt;         // input channel group
    logic [4:0]  y_cnt, x_cnt;  // output pixel
    logic [1:0]  id_cnt;
    logic        pe_soft_rst;
    logic        pe_rst;
    logic [NUMS_PE-1:0] pe_en;

    logic        set_xid, set_yid;
    logic [`XID_BITS-1:0] xid_scan, tag_x;
    logic [`YID_BITS-1:0] yid_scan, tag_y;

    assign pe_rst   = sys_rst | pe_soft_rst;
    assign set_xid  = (state == S_SET_ID);
    assign set_yid  = (state == S_SET_ID) && !id_cnt[1];
    assign xid_scan = `XID_BITS'(id_cnt[0]);
    assign yid_scan = `YID_BITS'(id_cnt[0]);
    assign tag_x    = `XID_BITS'(k_cnt == 2'd1);
    assign tag_y    = `YID_BITS'(k_cnt == 2'd2);

    logic        glb_ifmap_valid,  glb_ifmap_ready;
    logic        glb_filter_valid, glb_filter_ready;
    logic        glb_ipsum_valid,  glb_ipsum_ready;
    logic        glb_opsum_valid,  glb_opsum_ready;
    logic        feed_ready;
    logic [31:0] pe_data_in;
    logic [31:0] pe_data_out;
    logic [31:0] carry [0:MAX_P-1];   // psums passed between PEs through the GLB
    logic        first_group, last_group, last_c;

    assign first_group = (g_cnt == 5'd0);
    assign last_group  = (g_cnt == cfg_g - 5'd1);
    assign last_c      = (c_cnt == 2'(cfg_p - 3'd1));

    // Valid is held while in DRIVE; SRAM data is stable because raddr is held
    assign glb_filter_valid = (state == S_DRIVE) && (phase == PH_FILTER);
    assign glb_ifmap_valid  = (state == S_DRIVE) && (phase == PH_IFMAP);
    assign glb_ipsum_valid  = (state == S_DRIVE) && (phase == PH_IPSUM);
    assign glb_opsum_ready  = (state == S_COLLECT);

    always_comb begin
        case (phase)
            PH_FILTER: feed_ready = glb_filter_ready;
            PH_IFMAP:  feed_ready = glb_ifmap_ready;
            PH_IPSUM:  feed_ready = glb_ipsum_ready;
            default:   feed_ready = 1'b0;
        endcase
    end

    // ipsum of PE0: bias (first group) or the stored psum (later groups)
    // ipsum of PE1/PE2: psum of the PE above
    always_comb begin
        if (phase != PH_IPSUM)
            pe_data_in = glb_rdata;
        else if (k_cnt != 2'd0)
            pe_data_in = carry[c_cnt];
        else if (first_group && !cfg_bias)
            pe_data_in = 32'd0;
        else
            pe_data_in = glb_rdata;
    end

    PE_array #(
        .NUMS_PE_ROW (`NUMS_PE_ROW),
        .NUMS_PE_COL (`NUMS_PE_COL)
    ) u_pe_array (
        .clk                (clk),
        .rst                (pe_rst),
        .PE_en              (pe_en),
        .PE_config          (pe_config),

        .GLB_ifmap_valid    (glb_ifmap_valid),
        .GLB_ifmap_ready    (glb_ifmap_ready),
        .GLB_filter_valid   (glb_filter_valid),
        .GLB_filter_ready   (glb_filter_ready),
        .GLB_ipsum_valid    (glb_ipsum_valid),
        .GLB_ipsum_ready    (glb_ipsum_ready),
        .GLB_data_in        (pe_data_in),
        .GLB_opsum_valid    (glb_opsum_valid),
        .GLB_opsum_ready    (glb_opsum_ready),
        .GLB_data_out       (pe_data_out),

        .set_XID            (set_xid),
        .set_YID            (set_yid),
        .set_LN             (1'b0),
        .LN_config_in       ('0),
        .ifmap_XID_scan_in  (xid_scan), .filter_XID_scan_in(xid_scan),
        .ipsum_XID_scan_in  (xid_scan), .opsum_XID_scan_in (xid_scan),
        .ifmap_YID_scan_in  (yid_scan), .filter_YID_scan_in(yid_scan),
        .ipsum_YID_scan_in  (yid_scan), .opsum_YID_scan_in (yid_scan),
        .ifmap_tag_X        (tag_x),    .ifmap_tag_Y       (tag_y),
        .filter_tag_X       (tag_x),    .filter_tag_Y      (tag_y),
        .ipsum_tag_X        (tag_x),    .ipsum_tag_Y       (tag_y),
        .opsum_tag_X        (tag_x),    .opsum_tag_Y       (tag_y)
    );

    // ------------------------------------------------------------------------
    // 5. PPU (combinational, fed from the carry register of the current filter)
    // ------------------------------------------------------------------------
    logic [7:0] ppu_data_out;

    PPU u_ppu (
        .clk            (clk),
        .rst            (sys_rst),
        .data_in        (carry[c_cnt]),
        .scaling_factor ({1'b0, cfg_shift}),
        .maxpool_en     (1'b0),
        .maxpool_init   (1'b0),
        .relu_sel       (1'b1),
        .relu_en        (cfg_relu),
        .data_out       (ppu_data_out)
    );

    // ------------------------------------------------------------------------
    // 6. Main controller
    // ------------------------------------------------------------------------
    logic [7:0]        cmd;
    logic [2:0]        hdr_idx;
    logic [15:0]       xfer_len, xfer_cnt;   // words
    logic [ADDR_W-1:0] xfer_addr;
    logic [1:0]        byte_idx;
    logic              word_mode;            // 4 bytes per word (else 1)
    logic [23:0]       word_shift;
    logic [2:0]        soft_rst_cnt;
    logic [25:0]       mac_timer;
    logic [ADDR_W-1:0] group_base;   // g * W * W
    logic [ADDR_W-1:0] gfilt_base;   // filter_base + g * 9P
    logic [ADDR_W-1:0] row_base;     // group_base + y * W
    logic [ADDR_W-1:0] pix_base;     // psum_base + pixel * P
    logic              last_col, last_row;
    logic [1:0]        last_byte;

    assign last_col  = (x_cnt == cfg_f_m1);
    assign last_row  = (y_cnt == cfg_f_m1);
    assign last_byte = word_mode ? 2'd3 : 2'd0;

    // Address of ifmap tap j for PE k at output pixel (y, x)
    // First column loads 3 fresh taps (x + j), later columns slide by one (x + 2)
    logic [ADDR_W-1:0] ifmap_addr;
    always_comb begin
        ifmap_addr = row_base + ((x_cnt == 5'd0) ? ADDR_W'(j_cnt) : ADDR_W'(x_cnt) + 2);
        case (k_cnt)
            2'd1:    ifmap_addr = ifmap_addr + ADDR_W'(cfg_w);
            2'd2:    ifmap_addr = ifmap_addr + cfg_w2;
            default: ;
        endcase
    end

    // Filter word: group base + 9 * c + 3 * k + j (shift-add, no DSP)
    logic [ADDR_W-1:0] filter_addr;
    assign filter_addr = gfilt_base
                       + (ADDR_W'(c_cnt) << 3) + ADDR_W'(c_cnt)
                       + (ADDR_W'(k_cnt) << 1) + ADDR_W'(k_cnt)
                       + ADDR_W'(j_cnt);

    always_ff @(posedge clk) begin
        if (sys_rst) begin
            state        <= S_WAIT_CMD;
            phase        <= PH_FILTER;
            glb_we       <= 1'b0;
            glb_waddr    <= '0;
            glb_wdata    <= '0;
            glb_raddr    <= '0;
            pe_en        <= '0;
            pe_soft_rst  <= 1'b0;
            tx_start     <= 1'b0;
            tx_data      <= '0;
            cmd          <= '0;
            hdr_idx      <= '0;
            xfer_len     <= '0;
            xfer_cnt     <= '0;
            xfer_addr    <= '0;
            byte_idx     <= '0;
            word_mode    <= 1'b0;
            word_shift   <= '0;
            soft_rst_cnt <= '0;
            mac_timer    <= '0;
            // Default layout: legacy W=5, G=1, P=1
            cfg_w        <= 6'd5;
            cfg_g        <= 5'd1;
            cfg_p        <= 3'd1;
            cfg_shift    <= '0;
            cfg_relu     <= 1'b1;
            cfg_bias     <= 1'b0;
            cfg_ww       <= ADDR_W'(25);
            cfg_w2       <= ADDR_W'(10);
            cfg_p9       <= 6'd9;
            cfg_f_m1     <= 5'd2;
            filter_base  <= ADDR_W'(25);
            bias_base    <= ADDR_W'(34);
            psum_base    <= ADDR_W'(35);
            ofmap_words  <= (ADDR_W+1)'(9);
            n_w <= '0; n_g <= '0; n_p <= '0; n_shift <= '0;
            n_relu <= 1'b0; n_bias <= 1'b0; n_ok <= 1'b0; n_ack <= 1'b0;
            n_ww <= '0; n_ifw <= '0; n_ff <= '0; n_ofw <= '0; n_gp9 <= '0;
            arg_idx      <= '0;
            calc_step    <= '0;
            mul_a        <= '0;
            mul_b        <= '0;
            mul_acc      <= '0;
            k_cnt        <= '0;
            c_cnt        <= '0;
            j_cnt        <= '0;
            g_cnt        <= '0;
            x_cnt        <= '0;
            y_cnt        <= '0;
            id_cnt       <= '0;
            group_base   <= '0;
            gfilt_base   <= '0;
            row_base     <= '0;
            pix_base     <= '0;
            for (int i = 0; i < MAX_P; i++) carry[i] <= '0;
            err_flag     <= 1'b0;
        end else begin
            glb_we <= 1'b0;

            // Global timeout for the whole MAC run
            if (state >= S_MAC_RESET) begin
                mac_timer <= mac_timer + 1'b1;
                if (mac_timer == MAC_TIMEOUT) begin
                    pe_en    <= '0;
                    err_flag <= 1'b1;
                    state    <= S_WAIT_CMD;
                end
            end

            case (state)
                // ---------------- command dispatch ----------------
                S_WAIT_CMD: begin
                    tx_start <= 1'b0;
                    hdr_idx  <= '0;
                    arg_idx  <= '0;
                    byte_idx <= '0;
                    xfer_cnt <= '0;
                    if (rx_valid) begin
                        cmd <= rx_data;
                        case (rx_data)
                            CMD_WRITE_SRAM, CMD_WRITE_WORD, CMD_READ_WORD:
                                state <= S_RECV_HDR;
                            CMD_CONFIG, CMD_LAYER_CFG:
                                state <= S_CFG_ARGS;
                            CMD_START_MAC: begin
                                pe_en        <= '0;
                                pe_soft_rst  <= 1'b1;
                                soft_rst_cnt <= '0;
                                mac_timer    <= '0;
                                err_flag     <= 1'b0;
                                g_cnt        <= '0;
                                group_base   <= '0;
                                gfilt_base   <= filter_base;
                                state        <= S_MAC_RESET;
                            end
                            default: begin
                                tx_data <= rx_data;
                                state   <= S_ECHO;
                            end
                        endcase
                    end
                end

                S_ECHO: begin
                    if (!tx_start) begin
                        tx_start <= 1'b1;
                    end else if (tx_ready) begin
                        tx_start <= 1'b0;
                        state    <= S_WAIT_CMD;
                    end
                end

                // ---------------- transfer header ----------------
                // 0x01: LEN(2)            (byte mode, address 0)
                // 0x04 / 0x05: ADDR(2) N(2)
                S_RECV_HDR: begin
                    if (rx_valid) begin
                        hdr_idx <= hdr_idx + 1'b1;
                        if (cmd == CMD_WRITE_SRAM) begin
                            word_mode <= 1'b0;
                            xfer_addr <= '0;
                            if (hdr_idx == 3'd0) begin
                                xfer_len[7:0] <= rx_data;
                            end else begin
                                xfer_len[15:8] <= rx_data;
                                state <= ({rx_data, xfer_len[7:0]} == 16'd0) ? S_WAIT_CMD : S_RECV_BURST;
                            end
                        end else begin
                            word_mode <= 1'b1;
                            case (hdr_idx)
                                3'd0: xfer_addr[7:0] <= rx_data;
                                3'd1: xfer_addr[ADDR_W-1:8] <= rx_data[ADDR_W-9:0];
                                3'd2: xfer_len[7:0] <= rx_data;
                                default: begin
                                    xfer_len[15:8] <= rx_data;
                                    if ({rx_data, xfer_len[7:0]} == 16'd0)
                                        state <= S_WAIT_CMD;
                                    else if (cmd == CMD_READ_WORD)
                                        state <= S_READ_FETCH;
                                    else
                                        state <= S_RECV_BURST;
                                end
                            endcase
                        end
                    end
                end

                // ---------------- SRAM burst write ----------------
                S_RECV_BURST: begin
                    if (rx_valid) begin
                        word_shift <= {rx_data, word_shift[23:8]};
                        byte_idx   <= byte_idx + 1'b1;
                        if (byte_idx == last_byte) begin
                            byte_idx  <= '0;
                            glb_we    <= 1'b1;
                            glb_waddr <= xfer_addr;
                            glb_wdata <= word_mode ? {rx_data, word_shift} : {24'd0, rx_data};
                            xfer_addr <= xfer_addr + 1'b1;
                            xfer_cnt  <= xfer_cnt + 1'b1;
                            if (xfer_cnt + 1'b1 == xfer_len) state <= S_WAIT_CMD;
                        end
                    end
                end

                // ---------------- SRAM burst read ----------------
                S_READ_FETCH: begin
                    glb_raddr <= xfer_addr;
                    state     <= S_READ_WAIT;
                end

                S_READ_WAIT: begin
                    state <= S_READ_TX;
                end

                S_READ_TX: begin
                    tx_data <= glb_rdata[8 * byte_idx +: 8];
                    if (!tx_start) begin
                        tx_start <= 1'b1;
                    end else if (tx_ready) begin
                        tx_start <= 1'b0;
                        byte_idx <= byte_idx + 1'b1;
                        if (byte_idx == last_byte) begin
                            byte_idx  <= '0;
                            xfer_addr <= xfer_addr + 1'b1;
                            xfer_cnt  <= xfer_cnt + 1'b1;
                            state     <= (xfer_cnt + 1'b1 == xfer_len) ? S_WAIT_CMD : S_READ_FETCH;
                        end
                    end
                end

                // ---------------- configuration ----------------
                // 0x03: W SHIFT            0x06: W G P SHIFT FLAGS
                S_CFG_ARGS: begin
                    if (rx_valid) begin
                        arg_idx <= arg_idx + 1'b1;
                        if (cmd == CMD_CONFIG) begin
                            n_g    <= 5'd1;
                            n_p    <= 3'd1;
                            n_relu <= 1'b1;
                            n_bias <= 1'b0;
                            n_ack  <= 1'b0;
                            if (arg_idx == 3'd0) begin
                                // Legacy: out-of-range width keeps the old value
                                n_w <= (rx_data < 8'd3 || rx_data > 8'(MAX_W)) ? cfg_w : rx_data[5:0];
                            end else begin
                                n_shift <= rx_data[4:0];
                                n_ok    <= 1'b1;
                                state   <= S_CFG_CALC;
                            end
                        end else begin
                            n_ack <= 1'b1;
                            case (arg_idx)
                                3'd0: begin
                                    n_w  <= rx_data[5:0];
                                    n_ok <= (rx_data >= 8'd3 && rx_data <= 8'(MAX_W));
                                end
                                3'd1: begin
                                    n_g <= rx_data[4:0];
                                    if (rx_data < 8'd1 || rx_data > 8'(MAX_G)) n_ok <= 1'b0;
                                end
                                3'd2: begin
                                    n_p <= rx_data[2:0];
                                    if (rx_data < 8'd1 || rx_data > 8'(MAX_P)) n_ok <= 1'b0;
                                end
                                3'd3: begin
                                    n_shift <= rx_data[4:0];
                                    if (rx_data > 8'd31) n_ok <= 1'b0;
                                end
                                default: begin
                                    n_relu <= rx_data[0];
                                    n_bias <= rx_data[1];
                                    state  <= S_CFG_CALC;
                                end
                            endcase
                        end
                        // Arguments are complete when we enter S_CFG_CALC:
                        // first product is W * W
                        mul_a     <= 16'(n_w);
                        mul_b     <= n_w;
                        mul_acc   <= '0;
                        calc_step <= '0;
                    end
                end

                S_CFG_CALC: begin
                    // Serial shift-add multiplier, products:
                    //   0: W*W  1: G*WW  2: F*F  3: P*FF  4: G*9P
                    if (mul_b != 6'd0) begin
                        if (mul_b[0]) mul_acc <= mul_acc + mul_a;
                        mul_a <= mul_a << 1;
                        mul_b <= mul_b >> 1;
                    end else begin
                        mul_acc   <= '0;
                        calc_step <= calc_step + 1'b1;
                        case (calc_step)
                            3'd0: begin n_ww  <= mul_acc; mul_a <= mul_acc;      mul_b <= 6'(n_g); end
                            3'd1: begin n_ifw <= mul_acc; mul_a <= 16'(n_f);     mul_b <= n_f;     end
                            3'd2: begin n_ff  <= mul_acc; mul_a <= mul_acc;      mul_b <= 6'(n_p); end
                            3'd3: begin n_ofw <= mul_acc; mul_a <= 16'(n_p9);    mul_b <= 6'(n_g); end
                            default: begin
                                n_gp9 <= mul_acc;
                                state <= S_CFG_COMMIT;
                            end
                        endcase
                    end
                end

                S_CFG_COMMIT: begin
                    if (n_ok && n_total <= 16'(MEM_WORDS)) begin
                        cfg_w       <= n_w;
                        cfg_g       <= n_g;
                        cfg_p       <= n_p;
                        cfg_shift   <= n_shift;
                        cfg_relu    <= n_relu;
                        cfg_bias    <= n_bias;
                        cfg_ww      <= ADDR_W'(n_ww);
                        cfg_w2      <= ADDR_W'(n_w) << 1;
                        cfg_p9      <= n_p9;
                        cfg_f_m1    <= 5'(n_w - 6'd3);
                        filter_base <= ADDR_W'(n_ifw);
                        bias_base   <= ADDR_W'(n_ifw + n_gp9);
                        psum_base   <= ADDR_W'(n_ifw + n_gp9 + 16'(n_p));
                        ofmap_words <= (ADDR_W+1)'(n_ofw);
                        tx_data     <= RSP_ACK;
                    end else begin
                        tx_data     <= RSP_NACK;
                    end
                    state <= n_ack ? S_ECHO : S_WAIT_CMD;
                end

                // ---------------- MAC run ----------------
                S_MAC_RESET: begin
                    // Hold PE array in reset, then program IDs
                    soft_rst_cnt <= soft_rst_cnt + 1'b1;
                    if (soft_rst_cnt == 3'(SOFT_RST_CYCLES - 1)) begin
                        pe_soft_rst <= 1'b0;
                        id_cnt      <= '0;
                        state       <= S_SET_ID;
                    end
                end

                S_SET_ID: begin
                    id_cnt <= id_cnt + 1'b1;
                    if (id_cnt == 2'd3) begin
                        pe_en    <= NUMS_PE'(3'b111);   // PE0..PE2, PE3 idle
                        phase    <= PH_FILTER;
                        k_cnt    <= '0;
                        c_cnt    <= '0;
                        j_cnt    <= '0;
                        x_cnt    <= '0;
                        y_cnt    <= '0;
                        row_base <= group_base;
                        pix_base <= psum_base;
                        state    <= S_FETCH;
                    end
                end

                S_FETCH: begin
                    case (phase)
                        PH_FILTER: glb_raddr <= filter_addr;
                        PH_IFMAP:  glb_raddr <= ifmap_addr;
                        default:   glb_raddr <= (first_group ? bias_base : pix_base) + ADDR_W'(c_cnt);
                    endcase
                    state <= S_WAIT;
                end

                S_WAIT: begin
                    // glb_rdata becomes valid at the end of this cycle
                    state <= S_DRIVE;
                end

                S_DRIVE: begin
                    if (feed_ready) begin
                        state <= S_FETCH;
                        case (phase)
                            // PE k <- filter c row k taps 0..2, for every c
                            PH_FILTER: begin
                                j_cnt <= j_cnt + 1'b1;
                                if (j_cnt == 2'(KSIZE - 1)) begin
                                    j_cnt <= '0;
                                    c_cnt <= c_cnt + 1'b1;
                                    if (last_c) begin
                                        c_cnt <= '0;
                                        k_cnt <= k_cnt + 1'b1;
                                        if (k_cnt == 2'(KSIZE - 1)) begin
                                            k_cnt <= '0;
                                            phase <= PH_IFMAP;
                                        end
                                    end
                                end
                            end
                            PH_IFMAP: begin
                                j_cnt <= j_cnt + 1'b1;
                                if (x_cnt != 5'd0 || j_cnt == 2'(KSIZE - 1)) begin
                                    j_cnt <= '0;
                                    phase <= PH_IPSUM;
                                end
                            end
                            default: begin  // PH_IPSUM, one word per filter
                                c_cnt <= c_cnt + 1'b1;
                                if (last_c) begin
                                    c_cnt <= '0;
                                    state <= S_COLLECT;
                                end
                            end
                        endcase
                    end
                end

                S_COLLECT: begin
                    if (glb_opsum_valid) begin
                        carry[c_cnt] <= pe_data_out;
                        c_cnt        <= c_cnt + 1'b1;
                        if (last_c) begin
                            c_cnt <= '0;
                            phase <= PH_IFMAP;
                            if (k_cnt == 2'(KSIZE - 1)) begin
                                k_cnt <= '0;
                                state <= S_WRITE_OUT;
                            end else begin
                                k_cnt <= k_cnt + 1'b1;
                                state <= S_FETCH;
                            end
                        end
                    end
                end

                S_WRITE_OUT: begin
                    glb_we    <= 1'b1;
                    glb_waddr <= pix_base + ADDR_W'(c_cnt);
                    glb_wdata <= last_group ? {24'd0, ppu_data_out} : carry[c_cnt];
                    c_cnt     <= c_cnt + 1'b1;
                    if (last_c) begin
                        c_cnt    <= '0;
                        pix_base <= pix_base + ADDR_W'(cfg_p);
                        state    <= S_FETCH;
                        x_cnt    <= x_cnt + 1'b1;
                        if (last_col) begin
                            x_cnt    <= '0;
                            y_cnt    <= y_cnt + 1'b1;
                            row_base <= row_base + ADDR_W'(cfg_w);
                            if (last_row) state <= S_GROUP_END;
                        end
                    end
                end

                S_GROUP_END: begin
                    pe_en <= '0;
                    if (last_group) begin
                        // Reply: ofmap bytes (low byte of each psum word)
                        word_mode <= 1'b0;
                        byte_idx  <= '0;
                        xfer_addr <= psum_base;
                        xfer_len  <= 16'(ofmap_words);
                        xfer_cnt  <= '0;
                        state     <= S_READ_FETCH;
                    end else begin
                        // Next group: reload the PE array with its filters
                        g_cnt        <= g_cnt + 1'b1;
                        group_base   <= group_base + cfg_ww;
                        gfilt_base   <= gfilt_base + ADDR_W'(cfg_p9);
                        pe_soft_rst  <= 1'b1;
                        soft_rst_cnt <= '0;
                        state        <= S_MAC_RESET;
                    end
                end

                default: state <= S_WAIT_CMD;
            endcase
        end
    end

endmodule

// note
// 1. 多通道：一個 SRAM word 打包 4 個 int8 通道 (Eyeriss q=4)，輸入通道每 4 個一組 (G 組)
//    每顆 PE 同時算 P 個 filter (Eyeriss p，最多 4)
// 2. 2-D：PE k 存 filter 第 k 列，透過 GIN/GON tag 定址 (XID/YID 由 scan chain 設定)
//    每個輸出像素依序 PE0 -> PE1 -> PE2，P 個 psum 經 GLB (carry) 往下傳
// 3. 分組之間 psum 存在 GLB (psum 區)，下一組 PE0 的 ipsum 從 SRAM 讀回；第一組讀 bias
//    最後一組的結果經 PPU 量化後寫回同一塊區域，再從 SRAM 串流回主機
// 4. 換組時 soft reset PE array、重設 ID、載入下一組 filter
// 5. 0x06 的版面大小用 shift-add 序列乘法器計算 (不佔 DSP)，超出 SRAM 容量就回 0xEE
// 6. 0x01 / 0x03 保留舊協定 (G=1, P=1, 無 bias)，舊的主機腳本不用改
// 7. rst 按鈕尚未使用：OrangeCrab 按鈕極性/IO 標準請先確認再接
