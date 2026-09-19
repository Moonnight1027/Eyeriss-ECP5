`timescale 1ns/1ps
// Unit test: usb_uart_bridge_ep IN path terminates every stream with a short
// packet (a ZLP when the length is a multiple of the 32-byte packet size).
// The IN endpoint is a behavioural model of usb_fs_in_pe's buffer handling.
module tb_zlp;
    logic clk = 0;
    always #10 clk = ~clk;
    logic reset = 1;

    logic       in_ep_req, in_ep_grant, in_ep_data_free, in_ep_data_put, in_ep_data_done;
    logic [7:0] in_ep_data;
    logic [7:0] uart_in_data = 0;
    logic       uart_in_valid = 0, uart_in_ready;

    usb_uart_bridge_ep dut (
        .clk(clk), .reset(reset),
        .out_ep_req(), .out_ep_grant(1'b0), .out_ep_data_avail(1'b0), .out_ep_setup(1'b0),
        .out_ep_data_get(), .out_ep_data(8'd0), .out_ep_stall(), .out_ep_acked(1'b0),
        .in_ep_req(in_ep_req), .in_ep_grant(in_ep_grant), .in_ep_data_free(in_ep_data_free),
        .in_ep_data_put(in_ep_data_put), .in_ep_data(in_ep_data), .in_ep_data_done(in_ep_data_done),
        .in_ep_stall(), .in_ep_acked(1'b0),
        .uart_in_data(uart_in_data), .uart_in_valid(uart_in_valid), .uart_in_ready(uart_in_ready),
        .uart_out_data(), .uart_out_valid(), .uart_out_ready(1'b1),
        .debug()
    );

    // ---- endpoint model ----
    typedef enum {PUTTING, GETTING} ep_state_t;
    ep_state_t ep_state = PUTTING;
    int ep_cnt = 0, get_timer = 0;
    int pkts [$];
    int rx_bytes [$];
    byte buffer [32];

    assign in_ep_grant     = in_ep_req;
    assign in_ep_data_free = (ep_state == PUTTING) && (ep_cnt < 32);

    always @(posedge clk) begin
        case (ep_state)
            PUTTING: begin
                if (in_ep_data_put && ep_cnt < 32) begin
                    buffer[ep_cnt] = in_ep_data;
                    ep_cnt++;
                end
                if (in_ep_data_done || ep_cnt == 32) begin
                    ep_state  <= GETTING;
                    get_timer <= 0;
                end
            end
            GETTING: begin
                // host polls every ~200 cycles
                get_timer <= get_timer + 1;
                if (get_timer == 200) begin
                    pkts.push_back(ep_cnt);
                    for (int i = 0; i < ep_cnt; i++) rx_bytes.push_back(buffer[i]);
                    ep_cnt   = 0;
                    ep_state <= PUTTING;
                end
            end
        endcase
    end

    // ---- stream driver, controller-like gaps between bytes ----
    task automatic send_stream(input int n);
        for (int i = 0; i < n; i++) begin
            @(posedge clk);
            uart_in_data  <= i[7:0];
            uart_in_valid <= 1'b1;
            do @(posedge clk); while (!uart_in_ready);
            uart_in_valid <= 1'b0;
            repeat ($urandom_range(1, 5)) @(posedge clk);
        end
    endtask

    int n_err = 0;
    int lens [9] = '{1, 31, 32, 33, 64, 96, 256, 900, 1156};
    int n, total, last;
    initial begin
        repeat (10) @(posedge clk);
        reset <= 0;
        foreach (lens[t]) begin
            n = lens[t];
            total = 0;
            pkts.delete(); rx_bytes.delete();
            send_stream(n);
            repeat (3000) @(posedge clk);
            foreach (pkts[i]) total += pkts[i];
            last = pkts[pkts.size() - 1];
            if (total != n || last >= 32 || (n % 32 == 0) != (last == 0)) begin
                $display("FAIL len=%0d: %0d packets, %0d bytes, last=%0d", n, pkts.size(), total, last);
                n_err++;
            end else begin
                foreach (rx_bytes[i]) if (rx_bytes[i] != byte'(i)) n_err++;
                $display("ok   len=%0d: %0d packets, last=%0d", n, pkts.size(), last);
            end
            // no spurious packets while idle
            if (pkts.size() != (n + 31) / 32 + (n % 32 == 0)) begin
                $display("FAIL len=%0d: unexpected packet count %0d", n, pkts.size());
                n_err++;
            end
        end
        $display("RESULT: %s", n_err == 0 ? "PASS" : "FAIL");
        $finish;
    end
endmodule
