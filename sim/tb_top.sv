`timescale 1ns/1ps
`include "define.svh"
// Testbench: emulate the host protocol, check the 2-D conv against a golden model
module tb_top;
    logic clk = 0;
    always #10 clk = ~clk;

    wire usb_p, usb_n, usb_pullup, led;
    Eyeriss_SoC_Top dut (
        .clk(clk), .rst(1'b0),
        .usb_d_p(usb_p), .usb_d_n(usb_n),
        .usb_pullup(usb_pullup), .led(led)
    );

    int ifmap  [0:1023];   // uint8, row-major, W x W
    int filter [0:8];      // int8, row-major, 3 x 3
    int golden [0:899];
    int cur_w     = 5;
    int cur_shift = 0;
    int n_pass    = 0;
    int n_case    = 0;

    // Software golden model (same as host script)
    function automatic void calc_golden(input int w, input int shift);
        int f = w - 2;
        for (int y = 0; y < f; y++)
            for (int x = 0; x < f; x++) begin
                int psum = 0;
                for (int ky = 0; ky < 3; ky++)
                    for (int kx = 0; kx < 3; kx++)
                        psum += (ifmap[(y + ky) * w + x + kx] - 128) * filter[ky * 3 + kx];
                psum = psum >>> shift;
                if (psum > 127)  psum = 127;
                if (psum < -128) psum = -128;
                psum += 128;
                if (psum < 128) psum = 128;
                golden[y * f + x] = psum;
            end
    endfunction

    task automatic send(input int b);
        dut.u_usb_uart.send_byte(b[7:0]);
    endtask

    task automatic configure(input int w, input int shift);
        send(8'h03); send(w); send(shift);
        cur_w = w; cur_shift = shift;
    endtask

    task automatic run_case(input string name);
        int w = cur_w;
        int f = cur_w - 2;
        int len = w * w + 9;
        int start_len, errors;
        n_case++;
        calc_golden(w, cur_shift);
        // CMD_WRITE_SRAM + length (little endian) + payload
        send(8'h01); send(len & 8'hFF); send(len >> 8);
        for (int i = 0; i < w * w; i++) send(ifmap[i]);
        for (int i = 0; i < 9; i++)     send(filter[i]);
        // CMD_START_MAC
        start_len = dut.u_usb_uart.tx_count;
        send(8'h02);
        for (int t = 0; t < 2000000 && dut.u_usb_uart.tx_count != start_len + f * f; t++)
            @(posedge clk);
        if (dut.u_usb_uart.tx_count != start_len + f * f) begin
            $display("[%s] FAIL: timeout, got %0d / %0d bytes", name,
                     dut.u_usb_uart.tx_count - start_len, f * f);
            return;
        end
        errors = 0;
        for (int i = 0; i < f * f; i++)
            if (dut.u_usb_uart.tx_log[start_len + i] != golden[i]) begin
                if (errors < 5)
                    $display("  mismatch @%0d: hw=%0d golden=%0d", i,
                             dut.u_usb_uart.tx_log[start_len + i], golden[i]);
                errors++;
            end
        if (errors == 0) begin
            $display("[%s] PASS (W=%0d shift=%0d, %0d px)", name, w, cur_shift, f * f);
            n_pass++;
        end else
            $display("[%s] FAIL: %0d / %0d mismatches", name, errors, f * f);
        repeat (50) @(posedge clk);
    endtask

    task automatic randomize_data(input int w, input int fmax);
        for (int i = 0; i < w * w; i++) ifmap[i] = $urandom_range(0, 255);
        for (int i = 0; i < 9; i++)     filter[i] = $signed($urandom_range(0, 2 * fmax)) - fmax;
    endtask

    int seed = 32'hE7E5;
    int demo_ifmap [25] = '{150, 155, 160, 155, 150,
                            160, 180, 190, 185, 155,
                            155, 195, 220, 190, 160,
                            150, 185, 195, 180, 155,
                            145, 150, 155, 150, 145};

    initial begin
        void'($urandom(seed));
        repeat (400) @(posedge clk);   // wait for POR

        // PING
        send(8'hFF);
        repeat (20) @(posedge clk);
        if (dut.u_usb_uart.tx_count == 1 && dut.u_usb_uart.tx_log[0] == 8'hFF)
            $display("[ping] PASS");
        else
            $display("[ping] FAIL");

        // Default config (W=5, shift=0), deploy-script style data
        for (int i = 0; i < 25; i++) ifmap[i] = demo_ifmap[i];
        filter = '{0, 0, 0, 0, 1, 0, 0, 0, 0};
        run_case("identity kernel");
        filter = '{1, 1, 1, 1, 1, 1, 1, 1, 1};
        run_case("box kernel");
        run_case("box kernel repeat");
        filter = '{-1, -1, -1, -1, 8, -1, -1, -1, -1};
        run_case("laplacian");
        filter = '{-3, -2, -1, -2, -1, -1, -1, -1, -1};
        run_case("all-negative (ReLU)");

        // Random data with scaling, several widths
        configure(5, 8);
        repeat (3) begin randomize_data(5, 127); run_case("rand W=5"); end
        configure(3, 9);
        randomize_data(3, 127); run_case("rand W=3");
        configure(8, 6);
        repeat (2) begin randomize_data(8, 40); run_case("rand W=8"); end
        configure(16, 10);
        randomize_data(16, 127); run_case("rand W=16");
        configure(32, 10);
        randomize_data(32, 127); run_case("rand W=32");
        // Out-of-range width is ignored (keeps W=32), shift still updates
        configure(40, 11);
        cur_w = 32;
        randomize_data(32, 127); run_case("bad W ignored");
        // Back to default and check no state leaks between widths
        configure(5, 0);
        for (int i = 0; i < 25; i++) ifmap[i] = demo_ifmap[i];
        filter = '{0, 0, 0, 0, 1, 0, 0, 0, 0};
        run_case("identity after resize");

        $display("RESULT: %0d / %0d passed", n_pass, n_case);
        $finish;
    end
endmodule
