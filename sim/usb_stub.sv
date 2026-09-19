// Behavioral stub of usb_uart_np for simulation (replaces USB CDC IP)
module usb_uart_np (
    input  clk_48mhz,
    input  reset,
    inout  pin_usb_p,
    inout  pin_usb_n,
    input  [7:0] uart_in_data,
    input        uart_in_valid,
    output logic uart_in_ready,
    output logic [7:0] uart_out_data,
    output logic uart_out_valid,
    input        uart_out_ready,
    output [11:0] debug
);
    initial begin
        uart_out_valid = 1'b0;
        uart_out_data  = 8'h00;
        uart_in_ready  = 1'b0;
    end

    // TX side: accept a byte 2 cycles after valid rises
    int unsigned tx_wait = 0;
    int tx_log [0:65535];
    int tx_count = 0;
    always @(posedge clk_48mhz) begin
        if (uart_in_valid && uart_in_ready) begin
            tx_log[tx_count] = uart_in_data;
            tx_count++;
            uart_in_ready <= 1'b0;
            tx_wait = 0;
        end else if (uart_in_valid) begin
            tx_wait++;
            if (tx_wait >= 2) uart_in_ready <= 1'b1;
        end else begin
            uart_in_ready <= 1'b0;
            tx_wait = 0;
        end
    end

    // RX side: host -> FPGA byte injection, valid held until ready (like the core)
    task automatic send_byte(input byte unsigned value);
        @(posedge clk_48mhz);
        uart_out_data  <= value;
        uart_out_valid <= 1'b1;
        do @(posedge clk_48mhz); while (!uart_out_ready);
        uart_out_valid <= 1'b0;
        repeat (3) @(posedge clk_48mhz);
    endtask
endmodule
