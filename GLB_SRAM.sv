// GLB_SRAM.sv
module GLB_SRAM #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 10
)(
    input  logic clk,
    input  logic we,                            // Write enable (Active high)
    input  logic [ADDR_WIDTH-1:0] waddr,        // Write address
    input  logic [ADDR_WIDTH-1:0] raddr,        // Read address
    input  logic [DATA_WIDTH-1:0] wdata,        // Write data
    output logic [DATA_WIDTH-1:0] rdata         // Read data
);

    // Force Yosys to infer Embedded Block RAM (EBR)
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

    // Synchronous read and write required for BRAM inference
    always_ff @(posedge clk) begin
        if (we) begin
            mem[waddr] <= wdata;
        end
        rdata <= mem[raddr]; 
    end

endmodule