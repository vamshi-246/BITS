`timescale 1ns/1ps

//==============================================================================
// Module: tb_bcjr_core
// Description: Testbench for the bcjr_core top-level module.
//              Loads LLR data from input_llr.hex.
//              Captures extrinsic outputs to rtl_extrinsic.hex.
//==============================================================================
module tb_bcjr_core;

    localparam FRAME_LEN = 3072; // 6144/2 = 3072 per core (2 SISO modules)
    localparam WIN_LEN   = 30;  // Trellis steps per window

    // =========================================================================
    // Clock & Reset
    // =========================================================================
    reg clk;
    reg rst_n;
    
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end

    // =========================================================================
    // LLR Memory (Read from File)
    // Format: 15 bits per word => {apr(5), par(5), sys(5)}
    // Size: FRAME_LEN words. 
    // =========================================================================
    reg [14:0] llr_mem [0:4095]; // allocate enough for padded windows
    
    initial begin
        // Load actual test data from python generated hex file
        // Entries beyond file length remain 0 (simulator default)
        $readmemh("../data/input_llr.hex", llr_mem);
    end

    // =========================================================================
    // DUT Signals
    // =========================================================================
    reg  start;
    wire done;
    
    wire         llr_req;
    wire [11:0] fr_llr_addr;
    wire [11:0] br_llr_addr;
    wire [11:0] dbr_llr_addr;
    reg         llr_valid;
    
    reg signed [4:0] fr_sys_odd, fr_sys_even;
    reg signed [4:0] fr_par_odd, fr_par_even;
    reg signed [4:0] fr_apr_odd, fr_apr_even;

    reg signed [4:0] br_sys_odd, br_sys_even;
    reg signed [4:0] br_par_odd, br_par_even;
    reg signed [4:0] br_apr_odd, br_apr_even;

    reg signed [4:0] dbr_sys_odd, dbr_sys_even;
    reg signed [4:0] dbr_par_odd, dbr_par_even;
    reg signed [4:0] dbr_apr_odd, dbr_apr_even;

    wire signed [5:0] llr_extr_odd_out;
    wire signed [5:0] llr_extr_even_out;
    wire [11:0]       llr_out_addr;
    wire              llr_out_valid;

    // =========================================================================
    // Device Under Test
    // =========================================================================
    bcjr_core #(
        .CORE_ID(0),
        .NUM_SISO(2),
        .NUM_WINDOWS(103)   // ceil(3072/30) = 103
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .frame_len(FRAME_LEN),
        .done(done),

        .llr_req(llr_req),
        .fr_llr_addr(fr_llr_addr),
        .br_llr_addr(br_llr_addr),
        .dbr_llr_addr(dbr_llr_addr),
        .llr_valid(llr_valid),

        .fr_sys_odd(fr_sys_odd),   .fr_sys_even(fr_sys_even),
        .fr_par_odd(fr_par_odd),   .fr_par_even(fr_par_even),
        .fr_apr_odd(fr_apr_odd),   .fr_apr_even(fr_apr_even),

        .br_sys_odd(br_sys_odd),   .br_sys_even(br_sys_even),
        .br_par_odd(br_par_odd),   .br_par_even(br_par_even),
        .br_apr_odd(br_apr_odd),   .br_apr_even(br_apr_even),

        .dbr_sys_odd(dbr_sys_odd), .dbr_sys_even(dbr_sys_even),
        .dbr_par_odd(dbr_par_odd), .dbr_par_even(dbr_par_even),
        .dbr_apr_odd(dbr_apr_odd), .dbr_apr_even(dbr_apr_even),

        .llr_extr_odd_out(llr_extr_odd_out),
        .llr_extr_even_out(llr_extr_even_out),
        .llr_out_addr(llr_out_addr),
        .llr_out_valid(llr_out_valid)
    );

    // =========================================================================
    // Read LLR memory (1 cycle latency)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            llr_valid <= 1'b0;
        end else if (llr_req) begin
            llr_valid <= 1'b1;
            
            // Safeguard against out of bounds reading
            if ((fr_llr_addr + 1) < FRAME_LEN) begin
                {fr_apr_odd, fr_par_odd, fr_sys_odd}    <= llr_mem[fr_llr_addr];
                {fr_apr_even, fr_par_even, fr_sys_even} <= llr_mem[fr_llr_addr+1];
            end else begin
                {fr_apr_odd, fr_par_odd, fr_sys_odd}    <= 15'sd0;
                {fr_apr_even, fr_par_even, fr_sys_even} <= 15'sd0;
            end
            
            if ((br_llr_addr + 1) < FRAME_LEN) begin
                {br_apr_odd, br_par_odd, br_sys_odd}    <= llr_mem[br_llr_addr];
                {br_apr_even, br_par_even, br_sys_even} <= llr_mem[br_llr_addr+1];
            end else begin
                {br_apr_odd, br_par_odd, br_sys_odd}    <= 15'sd0;
                {br_apr_even, br_par_even, br_sys_even} <= 15'sd0;
            end
            
            if ((dbr_llr_addr + 1) < FRAME_LEN) begin
                {dbr_apr_odd, dbr_par_odd, dbr_sys_odd}    <= llr_mem[dbr_llr_addr];
                {dbr_apr_even, dbr_par_even, dbr_sys_even} <= llr_mem[dbr_llr_addr+1];
            end else begin
                {dbr_apr_odd, dbr_par_odd, dbr_sys_odd}    <= 15'sd0;
                {dbr_apr_even, dbr_par_even, dbr_sys_even} <= 15'sd0;
            end
        end else begin
            llr_valid <= 1'b0;
        end
    end

    // =========================================================================
    // Capture Output Extrinsics
    // =========================================================================
    integer f_out;
    
    initial begin
        f_out = $fopen("../data/rtl_extrinsic.hex", "w");
        if (f_out == 0) begin
            $display("ERROR: Cannot open output file rtl_extrinsic.hex!");
            $finish;
        end
    end

    always @(posedge clk) begin
        if (llr_out_valid) begin
            // Format: "ADDR: EXTR_ODD EXTR_EVEN"
            // Ensure values are printed as 6-bit 2's complement hex
            $fdisplay(f_out, "%03x: %02x %02x", 
                llr_out_addr, 
                (llr_extr_odd_out & 6'h3F), 
                (llr_extr_even_out & 6'h3F)
            );
        end
    end

    // =========================================================================
    // Main Stimulus
    // =========================================================================
    initial begin
        // Init
        start = 0;
        rst_n = 0;
        
        $display("=================================================");
        $display("Starting Simulation of BCJR Core");
        $display("=================================================");

        #100;
        rst_n = 1; // Release reset
        
        #20;
        start = 1; // Assert start
        #10;
        start = 0; // Deassert start
        
        // Wait for finish
        wait (done == 1'b1);
        $display("BCJR Core Finished Decoding.");
        
        #50;
        $fclose(f_out);
        $display("Outputs successfully written to rtl_extrinsic.hex. Terminating.");
        $finish;
    end
    
    // Safety timeout (103 windows need much more time)
    initial begin
        #1000000;
        $display("ERROR: Simulation timed out!");
        $finish;
    end

endmodule
