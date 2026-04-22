//==============================================================================
// Testbench: tb_turbo_decoder
// Description: Full-system testbench for the parallel LTE Turbo Decoder.
//   1. Preloads all 8 Group-A BRAMs via the load interface.
//   2. Asserts start for 1 cycle.
//   3. Simulates until decode_done.
//   4. Reads all 1536 ld_ram rows and writes to file.
//   5. Counts done pulses and verifies half_iter_cnt progression.
//==============================================================================
`timescale 1ns / 1ps

module tb_turbo_decoder;

    // =========================================================================
    // Clock and reset
    // =========================================================================
    reg clk, rst_n;
    localparam CLK_PERIOD = 10;

    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // =========================================================================
    // DUT signals
    // =========================================================================
    reg        start;
    reg        load_en;
    reg [2:0]  load_bram_sel;
    reg [10:0] load_addr;
    reg [9:0]  load_data;
    wire       decode_done;
    reg [10:0] ld_rd_addr;
    wire [11:0] ld_rd_data;
    wire [11:0] ld_rd_data_odd;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    turbo_decoder u_dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .load_en(load_en),
        .load_bram_sel(load_bram_sel),
        .load_addr(load_addr),
        .load_data(load_data),
        .decode_done(decode_done),
        .ld_rd_addr(ld_rd_addr),
        .ld_rd_data(ld_rd_data),
        .ld_rd_data_odd(ld_rd_data_odd)
    );

    // =========================================================================
    // BRAM hex file storage for loading
    // =========================================================================
    reg [9:0] sys_even_mem [0:1535];
    reg [9:0] sys_odd_mem  [0:1535];
    reg [9:0] par1_even_mem[0:1535];
    reg [9:0] par1_odd_mem [0:1535];
    reg [9:0] silv_even_mem[0:1535];
    reg [9:0] silv_odd_mem [0:1535];
    reg [9:0] par2_even_mem[0:1535];
    reg [9:0] par2_odd_mem [0:1535];

    initial begin
        $readmemh("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/sys_even_ram.hex",     sys_even_mem);
        $readmemh("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/sys_odd_ram.hex",      sys_odd_mem);
        $readmemh("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/par1_even_ram.hex",    par1_even_mem);
        $readmemh("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/par1_odd_ram.hex",     par1_odd_mem);
        $readmemh("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/sys_ilv_even_ram.hex", silv_even_mem);
        $readmemh("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/sys_ilv_odd_ram.hex",  silv_odd_mem);
        $readmemh("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/par2_even_ram.hex",    par2_even_mem);
        $readmemh("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/par2_odd_ram.hex",     par2_odd_mem);
    end

    // =========================================================================
    // Monitoring
    // =========================================================================
    integer done_count;
    integer half_iter_monitor;

    initial done_count = 0;

    always @(posedge clk) begin
        if (u_dut.c0_done) begin
            done_count = done_count + 1;
            $display("[%0t] Core done pulse #%0d, half_iter_cnt=%0d",
                     $time, done_count, u_dut.half_iter_cnt);
        end
    end

    // =========================================================================
    // X-detection monitors — only during active decode (ST_RUNNING=1)
    // =========================================================================
    integer x_count;
    initial x_count = 0;

    always @(posedge clk) begin
        if (u_dut.main_state == 3'd1) begin // ST_RUNNING
            if (^u_dut.pi_fr_even === 1'bx && u_dut.fetch_state == 3'd1) begin
                x_count = x_count + 1;
                if (x_count <= 5) $display("[%0t] X-DETECT: pi_fr_even=%h (fetch_state=%0d, half_iter=%0d)", $time, u_dut.pi_fr_even, u_dut.fetch_state, u_dut.half_iter_cnt);
            end
            if (^u_dut.mn_fr_e_row === 1'bx && u_dut.fetch_state == 3'd1) begin
                x_count = x_count + 1;
                if (x_count <= 5) $display("[%0t] X-DETECT: mn_fr_e_row=%h (fetch_state=%0d, half_iter=%0d)", $time, u_dut.mn_fr_e_row, u_dut.fetch_state, u_dut.half_iter_cnt);
            end
            if (^u_dut.c0_l_extr_even === 1'bx && u_dut.c0_llr_out_valid) begin
                x_count = x_count + 1;
                if (x_count <= 5) $display("[%0t] X-DETECT: c0_l_extr_even=%h (llr_out_valid=1)", $time, u_dut.c0_l_extr_even);
            end
            if (^u_dut.c1_l_extr_even === 1'bx && u_dut.c1_llr_out_valid) begin
                x_count = x_count + 1;
                if (x_count <= 5) $display("[%0t] X-DETECT: c1_l_extr_even=%h (llr_out_valid=1)", $time, u_dut.c1_l_extr_even);
            end
        end
    end

    // =========================================================================
    // Load task: write one BRAM via the load interface
    // =========================================================================
    task load_one_bram;
        input [2:0]  sel;
        input integer which; // 0-7 for the 8 BRAMs
        integer i;
        begin
            for (i = 0; i < 1536; i = i + 1) begin
                @(posedge clk);
                load_en       <= 1'b1;
                load_bram_sel <= sel;
                load_addr     <= i[10:0];
                case (sel)
                    3'b000: load_data <= sys_odd_mem[i];
                    3'b001: load_data <= sys_even_mem[i];
                    3'b010: load_data <= par1_odd_mem[i];
                    3'b011: load_data <= par1_even_mem[i];
                    3'b100: load_data <= silv_odd_mem[i];
                    3'b101: load_data <= silv_even_mem[i];
                    3'b110: load_data <= par2_odd_mem[i];
                    3'b111: load_data <= par2_even_mem[i];
                    default: load_data <= 10'd0;
                endcase
            end
            @(posedge clk);
            load_en <= 1'b0;
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    integer i;
    integer fd;

    initial begin
        // Waveform dump
        $dumpfile("tb_turbo_decoder.vcd");
        $dumpvars(0, tb_turbo_decoder);

        // Reset
        rst_n    = 0;
        start    = 0;
        load_en  = 0;
        load_bram_sel = 3'd0;
        load_addr     = 11'd0;
        load_data     = 10'd0;
        ld_rd_addr    = 11'd0;

        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // ---- Load all 8 Group-A BRAMs ----
        $display("[%0t] Loading BRAMs...", $time);
        load_one_bram(3'b000, 0); // sys_odd
        load_one_bram(3'b001, 1); // sys_even
        load_one_bram(3'b010, 2); // par1_odd
        load_one_bram(3'b011, 3); // par1_even
        load_one_bram(3'b100, 4); // silv_odd
        load_one_bram(3'b101, 5); // silv_even
        load_one_bram(3'b110, 6); // par2_odd
        load_one_bram(3'b111, 7); // par2_even
        $display("[%0t] BRAM loading complete.", $time);

        repeat (5) @(posedge clk);

        // ---- Start decoding ----
        $display("[%0t] Starting decode...", $time);
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // ---- Wait for decode_done ----
        wait (decode_done == 1'b1);
        @(posedge clk);
        $display("[%0t] decode_done asserted. Total done pulses = %0d", $time, done_count);

        // ---- Assertions ----
        if (done_count != 11)
            $display("ERROR: Expected 11 done pulses, got %0d", done_count);
        else
            $display("PASS: Correct number of done pulses (11).");

        // ---- Read LD RAM (even and odd) and dump to file ----
        fd = $fopen("C:/Users/USER/Documents/Digital_VLSI_grp_10_BITS/BITS/data/ld_ram_output.hex", "w");
        for (i = 0; i < 1536; i = i + 1) begin
            @(posedge clk);
            ld_rd_addr <= i[10:0];
            @(posedge clk); // 1-cycle read latency
            $fwrite(fd, "%03x %03x\n", ld_rd_data, ld_rd_data_odd);
        end
        $fclose(fd);
        $display("[%0t] LD RAM output (even+odd) written to ld_ram_output.hex", $time);

        repeat (10) @(posedge clk);
        $display("Simulation complete.");
        $finish;
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #(500_000_000); // 500ms timeout
        $display("ERROR: Simulation timed out!");
        $finish;
    end

endmodule
