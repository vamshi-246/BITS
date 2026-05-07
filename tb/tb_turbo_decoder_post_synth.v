//==============================================================================
// Testbench: tb_turbo_decoder_post_synth
// Description:
//   Port-only functional testbench for post-synthesis netlist simulation.
//   It avoids hierarchical references into the DUT because synthesized netlists
//   do not preserve most RTL-internal signal names.
//==============================================================================
`timescale 1ns / 1ps

module tb_turbo_decoder_post_synth;
    localparam TEST_K               = 3200;
    localparam TEST_NUM_SISO        = 2;
    localparam TEST_SEG_LEN         = TEST_K / TEST_NUM_SISO;
    localparam TEST_TAIL_LEN        = 3;
    localparam TEST_ROW_DEPTH       = TEST_SEG_LEN / 2;
    localparam TEST_INPUT_ROW_DEPTH = (TEST_SEG_LEN + TEST_TAIL_LEN + 1) / 2;
    localparam TEST_ROW_ADDR_W      = 10;
    localparam TEST_INPUT_WORD_W    = 10;
    localparam TEST_EXTR_WORD_W     = 12;

    reg clk;
    reg rst_n;
    reg start;
    reg load_en;
    reg [2:0] load_bram_sel;
    reg [TEST_ROW_ADDR_W-1:0] load_addr;
    reg [TEST_INPUT_WORD_W-1:0] load_data;
    wire decode_done;
    reg [TEST_ROW_ADDR_W-1:0] ld_rd_addr;
    wire [TEST_EXTR_WORD_W-1:0] ld_rd_data;
    wire [TEST_EXTR_WORD_W-1:0] ld_rd_data_odd;
    reg [TEST_ROW_ADDR_W-1:0] hard_rd_addr;
    wire [3:0] hard_rd_data;

    // The post-synthesis netlist has fixed parameters, so instantiate without
    // parameter overrides.
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
        .ld_rd_data_odd(ld_rd_data_odd),
        .hard_rd_addr(hard_rd_addr),
        .hard_rd_data(hard_rd_data)
    );

    reg [TEST_INPUT_WORD_W-1:0] sys_even_mem  [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] sys_odd_mem   [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] par1_even_mem [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] par1_odd_mem  [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] silv_even_mem [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] silv_odd_mem  [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] par2_even_mem [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] par2_odd_mem  [0:TEST_INPUT_ROW_DEPTH-1];
    reg true_info_bits [0:TEST_K-1];
    reg expected_hard_bits [0:TEST_K-1];
    reg decoded_bits [0:TEST_K-1];

    integer i;
    integer row;
    integer truth_errors;
    integer rtl_mismatches;
    integer first_rtl_mismatch;
    integer fd_bits;
    reg [3:0] hard_row;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task load_one_bram;
        input [2:0] sel;
        integer idx;
        begin
            for (idx = 0; idx < TEST_INPUT_ROW_DEPTH; idx = idx + 1) begin
                @(posedge clk);
                load_en       <= 1'b1;
                load_bram_sel <= sel;
                load_addr     <= idx[TEST_ROW_ADDR_W-1:0];
                case (sel)
                    3'b000: load_data <= sys_odd_mem[idx];
                    3'b001: load_data <= sys_even_mem[idx];
                    3'b010: load_data <= par1_odd_mem[idx];
                    3'b011: load_data <= par1_even_mem[idx];
                    3'b100: load_data <= silv_odd_mem[idx];
                    3'b101: load_data <= silv_even_mem[idx];
                    3'b110: load_data <= par2_odd_mem[idx];
                    3'b111: load_data <= par2_even_mem[idx];
                    default: load_data <= {TEST_INPUT_WORD_W{1'b0}};
                endcase
            end
            @(posedge clk);
            load_en <= 1'b0;
        end
    endtask

    initial begin
        $readmemh("data/sys_even_ram.hex",     sys_even_mem);
        $readmemh("data/sys_odd_ram.hex",      sys_odd_mem);
        $readmemh("data/par1_even_ram.hex",    par1_even_mem);
        $readmemh("data/par1_odd_ram.hex",     par1_odd_mem);
        $readmemh("data/sys_ilv_even_ram.hex", silv_even_mem);
        $readmemh("data/sys_ilv_odd_ram.hex",  silv_odd_mem);
        $readmemh("data/par2_even_ram.hex",    par2_even_mem);
        $readmemh("data/par2_odd_ram.hex",     par2_odd_mem);
        $readmemb("data/true_info_bits.txt",   true_info_bits);
        $readmemb("data/rtl_final_hard_bits.txt", expected_hard_bits);

        for (i = 0; i < TEST_K; i = i + 1)
            decoded_bits[i] = 1'b0;

        rst_n = 1'b0;
        start = 1'b0;
        load_en = 1'b0;
        load_bram_sel = 3'd0;
        load_addr = {TEST_ROW_ADDR_W{1'b0}};
        load_data = {TEST_INPUT_WORD_W{1'b0}};
        ld_rd_addr = {TEST_ROW_ADDR_W{1'b0}};
        hard_rd_addr = {TEST_ROW_ADDR_W{1'b0}};

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("[%0t] Loading decoder input BRAMs...", $time);
        load_one_bram(3'b000);
        load_one_bram(3'b001);
        load_one_bram(3'b010);
        load_one_bram(3'b011);
        load_one_bram(3'b100);
        load_one_bram(3'b101);
        load_one_bram(3'b110);
        load_one_bram(3'b111);
        $display("[%0t] Input BRAM loading complete.", $time);

        repeat (5) @(posedge clk);
        $display("[%0t] Starting post-synthesis decode.", $time);
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        wait (decode_done === 1'b1);
        @(posedge clk);
        $display("[%0t] decode_done observed.", $time);

        for (row = 0; row < TEST_ROW_DEPTH; row = row + 1) begin
            @(posedge clk);
            hard_rd_addr <= row[TEST_ROW_ADDR_W-1:0];
            @(posedge clk);
            #1;
            hard_row = hard_rd_data;
            decoded_bits[2 * row] = hard_row[0];
            decoded_bits[TEST_SEG_LEN + (2 * row)] = hard_row[1];
            decoded_bits[(2 * row) + 1] = hard_row[2];
            decoded_bits[TEST_SEG_LEN + (2 * row) + 1] = hard_row[3];
        end

        truth_errors = 0;
        rtl_mismatches = 0;
        first_rtl_mismatch = -1;
        fd_bits = $fopen("build/post_synth_funcsim/turbo_decoder_post_synth_hard_bits.txt", "w");
        for (i = 0; i < TEST_K; i = i + 1) begin
            if (fd_bits != 0)
                $fwrite(fd_bits, "%0d\n", decoded_bits[i]);
            if (decoded_bits[i] != true_info_bits[i])
                truth_errors = truth_errors + 1;
            if (decoded_bits[i] != expected_hard_bits[i]) begin
                if (first_rtl_mismatch < 0)
                    first_rtl_mismatch = i;
                rtl_mismatches = rtl_mismatches + 1;
            end
        end
        if (fd_bits != 0)
            $fclose(fd_bits);

        $display("------------------------------------------------------------");
        $display("Post-Synthesis Functional Simulation Results");
        $display("  Errors vs true_info_bits        : %0d/%0d", truth_errors, TEST_K);
        $display("  Mismatches vs RTL hard-bit file : %0d/%0d", rtl_mismatches, TEST_K);
        if (first_rtl_mismatch >= 0)
            $display("  First RTL mismatch index        : %0d", first_rtl_mismatch);
        if ((truth_errors == 1) && (rtl_mismatches == 0))
            $display("PASS: post-synthesis netlist matches RTL hard-bit output.");
        else
            $display("FAIL: post-synthesis netlist output mismatch.");
        $display("------------------------------------------------------------");

        repeat (10) @(posedge clk);
        $finish;
    end

    initial begin
        #(700_000_000);
        $display("ERROR: post-synthesis functional simulation timed out.");
        $finish;
    end
endmodule
