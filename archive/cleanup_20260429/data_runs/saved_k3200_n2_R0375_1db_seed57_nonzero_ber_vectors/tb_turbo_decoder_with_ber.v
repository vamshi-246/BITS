//==============================================================================
// Testbench: tb_turbo_decoder
// Description: Full-system testbench for the parallel LTE Turbo Decoder.
//   1. Preloads all 8 Group-A BRAMs via the load interface.
//   2. Asserts start for 1 cycle.
//   3. Simulates until decode_done.
//   4. Reads all ld_ram rows and writes to file.
//   5. Counts done pulses and verifies half_iter_cnt progression.
//==============================================================================
`timescale 1ns / 1ps

module tb_turbo_decoder;
    parameter TEST_K = 6144;
    parameter TEST_NUM_SISO = 2;
    parameter TEST_SEG_LEN = TEST_K / TEST_NUM_SISO;
    parameter TEST_TAIL_LEN = 0;
    parameter TEST_PAPER_BOUNDARY = 0;
    parameter TEST_ROW_DEPTH = TEST_SEG_LEN / 2;
    parameter TEST_INPUT_ROW_DEPTH = (TEST_SEG_LEN + TEST_TAIL_LEN + 1) / 2;
    parameter TEST_ROW_ADDR_W = 11;
    parameter TEST_INPUT_WORD_W = 10;
    parameter TEST_EXTR_WORD_W = 12;
    parameter TEST_NUM_HALF_ITER = 4'd11;
    parameter TEST_QPP_LUT_FILE = "data/qpp_6144.hex";

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
    reg [TEST_ROW_ADDR_W-1:0] load_addr;
    reg [TEST_INPUT_WORD_W-1:0] load_data;
    wire       decode_done;
    reg [TEST_ROW_ADDR_W-1:0] ld_rd_addr;
    wire [TEST_EXTR_WORD_W-1:0] ld_rd_data;
    wire [TEST_EXTR_WORD_W-1:0] ld_rd_data_odd;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    turbo_decoder #(
        .K(TEST_K),
        .NUM_SISO(TEST_NUM_SISO),
        .SEG_LEN(TEST_SEG_LEN),
        .TAIL_LEN(TEST_TAIL_LEN),
        .PAPER_BOUNDARY(TEST_PAPER_BOUNDARY),
        .ROW_DEPTH(TEST_ROW_DEPTH),
        .INPUT_ROW_DEPTH(TEST_INPUT_ROW_DEPTH),
        .ROW_ADDR_W(TEST_ROW_ADDR_W),
        .INPUT_WORD_W(TEST_INPUT_WORD_W),
        .EXTR_WORD_W(TEST_EXTR_WORD_W),
        .QPP_LUT_FILE(TEST_QPP_LUT_FILE),
        .NUM_HALF_ITER(TEST_NUM_HALF_ITER)
    ) u_dut (
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
    reg [TEST_INPUT_WORD_W-1:0] sys_even_mem [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] sys_odd_mem  [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] par1_even_mem[0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] par1_odd_mem [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] silv_even_mem[0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] silv_odd_mem [0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] par2_even_mem[0:TEST_INPUT_ROW_DEPTH-1];
    reg [TEST_INPUT_WORD_W-1:0] par2_odd_mem [0:TEST_INPUT_ROW_DEPTH-1];
    reg true_info_bits [0:TEST_K-1];

    initial begin
        $readmemh("data/sys_even_ram.hex",     sys_even_mem);
        $readmemh("data/sys_odd_ram.hex",      sys_odd_mem);
        $readmemh("data/par1_even_ram.hex",    par1_even_mem);
        $readmemh("data/par1_odd_ram.hex",     par1_odd_mem);
        $readmemh("data/sys_ilv_even_ram.hex", silv_even_mem);
        $readmemh("data/sys_ilv_odd_ram.hex",  silv_odd_mem);
        $readmemh("data/par2_even_ram.hex",    par2_even_mem);
        $readmemh("data/par2_odd_ram.hex",     par2_odd_mem);
        $readmemb("data/true_info_bits.txt",    true_info_bits);
    end

    // =========================================================================
    // Monitoring
    // =========================================================================
    integer done_count;
    integer half_iter_monitor;
    integer rtl_intr_count;
    integer capture_addr;
    integer init_idx;
    integer fd_boundary_trace;
    reg signed [9:0] rtl_final_intrinsic [0:TEST_K-1];

    initial begin
        done_count = 0;
        rtl_intr_count = 0;
        fd_boundary_trace = 0;
        for (init_idx = 0; init_idx < TEST_K; init_idx = init_idx + 1)
            rtl_final_intrinsic[init_idx] = 10'sd0;
        if ($test$plusargs("trace_boundary"))
            fd_boundary_trace = $fopen("data/rtl_boundary_trace.txt", "w");
    end

    always @(posedge clk) begin
        if (u_dut.c0_done) begin
            done_count = done_count + 1;
            $display("[%0t] Core done pulse #%0d, half_iter_cnt=%0d",
                     $time, done_count, u_dut.half_iter_cnt);
        end
    end

    // Dump the actual RTL final intrinsic L_D values used for hard decisions.
    // ld_ram_output.hex stores final extrinsics; BER must use these intrinsic
    // decision LLRs from the last half-iteration instead.
    always @(posedge clk) begin
        #1; // sample after nonblocking updates in the DUT
        if (u_dut.c0_llr_out_valid && (u_dut.half_iter_cnt == TEST_NUM_HALF_ITER - 4'd1)) begin
            capture_addr = u_dut.c0_llr_out_addr;
            rtl_final_intrinsic[capture_addr]        = u_dut.c0_l_intr_even;
            rtl_final_intrinsic[capture_addr + 1]    = u_dut.c0_l_intr_odd;
            rtl_final_intrinsic[capture_addr + TEST_SEG_LEN] = u_dut.c1_l_intr_even;
            rtl_final_intrinsic[capture_addr + TEST_SEG_LEN + 1] = u_dut.c1_l_intr_odd;
            rtl_intr_count = rtl_intr_count + 4;
        end
    end

    always @(posedge clk) begin
        #1;
        if (fd_boundary_trace != 0 && TEST_PAPER_BOUNDARY && u_dut.llr_valid_to_cores) begin
            $fwrite(fd_boundary_trace,
                    "LLR half=%0d c0_dbr=%0d c1_dbr=%0d cross=%0d c0_dbr_sys_e=%0d c0_dbr_sys_o=%0d c0_dbr_par_e=%0d c0_dbr_par_o=%0d c0_dbr_apr_e=%0d c0_dbr_apr_o=%0d c1_dbr_sys_e=%0d c1_dbr_sys_o=%0d c1_dbr_par_e=%0d c1_dbr_par_o=%0d\n",
                    u_dut.half_iter_cnt, u_dut.c0_dbr_addr, u_dut.c1_dbr_addr,
                    u_dut.c0_dbr_cross_fetch,
                    u_dut.c0_dbr_sys_even, u_dut.c0_dbr_sys_odd,
                    u_dut.c0_dbr_par_even, u_dut.c0_dbr_par_odd,
                    u_dut.c0_dbr_apr_even, u_dut.c0_dbr_apr_odd,
                    u_dut.c1_dbr_sys_even, u_dut.c1_dbr_sys_odd,
                    u_dut.c1_dbr_par_even, u_dut.c1_dbr_par_odd);
        end
        if (fd_boundary_trace != 0 && TEST_PAPER_BOUNDARY && u_dut.u_core1.dbr_window_done) begin
            $fwrite(fd_boundary_trace,
                    "C1_DBR_DONE half=%0d win=%0d beta=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                    u_dut.half_iter_cnt, u_dut.u_core1.dbr_win_idx,
                    u_dut.u_core1.dbr_final_beta_0, u_dut.u_core1.dbr_final_beta_1,
                    u_dut.u_core1.dbr_final_beta_2, u_dut.u_core1.dbr_final_beta_3,
                    u_dut.u_core1.dbr_final_beta_4, u_dut.u_core1.dbr_final_beta_5,
                    u_dut.u_core1.dbr_final_beta_6, u_dut.u_core1.dbr_final_beta_7);
        end
        if (fd_boundary_trace != 0 && TEST_PAPER_BOUNDARY && u_dut.u_core0.dbr_window_done) begin
            $fwrite(fd_boundary_trace,
                    "C0_DBR_DONE half=%0d win=%0d beta=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                    u_dut.half_iter_cnt, u_dut.u_core0.dbr_win_idx,
                    u_dut.u_core0.dbr_final_beta_0, u_dut.u_core0.dbr_final_beta_1,
                    u_dut.u_core0.dbr_final_beta_2, u_dut.u_core0.dbr_final_beta_3,
                    u_dut.u_core0.dbr_final_beta_4, u_dut.u_core0.dbr_final_beta_5,
                    u_dut.u_core0.dbr_final_beta_6, u_dut.u_core0.dbr_final_beta_7);
        end
        if (fd_boundary_trace != 0 && TEST_PAPER_BOUNDARY && u_dut.u_core1.fr_alpha_wr_en) begin
            $fwrite(fd_boundary_trace,
                    "C1_ALPHA_WRITE half=%0d win=%0d step=%0d alpha=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                    u_dut.half_iter_cnt, u_dut.u_core1.fr_win_idx, u_dut.u_core1.fr_alpha_wr_addr,
                    u_dut.u_core1.fr_alpha_wr_data_0, u_dut.u_core1.fr_alpha_wr_data_1,
                    u_dut.u_core1.fr_alpha_wr_data_2, u_dut.u_core1.fr_alpha_wr_data_3,
                    u_dut.u_core1.fr_alpha_wr_data_4, u_dut.u_core1.fr_alpha_wr_data_5,
                    u_dut.u_core1.fr_alpha_wr_data_6, u_dut.u_core1.fr_alpha_wr_data_7);
        end
        if (fd_boundary_trace != 0 && TEST_PAPER_BOUNDARY && u_dut.u_core1.br_load_beta) begin
            $fwrite(fd_boundary_trace,
                    "C1_BR_LOAD half=%0d win=%0d beta_reg=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                    u_dut.half_iter_cnt, u_dut.u_core1.br_win_idx,
                    u_dut.u_core1.br_beta_init_reg_0, u_dut.u_core1.br_beta_init_reg_1,
                    u_dut.u_core1.br_beta_init_reg_2, u_dut.u_core1.br_beta_init_reg_3,
                    u_dut.u_core1.br_beta_init_reg_4, u_dut.u_core1.br_beta_init_reg_5,
                    u_dut.u_core1.br_beta_init_reg_6, u_dut.u_core1.br_beta_init_reg_7);
        end
        if (fd_boundary_trace != 0 && TEST_PAPER_BOUNDARY && u_dut.c1_llr_out_valid) begin
            $fwrite(fd_boundary_trace,
                    "C1_LLR_OUT half=%0d addr=%0d intr_e=%0d intr_o=%0d extr_e=%0d extr_o=%0d\n",
                    u_dut.half_iter_cnt, u_dut.c1_llr_out_addr,
                    u_dut.c1_l_intr_even, u_dut.c1_l_intr_odd,
                    u_dut.c1_l_extr_even, u_dut.c1_l_extr_odd);
        end
        if (fd_boundary_trace != 0 && TEST_PAPER_BOUNDARY && u_dut.c0_llr_out_valid) begin
            $fwrite(fd_boundary_trace,
                    "C0_LLR_OUT half=%0d addr=%0d intr_e=%0d intr_o=%0d extr_e=%0d extr_o=%0d wb_row_e=%0d wb_row_o=%0d wb_perm_e=%0d wb_perm_o=%0d\n",
                    u_dut.half_iter_cnt, u_dut.c0_llr_out_addr,
                    u_dut.c0_l_intr_even, u_dut.c0_l_intr_odd,
                    u_dut.c0_l_extr_even, u_dut.c0_l_extr_odd,
                    u_dut.wb_row_even, u_dut.wb_row_odd,
                    u_dut.wb_perm_even, u_dut.wb_perm_odd);
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
            for (i = 0; i < TEST_INPUT_ROW_DEPTH; i = i + 1) begin
                @(posedge clk);
                load_en       <= 1'b1;
                load_bram_sel <= sel;
                load_addr     <= i[TEST_ROW_ADDR_W-1:0];
                case (sel)
                    3'b000: load_data <= sys_odd_mem[i];
                    3'b001: load_data <= sys_even_mem[i];
                    3'b010: load_data <= par1_odd_mem[i];
                    3'b011: load_data <= par1_even_mem[i];
                    3'b100: load_data <= silv_odd_mem[i];
                    3'b101: load_data <= silv_even_mem[i];
                    3'b110: load_data <= par2_odd_mem[i];
                    3'b111: load_data <= par2_even_mem[i];
                    default: load_data <= {TEST_INPUT_WORD_W{1'b0}};
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
    integer fd_intr;
    integer fd_bits;
    integer fd_ber;
    integer rtl_bit_errors;
    integer rtl_hard_decision;
    integer meta_fd;
    integer meta_scan;
    integer meta_k;
    integer meta_num_siso;
    integer meta_seg_len;
    integer meta_qpp_f1;
    integer meta_qpp_f2;
    integer meta_seed;
    integer meta_tail_len;
    integer meta_channel_errors;
    integer meta_quant_sys_errors;
    real    meta_ebn0_db;
    real    meta_code_rate;
    real    meta_channel_ber;
    real    meta_quant_sys_ber;

    task read_vector_metadata;
        begin
            meta_k = TEST_K;
            meta_num_siso = TEST_NUM_SISO;
            meta_seg_len = TEST_SEG_LEN;
            meta_qpp_f1 = 0;
            meta_qpp_f2 = 0;
            meta_ebn0_db = -1.0;
            meta_code_rate = -1.0;
            meta_seed = -1;
            meta_tail_len = TEST_TAIL_LEN;
            meta_channel_errors = -1;
            meta_channel_ber = -1.0;
            meta_quant_sys_errors = -1;
            meta_quant_sys_ber = -1.0;

            meta_fd = $fopen("data/vector_metadata.txt", "r");
            if (meta_fd == 0) begin
                $display("WARNING: Could not open data/vector_metadata.txt; vector specs will use testbench parameters only.");
            end else begin
                meta_scan = $fscanf(meta_fd, "K=%d\n", meta_k);
                meta_scan = $fscanf(meta_fd, "NUM_SISO=%d\n", meta_num_siso);
                meta_scan = $fscanf(meta_fd, "SEG_LEN=%d\n", meta_seg_len);
                meta_scan = $fscanf(meta_fd, "QPP_F1=%d\n", meta_qpp_f1);
                meta_scan = $fscanf(meta_fd, "QPP_F2=%d\n", meta_qpp_f2);
                meta_scan = $fscanf(meta_fd, "EBN0_DB=%f\n", meta_ebn0_db);
                meta_scan = $fscanf(meta_fd, "CODE_RATE=%f\n", meta_code_rate);
                meta_scan = $fscanf(meta_fd, "SEED=%d\n", meta_seed);
                meta_scan = $fscanf(meta_fd, "TAIL_LEN=%d\n", meta_tail_len);
                meta_scan = $fscanf(meta_fd, "CHANNEL_ERRORS=%d\n", meta_channel_errors);
                meta_scan = $fscanf(meta_fd, "CHANNEL_BER=%f\n", meta_channel_ber);
                meta_scan = $fscanf(meta_fd, "QUANT_SYS_ERRORS=%d\n", meta_quant_sys_errors);
                meta_scan = $fscanf(meta_fd, "QUANT_SYS_BER=%f\n", meta_quant_sys_ber);
                $fclose(meta_fd);
            end
        end
    endtask

    task print_vector_specs;
        begin
            $display("------------------------------------------------------------");
            $display("RTL Turbo Decoder Test Vector Specs");
            $display("  K                  = %0d", meta_k);
            $display("  NUM_SISO           = %0d", meta_num_siso);
            $display("  Segment length     = %0d", meta_seg_len);
            $display("  QPP f1/f2          = %0d/%0d", meta_qpp_f1, meta_qpp_f2);
            $display("  Eb/N0              = %0.3f dB", meta_ebn0_db);
            $display("  Channel/code rate  = %0.6f", meta_code_rate);
            $display("  Seed               = %0d", meta_seed);
            $display("  Tail trellis steps = %0d", meta_tail_len);
            $display("  Paper boundary     = %0d", TEST_PAPER_BOUNDARY);
            $display("  Half iterations    = %0d", TEST_NUM_HALF_ITER);
            $display("  QPP LUT file       = %s", TEST_QPP_LUT_FILE);
            $display("  Channel hard BER   = %0d/%0d = %0.12f", meta_channel_errors, TEST_K, meta_channel_ber);
            $display("  Quantized sys BER  = %0d/%0d = %0.12f", meta_quant_sys_errors, TEST_K, meta_quant_sys_ber);
            $display("------------------------------------------------------------");
        end
    endtask

    initial begin
        // Waveform dump is opt-in; full-frame BER runs otherwise create
        // very large VCD files and slow the simulator substantially.
        if ($test$plusargs("dump_vcd")) begin
            $dumpfile("tb_turbo_decoder.vcd");
            $dumpvars(0, tb_turbo_decoder);
        end

        read_vector_metadata();
        print_vector_specs();

        // Reset
        rst_n    = 0;
        start    = 0;
        load_en  = 0;
        load_bram_sel = 3'd0;
        load_addr     = {TEST_ROW_ADDR_W{1'b0}};
        load_data     = {TEST_INPUT_WORD_W{1'b0}};
        ld_rd_addr    = {TEST_ROW_ADDR_W{1'b0}};

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
        if (done_count != TEST_NUM_HALF_ITER)
            $display("ERROR: Expected %0d done pulses, got %0d", TEST_NUM_HALF_ITER, done_count);
        else
            $display("PASS: Correct number of done pulses (%0d).", TEST_NUM_HALF_ITER);

        // ---- Read LD RAM (even and odd) and dump to file ----
        fd = $fopen("data/ld_ram_output.hex", "w");
        for (i = 0; i < TEST_ROW_DEPTH; i = i + 1) begin
            @(posedge clk);
            ld_rd_addr <= i[TEST_ROW_ADDR_W-1:0];
            @(posedge clk); // 1-cycle read latency
            #1; // sample after ld_bram's nonblocking read update
            $fwrite(fd, "%03x %03x\n", ld_rd_data, ld_rd_data_odd);
        end
        $fclose(fd);
        $display("[%0t] LD RAM output (even+odd) written to ld_ram_output.hex", $time);

        // ---- Dump RTL final intrinsic decision LLRs and hard bits ----
        fd_intr = $fopen("data/rtl_final_intrinsic.txt", "w");
        fd_bits = $fopen("data/rtl_final_hard_bits.txt", "w");
        rtl_bit_errors = 0;
        for (i = 0; i < TEST_K; i = i + 1) begin
            rtl_hard_decision = (rtl_final_intrinsic[i] < 0);
            $fwrite(fd_intr, "%0d\n", rtl_final_intrinsic[i]);
            $fwrite(fd_bits, "%0d\n", rtl_hard_decision);
            if (rtl_hard_decision != true_info_bits[i])
                rtl_bit_errors = rtl_bit_errors + 1;
        end
        $fclose(fd_intr);
        $fclose(fd_bits);
        if (rtl_intr_count != TEST_K)
            $display("ERROR: Captured %0d RTL final intrinsic values, expected %0d.", rtl_intr_count, TEST_K);
        else
            $display("[%0t] RTL final intrinsic LLRs and hard bits written (%0d bits).", $time, rtl_intr_count);

        fd_ber = $fopen("data/rtl_ber_results.txt", "w");
        $fwrite(fd_ber, "RTL Testbench BER Results\n");
        $fwrite(fd_ber, "=========================\n");
        $fwrite(fd_ber, "K: %0d\n", TEST_K);
        $fwrite(fd_ber, "NUM_SISO: %0d\n", TEST_NUM_SISO);
        $fwrite(fd_ber, "Segment length: %0d\n", TEST_SEG_LEN);
        $fwrite(fd_ber, "QPP f1/f2: %0d/%0d\n", meta_qpp_f1, meta_qpp_f2);
        $fwrite(fd_ber, "Eb/N0 dB: %0.3f\n", meta_ebn0_db);
        $fwrite(fd_ber, "Channel/code rate: %0.6f\n", meta_code_rate);
        $fwrite(fd_ber, "Seed: %0d\n", meta_seed);
        $fwrite(fd_ber, "Tail trellis steps: %0d\n", TEST_TAIL_LEN);
        $fwrite(fd_ber, "Paper boundary mode: %0d\n", TEST_PAPER_BOUNDARY);
        $fwrite(fd_ber, "Half iterations: %0d\n", TEST_NUM_HALF_ITER);
        $fwrite(fd_ber, "Channel hard BER: %0d/%0d = %.12f\n", meta_channel_errors, TEST_K, meta_channel_ber);
        $fwrite(fd_ber, "Quantized systematic BER: %0d/%0d = %.12f\n", meta_quant_sys_errors, TEST_K, meta_quant_sys_ber);
        $fwrite(fd_ber, "BER signal: final intrinsic L_D hard decision\n");
        $fwrite(fd_ber, "Errors: %0d\n", rtl_bit_errors);
        $fwrite(fd_ber, "Bits: %0d\n", TEST_K);
        $fwrite(fd_ber, "BER: %.12f\n", rtl_bit_errors * 1.0 / TEST_K);
        $fclose(fd_ber);
        $display("------------------------------------------------------------");
        $display("RTL Turbo Decoder BER Results");
        $display("  Eb/N0              = %0.3f dB", meta_ebn0_db);
        $display("  Channel/code rate  = %0.6f", meta_code_rate);
        $display("  Seed               = %0d", meta_seed);
        $display("  Channel hard BER   = %0d/%0d = %0.12f", meta_channel_errors, TEST_K, meta_channel_ber);
        $display("  Quantized sys BER  = %0d/%0d = %0.12f", meta_quant_sys_errors, TEST_K, meta_quant_sys_ber);
        $display("  RTL final L_D BER  = %0d/%0d = %0.12f", rtl_bit_errors, TEST_K, rtl_bit_errors * 1.0 / TEST_K);
        $display("  Report file        = data/rtl_ber_results.txt");
        $display("------------------------------------------------------------");
        if (fd_boundary_trace != 0)
            $fclose(fd_boundary_trace);

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
