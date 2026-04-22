//==============================================================================
// Module: turbo_decoder
// Description: Top-level parallel LTE Turbo Decoder.
//   Instantiates 2 bcjr_core SISO decoders, 8 Group-A input BRAMs,
//   2 Group-B extrinsic BRAMs, 1 Group-C LD output BRAM,
//   1 QPP interleaver LUT ROM, 6 master_net + 6 slave_net instances.
//   Implements the fetch pipeline FSM and write-back logic for both
//   natural and interleaved half-iterations.
//
// Parameters: K=6144, N=2, S=3072, W=30, NUM_WINDOWS=103, 11 half-iters.
// Target: Zynq-7010 FPGA, Verilog-2001.
//==============================================================================
module turbo_decoder (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // Host load interface
    input  wire        load_en,
    input  wire [2:0]  load_bram_sel,
    input  wire [10:0] load_addr,
    input  wire [9:0]  load_data,

    // Result interface
    output reg         decode_done,
    input  wire [10:0] ld_rd_addr,
    output wire [11:0] ld_rd_data,
    output wire [11:0] ld_rd_data_odd
);

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam FRAME_LEN     = 12'd3072;
    localparam NUM_HALF_ITER = 4'd11;

    // Main FSM
    localparam ST_IDLE      = 3'd0;
    localparam ST_RUNNING   = 3'd1;
    localparam ST_HALF_DONE = 3'd2;
    localparam ST_WRITE_LD  = 3'd3;

    // Fetch sub-FSM
    localparam F_IDLE  = 3'd0;
    localparam F_C1    = 3'd1;
    localparam F_C2    = 3'd2;
    localparam F_STALL = 3'd3;
    localparam F_C2B   = 3'd4;

    // =========================================================================
    // FSM registers
    // =========================================================================
    reg [2:0]  main_state;
    reg [2:0]  fetch_state;
    reg [3:0]  half_iter_cnt;
    wire       is_interleaved = half_iter_cnt[0];

    // =========================================================================
    // Core control
    // =========================================================================
    reg  start_cores;
    reg  llr_valid_to_cores;

    // =========================================================================
    // Core 0 interface
    // =========================================================================
    wire        c0_done, c0_llr_req;
    wire [11:0] c0_fr_addr, c0_br_addr, c0_dbr_addr;
    wire        c0_llr_out_valid;
    wire [11:0] c0_llr_out_addr;
    wire signed [5:0] c0_l_extr_odd, c0_l_extr_even;

    // =========================================================================
    // Core 1 interface
    // =========================================================================
    wire        c1_done, c1_llr_req;
    wire [11:0] c1_fr_addr, c1_br_addr, c1_dbr_addr;
    wire        c1_llr_out_valid;
    wire [11:0] c1_llr_out_addr;
    wire signed [5:0] c1_l_extr_odd, c1_l_extr_even;

    // =========================================================================
    // LLR input registers — 18 per core, 5-bit (matching bcjr_core ports)
    // =========================================================================
    reg signed [4:0] c0_fr_sys_odd, c0_fr_sys_even;
    reg signed [4:0] c0_fr_par_odd, c0_fr_par_even;
    reg signed [4:0] c0_fr_apr_odd, c0_fr_apr_even;
    reg signed [4:0] c0_br_sys_odd, c0_br_sys_even;
    reg signed [4:0] c0_br_par_odd, c0_br_par_even;
    reg signed [4:0] c0_br_apr_odd, c0_br_apr_even;
    reg signed [4:0] c0_dbr_sys_odd, c0_dbr_sys_even;
    reg signed [4:0] c0_dbr_par_odd, c0_dbr_par_even;
    reg signed [4:0] c0_dbr_apr_odd, c0_dbr_apr_even;

    reg signed [4:0] c1_fr_sys_odd, c1_fr_sys_even;
    reg signed [4:0] c1_fr_par_odd, c1_fr_par_even;
    reg signed [4:0] c1_fr_apr_odd, c1_fr_apr_even;
    reg signed [4:0] c1_br_sys_odd, c1_br_sys_even;
    reg signed [4:0] c1_br_par_odd, c1_br_par_even;
    reg signed [4:0] c1_br_apr_odd, c1_br_apr_even;
    reg signed [4:0] c1_dbr_sys_odd, c1_dbr_sys_even;
    reg signed [4:0] c1_dbr_par_odd, c1_dbr_par_even;
    reg signed [4:0] c1_dbr_apr_odd, c1_dbr_apr_even;

    // =========================================================================
    // Address computation (both cores output same local addresses)
    // =========================================================================
    wire [10:0] fr_row  = c0_fr_addr[11:1];
    wire [10:0] br_row  = c0_br_addr[11:1];
    wire [10:0] dbr_row = c0_dbr_addr[11:1];

    // Out-of-range detection — last window may exceed valid address space.
    // BCJR core contract: top-level must provide zero LLRs for OOR addresses.
    wire fr_oor  = (c0_fr_addr  >= FRAME_LEN);
    wire br_oor  = (c0_br_addr  >= FRAME_LEN);
    wire dbr_oor = (c0_dbr_addr >= FRAME_LEN);

    // =========================================================================
    // Perm bit registers (interleaved phase, registered in F_C1)
    // =========================================================================
    reg perm_fr_even, perm_fr_odd;
    reg perm_br_even, perm_br_odd;
    reg perm_dbr_even, perm_dbr_odd;

    // Registered extrinsic read rows for stall detection
    reg [10:0] extr_rd_row_fr_e, extr_rd_row_br_e, extr_rd_row_dbr_e;
    reg [10:0] extr_rd_row_fr_o, extr_rd_row_br_o, extr_rd_row_dbr_o;

    // =========================================================================
    // Write-back pipeline (interleaved: 1-cycle delay for LUT latency)
    // =========================================================================
    reg        wb_ilv_valid_d1;
    reg signed [5:0] wb_c0_even_d1, wb_c0_odd_d1;
    reg signed [5:0] wb_c1_even_d1, wb_c1_odd_d1;

    // =========================================================================
    // LD copy registers
    // =========================================================================
    reg [10:0] ld_cnt;
    reg        ld_rd_valid;
    reg [10:0] ld_wr_idx;

    // =========================================================================
    // Group-A BRAM read-data wires (10-bit: {col1[4:0], col0[4:0]})
    // =========================================================================
    wire [9:0] sys_even_fr, sys_even_br, sys_even_dbr;
    wire [9:0] sys_odd_fr,  sys_odd_br,  sys_odd_dbr;
    wire [9:0] par1_even_fr, par1_even_br, par1_even_dbr;
    wire [9:0] par1_odd_fr,  par1_odd_br,  par1_odd_dbr;
    wire [9:0] silv_even_fr, silv_even_br, silv_even_dbr;
    wire [9:0] silv_odd_fr,  silv_odd_br,  silv_odd_dbr;
    wire [9:0] par2_even_fr, par2_even_br, par2_even_dbr;
    wire [9:0] par2_odd_fr,  par2_odd_br,  par2_odd_dbr;

    // Phase-selected sys/par MUX (natural vs interleaved)
    wire [9:0] s_even_fr  = is_interleaved ? silv_even_fr  : sys_even_fr;
    wire [9:0] s_even_br  = is_interleaved ? silv_even_br  : sys_even_br;
    wire [9:0] s_even_dbr = is_interleaved ? silv_even_dbr : sys_even_dbr;
    wire [9:0] s_odd_fr   = is_interleaved ? silv_odd_fr   : sys_odd_fr;
    wire [9:0] s_odd_br   = is_interleaved ? silv_odd_br   : sys_odd_br;
    wire [9:0] s_odd_dbr  = is_interleaved ? silv_odd_dbr  : sys_odd_dbr;
    wire [9:0] p_even_fr  = is_interleaved ? par2_even_fr  : par1_even_fr;
    wire [9:0] p_even_br  = is_interleaved ? par2_even_br  : par1_even_br;
    wire [9:0] p_even_dbr = is_interleaved ? par2_even_dbr : par1_even_dbr;
    wire [9:0] p_odd_fr   = is_interleaved ? par2_odd_fr   : par1_odd_fr;
    wire [9:0] p_odd_br   = is_interleaved ? par2_odd_br   : par1_odd_br;
    wire [9:0] p_odd_dbr  = is_interleaved ? par2_odd_dbr  : par1_odd_dbr;

    // =========================================================================
    // Group-B extrinsic BRAM read-data wires (12-bit: {col1[5:0], col0[5:0]})
    // =========================================================================
    wire [11:0] extr_even_fr, extr_even_br, extr_even_dbr;
    wire [11:0] extr_odd_fr,  extr_odd_br,  extr_odd_dbr;

    // =========================================================================
    // QPP LUT wires (13-bit outputs)
    // =========================================================================
    wire [12:0] pi_fr_even, pi_fr_odd;
    wire [12:0] pi_br_even, pi_br_odd;
    wire [12:0] pi_dbr_even, pi_dbr_odd;

    // LUT FR port address mux (reused for write-back during backward pass)
    wire use_lut_wb = is_interleaved && c0_llr_out_valid;
    wire [11:0] lut_fr_e_addr = use_lut_wb ? c0_llr_out_addr : c0_fr_addr;
    wire [11:0] lut_fr_o_addr = use_lut_wb ? (c0_llr_out_addr + 12'd1)
                                           : (c0_fr_addr + 12'd1);

    // =========================================================================
    // Master net wires (6 fetch instances)
    // =========================================================================
    wire [10:0] mn_fr_e_row, mn_fr_o_row;
    wire [10:0] mn_br_e_row, mn_br_o_row;
    wire [10:0] mn_dbr_e_row, mn_dbr_o_row;
    wire mn_fr_e_perm, mn_fr_o_perm;
    wire mn_br_e_perm, mn_br_o_perm;
    wire mn_dbr_e_perm, mn_dbr_o_perm;
    wire [12:0] mn_fr_e_pi1, mn_fr_o_pi1;
    wire [12:0] mn_br_e_pi1, mn_br_o_pi1;
    wire [12:0] mn_dbr_e_pi1, mn_dbr_o_pi1;

    // Write-back row/perm reuse FR master_net (LUT FR ports are muxed)
    wire [10:0] wb_row_even = mn_fr_e_row;
    wire [10:0] wb_row_odd  = mn_fr_o_row;
    wire        wb_perm_even = mn_fr_e_perm;
    wire        wb_perm_odd  = mn_fr_o_perm;

    // =========================================================================
    // Slave net wires (6 instances, 6-bit outputs)
    // =========================================================================
    wire signed [5:0] sl_fr_e_c0, sl_fr_e_c1;
    wire signed [5:0] sl_fr_o_c0, sl_fr_o_c1;
    wire signed [5:0] sl_br_e_c0, sl_br_e_c1;
    wire signed [5:0] sl_br_o_c0, sl_br_o_c1;
    wire signed [5:0] sl_dbr_e_c0, sl_dbr_e_c1;
    wire signed [5:0] sl_dbr_o_c0, sl_dbr_o_c1;

    // =========================================================================
    // Extrinsic BRAM address mux
    // =========================================================================
    wire [10:0] extr_e_rd0 = (main_state == ST_WRITE_LD) ? ld_cnt :
                              is_interleaved ? mn_fr_e_row : fr_row;
    wire [10:0] extr_e_rd1 = is_interleaved ? mn_br_e_row  : br_row;
    wire [10:0] extr_e_rd2 = is_interleaved ? mn_dbr_e_row : dbr_row;
    wire [10:0] extr_o_rd0 = (main_state == ST_WRITE_LD) ? ld_cnt :
                              is_interleaved ? mn_fr_o_row  : fr_row;
    wire [10:0] extr_o_rd1 = is_interleaved ? mn_br_o_row  : br_row;
    wire [10:0] extr_o_rd2 = is_interleaved ? mn_dbr_o_row : dbr_row;

    // =========================================================================
    // Extrinsic BRAM write mux
    // =========================================================================
    wire nat_wb = (!is_interleaved) && c0_llr_out_valid && (main_state == ST_RUNNING);
    wire ilv_wb = wb_ilv_valid_d1;

    wire        extr_e_wr = nat_wb || ilv_wb;
    wire [10:0] extr_e_wa = nat_wb ? c0_llr_out_addr[11:1] : wb_row_even;
    wire [11:0] extr_e_wd = nat_wb ? {c1_l_extr_even, c0_l_extr_even} :
                            wb_perm_even ? {wb_c0_even_d1, wb_c1_even_d1} :
                                           {wb_c1_even_d1, wb_c0_even_d1};

    wire        extr_o_wr = nat_wb || ilv_wb;
    wire [10:0] extr_o_wa = nat_wb ? c0_llr_out_addr[11:1] : wb_row_odd;
    wire [11:0] extr_o_wd = nat_wb ? {c1_l_extr_odd, c0_l_extr_odd} :
                            wb_perm_odd ? {wb_c0_odd_d1, wb_c1_odd_d1} :
                                          {wb_c1_odd_d1, wb_c0_odd_d1};

    // =========================================================================
    // LD BRAM write signals
    // =========================================================================
    wire        ld_w_en       = ld_rd_valid;
    wire [10:0] ld_w_addr     = ld_wr_idx;
    wire [11:0] ld_w_data     = extr_even_fr;
    wire [11:0] ld_w_data_odd = extr_odd_fr;

    // Stall detection wires (interleaved write vs extrinsic read conflict)
    wire stall_detect = ilv_wb && (
        (wb_row_even == extr_rd_row_fr_e) || (wb_row_even == extr_rd_row_br_e) ||
        (wb_row_even == extr_rd_row_dbr_e) ||
        (wb_row_odd  == extr_rd_row_fr_o)  || (wb_row_odd  == extr_rd_row_br_o) ||
        (wb_row_odd  == extr_rd_row_dbr_o));

    // =====================================================================
    //  CORE INSTANCES
    // =====================================================================
    wire in_idle = (main_state == ST_IDLE);

    bcjr_core #(.CORE_ID(0), .NUM_SISO(2), .NUM_WINDOWS(103)) u_core0 (
        .clk(clk), .rst_n(rst_n), .start(start_cores), .frame_len(FRAME_LEN),
        .done(c0_done), .llr_req(c0_llr_req),
        .fr_llr_addr(c0_fr_addr), .br_llr_addr(c0_br_addr), .dbr_llr_addr(c0_dbr_addr),
        .llr_valid(llr_valid_to_cores),
        .fr_sys_odd(c0_fr_sys_odd), .fr_sys_even(c0_fr_sys_even),
        .fr_par_odd(c0_fr_par_odd), .fr_par_even(c0_fr_par_even),
        .fr_apr_odd(c0_fr_apr_odd), .fr_apr_even(c0_fr_apr_even),
        .br_sys_odd(c0_br_sys_odd), .br_sys_even(c0_br_sys_even),
        .br_par_odd(c0_br_par_odd), .br_par_even(c0_br_par_even),
        .br_apr_odd(c0_br_apr_odd), .br_apr_even(c0_br_apr_even),
        .dbr_sys_odd(c0_dbr_sys_odd), .dbr_sys_even(c0_dbr_sys_even),
        .dbr_par_odd(c0_dbr_par_odd), .dbr_par_even(c0_dbr_par_even),
        .dbr_apr_odd(c0_dbr_apr_odd), .dbr_apr_even(c0_dbr_apr_even),
        .llr_extr_odd_out(c0_l_extr_odd), .llr_extr_even_out(c0_l_extr_even),
        .llr_out_addr(c0_llr_out_addr), .llr_out_valid(c0_llr_out_valid)
    );

    bcjr_core #(.CORE_ID(1), .NUM_SISO(2), .NUM_WINDOWS(103)) u_core1 (
        .clk(clk), .rst_n(rst_n), .start(start_cores), .frame_len(FRAME_LEN),
        .done(c1_done), .llr_req(c1_llr_req),
        .fr_llr_addr(c1_fr_addr), .br_llr_addr(c1_br_addr), .dbr_llr_addr(c1_dbr_addr),
        .llr_valid(llr_valid_to_cores),
        .fr_sys_odd(c1_fr_sys_odd), .fr_sys_even(c1_fr_sys_even),
        .fr_par_odd(c1_fr_par_odd), .fr_par_even(c1_fr_par_even),
        .fr_apr_odd(c1_fr_apr_odd), .fr_apr_even(c1_fr_apr_even),
        .br_sys_odd(c1_br_sys_odd), .br_sys_even(c1_br_sys_even),
        .br_par_odd(c1_br_par_odd), .br_par_even(c1_br_par_even),
        .br_apr_odd(c1_br_apr_odd), .br_apr_even(c1_br_apr_even),
        .dbr_sys_odd(c1_dbr_sys_odd), .dbr_sys_even(c1_dbr_sys_even),
        .dbr_par_odd(c1_dbr_par_odd), .dbr_par_even(c1_dbr_par_even),
        .dbr_apr_odd(c1_dbr_apr_odd), .dbr_apr_even(c1_dbr_apr_even),
        .llr_extr_odd_out(c1_l_extr_odd), .llr_extr_even_out(c1_l_extr_even),
        .llr_out_addr(c1_llr_out_addr), .llr_out_valid(c1_llr_out_valid)
    );

    // =====================================================================
    //  GROUP-A INPUT BRAM INSTANCES (8)
    //  load_bram_sel: 000=sys_odd 001=sys_even 010=par1_odd 011=par1_even
    //                 100=silv_odd 101=silv_even 110=par2_odd 111=par2_even
    // =====================================================================
    input_bram u_sys_even (.clk(clk),
        .wr_en(load_en && in_idle && (load_bram_sel==3'b001)),
        .wr_addr(load_addr), .wr_data(load_data),
        .rd_addr_0(fr_row), .rd_data_0(sys_even_fr),
        .rd_addr_1(br_row), .rd_data_1(sys_even_br),
        .rd_addr_2(dbr_row),.rd_data_2(sys_even_dbr));

    input_bram u_sys_odd (.clk(clk),
        .wr_en(load_en && in_idle && (load_bram_sel==3'b000)),
        .wr_addr(load_addr), .wr_data(load_data),
        .rd_addr_0(fr_row), .rd_data_0(sys_odd_fr),
        .rd_addr_1(br_row), .rd_data_1(sys_odd_br),
        .rd_addr_2(dbr_row),.rd_data_2(sys_odd_dbr));

    input_bram u_par1_even (.clk(clk),
        .wr_en(load_en && in_idle && (load_bram_sel==3'b011)),
        .wr_addr(load_addr), .wr_data(load_data),
        .rd_addr_0(fr_row), .rd_data_0(par1_even_fr),
        .rd_addr_1(br_row), .rd_data_1(par1_even_br),
        .rd_addr_2(dbr_row),.rd_data_2(par1_even_dbr));

    input_bram u_par1_odd (.clk(clk),
        .wr_en(load_en && in_idle && (load_bram_sel==3'b010)),
        .wr_addr(load_addr), .wr_data(load_data),
        .rd_addr_0(fr_row), .rd_data_0(par1_odd_fr),
        .rd_addr_1(br_row), .rd_data_1(par1_odd_br),
        .rd_addr_2(dbr_row),.rd_data_2(par1_odd_dbr));

    input_bram u_silv_even (.clk(clk),
        .wr_en(load_en && in_idle && (load_bram_sel==3'b101)),
        .wr_addr(load_addr), .wr_data(load_data),
        .rd_addr_0(fr_row), .rd_data_0(silv_even_fr),
        .rd_addr_1(br_row), .rd_data_1(silv_even_br),
        .rd_addr_2(dbr_row),.rd_data_2(silv_even_dbr));

    input_bram u_silv_odd (.clk(clk),
        .wr_en(load_en && in_idle && (load_bram_sel==3'b100)),
        .wr_addr(load_addr), .wr_data(load_data),
        .rd_addr_0(fr_row), .rd_data_0(silv_odd_fr),
        .rd_addr_1(br_row), .rd_data_1(silv_odd_br),
        .rd_addr_2(dbr_row),.rd_data_2(silv_odd_dbr));

    input_bram u_par2_even (.clk(clk),
        .wr_en(load_en && in_idle && (load_bram_sel==3'b111)),
        .wr_addr(load_addr), .wr_data(load_data),
        .rd_addr_0(fr_row), .rd_data_0(par2_even_fr),
        .rd_addr_1(br_row), .rd_data_1(par2_even_br),
        .rd_addr_2(dbr_row),.rd_data_2(par2_even_dbr));

    input_bram u_par2_odd (.clk(clk),
        .wr_en(load_en && in_idle && (load_bram_sel==3'b110)),
        .wr_addr(load_addr), .wr_data(load_data),
        .rd_addr_0(fr_row), .rd_data_0(par2_odd_fr),
        .rd_addr_1(br_row), .rd_data_1(par2_odd_br),
        .rd_addr_2(dbr_row),.rd_data_2(par2_odd_dbr));

    // =====================================================================
    //  GROUP-B EXTRINSIC BRAM INSTANCES (2)
    // =====================================================================
    extrinsic_bram u_extr_even (.clk(clk),
        .wr_en(extr_e_wr), .wr_addr(extr_e_wa), .wr_data(extr_e_wd),
        .rd_addr_0(extr_e_rd0), .rd_data_0(extr_even_fr),
        .rd_addr_1(extr_e_rd1), .rd_data_1(extr_even_br),
        .rd_addr_2(extr_e_rd2), .rd_data_2(extr_even_dbr));

    extrinsic_bram u_extr_odd (.clk(clk),
        .wr_en(extr_o_wr), .wr_addr(extr_o_wa), .wr_data(extr_o_wd),
        .rd_addr_0(extr_o_rd0), .rd_data_0(extr_odd_fr),
        .rd_addr_1(extr_o_rd1), .rd_data_1(extr_odd_br),
        .rd_addr_2(extr_o_rd2), .rd_data_2(extr_odd_dbr));

    // =====================================================================
    //  GROUP-C LD BRAM
    // =====================================================================
    ld_bram u_ld_ram_even (.clk(clk),
        .wr_en(ld_w_en), .wr_addr(ld_w_addr), .wr_data(ld_w_data),
        .rd_addr(ld_rd_addr), .rd_data(ld_rd_data));

    ld_bram u_ld_ram_odd (.clk(clk),
        .wr_en(ld_w_en), .wr_addr(ld_w_addr), .wr_data(ld_w_data_odd),
        .rd_addr(ld_rd_addr), .rd_data(ld_rd_data_odd));

    // =====================================================================
    //  QPP INTERLEAVER LUT — addresses clamped to avoid X from OOR reads
    // =====================================================================
    wire [11:0] qpp_fr_e  = (lut_fr_e_addr < FRAME_LEN) ? lut_fr_e_addr        : 12'd0;
    wire [11:0] qpp_fr_o  = (lut_fr_o_addr < FRAME_LEN) ? lut_fr_o_addr        : 12'd0;
    wire [11:0] qpp_br_e  = (c0_br_addr    < FRAME_LEN) ? c0_br_addr            : 12'd0;
    wire [11:0] qpp_br_o  = ((c0_br_addr + 12'd1) < FRAME_LEN) ? (c0_br_addr + 12'd1) : 12'd0;
    wire [11:0] qpp_dbr_e = (c0_dbr_addr   < FRAME_LEN) ? c0_dbr_addr           : 12'd0;
    wire [11:0] qpp_dbr_o = ((c0_dbr_addr + 12'd1) < FRAME_LEN) ? (c0_dbr_addr + 12'd1) : 12'd0;

    qpp_lut u_qpp_lut (.clk(clk),
        .addr_fr_even(qpp_fr_e),
        .addr_fr_odd(qpp_fr_o),
        .addr_br_even(qpp_br_e),
        .addr_br_odd(qpp_br_o),
        .addr_dbr_even(qpp_dbr_e),
        .addr_dbr_odd(qpp_dbr_o),
        .pi_fr_even(pi_fr_even), .pi_fr_odd(pi_fr_odd),
        .pi_br_even(pi_br_even), .pi_br_odd(pi_br_odd),
        .pi_dbr_even(pi_dbr_even), .pi_dbr_odd(pi_dbr_odd));

    // =====================================================================
    //  MASTER NET INSTANCES (6 for fetch; FR pair reused for write-back)
    // =====================================================================
    master_net u_mn_fr_e  (.addr_siso0(pi_fr_even),  .pi_siso1(mn_fr_e_pi1),
        .bram_row(mn_fr_e_row),  .perm_bit(mn_fr_e_perm));
    master_net u_mn_fr_o  (.addr_siso0(pi_fr_odd),   .pi_siso1(mn_fr_o_pi1),
        .bram_row(mn_fr_o_row),  .perm_bit(mn_fr_o_perm));
    master_net u_mn_br_e  (.addr_siso0(pi_br_even),  .pi_siso1(mn_br_e_pi1),
        .bram_row(mn_br_e_row),  .perm_bit(mn_br_e_perm));
    master_net u_mn_br_o  (.addr_siso0(pi_br_odd),   .pi_siso1(mn_br_o_pi1),
        .bram_row(mn_br_o_row),  .perm_bit(mn_br_o_perm));
    master_net u_mn_dbr_e (.addr_siso0(pi_dbr_even), .pi_siso1(mn_dbr_e_pi1),
        .bram_row(mn_dbr_e_row), .perm_bit(mn_dbr_e_perm));
    master_net u_mn_dbr_o (.addr_siso0(pi_dbr_odd),  .pi_siso1(mn_dbr_o_pi1),
        .bram_row(mn_dbr_o_row), .perm_bit(mn_dbr_o_perm));

    // =====================================================================
    //  SLAVE NET INSTANCES (6)
    // =====================================================================
    slave_net u_sl_fr_e  (.perm_bit(perm_fr_even),
        .col0(extr_even_fr[5:0]),  .col1(extr_even_fr[11:6]),
        .out_siso0(sl_fr_e_c0),  .out_siso1(sl_fr_e_c1));
    slave_net u_sl_fr_o  (.perm_bit(perm_fr_odd),
        .col0(extr_odd_fr[5:0]),   .col1(extr_odd_fr[11:6]),
        .out_siso0(sl_fr_o_c0),  .out_siso1(sl_fr_o_c1));
    slave_net u_sl_br_e  (.perm_bit(perm_br_even),
        .col0(extr_even_br[5:0]),  .col1(extr_even_br[11:6]),
        .out_siso0(sl_br_e_c0),  .out_siso1(sl_br_e_c1));
    slave_net u_sl_br_o  (.perm_bit(perm_br_odd),
        .col0(extr_odd_br[5:0]),   .col1(extr_odd_br[11:6]),
        .out_siso0(sl_br_o_c0),  .out_siso1(sl_br_o_c1));
    slave_net u_sl_dbr_e (.perm_bit(perm_dbr_even),
        .col0(extr_even_dbr[5:0]), .col1(extr_even_dbr[11:6]),
        .out_siso0(sl_dbr_e_c0), .out_siso1(sl_dbr_e_c1));
    slave_net u_sl_dbr_o (.perm_bit(perm_dbr_odd),
        .col0(extr_odd_dbr[5:0]),  .col1(extr_odd_dbr[11:6]),
        .out_siso0(sl_dbr_o_c0), .out_siso1(sl_dbr_o_c1));

    // =====================================================================
    //  WRITE-BACK PIPELINE (interleaved: 1-cycle LUT latency)
    // =====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wb_ilv_valid_d1 <= 1'b0;
            wb_c0_even_d1 <= 6'sd0; wb_c0_odd_d1 <= 6'sd0;
            wb_c1_even_d1 <= 6'sd0; wb_c1_odd_d1 <= 6'sd0;
        end else begin
            wb_ilv_valid_d1 <= is_interleaved && c0_llr_out_valid
                               && (main_state == ST_RUNNING);
            wb_c0_even_d1 <= c0_l_extr_even;
            wb_c0_odd_d1  <= c0_l_extr_odd;
            wb_c1_even_d1 <= c1_l_extr_even;
            wb_c1_odd_d1  <= c1_l_extr_odd;
        end
    end

    // =====================================================================
    //  MAIN FSM
    // =====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            main_state      <= ST_IDLE;
            fetch_state     <= F_IDLE;
            half_iter_cnt   <= 4'd0;
            start_cores     <= 1'b0;
            llr_valid_to_cores <= 1'b0;
            decode_done     <= 1'b0;
            ld_cnt          <= 11'd0;
            ld_rd_valid     <= 1'b0;
            ld_wr_idx       <= 11'd0;
            perm_fr_even <= 0; perm_fr_odd <= 0;
            perm_br_even <= 0; perm_br_odd <= 0;
            perm_dbr_even<= 0; perm_dbr_odd<= 0;
            extr_rd_row_fr_e <= 0; extr_rd_row_br_e <= 0; extr_rd_row_dbr_e <= 0;
            extr_rd_row_fr_o <= 0; extr_rd_row_br_o <= 0; extr_rd_row_dbr_o <= 0;
            {c0_fr_sys_odd,c0_fr_sys_even,c0_fr_par_odd,c0_fr_par_even} <= 20'd0;
            {c0_fr_apr_odd,c0_fr_apr_even} <= 10'd0;
            {c0_br_sys_odd,c0_br_sys_even,c0_br_par_odd,c0_br_par_even} <= 20'd0;
            {c0_br_apr_odd,c0_br_apr_even} <= 10'd0;
            {c0_dbr_sys_odd,c0_dbr_sys_even,c0_dbr_par_odd,c0_dbr_par_even} <= 20'd0;
            {c0_dbr_apr_odd,c0_dbr_apr_even} <= 10'd0;
            {c1_fr_sys_odd,c1_fr_sys_even,c1_fr_par_odd,c1_fr_par_even} <= 20'd0;
            {c1_fr_apr_odd,c1_fr_apr_even} <= 10'd0;
            {c1_br_sys_odd,c1_br_sys_even,c1_br_par_odd,c1_br_par_even} <= 20'd0;
            {c1_br_apr_odd,c1_br_apr_even} <= 10'd0;
            {c1_dbr_sys_odd,c1_dbr_sys_even,c1_dbr_par_odd,c1_dbr_par_even} <= 20'd0;
            {c1_dbr_apr_odd,c1_dbr_apr_even} <= 10'd0;
        end else begin
            // ---- defaults ----
            start_cores        <= 1'b0;
            llr_valid_to_cores <= 1'b0;
            decode_done        <= 1'b0;
            ld_rd_valid        <= 1'b0;

            case (main_state)
            // =============================================================
            ST_IDLE: begin
                half_iter_cnt <= 4'd0;
                fetch_state   <= F_IDLE;
                if (start) begin
                    start_cores <= 1'b1;
                    main_state  <= ST_RUNNING;
                end
            end

            // =============================================================
            ST_RUNNING: begin
                if (c0_done) begin
                    main_state <= ST_HALF_DONE;
                end else begin
                    case (fetch_state)
                    // ----- wait for llr_req -----
                    F_IDLE: begin
                        if (c0_llr_req)
                            fetch_state <= F_C1;
                    end

                    // ----- BRAM + LUT data ready (1 cycle after addr) -----
                    F_C1: begin
                        // Load 24 sys/par regs — zero when out-of-range
                        // FR (OOR check uses registered address from previous cycle)
                        if (fr_oor) begin
                            {c0_fr_sys_even,c1_fr_sys_even,c0_fr_sys_odd,c1_fr_sys_odd} <= 20'd0;
                            {c0_fr_par_even,c1_fr_par_even,c0_fr_par_odd,c1_fr_par_odd} <= 20'd0;
                        end else begin
                            c0_fr_sys_even<=s_even_fr[4:0]; c1_fr_sys_even<=s_even_fr[9:5];
                            c0_fr_sys_odd <=s_odd_fr[4:0];  c1_fr_sys_odd <=s_odd_fr[9:5];
                            c0_fr_par_even<=p_even_fr[4:0]; c1_fr_par_even<=p_even_fr[9:5];
                            c0_fr_par_odd <=p_odd_fr[4:0];  c1_fr_par_odd <=p_odd_fr[9:5];
                        end
                        // BR
                        if (br_oor) begin
                            {c0_br_sys_even,c1_br_sys_even,c0_br_sys_odd,c1_br_sys_odd} <= 20'd0;
                            {c0_br_par_even,c1_br_par_even,c0_br_par_odd,c1_br_par_odd} <= 20'd0;
                        end else begin
                            c0_br_sys_even<=s_even_br[4:0]; c1_br_sys_even<=s_even_br[9:5];
                            c0_br_sys_odd <=s_odd_br[4:0];  c1_br_sys_odd <=s_odd_br[9:5];
                            c0_br_par_even<=p_even_br[4:0]; c1_br_par_even<=p_even_br[9:5];
                            c0_br_par_odd <=p_odd_br[4:0];  c1_br_par_odd <=p_odd_br[9:5];
                        end
                        // DBR
                        if (dbr_oor) begin
                            {c0_dbr_sys_even,c1_dbr_sys_even,c0_dbr_sys_odd,c1_dbr_sys_odd} <= 20'd0;
                            {c0_dbr_par_even,c1_dbr_par_even,c0_dbr_par_odd,c1_dbr_par_odd} <= 20'd0;
                        end else begin
                            c0_dbr_sys_even<=s_even_dbr[4:0]; c1_dbr_sys_even<=s_even_dbr[9:5];
                            c0_dbr_sys_odd <=s_odd_dbr[4:0];  c1_dbr_sys_odd <=s_odd_dbr[9:5];
                            c0_dbr_par_even<=p_even_dbr[4:0]; c1_dbr_par_even<=p_even_dbr[9:5];
                            c0_dbr_par_odd <=p_odd_dbr[4:0];  c1_dbr_par_odd <=p_odd_dbr[9:5];
                        end

                        if (!is_interleaved) begin
                            // === NATURAL: also load 12 apr regs ===
                            if (half_iter_cnt == 4'd0) begin
                                // half-iter 0: force a-priori to zero
                                c0_fr_apr_even<=5'sd0; c0_fr_apr_odd<=5'sd0;
                                c1_fr_apr_even<=5'sd0; c1_fr_apr_odd<=5'sd0;
                                c0_br_apr_even<=5'sd0; c0_br_apr_odd<=5'sd0;
                                c1_br_apr_even<=5'sd0; c1_br_apr_odd<=5'sd0;
                                c0_dbr_apr_even<=5'sd0; c0_dbr_apr_odd<=5'sd0;
                                c1_dbr_apr_even<=5'sd0; c1_dbr_apr_odd<=5'sd0;
                            end else begin
                                // Extrinsic BRAM data → truncate 6→5 bit (zero if OOR)
                                if (fr_oor) begin
                                    {c0_fr_apr_even,c0_fr_apr_odd,c1_fr_apr_even,c1_fr_apr_odd} <= 20'd0;
                                end else begin
                                    c0_fr_apr_even <=extr_even_fr[4:0];
                                    c1_fr_apr_even <=extr_even_fr[10:6];
                                    c0_fr_apr_odd  <=extr_odd_fr[4:0];
                                    c1_fr_apr_odd  <=extr_odd_fr[10:6];
                                end
                                if (br_oor) begin
                                    {c0_br_apr_even,c0_br_apr_odd,c1_br_apr_even,c1_br_apr_odd} <= 20'd0;
                                end else begin
                                    c0_br_apr_even <=extr_even_br[4:0];
                                    c1_br_apr_even <=extr_even_br[10:6];
                                    c0_br_apr_odd  <=extr_odd_br[4:0];
                                    c1_br_apr_odd  <=extr_odd_br[10:6];
                                end
                                if (dbr_oor) begin
                                    {c0_dbr_apr_even,c0_dbr_apr_odd,c1_dbr_apr_even,c1_dbr_apr_odd} <= 20'd0;
                                end else begin
                                    c0_dbr_apr_even<=extr_even_dbr[4:0];
                                    c1_dbr_apr_even<=extr_even_dbr[10:6];
                                    c0_dbr_apr_odd <=extr_odd_dbr[4:0];
                                    c1_dbr_apr_odd <=extr_odd_dbr[10:6];
                                end
                            end
                            llr_valid_to_cores <= 1'b1;
                            fetch_state <= F_IDLE;
                        end else begin
                            // === INTERLEAVED: register perm bits, issue extrinsic reads ===
                            perm_fr_even  <= mn_fr_e_perm;
                            perm_fr_odd   <= mn_fr_o_perm;
                            perm_br_even  <= mn_br_e_perm;
                            perm_br_odd   <= mn_br_o_perm;
                            perm_dbr_even <= mn_dbr_e_perm;
                            perm_dbr_odd  <= mn_dbr_o_perm;
                            // Save read rows for stall detection
                            extr_rd_row_fr_e  <= mn_fr_e_row;
                            extr_rd_row_br_e  <= mn_br_e_row;
                            extr_rd_row_dbr_e <= mn_dbr_e_row;
                            extr_rd_row_fr_o  <= mn_fr_o_row;
                            extr_rd_row_br_o  <= mn_br_o_row;
                            extr_rd_row_dbr_o <= mn_dbr_o_row;
                            fetch_state <= F_C2;
                        end
                    end

                    // ----- INTERLEAVED: extrinsic data ready -----
                    F_C2: begin
                        // slave_net outputs → load 12 apr regs (truncate 6→5)
                        c0_fr_apr_even <=sl_fr_e_c0[4:0];
                        c1_fr_apr_even <=sl_fr_e_c1[4:0];
                        c0_fr_apr_odd  <=sl_fr_o_c0[4:0];
                        c1_fr_apr_odd  <=sl_fr_o_c1[4:0];
                        c0_br_apr_even <=sl_br_e_c0[4:0];
                        c1_br_apr_even <=sl_br_e_c1[4:0];
                        c0_br_apr_odd  <=sl_br_o_c0[4:0];
                        c1_br_apr_odd  <=sl_br_o_c1[4:0];
                        c0_dbr_apr_even<=sl_dbr_e_c0[4:0];
                        c1_dbr_apr_even<=sl_dbr_e_c1[4:0];
                        c0_dbr_apr_odd <=sl_dbr_o_c0[4:0];
                        c1_dbr_apr_odd <=sl_dbr_o_c1[4:0];

                        if (stall_detect) begin
                            fetch_state <= F_STALL;
                        end else begin
                            llr_valid_to_cores <= 1'b1;
                            fetch_state <= F_IDLE;
                        end
                    end

                    // ----- STALL: wait 1 cycle for write to complete -----
                    F_STALL: begin
                        // Write completed previous cycle; extrinsic re-reads are
                        // re-issuing via the combinatorial address mux.
                        fetch_state <= F_C2B;
                    end

                    // ----- Re-read extrinsic after stall -----
                    F_C2B: begin
                        c0_fr_apr_even <=sl_fr_e_c0[4:0];
                        c1_fr_apr_even <=sl_fr_e_c1[4:0];
                        c0_fr_apr_odd  <=sl_fr_o_c0[4:0];
                        c1_fr_apr_odd  <=sl_fr_o_c1[4:0];
                        c0_br_apr_even <=sl_br_e_c0[4:0];
                        c1_br_apr_even <=sl_br_e_c1[4:0];
                        c0_br_apr_odd  <=sl_br_o_c0[4:0];
                        c1_br_apr_odd  <=sl_br_o_c1[4:0];
                        c0_dbr_apr_even<=sl_dbr_e_c0[4:0];
                        c1_dbr_apr_even<=sl_dbr_e_c1[4:0];
                        c0_dbr_apr_odd <=sl_dbr_o_c0[4:0];
                        c1_dbr_apr_odd <=sl_dbr_o_c1[4:0];
                        llr_valid_to_cores <= 1'b1;
                        fetch_state <= F_IDLE;
                    end

                    default: fetch_state <= F_IDLE;
                    endcase
                end
            end

            // =============================================================
            ST_HALF_DONE: begin
                half_iter_cnt <= half_iter_cnt + 4'd1;
                if (half_iter_cnt == 4'd10) begin
                    main_state <= ST_WRITE_LD;
                    ld_cnt     <= 11'd0;
                end else begin
                    start_cores <= 1'b1;
                    main_state  <= ST_RUNNING;
                    fetch_state <= F_IDLE;
                end
            end

            // =============================================================
            ST_WRITE_LD: begin
                if (ld_cnt < 11'd1536) begin
                    ld_rd_valid <= 1'b1;
                    ld_wr_idx   <= ld_cnt;
                    ld_cnt      <= ld_cnt + 11'd1;
                end else if (ld_rd_valid) begin
                    // flush: last write in progress
                end else begin
                    decode_done <= 1'b1;
                    main_state  <= ST_IDLE;
                end
            end

            default: main_state <= ST_IDLE;
            endcase
        end
    end

endmodule
