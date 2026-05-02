//==============================================================================
// Module: turbo_decoder_button_bringup
// Description:
//   PL-only Zybo bring-up top for the LTE turbo decoder.
//
//   The current fixed input vectors are embedded into bitstream ROMs via
//   $readmemh. BTN0 runs a small hardware loader that copies those ROM rows
//   into the decoder's existing input BRAM load interface. BTN1 starts decode.
//   BTN2 resets the design. BTN3 replays the output sweep for ILA capture.
//
//   ILA usage:
//     On Z7-10, keep debug small. The minimal ILA captures one 5-bit probe:
//     {dbg_sweep_valid, dbg_hard_bits}. The sweep emits 800 valid rows on
//     consecutive clocks, so a 1024-sample ILA is enough and the valid sample
//     index is the output row number.
//==============================================================================
`ifdef USE_MARK_DEBUG
`define DBG_ATTR (* keep = "true", mark_debug = "true" *)
`else
`define DBG_ATTR
`endif

`ifndef TURBO_DATA_DIR
`define TURBO_DATA_DIR "data/"
`endif

module turbo_decoder_button_bringup #(
    parameter K               = 3200,
    parameter NUM_SISO        = 2,
    parameter SEG_LEN         = K / NUM_SISO,
    parameter TAIL_LEN        = 3,
    parameter WIN_LEN         = 30,
    parameter PAPER_BOUNDARY  = 1,
    parameter NUM_WINDOWS     = (SEG_LEN + TAIL_LEN + WIN_LEN - 1) / WIN_LEN,
    parameter ROW_DEPTH       = (SEG_LEN + 1) / 2,
    parameter INPUT_ROW_DEPTH = (SEG_LEN + TAIL_LEN + 1) / 2,
    parameter ROW_ADDR_W      = 10,
    parameter FRAME_ADDR_W    = 12,
    parameter PI_W            = 12,
    parameter INPUT_WORD_W    = 10,
    parameter EXTR_WORD_W     = 12,
    parameter DATA_DIR        = `TURBO_DATA_DIR,
    parameter QPP_LUT_FILE    = {DATA_DIR, "qpp_3200.hex"},
    parameter NUM_HALF_ITER   = 4'd11,
    parameter BUTTON_DEBOUNCE_CNT_W = 20
) (
    input  wire       clk,
    input  wire [3:0] btn,
    output wire [3:0] led
);

    // -------------------------------------------------------------------------
    // Reset and button conditioning
    // -------------------------------------------------------------------------
    reg [7:0] por_cnt = 8'd0;
    reg btn_reset_meta;
    reg btn_reset_sync;

    always @(posedge clk) begin
        if (por_cnt != 8'hff)
            por_cnt <= por_cnt + 1'b1;

        btn_reset_meta <= btn[2];
        btn_reset_sync <= btn_reset_meta;
    end

    wire por_done = &por_cnt;
    wire rst_n = por_done && !btn_reset_sync;

    wire btn_load_pulse;
    wire btn_start_pulse;
    wire btn_sweep_pulse;

    button_conditioner #(
        .CNT_W(BUTTON_DEBOUNCE_CNT_W)
    ) u_btn_load (
        .clk(clk),
        .rst_n(rst_n),
        .raw(btn[0]),
        .level(),
        .rise(btn_load_pulse)
    );

    button_conditioner #(
        .CNT_W(BUTTON_DEBOUNCE_CNT_W)
    ) u_btn_start (
        .clk(clk),
        .rst_n(rst_n),
        .raw(btn[1]),
        .level(),
        .rise(btn_start_pulse)
    );

    button_conditioner #(
        .CNT_W(BUTTON_DEBOUNCE_CNT_W)
    ) u_btn_sweep (
        .clk(clk),
        .rst_n(rst_n),
        .raw(btn[3]),
        .level(),
        .rise(btn_sweep_pulse)
    );

    // -------------------------------------------------------------------------
    // Hardware vector loader
    // -------------------------------------------------------------------------
    localparam LOAD_IDLE  = 2'd0;
    localparam LOAD_WAIT  = 2'd1;
    localparam LOAD_WRITE = 2'd2;

    reg [1:0] load_state;
    reg [2:0] load_sel;
    reg [ROW_ADDR_W-1:0] load_row;
    reg loaded;

    wire load_busy = (load_state != LOAD_IDLE);
    wire decoder_load_en = (load_state == LOAD_WRITE);

    // -------------------------------------------------------------------------
    // Bitstream-embedded vector ROMs
    // -------------------------------------------------------------------------
    wire [ROW_ADDR_W-1:0] rom_addr = load_row;
    wire [INPUT_WORD_W-1:0] rom_sys_odd;
    wire [INPUT_WORD_W-1:0] rom_sys_even;
    wire [INPUT_WORD_W-1:0] rom_par1_odd;
    wire [INPUT_WORD_W-1:0] rom_par1_even;
    wire [INPUT_WORD_W-1:0] rom_silv_odd;
    wire [INPUT_WORD_W-1:0] rom_silv_even;
    wire [INPUT_WORD_W-1:0] rom_par2_odd;
    wire [INPUT_WORD_W-1:0] rom_par2_even;

    turbo_vector_rom #(
        .WORD_W(INPUT_WORD_W),
        .DEPTH(INPUT_ROW_DEPTH),
        .ADDR_W(ROW_ADDR_W),
        .INIT_FILE({DATA_DIR, "sys_odd_ram.hex"})
    ) u_rom_sys_odd (
        .clk(clk),
        .addr(rom_addr),
        .data(rom_sys_odd)
    );

    turbo_vector_rom #(
        .WORD_W(INPUT_WORD_W),
        .DEPTH(INPUT_ROW_DEPTH),
        .ADDR_W(ROW_ADDR_W),
        .INIT_FILE({DATA_DIR, "sys_even_ram.hex"})
    ) u_rom_sys_even (
        .clk(clk),
        .addr(rom_addr),
        .data(rom_sys_even)
    );

    turbo_vector_rom #(
        .WORD_W(INPUT_WORD_W),
        .DEPTH(INPUT_ROW_DEPTH),
        .ADDR_W(ROW_ADDR_W),
        .INIT_FILE({DATA_DIR, "par1_odd_ram.hex"})
    ) u_rom_par1_odd (
        .clk(clk),
        .addr(rom_addr),
        .data(rom_par1_odd)
    );

    turbo_vector_rom #(
        .WORD_W(INPUT_WORD_W),
        .DEPTH(INPUT_ROW_DEPTH),
        .ADDR_W(ROW_ADDR_W),
        .INIT_FILE({DATA_DIR, "par1_even_ram.hex"})
    ) u_rom_par1_even (
        .clk(clk),
        .addr(rom_addr),
        .data(rom_par1_even)
    );

    turbo_vector_rom #(
        .WORD_W(INPUT_WORD_W),
        .DEPTH(INPUT_ROW_DEPTH),
        .ADDR_W(ROW_ADDR_W),
        .INIT_FILE({DATA_DIR, "sys_ilv_odd_ram.hex"})
    ) u_rom_silv_odd (
        .clk(clk),
        .addr(rom_addr),
        .data(rom_silv_odd)
    );

    turbo_vector_rom #(
        .WORD_W(INPUT_WORD_W),
        .DEPTH(INPUT_ROW_DEPTH),
        .ADDR_W(ROW_ADDR_W),
        .INIT_FILE({DATA_DIR, "sys_ilv_even_ram.hex"})
    ) u_rom_silv_even (
        .clk(clk),
        .addr(rom_addr),
        .data(rom_silv_even)
    );

    turbo_vector_rom #(
        .WORD_W(INPUT_WORD_W),
        .DEPTH(INPUT_ROW_DEPTH),
        .ADDR_W(ROW_ADDR_W),
        .INIT_FILE({DATA_DIR, "par2_odd_ram.hex"})
    ) u_rom_par2_odd (
        .clk(clk),
        .addr(rom_addr),
        .data(rom_par2_odd)
    );

    turbo_vector_rom #(
        .WORD_W(INPUT_WORD_W),
        .DEPTH(INPUT_ROW_DEPTH),
        .ADDR_W(ROW_ADDR_W),
        .INIT_FILE({DATA_DIR, "par2_even_ram.hex"})
    ) u_rom_par2_even (
        .clk(clk),
        .addr(rom_addr),
        .data(rom_par2_even)
    );

    reg [INPUT_WORD_W-1:0] load_data_mux;
    always @* begin
        case (load_sel)
            3'd0: load_data_mux = rom_sys_odd;
            3'd1: load_data_mux = rom_sys_even;
            3'd2: load_data_mux = rom_par1_odd;
            3'd3: load_data_mux = rom_par1_even;
            3'd4: load_data_mux = rom_silv_odd;
            3'd5: load_data_mux = rom_silv_even;
            3'd6: load_data_mux = rom_par2_odd;
            3'd7: load_data_mux = rom_par2_even;
            default: load_data_mux = {INPUT_WORD_W{1'b0}};
        endcase
    end

    // -------------------------------------------------------------------------
    // Decoder control and result sweep
    // -------------------------------------------------------------------------
    localparam SWEEP_IDLE  = 2'd0;
    localparam SWEEP_RUN   = 2'd1;
    localparam SWEEP_DRAIN = 2'd2;

    reg decoder_start;
    reg decode_busy;
    reg done_latched;
    reg [3:0] done_count;

    reg [1:0] sweep_state;
    reg [ROW_ADDR_W-1:0] sweep_req_row;
    reg [ROW_ADDR_W-1:0] sweep_row_pipe;
    reg [ROW_ADDR_W-1:0] sweep_row;
    reg sweep_data_pending;
    reg sweep_valid_q;
    reg [3:0] sweep_hard_bits;
    reg sweep_done;

    wire decode_done;
    wire [EXTR_WORD_W-1:0] ld_rd_data;
    wire [EXTR_WORD_W-1:0] ld_rd_data_odd;
    wire [3:0] hard_rd_data;

    wire sweep_active = (sweep_state != SWEEP_IDLE);
    wire sweep_valid = sweep_valid_q;
    wire start_sweep = decode_done || (btn_sweep_pulse && done_latched && !sweep_active);

    turbo_decoder #(
        .K(K),
        .NUM_SISO(NUM_SISO),
        .SEG_LEN(SEG_LEN),
        .TAIL_LEN(TAIL_LEN),
        .WIN_LEN(WIN_LEN),
        .PAPER_BOUNDARY(PAPER_BOUNDARY),
        .NUM_WINDOWS(NUM_WINDOWS),
        .ROW_DEPTH(ROW_DEPTH),
        .INPUT_ROW_DEPTH(INPUT_ROW_DEPTH),
        .ROW_ADDR_W(ROW_ADDR_W),
        .FRAME_ADDR_W(FRAME_ADDR_W),
        .PI_W(PI_W),
        .INPUT_WORD_W(INPUT_WORD_W),
        .EXTR_WORD_W(EXTR_WORD_W),
        .QPP_LUT_FILE(QPP_LUT_FILE),
        .NUM_HALF_ITER(NUM_HALF_ITER)
    ) u_decoder (
        .clk(clk),
        .rst_n(rst_n),
        .start(decoder_start),
        .load_en(decoder_load_en),
        .load_bram_sel(load_sel),
        .load_addr(load_row),
        .load_data(load_data_mux),
        .decode_done(decode_done),
        .ld_rd_addr(sweep_req_row),
        .ld_rd_data(ld_rd_data),
        .ld_rd_data_odd(ld_rd_data_odd),
        .hard_rd_addr(sweep_req_row),
        .hard_rd_data(hard_rd_data)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            load_state   <= LOAD_IDLE;
            load_sel     <= 3'd0;
            load_row     <= {ROW_ADDR_W{1'b0}};
            loaded       <= 1'b0;
            decoder_start <= 1'b0;
            decode_busy  <= 1'b0;
            done_latched <= 1'b0;
            done_count   <= 4'd0;
            sweep_state  <= SWEEP_IDLE;
            sweep_req_row <= {ROW_ADDR_W{1'b0}};
            sweep_row_pipe <= {ROW_ADDR_W{1'b0}};
            sweep_row    <= {ROW_ADDR_W{1'b0}};
            sweep_data_pending <= 1'b0;
            sweep_valid_q <= 1'b0;
            sweep_hard_bits <= 4'd0;
            sweep_done   <= 1'b0;
        end else begin
            decoder_start <= 1'b0;
            sweep_valid_q <= 1'b0;

            if (sweep_data_pending) begin
                sweep_valid_q   <= 1'b1;
                sweep_row       <= sweep_row_pipe;
                sweep_hard_bits <= hard_rd_data;
            end

            case (load_state)
                LOAD_IDLE: begin
                    if (btn_load_pulse && !decode_busy && !sweep_active) begin
                        load_sel     <= 3'd0;
                        load_row     <= {ROW_ADDR_W{1'b0}};
                        load_state   <= LOAD_WAIT;
                        loaded       <= 1'b0;
                        done_latched <= 1'b0;
                        sweep_done   <= 1'b0;
                    end
                end

                LOAD_WAIT: begin
                    load_state <= LOAD_WRITE;
                end

                LOAD_WRITE: begin
                    if ((load_sel == 3'd7) && (load_row == INPUT_ROW_DEPTH - 1'b1)) begin
                        load_state <= LOAD_IDLE;
                        loaded     <= 1'b1;
                    end else begin
                        if (load_row == INPUT_ROW_DEPTH - 1'b1) begin
                            load_row <= {ROW_ADDR_W{1'b0}};
                            load_sel <= load_sel + 1'b1;
                        end else begin
                            load_row <= load_row + 1'b1;
                        end
                        load_state <= LOAD_WAIT;
                    end
                end

                default: begin
                    load_state <= LOAD_IDLE;
                end
            endcase

            if (btn_start_pulse && loaded && !load_busy && !decode_busy && !sweep_active) begin
                decoder_start <= 1'b1;
                decode_busy   <= 1'b1;
                done_latched  <= 1'b0;
                sweep_done    <= 1'b0;
            end

            if (decode_done) begin
                decode_busy  <= 1'b0;
                done_latched <= 1'b1;
                done_count   <= done_count + 1'b1;
            end

            case (sweep_state)
                SWEEP_IDLE: begin
                    if (start_sweep) begin
                        sweep_req_row      <= {ROW_ADDR_W{1'b0}};
                        sweep_row_pipe     <= {ROW_ADDR_W{1'b0}};
                        sweep_row          <= {ROW_ADDR_W{1'b0}};
                        sweep_data_pending <= 1'b0;
                        sweep_state        <= SWEEP_RUN;
                        sweep_done         <= 1'b0;
                    end
                end

                SWEEP_RUN: begin
                    sweep_row_pipe     <= sweep_req_row;
                    sweep_data_pending <= 1'b1;

                    if (sweep_req_row == ROW_DEPTH - 1'b1) begin
                        sweep_state <= SWEEP_DRAIN;
                    end else begin
                        sweep_req_row <= sweep_req_row + 1'b1;
                    end
                end

                SWEEP_DRAIN: begin
                    sweep_data_pending <= 1'b0;
                    sweep_state        <= SWEEP_IDLE;
                    sweep_done         <= 1'b1;
                end

                default: begin
                    sweep_state <= SWEEP_IDLE;
                end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Board LEDs
    // -------------------------------------------------------------------------
    assign led[0] = loaded;
    assign led[1] = load_busy || decode_busy;
    assign led[2] = done_latched;
    assign led[3] = sweep_active;

    // -------------------------------------------------------------------------
    // ILA-friendly debug probes. Set up an ILA on these marked nets.
    // -------------------------------------------------------------------------
    `DBG_ATTR wire        dbg_btn_load_pulse  = btn_load_pulse;
    `DBG_ATTR wire        dbg_btn_start_pulse = btn_start_pulse;
    `DBG_ATTR wire        dbg_btn_sweep_pulse = btn_sweep_pulse;
    `DBG_ATTR wire [1:0]  dbg_load_state      = load_state;
    `DBG_ATTR wire [2:0]  dbg_load_sel        = load_sel;
    `DBG_ATTR wire [9:0]  dbg_load_row        = load_row;
    `DBG_ATTR wire        dbg_loaded          = loaded;
    `DBG_ATTR wire        dbg_decoder_start   = decoder_start;
    `DBG_ATTR wire        dbg_decode_busy     = decode_busy;
    `DBG_ATTR wire        dbg_decode_done     = decode_done;
    `DBG_ATTR wire        dbg_done_latched    = done_latched;
    `DBG_ATTR wire [3:0]  dbg_done_count      = done_count;
    `DBG_ATTR wire [1:0]  dbg_sweep_state     = sweep_state;
    `DBG_ATTR wire        dbg_sweep_valid     = sweep_valid;
    `DBG_ATTR wire        dbg_sweep_done      = sweep_done;
    `DBG_ATTR wire [9:0]  dbg_sweep_row       = sweep_row;
    `DBG_ATTR wire [3:0]  dbg_hard_bits       = sweep_hard_bits;
    `DBG_ATTR wire [11:0] dbg_ld_even         = ld_rd_data;
    `DBG_ATTR wire [11:0] dbg_ld_odd          = ld_rd_data_odd;
    `DBG_ATTR wire [4:0]  dbg_packed_output   = {dbg_sweep_valid, dbg_hard_bits};

`ifdef USE_MIN_ILA_IP
    ila_0 u_ila_0 (
        .clk(clk),
        .probe0(dbg_packed_output)
    );
`elsif USE_ILA_IP
    ila_0 u_ila_0 (
        .clk(clk),
        .probe0(dbg_btn_load_pulse),
        .probe1(dbg_btn_start_pulse),
        .probe2(dbg_btn_sweep_pulse),
        .probe3(dbg_load_state),
        .probe4(dbg_load_sel),
        .probe5(dbg_load_row),
        .probe6(dbg_loaded),
        .probe7(dbg_decoder_start),
        .probe8(dbg_decode_busy),
        .probe9(dbg_decode_done),
        .probe10(dbg_done_latched),
        .probe11(dbg_done_count),
        .probe12(dbg_sweep_state),
        .probe13(dbg_sweep_valid),
        .probe14(dbg_sweep_done),
        .probe15(dbg_sweep_row),
        .probe16(dbg_hard_bits),
        .probe17(dbg_ld_even),
        .probe18(dbg_ld_odd)
    );
`endif

endmodule

//------------------------------------------------------------------------------
// Synchronous ROM for one embedded input vector memory.
//------------------------------------------------------------------------------
module turbo_vector_rom #(
    parameter WORD_W = 10,
    parameter DEPTH = 802,
    parameter ADDR_W = 10,
    parameter INIT_FILE = "data/sys_odd_ram.hex"
) (
    input  wire              clk,
    input  wire [ADDR_W-1:0] addr,
    output reg  [WORD_W-1:0] data
);
    (* rom_style = "block" *) reg [WORD_W-1:0] mem [0:DEPTH-1];

    initial begin
        $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) begin
        data <= mem[addr];
    end
endmodule

//------------------------------------------------------------------------------
// Button synchronizer/debouncer with one-clock rising-edge pulse.
//------------------------------------------------------------------------------
module button_conditioner #(
    parameter CNT_W = 20
) (
    input  wire clk,
    input  wire rst_n,
    input  wire raw,
    output reg  level,
    output wire rise
);
    reg sync0;
    reg sync1;
    reg level_d;
    reg [CNT_W-1:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync0   <= 1'b0;
            sync1   <= 1'b0;
            level   <= 1'b0;
            level_d <= 1'b0;
            cnt     <= {CNT_W{1'b0}};
        end else begin
            sync0   <= raw;
            sync1   <= sync0;
            level_d <= level;

            if (sync1 == level) begin
                cnt <= {CNT_W{1'b0}};
            end else begin
                cnt <= cnt + 1'b1;
                if (&cnt) begin
                    level <= sync1;
                    cnt   <= {CNT_W{1'b0}};
                end
            end
        end
    end

    assign rise = level && !level_d;
endmodule
