`timescale 1ns/1ps

module tb_dbr_standalone;

    // Clock & Reset
    reg clk;
    reg rst_n;
    
    // Inputs
    reg active;
    reg init_sm;
    reg [3:0] win_len_r4;
    
    reg signed [4:0] sys_odd, sys_even;
    reg signed [4:0] par_odd, par_even;
    reg signed [4:0] apr_odd, apr_even;
    
    // Outputs
    wire signed [9:0] final_beta_0, final_beta_1;
    wire signed [9:0] final_beta_2, final_beta_3;
    wire signed [9:0] final_beta_4, final_beta_5;
    wire signed [9:0] final_beta_6, final_beta_7;
    wire window_done;

    // Instance
    dummy_backward_recursion_unit dut (
        .clk(clk),
        .rst_n(rst_n),
        .active(active),
        .init_sm(init_sm),
        .win_len_r4(win_len_r4),
        .sys_odd(sys_odd), .sys_even(sys_even),
        .par_odd(par_odd), .par_even(par_even),
        .apr_odd(apr_odd), .apr_even(apr_even),
        .final_beta_0(final_beta_0), .final_beta_1(final_beta_1),
        .final_beta_2(final_beta_2), .final_beta_3(final_beta_3),
        .final_beta_4(final_beta_4), .final_beta_5(final_beta_5),
        .final_beta_6(final_beta_6), .final_beta_7(final_beta_7),
        .window_done(window_done)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // Test sequence
    initial begin
        // VCD Dump
        $dumpfile("dbr_standalone.vcd");
        $dumpvars(0, tb_dbr_standalone);
        
        // Initialize
        rst_n = 0;
        active = 0;
        init_sm = 0;
        win_len_r4 = 4'd15;
        
        sys_odd = 5'sd1; sys_even = 5'sd2;
        par_odd = 5'sd3; par_even = 5'sd4;
        apr_odd = 0;     apr_even = 0;

        #20;
        rst_n = 1;
        #10;
        
        // Emulate bcjr_core ST_IDLE
        init_sm = 1;
        #10;
        init_sm = 0;
        
        // Emulate bcjr_core running 15 steps
        // Each step takes 3 cycles: LLR_REQ, LLR_WAIT, COMPUTE (active block)
        repeat (15) begin
            // LLR_REQ
            active = 0;
            #10;
            // LLR_WAIT
            active = 0;
            #10;
            // COMPUTE
            active = 1;
            
            // Randomize inputs slightly to see SM accumulation
            sys_odd = sys_odd + 1;
            sys_even = sys_even - 1;
            
            #10;
        end
        
        // After 15th compute, one more cycle should trigger window_done delayed capture
        active = 0;
        #10; 
        
        // Check window_done
        if (window_done) begin
            $display("SUCCESS: window_done triggered!");
            $display("Final Betas: %d, %d, %d, %d, %d, %d, %d, %d", 
                final_beta_0, final_beta_1, final_beta_2, final_beta_3,
                final_beta_4, final_beta_5, final_beta_6, final_beta_7);
        end else begin
            $display("ERROR: window_done did NOT trigger.");
        end

        #50;
        $finish;
    end

    // Monitor
    always @(posedge clk) begin
        if (active && dut.running) begin
            $display("Time=%0t | Step=%d | Active=%b | sm_0=%d", $time, dut.step_cnt, active, dut.sm[0]);
        end
    end

endmodule
