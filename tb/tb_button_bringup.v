//==============================================================================
// Testbench: tb_button_bringup
// Description:
//   Simulates the PL-only button bring-up top. Uses a short debounce setting,
//   presses BTN0/BTN1, captures the automatic output sweep, and compares the
//   captured hard bits against true_info_bits and rtl_final_hard_bits.
//==============================================================================
`timescale 1ns / 1ps

module tb_button_bringup;
    parameter TEST_K = 3200;
    parameter TEST_SEG_LEN = 1600;
    parameter TEST_ROW_DEPTH = 800;

    reg clk;
    reg [3:0] btn;
    wire [3:0] led;

    reg true_info_bits [0:TEST_K-1];
    reg rtl_expected_hard_bits [0:TEST_K-1];
    reg decoded_bits [0:TEST_K-1];

    integer i;
    integer row;
    integer captured_rows;
    integer truth_errors;
    integer rtl_mismatches;
    integer first_rtl_mismatch;
    integer fd_bits;
    reg [3:0] hard;

    turbo_decoder_button_bringup #(
        .BUTTON_DEBOUNCE_CNT_W(2)
    ) u_dut (
        .clk(clk),
        .btn(btn),
        .led(led)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task press_button;
        input integer idx;
        begin
            btn[idx] = 1'b1;
            repeat (8) @(posedge clk);
            btn[idx] = 1'b0;
            repeat (8) @(posedge clk);
        end
    endtask

    initial begin
        $readmemb("data/true_info_bits.txt", true_info_bits);
        $readmemb("data/rtl_final_hard_bits.txt", rtl_expected_hard_bits);

        for (i = 0; i < TEST_K; i = i + 1)
            decoded_bits[i] = 1'b0;

        btn = 4'b0000;
        captured_rows = 0;
        truth_errors = 0;
        rtl_mismatches = 0;
        first_rtl_mismatch = -1;

        repeat (300) @(posedge clk);

        $display("[%0t] Pressing BTN0 load", $time);
        press_button(0);
        wait (led[0] == 1'b1);
        $display("[%0t] Load complete", $time);

        $display("[%0t] Pressing BTN1 start", $time);
        press_button(1);
        wait (led[2] == 1'b1);
        $display("[%0t] Decode done latched", $time);

        wait (u_dut.dbg_sweep_done == 1'b1);
        repeat (4) @(posedge clk);
        $display("[%0t] Sweep done, captured_rows=%0d", $time, captured_rows);

        fd_bits = $fopen("build/button_bringup_hard_bits.txt", "w");
        for (i = 0; i < TEST_K; i = i + 1) begin
            $fwrite(fd_bits, "%0d\n", decoded_bits[i]);
            if (decoded_bits[i] != true_info_bits[i])
                truth_errors = truth_errors + 1;
            if (decoded_bits[i] != rtl_expected_hard_bits[i]) begin
                if (first_rtl_mismatch < 0)
                    first_rtl_mismatch = i;
                rtl_mismatches = rtl_mismatches + 1;
            end
        end
        $fclose(fd_bits);

        $display("Button bring-up captured rows: %0d/%0d", captured_rows, TEST_ROW_DEPTH);
        $display("Errors vs true_info_bits: %0d/%0d", truth_errors, TEST_K);
        $display("Mismatches vs rtl_final_hard_bits: %0d/%0d", rtl_mismatches, TEST_K);
        if (first_rtl_mismatch >= 0)
            $display("First RTL mismatch index: %0d", first_rtl_mismatch);

        if ((captured_rows == TEST_ROW_DEPTH) && (truth_errors == 1) && (rtl_mismatches == 0))
            $display("PASS: button bring-up output matches expected RTL vector ordering.");
        else
            $display("FAIL: button bring-up output mismatch.");

        $finish;
    end

    always @(posedge clk) begin
        if (u_dut.dbg_sweep_valid) begin
            row = u_dut.dbg_sweep_row;
            hard = u_dut.dbg_hard_bits;

            decoded_bits[2 * row] = hard[0];
            decoded_bits[TEST_SEG_LEN + (2 * row)] = hard[1];
            decoded_bits[(2 * row) + 1] = hard[2];
            decoded_bits[TEST_SEG_LEN + (2 * row) + 1] = hard[3];
            captured_rows = captured_rows + 1;
        end
    end

    initial begin
        #(700_000_000);
        $display("ERROR: tb_button_bringup timed out.");
        $finish;
    end
endmodule
