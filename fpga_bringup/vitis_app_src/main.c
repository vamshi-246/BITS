#include "turbo_decoder_axi.h"
#include "turbo_test_vectors.h"

#define BUSY_TIMEOUT_POLLS      1000000U
#define DECODE_TIMEOUT_POLLS    200000000U

static uint8_t decoded_bits[TD_K];

static void print_status(const turbo_decoder_t *dev, const char *label)
{
    u32 status = td_read_status(dev);
    xil_printf("%s STATUS=0x%x done=%d busy=%d pulse=%d\r\n",
               label,
               (unsigned int)status,
               (int)((status & TD_STATUS_DONE) != 0U),
               (int)((status & TD_STATUS_BUSY) != 0U),
               (int)((status & TD_STATUS_DONE_PULSE) != 0U));
}

static int report_step_result(const char *name, int rc)
{
    if (rc == TD_OK) {
        xil_printf("[PASS] %s\r\n", name);
    } else {
        xil_printf("[FAIL] %s rc=%d\r\n", name, rc);
    }
    return rc;
}

int main(void)
{
    turbo_decoder_t dev;
    u32 status;
    u32 sel;
    u32 truth_errors;
    u32 rtl_mismatches;
    int first;
    int rc;

    td_init(&dev, (UINTPTR)TURBO_DECODER_BASEADDR);

    xil_printf("\r\n=== Turbo Decoder FPGA Bring-up ===\r\n");
    xil_printf("Base address: 0x%x\r\n", (unsigned int)dev.baseaddr);
    xil_printf("Vector set: K=%d, input rows=%d, hard rows=%d\r\n",
               (int)TD_K, (int)TD_INPUT_ROW_DEPTH, (int)TD_ROW_DEPTH);
    xil_printf("Expected RTL mismatches: 0, expected BER errors vs truth: 1/3200\r\n\r\n");

    xil_printf("1. AXI read/write smoke test\r\n");
    print_status(&dev, "initial");
    rc = td_axi_smoke_test(&dev);
    if (report_step_result("AXI register smoke", rc) != TD_OK) {
        goto done;
    }
    print_status(&dev, "after smoke");

    xil_printf("\r\n2. Load one BRAM row and check no hang\r\n");
    rc = td_load_one_row(&dev, TD_LOAD_SEL_SYS_ODD, 0U, turbo_input_bram[TD_LOAD_SEL_SYS_ODD][0]);
    if (report_step_result("single sys_odd row load", rc) != TD_OK) {
        goto done;
    }
    print_status(&dev, "after one-row load");

    xil_printf("\r\n3. Load all 8 input BRAMs\r\n");
    xil_printf("Load order:\r\n");
    for (sel = 0U; sel < TD_NUM_INPUT_BRAMS; sel++) {
        xil_printf("  %d: %s\r\n", (int)sel, turbo_input_bram_names[sel]);
    }
    rc = td_load_all_input_brams(&dev, turbo_input_bram);
    if (report_step_result("all input BRAM load", rc) != TD_OK) {
        goto done;
    }
    xil_printf("Loaded %d memories x %d rows\r\n", (int)TD_NUM_INPUT_BRAMS, (int)TD_INPUT_ROW_DEPTH);
    print_status(&dev, "after full load");

    xil_printf("\r\n4. Start decoder and verify busy\r\n");
    td_start_decode(&dev);
    rc = td_wait_busy_asserted(&dev, BUSY_TIMEOUT_POLLS, &status);
    if (report_step_result("busy observed", rc) != TD_OK) {
        xil_printf("last STATUS=0x%x\r\n", (unsigned int)status);
        goto done;
    }
    xil_printf("start STATUS=0x%x\r\n", (unsigned int)status);

    xil_printf("\r\n5. Wait for done\r\n");
    rc = td_wait_done(&dev, DECODE_TIMEOUT_POLLS, &status);
    if (report_step_result("decode done", rc) != TD_OK) {
        xil_printf("last STATUS=0x%x\r\n", (unsigned int)status);
        goto done;
    }
    xil_printf("done STATUS=0x%x\r\n", (unsigned int)status);

    xil_printf("\r\n6. Read first 10 hard rows\r\n");
    td_print_hard_rows(&dev, 10U);

    xil_printf("\r\n7. Read all 3200 decoded bits\r\n");
    rc = td_read_all_hard_bits(&dev, decoded_bits, TD_K);
    if (report_step_result("read all hard bits", rc) != TD_OK) {
        goto done;
    }

    xil_printf("\r\n8. Compare with true_info_bits and RTL expected hard bits\r\n");
    truth_errors = td_count_bit_errors(decoded_bits, turbo_true_info_bits, TD_K);
    rtl_mismatches = td_count_bit_errors(decoded_bits, turbo_rtl_expected_hard_bits, TD_K);

    xil_printf("Errors vs true_info_bits: %d/%d\r\n", (int)truth_errors, (int)TD_K);
    xil_printf("Mismatches vs rtl_final_hard_bits: %d/%d\r\n", (int)rtl_mismatches, (int)TD_K);

    first = td_first_mismatch(decoded_bits, turbo_rtl_expected_hard_bits, TD_K);
    if (first >= 0) {
        xil_printf("First RTL mismatch index %d: got %d expected %d\r\n",
                   first,
                   (int)decoded_bits[first],
                   (int)turbo_rtl_expected_hard_bits[first]);
    }

    if (rtl_mismatches == 0U) {
        xil_printf("[PASS] FPGA output matches RTL simulation vector ordering\r\n");
    } else {
        xil_printf("[FAIL] FPGA output differs from RTL simulation vector ordering\r\n");
    }

done:
    xil_printf("\r\nBring-up program finished. Press CPU reset to rerun.\r\n");
    while (1) {
    }

    return 0;
}
