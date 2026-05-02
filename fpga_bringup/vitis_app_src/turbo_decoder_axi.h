#ifndef TURBO_DECODER_AXI_H
#define TURBO_DECODER_AXI_H

#include <stdint.h>

#include "xil_io.h"
#include "xil_printf.h"
#include "xil_types.h"
#include "xparameters.h"

/* Match rtl/turbo_decoder_axi_lite.v parameters for the current K=3200 build. */
#define TD_K                    3200U
#define TD_NUM_SISO             2U
#define TD_SEG_LEN              1600U
#define TD_TAIL_LEN             3U
#define TD_ROW_DEPTH            800U
#define TD_INPUT_ROW_DEPTH      802U
#define TD_NUM_INPUT_BRAMS      8U

#define TD_REG_CONTROL          0x00U
#define TD_REG_STATUS           0x04U
#define TD_REG_LOAD_ADDR        0x08U
#define TD_REG_LOAD_DATA        0x0CU
#define TD_REG_LOAD_SEL         0x10U
#define TD_REG_LOAD_COMMIT      0x14U
#define TD_REG_LD_ADDR          0x18U
#define TD_REG_LD_EVEN          0x1CU
#define TD_REG_LD_ODD           0x20U
#define TD_REG_HARD_ADDR        0x24U
#define TD_REG_HARD_BITS        0x28U

#define TD_CONTROL_START        0x00000001U
#define TD_CONTROL_CLEAR_DONE   0x00000002U

#define TD_STATUS_DONE          0x00000001U
#define TD_STATUS_BUSY          0x00000002U
#define TD_STATUS_DONE_PULSE    0x00000004U

#define TD_LOAD_SEL_SYS_ODD     0U
#define TD_LOAD_SEL_SYS_EVEN    1U
#define TD_LOAD_SEL_PAR1_ODD    2U
#define TD_LOAD_SEL_PAR1_EVEN   3U
#define TD_LOAD_SEL_SYS_ILV_ODD 4U
#define TD_LOAD_SEL_SYS_ILV_EVEN 5U
#define TD_LOAD_SEL_PAR2_ODD    6U
#define TD_LOAD_SEL_PAR2_EVEN   7U

#define TD_OK                   0
#define TD_ERR_BAD_ARG          -1
#define TD_ERR_AXI_READBACK     -2
#define TD_ERR_TIMEOUT          -3

/*
 * Vitis names this macro from the block-design instance name. Keep the common
 * cases here, then fall back to the address Vivado assigned in your logs.
 */
#ifndef TURBO_DECODER_BASEADDR
#if defined(XPAR_TURBO_DECODER_AXI_LI_0_BASEADDR)
#define TURBO_DECODER_BASEADDR XPAR_TURBO_DECODER_AXI_LI_0_BASEADDR
#elif defined(XPAR_TURBO_DECODER_AXI_LI_0_S_AXI_BASEADDR)
#define TURBO_DECODER_BASEADDR XPAR_TURBO_DECODER_AXI_LI_0_S_AXI_BASEADDR
#elif defined(XPAR_TURBO_DECODER_AXI_LITE_0_BASEADDR)
#define TURBO_DECODER_BASEADDR XPAR_TURBO_DECODER_AXI_LITE_0_BASEADDR
#elif defined(XPAR_TURBO_DECODER_AXI_LITE_0_S_AXI_BASEADDR)
#define TURBO_DECODER_BASEADDR XPAR_TURBO_DECODER_AXI_LITE_0_S_AXI_BASEADDR
#elif defined(XPAR_SYSTEM_TURBO_DECODER_AXI_LI_0_0_BASEADDR)
#define TURBO_DECODER_BASEADDR XPAR_SYSTEM_TURBO_DECODER_AXI_LI_0_0_BASEADDR
#elif defined(XPAR_SYSTEM_TURBO_DECODER_AXI_LI_0_0_S_AXI_BASEADDR)
#define TURBO_DECODER_BASEADDR XPAR_SYSTEM_TURBO_DECODER_AXI_LI_0_0_S_AXI_BASEADDR
#else
#warning "Using default TURBO_DECODER_BASEADDR=0x40000000. Check Vivado Address Editor or xparameters.h."
#define TURBO_DECODER_BASEADDR 0x40000000U
#endif
#endif

typedef struct {
    UINTPTR baseaddr;
} turbo_decoder_t;

void td_init(turbo_decoder_t *dev, UINTPTR baseaddr);
u32 td_read_reg(const turbo_decoder_t *dev, u32 offset);
void td_write_reg(const turbo_decoder_t *dev, u32 offset, u32 value);
u32 td_read_status(const turbo_decoder_t *dev);

void td_clear_done(const turbo_decoder_t *dev);
int td_axi_smoke_test(const turbo_decoder_t *dev);
int td_load_one_row(const turbo_decoder_t *dev, u32 bram_sel, u32 row, u16 value);
int td_load_input_bram(const turbo_decoder_t *dev, u32 bram_sel,
                       const uint16_t *rows, u32 row_count);
int td_load_all_input_brams(const turbo_decoder_t *dev,
                            const uint16_t input_bram[TD_NUM_INPUT_BRAMS][TD_INPUT_ROW_DEPTH]);
void td_start_decode(const turbo_decoder_t *dev);
int td_wait_busy_asserted(const turbo_decoder_t *dev, u32 timeout_polls, u32 *last_status);
int td_wait_done(const turbo_decoder_t *dev, u32 timeout_polls, u32 *last_status);
u8 td_read_hard_row(const turbo_decoder_t *dev, u32 row);
int td_read_all_hard_bits(const turbo_decoder_t *dev, uint8_t *decoded_bits, u32 bit_count);
void td_print_hard_rows(const turbo_decoder_t *dev, u32 row_count);
u32 td_count_bit_errors(const uint8_t *actual, const uint8_t *expected, u32 bit_count);
int td_first_mismatch(const uint8_t *actual, const uint8_t *expected, u32 bit_count);

#endif
