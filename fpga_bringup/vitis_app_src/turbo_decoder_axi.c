#include "turbo_decoder_axi.h"

void td_init(turbo_decoder_t *dev, UINTPTR baseaddr)
{
    if (dev != 0) {
        dev->baseaddr = baseaddr;
    }
}

u32 td_read_reg(const turbo_decoder_t *dev, u32 offset)
{
    return Xil_In32(dev->baseaddr + (UINTPTR)offset);
}

void td_write_reg(const turbo_decoder_t *dev, u32 offset, u32 value)
{
    Xil_Out32(dev->baseaddr + (UINTPTR)offset, value);
}

u32 td_read_status(const turbo_decoder_t *dev)
{
    return td_read_reg(dev, TD_REG_STATUS);
}

void td_clear_done(const turbo_decoder_t *dev)
{
    td_write_reg(dev, TD_REG_CONTROL, TD_CONTROL_CLEAR_DONE);
}

int td_axi_smoke_test(const turbo_decoder_t *dev)
{
    u32 readback;

    if (dev == 0) {
        return TD_ERR_BAD_ARG;
    }

    td_clear_done(dev);

    td_write_reg(dev, TD_REG_LOAD_SEL, TD_LOAD_SEL_SYS_ILV_EVEN);
    readback = td_read_reg(dev, TD_REG_LOAD_SEL) & 0x7U;
    if (readback != TD_LOAD_SEL_SYS_ILV_EVEN) {
        return TD_ERR_AXI_READBACK;
    }

    td_write_reg(dev, TD_REG_LOAD_ADDR, 123U);
    readback = td_read_reg(dev, TD_REG_LOAD_ADDR) & 0x3FFU;
    if (readback != 123U) {
        return TD_ERR_AXI_READBACK;
    }

    td_write_reg(dev, TD_REG_LOAD_DATA, 0x155U);
    readback = td_read_reg(dev, TD_REG_LOAD_DATA) & 0x3FFU;
    if (readback != 0x155U) {
        return TD_ERR_AXI_READBACK;
    }

    td_write_reg(dev, TD_REG_HARD_ADDR, 7U);
    readback = td_read_reg(dev, TD_REG_HARD_ADDR) & 0x3FFU;
    if (readback != 7U) {
        return TD_ERR_AXI_READBACK;
    }

    return TD_OK;
}

int td_load_one_row(const turbo_decoder_t *dev, u32 bram_sel, u32 row, u16 value)
{
    if ((dev == 0) || (bram_sel >= TD_NUM_INPUT_BRAMS) || (row >= TD_INPUT_ROW_DEPTH)) {
        return TD_ERR_BAD_ARG;
    }

    td_write_reg(dev, TD_REG_LOAD_SEL, bram_sel);
    td_write_reg(dev, TD_REG_LOAD_ADDR, row);
    td_write_reg(dev, TD_REG_LOAD_DATA, ((u32)value) & 0x3FFU);
    td_write_reg(dev, TD_REG_LOAD_COMMIT, 1U);

    return TD_OK;
}

int td_load_input_bram(const turbo_decoder_t *dev, u32 bram_sel,
                       const uint16_t *rows, u32 row_count)
{
    u32 row;

    if ((dev == 0) || (rows == 0) || (bram_sel >= TD_NUM_INPUT_BRAMS)) {
        return TD_ERR_BAD_ARG;
    }

    if (row_count > TD_INPUT_ROW_DEPTH) {
        return TD_ERR_BAD_ARG;
    }

    td_write_reg(dev, TD_REG_LOAD_SEL, bram_sel);
    for (row = 0U; row < row_count; row++) {
        td_write_reg(dev, TD_REG_LOAD_ADDR, row);
        td_write_reg(dev, TD_REG_LOAD_DATA, ((u32)rows[row]) & 0x3FFU);
        td_write_reg(dev, TD_REG_LOAD_COMMIT, 1U);
    }

    return TD_OK;
}

int td_load_all_input_brams(const turbo_decoder_t *dev,
                            const uint16_t input_bram[TD_NUM_INPUT_BRAMS][TD_INPUT_ROW_DEPTH])
{
    u32 sel;
    int rc;

    if ((dev == 0) || (input_bram == 0)) {
        return TD_ERR_BAD_ARG;
    }

    for (sel = 0U; sel < TD_NUM_INPUT_BRAMS; sel++) {
        rc = td_load_input_bram(dev, sel, input_bram[sel], TD_INPUT_ROW_DEPTH);
        if (rc != TD_OK) {
            return rc;
        }
    }

    return TD_OK;
}

void td_start_decode(const turbo_decoder_t *dev)
{
    td_clear_done(dev);
    td_write_reg(dev, TD_REG_CONTROL, TD_CONTROL_START);
}

int td_wait_busy_asserted(const turbo_decoder_t *dev, u32 timeout_polls, u32 *last_status)
{
    u32 i;
    u32 status = 0U;

    if (dev == 0) {
        return TD_ERR_BAD_ARG;
    }

    for (i = 0U; i < timeout_polls; i++) {
        status = td_read_status(dev);
        if ((status & TD_STATUS_BUSY) != 0U) {
            if (last_status != 0) {
                *last_status = status;
            }
            return TD_OK;
        }
    }

    if (last_status != 0) {
        *last_status = status;
    }
    return TD_ERR_TIMEOUT;
}

int td_wait_done(const turbo_decoder_t *dev, u32 timeout_polls, u32 *last_status)
{
    u32 i;
    u32 status = 0U;

    if (dev == 0) {
        return TD_ERR_BAD_ARG;
    }

    for (i = 0U; i < timeout_polls; i++) {
        status = td_read_status(dev);
        if ((status & TD_STATUS_DONE) != 0U) {
            if (last_status != 0) {
                *last_status = status;
            }
            return TD_OK;
        }
    }

    if (last_status != 0) {
        *last_status = status;
    }
    return TD_ERR_TIMEOUT;
}

u8 td_read_hard_row(const turbo_decoder_t *dev, u32 row)
{
    if ((dev == 0) || (row >= TD_ROW_DEPTH)) {
        return 0U;
    }

    td_write_reg(dev, TD_REG_HARD_ADDR, row);

    /*
     * The hard-decision RAM is synchronous in the PL. The dummy read makes the
     * software sequence robust even if the AXI read arrives close to the
     * address update.
     */
    (void)td_read_reg(dev, TD_REG_HARD_BITS);
    return (u8)(td_read_reg(dev, TD_REG_HARD_BITS) & 0xFU);
}

int td_read_all_hard_bits(const turbo_decoder_t *dev, uint8_t *decoded_bits, u32 bit_count)
{
    u32 row;
    u8 hard;

    if ((dev == 0) || (decoded_bits == 0) || (bit_count < TD_K)) {
        return TD_ERR_BAD_ARG;
    }

    for (row = 0U; row < TD_ROW_DEPTH; row++) {
        hard = td_read_hard_row(dev, row);

        decoded_bits[(2U * row)]                 = (uint8_t)((hard >> 0) & 0x1U);
        decoded_bits[TD_SEG_LEN + (2U * row)]    = (uint8_t)((hard >> 1) & 0x1U);
        decoded_bits[(2U * row) + 1U]            = (uint8_t)((hard >> 2) & 0x1U);
        decoded_bits[TD_SEG_LEN + (2U * row) + 1U] = (uint8_t)((hard >> 3) & 0x1U);
    }

    return TD_OK;
}

void td_print_hard_rows(const turbo_decoder_t *dev, u32 row_count)
{
    u32 row;
    u8 hard;

    if (row_count > TD_ROW_DEPTH) {
        row_count = TD_ROW_DEPTH;
    }

    for (row = 0U; row < row_count; row++) {
        hard = td_read_hard_row(dev, row);
        xil_printf("row %d hard=0x%x bits: idx %d=%d, %d=%d, %d=%d, %d=%d\r\n",
                   (int)row,
                   (unsigned int)hard,
                   (int)(2U * row), (int)((hard >> 0) & 0x1U),
                   (int)(TD_SEG_LEN + (2U * row)), (int)((hard >> 1) & 0x1U),
                   (int)((2U * row) + 1U), (int)((hard >> 2) & 0x1U),
                   (int)(TD_SEG_LEN + (2U * row) + 1U), (int)((hard >> 3) & 0x1U));
    }
}

u32 td_count_bit_errors(const uint8_t *actual, const uint8_t *expected, u32 bit_count)
{
    u32 i;
    u32 errors = 0U;

    if ((actual == 0) || (expected == 0)) {
        return bit_count;
    }

    for (i = 0U; i < bit_count; i++) {
        if ((actual[i] & 0x1U) != (expected[i] & 0x1U)) {
            errors++;
        }
    }

    return errors;
}

int td_first_mismatch(const uint8_t *actual, const uint8_t *expected, u32 bit_count)
{
    u32 i;

    if ((actual == 0) || (expected == 0)) {
        return -1;
    }

    for (i = 0U; i < bit_count; i++) {
        if ((actual[i] & 0x1U) != (expected[i] & 0x1U)) {
            return (int)i;
        }
    }

    return -1;
}
