/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright(c) 2021 Sanpe <sanpeqf@gmail.com>
 */

/* project specific includes */
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>
#include "libusb_helper.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define PL2303_WRITE_REQ_TYPE       0x40
#define PL2303_WRITE_REQ            0x01
#define PL2303_WRITE_NREQ           0x80

#define PL2303_CTL_REQ_TYPE         0x21
#define PL2303_CTL_REQ              0x22
#define PL2303_CTL_DTR              0x01
#define PL2303_CTL_RTS              0x02

#define PL2303_REG0_OFF             0x00
#define PL2303_REG0_RFRCTS          0x01
#define PL2303_REG0_DTRDSR          0x08
#define PL2303_REG0_RTSCTS          0x41
#define PL2303_REG0_XONXOFF         0xc0

#define PL2303_REG1_GP0_ENABLE      0x10
#define PL2303_REG1_GP1_ENABLE      0x20
#define PL2303_REG1_GP0_VALUE       0x40
#define PL2303_REG1_GP1_VALUE       0x80

#define PL2303_REG6_GP4_ENABLE      0x03
#define PL2303_REG6_GP5_ENABLE      0x0c
#define PL2303_REG6_GP6_ENABLE      0x30
#define PL2303_REG6_GP7_ENABLE      0xc0

#define PL2303_REG7_GP4_VALUE       0x01
#define PL2303_REG7_GP5_VALUE       0x02
#define PL2303_REG7_GP6_VALUE       0x04
#define PL2303_REG7_GP7_VALUE       0x08

#define PL2303_REG12_GP2_ENABLE     0x03
#define PL2303_REG12_GP3_ENABLE     0x0c

#define PL2303_REG12_GP2_VALUE      0x01
#define PL2303_REG12_GP3_VALUE      0x02

#define PL2303_EXT_DEVTYPE_LOW      0x00
#define PL2303_EXT_DEVTYPE_HIGH     0x01

enum pl2303_pin_num {
    PL2303_PIN_D0   = 0,
    PL2303_PIN_D1   = 1,
    PL2303_PIN_D2   = 2,
    PL2303_PIN_D3   = 3,
    PL2303_PIN_D4   = 4,
    PL2303_PIN_D5   = 5,
    PL2303_PIN_D6   = 6,
    PL2303_PIN_D7   = 7,
    PL2303_PIN_D8   = 8,
    PL2303_PIN_D9   = 9,
    PL2303_PIN_NULL,
};

static const char *pl2303_pin_name[] = {
    [PL2303_PIN_D0] = "GP0", /* IO:  General IO 0 */
    [PL2303_PIN_D1] = "GP1", /* IO:  General IO 1 */
    [PL2303_PIN_D2] = "GP2", /* ICO: General IO 2 */
    [PL2303_PIN_D3] = "GP3", /* ICO: General IO 3 */
    [PL2303_PIN_D4] = "RIN", /* ICO: Ring In */
    [PL2303_PIN_D5] = "DCD", /* ICO: Data Carrier Detect */
    [PL2303_PIN_D6] = "DSR", /* ICO: Data Set Ready */
    [PL2303_PIN_D7] = "CTS", /* ICO: Clear To Send */
    [PL2303_PIN_D8] = "DTR", /* OUT: Data Terminal Ready */
    [PL2303_PIN_D9] = "RTS", /* OUT: Request To Send */
};

#define PL2303_D0_BIT       (1U << PL2303_PIN_D0)
#define PL2303_D1_BIT       (1U << PL2303_PIN_D1)
#define PL2303_D2_BIT       (1U << PL2303_PIN_D2)
#define PL2303_D3_BIT       (1U << PL2303_PIN_D3)
#define PL2303_D4_BIT       (1U << PL2303_PIN_D4)
#define PL2303_D5_BIT       (1U << PL2303_PIN_D5)
#define PL2303_D6_BIT       (1U << PL2303_PIN_D6)
#define PL2303_D7_BIT       (1U << PL2303_PIN_D7)
#define PL2303_D8_BIT       (1U << PL2303_PIN_D8)
#define PL2303_D9_BIT       (1U << PL2303_PIN_D9)

static unsigned int pl2303_tck_gpio     = PL2303_PIN_D8;
static unsigned int pl2303_tms_gpio     = PL2303_PIN_D9;
static unsigned int pl2303_tdo_gpio     = PL2303_PIN_D4;
static unsigned int pl2303_tdi_gpio     = PL2303_PIN_D5;
static unsigned int pl2303_trst_gpio    = PL2303_PIN_D0;
static unsigned int pl2303_srst_gpio    = PL2303_PIN_D1;

#define PL2303_TCK_BIT      (1U << pl2303_tck_gpio)
#define PL2303_TMS_BIT      (1U << pl2303_tms_gpio)
#define PL2303_TDO_BIT      (1U << pl2303_tdo_gpio)
#define PL2303_TDI_BIT      (1U << pl2303_tdi_gpio)
#define PL2303_TRST_BIT     (1U << pl2303_trst_gpio)
#define PL2303_SRST_BIT     (1U << pl2303_srst_gpio)

static uint16_t pl2303_vid = 0x067b;
static uint16_t pl2303_pid = 0x2303;
static struct libusb_device_handle *pl2303_adapter;

static uint8_t pl2303_reg0_shadow   = 0x01;
static uint8_t pl2303_reg1_shadow   = 0x00;
static uint8_t pl2303_reg2_shadow   = 0x44;
static uint8_t pl2303_reg6_shadow   = 0x00;
static uint8_t pl2303_reg7_shadow   = 0x00;
static uint8_t pl2303_reg12_shadow  = 0x00;
static uint8_t pl2303_reg13_shadow  = 0x00;

static uint16_t *pl2303_port_buffer;
static unsigned long pl2303_port_buffer_curr;
static unsigned long pl2303_port_buffer_size = 4096;

static enum pl2303_pin_num pl2303_name_to_pin(const char *name)
{
    unsigned int count;

    for (count = 0; count < ARRAY_SIZE(pl2303_pin_name); ++count)
        if (!strcmp(pl2303_pin_name[count], name))
            return count;

    return PL2303_PIN_NULL;
}

static bool pl2303_pin_is_read(enum pl2303_pin_num num)
{
    return num <= PL2303_PIN_D7;
}

static bool pl2303_pin_is_write(enum pl2303_pin_num num)
{
    return num <= PL2303_PIN_D9;
}

COMMAND_HANDLER(pl2303_handle_vid_pid_command)
{
    if (CMD_ARGC > 2) {
        LOG_WARNING("ignoring extra IDs in pl2303_vid_pid "
                    "(maximum is 1 pair)");
        CMD_ARGC = 2;
    }
    if (CMD_ARGC == 2) {
        COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], pl2303_vid);
        COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], pl2303_pid);
    } else
        LOG_WARNING("incomplete pl2303_vid_pid configuration");

    return ERROR_OK;
}

COMMAND_HANDLER(pl2303_handle_jtag_nums_command)
{
    if (CMD_ARGC != 8)
        return ERROR_COMMAND_SYNTAX_ERROR;

    pl2303_tck_gpio     = pl2303_name_to_pin(CMD_ARGV[0]);
    pl2303_tms_gpio     = pl2303_name_to_pin(CMD_ARGV[1]);
    pl2303_tdo_gpio     = pl2303_name_to_pin(CMD_ARGV[2]);
    pl2303_tdi_gpio     = pl2303_name_to_pin(CMD_ARGV[3]);
    pl2303_trst_gpio    = pl2303_name_to_pin(CMD_ARGV[4]);
    pl2303_srst_gpio    = pl2303_name_to_pin(CMD_ARGV[5]);

    if (!pl2303_pin_is_write(pl2303_tck_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    if (!pl2303_pin_is_write(pl2303_tms_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    if (!pl2303_pin_is_read(pl2303_tdo_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    if (!pl2303_pin_is_write(pl2303_tdi_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    if (!pl2303_pin_is_write(pl2303_trst_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    if (!pl2303_pin_is_write(pl2303_srst_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    command_print(
        CMD,
        "pl2303 nums: "
        "TCK = %d %s, TMS = %d %s, TDI = %d %s,"
        "TDO = %d %s, TRST = %d %s, SRST = %d %s",
        pl2303_tck_gpio,    pl2303_pin_name[pl2303_tck_gpio],
        pl2303_tms_gpio,    pl2303_pin_name[pl2303_tms_gpio],
        pl2303_tdo_gpio,    pl2303_pin_name[pl2303_tdo_gpio],
        pl2303_tdi_gpio,    pl2303_pin_name[pl2303_tdi_gpio],
        pl2303_trst_gpio,   pl2303_pin_name[pl2303_trst_gpio],
        pl2303_srst_gpio,   pl2303_pin_name[pl2303_srst_gpio]
    );

    return ERROR_OK;
}

COMMAND_HANDLER(pl2303_handle_tck_num_command)
{
    if (CMD_ARGC != 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    pl2303_tck_gpio = pl2303_name_to_pin(CMD_ARGV[0]);

    if (!pl2303_pin_is_write(pl2303_tck_gpio))
        return ERROR_COMMAND_SYNTAX_ERROR;

    command_print(
        CMD, "pl2303 num: TCK = %d %s",
        pl2303_tck_gpio, pl2303_pin_name[pl2303_tck_gpio]
    );

    return ERROR_OK;
}

COMMAND_HANDLER(pl2303_handle_tms_num_command)
{
    if (CMD_ARGC != 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    pl2303_tms_gpio = pl2303_name_to_pin(CMD_ARGV[0]);

    if (!pl2303_pin_is_write(pl2303_tms_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    command_print(
        CMD, "pl2303 num: TMS = %d %s",
        pl2303_tms_gpio, pl2303_pin_name[pl2303_tms_gpio]
    );

    return ERROR_OK;
}

COMMAND_HANDLER(pl2303_handle_tdo_num_command)
{
    if (CMD_ARGC != 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    pl2303_tdo_gpio = pl2303_name_to_pin(CMD_ARGV[0]);

    if (!pl2303_pin_is_read(pl2303_tdo_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    command_print(
        CMD, "pl2303 num: TDO = %d %s",
        pl2303_tdo_gpio, pl2303_pin_name[pl2303_tdo_gpio]
    );

    return ERROR_OK;
}

COMMAND_HANDLER(pl2303_handle_tdi_num_command)
{
    if (CMD_ARGC != 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    pl2303_tdi_gpio = pl2303_name_to_pin(CMD_ARGV[0]);

    if (!pl2303_pin_is_write(pl2303_tdi_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    command_print(
        CMD, "pl2303 num: TDI = %d %s",
        pl2303_tdi_gpio, pl2303_pin_name[pl2303_tdi_gpio]
    );

    return ERROR_OK;
}

COMMAND_HANDLER(pl2303_handle_trst_num_command)
{
    if (CMD_ARGC != 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    pl2303_trst_gpio = pl2303_name_to_pin(CMD_ARGV[0]);

    if (!pl2303_pin_is_write(pl2303_trst_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    command_print(
        CMD, "pl2303 num: TRST = %d %s",
        pl2303_trst_gpio, pl2303_pin_name[pl2303_trst_gpio]
    );

    return ERROR_OK;
}

COMMAND_HANDLER(pl2303_handle_srst_num_command)
{
    if (CMD_ARGC != 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    pl2303_srst_gpio = pl2303_name_to_pin(CMD_ARGV[0]);

    if (!pl2303_pin_is_write(pl2303_srst_gpio))
        return ERROR_COMMAND_CLOSE_CONNECTION;

    command_print(
        CMD, "pl2303 num: SRST = %d %s",
        pl2303_srst_gpio, pl2303_pin_name[pl2303_srst_gpio]
    );

    return ERROR_OK;
}

static inline uint8_t pl2303_read_register(uint16_t idx)
{
    uint8_t value;

    jtag_libusb_control_transfer(
        pl2303_adapter,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
        LIBUSB_RECIPIENT_DEVICE, PL2303_WRITE_REQ,
        (idx | 0x80), 0, (void *)&value, 1, 1000
    );

    return value;
}

static inline void pl2303_write_register(uint16_t idx, uint8_t value)
{
    jtag_libusb_control_transfer(
        pl2303_adapter,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
        LIBUSB_RECIPIENT_DEVICE, PL2303_WRITE_REQ,
        idx, value, NULL, 0, 1000
    );

    pl2303_read_register(idx);
}

static inline uint8_t pl2303_read_extended(uint16_t idx)
{
    pl2303_read_register(4);
    pl2303_write_register(4, idx);
    return pl2303_read_register(3);
}

static inline void pl2303_write_extended(uint16_t idx, uint8_t value)
{
    pl2303_read_register(4);
    pl2303_write_register(4, idx);
    pl2303_write_register(3, value);
}

static void pl2303_port_direction(uint16_t dire)
{
    if (dire & PL2303_D0_BIT)
        pl2303_reg1_shadow |= PL2303_REG1_GP0_ENABLE;
    else
        pl2303_reg1_shadow &= ~PL2303_REG1_GP0_ENABLE;

    if (dire & PL2303_D1_BIT)
        pl2303_reg1_shadow |= PL2303_REG1_GP1_ENABLE;
    else
        pl2303_reg1_shadow &= ~PL2303_REG1_GP1_ENABLE;

    pl2303_write_register(1, pl2303_reg1_shadow);

    if (dire & PL2303_D2_BIT)
        pl2303_reg12_shadow |= PL2303_REG12_GP2_ENABLE;
    else
        pl2303_reg12_shadow &= ~PL2303_REG12_GP2_ENABLE;

    if (dire & PL2303_D3_BIT)
        pl2303_reg12_shadow |= PL2303_REG12_GP3_ENABLE;
    else
        pl2303_reg12_shadow &= ~PL2303_REG12_GP3_ENABLE;

    pl2303_write_register(12, pl2303_reg12_shadow);

    if (dire & PL2303_D4_BIT)
        pl2303_reg6_shadow |= PL2303_REG6_GP4_ENABLE;
    else
        pl2303_reg6_shadow &= ~PL2303_REG6_GP4_ENABLE;

    if (dire & PL2303_D5_BIT)
        pl2303_reg6_shadow |= PL2303_REG6_GP5_ENABLE;
    else
        pl2303_reg6_shadow &= ~PL2303_REG6_GP5_ENABLE;

    if (dire & PL2303_D6_BIT)
        pl2303_reg6_shadow |= PL2303_REG6_GP6_ENABLE;
    else
        pl2303_reg6_shadow &= ~PL2303_REG6_GP6_ENABLE;

    if (dire & PL2303_D7_BIT)
        pl2303_reg6_shadow |= PL2303_REG6_GP7_ENABLE;
    else
        pl2303_reg6_shadow &= ~PL2303_REG6_GP7_ENABLE;

    pl2303_write_register(6, pl2303_reg6_shadow);
}

static void pl2303_port_transfer(uint16_t *buff, unsigned int len, bool read)
{
    for (; len--; buff++) {
        uint8_t value = 0;

        if (*buff & PL2303_D0_BIT)
            pl2303_reg1_shadow |= PL2303_REG1_GP0_VALUE;
        else
            pl2303_reg1_shadow &= ~PL2303_REG1_GP0_VALUE;

        if (*buff & PL2303_D1_BIT)
            pl2303_reg1_shadow |= PL2303_REG1_GP1_VALUE;
        else
            pl2303_reg1_shadow &= ~PL2303_REG1_GP1_VALUE;

        pl2303_write_register(1, pl2303_reg1_shadow);

        if (*buff & PL2303_D2_BIT)
            pl2303_reg12_shadow |= PL2303_REG12_GP2_VALUE;
        else
            pl2303_reg12_shadow &= ~PL2303_REG12_GP2_VALUE;

        if (*buff & PL2303_D3_BIT)
            pl2303_reg12_shadow |= PL2303_REG12_GP3_VALUE;
        else
            pl2303_reg12_shadow &= ~PL2303_REG12_GP3_VALUE;

        pl2303_write_register(12, pl2303_reg12_shadow);

        if (*buff & PL2303_D4_BIT)
            pl2303_reg7_shadow |= PL2303_REG7_GP4_VALUE;
        else
            pl2303_reg7_shadow &= ~PL2303_REG7_GP4_VALUE;

        if (*buff & PL2303_D5_BIT)
            pl2303_reg7_shadow |= PL2303_REG7_GP5_VALUE;
        else
            pl2303_reg7_shadow &= ~PL2303_REG7_GP5_VALUE;

        if (*buff & PL2303_D6_BIT)
            pl2303_reg7_shadow |= PL2303_REG7_GP6_VALUE;
        else
            pl2303_reg7_shadow &= ~PL2303_REG7_GP6_VALUE;

        if (*buff & PL2303_D7_BIT)
            pl2303_reg7_shadow |= PL2303_REG7_GP7_VALUE;
        else
            pl2303_reg7_shadow &= ~PL2303_REG7_GP7_VALUE;

        pl2303_write_register(7, pl2303_reg7_shadow);

        if (!(*buff & PL2303_D8_BIT))
            value |= PL2303_CTL_DTR;

        if (!(*buff & PL2303_D9_BIT))
            value |= PL2303_CTL_RTS;

        jtag_libusb_control_transfer(
            pl2303_adapter, PL2303_CTL_REQ_TYPE,
            PL2303_CTL_REQ, value, 0, NULL, 0, 1000
        );

        if (read) {
            uint8_t input = 0;

            value = pl2303_read_register(1);

            if (value & PL2303_REG1_GP0_VALUE)
                input |= PL2303_D0_BIT;

            if (value & PL2303_REG1_GP1_VALUE)
                input |= PL2303_D1_BIT;

            value = pl2303_read_register(12);

            if (value & PL2303_REG12_GP2_VALUE)
                input |= PL2303_D2_BIT;

            if (value & PL2303_REG12_GP3_VALUE)
                input |= PL2303_D3_BIT;

            value = pl2303_read_register(7);

            if (value & PL2303_REG7_GP4_VALUE)
                input |= PL2303_D4_BIT;

            if (value & PL2303_REG7_GP5_VALUE)
                input |= PL2303_D5_BIT;

            if (value & PL2303_REG7_GP6_VALUE)
                input |= PL2303_D6_BIT;

            if (value & PL2303_REG7_GP7_VALUE)
                input |= PL2303_D7_BIT;

            *buff = input;
        }
    }
}

static void pl2303_port_buffer_flush(bool read)
{
    pl2303_port_transfer(pl2303_port_buffer, pl2303_port_buffer_curr, read);
    pl2303_port_buffer_curr = 0;
}

static void pl2303_port_buffer_increase(unsigned long new_size)
{
    uint16_t *new_ptr;

    if (new_size < pl2303_port_buffer_size)
        return;

    new_size = pl2303_port_buffer_size * 2;
    new_ptr = realloc(pl2303_port_buffer, new_size);
    if (!new_ptr)
        return;

    pl2303_port_buffer = new_ptr;
    pl2303_port_buffer_size = new_size;
}

static inline void pl2303_jtag_write(bool tck, bool tms, bool tdi)
{
    uint16_t value = ~0;

    value = tck ? value | PL2303_TCK_BIT : value & ~PL2303_TCK_BIT;
    value = tms ? value | PL2303_TMS_BIT : value & ~PL2303_TMS_BIT;
    value = tdi ? value | PL2303_TDI_BIT : value & ~PL2303_TDI_BIT;

    pl2303_port_buffer_increase(pl2303_port_buffer_curr);
    if (pl2303_port_buffer_curr >= pl2303_port_buffer_size) {
        LOG_ERROR("pl2303_jtag_write: buffer overflow");
        exit(1);
    }

    pl2303_port_buffer[pl2303_port_buffer_curr++] = value;
}

static inline void pl2303_jtag_reset(bool trst, bool srst)
{
    uint16_t value = ~0;

    value = trst ? value & ~(1 << pl2303_trst_gpio) : value | (1 << pl2303_trst_gpio);
    value = srst ? value & ~(1 << pl2303_srst_gpio) : value | (1 << pl2303_srst_gpio);

    pl2303_port_buffer_increase(pl2303_port_buffer_curr);
    if (pl2303_port_buffer_curr >= pl2303_port_buffer_size) {
        LOG_ERROR("pl2303_jtag_reset: buffer overflow");
        exit(1);
    }

    pl2303_port_buffer[pl2303_port_buffer_curr++] = value;
    pl2303_port_buffer_flush(false);
}

static void syncbb_end_state(tap_state_t state)
{
    if (tap_is_state_stable(state))
        tap_set_end_state(state);
    else {
        LOG_ERROR("BUG: %i is not a valid end state", state);
        exit(-1);
    }
}

static void syncbb_state_move(int skip)
{
    uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
    int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
    bool tms = 0;
    int count;

    for (count = skip; count < tms_count; ++count) {
        tms = (tms_scan >> count) & 0x01;
        pl2303_jtag_write(0, tms, 0);
        pl2303_jtag_write(1, tms, 0);
    }

    pl2303_jtag_write(0, tms, 0);
    tap_set_state(tap_get_end_state());
}

static void syncbb_execute_tms(struct jtag_command *cmd)
{
    unsigned num_bits = cmd->cmd.tms->num_bits;
    const uint8_t *bits = cmd->cmd.tms->bits;
    bool tms = 0;

    LOG_DEBUG_IO("TMS: %d bits", num_bits);

    for (unsigned i = 0; i < num_bits; i++) {
        tms = ((bits[i / 8] >> (i % 8)) & 1);
        pl2303_jtag_write(0, tms, 0);
        pl2303_jtag_write(1, tms, 0);
    }

    pl2303_jtag_write(0, tms, 0);
}

static void syncbb_path_move(struct pathmove_command *cmd)
{
    int num_states = cmd->num_states;
    int state_count = 0;
    bool tms = 0;

    while (num_states--) {
        if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count]) {
            tms = 0;
        } else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count]) {
            tms = 1;
        } else {
            LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
                tap_state_name(tap_get_state()),
                tap_state_name(cmd->path[state_count]));
            exit(-1);
        }

        pl2303_jtag_write(0, tms, 0);
        pl2303_jtag_write(1, tms, 0);
        tap_set_state(cmd->path[state_count]);
        state_count++;
    }

    pl2303_jtag_write(0, tms, 0);
    tap_set_end_state(tap_get_state());
}

static void syncbb_runtest(unsigned int cycles)
{
    tap_state_t saved_end_state = tap_get_end_state();
    unsigned int count;

    if (tap_get_state() != TAP_IDLE) {
        syncbb_end_state(TAP_IDLE);
        syncbb_state_move(0);
    }

    for (count = 0; count < cycles; ++count) {
        pl2303_jtag_write(0, 0, 0);
        pl2303_jtag_write(1, 0, 0);
    }

    pl2303_jtag_write(0, 0, 0);
    syncbb_end_state(saved_end_state);

    if (tap_get_state() != tap_get_end_state())
        syncbb_state_move(0);
}

static void syncbb_stableclocks(unsigned int cycles)
{
    bool tms = tap_get_state() == TAP_RESET;
    unsigned int count;

    for (count = 0; count < cycles; ++count) {
        pl2303_jtag_write(1, tms, 0);
        pl2303_jtag_write(0, tms, 0);
    }
}

static void syncbb_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
    tap_state_t saved_end_state = tap_get_end_state();
    unsigned long bit_base;
    int bit_cnt;

    if (!((!ir_scan && (tap_get_state() == TAP_DRSHIFT)) || (ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
        if (ir_scan)
            syncbb_end_state(TAP_IRSHIFT);
        else
            syncbb_end_state(TAP_DRSHIFT);

        syncbb_state_move(0);
        syncbb_end_state(saved_end_state);
    }

    bit_base = pl2303_port_buffer_curr;

    for (bit_cnt = 0; bit_cnt < scan_size; ++bit_cnt) {
        int bcval, bytec;
        bool tdi, tms;

        bcval = 1 << (bit_cnt % 8);
        bytec = bit_cnt / 8;
        tms = bit_cnt == scan_size - 1;
        tdi = (type != SCAN_IN) && (buffer[bytec] & bcval);

        pl2303_jtag_write(0, tms, tdi);
        pl2303_jtag_write(1, tms, tdi);
    }

    if (tap_get_state() != tap_get_end_state())
        syncbb_state_move(1);

    pl2303_port_buffer_flush(type != SCAN_OUT);

    if (type != SCAN_OUT) {
        for (bit_cnt = 0; bit_cnt < scan_size; ++bit_cnt) {
            int bcval, bytec;
            uint8_t value;

            bcval = 1 << (bit_cnt % 8);
            bytec = bit_cnt / 8;
            value = pl2303_port_buffer[bit_base + bit_cnt * 2 + 1];

            if (value & PL2303_TDO_BIT)
                buffer[bytec] |= bcval;
            else
                buffer[bytec] &= ~bcval;
        }
    }
}

static int pl2303_execute_queue(void)
{
    struct jtag_command *cmd = jtag_command_queue;
    enum scan_type type;
    int scan_size;
    uint8_t *buffer;
    int retval = ERROR_OK;

    while (cmd) {
        switch (cmd->type) {
            case JTAG_RESET:
                LOG_DEBUG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
                if ((cmd->cmd.reset->trst == 1) ||
                    (cmd->cmd.reset->srst &&
                    (jtag_get_reset_config() & RESET_SRST_PULLS_TRST))) {
                    tap_set_state(TAP_RESET);
                }
                pl2303_jtag_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
                break;

            case JTAG_RUNTEST:
                LOG_DEBUG_IO("runtest %i cycles, end in %s", cmd->cmd.runtest->num_cycles,
                    tap_state_name(cmd->cmd.runtest->end_state));
                syncbb_end_state(cmd->cmd.runtest->end_state);
                syncbb_runtest(cmd->cmd.runtest->num_cycles);
                break;

            case JTAG_STABLECLOCKS:
                syncbb_stableclocks(cmd->cmd.stableclocks->num_cycles);
                break;

            case JTAG_TLR_RESET:
                LOG_DEBUG_IO("statemove end in %s", tap_state_name(cmd->cmd.statemove->end_state));
                syncbb_end_state(cmd->cmd.statemove->end_state);
                syncbb_state_move(0);
                break;

            case JTAG_PATHMOVE:
                LOG_DEBUG_IO("pathmove: %i states, end in %s", cmd->cmd.pathmove->num_states,
                    tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
                syncbb_path_move(cmd->cmd.pathmove);
                break;

            case JTAG_SCAN:
                LOG_DEBUG_IO("%s scan end in %s",  (cmd->cmd.scan->ir_scan) ? "IR" : "DR",
                    tap_state_name(cmd->cmd.scan->end_state));
                syncbb_end_state(cmd->cmd.scan->end_state);
                scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
                type = jtag_scan_type(cmd->cmd.scan);
                syncbb_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
                if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
                    retval = ERROR_JTAG_QUEUE_FAILED;
                free(buffer);
                break;

            case JTAG_SLEEP:
                LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);
                jtag_sleep(cmd->cmd.sleep->us);
                break;

            case JTAG_TMS:
                syncbb_execute_tms(cmd);
                break;

            default:
                LOG_ERROR("BUG: unknown JTAG command type encountered");
                exit(-1);
        }

        if (pl2303_port_buffer_curr)
            pl2303_port_buffer_flush(false);

        cmd = cmd->next;
    }

    return retval;
}

static void pl2303_hwinit(void)
{
    uint8_t dtl, dth;

    pl2303_write_register(0, pl2303_reg0_shadow);
    pl2303_write_register(1, pl2303_reg1_shadow);
    pl2303_write_register(2, pl2303_reg2_shadow);

    dtl = pl2303_read_extended(PL2303_EXT_DEVTYPE_LOW);
    dth = pl2303_read_extended(PL2303_EXT_DEVTYPE_HIGH);
    LOG_INFO("pl2303 version %02x:%02x", dtl, dth);

    pl2303_port_direction(
        PL2303_TDI_BIT |
        PL2303_TCK_BIT |
        PL2303_TMS_BIT |
        PL2303_TRST_BIT |
        PL2303_SRST_BIT
    );
}

static int pl2303_init(void)
{
    uint16_t avids[] = {pl2303_vid, 0};
    uint16_t apids[] = {pl2303_pid, 0};
    uint8_t desc[0x12];

    if (jtag_libusb_open(avids, apids, &pl2303_adapter, NULL)) {
        const char *pl2303_serial_desc = adapter_get_required_serial();
        LOG_ERROR("pl2303 not found: vid=%04x, pid=%04x, serial=%s\n",
            pl2303_vid, pl2303_pid, (!pl2303_serial_desc) ? "[any]" : pl2303_serial_desc);
        return ERROR_JTAG_INIT_FAILED;
    }

    if (libusb_kernel_driver_active(pl2303_adapter, 0)) {
        if (libusb_detach_kernel_driver(pl2303_adapter, 0)) {
            LOG_ERROR("Failed to detach kernel driver");
            return ERROR_JTAG_INIT_FAILED;
        }
    }

    if (libusb_claim_interface(pl2303_adapter, 0)) {
        LOG_ERROR("Failed to claim interface 0");
        return ERROR_JTAG_INIT_FAILED;
    }

    if (libusb_get_descriptor(pl2303_adapter, LIBUSB_DT_DEVICE, 0x00, desc, 0x12) < 0) {
        LOG_ERROR("Failed to get device descriptor");
        return ERROR_JTAG_INIT_FAILED;
    }

    pl2303_port_buffer = malloc(pl2303_port_buffer_size);
    if (!pl2303_port_buffer) {
        LOG_ERROR("Unable to allocate memory for the buffer");
        return ERROR_JTAG_INIT_FAILED;
    }

    pl2303_hwinit();

    return ERROR_OK;
}

static int pl2303_quit(void)
{
    jtag_libusb_close(pl2303_adapter);

	free(pl2303_port_buffer);
	pl2303_port_buffer = NULL;
	pl2303_port_buffer_curr = 0;
	pl2303_port_buffer_size = 4096;

    pl2303_reg0_shadow   = 0x01;
    pl2303_reg1_shadow   = 0x00;
    pl2303_reg2_shadow   = 0x44;
    pl2303_reg6_shadow   = 0x00;
    pl2303_reg7_shadow   = 0x00;
    pl2303_reg12_shadow  = 0x00;
    pl2303_reg13_shadow  = 0x00;

    return ERROR_OK;
}

static const struct command_registration pl2303_subcommand_handlers[] = {
    {
        .name = "vid_pid",
        .handler = pl2303_handle_vid_pid_command,
        .mode = COMMAND_CONFIG,
        .help = "USB VID and PID of the adapter",
        .usage = "vid pid",
    }, {
        .name = "jtag_nums",
        .handler = pl2303_handle_jtag_nums_command,
        .mode = COMMAND_CONFIG,
        .help = "gpio numbers for tck, tms, tdo, tdi, trst, srst, swio, swclk.",
        .usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
    }, {
        .name = "tck_num",
        .handler = pl2303_handle_tck_num_command,
        .mode = COMMAND_CONFIG,
        .help = "gpio number for tck.",
        .usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
    }, {
        .name = "tms_num",
        .handler = pl2303_handle_tms_num_command,
        .mode = COMMAND_CONFIG,
        .help = "gpio number for tms.",
        .usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
    }, {
        .name = "tdo_num",
        .handler = pl2303_handle_tdo_num_command,
        .mode = COMMAND_CONFIG,
        .help = "gpio number for tdo.",
        .usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
    }, {
        .name = "tdi_num",
        .handler = pl2303_handle_tdi_num_command,
        .mode = COMMAND_CONFIG,
        .help = "gpio number for tdi.",
        .usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
    }, {
        .name = "trst_num",
        .handler = pl2303_handle_trst_num_command,
        .mode = COMMAND_CONFIG,
        .help = "gpio number for trst.",
        .usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
    }, {
        .name = "srst_num",
        .handler = pl2303_handle_srst_num_command,
        .mode = COMMAND_CONFIG,
        .help = "gpio number for srst.",
        .usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration pl2303_command_handlers[] = {
    {
        .name = "pl2303",
        .mode = COMMAND_ANY,
        .help = "perform pl2303 management",
        .chain = pl2303_subcommand_handlers,
        .usage = "",
    },
    COMMAND_REGISTRATION_DONE
};

static struct jtag_interface pl2303_jtag_ops = {
    .supported = DEBUG_CAP_TMS_SEQ,
    .execute_queue = pl2303_execute_queue,
};

struct adapter_driver pl2303_adapter_driver = {
    .name = "pl2303",
    .transports = jtag_only,
    .commands = pl2303_command_handlers,

    .init = pl2303_init,
    .quit = pl2303_quit,

    .jtag_ops = &pl2303_jtag_ops,
};
