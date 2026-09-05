/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Register map and bus timings for the PixArt PMW3610. The register
 * definitions and timing constants are taken from the datasheet, by way of
 * the ZMK community drivers this one replaces.
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bus timings, in microseconds. Sub-microsecond figures are rounded up
 * because k_busy_wait cannot express them. These are not optional padding:
 * T_SRAD in particular is the sensor's address-to-data turnaround, and
 * skipping it makes reads return data from the previous transaction. */
#define T_NCS_SCLK 1     /* 120 ns */
#define T_SCLK_NCS_WR 10 /* 10 us */
#define T_SRAD 4         /* 4 us */
#define T_SRAD_MOTBR 4   /* same as T_SRAD */
#define T_SRX 1          /* 250 ns */
#define T_SWX 30         /* SWW: 30 us, SWR: 20 us */
#define T_BEXIT 1        /* 250 ns */

/* Registers */
#define PMW3610_REG_PRODUCT_ID 0x00
#define PMW3610_REG_MOTION 0x02
#define PMW3610_REG_DELTA_X_L 0x03
#define PMW3610_REG_DELTA_Y_L 0x04
#define PMW3610_REG_DELTA_XY_H 0x05
#define PMW3610_REG_SQUAL 0x06
#define PMW3610_REG_SHUTTER_HIGHER 0x07
#define PMW3610_REG_SHUTTER_LOWER 0x08
#define PMW3610_REG_PERFORMANCE 0x11
#define PMW3610_REG_MOTION_BURST 0x12
#define PMW3610_REG_RUN_DOWNSHIFT 0x1B
#define PMW3610_REG_REST1_PERIOD 0x1C
#define PMW3610_REG_REST1_DOWNSHIFT 0x1D
#define PMW3610_REG_OBSERVATION 0x2D
#define PMW3610_REG_SMART_MODE 0x32
#define PMW3610_REG_POWER_UP_RESET 0x3A
#define PMW3610_REG_SPI_CLK_ON_REQ 0x41
#define PMW3610_REG_SPI_PAGE0 0x7F
#define PMW3610_REG_RES_STEP 0x85

#define PMW3610_PRODUCT_ID 0x3E

#define PMW3610_POWERUP_CMD_RESET 0x5A
#define PMW3610_SPI_CLOCK_CMD_ENABLE 0xBA
#define PMW3610_SPI_CLOCK_CMD_DISABLE 0xB5

#define PMW3610_BURST_SIZE 7
#define PMW3610_MAX_BURST_SIZE 10

/* Byte offsets within a motion burst */
#define PMW3610_X_L_POS 1
#define PMW3610_Y_L_POS 2
#define PMW3610_XY_H_POS 3
#define PMW3610_SQUAL_POS 4
#define PMW3610_SHUTTER_H_POS 5
#define PMW3610_SHUTTER_L_POS 6

#define PMW3610_MAX_CPI 3200
#define PMW3610_MIN_CPI 200

#define SPI_WRITE_BIT BIT(7)

/* Shutter value below which the smart algorithm is disengaged. */
#define PMW3610_SMART_SHUTTER_THRESHOLD 45

#if defined(CONFIG_PMW3610_POLLING_RATE_250) || defined(CONFIG_PMW3610_POLLING_RATE_125_SW)
#define PMW3610_POLLING_RATE_VALUE 0x0D
#elif defined(CONFIG_PMW3610_POLLING_RATE_125)
#define PMW3610_POLLING_RATE_VALUE 0x00
#else
#error "A valid PMW3610 polling rate must be selected"
#endif

#ifdef CONFIG_PMW3610_FORCE_AWAKE
#define PMW3610_FORCE_MODE_VALUE 0xF0
#else
#define PMW3610_FORCE_MODE_VALUE 0x00
#endif

#define PMW3610_PERFORMANCE_VALUE (PMW3610_FORCE_MODE_VALUE | PMW3610_POLLING_RATE_VALUE)

#ifdef CONFIG_PMW3610_INVERT_SCROLL_X
#define PMW3610_SCROLL_X_NEGATIVE 1
#define PMW3610_SCROLL_X_POSITIVE -1
#else
#define PMW3610_SCROLL_X_NEGATIVE -1
#define PMW3610_SCROLL_X_POSITIVE 1
#endif

#ifdef CONFIG_PMW3610_INVERT_SCROLL_Y
#define PMW3610_SCROLL_Y_NEGATIVE 1
#define PMW3610_SCROLL_Y_POSITIVE -1
#else
#define PMW3610_SCROLL_Y_NEGATIVE -1
#define PMW3610_SCROLL_Y_POSITIVE 1
#endif

enum pmw3610_input_mode { PMW3610_MODE_MOVE = 0, PMW3610_MODE_SCROLL, PMW3610_MODE_SNIPE };

struct pmw3610_data {
    const struct device *dev;

    enum pmw3610_input_mode curr_mode;
    uint32_t curr_cpi;
    int32_t scroll_delta_x;
    int32_t scroll_delta_y;

#ifdef CONFIG_PMW3610_POLLING_RATE_125_SW
    int64_t last_poll_time;
    int16_t last_x;
    int16_t last_y;
#endif

    bool sw_smart_flag;

    struct gpio_callback irq_gpio_cb;
    struct k_work trigger_work;

    struct k_work_delayable init_work;
    int async_init_step;
    uint32_t init_attempts;

    struct k_work_delayable health_work;
    uint32_t consecutive_errs;
    uint32_t implausible_reports;
    uint32_t health_fails;
    int64_t last_report_time;

    bool ready;
};

struct pmw3610_config {
    struct gpio_dt_spec irq_gpio;
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
    const int32_t *scroll_layers;
    size_t scroll_layers_len;
    const int32_t *snipe_layers;
    size_t snipe_layers_len;
};

#ifdef __cplusplus
}
#endif
