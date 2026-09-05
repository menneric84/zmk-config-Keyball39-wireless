/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * PMW3610 trackball driver.
 *
 * The register-level sequences here follow the datasheet and the ZMK
 * community drivers. What differs is the handling of bus faults, because on
 * this hardware they are routine rather than exceptional: the BLE radio and
 * the sensor share a power rail, and a radio burst can corrupt a SPI
 * transaction in flight. Three properties follow from that:
 *
 *   1. Chip-select is always released, on every exit path. A transfer that
 *      fails partway through otherwise leaves CS asserted, and the sensor
 *      sits mid-transaction with its framing misaligned until it loses
 *      power -- which is why a button reset does not clear it.
 *   2. Initialization retries instead of giving up. One corrupted read at
 *      boot costs a few milliseconds, not a dead trackball for that session.
 *   3. A desync is detected and repaired at runtime, both from repeated
 *      transfer failures and from a periodic health check that catches a
 *      sensor which has stopped asserting its motion interrupt entirely.
 */

#define DT_DRV_COMPAT pixart_pmw3610

/* 12-bit two's complement to int16_t */
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <stdlib.h>

#include <zmk/keymap.h>

#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_INPUT_LOG_LEVEL);

enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,
    ASYNC_INIT_STEP_CLEAR_OB1,
    ASYNC_INIT_STEP_CHECK_OB1,
    ASYNC_INIT_STEP_CONFIGURE,

    ASYNC_INIT_STEP_COUNT
};

/* Settling time required after each step, in milliseconds. The datasheet
 * minimums are considerably shorter; these are the values the community
 * drivers converged on after finding the spec figures too tight in practice,
 * especially when an I2C display is initializing concurrently. */
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10,
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200,
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static void pmw3610_restart(const struct device *dev, const char *reason);

/* ------------------------------------------------------------------ */
/* Bus access                                                          */
/* ------------------------------------------------------------------ */

static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pmw3610_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed (%d)", err);
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    const struct pmw3610_config *config = dev->config;
    int err, cs_err;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write (%d)", err);
        goto release;
    }

    k_busy_wait(T_SRAD);

    const struct spi_buf rx_buf = {.buf = buf, .len = 1};
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read (%d)", err);
    }

release:
    cs_err = spi_cs_ctrl(dev, false);
    k_busy_wait(T_SRX);

    return err ? err : cs_err;
}

/* Primitive write, without toggling the sensor's SPI clock request. */
static int _reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    const struct pmw3610_config *config = dev->config;
    int err, cs_err;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write (%d)", err);
    } else {
        k_busy_wait(T_SCLK_NCS_WR);
    }

    cs_err = spi_cs_ctrl(dev, false);
    k_busy_wait(T_SWX);

    return err ? err : cs_err;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (err) {
        return err;
    }

    err = _reg_write(dev, reg, val);

    /* Drop the clock request even after a failure, so the sensor is not left
     * burning power with its clock forced on. */
    int clk_err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    return err ? err : clk_err;
}

static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {
    const struct pmw3610_config *config = dev->config;
    int err, cs_err;

    __ASSERT_NO_MSG(burst_size <= PMW3610_MAX_BURST_SIZE);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t reg_buf[] = {PMW3610_REG_MOTION_BURST};
    const struct spi_buf tx_buf = {.buf = reg_buf, .len = ARRAY_SIZE(reg_buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI write (%d)", err);
        goto release;
    }

    k_busy_wait(T_SRAD_MOTBR);

    const struct spi_buf rx_buf = {.buf = buf, .len = burst_size};
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI read (%d)", err);
    }

release:
    cs_err = spi_cs_ctrl(dev, false);
    k_busy_wait(T_BEXIT);

    return err ? err : cs_err;
}

/* ------------------------------------------------------------------ */
/* Fault tracking                                                      */
/* ------------------------------------------------------------------ */

static void bus_error(const struct device *dev) {
    struct pmw3610_data *data = dev->data;

    data->consecutive_errs++;
    if (data->consecutive_errs >= CONFIG_PMW3610_MAX_CONSECUTIVE_ERRORS) {
        pmw3610_restart(dev, "consecutive bus errors");
    }
}

static void bus_ok(const struct device *dev) {
    struct pmw3610_data *data = dev->data;

    data->consecutive_errs = 0;
}

/* ------------------------------------------------------------------ */
/* Sensor configuration                                                */
/* ------------------------------------------------------------------ */

static void set_interrupt(const struct device *dev, const bool en) {
    const struct pmw3610_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("Can't set interrupt (%d)", ret);
    }
}

static int set_cpi(const struct device *dev, uint32_t cpi) {
    struct pmw3610_data *data = dev->data;

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    /* Resolution is expressed in hardware steps of 200 CPI, and the register
     * holding it lives on register page 1. */
    uint8_t value = cpi / 200;

    /* Treat the resolution as unknown for the duration: if any step below
     * fails, the caller must not believe the cached value. */
    data->curr_cpi = 0;

    int err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (err) {
        return err;
    }

    err = _reg_write(dev, PMW3610_REG_SPI_PAGE0, 0xFF);
    if (!err) {
        err = _reg_write(dev, PMW3610_REG_RES_STEP, value);
    }

    /*
     * The page must be restored even when the write above failed. Leaving the
     * sensor on page 1 makes every later read -- motion bursts included --
     * return unrelated registers, which surfaces as scrambled axes rather
     * than as an error, and persists until the sensor is reinitialized.
     */
    int page_err = _reg_write(dev, PMW3610_REG_SPI_PAGE0, 0x00);
    int clk_err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err || page_err || clk_err) {
        LOG_ERR("Failed to set CPI (write %d, page restore %d, clock %d)", err, page_err, clk_err);
        return err ? err : (page_err ? page_err : clk_err);
    }

    LOG_INF("CPI set to %u (reg 0x%x)", cpi, value);
    data->curr_cpi = cpi;

    return 0;
}

static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    struct pmw3610_data *data = dev->data;

    if (cpi == data->curr_cpi) {
        return 0;
    }

    return set_cpi(dev, cpi);
}

static int set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    const uint32_t mintime = 10;
    const uint32_t maxtime = 2550;

    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    return reg_write(dev, reg_addr, sample_time / mintime);
}

static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t mintime, maxtime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /* Run downshift = reg * 8 * position rate, and the position rate is
         * fixed at 4 ms by the performance register written during init. */
        mintime = 32;
        maxtime = 32 * 255;
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /* Rest1 downshift = reg * 16 * the Rest1 sample period. */
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        maxtime = 255 * mintime;
        break;

    default:
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range [%u, %u]", time, mintime, maxtime);
        return -EINVAL;
    }

    return reg_write(dev, reg_addr, time / mintime);
}

/* ------------------------------------------------------------------ */
/* Initialization                                                      */
/* ------------------------------------------------------------------ */

static int pmw3610_async_init_power_up(const struct device *dev) {
    /* Cycle CS to put the sensor's SPI state machine at a known starting
     * point before the first real transaction. */
    spi_cs_ctrl(dev, false);
    spi_cs_ctrl(dev, true);

    return reg_write(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return reg_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;

    int err = reg_read(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        return err;
    }

    /* The sensor sets the low nibble once its internal self-test passes. */
    if ((value & 0x0F) != 0x0F) {
        LOG_WRN("Self-test not passed (0x%x)", value);
        return -EINVAL;
    }

    uint8_t product_id = 0;
    err = reg_read(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_WRN("Incorrect product id 0x%x (expecting 0x%x)", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    struct pmw3610_data *data = dev->data;
    int err = 0;

    /* The datasheet requires the motion registers be read out once before
     * normal operation, to discard whatever accumulated during reset. */
    for (uint8_t reg = PMW3610_REG_MOTION; (reg <= PMW3610_REG_DELTA_XY_H) && !err; reg++) {
        uint8_t buf;
        err = reg_read(dev, reg, &buf);
    }

    /*
     * Everything the driver caches about the sensor is invalidated by the
     * power-up reset that preceded this step. Clearing it here rather than in
     * pmw3610_init is what makes a reinitialization a genuine fresh start --
     * sw_smart_flag in particular, because a stale value leaves the driver
     * believing the smart-mode register holds a value the reset just cleared,
     * and it would not correct it until the shutter next crossed the
     * threshold. Resetting to false lets the first motion report re-establish
     * the correct state.
     */
    data->curr_cpi = 0;
    data->sw_smart_flag = false;
#ifdef CONFIG_PMW3610_POLLING_RATE_125_SW
    data->last_poll_time = 0;
    data->last_x = 0;
    data->last_y = 0;
#endif

    if (!err) {
        err = set_cpi(dev, CONFIG_PMW3610_CPI);
    }
    if (!err) {
        err = reg_write(dev, PMW3610_REG_PERFORMANCE, PMW3610_PERFORMANCE_VALUE);
    }
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                 CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
    }
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST1_PERIOD, CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
    }
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                 CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
    }

    return err;
}

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

/* Delay before the next init attempt, doubling per attempt up to a cap. */
static uint32_t retry_delay_ms(uint32_t attempts) {
    uint32_t delay = CONFIG_PMW3610_INIT_RETRY_BACKOFF_MS;

    for (uint32_t i = 1; i < attempts && delay < CONFIG_PMW3610_RETRY_BACKOFF_MAX_MS; i++) {
        delay *= 2;
    }

    return MIN(delay, (uint32_t)CONFIG_PMW3610_RETRY_BACKOFF_MAX_MS);
}

/*
 * Take the sensor offline and schedule a fresh init sequence.
 *
 * Chip-select is released and left high for the duration of the scheduled
 * delay. That hold is the part that matters: a sensor whose SPI framing has
 * drifted will not respond correctly to any command, including the reset
 * command, so the bus has to be resynchronized electrically first. CS held
 * high past the sensor's timeout returns its interface to idle, which is
 * what a power cycle was previously accomplishing by brute force.
 */
static void pmw3610_restart(const struct device *dev, const char *reason) {
    struct pmw3610_data *data = dev->data;

    if (!data->ready && data->async_init_step == ASYNC_INIT_STEP_POWER_UP) {
        /* A restart is already pending; don't reset its backoff. */
        return;
    }

    LOG_WRN("Reinitializing trackball: %s", reason);

    data->ready = false;
    data->consecutive_errs = 0;
    data->health_fails = 0;
    data->framing_fails = 0;
    data->implausible_reports = 0;
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;

    set_interrupt(dev, false);
    spi_cs_ctrl(dev, false);

    k_work_reschedule(&data->init_work, K_MSEC(CONFIG_PMW3610_BUS_RECOVERY_HOLD_MS));
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pmw3610_data *data = CONTAINER_OF(dwork, struct pmw3610_data, init_work);
    const struct device *dev = data->dev;

    int err = async_init_fn[data->async_init_step](dev);

    if (err) {
        data->init_attempts++;
        uint32_t delay = retry_delay_ms(data->init_attempts);

        /* Only shout about it occasionally; a marginal bus can retry a lot. */
        if (data->init_attempts <= 3 || (data->init_attempts % 20) == 0) {
            LOG_WRN("Init step %d failed (%d), attempt %u, retrying in %u ms",
                    data->async_init_step, err, data->init_attempts, delay);
        }

        /* Restart the whole sequence: the later steps assume the earlier ones
         * landed, so resuming mid-sequence would configure a sensor that was
         * never properly reset. */
        data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
        spi_cs_ctrl(dev, false);
        k_work_reschedule(&data->init_work, K_MSEC(delay + CONFIG_PMW3610_BUS_RECOVERY_HOLD_MS));
        return;
    }

    data->async_init_step++;

    if (data->async_init_step != ASYNC_INIT_STEP_COUNT) {
        k_work_reschedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        return;
    }

    data->ready = true;
    data->consecutive_errs = 0;

    if (data->init_attempts > 0) {
        LOG_INF("PMW3610 initialized after %u retries", data->init_attempts);
    } else {
        LOG_INF("PMW3610 initialized");
    }
    data->init_attempts = 0;

    set_interrupt(dev, true);
}

/* ------------------------------------------------------------------ */
/* Health monitoring                                                   */
/* ------------------------------------------------------------------ */

#if CONFIG_PMW3610_HEALTH_POLL_INTERVAL_MS > 0
/*
 * Detect a motion burst that has lost byte alignment.
 *
 * A single-register read is self-framing -- address out, one byte back -- so
 * it keeps answering correctly even while the multi-byte burst is shifted.
 * That is why polling the product ID alone reports a healthy sensor while
 * tracking is visibly wrong. The only way to see the misalignment is to read
 * the same registers both ways and compare.
 *
 * SQUAL and the shutter value are stable while the ball is still, so the
 * caller runs this only when no motion has been reported recently.
 */
static int check_burst_framing(const struct device *dev) {
    uint8_t burst[PMW3610_BURST_SIZE];
    uint8_t squal, shutter_h, shutter_l;

    int err = motion_burst_read(dev, burst, sizeof(burst));
    if (err) {
        return err;
    }

    err = reg_read(dev, PMW3610_REG_SQUAL, &squal);
    if (!err) {
        err = reg_read(dev, PMW3610_REG_SHUTTER_HIGHER, &shutter_h);
    }
    if (!err) {
        err = reg_read(dev, PMW3610_REG_SHUTTER_LOWER, &shutter_l);
    }
    if (err) {
        return err;
    }

    if (burst[PMW3610_SQUAL_POS] != squal || burst[PMW3610_SHUTTER_H_POS] != shutter_h ||
        burst[PMW3610_SHUTTER_L_POS] != shutter_l) {
        LOG_WRN("Burst framing mismatch: burst squal 0x%x shutter 0x%x%x, "
                "registers 0x%x 0x%x%x",
                burst[PMW3610_SQUAL_POS], burst[PMW3610_SHUTTER_H_POS],
                burst[PMW3610_SHUTTER_L_POS], squal, shutter_h, shutter_l);
        return -EILSEQ;
    }

    return 0;
}

/*
 * A sensor that has desynced usually stops asserting its motion pin, so no
 * interrupt arrives and no other code path in this driver ever runs again.
 * Nothing would notice the failure without an independent timer, which is
 * why this poll exists rather than relying on the error counters alone.
 */
static void pmw3610_health_check(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pmw3610_data *data = CONTAINER_OF(dwork, struct pmw3610_data, health_work);
    const struct device *dev = data->dev;

    if (!data->ready) {
        /* Init is in progress and has its own retry schedule. */
        goto reschedule;
    }

    uint8_t product_id = 0;
    int err = reg_read(dev, PMW3610_REG_PRODUCT_ID, &product_id);

    if (!err && product_id == PMW3610_PRODUCT_ID) {
        /* The sensor is answering, but that only clears single-register
         * reads. Confirm the burst is still aligned too, and only while the
         * ball is idle, since the compared values move when it is not. */
        data->health_fails = 0;

        if ((k_uptime_get() - data->last_report_time) < 1000) {
            goto reschedule;
        }

        int framing = check_burst_framing(dev);

        if (framing == 0) {
            data->framing_fails = 0;
            goto reschedule;
        }

        if (framing != -EILSEQ) {
            /* The check could not complete. That says nothing about the
             * framing either way, so leave the evidence as it stands. */
            goto reschedule;
        }

        /* The comparison is exact, and the sensor keeps sampling between the
         * burst and the register reads that follow it, so a shutter or SQUAL
         * value that wobbles by one count in that gap looks identical to a
         * frame slip. A real slip does not repair itself, so make it prove
         * itself on a second poll rather than spending a reinitialization --
         * and the cursor -- on a stale byte. */
        if (++data->framing_fails < 2) {
            goto reschedule;
        }

        data->framing_fails = 0;
        pmw3610_restart(dev, "motion burst lost byte alignment");
        goto reschedule;
    }

    if (!err) {
        /* The bus answered, but with the wrong byte: the sensor's framing has
         * shifted and every subsequent read would be garbage. No point
         * confirming this one -- it does not resolve on its own. */
        data->health_fails = 0;
        pmw3610_restart(dev, "health check read a corrupt product id");
        goto reschedule;
    }

    /* A failed transfer on its own may just be a glitch that the motion path
     * would have shrugged off, so wait for a second opinion before forcing
     * everything offline for a reinitialization. */
    if (++data->health_fails >= 2) {
        data->health_fails = 0;
        pmw3610_restart(dev, "health check could not reach the sensor");
    }

reschedule:
    k_work_reschedule(&data->health_work, K_MSEC(CONFIG_PMW3610_HEALTH_POLL_INTERVAL_MS));
}
#endif

/* ------------------------------------------------------------------ */
/* Automouse layer                                                     */
/* ------------------------------------------------------------------ */

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))

#if AUTOMOUSE_LAYER > 0
static bool automouse_triggered;

static void deactivate_automouse_layer(struct k_timer *timer) {
    automouse_triggered = false;
    zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
}

K_TIMER_DEFINE(automouse_layer_timer, deactivate_automouse_layer, NULL);

static void activate_automouse_layer(void) {
    if (!automouse_triggered) {
        zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
        automouse_triggered = true;
    }

    k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_PMW3610_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
}
#endif

/* ------------------------------------------------------------------ */
/* Reporting                                                           */
/* ------------------------------------------------------------------ */

static enum pmw3610_input_mode get_input_mode_for_current_layer(const struct device *dev) {
    const struct pmw3610_config *config = dev->config;
    uint8_t curr_layer = zmk_keymap_highest_layer_active();

    for (size_t i = 0; i < config->scroll_layers_len; i++) {
        if (curr_layer == config->scroll_layers[i]) {
            return PMW3610_MODE_SCROLL;
        }
    }

    for (size_t i = 0; i < config->snipe_layers_len; i++) {
        if (curr_layer == config->snipe_layers[i]) {
            return PMW3610_MODE_SNIPE;
        }
    }

    return PMW3610_MODE_MOVE;
}

static void apply_orientation(int16_t raw_x, int16_t raw_y, int16_t *out_x, int16_t *out_y) {
    int16_t x, y;

    if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_90)) {
        x = raw_y;
        y = -raw_x;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_180)) {
        x = raw_x;
        y = -raw_y;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_270)) {
        x = -raw_y;
        y = raw_x;
    } else {
        x = -raw_x;
        y = raw_y;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_X)) {
        x = -x;
    }
    if (IS_ENABLED(CONFIG_PMW3610_INVERT_Y)) {
        y = -y;
    }

    *out_x = x;
    *out_y = y;
}

static int pmw3610_report_data(const struct device *dev) {
    struct pmw3610_data *data = dev->data;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        return -EBUSY;
    }

    enum pmw3610_input_mode input_mode = get_input_mode_for_current_layer(dev);

    if (data->curr_mode != input_mode) {
        /* Partial scroll accumulation from the previous mode would emit a
         * spurious tick on the first movement after switching. */
        data->scroll_delta_x = 0;
        data->scroll_delta_y = 0;
        data->curr_mode = input_mode;
    }

    /* CPI is applied in hardware, so slow movement is preserved exactly
     * rather than being divided away in software. */
    int err = set_cpi_if_needed(dev, input_mode == PMW3610_MODE_SNIPE ? CONFIG_PMW3610_SNIPE_CPI
                                                                     : CONFIG_PMW3610_CPI);
    if (err) {
        /* Unlike a failed read, this leaves the sensor's register page in an
         * unknown state, and reads taken from the wrong page come back as
         * plausible-looking nonsense rather than as errors. Reinitialize now
         * instead of waiting for the error count to build. */
        pmw3610_restart(dev, "CPI update failed, register page is uncertain");
        return err;
    }

    err = motion_burst_read(dev, buf, sizeof(buf));
    if (err) {
        bus_error(dev);
        return err;
    }

    bus_ok(dev);

    int16_t raw_x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t raw_y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

    /* A burst corrupted by a radio glitch still decodes as a well-formed
     * 12-bit delta -- there is no error to catch -- and the host renders it
     * as the cursor teleporting to a corner. The only thing separating it
     * from real data is magnitude, since a hand cannot move the ball that
     * far in one 4 ms sample. Drop the whole report rather than clamp it: a
     * clamped garbage delta is still a jump, just a shorter one. */
#if CONFIG_PMW3610_MAX_DELTA > 0
    if (abs(raw_x) > CONFIG_PMW3610_MAX_DELTA || abs(raw_y) > CONFIG_PMW3610_MAX_DELTA) {
        LOG_WRN("Discarding implausible motion delta (%d, %d)", raw_x, raw_y);

        /* One of these is a glitch. A run of them means the burst is framed
         * wrong and every future read is garbage too, which only a
         * reinitialization fixes. */
        if (++data->implausible_reports >= CONFIG_PMW3610_MAX_CONSECUTIVE_ERRORS) {
            pmw3610_restart(dev, "repeated implausible motion deltas");
        }
        return 0;
    }

    /* Decay rather than clear. A misframed burst still decodes as a small
     * delta much of the time -- whatever byte lands in the high nibble is
     * often near zero -- so clearing on every plausible report would let a
     * persistently broken frame stay below the threshold indefinitely. */
    if (data->implausible_reports > 0) {
        data->implausible_reports--;
    }
#endif

    data->last_report_time = k_uptime_get();

    int16_t x, y;
    apply_orientation(raw_x, raw_y, &x, &y);

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    int16_t shutter =
        ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) + buf[PMW3610_SHUTTER_L_POS];

    bool smart_wanted = data->sw_smart_flag;

    if (shutter < PMW3610_SMART_SHUTTER_THRESHOLD) {
        smart_wanted = false;
    } else if (shutter > PMW3610_SMART_SHUTTER_THRESHOLD) {
        smart_wanted = true;
    }

    if (smart_wanted != data->sw_smart_flag) {
        /* Cache the new state only once the sensor has actually taken it.
         * This is the one register write left in the report path, and its
         * result was previously discarded: a failure would leave the driver
         * believing a mode the sensor is not in, and it would not try again
         * until the shutter crossed the threshold from the other side. */
        int smart_err = reg_write(dev, PMW3610_REG_SMART_MODE, smart_wanted ? 0x80 : 0x00);
        if (smart_err) {
            LOG_WRN("Smart mode write failed (%d), retrying on the next report", smart_err);
        } else {
            data->sw_smart_flag = smart_wanted;
        }
    }
#endif

#ifdef CONFIG_PMW3610_POLLING_RATE_125_SW
    /* The sensor runs at 250 Hz; hold every other sample back and report the
     * pair summed, which halves the report rate without discarding motion.
     * A stale held sample is dropped rather than added to a much later one. */
    int64_t curr_time = k_uptime_get();

    if (data->last_poll_time == 0 || (curr_time - data->last_poll_time) > 128) {
        data->last_poll_time = curr_time;
        data->last_x = x;
        data->last_y = y;
        return 0;
    }

    x += data->last_x;
    y += data->last_y;
    data->last_poll_time = 0;
    data->last_x = 0;
    data->last_y = 0;
#endif

    if (x == 0 && y == 0) {
        return 0;
    }

    switch (input_mode) {
    case PMW3610_MODE_MOVE:
#if AUTOMOUSE_LAYER > 0
        if ((abs(x) + abs(y)) > CONFIG_PMW3610_MOVEMENT_THRESHOLD) {
            activate_automouse_layer();
        }
#endif
        __fallthrough;
    case PMW3610_MODE_SNIPE:
        input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
        break;

    case PMW3610_MODE_SCROLL:
        data->scroll_delta_x += x;
        data->scroll_delta_y += y;

        if (abs(data->scroll_delta_y) > CONFIG_PMW3610_SCROLL_TICK) {
            input_report_rel(dev, INPUT_REL_WHEEL,
                             data->scroll_delta_y > 0 ? PMW3610_SCROLL_Y_NEGATIVE
                                                      : PMW3610_SCROLL_Y_POSITIVE,
                             true, K_FOREVER);
            data->scroll_delta_x = 0;
            data->scroll_delta_y = 0;
        } else if (abs(data->scroll_delta_x) > CONFIG_PMW3610_SCROLL_TICK) {
            input_report_rel(dev, INPUT_REL_HWHEEL,
                             data->scroll_delta_x > 0 ? PMW3610_SCROLL_X_NEGATIVE
                                                      : PMW3610_SCROLL_X_POSITIVE,
                             true, K_FOREVER);
            data->scroll_delta_x = 0;
            data->scroll_delta_y = 0;
        }
        break;
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* Interrupt plumbing                                                  */
/* ------------------------------------------------------------------ */

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pmw3610_data *data = CONTAINER_OF(cb, struct pmw3610_data, irq_gpio_cb);

    /* The motion pin is level-triggered, so it must be masked until the burst
     * read below clears the sensor's motion flag. */
    set_interrupt(data->dev, false);
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pmw3610_data *data = CONTAINER_OF(work, struct pmw3610_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3610_report_data(dev);

    /* A restart may have run during reporting; it re-enables the interrupt
     * itself once the sensor is back, and re-enabling here would let a stuck
     * motion pin spin the workqueue in the meantime. */
    if (data->ready) {
        set_interrupt(dev, true);
    }
}

static int pmw3610_init_irq(const struct device *dev) {
    struct pmw3610_data *data = dev->data;
    const struct pmw3610_config *config = dev->config;
    int err;

    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO (%d)", err);
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback (%d)", err);
    }

    return err;
}

static int pmw3610_init(const struct device *dev) {
    struct pmw3610_data *data = dev->data;
    const struct pmw3610_config *config = dev->config;
    int err;

    data->dev = dev;
    data->sw_smart_flag = false;
    data->ready = false;
    data->curr_mode = PMW3610_MODE_MOVE;
    data->curr_cpi = 0;

    k_work_init(&data->trigger_work, pmw3610_work_callback);

    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO (%d)", err);
        return err;
    }

    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    k_work_init_delayable(&data->init_work, pmw3610_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[ASYNC_INIT_STEP_POWER_UP]));

#if CONFIG_PMW3610_HEALTH_POLL_INTERVAL_MS > 0
    k_work_init_delayable(&data->health_work, pmw3610_health_check);
    k_work_schedule(&data->health_work, K_MSEC(CONFIG_PMW3610_HEALTH_POLL_INTERVAL_MS));
#endif

    return 0;
}

/*
 * Emit a layer list with a trailing sentinel. Either list is allowed to be
 * empty, and a zero-length array initializer is not valid C -- the sentinel
 * keeps the array well formed. It is never read: the matching loops are
 * bounded by DT_PROP_LEN, which does not count it.
 */
#define PMW3610_LAYER_ELEM(node_id, prop, idx) DT_PROP_BY_IDX(node_id, prop, idx),

#define PMW3610_LAYER_ARRAY(n, prop)                                                               \
    {DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), prop, PMW3610_LAYER_ELEM) - 1}

#define PMW3610_DEFINE(n)                                                                          \
    static struct pmw3610_data data##n;                                                            \
                                                                                                   \
    static const int32_t scroll_layers##n[] = PMW3610_LAYER_ARRAY(n, scroll_layers);               \
    static const int32_t snipe_layers##n[] = PMW3610_LAYER_ARRAY(n, snipe_layers);                 \
                                                                                                   \
    static const struct pmw3610_config config##n = {                                               \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .bus =                                                                                     \
            {                                                                                      \
                .bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
                .config =                                                                          \
                    {                                                                              \
                        .frequency = DT_INST_PROP(n, spi_max_frequency),                           \
                        .operation =                                                               \
                            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,    \
                        .slave = DT_INST_REG_ADDR(n),                                              \
                    },                                                                             \
            },                                                                                     \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .scroll_layers = scroll_layers##n,                                                         \
        .scroll_layers_len = DT_PROP_LEN(DT_DRV_INST(n), scroll_layers),                           \
        .snipe_layers = snipe_layers##n,                                                           \
        .snipe_layers_len = DT_PROP_LEN(DT_DRV_INST(n), snipe_layers),                             \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)
