/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <driver/gpio.h>
#include <driver/dedic_gpio.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include "hal/gpio_ll.h"
#include "esp_io.h"
#include "compiler.h"
#include "sdkconfig.h"

extern dedic_gpio_bundle_handle_t dedic_gpio_out_bundle;
extern dedic_gpio_bundle_handle_t dedic_gpio_io_bundle;
extern gpio_dev_t *const dedic_gpio_dev;
extern uint32_t dedic_gpio_conf;

/**
 * @brief Initialize debug GPIO pins for JTAG mode
 */
void debug_probe_init_jtag_pins(void);

/**
 * @brief Initialize debug GPIO pins for SWD mode
 */
void debug_probe_init_swd_pins(void);

/**
 * @brief Reset and cleanup debug GPIO pins
 */
void debug_probe_reset_pins(void);

/**
 * @brief Notify debug activity
 * @param active true when debug activity is happening, false when idle
 */
void debug_probe_notify_activity(bool active);

/* High-performance inline GPIO functions for debug operations */

__STATIC_FORCEINLINE void debug_probe_swdio_out_enable(void)
{
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (GPIO_SWDIO * 4), dedic_gpio_conf);
    gpio_ll_output_enable(dedic_gpio_dev, GPIO_SWDIO);
}

__STATIC_FORCEINLINE void debug_probe_swdio_out_disable(void)
{
    gpio_ll_output_disable(dedic_gpio_dev, GPIO_SWDIO);
}

__STATIC_FORCEINLINE int debug_probe_swdio_read(void)
{
    return dedic_gpio_cpu_ll_read_in();
}

__STATIC_FORCEINLINE void debug_probe_swclk_set(void)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_SWCLK_MASK, GPIO_SWCLK_MASK);
}

__STATIC_FORCEINLINE void debug_probe_swclk_clr(void)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_SWCLK_MASK, 0);
}

__STATIC_FORCEINLINE void debug_probe_swdio_set(void)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_SWDIO_OUT_MASK, GPIO_SWDIO_OUT_MASK);
}

__STATIC_FORCEINLINE void debug_probe_swdio_clr(void)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_SWDIO_OUT_MASK, 0);
}

__STATIC_FORCEINLINE void debug_probe_swdio_write(int val)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_SWDIO_OUT_MASK, (val & 0x01) ? GPIO_SWDIO_OUT_MASK : 0);
}

__STATIC_FORCEINLINE void debug_probe_swd_blink(int on)
{
    gpio_ll_set_level(s_gpio_dev, LED_JTAG, (on & 0x01) ? LED_JTAG_ON : LED_JTAG_OFF);
}

__STATIC_FORCEINLINE void debug_probe_jtag_led_off(void)
{
    gpio_ll_set_level(s_gpio_dev, LED_JTAG, LED_JTAG_OFF);
}

__STATIC_FORCEINLINE void debug_probe_jtag_led_on(void)
{
    gpio_ll_set_level(s_gpio_dev, LED_JTAG, LED_JTAG_ON);
}

__STATIC_FORCEINLINE int debug_probe_tdo_read(void)
{
    return dedic_gpio_cpu_ll_read_in();
}

__STATIC_FORCEINLINE void debug_probe_tdi_write(int val)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_TDI_MASK, (val & 0x01) ? GPIO_TDI_MASK : 0);
}

__STATIC_FORCEINLINE void debug_probe_tck_set(void)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_TCK_MASK, GPIO_TCK_MASK);
}

__STATIC_FORCEINLINE void debug_probe_tck_clr(void)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_TCK_MASK, 0);
}

__STATIC_FORCEINLINE void debug_probe_write_tmstck(uint8_t tms_tdi_mask)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_TMS_TDI_MASK, tms_tdi_mask);
}

/* Basic GPIO mode functions for debug pins */
__STATIC_FORCEINLINE void debug_probe_mode_input_enable(int gpio_num)
{
    gpio_ll_output_disable(dedic_gpio_dev, gpio_num);
    gpio_ll_input_enable(dedic_gpio_dev, gpio_num);
}

__STATIC_FORCEINLINE void debug_probe_mode_out_enable(int gpio_num)
{
    gpio_ll_input_disable(dedic_gpio_dev, gpio_num);
    gpio_ll_output_enable(dedic_gpio_dev, gpio_num);
}

__STATIC_FORCEINLINE void debug_probe_mode_in_out_enable(int gpio_num)
{
    gpio_ll_input_enable(dedic_gpio_dev, gpio_num);
    gpio_ll_output_enable(dedic_gpio_dev, gpio_num);
}

__STATIC_FORCEINLINE void debug_probe_set(int gpio_num)
{
    gpio_ll_set_level(dedic_gpio_dev, gpio_num, 1);
}

__STATIC_FORCEINLINE void debug_probe_clear(int gpio_num)
{
    gpio_ll_set_level(dedic_gpio_dev, gpio_num, 0);
}
