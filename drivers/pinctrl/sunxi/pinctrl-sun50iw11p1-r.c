/*
 * Allwinner sun50iw10p1 SoCs R_PIO pinctrl driver.
 *
 * Copyright(c) 2012-2016 Allwinnertech Co., Ltd.
 * Author: huanghuafeng <huafenghuang@allwinnertech.com>
 *
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include "pinctrl-sunxi.h"

static const struct sunxi_desc_pin sun50iw11p1_r_pins[] = {
	//Register Name: PL_CFG0
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_i2s0"),		/* LRCK */
		SUNXI_FUNCTION(0x4, "s_dmic"),		/* DATA3 */
		SUNXI_FUNCTION(0x5, "s_pwm0"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 0),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_i2s0"),		/* BCLK */
		SUNXI_FUNCTION(0x4, "s_dmic"),		/* DATA2 */
		SUNXI_FUNCTION(0x5, "s_pwm1"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 1),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_i2s0"),		/* DOUT0 */
		SUNXI_FUNCTION(0x3, "s_i2s0r"),		/* DIN1 */
		SUNXI_FUNCTION(0x4, "s_dmic"),		/* DATA1 */
		SUNXI_FUNCTION(0x5, "s_pwm2"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 2),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_i2s0r"),		/* DOUT1 */
		SUNXI_FUNCTION(0x3, "s_i2s0"),		/* DIN0 */
		SUNXI_FUNCTION(0x4, "s_dmic"),		/* DATA0 */
		SUNXI_FUNCTION(0x5, "s_twi0"),		/* SDA */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 3),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 4),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_i2s0"),		/* MCLK */
		SUNXI_FUNCTION(0x3, "s_ir"),		/* RX */
		SUNXI_FUNCTION(0x4, "s_dmic"),		/* CLK */
		SUNXI_FUNCTION(0x5, "s_twi0"),		/* SCK */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 4),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 5),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_twi0"),		/* SDA */
		SUNXI_FUNCTION(0x5, "s_pwm3"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 5),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 6),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_twi0"),		/* SCK */
		SUNXI_FUNCTION(0x5, "s_pwm4"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 6),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 7),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_ir"),		/* RX */
		SUNXI_FUNCTION(0x4, "x32kfout"),	/* 32kFOUT */
		SUNXI_FUNCTION(0x5, "s_pwm5"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 7),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 8),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_uart0"),		/* TX */
		SUNXI_FUNCTION(0x3, "s_twi0"),		/* SDA */
		SUNXI_FUNCTION(0x4, "s_ir"),		/* RX */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 8),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 9),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_uart0"),		/* RX */
		SUNXI_FUNCTION(0x3, "s_twi0"),		/* SCK */
		SUNXI_FUNCTION(0x4, "x32kfout"),	/* 32kFOUT */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 9),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 10),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 0, 10),
		SUNXI_FUNCTION(0x7, "io_disabled")),

	//PM CFG
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_uart0"),		/* TX */
		SUNXI_FUNCTION(0x3, "s_jtag0"),		/* MS */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 0),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_uart0"),		/* RX */
		SUNXI_FUNCTION(0x3, "s_jtag0"),		/* CK */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 1),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x3, "s_jtag0"),		/* DO */
		SUNXI_FUNCTION(0x4, "s_twi0"),		/* SDA */
		SUNXI_FUNCTION(0x5, "s_ir"),		/* RX */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 2),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_twi0"),		/* SDA */
		SUNXI_FUNCTION(0x3, "s_ir"),		/* RX */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 3),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 4),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_twi0"),		/* SCK */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 4),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 5),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "x32kfout"),	/* 32KFOUT */
		SUNXI_FUNCTION(0x3, "s_jtag0"),		/* DI */
		SUNXI_FUNCTION(0x4, "s_twi0"),		/* SCK */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 5),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 6),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "nmi"),
		SUNXI_FUNCTION(0x3, "s_ir"),		/* RX */
		SUNXI_FUNCTION(0x4, "x32kfout"),	/* 32KFOUT */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 6),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 7),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "s_ir"),		/* RX */
		SUNXI_FUNCTION(0x3, "32kfout"),		/* 32KFOUT */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 7),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(M, 8),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 1, 8),
		SUNXI_FUNCTION(0x7, "io_disabled")),

	//PN CFG
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 0),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* MDC */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 1),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* MDIO */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 2),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* RXD3 */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 3),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 4),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* TXCK */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 4),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 5),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* RXCTL */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 5),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 6),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* NULL */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 6),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 7),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* TXD3 */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 7),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 8),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* TXD2 */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 8),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 9),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* TXD1 */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 9),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 10),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* TXD0 */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 10),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 11),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* TXCK */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 11),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 12),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* TXCTL */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 12),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 13),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),		/* CLKIN */
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 13),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 14),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 14),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 15),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION(0x3, "gmac0"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 15),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 16),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "pscs"),
		SUNXI_FUNCTION(0x3, "gmac0"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 16),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 17),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psck"),
		SUNXI_FUNCTION(0x3, "gmac0"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 17),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 18),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psckb"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 18),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 19),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdm0"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 19),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 20),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdm1"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 20),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 21),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 21),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 22),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION(0x2, "psdq"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 22),
		SUNXI_FUNCTION(0x7, "io_disabled")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(N, 23),
		SUNXI_FUNCTION(0x0, "gpio_in"),
		SUNXI_FUNCTION(0x1, "gpio_out"),
		SUNXI_FUNCTION_IRQ_BANK(0x6, 2, 23),
		SUNXI_FUNCTION(0x7, "io_disabled")),
};

static const unsigned sun50iw11p1_r_bank_base[] = {
	SUNXI_R_PIO_BANK_BASE(PL_BASE, 0),
	SUNXI_R_PIO_BANK_BASE(PM_BASE, 1),
	SUNXI_R_PIO_BANK_BASE(PN_BASE, 2),
};
static const unsigned sun50iw11p1_r_irq_bank_base[] = {
	SUNXI_R_PIO_BANK_BASE(PL_BASE, 0),
	SUNXI_R_PIO_BANK_BASE(PM_BASE, 1),
	SUNXI_R_PIO_BANK_BASE(PN_BASE, 2),
};

static const struct sunxi_pinctrl_desc sun50iw11p1_r_pinctrl_data = {
	.pins = sun50iw11p1_r_pins,
	.npins = ARRAY_SIZE(sun50iw11p1_r_pins),
	.pin_base = PL_BASE,
	.banks = ARRAY_SIZE(sun50iw11p1_r_bank_base),
	.bank_base = sun50iw11p1_r_bank_base,
	.irq_banks = ARRAY_SIZE(sun50iw11p1_r_irq_bank_base),
	.irq_bank_base = sun50iw11p1_r_irq_bank_base,
};

static int sun50iw11p1_r_pinctrl_probe(struct platform_device *pdev)
{
	return sunxi_pinctrl_init(pdev, &sun50iw11p1_r_pinctrl_data);
}

static struct of_device_id sun50iw11p1_r_pinctrl_match[] = {
	{ .compatible = "allwinner,sun50iw11p1-r-pinctrl", },
	{}
};
MODULE_DEVICE_TABLE(of, sun50iw11p1_r_pinctrl_match);

static struct platform_driver sun50iw11p1_r_pinctrl_driver = {
	.probe	= sun50iw11p1_r_pinctrl_probe,
	.driver	= {
		.name		= "sun50iw11p1-r-pinctrl",
		.owner		= THIS_MODULE,
		.pm		= &sunxi_pinctrl_pm_ops,
		.of_match_table	= sun50iw11p1_r_pinctrl_match,
	},
};

static int __init sun50iw11p1_r_pio_init(void)
{
	int ret;
	ret = platform_driver_register(&sun50iw11p1_r_pinctrl_driver);
	if (ret) {
		pr_debug("register sun50i r-pio controller failed\n");
		return -EINVAL;
	}
	return 0;
}
postcore_initcall(sun50iw11p1_r_pio_init);

MODULE_AUTHOR("Huanghuafeng<huafenghuang@allwinnertech.com>");
MODULE_DESCRIPTION("Allwinner sun50iw11p1 R_PIO pinctrl driver");
MODULE_LICENSE("GPL");
