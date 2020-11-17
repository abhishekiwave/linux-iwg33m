// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */

#include <common.h>
#include <env.h>
#include <init.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/tcpc.h"
#include "../common/pfuze.h"
#include <usb.h>
#include <dwc3-uboot.h>

#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

#define GPIO_PAD_CTRL	(PAD_CTL_DSE1)

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const led_rst_pad[] = {
        IMX8MQ_PAD_GPIO1_IO03__GPIO1_IO3 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static iomux_v3_cfg_t const usb_vbus_pads[] = {
        IMX8MQ_PAD_SAI5_RXFS__GPIO3_IO19 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static iomux_v3_cfg_t const typec_ss_pads[] = {
        IMX8MQ_PAD_GPIO1_IO10__GPIO1_IO10 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static iomux_v3_cfg_t const som_revision_pads[] = {
	IMX8MQ_PAD_NAND_CE1_B__GPIO3_IO2 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	IMX8MQ_PAD_SAI3_RXFS__GPIO4_IO28 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	IMX8MQ_PAD_SAI3_RXC__GPIO4_IO29	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	IMX8MQ_PAD_SAI3_MCLK__GPIO5_IO2	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	IMX8MQ_PAD_SAI2_RXC__GPIO4_IO22	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	IMX8MQ_PAD_SAI2_RXFS__GPIO4_IO21 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	IMX8MQ_PAD_NAND_DQS__GPIO3_IO14	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static int led_reset(void)
{
	struct gpio_desc led_reset_desc;
	int ret;

	imx_iomux_v3_setup_multiple_pads(led_rst_pad, ARRAY_SIZE(led_rst_pad));

	ret = dm_gpio_lookup_name("GPIO1_3", &led_reset_desc);
	if (ret) {
		printf("%s lookup GPIO1_3 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&led_reset_desc, "led_reset");
	if (ret) {
		printf("%s request led_reset gpio failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&led_reset_desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&led_reset_desc, 1);
        mdelay(200);
        dm_gpio_set_value(&led_reset_desc, 1);
	mdelay(10);
	return ret;
}

struct gpio_desc usb_vbus_desc;

static int init_usb_vbus(void)
{
	int ret;
	imx_iomux_v3_setup_multiple_pads(usb_vbus_pads, ARRAY_SIZE(usb_vbus_pads));
#ifdef CONFIG_DM_GPIO
	ret = dm_gpio_lookup_name("GPIO3_19", &usb_vbus_desc);
	if (ret) {
		printf("%s lookup GPIO3_19 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&usb_vbus_desc, "usb_vbus");
	if (ret) {
		printf("%s request usb_vbus gpio failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&usb_vbus_desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&usb_vbus_desc, 0);
#else
	gpio_request(IMX_GPIO_NR(3, 19), "usb_vbus");
	gpio_direction_output(IMX_GPIO_NR(3, 19), 0);
#endif
	mdelay(10);
	return ret;
}

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));
	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	return 0;
}

#ifdef CONFIG_FSL_QSPI
int board_qspi_init(void)
{
	set_clk_qspi();

	return 0;
}
#endif

#ifdef CONFIG_MXC_SPI
#define SPI_PAD_CTRL	(PAD_CTL_DSE2 | PAD_CTL_HYS)
static iomux_v3_cfg_t const ecspi1_pads[] = {
	IMX8MQ_PAD_ECSPI1_SCLK__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	IMX8MQ_PAD_ECSPI1_MOSI__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	IMX8MQ_PAD_ECSPI1_MISO__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	IMX8MQ_PAD_ECSPI1_SS0__GPIO5_IO9 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	//gpio_request(IMX_GPIO_NR(5, 9), "ECSPI1 CS");

	init_clk_ecspi(0);
}

#endif


#ifdef CONFIG_FEC_MXC
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1],
		IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK, 0);
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef CONFIG_USB_DWC3

#define USB_PHY_CTRL0			0xF0040
#define USB_PHY_CTRL0_REF_SSP_EN	BIT(2)

#define USB_PHY_CTRL1			0xF0044
#define USB_PHY_CTRL1_RESET		BIT(0)
#define USB_PHY_CTRL1_COMMONONN		BIT(1)
#define USB_PHY_CTRL1_ATERESET		BIT(3)
#define USB_PHY_CTRL1_VDATSRCENB0	BIT(19)
#define USB_PHY_CTRL1_VDATDETENB0	BIT(20)

#define USB_PHY_CTRL2			0xF0048
#define USB_PHY_CTRL2_TXENABLEN0	BIT(8)

static struct dwc3_device dwc3_device_data = {
#ifdef CONFIG_SPL_BUILD
	.maximum_speed = USB_SPEED_HIGH,
#else
	.maximum_speed = USB_SPEED_SUPER,
#endif
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

int usb_gadget_handle_interrupts(void)
{
	dwc3_uboot_handle_interrupt(0);
	return 0;
}

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 RegData;

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_VDATSRCENB0 | USB_PHY_CTRL1_VDATDETENB0 |
			USB_PHY_CTRL1_COMMONONN);
	RegData |= USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET;
	writel(RegData, dwc3->base + USB_PHY_CTRL1);

	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData |= USB_PHY_CTRL0_REF_SSP_EN;
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL2);
	RegData |= USB_PHY_CTRL2_TXENABLEN0;
	writel(RegData, dwc3->base + USB_PHY_CTRL2);

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET);
	writel(RegData, dwc3->base + USB_PHY_CTRL1);
}
#endif

#ifdef CONFIG_FUSB302
struct gpio_desc typec_sel_desc;

void ss_mux_select(enum typec_cc_polarity pol)
{
	if (pol == TYPEC_POLARITY_CC1)
		dm_gpio_set_value(&typec_sel_desc, 0);
	else
		dm_gpio_set_value(&typec_sel_desc, 1);
}

static int setup_typec(void)
{
	int ret;

	imx_iomux_v3_setup_multiple_pads(typec_ss_pads, ARRAY_SIZE(typec_ss_pads));

	ret = dm_gpio_lookup_name("GPIO1_10", &typec_sel_desc);
	if (ret) {
		printf("%s lookup GPIO1_10 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&typec_sel_desc, "typec_sel");
	if (ret) {
		printf("%s request typec_sel failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&typec_sel_desc, GPIOD_IS_OUT);

	return ret;
}
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;
	uint8_t vbus_high = 0;

#ifdef CONFIG_FUSB302
	struct udevice *bus;
	struct udevice **i2c_dev;
	uint8_t const value[3] = {0x07, 0x02, 0x03};
	uint8_t buf[1];
	uint8_t tog_ss = 0;

	uclass_get_device_by_seq(UCLASS_I2C, 1, &bus);
	i2c_get_chip(bus, 0x22, 1, i2c_dev);
	dm_i2c_write(*i2c_dev, 0x0B, value, 1);
	dm_i2c_write(*i2c_dev, 0x08, value+1, 1);
	dm_i2c_write(*i2c_dev, 0x08, value+2, 1);
	mdelay(100);
	dm_i2c_read(*i2c_dev, 0x3d, buf, 1);
	tog_ss = (buf[0] & 0x38) >> 3;
	dm_i2c_read(*i2c_dev, 0x40, buf, 1);
	vbus_high = (buf[0] & 0x80) >> 7;
	if (tog_ss % 2)
		ss_mux_select(TYPEC_POLARITY_CC1);
	else
		ss_mux_select(TYPEC_POLARITY_CC2);
#endif

	if (index == 0 && init == USB_INIT_DEVICE) {
		imx8m_usb_power(index, true);
#ifdef CONFIG_DM_GPIO
		dm_gpio_set_value(&usb_vbus_desc, 0);
#else
		gpio_direction_output(IMX_GPIO_NR(3, 19), 0);
#endif
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
		/*Since vbus is shared between type-c port and usb 3.0 upper port,
		 *don't drive the vbus if host connected to type-c port
		 *is already driving the vbus high.
		 */
		if (!vbus_high)
#ifdef CONFIG_DM_GPIO
			dm_gpio_set_value(&usb_vbus_desc, 1);
#else
			gpio_direction_output(IMX_GPIO_NR(3, 19), 1);
#endif
		return ret;
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_uboot_exit(index);
		imx8m_usb_power(index, false);
	} else if (index == 0 && init == USB_INIT_HOST) {
#ifdef CONFIG_DM_GPIO
		dm_gpio_set_value(&usb_vbus_desc, 0);
#else
		gpio_direction_output(IMX_GPIO_NR(3, 19), 0);
#endif
	}
	return ret;
}
#endif

static struct gpio_desc revision_desc[7];
static char *revision_gpios[] = {
	"GPIO3_2",
	"GPIO4_28",
	"GPIO4_29",
	"GPIO5_2",
	"GPIO4_22",
	"GPIO4_21",
	"GPIO3_14",
};

static int setup_som_revision(void)
{
	int ret;
	int i;

	imx_iomux_v3_setup_multiple_pads(som_revision_pads, ARRAY_SIZE(som_revision_pads));

	for (i = 0; i < ARRAY_SIZE(som_revision_pads); i++) {
		ret = dm_gpio_lookup_name(revision_gpios[i], &revision_desc[i]);
		if (ret) {
			printf("%s lookup %s failed ret = %d\n", __func__, revision_gpios[i], ret);
			return -ENODEV;
		}

		ret = dm_gpio_request(&revision_desc[i], revision_gpios[i]);
		if (ret) {
			printf("%s request %s failed ret = %d\n", __func__, revision_gpios[i], ret);
			return -ENODEV;
		}

		dm_gpio_set_dir_flags(&revision_desc[i], GPIOD_IS_IN);
	}
	return ret;
}

int board_init(void)
{
#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	init_usb_clk();
	init_usb_vbus();
#endif

#ifdef CONFIG_FUSB302
	setup_typec();
#endif
	led_reset();
	setup_som_revision();
	return 0;
}

#define BSP_VERSION "iW-PRFSZ-SC-01-R3.0-REL1.0a-Linux5.4.24"

static void printboard_info(void)
{
        int i;
	int pcb_rev = 0;
	int bom_rev = 0;

        for (i=0;i<ARRAY_SIZE(som_revision_pads);i++)
        {
                if(i<=3)
                {
                        pcb_rev |= (dm_gpio_get_value(&revision_desc[i]) << i);
                }
                else
                {
                        bom_rev |= (dm_gpio_get_value(&revision_desc[i]) << (i-4));
                }
        }

        printf ("\n");
        printf ("Board Info:\n");
        printf ("\tBSP Version     : %s\n", BSP_VERSION);
        printf ("\tSOM Version     : iW-PRFSZ-AP-01-R%x.%x\n", bom_rev+1, pcb_rev);
        printf ("\n");
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "IWG33M");
	env_set("board_rev", "iMX8M");
#endif
	printboard_info();

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

