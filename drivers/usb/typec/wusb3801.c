// SPDX-License-Identifier: GPL-2.0
/*
 * Willsemi WUSB3801 Type-C port controller driver
 *
 * Copyright (C) 2022 Samuel Holland <samuel@sholland.org>
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
//#include <linux/usb/typec.h>
#include <linux/gpio/consumer.h>

#define __TEST_CC_PATCH__

#define WUSB3801_REG_DEVICE_ID		0x01
#define WUSB3801_REG_CTRL0		0x02
#define WUSB3801_REG_INT		0x03
#define WUSB3801_REG_STAT		0x04
#define WUSB3801_REG_CTRL1		0x05
#define WUSB3801_REG_TEST00		0x06
#define WUSB3801_REG_TEST01		0x07
#define WUSB3801_REG_TEST02		0x08
#define WUSB3801_REG_TEST03		0x09
#define WUSB3801_REG_TEST04		0x0a
#define WUSB3801_REG_TEST05		0x0b
#define WUSB3801_REG_TEST06		0x0c
#define WUSB3801_REG_TEST07		0x0d
#define WUSB3801_REG_TEST08		0x0e
#define WUSB3801_REG_TEST09		0x0f
#define WUSB3801_REG_TEST0A		0x10
#define WUSB3801_REG_TEST0B		0x11
#define WUSB3801_REG_TEST0C		0x12
#define WUSB3801_REG_TEST0D		0x13
#define WUSB3801_REG_TEST0E		0x14
#define WUSB3801_REG_TEST0F		0x15
#define WUSB3801_REG_TEST10		0x16
#define WUSB3801_REG_TEST11		0x17
#define WUSB3801_REG_TEST12		0x18

#define WUSB3801_DEVICE_ID_VERSION_ID	GENMASK(7, 3)
#define WUSB3801_DEVICE_ID_VENDOR_ID	GENMASK(2, 0)

#define WUSB3801_CTRL0_DIS_ACC_SUPPORT	BIT(7)
#define WUSB3801_CTRL0_TRY		GENMASK(6, 5)
#define WUSB3801_CTRL0_TRY_NONE		(0x0 << 5)
#define WUSB3801_CTRL0_TRY_SNK		(0x1 << 5)
#define WUSB3801_CTRL0_TRY_SRC		(0x2 << 5)
#define WUSB3801_CTRL0_CURRENT		GENMASK(4, 3) /* SRC */
#define WUSB3801_CTRL0_CURRENT_DEFAULT	(0x0 << 3)
#define WUSB3801_CTRL0_CURRENT_1_5A	(0x1 << 3)
#define WUSB3801_CTRL0_CURRENT_3_0A	(0x2 << 3)
#define WUSB3801_CTRL0_ROLE		GENMASK(2, 1)
#define WUSB3801_CTRL0_ROLE_SNK		(0x0 << 1)
#define WUSB3801_CTRL0_ROLE_SRC		(0x1 << 1)
#define WUSB3801_CTRL0_ROLE_DRP		(0x2 << 1)
#define WUSB3801_CTRL0_INT_MASK		BIT(0)

#define WUSB3801_INT_ATTACHED		BIT(0)
#define WUSB3801_INT_DETACHED		BIT(1)

#define WUSB3801_STAT_VBUS_DETECTED	BIT(7)
#define WUSB3801_STAT_CURRENT		GENMASK(6, 5) /* SNK */
#define WUSB3801_STAT_CURRENT_STANDBY	(0x0 << 5)
#define WUSB3801_STAT_CURRENT_DEFAULT	(0x1 << 5)
#define WUSB3801_STAT_CURRENT_1_5A	(0x2 << 5)
#define WUSB3801_STAT_CURRENT_3_0A	(0x3 << 5)
#define WUSB3801_STAT_PARTNER		GENMASK(4, 2)
#define WUSB3801_STAT_PARTNER_STANDBY	(0x0 << 2)
#define WUSB3801_STAT_PARTNER_SNK	(0x1 << 2)
#define WUSB3801_STAT_PARTNER_SRC	(0x2 << 2)
#define WUSB3801_STAT_PARTNER_AUDIO	(0x3 << 2)
#define WUSB3801_STAT_PARTNER_DEBUG	(0x4 << 2)
#define WUSB3801_STAT_ORIENTATION	GENMASK(1, 0)
#define WUSB3801_STAT_ORIENTATION_NONE	(0x0 << 0)
#define WUSB3801_STAT_ORIENTATION_CC1	(0x1 << 0)
#define WUSB3801_STAT_ORIENTATION_CC2	(0x2 << 0)
#define WUSB3801_STAT_ORIENTATION_BOTH	(0x3 << 0)

#define WUSB3801_CTRL1_SM_RESET		BIT(0)

#define WUSB3801_TEST01_VENDOR_SUB_ID	(BIT(7) | BIT(5))

#define WUSB3801_TEST02_FORCE_ERR_RCY	BIT(8)

#define WUSB3801_TEST0A_WAIT_VBUS	BIT(5)

#define WUSB3801_CC2_CONNECTED 1
#define WUSB3801_CC1_CONNECTED 0

struct wusb3801 {
	struct device		*dev;
	struct regmap		*regmap;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct gpio_desc *ch482d_sel_gpio;
	u32 dev_sub_id;
	u32 cc_sts;
};

#ifdef __TEST_CC_PATCH__
static int wusb3801_reg_wr_rd(struct wusb3801 *chip, u8 reg, u8 val)
{
	unsigned int i, rc;
	u8 rc_8;

#define WUSB3801_REG_CTRL_DELAY_MIN (2 * 1000)
#define WUSB3801_REG_CTRL_DELAY_MAX (2 * 1050)
#define WUSB3801_REG_CTRL_CNT 20


	for (i = 0; i < WUSB3801_REG_CTRL_CNT; i++)	{
		regmap_write(chip->regmap, reg, val);
		usleep_range(WUSB3801_REG_CTRL_DELAY_MIN, WUSB3801_REG_CTRL_DELAY_MAX);
		regmap_read(chip->regmap, reg, &rc);
		rc_8 = (u8)rc;
		if (!(rc_8 ^ val)) {
			return 0;
		}
	}

	return ((i == WUSB3801_REG_CTRL_CNT)? 1 : 0);
}

static int test_cc_patch(struct wusb3801 *chip)
{
	unsigned int rc = 0, rc_reg_08 = 0, int_stat = 0;

#define WUSB3801_SET08_ENTR_RCVRY_MODE 0x82
#define WUSB3801_SET0F_ENTR_RCVRY_MODE 0xC0
#define WUSB3801_SET08_EXIT_RCVRY_MODE 0x80
#define WUSB3801_SET_EXIT_RCVRY_MODE 0x00
#define WUSB3801_WAIT_ATTACH_MS 80

	rc = wusb3801_reg_wr_rd(chip, WUSB3801_REG_TEST02, WUSB3801_SET08_ENTR_RCVRY_MODE);
	if (rc) {
		printk("%s write & read WUSB3801_REG_TEST02 fail \n", __func__);
		return (chip->cc_sts);
	}
	rc = wusb3801_reg_wr_rd(chip, WUSB3801_REG_TEST09, WUSB3801_SET0F_ENTR_RCVRY_MODE);
	if (rc) {
		printk("%s write & read WUSB3801_REG_TEST09 fail \n", __func__);
		return(chip->cc_sts);
	}
	regmap_read(chip->regmap, WUSB3801_REG_TEST00, &rc_reg_08);		//get orientation status
	rc = wusb3801_reg_wr_rd(chip, WUSB3801_REG_TEST09, WUSB3801_SET_EXIT_RCVRY_MODE);
	if (rc) {
		printk("%s write & read WUSB3801_REG_TEST09 exit fail \n", __func__);
		return (chip->cc_sts);
	}
	rc = wusb3801_reg_wr_rd(chip, WUSB3801_REG_TEST02, WUSB3801_SET08_EXIT_RCVRY_MODE);
	if (rc) {
		printk("%s write & read WUSB3801_REG_TEST02 exit fail \n", __func__);
		return (chip->cc_sts);
	}
	rc = wusb3801_reg_wr_rd(chip, WUSB3801_REG_TEST02, WUSB3801_SET_EXIT_RCVRY_MODE);
	if (rc) {
		printk("%s write & read WUSB3801_REG_TEST02 exit 0 fail \n", __func__);
		return (chip->cc_sts);
	}
	rc = wusb3801_reg_wr_rd(chip, WUSB3801_REG_TEST09, WUSB3801_SET_EXIT_RCVRY_MODE);
	if (rc) {
		printk("%s write & read WUSB3801_REG_TEST09 exit 0 fail \n", __func__);
		return (chip->cc_sts);
	}
	msleep(WUSB3801_WAIT_ATTACH_MS);
	regmap_read(chip->regmap, WUSB3801_REG_INT, &int_stat);
	return ((rc_reg_08 >> 6) & 1);
}
#endif /* __TEST_CC_PATCH__ */

static int wusb3801_hw_init(struct wusb3801 *wusb3801)
{
	// return regmap_write(wusb3801->regmap, WUSB3801_REG_CTRL0,
	// 		    wusb3801_map_try_role(wusb3801->cap.prefer_role) |
	// 		    wusb3801_map_pwr_opmode(wusb3801->pwr_opmode) |
	// 		    wusb3801_map_port_type(wusb3801->port_type));

	return regmap_write(wusb3801->regmap, WUSB3801_REG_CTRL0,
			    WUSB3801_CTRL0_TRY_SNK |
			    WUSB3801_CTRL0_CURRENT_DEFAULT |
			    WUSB3801_CTRL0_ROLE_DRP);
}

static void wusb3801_hw_update(struct wusb3801 *wusb3801)
{
	struct device *dev = wusb3801->dev;
	unsigned int status;
	unsigned int partner_type;
	int ret;

	ret = regmap_read(wusb3801->regmap, WUSB3801_REG_STAT, &status);
	if (ret) {
		dev_warn(dev, "Failed to read port status: %d\n", ret);
		status = 0;
	}
	dev_dbg(dev, "status = 0x%02x\n", status);

	partner_type = status & WUSB3801_STAT_PARTNER;

	switch (status & WUSB3801_STAT_ORIENTATION) {
		case WUSB3801_STAT_ORIENTATION_NONE:
		case WUSB3801_STAT_ORIENTATION_BOTH:
		default:
			gpiod_set_value(wusb3801->ch482d_sel_gpio, 1);
			break;
		case WUSB3801_STAT_ORIENTATION_CC1:
			gpiod_set_value(wusb3801->ch482d_sel_gpio, 1);
			break;
		case WUSB3801_STAT_ORIENTATION_CC2:
			gpiod_set_value(wusb3801->ch482d_sel_gpio, 0);
			break;
	}
}

static void wusb3801_worker(struct work_struct *work)
{
	struct wusb3801 *wusb3801 = container_of(work, struct wusb3801, work);

	// struct typec_port *port = wusb3801->port;
	struct device *dev = wusb3801->dev;
	unsigned int status;
	unsigned int partner_type;
	int ret, cc_stat;

	ret = regmap_read(wusb3801->regmap, WUSB3801_REG_STAT, &status);
	if (ret) {
		dev_warn(dev, "Failed to read port status: %d\n", ret);
		status = 0;
	}
	dev_dbg(dev, "status = 0x%02x\n", status);

	partner_type = status & WUSB3801_STAT_PARTNER;

#ifdef __TEST_CC_PATCH__
	if ((wusb3801->dev_sub_id != 0xA0) &&
			((status & 0x03) == 0))	{	// Reg.04 = 0x88
		cc_stat = test_cc_patch(wusb3801);
		// printk("%s: cc_sts[0x%02x]\n", __func__, cc_stat);
		if (cc_stat == WUSB3801_CC2_CONNECTED)
			status = status | 0x02;
		else if (cc_stat == WUSB3801_CC1_CONNECTED)
			status = status | 0x01;
		// printk("%s: cc_test_patch rc[0x%02x]\n", __func__, status);
	}
#endif	/* __TEST_CC_PATCH__ */

	/* no partner recognize */
	switch (status & WUSB3801_STAT_ORIENTATION) {
		case WUSB3801_STAT_ORIENTATION_NONE:
		case WUSB3801_STAT_ORIENTATION_BOTH:
		default:
			gpiod_set_value(wusb3801->ch482d_sel_gpio, 1);
			// printk("[%s %d]\n", __func__, __LINE__);
			break;
		case WUSB3801_STAT_ORIENTATION_CC1:
			gpiod_set_value(wusb3801->ch482d_sel_gpio, 1);
			// printk("[%s %d]\n", __func__, __LINE__);
			break;
		case WUSB3801_STAT_ORIENTATION_CC2:
			gpiod_set_value(wusb3801->ch482d_sel_gpio, 0);
			// printk("[%s %d]\n", __func__, __LINE__);
			break;
	}
}

static irqreturn_t wusb3801_irq(int irq, void *data)
{
	struct wusb3801 *wusb3801 = data;
	unsigned int dummy;

	/*
	 * The interrupt register must be read in order to clear the IRQ,
	 * but all of the useful information is in the status register.
	 */
	regmap_read(wusb3801->regmap, WUSB3801_REG_INT, &dummy);

	// wusb3801_hw_update(wusb3801);
	queue_work(wusb3801->wq, &wusb3801->work);

	return IRQ_HANDLED;
}

static const struct regmap_config config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= WUSB3801_REG_TEST12,
};

static int wusb3801_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	unsigned int device_id, test01;
	struct wusb3801 *wusb3801;
	int ret;

	wusb3801 = devm_kzalloc(dev, sizeof(*wusb3801), GFP_KERNEL);
	if (!wusb3801)
		return -ENOMEM;

	i2c_set_clientdata(client, wusb3801);

	wusb3801->dev = dev;

	wusb3801->regmap = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(wusb3801->regmap))
		return PTR_ERR(wusb3801->regmap);

	regmap_read(wusb3801->regmap, WUSB3801_REG_DEVICE_ID, &device_id);
	regmap_read(wusb3801->regmap, WUSB3801_REG_TEST01, &test01);
	dev_info(dev, "Vendor ID: %ld, Version ID: %ld, Vendor SubID: 0x%02lx\n",
		 device_id & WUSB3801_DEVICE_ID_VENDOR_ID,
		 (device_id & WUSB3801_DEVICE_ID_VERSION_ID) >> 3,
		 test01 & WUSB3801_TEST01_VENDOR_SUB_ID);
	wusb3801->dev_sub_id = test01 & WUSB3801_TEST01_VENDOR_SUB_ID;

	/* Initialize the hardware with the devicetree settings. */
	ret = wusb3801_hw_init(wusb3801);
	if (ret)
		return ret;

	/* Initialize the port attributes from the hardware state. */
	wusb3801_hw_update(wusb3801);

	wusb3801->cc_sts = 0xff;

	wusb3801->wq = create_singlethread_workqueue(dev_name(dev));
	if (!wusb3801->wq) {
		dev_err(dev, "Could not create workqueue\n");
		goto err_unregister_port;
	}
	flush_workqueue(wusb3801->wq);
	INIT_WORK(&wusb3801->work, wusb3801_worker);

	ret = request_threaded_irq(client->irq, NULL, wusb3801_irq,
				   IRQF_ONESHOT, "wusb3801", wusb3801);
	if (ret)
		goto err_unregister_port;

	wusb3801->ch482d_sel_gpio = devm_gpiod_get_optional(dev, "ch482d_sel",
						     GPIOD_OUT_LOW);
	if (IS_ERR(wusb3801->ch482d_sel_gpio)) {
		ret = PTR_ERR(wusb3801->ch482d_sel_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get ch482d_sel GPIO: %d\n", ret);
		goto err_unregister_port;
	}
	gpiod_set_value(wusb3801->ch482d_sel_gpio, 1);

	return 0;

err_unregister_port:

	return ret;
}

static int wusb3801_remove(struct i2c_client *client)
{
	struct wusb3801 *wusb3801 = i2c_get_clientdata(client);

	cancel_work_sync(&wusb3801->work);
	free_irq(client->irq, wusb3801);
	destroy_workqueue(wusb3801->wq);

	return 0;
}

static const struct of_device_id wusb3801_of_match[] = {
	{ .compatible = "willsemi,wusb3801" },
	{}
};
MODULE_DEVICE_TABLE(of, wusb3801_of_match);

static struct i2c_driver wusb3801_driver = {
	.probe_new	= wusb3801_probe,
	.remove		= wusb3801_remove,
	.driver		= {
		.name		= "wusb3801",
		.of_match_table	= wusb3801_of_match,
	},
};

module_i2c_driver(wusb3801_driver);

MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("Willsemi WUSB3801 Type-C port controller driver");
MODULE_LICENSE("GPL");
