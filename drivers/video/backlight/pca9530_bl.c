/*
 * pca9530_bl.c - Backlight driver for PCA9530
 *
 * Copyright (c) 2010 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author: Markus Pietrek
 *
 * Only LED0 is used.
 *
 **/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/i2c/pca9530_bl.h>

#define PWM0		2
#define LED_SELECTOR   	5

#define LED_OFF        0
#define LED_ON         1
#define LED_PWM0       2

static int pca9530bl_send_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = bl_get_data(bd);
	struct pca9530bl_platform_data *pdata = client->dev.platform_data;
	int ledsel;
	int ret = 0;
	int brightness = bd->props.brightness;

	if (brightness > bd->props.max_brightness || brightness < 0)
		return -EINVAL;

	if (bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;
	if (bd->props.state & BL_CORE_FBBLANK)
		brightness = 0;
	if (bd->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	if (brightness == bd->props.max_brightness)
		ledsel = pdata->invert ? LED_OFF : LED_ON;
	else if (brightness == 0)
		ledsel = pdata->invert ? LED_ON : LED_OFF;
	else {
		ledsel = LED_PWM0;
		if (pdata->invert)
			brightness = bd->props.max_brightness-brightness;
		ret = i2c_smbus_write_byte_data(client, PWM0,
						brightness);
		if (ret < 0)
			goto exit;
	}
	
	ret = i2c_smbus_write_byte_data(client, LED_SELECTOR, ledsel);
exit:
	return ret;
}

static int pca9530bl_get_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = bl_get_data(bd);
	struct pca9530bl_platform_data *pdata = client->dev.platform_data;
	int ledsel;
	int ret;
	
	ret = i2c_smbus_read_byte_data(client, LED_SELECTOR);
	if (ret < 0)
		goto exit;

	ledsel = ret & 0x3;	/* LED 0 */

	if (ledsel == LED_ON)
		ret = pdata->invert ? 0 : bd->props.max_brightness;
	else if (ledsel == LED_OFF)
		ret = pdata->invert ? bd->props.max_brightness : 0;
	else {
		ledsel = LED_PWM0;
		ret = i2c_smbus_read_byte_data(client, PWM0);
		if (pdata->invert)
			ret = bd->props.max_brightness - ret;
	}

exit:
	return ret;
}

static const struct backlight_ops pca9530bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = pca9530bl_get_intensity,
	.update_status  = pca9530bl_send_intensity,
};

static int __devinit pca9530bl_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct backlight_properties props;
	struct pca9530bl_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct backlight_device *bd;
	int err = 0;

	if (!pdata) {
		dev_err(&client->dev, "Need platform data\n");
		err = -EINVAL;
		goto error;
	}

	if (!i2c_check_functionality(adapter,
				     I2C_FUNC_SMBUS_BYTE |
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA)) {
		err = -EIO;
		goto error_i2c;
	}

	if (i2c_smbus_read_byte_data(client, LED_SELECTOR) <0) {
		/* There are displays adapters out there without any backlight control. */
		dev_info(&client->dev, "No PCA9530 backlight control available\n");
		err = -ENODEV;
		goto error_i2c;
	}

	memset(&props, 0, sizeof(props));
	bd = backlight_device_register(client->name,
				       &client->dev, client, &pca9530bl_ops,
				       &props);
	if (IS_ERR(bd)) {
		err = PTR_ERR(bd);
		goto error_bd;
	}

	i2c_set_clientdata(client, bd);

	/* initialize backlight */
	bd->props.max_brightness = 256;
	bd->props.power          = FB_BLANK_UNBLANK;
	bd->props.brightness     = bd->props.max_brightness;

	backlight_update_status(bd);

	dev_info(&client->dev, "initialized\n");

	return 0;

error_bd:
error_i2c:
error:
	return err;
}

static int __devexit pca9530bl_remove(struct i2c_client *client)
{
	struct backlight_device *bd = i2c_get_clientdata(client);

	bd->props.power = 0;
	bd->props.brightness = 0;
	backlight_update_status(bd);

	backlight_device_unregister(bd);

	return 0;
}

static const struct i2c_device_id pca9530bl_id[] = {
       { "pca9530", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, pca9530bl_id);

static struct i2c_driver pca9530bl_driver = {
       .driver = {
               .name   = "pca9530",
               .owner  = THIS_MODULE,
       },
       .probe  = pca9530bl_probe,
       .remove = __devexit_p(pca9530bl_remove),
       .id_table = pca9530bl_id,
};

static int __init pca9530bl_init(void)
{
       return i2c_add_driver(&pca9530bl_driver);
}

static void __exit pca9530bl_exit(void)
{
       i2c_del_driver(&pca9530bl_driver);
}

MODULE_AUTHOR("Markus Pietrek");
MODULE_DESCRIPTION("PCA9530 backlight driver");
MODULE_LICENSE("GPL");

module_init(pca9530bl_init);
module_exit(pca9530bl_exit);

