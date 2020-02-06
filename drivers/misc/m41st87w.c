/*
 * m41st87w.c
 *
 * Copyright (c) 2009 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 * Description: Provides RTC, Watchdog and Tamper Detection
 * References:
 *   [1] M41ST87W.pdf
 *
 **/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/mutex.h>

enum m41st87w_regs {
        REG_CSEC 	= 0x00,
        REG_SEC  	= 0x01,
        REG_MIN  	= 0x02,
        REG_HOUR 	= 0x03,
        REG_DAY_WEEK 	= 0x04,
        REG_DAY_MONTH 	= 0x05,
        REG_MONTH       = 0x06,
        REG_YEAR        = 0x07,
        REG_CONTROL     = 0x08,
        REG_WATCHDOG    = 0x09,
        REG_ALARM_MONTH = 0x0A,
        REG_ALARM_DAY   = 0x0B,
        REG_ALARM_HOUR  = 0x0C,
        REG_ALARM_MINUTE= 0x0D,
        REG_ALARM_SEC   = 0x0E,
        REG_FLAGS       = 0x0F,
        REG_SQW         = 0x13,
        REG_TAMPER1     = 0x14,
        REG_TAMPER2     = 0x15,
        REG_SERIAL      = 0x16,
        REG_USER        = 0x20,
};

#define REG_SERIAL_LEN 8
#define REG_USER_LEN   128

#define REG_SEC_STOP	(1 << 7)
#define REG_MIN_OFIE	(1 << 7)
#define REG_HOUR_CB1	(1 << 7)
#define REG_HOUR_CB0	(1 << 6)
#define REG_DAY_WEEK_TR	(1 << 7)
#define REG_DAY_WEEK_THS    (1 << 6)
#define REG_DAY_WEEK_CLRPW1 (1 << 5)
#define REG_DAY_WEEK_CLRPW0 (1 << 4)
#define REG_DAY_WEEK_32KE   (1 << 3)
#define REG_CONTROL_OUT (1 << 7)
#define REG_CONTROL_FT  (1 << 6)
#define REG_CONTROL_S   (1 << 5)
#define REG_WATCHDOG_WDS  (1 << 7)
#define REG_WATCHDOG_BMB4 (1 << 6)
#define REG_WATCHDOG_BMB3 (1 << 5)
#define REG_WATCHDOG_BMB2 (1 << 4)
#define REG_WATCHDOG_BMB1 (1 << 3)
#define REG_WATCHDOG_BMB0 (1 << 2)
#define REG_WATCHDOG_RB1  (1 << 1)
#define REG_WATCHDOG_RB0  (1 << 5)
#define REG_ALARM_MONTH_AFE  (1 << 7)
#define REG_ALARM_MONTH_SQWE (1 << 6)
#define REG_ALARM_MONTH_ABE  (1 << 5)
#define REG_ALARM_HOUR_HT    (1 << 6)
#define REG_FLAGS_WDF	(1 << 7)
#define REG_FLAGS_AF	(1 << 6)
#define REG_FLAGS_BL	(1 << 4)
#define REG_FLAGS_OF	(1 << 2)
#define REG_FLAGS_TB1	(1 << 1)
#define REG_FLAGS_TB0	(1 << 0)

struct m41st87w_priv {
	struct i2c_client *client;
	struct rtc_device *rtc;

        u8 regs[REG_SERIAL];
};

/* ********** RTC Operations ********** */

static int m41st87w_get_time(struct device *dev, struct rtc_time *t)
{
	struct m41st87w_priv *priv = dev_get_drvdata(dev);
        int error;

        error = i2c_smbus_read_i2c_block_data(priv->client, REG_CSEC, REG_YEAR+1, priv->regs);
        if (error < 0) {
		dev_err(dev, "%s error %d\n", "read", error);
		return error;
        }

        /* see man mktime for base */
        t->tm_sec   = bcd2bin(priv->regs[REG_SEC] & 0x7F);
        t->tm_min   = bcd2bin(priv->regs[REG_MIN] & 0x7F);
        t->tm_hour  = bcd2bin(priv->regs[REG_HOUR] & 0x3F);
        t->tm_mday  = bcd2bin(priv->regs[REG_DAY_MONTH] & 0x3F);
        t->tm_mon   = bcd2bin(priv->regs[REG_MONTH] & 0x1F)-1;
        t->tm_year  = bcd2bin(priv->regs[REG_YEAR]);
        t->tm_year += 100;      /* since 2000 */
        if (priv->regs[REG_HOUR] & REG_HOUR_CB0)
                t->tm_year += 200;
        if (priv->regs[REG_HOUR] & REG_HOUR_CB1)
                t->tm_year += 100;
        t->tm_wday  = bcd2bin(priv->regs[REG_DAY_WEEK] & 0x07)-1;
        t->tm_yday  = -1;
        t->tm_isdst = -1;

	return rtc_valid_tm(t);
}

static int m41st87w_set_time(struct device *dev, struct rtc_time *t)
{
	struct m41st87w_priv *priv = dev_get_drvdata(dev);
        int error;
        int year;

        if (t->tm_year < 100) {
		/* base is 1900 */
		dev_err(dev, "year must be >= 2000\n");
		return -EINVAL;
        }

        /* keep upper control bits */
        priv->regs[REG_CSEC]       = 0x00;
        priv->regs[REG_SEC]       &= ~0x7F;
        priv->regs[REG_SEC]       |= bin2bcd(t->tm_sec) & 0x7F;
        priv->regs[REG_MIN]       &= ~0x7F;
        priv->regs[REG_MIN]       |= bin2bcd(t->tm_min) & 0x7F;
        priv->regs[REG_HOUR]      &= ~0x3F;
        priv->regs[REG_HOUR]      |= bin2bcd(t->tm_hour) & 0x3F;
        priv->regs[REG_DAY_WEEK]  &= ~0x07;
        priv->regs[REG_DAY_WEEK]  |= bin2bcd(t->tm_wday+1) & 0x07;
        /* always enable 32kHz clock output */
        priv->regs[REG_DAY_WEEK]  |= REG_DAY_WEEK_32KE;
        priv->regs[REG_DAY_MONTH] &= ~0x3F;
        priv->regs[REG_DAY_MONTH] |= bin2bcd(t->tm_mday) & 0x3F;
        priv->regs[REG_MONTH]     &= ~0x1F;
        priv->regs[REG_MONTH]     |= bin2bcd(t->tm_mon+1) & 0x1F;

        year = t->tm_year - 100;   /* since 2000 */
        priv->regs[REG_HOUR] &= ~0xC0;
        if (year >= 200) {
                priv->regs[REG_HOUR] |= REG_HOUR_CB0;
                year -= 200;
        }
        if (year >= 100) {
                priv->regs[REG_HOUR] |= REG_HOUR_CB1;
                year -= 100;
        }

        priv->regs[REG_YEAR]       = bin2bcd(year);

	error = i2c_smbus_write_i2c_block_data(priv->client, REG_CSEC, REG_YEAR+1, priv->regs);
        if (error < 0) {
		dev_err(dev, "%s error %d\n", "write", error);
		return -EIO;
        }

        return 0;
}

static const struct rtc_class_ops m41st87w_rtc_ops = {
	.read_time	= m41st87w_get_time,
	.set_time	= m41st87w_set_time,
};

/* ********** Driver Operations ********** */

static int m41st87w_hw_init(struct m41st87w_priv *priv)
{
        int tmp;

        tmp = i2c_smbus_read_i2c_block_data(priv->client, REG_CSEC, REG_SERIAL+REG_SERIAL_LEN+1, priv->regs);
        if (tmp != REG_SERIAL+REG_SERIAL_LEN+1) {
		dev_err(&priv->client->dev, "%s error %d\n", "read", tmp);
		return -EIO;
        }

        if (priv->regs[REG_ALARM_HOUR] & REG_ALARM_HOUR_HT) {
                dev_info(&priv->client->dev, "resetting halt update\n");
                priv->regs[REG_ALARM_HOUR] &= ~REG_ALARM_HOUR_HT;
                i2c_smbus_write_byte_data(priv->client, REG_ALARM_HOUR, priv->regs[REG_ALARM_HOUR]);
        }

        if (priv->regs[REG_FLAGS] & REG_FLAGS_OF) {
                struct rtc_time t;

                dev_info(&priv->client->dev, "clock oscillator had stopped - SET TIME\n");
                /* see [1] 3.16. We assume that at least 4seconds have passed
                 * since applying battery power */
                priv->regs[REG_FLAGS] &= ~REG_FLAGS_OF;
                i2c_smbus_write_byte_data(priv->client, REG_FLAGS, priv->regs[REG_FLAGS]);

                /* set a date so clock is running and doesn't return with any insane values */
                rtc_time_to_tm(mktime(2000, 1, 1, 0, 0, 0), &t);

                m41st87w_set_time(&priv->client->dev, &t);
        }

        return 0;
}

static int __devinit m41st87w_probe(struct i2c_client *client,
                                    const struct i2c_device_id *id)
{
        struct m41st87w_priv *priv;
	struct i2c_adapter   *adapter = to_i2c_adapter(client->dev.parent);
        int error;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA) &&
	    !i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
                dev_err(&client->dev, "Wrong I2C functionality\n");
                error = -EIO;
                goto error;
        }

	if (!(priv = kzalloc(sizeof(*priv), GFP_KERNEL))) {
                dev_err(&client->dev, "no memory\n");
                error = -ENOMEM;
                goto error;
        }
	i2c_set_clientdata(client, priv);
        priv->client = client;

        priv->rtc = rtc_device_register(client->name, &client->dev,
                                        &m41st87w_rtc_ops, THIS_MODULE);
        if (IS_ERR(priv->rtc)) {
                error = PTR_ERR(priv->rtc);
                dev_err(&client->dev,
			"unable to register the class device\n");
		goto error_rtc;
        }

        if ((error = m41st87w_hw_init(priv)))
                goto error_init;

        return 0;

error_init:
	rtc_device_unregister(priv->rtc);

error_rtc:
        kfree(priv);

error:
        return error;
}

static int __devexit m41st87w_remove(struct i2c_client *client)
{
	struct m41st87w_priv *priv = i2c_get_clientdata(client);

        rtc_device_unregister(priv->rtc);
        kfree(priv);

        return 0;
}

static const struct i2c_device_id m41st87w_id[] = {
        { "m41st87w", 0 },
	{ }
};

static struct i2c_driver m41st87w_driver = {
	.driver = {
		.name	= "m41st87w",
		.owner	= THIS_MODULE,
	},
	.probe		= m41st87w_probe,
	.remove		= __devexit_p(m41st87w_remove),
	.id_table	= m41st87w_id,
};

static int __init m41st87w_init(void)
{
        return i2c_add_driver(&m41st87w_driver);
}

static void __exit m41st87w_exit(void)
{
        i2c_del_driver(&m41st87w_driver);
}

MODULE_AUTHOR("Markus Pietrek");
MODULE_DESCRIPTION("Timer Pulse Unit TPU control");
MODULE_LICENSE("GPL");

module_init(m41st87w_init);
module_exit(m41st87w_exit);
