/*
 * Touch Screen driver for TSC2007
 *
 * Copyright (c) 2008 Martin Nylund
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU  General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/timer.h>

struct tsc2007_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	int irq;
	int first;
};

enum tsc2007_pd {
  PD_POWERDOWN = 0, /* penirq */
  PD_IREFOFF_ADCON = 1, /* no penirq */
  PD_IREFON_ADCOFF = 2, /* penirq */
  PD_IREFON_ADCON = 3, /* no penirq */
  PD_PENIRQ_ARM = PD_IREFON_ADCOFF,
  PD_PENIRQ_DISARM = PD_IREFON_ADCON,
};

enum tsc2007_m {
  M_12BIT = 0,
  M_8BIT = 1
};

enum tsc2007_cmd {
  MEAS_TEMP0 = 0,
  MEAS_VBAT1 = 1,
  MEAS_IN1 = 2,
  MEAS_TEMP1 = 4,
  MEAS_VBAT2 = 5,
  MEAS_IN2 = 6,
  ACTIVATE_NX_DRIVERS = 8,
  ACTIVATE_NY_DRIVERS = 9,
  ACTIVATE_YNX_DRIVERS = 10,
  SETUP = 11,
  MEAS_XPOS = 12,
  MEAS_YPOS = 13,
  MEAS_Z1POS = 14,
  MEAS_Z2POS = 15
};

#define TSC2007_CMD(cn,pdn,m) (((cn) << 4) | ((pdn) << 2) | ((m) << 1))

void tsc2007_powerdown(	struct i2c_client *client )
{
	char command;
		
	command = TSC2007_CMD(MEAS_XPOS,PD_POWERDOWN, M_12BIT);
	i2c_smbus_write_byte(client, command);	
}

uint16_t tsc2007_read_xpos(struct i2c_client *client)
{
	unsigned char  cmd;
	unsigned short ret;
		
	cmd = TSC2007_CMD(ACTIVATE_NX_DRIVERS, PD_PENIRQ_DISARM, M_12BIT);
	i2c_smbus_write_byte(client, cmd);

	cmd = TSC2007_CMD(MEAS_XPOS, PD_PENIRQ_DISARM, M_12BIT);
	ret = i2c_smbus_read_word_data(client, cmd);

	return (((ret & 0xFF) << 4) | ((ret&0xF000) >> 12));
}

uint16_t tsc2007_read_ypos(struct i2c_client *client)
{
	unsigned char  cmd;
	unsigned short ret;
		
	cmd = TSC2007_CMD(ACTIVATE_NY_DRIVERS, PD_PENIRQ_DISARM, M_12BIT);
	i2c_smbus_write_byte(client, cmd);

	cmd = TSC2007_CMD(MEAS_YPOS, PD_PENIRQ_DISARM, M_12BIT);
	ret = i2c_smbus_read_word_data(client, cmd);

	return (((ret & 0xFF) << 4) | ((ret&0xF000) >> 12));
}


uint16_t tsc2007_read_zpos(struct i2c_client *client)
{  	
	unsigned char command;
	unsigned short pos;
	
	command = TSC2007_CMD(MEAS_Z1POS, PD_PENIRQ_DISARM, M_12BIT);
	pos = i2c_smbus_read_word_data(client, command);
	return (((pos & 0xFF) << 4) | ((pos&0xF000) >> 12));	
}


int tsc2007_pen_is_down(void);
//#define blah(...) printk(__VA_ARGS__)
#define blah(...) 
static void tsc2007_ts_poscheck(struct work_struct *work)
{
	struct tsc2007_ts_priv *priv = container_of(work,
						  struct tsc2007_ts_priv,
						  work.work);
	unsigned short xpos, ypos, zpos;

	blah("touch: ");

	if(tsc2007_pen_is_down()){
	    if(priv->first){
		input_report_key(priv->input, BTN_TOUCH, 1);
		priv->first = 0;
	    }

	    ypos = tsc2007_read_ypos(priv->client);
	    xpos = tsc2007_read_xpos(priv->client);
	    zpos = tsc2007_read_zpos(priv->client);

	    
	    /* If the measured zpos is less than 10, the panel is pressed
	     * to lightly and the values are most probably garbage */
	    blah("xpos=%d, ypos=%d, zpos=%d",xpos,ypos,zpos);
	    if(zpos>10){
		input_report_abs(priv->input, ABS_X, xpos);
		input_report_abs(priv->input, ABS_Y, ypos);
		input_report_abs(priv->input, ABS_PRESSURE, zpos);
		input_sync(priv->input);
	    } else {
		blah(" (ignored)");
	    }

	    tsc2007_powerdown(priv->client);

	    schedule_delayed_work(&priv->work, HZ / 20);
	} else {

	    /* don't bother to report penup, if no position data was
	     * reported */
	    if(!priv->first){
		input_report_abs(priv->input, ABS_PRESSURE, 0);
		input_report_key(priv->input, BTN_TOUCH, 0);
		input_sync(priv->input);
		blah("penup");
	    } else {
		blah("ignored");
	    }


	    priv->first = 0;
	    enable_irq(priv->irq);
	}
	blah("\n");
}

static irqreturn_t tsc2007_ts_isr(int irq, void *dev_id)
{
	struct tsc2007_ts_priv *priv = dev_id;

	/* the touch screen controller chip is hooked up to the cpu
	 * using i2c and a single interrupt line. the interrupt line
	 * is pulled low whenever someone taps the screen. to deassert
	 * the interrupt line we need to acknowledge the interrupt by
	 * communicating with the controller over the slow i2c bus.
	 *
	 * we can't acknowledge from interrupt context since the i2c
	 * bus controller may sleep, so we just disable the interrupt
	 * here and handle the acknowledge using delayed work.
	 */

	disable_irq_nosync(irq);
	priv->first = 1;
	schedule_delayed_work(&priv->work, HZ / 20);

	return IRQ_HANDLED;
}


static int tsc2007_ts_open(struct input_dev *dev)
{
	/*
	struct tsc2007_ts_priv *priv = input_get_drvdata(dev);
	struct i2c_client *client = priv->client;
	*/
	return 0;
}

static void tsc2007_ts_close(struct input_dev *dev)
{
	/*
	struct tsc2007_ts_priv *priv = input_get_drvdata(dev);
	struct i2c_client *client = priv->client;
	*/
}


static int tsc2007_ts_probe(struct i2c_client *client,
			  const struct i2c_device_id *idp)
{
	struct tsc2007_ts_priv *priv;
	struct input_dev *input;
	int error;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		error = -ENOMEM;
		goto err0;
	}

	dev_set_drvdata(&client->dev, priv);

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "Failed to allocate input device.\n");
		error = -ENOMEM;
		goto err1;
	}

	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input, ABS_X, 160, 4000, 0, 0);
	input_set_abs_params(input, ABS_Y, 160, 4000, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 10, 4000, 0, 0);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input->open = tsc2007_ts_open;
	input->close = tsc2007_ts_close;

	input_set_drvdata(input, priv);

	priv->client = client;
	priv->input = input;
	INIT_DELAYED_WORK(&priv->work, tsc2007_ts_poscheck);
	priv->irq = client->irq;

	error = input_register_device(input);
	if (error)
		goto err1;

	error = request_irq(priv->irq, tsc2007_ts_isr, IRQF_TRIGGER_LOW,
			    client->name, priv);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err2;
	}


	return 0;

 err2:
	input_unregister_device(input);
	input = NULL; /* so we dont try to free it below */
 err1:
	input_free_device(input);
	kfree(priv);
 err0:
	dev_set_drvdata(&client->dev, NULL);
	return error;
}

static int tsc2007_ts_remove(struct i2c_client *client)
{
	struct tsc2007_ts_priv *priv = dev_get_drvdata(&client->dev);

	free_irq(priv->irq, priv);
	input_unregister_device(priv->input);
	kfree(priv);

	dev_set_drvdata(&client->dev, NULL);

	return 0;
}

static const struct i2c_device_id tsc2007_ts_id[] = {
	{ "tsc2007", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tsc2007_ts_id);

static struct i2c_driver tsc2007_ts_driver = {
	.driver = {
		.name = "tsc2007",
	},
	.probe = tsc2007_ts_probe,
	.remove = tsc2007_ts_remove,
	.id_table = tsc2007_ts_id,
};

static int __init tsc2007_ts_init(void)
{
	return i2c_add_driver(&tsc2007_ts_driver);
}

static void __exit tsc2007_ts_exit(void)
{
	i2c_del_driver(&tsc2007_ts_driver);
}

MODULE_DESCRIPTION("TSC2007 Touchscreen driver");
MODULE_AUTHOR("Martin Nylund <mnylund@emtrion.de>");
MODULE_LICENSE("GPL");

module_init(tsc2007_ts_init);
module_exit(tsc2007_ts_exit);
