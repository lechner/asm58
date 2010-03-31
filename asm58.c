/*
 * asm58.c - Part of lm_sensors, Linux kernel modules for hardware monitoring
 *
 * Copyright (c) 2009-2010 Michael Lossin <michael.lossin@web.de>
 *
 * based on the 2.6.15 version by Rigel Freden <rigelf@users.sf.net>,
 * inspired by the w83l785ts driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/version.h>

/*
 * Address to scan
 * Address is fully defined internally and cannot be changed.
 */

static const unsigned short normal_i2c[] = { 0x77, I2C_CLIENT_END };


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
/* Insmod parameters */
I2C_CLIENT_INSMOD_1(asm58);
#endif

/*
 * The ASM58 registers
 */

#define ASM58_REG_CONFIG	0x40
#define ASM58_REG_MODE		0x4E
#define ASM58_REG_CHIP_ID	0x58
#define ASM58_REG_VEND_ID	0x4F
#define ASM58_REG_TEMP(i)	((i) ? 0x13 : 0x27)
#define ASM58_REG_FAN(i)	(0x28 + (i))
#define ASM58_REG_FANDIV	0xA1
#define ASM58_REG_IN(i)		(!(i) ? 0x20 : 0x20 + (i) + 1)

/*
 * Conversions
 */

#define ASM58_FANDIV_FROM_REG(r) (1 << (r))

#define ASM58_RPM_FROM_REG(f, r) \
(((f) == 255 || (f) == 0) ? 0 : 1350000 / ((f) * (r)))

#define ASM58_TEMP_FROM_REG(t) (((t) & 0x80 ? (t) - 0x100 : (t)) * 1000)

#define ASM58_IN_FROM_REG(i) ((i) * 16)

/*
 * Functions declaration
 */
static int asm58_probe(struct i2c_client *client,
		       const struct i2c_device_id *id);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)
static int asm58_detect(struct i2c_client *client,
			struct i2c_board_info *info);
#else
static int asm58_detect(struct i2c_client *client, int kind,
			struct i2c_board_info *info);
#endif

static int asm58_remove(struct i2c_client *client);
static struct asm58_data *asm58_update_device(struct device *dev);

/*
 * Driver data (common to all clients)
 */

static const struct i2c_device_id asm58_id[] = {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)
	{"asm58", 0},
#else
	{"asm58", asm58},
#endif

	{}
};

MODULE_DEVICE_TABLE(i2c, asm58_id);

static struct i2c_driver asm58_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		   .name = "asm58",
		   },
	.probe = asm58_probe,
	.remove = asm58_remove,
	.id_table = asm58_id,
	.detect = asm58_detect,

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)
	.address_list = normal_i2c,
#else
	.address_data= &addr_data,
#endif

};

static int asm58_read_byte(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int asm58_write_byte(struct i2c_client *client, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

/*
 * Client data (each client gets its own)
 */

struct asm58_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	char valid;		/* zero until following fields are valid */
	unsigned long last_updated;	/* in jiffies */

	/* registers values */
	u8 temp[2];
	u8 fan[2];
	u8 fan_div[2];		/* shifted to the right */
	u8 in[4];
};

/*
 * Sysfs stuff
 */

static ssize_t show_temp(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct asm58_data *data = asm58_update_device(dev);
	return sprintf(buf, "%d\n",
		       ASM58_TEMP_FROM_REG(data->temp[attr->index]));
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_temp, NULL, 1);

static int asm58_fan_min_lim[] = { 5314, 2657, 1328, 664 };

static ssize_t show_fan(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	int n, fan_rpm, div_mod = 0;
	struct asm58_data *data = asm58_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	n = attr->index;
	fan_rpm = ASM58_RPM_FROM_REG(data->fan[n],
				     ASM58_FANDIV_FROM_REG(data->fan_div[n]));

	/* if the fan is spinning below 125% of the current divisor limit,
	   increment */
	if ((fan_rpm < (5 * asm58_fan_min_lim[data->fan_div[n]]) / 4)
	    && (data->fan_div[n] < 3))
		div_mod = 1;
	/* if above 300%, decrement */
	else if ((fan_rpm > 3 * asm58_fan_min_lim[data->fan_div[n]])
		 && (data->fan_div[n] > 0))
		div_mod = -1;

	if (div_mod) {
		mutex_lock(&data->update_lock);
		data->fan_div[n] += div_mod;
		data->valid = 0;
		/* just masking and shifting for the hardware fandiv reg */
		asm58_write_byte(to_i2c_client(dev), ASM58_REG_FANDIV,
				 ((data->fan_div[0] & 0x03) << 4) |
				 ((data->fan_div[1] & 0x03) << 6));
		mutex_unlock(&data->update_lock);
	}

	return sprintf(buf, "%d\n", fan_rpm);
}

static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, show_fan, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, show_fan, NULL, 1);

static ssize_t show_in(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct asm58_data *data = asm58_update_device(dev);
	return sprintf(buf, "%d\n", ASM58_IN_FROM_REG(data->in[attr->index]));
}

static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, show_in, NULL, 3);

/*
 * Real code
 */

/* Return 0 if detection is successful, -ENODEV otherwise */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)
static int asm58_detect(struct i2c_client *new_client,
			struct i2c_board_info *info)
#else
static int asm58_detect(struct i2c_client *new_client, int kind,
			struct i2c_board_info *info)
#endif

{
	int chipid, subtype, vendid;
	struct i2c_adapter *adapter = new_client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	chipid = asm58_read_byte(new_client, ASM58_REG_CHIP_ID);
	subtype = asm58_read_byte(new_client, ASM58_REG_MODE);
	if (!(chipid == 0x56 && subtype == 0x94) &&
	    !(chipid == 0x10 && subtype == 0x5c))
		return -ENODEV;

	asm58_write_byte(new_client, ASM58_REG_MODE, 0x00);
	vendid = asm58_read_byte(new_client, ASM58_REG_VEND_ID);
	if (!((chipid == 0x56 && subtype == 0x94 && vendid == 0x36) ||
	      (chipid == 0x56 && subtype == 0x94 && vendid == 0x06) ||
	      (chipid == 0x10 && subtype == 0x5c && vendid == 0xa3)))
		return -ENODEV;

	strlcpy(info->type, "asm58", I2C_NAME_SIZE);

	return 0;
}

static int asm58_probe(struct i2c_client *new_client,
		       const struct i2c_device_id *id)
{
	struct asm58_data *data;
	int err = 0;

	data = kzalloc(sizeof(struct asm58_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(new_client, data);
	data->valid = 0;
	mutex_init(&data->update_lock);

	/* Default values in case the first read fails (unlikely). */
	data->temp[0] = data->temp[1] = 0;
	data->fan[0] = data->fan[1] = 0;
	data->fan_div[0] = data->fan_div[1] = 0;
	data->in[0] = data->in[1] = data->in[2] = data->in[3] = 0;

	/*
	 * Initialize the ASM58 chip
	 * Nothing yet, assume it is already started.
	 */

	if ((err = device_create_file(&new_client->dev,
				      &sensor_dev_attr_temp1_input.dev_attr))
	    || (err = device_create_file(&new_client->dev,
					 &sensor_dev_attr_temp2_input.dev_attr))
	    || (err = device_create_file(&new_client->dev,
					 &sensor_dev_attr_fan1_input.dev_attr))
	    || (err = device_create_file(&new_client->dev,
					 &sensor_dev_attr_fan2_input.dev_attr))
	    || (err = device_create_file(&new_client->dev,
					 &sensor_dev_attr_in0_input.dev_attr))
	    || (err = device_create_file(&new_client->dev,
					 &sensor_dev_attr_in1_input.dev_attr))
	    || (err = device_create_file(&new_client->dev,
					 &sensor_dev_attr_in2_input.dev_attr))
	    || (err = device_create_file(&new_client->dev,
					 &sensor_dev_attr_in3_input.dev_attr)))
		goto exit_remove;

	/* Register sysfs hooks */
	data->hwmon_dev = hwmon_device_register(&new_client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	return 0;

exit_remove:
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_temp1_input.dev_attr);
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_temp2_input.dev_attr);
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_fan1_input.dev_attr);
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_fan2_input.dev_attr);
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_in0_input.dev_attr);
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_in1_input.dev_attr);
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_in2_input.dev_attr);
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_in3_input.dev_attr);
	kfree(data);
exit:
	return err;
}

static int asm58_remove(struct i2c_client *client)
{
	struct asm58_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	device_remove_file(&client->dev, &sensor_dev_attr_temp1_input.dev_attr);
	device_remove_file(&client->dev, &sensor_dev_attr_temp2_input.dev_attr);
	device_remove_file(&client->dev, &sensor_dev_attr_fan1_input.dev_attr);
	device_remove_file(&client->dev, &sensor_dev_attr_fan2_input.dev_attr);
	device_remove_file(&client->dev, &sensor_dev_attr_in0_input.dev_attr);
	device_remove_file(&client->dev, &sensor_dev_attr_in1_input.dev_attr);
	device_remove_file(&client->dev, &sensor_dev_attr_in2_input.dev_attr);
	device_remove_file(&client->dev, &sensor_dev_attr_in3_input.dev_attr);
	kfree(data);
	return 0;
}

static struct asm58_data *asm58_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct asm58_data *data = i2c_get_clientdata(client);
	int i;
	int fandiv;

	mutex_lock(&data->update_lock);

	if (!data->valid || time_after(jiffies, data->last_updated + HZ * 2)) {
		dev_dbg(&client->dev, "Updating asm58 data.\n");

		for (i = 0; i < 4; i++)
			data->in[i] = asm58_read_byte(client, ASM58_REG_IN(i));

		fandiv = asm58_read_byte(client, ASM58_REG_FANDIV);
		data->fan_div[0] = (fandiv >> 4) & 0x03;
		data->fan_div[1] = fandiv >> 6;

		for (i = 0; i < 2; i++) {
			data->fan[i] =
			    asm58_read_byte(client, ASM58_REG_FAN(i));
			data->temp[i] =
			    asm58_read_byte(client, ASM58_REG_TEMP(i));
		}

		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

static int __init sensors_asm58_init(void)
{
	return i2c_add_driver(&asm58_driver);
}

static void __exit sensors_asm58_exit(void)
{
	i2c_del_driver(&asm58_driver);
}

MODULE_AUTHOR("Michael Lossin <michael.lossin@web.de>");
MODULE_DESCRIPTION("Asus Mozart-2 driver");
MODULE_LICENSE("GPL");

module_init(sensors_asm58_init);
module_exit(sensors_asm58_exit);
