/*
 * mpu6050 Six-axis IMU driver
 *
 * Copyright (C) 2018  Juan José Tarrio <juan.tarrio@gmail.com>
 *
 * Lite driver for the MPU6050 to be used with the Raspberry PI 2 with GPIO interrupts
 *
 * This program is based on mpu3050.c by Joseph Lai and Alan Cox
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>


#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/tty.h>
#include <linux/sched.h>

#include <linux/version.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/signal.h>
#include <linux/irq.h>

//#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */



//BCM2835
void * gpio_mem;

#define BCM2835_GPIO_ADDR   0x3F200000 //0x7E200000;
#define BCM2835_GPIO_SIZE   0xb3

#define BCM2835_GPFSEL0     0x00
#define BCM2835_GPREN0      0x4C
#define BCM2835_GPEDS0      0x40

#define BCM2835_GPFSET0     0x1C
#define BCM2835_GPFCLR0     0x28

#define BCM2835_GPIO_INT            49

#define BCM2835_I2C_ADAPTER         1


#define BCM2835_GPIO_FN_IN  0b000
#define BCM2835_GPIO_FN_OUT 0b001


/* MPU6050*/

#define mpu6050_CHIP_ADDR	0b1101000
#define mpu6050_CHIP_ID		0x68

#define mpu6050_AUTO_DELAY	1000

#define mpu6050_MIN_VALUE	-32768
#define mpu6050_MAX_VALUE	32767

/* Register map */
#define mpu6050_CHIP_ID_REG     0x75
#define mpu6050_SMPLRT_DIV      0x19
#define mpu6050_CONFIG          0x1A
#define mpu6050_GIROCONFIG      0x1B
#define mpu6050_ACELCONFIG      0x1C
#define mpu6050_INT_CFG         0x37
#define mpu6050_INT_ENABLE		0x38
#define mpu6050_ACEL_XOUT_H		0x3B
#define mpu6050_PWR_MGM         0x6B


/* Register bits */

/* DLPF_FS_SYNC */
#define mpu6050_EXT_SYNC_NONE		0x00
#define mpu6050_EXT_SYNC_TEMP		0x20
#define mpu6050_EXT_SYNC_GYROX		0x40
#define mpu6050_EXT_SYNC_GYROY		0x60
#define mpu6050_EXT_SYNC_GYROZ		0x80
#define mpu6050_EXT_SYNC_ACCELX     0xA0
#define mpu6050_EXT_SYNC_ACCELY     0xC0
#define mpu6050_EXT_SYNC_ACCELZ     0xE0
#define mpu6050_EXT_SYNC_MASK		0xE0


#define mpu6050_DLPF_CFG_256HZ_NOLPF2	0x00
#define mpu6050_DLPF_CFG_188HZ		0x01
#define mpu6050_DLPF_CFG_98HZ		0x02
#define mpu6050_DLPF_CFG_42HZ		0x03
#define mpu6050_DLPF_CFG_20HZ		0x04
#define mpu6050_DLPF_CFG_10HZ		0x05
#define mpu6050_DLPF_CFG_5HZ		0x06
#define mpu6050_DLPF_CFG_MASK		0x07


/* ACEL CFG - GIRO CFG*/

#define mpu6050_FS_SEL_POS          0x03
#define mpu6050_FS_SEL_GIRO_250     0x00
#define mpu6050_FS_SEL_GIRO_500     0x01
#define mpu6050_FS_SEL_GIRO_1000    0x02
#define mpu6050_FS_SEL_GIRO_2000    0x03
#define mpu6050_FS_SEL_ACEL_2       0x00
#define mpu6050_FS_SEL_ACEL_4       0x01
#define mpu6050_FS_SEL_ACEL_8       0x02
#define mpu6050_FS_SEL_ACEL_16      0x03

/* INT_CFG */
#define mpu6050_DATA_RDY            0x01
#define mpu6050_RD_CLEAR            0x10
#define mpu6050_LATCH_INT_EN		0x20
/* PWR_MGM */
#define mpu6050_PWR_MGM_PLL_X		0x01
#define mpu6050_PWR_MGM_PLL_Y		0x02
#define mpu6050_PWR_MGM_PLL_Z		0x03
#define mpu6050_PWR_MGM_CLKSEL		0x07
#define mpu6050_PWR_MGM_RESET		0x80
#define mpu6050_PWR_MGM_POS	6
#define mpu6050_PWR_MGM_MASK	(1<<mpu6050_PWR_MGM_POS)


//Module params


static int BCM2835_GPIO_IN0 = 4;
static int BCM2835_GPIO_TRIGGER_OUT = 14;
static int mpu6050_SAMPLE_PERIOD_MS = 10;
static int TRIGGER_PERIODS = 4;
static int mpu6050_FS_SEL_GIRO = mpu6050_FS_SEL_GIRO_2000;
static int mpu6050_FS_SEL_ACEL = mpu6050_FS_SEL_ACEL_16;
static int mpu6050_CFG_BW = mpu6050_DLPF_CFG_42HZ;


module_param(BCM2835_GPIO_IN0, int, S_IRUGO);
module_param(BCM2835_GPIO_TRIGGER_OUT, int, S_IRUGO);
module_param(mpu6050_SAMPLE_PERIOD_MS, int, S_IRUGO);
module_param(TRIGGER_PERIODS, int, S_IRUGO);
module_param(mpu6050_FS_SEL_GIRO, int, S_IRUGO);
module_param(mpu6050_FS_SEL_ACEL, int, S_IRUGO);
module_param(mpu6050_CFG_BW, int, S_IRUGO);



struct axis_data {
	s16 x;
	s16 y;
	s16 z;
    s16 t;
    s16 gx;
    s16 gy;
    s16 gz;
};

struct mpu6050_sensor {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *idev;
};

/**
 * @brief configBCM2835_GPIOFS - sets the funcion of a GPIO PIN
 * @param pin: pin to set
 * @param fn: function (in/out/alt1/etc)
 * @return
 */

static int configBCM2835_GPIOFS(int pin,int fn){


    int regId=pin/10;
    int pinPos=(pin%10)*3;
    int regAddr=BCM2835_GPFSEL0+regId*4;
    int tmp;

    if(pin>53)
        return -1;

    tmp=ioread32(gpio_mem+regAddr);
    tmp&=~(0b111<<pinPos);
    tmp|=(fn&0b111)<<pinPos;
    iowrite32(tmp,gpio_mem+regAddr); //put GPIO pin as fn

    return 0;
}

/**
 * @brief setBCM2835_GPIOPIN - sets GIO pin on/off
 * @param pin
 * @param state
 * @return
 */

static int setBCM2835_GPIOPIN(int pin,int state){

    if(pin>31)
        return -1;

    if(state)
        iowrite32((1<<pin),gpio_mem+BCM2835_GPFSET0);
    else
        iowrite32((1<<pin),gpio_mem+BCM2835_GPFCLR0);

    return 0;
}

/**
 *	mpu6050_xyz_read_reg	-	read the axes values
 *	@buffer: provide register addr and get register
 *	@length: length of register
 *
 *	Reads the register values in one transaction or returns a negative
 *	error code on failure.
 */
static int mpu6050_xyz_read_reg(struct i2c_client *client,
			       u8 *buffer, int length)
{
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */
    char cmd = mpu6050_ACEL_XOUT_H;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &cmd,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = buffer,
		},
	};

	return i2c_transfer(client->adapter, msg, 2);
}

/**
 *	mpu6050_read_xyz	-	get co-ordinates from device
 *	@client: i2c address of sensor
 *	@coords: co-ordinates to update
 *
 *	Return the converted X Y and Z co-ordinates from the sensor device
 */
static void mpu6050_read_xyz(struct i2c_client *client,
			     struct axis_data *coords)
{
    u16 buffer[7];

    mpu6050_xyz_read_reg(client, (u8 *)buffer, 14);
	coords->x = be16_to_cpu(buffer[0]);
	coords->y = be16_to_cpu(buffer[1]);
	coords->z = be16_to_cpu(buffer[2]);
    coords->t = be16_to_cpu(buffer[3]);
    coords->gx = be16_to_cpu(buffer[4]);
    coords->gy = be16_to_cpu(buffer[5]);
    coords->gz = be16_to_cpu(buffer[6]);
	dev_dbg(&client->dev, "%s: x %d, y %d, z %d\n", __func__,
					coords->x, coords->y, coords->z);
}

/**
 *	mpu6050_set_power_mode	-	set the power mode
 *	@client: i2c client for the sensor
 *	@val: value to switch on/off of power, 1: normal power, 0: low power
 *
 *	Put device to normal-power mode or low-power mode.
 */
static void mpu6050_set_power_mode(struct i2c_client *client, u8 val)
{
	u8 value;

    value = i2c_smbus_read_byte_data(client, mpu6050_PWR_MGM);
    value = (value & ~mpu6050_PWR_MGM_MASK) |
        (((val << mpu6050_PWR_MGM_POS) & mpu6050_PWR_MGM_MASK) ^
         mpu6050_PWR_MGM_MASK);
    i2c_smbus_write_byte_data(client, mpu6050_PWR_MGM, value);
}

/**
 *	mpu6050_input_open	-	called on input event open
 *	@input: input dev of opened device
 *
 *	The input layer calls this function when input event is opened. The
 *	function will push the device to resume. Then, the device is ready
 *	to provide data.
 */
static int mpu6050_input_open(struct input_dev *input)
{
    struct mpu6050_sensor *sensor = input_get_drvdata(input);
	int error;

	pm_runtime_get(sensor->dev);

	/* Enable interrupts */
    error = i2c_smbus_write_byte_data(sensor->client, mpu6050_INT_CFG,
                      mpu6050_LATCH_INT_EN |
                      mpu6050_RD_CLEAR);
	if (error < 0) {
		pm_runtime_put(sensor->dev);
		return error;
	}

    error = i2c_smbus_write_byte_data(sensor->client, mpu6050_INT_ENABLE,
                      mpu6050_DATA_RDY);
    if (error < 0) {
        pm_runtime_put(sensor->dev);
        return error;
    }

    printk("MPU6050custom: input open\n");

	return 0;
}

/**
 *	mpu6050_input_close	-	called on input event close
 *	@input: input dev of closed device
 *
 *	The input layer calls this function when input event is closed. The
 *	function will push the device to suspend.
 */
static void mpu6050_input_close(struct input_dev *input)
{
    struct mpu6050_sensor *sensor = input_get_drvdata(input);

	pm_runtime_put(sensor->dev);
}

/**
 *	mpu6050_interrupt_thread	-	handle an IRQ
 *	@irq: interrupt numner
 *	@data: the sensor
 *
 *	Called by the kernel single threaded after an interrupt occurs. Read
 *	the sensor data and generate an input event for it.
 */
static irqreturn_t mpu6050_interrupt_thread(int irq, void *data)
{
    struct mpu6050_sensor *sensor = data;
	struct axis_data axis;

    mpu6050_read_xyz(sensor->client, &axis);

    input_report_abs(sensor->idev, ABS_X, axis.x);
	input_report_abs(sensor->idev, ABS_Y, axis.y);
	input_report_abs(sensor->idev, ABS_Z, axis.z);
    input_report_abs(sensor->idev, ABS_RX, axis.gx);
    input_report_abs(sensor->idev, ABS_RY, axis.gy);
    input_report_abs(sensor->idev, ABS_RZ, axis.gz);
    input_report_abs(sensor->idev, ABS_THROTTLE, axis.t);
	input_sync(sensor->idev);

	return IRQ_HANDLED;
}

/**
 * @brief pimary_int_handler - a primary INT handler to use with the GPIO shred interupt
 * @param irq
 * @param dev_id
 * @return
 */

static irqreturn_t pimary_int_handler(int irq, void *dev_id)
{

    static int counter=0;

    int tmp;


    tmp=ioread32(gpio_mem+BCM2835_GPEDS0);
    iowrite32(tmp|(1<<BCM2835_GPIO_IN0),gpio_mem+BCM2835_GPEDS0);

    if(BCM2835_GPIO_TRIGGER_OUT>=0){

        if((++counter)==TRIGGER_PERIODS){
            setBCM2835_GPIOPIN(BCM2835_GPIO_TRIGGER_OUT,1);
            counter=0;
        }else{
            setBCM2835_GPIOPIN(BCM2835_GPIO_TRIGGER_OUT,0);
        }

    }

    return IRQ_WAKE_THREAD;

}

/**
 *	mpu6050_hw_init	-	initialize hardware
 *	@sensor: the sensor
 *
 *	Called during device probe; configures the sampling method.
 */
static int mpu6050_hw_init(struct mpu6050_sensor *sensor)
{
	struct i2c_client *client = sensor->client;
	int ret;
	u8 reg;

	/* Reset */
    ret = i2c_smbus_write_byte_data(client, mpu6050_PWR_MGM,
                    mpu6050_PWR_MGM_RESET);
	if (ret < 0)
		return ret;

    ret = i2c_smbus_read_byte_data(client, mpu6050_PWR_MGM);
	if (ret < 0)
		return ret;

    ret = i2c_smbus_write_byte_data(client, mpu6050_PWR_MGM, mpu6050_PWR_MGM_PLL_Z);
	if (ret < 0)
		return ret;

	/* Output frequency divider. The poll interval */
    ret = i2c_smbus_write_byte_data(client, mpu6050_SMPLRT_DIV,
                    mpu6050_SAMPLE_PERIOD_MS - 1);         //Sample time
	if (ret < 0)
		return ret;

	/* Set low pass filter and full scale */
    reg= mpu6050_CFG_BW;
    ret = i2c_smbus_write_byte_data(client, mpu6050_CONFIG, reg);   //Bandwidth, no frame sinc
	if (ret < 0)
		return ret;


    /* Sendor Range */
    reg= mpu6050_FS_SEL_ACEL<<mpu6050_FS_SEL_POS;
    ret = i2c_smbus_write_byte_data(client, mpu6050_ACELCONFIG, reg);
    if (ret < 0)
        return ret;

    /* Sendor Range */
    reg= mpu6050_FS_SEL_GIRO<<mpu6050_FS_SEL_POS;
    ret = i2c_smbus_write_byte_data(client, mpu6050_GIROCONFIG, reg);
    if (ret < 0)
        return ret;

	return 0;
}


/**
 * @brief mpu6050_configGPIO -- sets IN0 as input interrupt and trigger out as otuput trigger
 */
static void mpu6050_configGPIO(void)
{

    int tmp;
   // tmp=ioread32(gpio_mem+BCM2835_GPFSEL0);
   // iowrite32(tmp&(~GPIO_FSEL4_MASK),gpio_mem+BCM2835_GPFSEL0); //put GPIO4 as input

    configBCM2835_GPIOFS(BCM2835_GPIO_IN0,BCM2835_GPIO_FN_IN);


    if(BCM2835_GPIO_TRIGGER_OUT>=0){
        configBCM2835_GPIOFS(BCM2835_GPIO_TRIGGER_OUT,BCM2835_GPIO_FN_OUT);
    }

    tmp=ioread32(gpio_mem+BCM2835_GPREN0);
    iowrite32(tmp|(1<<BCM2835_GPIO_IN0),gpio_mem+BCM2835_GPREN0);       //

}

/**
 * @brief mpu6050_unconfigGPIO -- disables interrupt and output
 */
static void mpu6050_unconfigGPIO(void)
{

    int tmp;
   // tmp=ioread32(gpio_mem+BCM2835_GPFSEL0);
   // iowrite32(tmp&(~GPIO_FSEL4_MASK),gpio_mem+BCM2835_GPFSEL0); //put GPIO4 as input


    if(BCM2835_GPIO_TRIGGER_OUT>=0){
        configBCM2835_GPIOFS(BCM2835_GPIO_TRIGGER_OUT,BCM2835_GPIO_FN_IN);
    }

    tmp=ioread32(gpio_mem+BCM2835_GPREN0);
    iowrite32(tmp&(~(1<<BCM2835_GPIO_IN0)),gpio_mem+BCM2835_GPREN0);       //

}

/**
 *	mpu6050_probe	-	device detection callback
 *	@client: i2c client of found device
 *	@id: id match information
 *
 *	The I2C layer calls us when it believes a sensor is present at this
 *	address. Probe to see if this is correct and to validate the device.
 *
 *	If present install the relevant sysfs interfaces and input device.
 */
static int mpu6050_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{

    struct mpu6050_sensor *sensor;
	struct input_dev *idev;
	int ret;
	int error;

    printk("MPU6050 Driver: Probing...\n");

    sensor = kzalloc(sizeof(struct mpu6050_sensor), GFP_KERNEL);
	idev = input_allocate_device();
	if (!sensor || !idev) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	sensor->client = client;
	sensor->dev = &client->dev;
	sensor->idev = idev;

    mpu6050_set_power_mode(client, 1);
	msleep(10);

    ret = i2c_smbus_read_byte_data(client, mpu6050_CHIP_ID_REG);
	if (ret < 0) {
		dev_err(&client->dev, "failed to detect device\n");
		error = -ENXIO;
		goto err_free_mem;
	}

    if (ret != mpu6050_CHIP_ID) {
		dev_err(&client->dev, "unsupported chip id\n");
		error = -ENXIO;
		goto err_free_mem;
	}

    idev->name = "mpu6050";
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;

    idev->open = mpu6050_input_open;
    idev->close = mpu6050_input_close;

	__set_bit(EV_ABS, idev->evbit);
	input_set_abs_params(idev, ABS_X,
                 mpu6050_MIN_VALUE, mpu6050_MAX_VALUE, 0, 0);
	input_set_abs_params(idev, ABS_Y,
                 mpu6050_MIN_VALUE, mpu6050_MAX_VALUE, 0, 0);
	input_set_abs_params(idev, ABS_Z,
                 mpu6050_MIN_VALUE, mpu6050_MAX_VALUE, 0, 0);
    input_set_abs_params(idev, ABS_RX,
                 mpu6050_MIN_VALUE, mpu6050_MAX_VALUE, 0, 0);
    input_set_abs_params(idev, ABS_RY,
                 mpu6050_MIN_VALUE, mpu6050_MAX_VALUE, 0, 0);
    input_set_abs_params(idev, ABS_RZ,
                 mpu6050_MIN_VALUE, mpu6050_MAX_VALUE, 0, 0);
    input_set_abs_params(idev, ABS_THROTTLE,
                 mpu6050_MIN_VALUE, mpu6050_MAX_VALUE, 0, 0);

	input_set_drvdata(idev, sensor);

	pm_runtime_set_active(&client->dev);

    error = mpu6050_hw_init(sensor);
	if (error)
		goto err_pm_set_suspended;

	error = request_threaded_irq(client->irq,
                     pimary_int_handler, mpu6050_interrupt_thread,
                      IRQF_SHARED,
                     "mpu6050", sensor);
	if (error) {
		dev_err(&client->dev,
			"can't get IRQ %d, error %d\n", client->irq, error);
		goto err_pm_set_suspended;
	}

    mpu6050_configGPIO();


    error = input_register_device(idev);
	if (error) {
		dev_err(&client->dev, "failed to register input device\n");
		goto err_free_irq;
	}

	pm_runtime_enable(&client->dev);
    pm_runtime_set_autosuspend_delay(&client->dev, mpu6050_AUTO_DELAY);

	i2c_set_clientdata(client, sensor);

    printk("MPU6050 Driver: Probing OK\n");


	return 0;

err_free_irq:
	free_irq(client->irq, sensor);
    mpu6050_unconfigGPIO();
err_pm_set_suspended:
	pm_runtime_set_suspended(&client->dev);
err_free_mem:
	input_free_device(idev);
	kfree(sensor);
	return error;
}

/**
 *	mpu6050_remove	-	remove a sensor
 *	@client: i2c client of sensor being removed
 *
 *	Our sensor is going away, clean up the resources.
 */
static int mpu6050_remove(struct i2c_client *client)
{
    struct mpu6050_sensor *sensor = i2c_get_clientdata(client);

    mpu6050_unconfigGPIO();
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	free_irq(client->irq, sensor);
	input_unregister_device(sensor->idev);
	kfree(sensor);

	return 0;
}

#ifdef CONFIG_PM
/**
 *	mpu6050_suspend		-	called on device suspend
 *	@dev: device being suspended
 *
 *	Put the device into sleep mode before we suspend the machine.
 */
static int mpu6050_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

    mpu6050_set_power_mode(client, 0);

	return 0;
}

/**
 *	mpu6050_resume		-	called on device resume
 *	@dev: device being resumed
 *
 *	Put the device into powered mode on resume.
 */
static int mpu6050_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

    mpu6050_set_power_mode(client, 1);
	msleep(100);  /* wait for gyro chip resume */

	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(mpu6050_pm, mpu6050_suspend, mpu6050_resume, NULL);

static const struct i2c_device_id mpu6050_ids[] = {
    { "mpu6050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_ids);

static const struct of_device_id mpu6050_of_match[] = {
    { .compatible = "invn,mpu6050", },
	{ },
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static struct i2c_driver mpu6050_i2c_driver = {
	.driver	= {
        .name	= "mpu6050",
		.owner	= THIS_MODULE,
        .pm	= &mpu6050_pm,
        .of_match_table = mpu6050_of_match,
	},
    .probe		= mpu6050_probe,
    .remove		= mpu6050_remove,
    .id_table	= mpu6050_ids,
};



static struct i2c_board_info mpu6050_i2c_board_info __initdata = {

        I2C_BOARD_INFO("mpu6050", mpu6050_CHIP_ADDR),
        .irq		= (BCM2835_GPIO_INT),

};



static int __init mpu6050custom_init(void)
{

    struct i2c_adapter *i2c_adap;
    printk(KERN_INFO "MPU6050 custom driver loading\n");


    if(BCM2835_GPIO_IN0 > 31 || BCM2835_GPIO_IN0 < 0){
        printk(KERN_ALERT "MPU6050custom: GPIO INT Pin must be between 0 and 31\n");
        return -1;
    }

   // i2c_register_board_info(1, mpu6050_i2c_board_info,
     //       ARRAY_SIZE(mpu6050_i2c_board_info));

    i2c_adap = i2c_get_adapter(BCM2835_I2C_ADAPTER);

    if(i2c_adap==NULL){

        printk(KERN_ALERT "MPU6050custom: error geting adapter\n");
        return -1;
    }



    if(i2c_new_device(i2c_adap,&mpu6050_i2c_board_info)==NULL){

        printk(KERN_ALERT "MPU6050custom: error registerin device\n");

    }

    gpio_mem=ioremap_nocache(BCM2835_GPIO_ADDR, BCM2835_GPIO_SIZE);

    if(gpio_mem==NULL){
        printk(KERN_ALERT "MPU6050custom: Error on ioremap");
        return -1;
    }

    return i2c_add_driver(&mpu6050_i2c_driver);;
}
module_init(mpu6050custom_init);

static void __exit mpu6050custom_cleanup(void)
{
    iounmap(gpio_mem);
    i2c_del_driver(&mpu6050_i2c_driver);
}
module_exit(mpu6050custom_cleanup);


MODULE_AUTHOR("Free Corp.");
MODULE_DESCRIPTION("MPU6050 Tri-axis gyroscope-acelerometer driver");
MODULE_LICENSE("GPL");
