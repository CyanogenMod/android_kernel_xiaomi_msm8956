/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (C) 2016 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <soc/qcom/scm.h>
#include <linux/wakelock.h>

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250

#define FPC_TTW_HOLD_TIME 1000


typedef struct fpc1020_data {
	struct device *dev;
	int irq_gpio;
	int cs0_gpio;
	int cs1_gpio;
	int rst_gpio;
#ifdef USE_PINCTL
	struct pinctrl         *ts_pinctrl;
	struct pinctrl_state   *gpio_state_active;
	struct pinctrl_state   *gpio_state_suspend;
#endif
	struct regulator *vdd_ana;

	struct wake_lock ttw_wl;
	bool wakeup_enabled;
} fpc1020_data_t;

static int fpc1020_request_named_gpio(fpc1020_data_t *fpc1020,
		const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);
	return 0;
}

#ifdef USE_PINCTL
/* -------------------------------------------------------------------- */
static int fpc1020_pinctrl_init(fpc1020_data_t *fpc1020)
{
	int ret = 0;
	struct device *dev = fpc1020->dev;

	fpc1020->ts_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(fpc1020->ts_pinctrl)) {
		dev_err(dev, "Target does not use pinctrl\n");
		ret = PTR_ERR(fpc1020->ts_pinctrl);
		goto err;
	}

	fpc1020->gpio_state_active =
		pinctrl_lookup_state(fpc1020->ts_pinctrl, "pmx_fp_active");
	if (IS_ERR_OR_NULL(fpc1020->gpio_state_active)) {
		dev_err(dev, "Cannot get active pinstate\n");
		ret = PTR_ERR(fpc1020->gpio_state_active);
		goto err;
	}

	fpc1020->gpio_state_suspend =
		pinctrl_lookup_state(fpc1020->ts_pinctrl, "pmx_fp_suspend");
	if (IS_ERR_OR_NULL(fpc1020->gpio_state_suspend)) {
		dev_err(dev, "Cannot get sleep pinstate\n");
		ret = PTR_ERR(fpc1020->gpio_state_suspend);
		goto err;
	}

	return 0;
err:
	fpc1020->ts_pinctrl = NULL;
	fpc1020->gpio_state_active = NULL;
	fpc1020->gpio_state_suspend = NULL;
	return ret;
}

/* -------------------------------------------------------------------- */
static int fpc1020_pinctrl_select(fpc1020_data_t *fpc1020, bool on)
{
	int ret = 0;
	struct pinctrl_state *pins_state;
	struct device *dev = fpc1020->dev;

	pins_state = on ? fpc1020->gpio_state_active : fpc1020->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(fpc1020->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(dev, "can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(dev, "not a valid '%s' pinstate\n",
			on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return ret;
}
#endif

static int fpc_power_init(fpc1020_data_t *fpc1020)
{
	int ret;
	struct device *dev = fpc1020->dev;
	fpc1020->vdd_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(fpc1020->vdd_ana)) {
		ret = PTR_ERR(fpc1020->vdd_ana);
		dev_err(dev,
			"Regulator get failed vdd_ana ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(fpc1020->vdd_ana) > 0) {
		ret = regulator_set_voltage(fpc1020->vdd_ana,
				1800000,
				1800000);
		if (ret) {
			dev_err(dev,
				"Regulator set failed vdd_ana ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	ret = regulator_enable(fpc1020->vdd_ana);
	if (ret) {
		dev_err(dev,
				"Regulator enable failed ret=%d\n",
				ret);
	}

	return 0;

reg_vdd_put:
	regulator_put(fpc1020->vdd_ana);
	return ret;
}

/* -------------------------------------------------------------------- */
static int fpc1020_suspend(struct device *dev)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	dev_info(fpc1020->dev, "%s\n", __func__);

	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc1020_resume(struct device *dev)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	dev_info(fpc1020->dev, "%s\n", __func__);

	return 0;
}

static int hw_reset(struct fpc1020_data *fpc1020)
{
	gpio_set_value(fpc1020->rst_gpio, 1);
	udelay(FPC1020_RESET_HIGH1_US);

	gpio_set_value(fpc1020->rst_gpio, 0);
	udelay(FPC1020_RESET_LOW_US);

	gpio_set_value(fpc1020->rst_gpio, 1);
	udelay(FPC1020_RESET_HIGH2_US);

	return 0;
}

static ssize_t hw_reset_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		rc = hw_reset(fpc1020);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/*
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1020->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1020->wakeup_enabled = false;
		smp_wmb();
	} else {
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device* device,
			     struct device_attribute* attribute,
			     char* buffer)
{
	struct fpc1020_data* fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device* device,
			     struct device_attribute* attribute,
			     const char* buffer, size_t count)
{
	struct fpc1020_data* fpc1020 = dev_get_drvdata(device);
	dev_dbg(fpc1020->dev, "%s\n", __func__);
	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static struct attribute *attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_irq.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;
	dev_dbg(fpc1020->dev, "%s\n", __func__);

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	smp_rmb();

	if (fpc1020->wakeup_enabled ) {
		wake_lock_timeout(&fpc1020->ttw_wl, msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int rc = 0;
	int irqf = 0;

	fpc1020_data_t *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);
	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data_t\n");
		rc = -ENOMEM;
		goto exit;
	}

	printk(KERN_INFO "%s\n", __func__);

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,irq_gpio",
			&fpc1020->irq_gpio);
	if (rc) {
		dev_err(fpc1020->dev,
				"fpc1020_request_named_gpio (irq) failed.\n");
		goto exit;
	}

	rc = gpio_direction_input(fpc1020->irq_gpio);
	if (rc) {
		dev_err(fpc1020->dev,
				"gpio_direction_input (irq) failed.\n");
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,reset_gpio",
			&fpc1020->rst_gpio);
	if (rc) {
		dev_err(fpc1020->dev,
				"fpc1020_request_named_gpio failed\n");
		goto exit;
	}

	rc = gpio_direction_output(fpc1020->rst_gpio, 1);
	if (rc) {
		dev_err(fpc1020->dev,
				"gpio_direction_output (reset) failed.\n");
		goto exit;
	}

#ifdef USE_PINCTL
	rc = fpc1020_pinctrl_init(fpc1020);
	if (rc)
		goto exit;

	rc = fpc1020_pinctrl_select(fpc1020, true);
	if (rc)
		goto exit;
#endif

	rc = fpc_power_init(fpc1020);
	if (rc) {
		dev_err(fpc1020->dev,
				"fpc_power_init failed.\n");
		goto exit;
	}
	
	rc = hw_reset(fpc1020);
	if (rc) {
		dev_err(fpc1020->dev,
				"hw_reset failed.\n");
		goto exit;
	}

	fpc1020->wakeup_enabled = false;

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(np, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}

	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}

	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
}
/* -------------------------------------------------------------------- */
static int fpc1020_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	sysfs_remove_group(&dev->kobj, &attribute_group);
	wake_lock_destroy(&fpc1020->ttw_wl);
	if (gpio_is_valid(fpc1020->irq_gpio)) {
		devm_free_irq(dev, gpio_to_irq(fpc1020->irq_gpio), fpc1020);
		devm_gpio_free(dev, fpc1020->irq_gpio);
		pr_info("remove irq_gpio\n");
	}
	if (gpio_is_valid(fpc1020->rst_gpio)) {
		devm_gpio_free(dev, fpc1020->rst_gpio);
		pr_info("remove reset_gpio\n");
	}
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static const struct dev_pm_ops fpc1020_dev_pm_ops = {
	.suspend = fpc1020_suspend,
	.resume = fpc1020_resume,
};
static struct platform_driver fpc1020_driver = {
	.driver = {
		.name		= "fpc1020",
		.owner		= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
		.pm = &fpc1020_dev_pm_ops,
	},
	.probe = fpc1020_probe,
	.remove = fpc1020_remove
};
module_platform_driver(fpc1020_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
