/*
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define IPROC_PWM_CTRL_OFFSET			0x00
#define IPROC_PWM_CTRL_TYPE_SHIFT(ch)		(15 + (ch))
#define IPROC_PWM_CTRL_POLARITY_SHIFT(ch)	(8 + (ch))
#define IPROC_PWM_CTRL_EN_SHIFT(ch)		(ch)

#define IPROC_PWM_PERIOD_OFFSET(ch)		(0x04 + ((ch) << 3))
#define IPROC_PWM_PERIOD_MIN			0x02
#define IPROC_PWM_PERIOD_MAX			0xffff

#define IPROC_PWM_DUTY_CYCLE_OFFSET(ch)		(0x08 + ((ch) << 3))
#define IPROC_PWM_DUTY_CYCLE_MIN		0x00
#define IPROC_PWM_DUTY_CYCLE_MAX		0xffff

#define IPROC_PWM_PRESCALE_OFFSET		0x24
#define IPROC_PWM_PRESCALE_BITS			0x06
#define IPROC_PWM_PRESCALE_SHIFT(ch)		((3 - (ch)) * \
						 IPROC_PWM_PRESCALE_BITS)
#define IPROC_PWM_PRESCALE_MASK(ch)		(IPROC_PWM_PRESCALE_MAX << \
						 IPROC_PWM_PRESCALE_SHIFT(ch))
#define IPROC_PWM_PRESCALE_MIN			0x00
#define IPROC_PWM_PRESCALE_MAX			0x3f

struct iproc_pwmc {
	struct pwm_chip chip;
	void __iomem *base;
	struct clk *clk;
};

static inline struct iproc_pwmc *to_iproc_pwmc(struct pwm_chip *chip)
{
	return container_of(chip, struct iproc_pwmc, chip);
}

static int iproc_pwmc_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	u32 value;

	value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);
	value |= 1 << IPROC_PWM_CTRL_EN_SHIFT(pwm->hwpwm);
	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);

	/* must be a 400 ns delay between clearing and setting enable bit */
	ndelay(400);

	return 0;
}

static void iproc_pwmc_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	u32 value;

	value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);
	value &= ~(1 << IPROC_PWM_CTRL_EN_SHIFT(pwm->hwpwm));
	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);

	/* must be a 400 ns delay between clearing and setting enable bit */
	ndelay(400);
}

static int iproc_pwmc_set_polarity(struct pwm_chip *chip,
				   struct pwm_device *pwm,
				   enum pwm_polarity polarity)
{
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	u32 value;

	/* set polarity */
	value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);

	if (polarity == PWM_POLARITY_NORMAL)
		value |= 1 << IPROC_PWM_CTRL_POLARITY_SHIFT(pwm->hwpwm);
	else
		value &= ~(1 << IPROC_PWM_CTRL_POLARITY_SHIFT(pwm->hwpwm));

	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);

	return 0;
}

static int iproc_pwmc_config(struct pwm_chip *chip, struct pwm_device *pwm,
		      int duty_ns, int period_ns)
{
	unsigned long prescale = IPROC_PWM_PRESCALE_MIN;
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	u32 value, period, duty;
	u64 rate;

	rate = clk_get_rate(ip->clk);

	/*
	 * Find period count, duty count and prescale to suit duty_cycle and
	 * period. This is done according to formulas described below:
	 *
	 * period_ns = 10^9 * (PRESCALE + 1) * PC / PWM_CLK_RATE
	 * duty_ns = 10^9 * (PRESCALE + 1) * DC / PWM_CLK_RATE
	 *
	 * PC = (PWM_CLK_RATE * period_ns) / (10^9 * (PRESCALE + 1))
	 * DC = (PWM_CLK_RATE * duty_ns) / (10^9 * (PRESCALE + 1))
	 */
	while (1) {
		u64 value, div;

		div = NSEC_PER_SEC * (prescale + 1);
		value = rate * period_ns;
		period = div64_u64(value, div);
		value = rate * duty_ns;
		duty = div64_u64(value, div);

		if (period < IPROC_PWM_PERIOD_MIN ||
		    duty < IPROC_PWM_DUTY_CYCLE_MIN)
			return -EINVAL;

		if (period <= IPROC_PWM_PERIOD_MAX &&
		     duty <= IPROC_PWM_DUTY_CYCLE_MAX)
			break;

		/* Otherwise, increase prescale and recalculate counts */
		if (++prescale > IPROC_PWM_PRESCALE_MAX)
			return -EINVAL;
	}

	iproc_pwmc_disable(chip, pwm);

	/* Set prescale */
	value = readl(ip->base + IPROC_PWM_PRESCALE_OFFSET);
	value &= ~IPROC_PWM_PRESCALE_MASK(pwm->hwpwm);
	value |= prescale << IPROC_PWM_PRESCALE_SHIFT(pwm->hwpwm);
	writel(value, ip->base + IPROC_PWM_PRESCALE_OFFSET);

	/* set period and duty cycle */
	writel(period, ip->base + IPROC_PWM_PERIOD_OFFSET(pwm->hwpwm));
	writel(duty, ip->base + IPROC_PWM_DUTY_CYCLE_OFFSET(pwm->hwpwm));

	return 0;
}

static const struct pwm_ops iproc_pwm_ops = {
	.config = iproc_pwmc_config,
	.set_polarity = iproc_pwmc_set_polarity,
	.enable = iproc_pwmc_enable,
	.disable = iproc_pwmc_disable,
	.owner = THIS_MODULE,
};

static void iproc_pwmc_hw_set(struct iproc_pwmc *ip)
{
	unsigned int i;
	u32 value;

	/* Set full drive and normal polarity for all channels */
	value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);

	for (i = 0; i < ip->chip.npwm; i++) {
		value &= ~(1 << IPROC_PWM_CTRL_TYPE_SHIFT(i));
		value |= 1 << IPROC_PWM_CTRL_POLARITY_SHIFT(i);
	}
	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);
}

static int iproc_pwmc_probe(struct platform_device *pdev)
{
	struct iproc_pwmc *ip;
	struct resource *res;
	int ret;

	ip = devm_kzalloc(&pdev->dev, sizeof(*ip), GFP_KERNEL);
	if (!ip)
		return -ENOMEM;

	platform_set_drvdata(pdev, ip);

	ip->chip.dev = &pdev->dev;
	ip->chip.ops = &iproc_pwm_ops;
	ip->chip.base = -1;
	ip->chip.npwm = 4;
	ip->chip.of_xlate = of_pwm_xlate_with_flags;
	ip->chip.of_pwm_n_cells = 3;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ip->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ip->base))
		return PTR_ERR(ip->base);

	ip->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ip->clk)) {
		dev_err(&pdev->dev, "failed to get clock: %ld\n",
			PTR_ERR(ip->clk));
		return PTR_ERR(ip->clk);
	}

	ret = clk_prepare_enable(ip->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	iproc_pwmc_hw_set(ip);

	ret = pwmchip_add(&ip->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		clk_disable_unprepare(ip->clk);
	}

	return ret;
}

static int iproc_pwmc_remove(struct platform_device *pdev)
{
	struct iproc_pwmc *ip = platform_get_drvdata(pdev);

	clk_disable_unprepare(ip->clk);

	return pwmchip_remove(&ip->chip);
}

#ifdef CONFIG_PM_SLEEP
static int iproc_pwmc_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* nothing to do on suspend */
	return 0;
}

static int iproc_pwmc_resume(struct platform_device *pdev)
{
	struct iproc_pwmc *ip;

	ip = dev_get_drvdata(&pdev->dev);
	if (!ip)
		return -EINVAL;

	iproc_pwmc_hw_set(ip);
}
#endif /* CONFIG_PM */


static const struct of_device_id bcm_iproc_pwmc_dt[] = {
	{ .compatible = "brcm,iproc-pwm" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_iproc_pwmc_dt);

static struct platform_driver iproc_pwmc_driver = {
	.driver = {
		.name = "bcm-iproc-pwm",
		.of_match_table = bcm_iproc_pwmc_dt,
	},
	.probe = iproc_pwmc_probe,
	.remove = iproc_pwmc_remove,
#ifdef CONFIG_PM_SLEEP
	.suspend = iproc_pwmc_suspend,
	.resume = iproc_pwmc_resume,
#endif
};
module_platform_driver(iproc_pwmc_driver);

MODULE_AUTHOR("Yendapally Reddy Dhananjaya Reddy <yendapally.reddy@broadcom.com>");
MODULE_DESCRIPTION("Broadcom iProc PWM driver");
MODULE_LICENSE("GPL v2");
