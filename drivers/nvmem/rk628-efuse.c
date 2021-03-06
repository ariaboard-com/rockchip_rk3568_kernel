// SPDX-License-Identifier: GPL-2.0
/*
 * RK628 eFuse Driver
 *
 * Copyright (c) 2020 Rockchip Electronics Co. Ltd.
 *
 * Author: Weixin Zhou <zwx@rock-chips.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/rk628.h>
#include <linux/mfd/rk630.h>

#define EFUSE_SIZE		64

#define T_CSB_P_S		0
#define T_PGENB_P_S		(15 + 200)
#define T_LOAD_P_S		0
#define T_ADDR_P_S		(15 + 200 + 5)
#define T_STROBE_P_S		((150 + 2000 + 100) / 9)
#define T_CSB_P_L		0
#define T_PGENB_P_L		(15 + 200 + 10 + 200 + 190 + 10)
#define T_LOAD_P_L		(15 + 200 + 200 + 190 + 10 + 100 + 15)
#define T_ADDR_P_L		(15 + 200 + 5 + 200 + 5)
#define T_STROBE_P_L		((150 + 2000 + 100 + 2000) / 9)
#define T_CSB_R_S		0
#define T_PGENB_R_S		0
#define T_LOAD_R_S		15
#define T_ADDR_R_S		(15 + 9)
#define T_STROBE_R_S		((150 + 100) / 9)
#define T_CSB_R_L		0
#define T_PGENB_R_L		0
#define T_LOAD_R_L		(15 + 5 + 5 + 10 + 15)
#define T_ADDR_R_L		(15 + 10 + 5 + 1)
#define T_STROBE_R_L		((150 + 100 + 50) / 8)

#define T_CSB_P			0x28
#define T_PGENB_P		0x2c
#define T_LOAD_P		0x30
#define T_ADDR_P		0x34
#define T_STROBE_P		0x38
#define T_CSB_R			0x3c
#define T_PGENB_R		0x40
#define T_LOAD_R		0x44
#define T_ADDR_R		0x48
#define T_STROBE_R		0x4c
#define EFUSE_REVISION		0x50

#define RK628_EFUSE_BASE	0xb0000
#define RK630_EFUSE_BASE	0x50000
#define RK628_MOD		0x00
#define RK628_INT_STATUS	0x0018
#define RK628_DOUT		0x0020
#define RK628_AUTO_CTRL		0x0024
#define RK628_USER_MODE		BIT(0)
#define RK628_INT_FINISH	BIT(0)
#define RK628_AUTO_ENB		BIT(0)
#define RK628_AUTO_RD		BIT(1)
#define RK628_ADDR_ROW		16
#define RK628_ADDR_COL		22
#define RK628_A_SHIFT		16
#define RK628_A_MASK		0x3ff
#define RK628_NBYTES		1

#define REG_EFUSE_CTRL		0x0000
#define REG_EFUSE_DOUT		0x0004

enum {
	RK628_EFUSE,
	RK630_EFUSE,
};

struct rk6xx_efuse_plat_data {
	int device_type;
	struct nvmem_config *econfig;
};

struct rk628_efuse_chip {
	struct device *dev;
	u32 base;
	struct clk *clk;
	struct regmap *regmap;
	struct regmap *cru;
	struct gpio_desc *avdd_gpio;
};

static int rk628_read(struct rk628_efuse_chip *efuse, u32 reg)
{
	int ret;
	u32 val;
	struct regmap *regmap = efuse->regmap;

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(efuse->dev, "failed to read reg 0x%x\n", reg);
		return ret;
	}

	return val;
}

static int rk628_write(struct rk628_efuse_chip *efuse, u32 val, u32 reg)
{
	int ret;
	struct regmap *regmap = efuse->regmap;

	ret = regmap_write(regmap, reg, val);
	if (ret)
		dev_err(efuse->dev, "failed to write reg 0x%x\n", reg);

	return ret;
}

static void rk628_efuse_timing_init(struct rk628_efuse_chip *efuse)
{
	u32 base = efuse->base;
	/* enable auto mode */
	rk628_write(efuse,
		    rk628_read(efuse, base + RK628_MOD) & (~RK628_USER_MODE),
		    base + RK628_MOD);

	/* setup efuse timing */
	rk628_write(efuse, (T_CSB_P_S << 16) | T_CSB_P_L, base + T_CSB_P);
	rk628_write(efuse, (T_PGENB_P_S << 16) | T_PGENB_P_L, base + T_PGENB_P);
	rk628_write(efuse, (T_LOAD_P_S << 16) | T_LOAD_P_L, base + T_LOAD_P);
	rk628_write(efuse, (T_ADDR_P_S << 16) | T_ADDR_P_L, base + T_ADDR_P);
	rk628_write(efuse, (T_STROBE_P_S << 16) | T_STROBE_P_L, base + T_STROBE_P);
	rk628_write(efuse, (T_CSB_R_S << 16) | T_CSB_R_L, base + T_CSB_R);
	rk628_write(efuse, (T_PGENB_R_S << 16) | T_PGENB_R_L, base + T_PGENB_R);
	rk628_write(efuse, (T_LOAD_R_S << 16) | T_LOAD_R_L, base + T_LOAD_R);
	rk628_write(efuse, (T_ADDR_R_S << 16) | T_ADDR_R_L, base + T_ADDR_R);
	rk628_write(efuse, (T_STROBE_R_S << 16) | T_STROBE_R_L, base + T_STROBE_R);
}

static void rk628_efuse_timing_deinit(struct rk628_efuse_chip *efuse)
{
	u32 base = efuse->base;
	/* disable auto mode */
	rk628_write(efuse,
		    rk628_read(efuse, base + RK628_MOD) | RK628_USER_MODE, base + RK628_MOD);

	/* clear efuse timing */
	rk628_write(efuse, 0, base + T_CSB_P);
	rk628_write(efuse, 0, base + T_PGENB_P);
	rk628_write(efuse, 0, base + T_LOAD_P);
	rk628_write(efuse, 0, base + T_ADDR_P);
	rk628_write(efuse, 0, base + T_STROBE_P);
	rk628_write(efuse, 0, base + T_CSB_R);
	rk628_write(efuse, 0, base + T_PGENB_R);
	rk628_write(efuse, 0, base + T_LOAD_R);
	rk628_write(efuse, 0, base + T_ADDR_R);
	rk628_write(efuse, 0, base + T_STROBE_R);
}

static int rk628_efuse_read(void *context, unsigned int offset,
				      void *val, size_t bytes)
{
	struct rk628_efuse_chip *efuse = context;
	unsigned int addr_start, addr_end, addr_offset, addr_len;
	u32 out_value, status;
	u8 *buf;
	int ret = 0, i = 0;

	if (efuse->clk) {
		ret = clk_prepare_enable(efuse->clk);
		if (ret < 0) {
			dev_err(efuse->dev, "failed to prepare/enable efuse pclk\n");
			return ret;
		}
	} else {
		regmap_write(efuse->cru, CRU_GATE_CON0, PCLK_EFUSE_EN_MASK);
	}

	addr_start = rounddown(offset, RK628_NBYTES) / RK628_NBYTES;
	addr_end = roundup(offset + bytes, RK628_NBYTES) / RK628_NBYTES;
	addr_offset = offset % RK628_NBYTES;
	addr_len = addr_end - addr_start;

	buf = kzalloc(sizeof(*buf) * addr_len * RK628_NBYTES, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto nomem;
	}

	rk628_efuse_timing_init(efuse);

	while (addr_len--) {
		rk628_write(efuse, RK628_AUTO_RD | RK628_AUTO_ENB |
		       ((addr_start++ & RK628_A_MASK) << RK628_A_SHIFT),
		       efuse->base + RK628_AUTO_CTRL);
		udelay(2);
		status = rk628_read(efuse, efuse->base + RK628_INT_STATUS);
		if (!(status & RK628_INT_FINISH)) {
			ret = -EIO;
			goto err;
		}
		out_value = rk628_read(efuse, efuse->base + RK628_DOUT);
		rk628_write(efuse, RK628_INT_FINISH, efuse->base + RK628_INT_STATUS);

		memcpy(&buf[i], &out_value, RK628_NBYTES);
		i += RK628_NBYTES;
	}
	memcpy(val, buf + addr_offset, bytes);
err:
	rk628_efuse_timing_deinit(efuse);
	kfree(buf);
nomem:
	if (efuse->clk)
		clk_disable_unprepare(efuse->clk);
	else
		regmap_write(efuse->cru, CRU_GATE_CON0, PCLK_EFUSE_EN_MASK | PCLK_EFUSE_DISABLE);

	return ret;
}

static struct nvmem_config rk628_econfig = {
	.name = "rk628-efuse",
	.owner = THIS_MODULE,
	.stride = 1,
	.word_size = 1,
	.read_only = true,
};

static struct nvmem_config rk630_econfig = {
	.name = "rk630-efuse",
	.owner = THIS_MODULE,
	.stride = 1,
	.word_size = 1,
	.read_only = true,
};

static const struct regmap_range rk628_efuse_readable_ranges[] = {
	regmap_reg_range(RK628_EFUSE_BASE, RK628_EFUSE_BASE + EFUSE_REVISION),
};

static const struct regmap_access_table rk628_efuse_readable_table = {
	.yes_ranges     = rk628_efuse_readable_ranges,
	.n_yes_ranges   = ARRAY_SIZE(rk628_efuse_readable_ranges),
};

static const struct regmap_config rk628_efuse_regmap_config = {
	.name = "rk628-efuse",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = RK628_EFUSE_BASE + EFUSE_REVISION,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &rk628_efuse_readable_table,
};

static const struct regmap_range rk630_efuse_readable_ranges[] = {
	regmap_reg_range(RK630_EFUSE_BASE, RK630_EFUSE_BASE + EFUSE_REVISION),
};

static const struct regmap_access_table rk630_efuse_readable_table = {
	.yes_ranges     = rk630_efuse_readable_ranges,
	.n_yes_ranges   = ARRAY_SIZE(rk630_efuse_readable_ranges),
};

const struct regmap_config rk630_efuse_regmap_config = {
	.name = "rk630-efuse",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = RK630_EFUSE_BASE + EFUSE_REVISION,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &rk630_efuse_readable_table,
};
EXPORT_SYMBOL_GPL(rk630_efuse_regmap_config);

static const struct rk6xx_efuse_plat_data rk628_efuse_drv_data = {
	.device_type = RK628_EFUSE,
	.econfig = &rk628_econfig,
};

static const struct rk6xx_efuse_plat_data rk630_efuse_drv_data = {
	.device_type = RK630_EFUSE,
	.econfig = &rk630_econfig,
};

static const struct of_device_id rk628_efuse_match[] = {
	{
		.compatible = "rockchip,rk628-efuse",
		.data = &rk628_efuse_drv_data
	},
	{
		.compatible = "rockchip,rk630-efuse",
		.data = &rk630_efuse_drv_data
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, rk628_efuse_match);

static int rk628_efuse_probe(struct platform_device *pdev)
{
	struct nvmem_device *nvmem;
	struct rk628_efuse_chip *efuse;
	struct device *dev = &pdev->dev;
	struct rk6xx_efuse_plat_data *plat_data;
	const struct of_device_id *match;
	int ret;

	efuse = devm_kzalloc(&pdev->dev, sizeof(struct rk628_efuse_chip),
			     GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;

	match = of_match_node(rk628_efuse_match, pdev->dev.of_node);
	plat_data = (struct rk6xx_efuse_plat_data *)match->data;
	if (!plat_data)
		return -ENOMEM;

	if (plat_data->device_type == RK628_EFUSE) {
		struct rk628 *rk628 = dev_get_drvdata(pdev->dev.parent);

		efuse->regmap = devm_regmap_init_i2c(rk628->client,
						     &rk628_efuse_regmap_config);
		if (IS_ERR(efuse->regmap)) {
			ret = PTR_ERR(efuse->regmap);
			dev_err(dev, "failed to allocate register map: %d\n",
					   ret);
			return ret;
		}

		efuse->clk = devm_clk_get(&pdev->dev, "pclk");
		if (IS_ERR(efuse->clk)) {
			dev_err(dev, "failed to get pclk: %ld\n", PTR_ERR(efuse->clk));
			return PTR_ERR(efuse->clk);
		}

		efuse->base = RK628_EFUSE_BASE;
	} else {
		struct rk630 *rk630 = dev_get_drvdata(pdev->dev.parent);

		efuse->regmap = rk630->efuse;
		efuse->cru = rk630->cru;
		efuse->base = RK630_EFUSE_BASE;

		if (!efuse->regmap | !efuse->cru)
			return -ENODEV;

		efuse->clk = NULL;
	}

	efuse->avdd_gpio = devm_gpiod_get_optional(dev, "efuse", GPIOD_OUT_LOW);
	efuse->dev = &pdev->dev;
	plat_data->econfig->size = EFUSE_SIZE;
	plat_data->econfig->reg_read = (void *)&rk628_efuse_read;
	plat_data->econfig->priv = efuse;
	plat_data->econfig->dev = efuse->dev;
	nvmem = devm_nvmem_register(dev, plat_data->econfig);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	platform_set_drvdata(pdev, nvmem);

	return 0;
}

static struct platform_driver rk628_efuse_driver = {
	.probe = rk628_efuse_probe,
	.driver = {
		.name = "rk628-efuse",
		.of_match_table = rk628_efuse_match,
	},
};

module_platform_driver(rk628_efuse_driver);

MODULE_DESCRIPTION("rk628_efuse driver");
MODULE_LICENSE("GPL v2");
