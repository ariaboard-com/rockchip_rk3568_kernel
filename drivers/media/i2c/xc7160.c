// SPDX-License-Identifier: GPL-2.0
/*
 * xc7160 driver
 */


#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include "xc7160_regs.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#define XC7160_REG_HIGH_SELECT 0xfffd
#define XC7160_REG_PAGE_SELECT  0xfffe

#define XC7160_CHIP_REG_ID1_ZYK		0xfffb
#define XC7160_CHIP_REG_ID2_ZYK		0xfffc

#define XC7160_CHIP_ID1_ZYK			0x71
#define XC7160_CHIP_ID2_ZYK			0x60

#define SC8238_CHIP_REG_ID1_ZYK		0x3107
#define SC8238_CHIP_REG_ID2_ZYK		0x3108

#define SC8238_CHIP_ID1_ZYK			0x82
#define SC8238_CHIP_ID2_ZYK			0x35

#define XC7160_REG_VALUE_08BIT		1
#define XC7160_REG_VALUE_16BIT		2
#define XC7160_REG_VALUE_24BIT		3

#define	XC7160_EXPOSURE_MIN		4
#define	XC7160_EXPOSURE_STEP		1
#define XC7160_VTS_MAX			0x7fff
#define XC7160_GAIN_MIN		0x10
#define XC7160_GAIN_MAX		0xf8
#define XC7160_GAIN_STEP		1
#define XC7160_GAIN_DEFAULT		0x10



static DEFINE_MUTEX(xc7160_power_mutex);

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define XC7160_NAME			"xc7160"

//#define XC7160_LINK_FREQ_XXX_MHZ	504000000U
//#define XC7160_LINK_FREQ_XXX_MHZ	632000000U
 #define XC7160_LINK_FREQ_XXX_MHZ	576000000U

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define XC7160_PIXEL_RATE		(XC7160_LINK_FREQ_XXX_MHZ *2 *4/10)
#define XC7160_XVCLK_FREQ		24000000

static const struct regval *xc7160_global_regs = xc7160_1080p_t20210908_regs;
static const struct regval *sc8238_global_regs = sensor_30fps_t20210908_initial_regs;
static u32 clkout_enabled_index = 1;

static const char * const xc7160_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define XC7160_NUM_SUPPLIES ARRAY_SIZE(xc7160_supply_names)



struct xc7160_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 colorspace;
	const struct regval *reg_list;
};

struct xc7160 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*mipi_pwr_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[XC7160_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	bool            isp_out_colorbar;
	const struct xc7160_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32 lane_data_num;
};


#define to_xc7160(sd) container_of(sd, struct xc7160, subdev)



static const struct xc7160_mode supported_modes[] = {
	// {
	// 	.width = 3840,
	// 	.height = 2160,
	// 	.max_fps = {
	// 		.numerator = 10000,
	// 		.denominator = 250000,
	// 	},
	// 	.colorspace = V4L2_COLORSPACE_SRGB,
	// 	.exp_def = 0x000c,
	// 	.hts_def = 0x04E0,
	// 	.vts_def = 0x16DA,
	// 	.reg_list =xc7160_4k_t20210826_regs,
	// },
	 {
		.width = 1920,
		.height = 1080,
		.max_fps = {
				.numerator = 10000,
				.denominator = 300000,
		},
		.colorspace = V4L2_COLORSPACE_SRGB,
		.exp_def = 0x000c,
		.hts_def = 0x011E,
		.vts_def = 0x20D0,
		.reg_list = xc7160_1080p_t20210908_regs,
	},

		//driver setting
};

static const s64 link_freq_menu_items[] = {
	XC7160_LINK_FREQ_XXX_MHZ
};

static const char * const xc7160_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int xc7160_write_reg(struct i2c_client *client, u16 reg,
			     u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];


	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int xc7160_write_array(struct i2c_client * client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;
	u8 value = 0;
	struct xc7160* xc7160 = NULL;

	xc7160 = to_xc7160(dev_get_drvdata(&client->dev));

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++){
		value = regs[i].val;
		if(regs[i].addr == 0x2023 && xc7160 != NULL){
			if(xc7160->lane_data_num == 2 )
				value = 0x03; //isp mipi data uses 2 lanes
			else
				value = 0x0f; //isp mipi data uses 4 lanes
		}
		ret = xc7160_write_reg(client, regs[i].addr, XC7160_REG_VALUE_08BIT, value);
		if(ret){
			dev_err(&client->dev,"%s: write xc7160 array reg 0x%02x failed\n",__func__,regs[i].addr);
			return ret;
		}

	}
	return ret;
}

/* Read registers up to 4 at a time */
static int xc7160_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];


	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/* Write registers up to 4 at a time */
static int sc8238_write_reg(struct i2c_client *client, u16 reg,
			     u32 len, u32 val)
{
	struct i2c_msg msg;
	u8 sc8238_buf[4];
	int ret;
	u32 buf_i, val_i;
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	sc8238_buf[0]= reg >> 8;
	sc8238_buf[1]= reg & 0xff;

	while (val_i < 4)
		sc8238_buf[buf_i++] = val_p[val_i++];

	/* Write register address */
	msg.addr = 0x30;
	msg.flags = 0;
	msg.len =3;
	msg.buf = sc8238_buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1){
		dev_err(&client->dev,"%s: xc7160_8238 i2c transfer failed\n",__func__);
		return -EIO;
	}

	return 0;
}

/* Read registers up to 4 at a time */
static int sc8238_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = 0x30;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = 0x30;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int sc8238_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	u32 value;
	struct device* dev = &client->dev;
	int ret = 0;

	value =0;
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++){
		ret = sc8238_write_reg(client, regs[i].addr,
					XC7160_REG_VALUE_08BIT,
					regs[i].val);
		if(ret){
				dev_err(dev,"xc7160_8238 write sc8238 regs array get failed, 0x%02x\n",regs[i].addr);
				return ret;
		}
#ifdef 		FIREFLY_DEBUG
		sc8238_read_reg(client,regs[i].addr,XC7160_REG_VALUE_08BIT, &value);
		if(regs[i].val != value){
			dev_info(dev,"firefly->debug: xc7160_8238 reg0x%02x write and read are different\n",regs[i].addr);
		}
#endif
	}
	return ret;
}



static int xc7160_get_reso_dist(const struct xc7160_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct xc7160_mode *
xc7160_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = xc7160_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int xc7160_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct xc7160 *xc7160 = to_xc7160(sd);
	const struct xc7160_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&xc7160->mutex);

	mode = xc7160_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_YUYV8_2X8;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&xc7160->mutex);
		return -ENOTTY;
#endif
	} else {
		xc7160->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(xc7160->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(xc7160->vblank, vblank_def,
					 XC7160_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&xc7160->mutex);

	return 0;
}

static int xc7160_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct xc7160 *xc7160 = to_xc7160(sd);
	const struct xc7160_mode *mode = xc7160->cur_mode;

	mutex_lock(&xc7160->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&xc7160->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_YUYV8_2X8;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&xc7160->mutex);

	return 0;
}

static int xc7160_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_YUYV8_2X8;

	return 0;
}

static int xc7160_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_YUYV8_2X8/*MEDIA_BUS_FMT_YUYV8_2X8*/)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

//static int xc7160_enable_test_pattern(struct xc7160 *xc7160, u32 pattern)
//{
//	return 0;
//}

static int xc7160_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct xc7160 *xc7160 = to_xc7160(sd);
	const struct xc7160_mode *mode = xc7160->cur_mode;

	mutex_lock(&xc7160->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&xc7160->mutex);

	return 0;
}

#define XC7160_LANES 4
static int xc7160_g_mbus_config(struct v4l2_subdev *sd,
                                 struct v4l2_mbus_config *config)
{
        u32 val = 0;
		u32 xc7160_lanes = 0;
		struct xc7160 *xc7160 = to_xc7160(sd);

		xc7160_lanes = xc7160->lane_data_num;
        val = 1 << (xc7160_lanes - 1) |
        V4L2_MBUS_CSI2_CHANNEL_0 |
        V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

        config->type = V4L2_MBUS_CSI2;
        config->flags = val;

        return 0;
}

static void xc7160_get_module_inf(struct xc7160 *xc7160,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, XC7160_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, xc7160->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, xc7160->len_name, sizeof(inf->base.lens));
}

static long xc7160_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct xc7160 *xc7160 = to_xc7160(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		xc7160_get_module_inf(xc7160, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long xc7160_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = xc7160_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = xc7160_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

#ifdef FIREFLY_DEBUG
static int xc7160_check_isp_reg(struct xc7160 *xc7160)
{
	u32 val;
	int ret;
	struct device *dev = &xc7160->client->dev;

	ret = xc7160_write_reg(xc7160->client, 0xfffd, XC7160_REG_VALUE_08BIT, 0x80);
	if (ret){
		dev_err(dev, "write reg0xfffd failed\n");
		return ret;
	}

	ret = xc7160_write_reg(xc7160->client, 0xfffe, XC7160_REG_VALUE_08BIT, 0x26);
	if (ret){
		dev_err(dev, "write reg0xfffe failed\n");
		return ret;
	}
	ret = xc7160_read_reg(xc7160->client, 0x8012, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x8012=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x8013, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x8013=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x8014, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x8014=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x8015, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x8015=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x8010, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x8010=0x%02x\n", val);

	ret = xc7160_write_reg(xc7160->client, 0xfffe, XC7160_REG_VALUE_08BIT, 0x50);
	if (ret){
		dev_err(dev, "write reg0xfffe failed\n");
		return ret;
	}
	ret = xc7160_read_reg(xc7160->client, 0x0090, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x0090=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x0033, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x0033=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x0058, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x0058=0x%02x\n", val);

	ret = xc7160_write_reg(xc7160->client, 0xfffe, XC7160_REG_VALUE_08BIT, 0x30);
	if (ret){
		dev_err(dev, "write reg0xfffe failed\n");
		return ret;
	}
	ret = xc7160_read_reg(xc7160->client, 0x2f06, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x2f06=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x2f07, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x2f07=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x2f08, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x2f08=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x2f09, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x2f09=0x%02x\n", val);

	ret = xc7160_read_reg(xc7160->client, 0x0028, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x0028=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x0029, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x0029=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x002a, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x002a=0x%02x\n", val);
	ret = xc7160_read_reg(xc7160->client, 0x002b, XC7160_REG_VALUE_08BIT, &val);
	dev_info(dev, "reg0x002b=0x%02x\n", val);

	return ret;

}
#endif // DEBUG


static int xc7160_check_isp_id(struct xc7160 *xc7160,
				   struct i2c_client *client)
{
	struct device *dev = &xc7160->client->dev;
	u32 id = 0;
	int ret;

	ret = xc7160_write_reg(client, XC7160_REG_HIGH_SELECT,XC7160_REG_VALUE_08BIT, 0x80);
	if (ret){
		dev_err(dev, "write XC7160_REG_HIGH_SELECT failed\n");
		return ret;
	}

	ret = xc7160_read_reg(client, XC7160_CHIP_REG_ID1_ZYK	,
			       XC7160_REG_VALUE_08BIT, &id);
	if (id == XC7160_CHIP_ID1_ZYK	) {
		dev_info(dev, "isp chip is xc7160\n");
		ret = xc7160_read_reg(client, XC7160_CHIP_REG_ID2_ZYK	,
		       XC7160_REG_VALUE_08BIT, &id);
		if (id != XC7160_CHIP_ID2_ZYK	) {
			dev_err(dev, "Unexpected sensor of XC7160_CHIP_ID_REG2, id(%06x), ret(%d)\n", id, ret);
			return ret;
		}
	}
	
	return 0;
	
}

static int sc8238_check_sensor_id(struct xc7160 *xc7160,
				   struct i2c_client *client)
{
	struct device *dev = &xc7160->client->dev;
	u32 id = 0;
	int ret;

	ret = xc7160_write_array(xc7160->client, xc7160_i2c_bypass_on_regs);
	if (ret) {
		dev_err(dev, "%s: could not set bypass on registers\n",__func__);
		return ret;
	}

	ret = sc8238_read_reg(client, SC8238_CHIP_REG_ID1_ZYK	,
			       XC7160_REG_VALUE_08BIT, &id);
	if (id == SC8238_CHIP_ID1_ZYK	) {
		//dev_info(dev, "sensor chip is SC8238, id_H is 0x%02x\n",id);
		ret = sc8238_read_reg(client, SC8238_CHIP_REG_ID2_ZYK	,
		       XC7160_REG_VALUE_08BIT, &id);
		if (id != SC8238_CHIP_ID2_ZYK	) {
			dev_err(dev, "Unexpected sensor of SC8238_CHIP_ID_REG2, id(%06x), ret(%d)\n", id, ret);
			return ret;
		}			
		//dev_info(dev, "sensor chip is SC8238, id_L is 0x%02x\n",id);
		dev_info(dev, "sensor chip is SC8238\n");
	}

	ret = xc7160_write_array(client, xc7160_i2c_bypass_off_regs);
	if (ret) {
		dev_err(dev, "%s: could not set bypass off registers\n", __func__);
		return ret;
	}	

	return 0;	
}

static int __xc7160_start_stream(struct xc7160 *xc7160)
{
	int ret, i;
	struct device *dev = &xc7160->client->dev;

	//driver setting
	//xc7160_global_regs =xc7160_1080p_t20210831_regs;
	ret = xc7160_write_array(xc7160->client, xc7160_global_regs);
	if(ret){
		dev_err(dev, "isp xc7160 initial failed\n");
		goto lock_and_return;
	}

	xc7160->isp_out_colorbar = false;
	ret=sc8238_check_sensor_id(xc7160, xc7160->client);
	if (ret){
		dev_err(dev, "check sensor sc8238 id failed, color bar mode may be output!!!\n");
		xc7160->isp_out_colorbar = true;
	}

	if(xc7160->isp_out_colorbar == true){
		dev_err(dev, "output color bar\n");
		ret = xc7160_write_array(xc7160->client, xc7160_colorbar_on_regs);
	}else{
	//driver setting
	//sc8238_global_regs = sensor_30fps_t20210831_initial_regs;
		ret = xc7160_write_array(xc7160->client, xc7160_i2c_bypass_on_regs);
		if (ret)
			dev_err(dev, "could not set bypass on registers\n");

		for (i = 0; i <= 3; i++){
			ret = sc8238_write_array(xc7160->client, sc8238_global_regs);
			if (!ret)
				break;
		}
		xc7160_write_array(xc7160->client, xc7160_i2c_bypass_off_regs);
		if (ret){
			dev_err(dev, "failed to initialize sc8238 register, output color bar\n");
			ret = xc7160_write_array(xc7160->client, xc7160_colorbar_on_regs);
		}
	}

	if(ret)
		dev_err(dev, "xc7160 write stream or colorbar regs failed\n");

#ifdef FIREFLY_DEBUG
		xc7160_check_isp_reg(xc7160);
#endif // DEBUG

lock_and_return:
	/* In case these controls are set before streaming */
	mutex_unlock(&xc7160->mutex);
	ret = v4l2_ctrl_handler_setup(&xc7160->ctrl_handler);
	mutex_lock(&xc7160->mutex);
	if (ret)
		return ret;

	return 0;
}

static int __xc7160_stop_stream(struct xc7160 *xc7160)
{
	int ret;

	ret = xc7160_write_array(xc7160->client, xc7160_stream_off_regs);
	if(ret)
		printk("%s: write stream off failed\n",__func__);

	return ret;
}


static int __xc7160_power_on(struct xc7160 *xc7160);
static void __xc7160_power_off(struct xc7160 *xc7160);
static int xc7160_s_power(struct v4l2_subdev *sd, int on)
{
	struct xc7160 *xc7160 = to_xc7160(sd);
	struct i2c_client *client = xc7160->client;
	struct device *dev = &xc7160->client->dev;
	int ret = 0;
	
	mutex_lock(&xc7160->mutex);

	/* If the power state is not modified - no work to do. */
	if (xc7160->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __xc7160_power_on(xc7160);
		if(ret){
			dev_err(dev, "xc7160 power on failed\n");
			goto unlock_and_return;
		}
		xc7160->power_on = true;

		ret = xc7160_check_isp_id(xc7160,xc7160->client);
		if (ret){
			dev_err(dev, "write XC7160_REG_HIGH_SELECT failed\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}


		/* export gpio */
		if (!IS_ERR(xc7160->reset_gpio))
			gpiod_export(xc7160->reset_gpio, false);
		if (!IS_ERR(xc7160->pwdn_gpio))
			gpiod_export(xc7160->pwdn_gpio, false);
	} else {
		pm_runtime_put(&client->dev);
		__xc7160_power_off(xc7160);
		xc7160->power_on = false;
		/* unexport gpio */
		if (!IS_ERR(xc7160->reset_gpio))
			gpiod_unexport(xc7160->reset_gpio);
		if (!IS_ERR(xc7160->pwdn_gpio))
			gpiod_unexport(xc7160->pwdn_gpio);

	}

unlock_and_return:
	//ret = xc7160_write_array(xc7160->client, xc7160_i2c_bypass_off_regs);
	mutex_unlock(&xc7160->mutex);

	return ret;
}


static int xc7160_s_stream(struct v4l2_subdev *sd, int on)
{
	struct xc7160 *xc7160 = to_xc7160(sd);
	struct i2c_client *client = xc7160->client;
	int ret = 0;


	mutex_lock(&xc7160->mutex);
	on = !!on;
	if (on == xc7160->streaming){
		goto unlock_and_return;
	}
	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __xc7160_start_stream(xc7160);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__xc7160_stop_stream(xc7160);
		pm_runtime_put(&client->dev);
	}

	xc7160->streaming = on;

unlock_and_return:
	mutex_unlock(&xc7160->mutex);

	return ret;
}


/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 xc7160_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, XC7160_XVCLK_FREQ / 1000 / 1000);
}

static int __xc7160_power_on(struct xc7160 *xc7160)
{
	int ret;
	u32 delay_us;
	struct device *dev = &xc7160->client->dev;
	
	if (!IS_ERR_OR_NULL(xc7160->pins_default)) {
		ret = pinctrl_select_state(xc7160->pinctrl,
					   xc7160->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	if (!IS_ERR(xc7160->reset_gpio))
		gpiod_set_value_cansleep(xc7160->reset_gpio, 0);	


	if (!IS_ERR(xc7160->pwdn_gpio))
		gpiod_set_value_cansleep(xc7160->pwdn_gpio, 0);

	msleep(4);

	if (clkout_enabled_index){
		ret = clk_prepare_enable(xc7160->xvclk);
		if (ret < 0) {
			dev_err(dev, "Failed to enable xvclk\n");
			return ret;
		}
	}

	ret = regulator_bulk_enable(XC7160_NUM_SUPPLIES, xc7160->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(xc7160->mipi_pwr_gpio))
		gpiod_set_value_cansleep(xc7160->mipi_pwr_gpio, 1);


	usleep_range(500, 1000);
	if (!IS_ERR(xc7160->reset_gpio))
		gpiod_set_value_cansleep(xc7160->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(xc7160->pwdn_gpio))
		gpiod_set_value_cansleep(xc7160->pwdn_gpio, 1);

	//  msleep(25);
	// /* 8192 cycles prior to first SCCB transaction */
	delay_us = xc7160_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	// xc7160->power_on = true;

	return 0;

disable_clk:
	if (clkout_enabled_index)
		clk_disable_unprepare(xc7160->xvclk);

	return ret;
}

static void __xc7160_power_off(struct xc7160 *xc7160)
{
	int ret;
	struct device *dev = &xc7160->client->dev;

	if (!IS_ERR(xc7160->reset_gpio))
		gpiod_set_value_cansleep(xc7160->reset_gpio, 1);
	if (!IS_ERR(xc7160->pwdn_gpio))
		gpiod_set_value_cansleep(xc7160->pwdn_gpio,1);
	if (!IS_ERR(xc7160->mipi_pwr_gpio))
		gpiod_set_value_cansleep(xc7160->mipi_pwr_gpio,1);
	if (clkout_enabled_index)
		clk_disable_unprepare(xc7160->xvclk);
	if (!IS_ERR_OR_NULL(xc7160->pins_sleep)) {
		ret = pinctrl_select_state(xc7160->pinctrl,
					   xc7160->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(XC7160_NUM_SUPPLIES, xc7160->supplies);
	
}
static int xc7160_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7160 *xc7160 = to_xc7160(sd);

	if(xc7160->power_on == false)
		return __xc7160_power_on(xc7160);
	else
		printk("xc7160 is power on, nothing to do\n");

	return 0;
}

static int xc7160_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7160 *xc7160 = to_xc7160(sd);

	if(xc7160->power_on == true){
		__xc7160_power_off(xc7160);
		xc7160->power_on = false;
	}
	return 0;
}



static int xc7160_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_YUYV8_2X8)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int xc7160_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct xc7160 *xc7160 = to_xc7160(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct xc7160_mode *def_mode = &supported_modes[0];

	mutex_lock(&xc7160->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_YUYV8_2X8;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&xc7160->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static const struct dev_pm_ops xc7160_pm_ops = {
	SET_RUNTIME_PM_OPS(xc7160_runtime_suspend,
			   xc7160_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops xc7160_internal_ops = {
	.open = xc7160_open,
};
#endif

static const struct v4l2_subdev_core_ops xc7160_core_ops = {
        .log_status = v4l2_ctrl_subdev_log_status,
        .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
        .unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.s_power = xc7160_s_power,
	.ioctl = xc7160_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = xc7160_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops xc7160_video_ops = {
	.s_stream = xc7160_s_stream,
	.g_frame_interval = xc7160_g_frame_interval,
	.g_mbus_config = xc7160_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops xc7160_pad_ops = {
	.enum_mbus_code = xc7160_enum_mbus_code,
	.enum_frame_size = xc7160_enum_frame_sizes,
	.enum_frame_interval = xc7160_enum_frame_interval,
	.get_fmt = xc7160_get_fmt,
	.set_fmt = xc7160_set_fmt,
};

static const struct v4l2_subdev_ops xc7160_subdev_ops = {
	.core	= &xc7160_core_ops,
	.video	= &xc7160_video_ops,
	.pad	= &xc7160_pad_ops,
};

static int xc7160_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct xc7160 *xc7160 = container_of(ctrl->handler,
					     struct xc7160, ctrl_handler);
	struct i2c_client *client = xc7160->client;
	s64 max;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = xc7160->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(xc7160->exposure,
					 xc7160->exposure->minimum, max,
					 xc7160->exposure->step,
					 xc7160->exposure->default_value);
		break;
	}
	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	pm_runtime_put(&client->dev);
	return 0;
}

static const struct v4l2_ctrl_ops xc7160_ctrl_ops = {
	.s_ctrl = xc7160_set_ctrl,
};

static int xc7160_initialize_controls(struct xc7160 *xc7160)
{
	const struct xc7160_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &xc7160->ctrl_handler;
	mode = xc7160->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &xc7160->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, XC7160_PIXEL_RATE, 1, XC7160_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	xc7160->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (xc7160->hblank)
		xc7160->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	xc7160->vblank = v4l2_ctrl_new_std(handler, &xc7160_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				XC7160_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	xc7160->exposure = v4l2_ctrl_new_std(handler, &xc7160_ctrl_ops,
				V4L2_CID_EXPOSURE, XC7160_EXPOSURE_MIN,
				exposure_max, XC7160_EXPOSURE_STEP,
				mode->exp_def);

	xc7160->anal_gain = v4l2_ctrl_new_std(handler, &xc7160_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, XC7160_GAIN_MIN,
				XC7160_GAIN_MAX, XC7160_GAIN_STEP,
				XC7160_GAIN_DEFAULT);

	xc7160->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&xc7160_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(xc7160_test_pattern_menu) - 1,
				0, 0, xc7160_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&xc7160->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	xc7160->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}



static int xc7160_configure_regulators(struct xc7160 *xc7160)
{
	unsigned int i;

	for (i = 0; i < XC7160_NUM_SUPPLIES; i++)
		xc7160->supplies[i].supply = xc7160_supply_names[i];

	return devm_regulator_bulk_get(&xc7160->client->dev,
				       XC7160_NUM_SUPPLIES,
				       xc7160->supplies);
}

static void free_gpio(struct xc7160 *xc7160)
{
	if (!IS_ERR(xc7160->pwdn_gpio))
		gpio_free(desc_to_gpio(xc7160->pwdn_gpio));
        if (!IS_ERR(xc7160->reset_gpio))
                gpio_free(desc_to_gpio(xc7160->reset_gpio));
        if (!IS_ERR(xc7160->mipi_pwr_gpio))
		gpio_free(desc_to_gpio(xc7160->mipi_pwr_gpio));
}

static int xc7160_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct device_node *endpoint_node = NULL;
	struct v4l2_fwnode_endpoint vep;
	struct xc7160 *xc7160;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	//u32 val = 0;

	dev_info(dev, "T-chip firefly camera driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	xc7160 = devm_kzalloc(dev, sizeof(*xc7160), GFP_KERNEL);
	if (!xc7160)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &xc7160->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &xc7160->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &xc7160->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &xc7160->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	xc7160->client = client;
	xc7160->cur_mode = &supported_modes[0];

	if (clkout_enabled_index){
		xc7160->xvclk = devm_clk_get(dev, "xvclk");
		if (IS_ERR(xc7160->xvclk)) {
			dev_err(dev, "Failed to get xvclk\n");
			return -EINVAL;
		}
		ret = clk_set_rate(xc7160->xvclk, XC7160_XVCLK_FREQ);
		if (ret < 0) {
			dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
			return ret;
		}
		if (clk_get_rate(xc7160->xvclk) != XC7160_XVCLK_FREQ)
			dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	}

	xc7160->mipi_pwr_gpio = devm_gpiod_get(dev, "mipi-pwr", GPIOD_OUT_LOW);
	if (IS_ERR(xc7160->mipi_pwr_gpio))
		dev_warn(dev, "Failed to get power-gpios, maybe no use\n");

	xc7160->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(xc7160->reset_gpio)) {
	   dev_info(dev, "Failed to get reset-gpios, maybe no use\n");
	}

	xc7160->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(xc7160->pwdn_gpio)) {
	  dev_info(dev, "Failed to get pwdn-gpios, maybe no use\n");
	}

	ret = xc7160_configure_regulators(xc7160);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	xc7160->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(xc7160->pinctrl)) {
		xc7160->pins_default =
			pinctrl_lookup_state(xc7160->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(xc7160->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		xc7160->pins_sleep =
			pinctrl_lookup_state(xc7160->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(xc7160->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	xc7160->isp_out_colorbar = false;
	endpoint_node = of_find_node_by_name(node,"endpoint");
	if(endpoint_node != NULL){
		//printk("xc7160 get endpoint node success\n");
		ret=v4l2_fwnode_endpoint_parse(&endpoint_node->fwnode, &vep);
		if(ret){
			dev_info(dev, "Failed to get xc7160 endpoint data lanes, set a default value\n");
			xc7160->lane_data_num = 4;
		}else{
			dev_info(dev, "Success to get xc7160 endpoint data lanes, dts uses %d lanes\n", vep.bus.mipi_csi2.num_data_lanes);
			xc7160->lane_data_num = vep.bus.mipi_csi2.num_data_lanes;
		}
	}else{
		dev_info(dev,"xc7160 get endpoint node failed\n");
		return -ENOENT;
	}
	dev_info(dev,"xc7160 num data lanes is %d\n", xc7160->lane_data_num);

	mutex_init(&xc7160->mutex);

	sd = &xc7160->subdev;
	v4l2_i2c_subdev_init(sd, client, &xc7160_subdev_ops);
	ret = xc7160_initialize_controls(xc7160);
	if (ret)
		goto err_destroy_mutex;

	ret = __xc7160_power_on(xc7160);
	if (ret) {
		dev_err(dev, "--xc--__xc7160_power_on failed\n");
		goto err_power_off;
	}

	ret = xc7160_check_isp_id(xc7160, client);
	if (ret)
		goto err_power_off;

	ret=sc8238_check_sensor_id(xc7160, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &xc7160_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	xc7160->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &xc7160->pad);
	if (ret < 0)
		goto err_power_off;
	
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(xc7160->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 xc7160->module_index, facing,
		 XC7160_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__xc7160_power_off(xc7160);
	free_gpio(xc7160);
//err_free_handler:
	v4l2_ctrl_handler_free(&xc7160->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&xc7160->mutex);

	return ret;
}

static int xc7160_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7160 *xc7160 = to_xc7160(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&xc7160->ctrl_handler);
	mutex_destroy(&xc7160->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__xc7160_power_off(xc7160);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id xc7160_of_match[] = {
	{ .compatible = "firefly,xc7160" },
	{},
};
MODULE_DEVICE_TABLE(of, xc7160_of_match);
#endif

static const struct i2c_device_id xc7160_match_id[] = {
	{ "firefly,xc7160", 0 },
	{ },
};

static struct i2c_driver xc7160_i2c_driver = {
	.driver = {
		.name = "xc7160",
		.pm = &xc7160_pm_ops,
		.of_match_table = of_match_ptr(xc7160_of_match),
	},
	.probe		= &xc7160_probe,
	.remove		= &xc7160_remove,
	.id_table	= xc7160_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&xc7160_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&xc7160_i2c_driver);
}

late_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision xc7160 sensor driver");
MODULE_LICENSE("GPL v2");
