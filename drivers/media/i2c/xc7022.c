// SPDX-License-Identifier: GPL-2.0
/*
 * xc7022 driver
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


#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#define XC7022_LINK_FREQ_300MHZ	300000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define XC7022_PIXEL_RATE		(XC7022_LINK_FREQ_300MHZ * 2 * 2 / 10)
#define XC7022_XVCLK_FREQ		24000000

#define CHECK_CHIP_ID_REG1		0xfffd
#define CHECK_CHIP_ID_REG2		0xfffe
#define CHECK_CHIP_ID_VAL		0x80
#define XC7022_CHIP_ID_REG1		0xfffb
#define XC7022_CHIP_ID_REG2		0xfffc
#define XC7022_CHIP_ID1			0x71
#define XC7022_CHIP_ID2			0x60

#define XC6130_CHIP_ID_REG1		0x0002
#define XC6130_CHIP_ID_REG2    		0x0003
#define XC6130_CHIP_ID1			0x43
#define XC6130_CHIP_ID2			0x58

#define XC7022_REG_VALUE_08BIT		1
#define XC7022_REG_VALUE_16BIT		2
#define XC7022_REG_VALUE_24BIT		3

#define	XC7022_EXPOSURE_MIN		4
#define	XC7022_EXPOSURE_STEP		1
#define XC7022_VTS_MAX			0x7fff
#define XC7022_GAIN_MIN		0x10
#define XC7022_GAIN_MAX		0xf8
#define XC7022_GAIN_STEP		1
#define XC7022_GAIN_DEFAULT		0x10

#define REG_NULL			0xFFFF


static DEFINE_MUTEX(xc7022_power_mutex);


#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define XC7022_NAME			"xc7022"

static const struct regval *xc7022_global_regs;
static u32 clkout_enabled_index;

static const char * const xc7022_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define XC7022_NUM_SUPPLIES ARRAY_SIZE(xc7022_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct xc7022_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 colorspace;
	const struct regval *reg_list;
};

struct xc7022 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*mipi_pwr_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[XC7022_NUM_SUPPLIES];

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
	const struct xc7022_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};


#define to_xc7022(sd) container_of(sd, struct xc7022, subdev)

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 600Mbps
 */
static const struct regval xc7022_480p_regs[] = {
	{0xfffd, 0x80}, // AE_avg                                     
	{0xfffe, 0x30}, // AE_avg                                     
	{0x1f04, 0x05}, // WIN width              
	{0x1f05, 0xa0},                           
	{0x1f06, 0x03}, // WIN height             
	{0x1f07, 0x60},                           
	{0x1f08, 0x03},                                      
	{0xfffe, 0x2d}, 
	{0x0003, 0x38}, 
	{0xfffe, 0x26},
	{0x8010, 0x05},
	{0x8012, 0xA0},
	{0x8013, 0x05},
	{0x8016, 0xf0},
	{0xfffe, 0x2c},
	{0x0001, 0x02},
	{0x0002, 0x80},
	{0x0004, 0x01},
	{0x0005, 0xE0},
	{0x0048, 0x04},
	{0x0049, 0xF0},
	{0xfffe, 0x26},
	{0x2019, 0x02},
	{0x201a, 0x80},
	{0x201b, 0x01},
	{0x201c, 0xE0},
	{0xfffe, 0x30},
	{0x0001, 0x92},
	{0x005e, 0x9F},
	{0x005f, 0x05},
	{0x0060, 0x37},
	{0x0061, 0x04},
	{0x0064, 0x80},
	{0x0065, 0x02},
	{0x0066, 0xE0},
	{0x0067, 0x01},
	{0x0006, 0x05},
	{0x0007, 0xA0},
	{0x0008, 0x04},
	{0x0009, 0x38},
	{0x000a, 0x02},
	{0x000b, 0x80},
	{0x000c, 0x01},
	{0x000d, 0xE0},
	{0xfffd, 0x80},
	{0xfffe, 0x50},
	{0x001a, 0x08},
	{0x001a, 0x00},
	{0xfffe, 0x14}, //expose
	{0x00fa, 0x00}, //0x00

	{REG_NULL, 0x00},
};
static const struct regval xc7022_720p_regs[] = {
	{0xfffd, 0x80}, // AE_avg                                     
        {0xfffe, 0x30},   // AE_avg                                     
        {0x1f04, 0x07},   // WIN width              
        {0x1f05, 0x80},
        {0x1f06, 0x03},   // WIN height             
        {0x1f07, 0x60},
        {0x1f08, 0x03},
        {0xfffe, 0x2d},
        {0x0003, 0x39},
        {0xfffe, 0x26},
        {0x8010, 0x05},
        {0x8012, 0x80},
        {0x8013, 0x07},
        {0x8016, 0x00},
        {0xfffe, 0x2c},
        {0x0001, 0x05},
        {0x0002, 0x00},
        {0x0004, 0x02},
        {0x0005, 0xD0},
        {0x0048, 0x09},
        {0x0049, 0xF0},
        {0xfffe, 0x26},
        {0x2019, 0x05},
        {0x201a, 0x00},
        {0x201b, 0x02},
        {0x201c, 0xD0},
        {0xfffe, 0x30},
        {0x0001, 0x92},
        {0x005e, 0x7f},
        {0x005f, 0x07},
        {0x0060, 0x37},
        {0x0061, 0x04},
        {0x0064, 0x00},
        {0x0065, 0x05},
        {0x0066, 0xD0},
        {0x0067, 0x02},
        {0x0006, 0x07},
        {0x0007, 0x80},   
        {0x0008, 0x04},   
        {0x0009, 0x38},
        {0x000a, 0x05},   
        {0x000b, 0x00},
        {0x000c, 0x02},
        {0x000d, 0xD0},
        {0xfffd, 0x80},
        {0xfffe, 0x50},
        {0x001a, 0x08},
        {0x001a, 0x00},

	{REG_NULL, 0x00},
};

static const struct regval xc7022_stream_on_regs[] = {
	{0xfffd, 0x80},
	{0xfffe, 0x26},
	{0x8010, 0x0d},
	{REG_NULL, 0x00},
};

static const struct regval xc7022_stream_off_regs[] = {
	{0xfffd, 0x80},
	{0xfffe, 0x26},
	{0x8010, 0x09},
	{REG_NULL, 0x00},
};

static const struct regval xc6130_480p_regs[] = {
{0xfffd, 0x80},
{0xfffe, 0x21},
{0x0684, 0x05},
{0x0685, 0xa0},
{0x0686, 0x04},
{0x0687, 0x38},
{0xfffe, 0x26},
{0x6006, 0x5,},
{0x6007, 0x78},
{0x6008, 0x4,},
{0x6009, 0xFC},
{0x2019, 0x2,},
{0x201a, 0x80},
{0x201b, 0x1,},
{0x201c, 0xE0},
{0x8010, 0x5,},
{0x8012, 0xA0},
{0x8013, 0x5,},
{0x8014, 0x38},
{0x8015, 0x4,},
{0x8016, 0xf0},
{0x8017, 0x0,},
{0xfffe, 0x21},
{0x0001, 0x92},
{0x0004, 0x18},
{0x0708, 0x0,},
{0x0072, 0xc0},
{0x0074, 0x0a},
{0x0006, 0x5,},
{0x0007, 0xa0},
{0x0008, 0x4,},
{0x0009, 0x38},
{0x000a, 0x2,},
{0x000b, 0x80},
{0x000c, 0x1,},
{0x000d, 0xE0},
{0x001e, 0x5,},
{0x001f, 0xA0},
{0x0020, 0x4,},
{0x0021, 0x38},
{0x005e, 0x9F},
{0x005f, 0x5,},
{0x0060, 0x37},
{0x0061, 0x4,},
{0x0064, 0xA0},
{0x0065, 0x5,},
{0x0066, 0x38},
{0x0067, 0x4,},
{REG_NULL, 0x00},

};
static const struct regval xc6130_720p_regs[] = {
{0xfffd, 0x80},
{0xfffe, 0x21},
{0x0684, 0x07},
{0x0685, 0x80},
{0x0686, 0x04},
{0x0687, 0x38},
{0xfffe, 0x26},
{0x6006, 0xA },
{0x6007, 0x8C},
{0x6008, 0x9 },
{0x6009, 0xFC},
{0x2019, 0x5 },
{0x201a, 0x0 },
{0x201b, 0x2 },
{0x201c, 0xD0},
{0x8010, 0x5 },
{0x8012, 0x80},
{0x8013, 0x7 },
{0x8014, 0x38},
{0x8015, 0x4 },
{0x8016, 0x00},
{0x8017, 0x0 },
{0xfffe, 0x21},
{0x0001, 0x92},
{0x0004, 0x18},
{0x0708, 0x0 },
{0x0072, 0xc0},
{0x0074, 0x0a},
{0x0006, 0x7 },
{0x0007, 0x80},
{0x0008, 0x4 },
{0x0009, 0x38},
{0x000a, 0x5 },
{0x000b, 0x0 },
{0x000c, 0x2 },
{0x000d, 0xD0},
{0x001e, 0x7 },
{0x001f, 0x80},
{0x0020, 0x4 },
{0x0021, 0x38},
{0x005e, 0x7F},
{0x005f, 0x7 },
{0x0060, 0x37},
{0x0061, 0x4 },
{0x0064, 0x80},
{0x0065, 0x7 },
{0x0066, 0x38},
{0x0067, 0x4 },

{REG_NULL, 0x00},
};

static const struct xc7022_mode supported_modes[] = {
	{
		.width = 640,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.colorspace = V4L2_COLORSPACE_SRGB,
		.exp_def = 0x0600,
		.hts_def = 0x12c0,
		.vts_def = 0x0680,
		.reg_list = xc7022_480p_regs,
	},

        {
                .width = 1280,
                .height = 720,
                .max_fps = {
                        .numerator = 10000,
                        .denominator = 300000,
                },
                .colorspace = V4L2_COLORSPACE_SRGB,
                .exp_def = 0x0600,
                .hts_def = 0x12c0,
                .vts_def = 0x0680,
                .reg_list = xc7022_720p_regs,
        },

	{
		.width = 640,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.colorspace = V4L2_COLORSPACE_SRGB,
		.exp_def = 0x0600,
		.hts_def = 0x12c0,
		.vts_def = 0x0680,
		.reg_list = xc6130_480p_regs,
	},
	        {
                .width = 1280,
                .height = 720,
                .max_fps = {
                        .numerator = 10000,
                        .denominator = 300000,
                },
                .colorspace = V4L2_COLORSPACE_SRGB,
                .exp_def = 0x0600,
                .hts_def = 0x12c0,
                .vts_def = 0x0680,
                .reg_list = xc6130_720p_regs,
        },


};

static const s64 link_freq_menu_items[] = {
	XC7022_LINK_FREQ_300MHZ
};

static const char * const xc7022_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int xc7022_write_reg(struct i2c_client *client, u16 reg,
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

static int xc7022_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = xc7022_write_reg(client, regs[i].addr,
					XC7022_REG_VALUE_08BIT,
					regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int xc7022_read_reg(struct i2c_client *client, u16 reg,
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

static int xc7022_get_reso_dist(const struct xc7022_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct xc7022_mode *
xc7022_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = xc7022_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int xc7022_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	const struct xc7022_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&xc7022->mutex);

	mode = xc7022_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_SBGGR10_1X10;//MEDIA_BUS_FMT_YUYV10_2X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&xc7022->mutex);
		return -ENOTTY;
#endif
	} else {
		xc7022->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(xc7022->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(xc7022->vblank, vblank_def,
					 XC7022_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&xc7022->mutex);

	return 0;
}

static int xc7022_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	const struct xc7022_mode *mode = xc7022->cur_mode;

	mutex_lock(&xc7022->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&xc7022->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_YUYV10_2X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&xc7022->mutex);

	return 0;
}

static int xc7022_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_YUYV10_2X10;

	return 0;
}

static int xc7022_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_YUYV8_2X8/*MEDIA_BUS_FMT_SBGGR10_1X10*/)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

//static int xc7022_enable_test_pattern(struct xc7022 *xc7022, u32 pattern)
//{
//	return 0;
//}

static int xc7022_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	const struct xc7022_mode *mode = xc7022->cur_mode;

	mutex_lock(&xc7022->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&xc7022->mutex);

	return 0;
}

#define XC7022_LANES 2
static int xc7022_g_mbus_config(struct v4l2_subdev *sd,
                                 struct v4l2_mbus_config *config)
{
        u32 val = 0;
        val = 1 << (XC7022_LANES - 1) |
        V4L2_MBUS_CSI2_CHANNEL_0 |
        V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

        config->type = V4L2_MBUS_CSI2;
        config->flags = val;

        return 0;
}

static void xc7022_get_module_inf(struct xc7022 *xc7022,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, XC7022_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, xc7022->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, xc7022->len_name, sizeof(inf->base.lens));
}

static long xc7022_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		xc7022_get_module_inf(xc7022, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long xc7022_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = xc7022_ioctl(sd, cmd, inf);
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
			ret = xc7022_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __xc7022_start_stream(struct xc7022 *xc7022)
{
	int ret;
	
	ret = xc7022_write_array(xc7022->client, xc7022_stream_on_regs);
	if(ret)
		printk("write stream on failed\n");

	/* In case these controls are set before streaming */
	mutex_unlock(&xc7022->mutex);
	ret = v4l2_ctrl_handler_setup(&xc7022->ctrl_handler);
	mutex_lock(&xc7022->mutex);
	if (ret)
		return ret;

	return 0;
}

static int __xc7022_stop_stream(struct xc7022 *xc7022)
{	
	int ret;

	ret = xc7022_write_array(xc7022->client, xc7022_stream_off_regs);
	if(ret)
		printk("write stream off failed\n");
		
	return ret;
}

static int xc7022_s_stream(struct v4l2_subdev *sd, int on)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	struct i2c_client *client = xc7022->client;
	int ret = 0;

	mutex_lock(&xc7022->mutex);
	on = !!on;
	if (on == xc7022->streaming){
		goto unlock_and_return;
	}
	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __xc7022_start_stream(xc7022);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__xc7022_stop_stream(xc7022);
		pm_runtime_put(&client->dev);
	}

	xc7022->streaming = on;

unlock_and_return:
	mutex_unlock(&xc7022->mutex);

	return ret;
}

static int __xc7022_power_on(struct xc7022 *xc7022);
static void __xc7022_power_off(struct xc7022 *xc7022);
static int xc7022_s_power(struct v4l2_subdev *sd, int on)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	struct i2c_client *client = xc7022->client;
	struct device *dev = &xc7022->client->dev;
	int ret = 0;
	u32 id = 0;

	mutex_lock(&xc7022->mutex);

	/* If the power state is not modified - no work to do. */
	if (xc7022->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		__xc7022_power_on(xc7022);

        ret = xc7022_write_reg(client, CHECK_CHIP_ID_REG1,
                                                        XC7022_REG_VALUE_08BIT,
                                                        CHECK_CHIP_ID_VAL);
        if (ret){
                dev_err(dev, "write CHECK_CHIP_ID_REG1 failed\n");
                return ret;
        }

        ret = xc7022_write_reg(client, CHECK_CHIP_ID_REG2,
                                                        XC7022_REG_VALUE_08BIT,
                                                        CHECK_CHIP_ID_VAL);
        if (ret){
                dev_err(dev, "write CHECK_CHIP_ID_REG2 failed\n");
                return ret;
        }

	ret = xc7022_read_reg(client, XC7022_CHIP_ID_REG1,
			       XC7022_REG_VALUE_08BIT, &id);
	if (id == XC7022_CHIP_ID1) {
		dev_info(dev, "chip is xc7022\n");
		ret = xc7022_read_reg(client, XC7022_CHIP_ID_REG2,
		       XC7022_REG_VALUE_08BIT, &id);
		if (id != XC7022_CHIP_ID2) {
			dev_err(dev, "Unexpected sensor of XC7022_CHIP_ID_REG2, id(%06x), ret(%d)\n", id, ret);
			//return ret;
		}
	xc7022_global_regs = xc7022_720p_regs;
	} else {
		ret = xc7022_read_reg(client, XC6130_CHIP_ID_REG1,
			       XC7022_REG_VALUE_08BIT, &id);
		if (id == XC6130_CHIP_ID1)	{
			dev_info(dev, "chip is xc6130\n");
			ret = xc7022_read_reg(client, XC6130_CHIP_ID_REG2,
			       XC7022_REG_VALUE_08BIT, &id);
			if (id != XC6130_CHIP_ID2) {
				dev_err(dev, "Unexpected sensor of XC6130_CHIP_ID_REG2, id(%06x), ret(%d)\n", id, ret);
				return ret;
			}
		xc7022_global_regs = xc6130_720p_regs;
		} else {
			xc7022_global_regs = xc6130_720p_regs;
			dev_err(dev, "Unexpected sensor of xc6130, open it directly ...id(%06x),ret(%d)\n",id,ret);
		}
	}

		ret = xc7022_write_array(xc7022->client, xc7022_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		xc7022->power_on = true;
		/* export gpio */
		if (!IS_ERR(xc7022->reset_gpio))
			gpiod_export(xc7022->reset_gpio, false);
		if (!IS_ERR(xc7022->pwdn_gpio))
			gpiod_export(xc7022->pwdn_gpio, false);
	} else {
		pm_runtime_put(&client->dev);
		__xc7022_power_off(xc7022);
		xc7022->power_on = false;
		/* unexport gpio */
		if (!IS_ERR(xc7022->reset_gpio))
			gpiod_unexport(xc7022->reset_gpio);
		if (!IS_ERR(xc7022->pwdn_gpio))
			gpiod_unexport(xc7022->pwdn_gpio);
	}

unlock_and_return:
	mutex_unlock(&xc7022->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 xc7022_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, XC7022_XVCLK_FREQ / 1000 / 1000);
}

static int __xc7022_power_on(struct xc7022 *xc7022)
{
	int ret;
	u32 delay_us;
	struct device *dev = &xc7022->client->dev;

	if (!IS_ERR_OR_NULL(xc7022->pins_default)) {
		ret = pinctrl_select_state(xc7022->pinctrl,
					   xc7022->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	if (clkout_enabled_index){
		ret = clk_prepare_enable(xc7022->xvclk);
		if (ret < 0) {
			dev_err(dev, "Failed to enable xvclk\n");
			return ret;
		}
	}

	if (!IS_ERR(xc7022->reset_gpio))
		gpiod_set_value_cansleep(xc7022->reset_gpio, 0);

	ret = regulator_bulk_enable(XC7022_NUM_SUPPLIES, xc7022->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(xc7022->mipi_pwr_gpio))
		gpiod_set_value_cansleep(xc7022->mipi_pwr_gpio, 1);

	if (!IS_ERR(xc7022->reset_gpio))
		gpiod_set_value_cansleep(xc7022->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(xc7022->pwdn_gpio))
		gpiod_set_value_cansleep(xc7022->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = xc7022_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	if (clkout_enabled_index)
		clk_disable_unprepare(xc7022->xvclk);

	return ret;
}

static void __xc7022_power_off(struct xc7022 *xc7022)
{
	int ret;
	struct device *dev = &xc7022->client->dev;

	if (!IS_ERR(xc7022->pwdn_gpio))
		gpiod_set_value_cansleep(xc7022->pwdn_gpio, 0);
	if (clkout_enabled_index)
		clk_disable_unprepare(xc7022->xvclk);
	if (!IS_ERR(xc7022->mipi_pwr_gpio))
		gpiod_set_value_cansleep(xc7022->mipi_pwr_gpio, 0);
	if (!IS_ERR(xc7022->reset_gpio))
		gpiod_set_value_cansleep(xc7022->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(xc7022->pins_sleep)) {
		ret = pinctrl_select_state(xc7022->pinctrl,
					   xc7022->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(XC7022_NUM_SUPPLIES, xc7022->supplies);
}
static int xc7022_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7022 *xc7022 = to_xc7022(sd);

	return __xc7022_power_on(xc7022);
}

static int xc7022_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7022 *xc7022 = to_xc7022(sd);

	__xc7022_power_off(xc7022);

	return 0;
}



static int xc7022_enum_frame_interval(struct v4l2_subdev *sd,
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
static int xc7022_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct xc7022_mode *def_mode = &supported_modes[0];

	mutex_lock(&xc7022->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;//MEDIA_BUS_FMT_YUYV10_2X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&xc7022->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static const struct dev_pm_ops xc7022_pm_ops = {
	SET_RUNTIME_PM_OPS(xc7022_runtime_suspend,
			   xc7022_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops xc7022_internal_ops = {
	.open = xc7022_open,
};
#endif

static const struct v4l2_subdev_core_ops xc7022_core_ops = {
        .log_status = v4l2_ctrl_subdev_log_status,
        .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
        .unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.s_power = xc7022_s_power,
	.ioctl = xc7022_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = xc7022_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops xc7022_video_ops = {
	.s_stream = xc7022_s_stream,
	.g_frame_interval = xc7022_g_frame_interval,
	.g_mbus_config = xc7022_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops xc7022_pad_ops = {
	.enum_mbus_code = xc7022_enum_mbus_code,
	.enum_frame_size = xc7022_enum_frame_sizes,
	.enum_frame_interval = xc7022_enum_frame_interval,
	.get_fmt = xc7022_get_fmt,
	.set_fmt = xc7022_set_fmt,
};

static const struct v4l2_subdev_ops xc7022_subdev_ops = {
	.core	= &xc7022_core_ops,
	.video	= &xc7022_video_ops,
	.pad	= &xc7022_pad_ops,
};

static int xc7022_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct xc7022 *xc7022 = container_of(ctrl->handler,
					     struct xc7022, ctrl_handler);
	struct i2c_client *client = xc7022->client;
	s64 max;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = xc7022->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(xc7022->exposure,
					 xc7022->exposure->minimum, max,
					 xc7022->exposure->step,
					 xc7022->exposure->default_value);
		break;
	}
	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	pm_runtime_put(&client->dev);
	return 0;
}

static const struct v4l2_ctrl_ops xc7022_ctrl_ops = {
	.s_ctrl = xc7022_set_ctrl,
};

static int xc7022_initialize_controls(struct xc7022 *xc7022)
{
	const struct xc7022_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &xc7022->ctrl_handler;
	mode = xc7022->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &xc7022->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, XC7022_PIXEL_RATE, 1, XC7022_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	xc7022->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (xc7022->hblank)
		xc7022->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	xc7022->vblank = v4l2_ctrl_new_std(handler, &xc7022_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				XC7022_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	xc7022->exposure = v4l2_ctrl_new_std(handler, &xc7022_ctrl_ops,
				V4L2_CID_EXPOSURE, XC7022_EXPOSURE_MIN,
				exposure_max, XC7022_EXPOSURE_STEP,
				mode->exp_def);

	xc7022->anal_gain = v4l2_ctrl_new_std(handler, &xc7022_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, XC7022_GAIN_MIN,
				XC7022_GAIN_MAX, XC7022_GAIN_STEP,
				XC7022_GAIN_DEFAULT);

	xc7022->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&xc7022_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(xc7022_test_pattern_menu) - 1,
				0, 0, xc7022_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&xc7022->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	xc7022->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int xc7022_check_sensor_id(struct xc7022 *xc7022,
				   struct i2c_client *client)
{
	struct device *dev = &xc7022->client->dev;
	u32 id = 0;
	int ret;

#if 1
	ret = xc7022_write_reg(client, CHECK_CHIP_ID_REG1,
							XC7022_REG_VALUE_08BIT, 
							CHECK_CHIP_ID_VAL);
	if (ret){
		dev_err(dev, "write CHECK_CHIP_ID_REG1 failed\n");
		return ret;
	}	

	ret = xc7022_write_reg(client, CHECK_CHIP_ID_REG2,
							XC7022_REG_VALUE_08BIT, 
							CHECK_CHIP_ID_VAL);
	if (ret){
		dev_err(dev, "write CHECK_CHIP_ID_REG2 failed\n");
		return ret;
	}
#endif

	ret = xc7022_read_reg(client, XC7022_CHIP_ID_REG1,
			       XC7022_REG_VALUE_08BIT, &id);
	if (id == XC7022_CHIP_ID1) {
		dev_info(dev, "chip is xc7022\n");
		ret = xc7022_read_reg(client, XC7022_CHIP_ID_REG2,
		       XC7022_REG_VALUE_08BIT, &id);
		if (id != XC7022_CHIP_ID2) {
			dev_err(dev, "Unexpected sensor of XC7022_CHIP_ID_REG2, id(%06x), ret(%d)\n", id, ret);
			return ret;
		}
	xc7022_global_regs = xc7022_720p_regs;
	} else {
		ret = xc7022_read_reg(client, XC6130_CHIP_ID_REG1,
			       XC7022_REG_VALUE_08BIT, &id);
		if (id == XC6130_CHIP_ID1)	{
			dev_info(dev, "chip is xc6130\n");
			ret = xc7022_read_reg(client, XC6130_CHIP_ID_REG2,
			       XC7022_REG_VALUE_08BIT, &id);
			if (id != XC6130_CHIP_ID2) {
				dev_err(dev, "Unexpected sensor of XC6130_CHIP_ID_REG2, id(%06x), ret(%d)\n", id, ret);
				return ret;
			}
		xc7022_global_regs = xc6130_720p_regs;
		} else {
			dev_err(dev, "Check chip ID failed\n");
			//xc7022_global_regs = xc7022_480p_regs; // for default, if not, it cannot init sensor
			return ret;
		}
	}
	
	return 0;
	
}

static int xc7022_configure_regulators(struct xc7022 *xc7022)
{
	unsigned int i;

	for (i = 0; i < XC7022_NUM_SUPPLIES; i++)
		xc7022->supplies[i].supply = xc7022_supply_names[i];

	return devm_regulator_bulk_get(&xc7022->client->dev,
				       XC7022_NUM_SUPPLIES,
				       xc7022->supplies);
}

static void free_gpio(struct xc7022 *xc7022)
{
	if (!IS_ERR(xc7022->pwdn_gpio))
		gpio_free(desc_to_gpio(xc7022->pwdn_gpio));
        if (!IS_ERR(xc7022->reset_gpio))
                gpio_free(desc_to_gpio(xc7022->reset_gpio));
        if (!IS_ERR(xc7022->mipi_pwr_gpio))
		gpio_free(desc_to_gpio(xc7022->mipi_pwr_gpio));
}

static int xc7022_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct xc7022 *xc7022;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;


	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	xc7022 = devm_kzalloc(dev, sizeof(*xc7022), GFP_KERNEL);
	if (!xc7022)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &xc7022->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &xc7022->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &xc7022->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &xc7022->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	xc7022->client = client;
	xc7022->cur_mode = &supported_modes[0];

	

	if (clkout_enabled_index){
		xc7022->xvclk = devm_clk_get(dev, "xvclk");
		if (IS_ERR(xc7022->xvclk)) {
			dev_err(dev, "Failed to get xvclk\n");
			return -EINVAL;
		}
		ret = clk_set_rate(xc7022->xvclk, XC7022_XVCLK_FREQ);
		if (ret < 0) {
			dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
			return ret;
		}
		if (clk_get_rate(xc7022->xvclk) != XC7022_XVCLK_FREQ)
			dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	}


	xc7022->mipi_pwr_gpio = devm_gpiod_get(dev, "mipi-pwr", GPIOD_OUT_LOW);
	if (IS_ERR(xc7022->mipi_pwr_gpio))
		dev_warn(dev, "Failed to get power-gpios, maybe no use\n");

	xc7022->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(xc7022->reset_gpio)) {
	   dev_info(dev, "Failed to get reset-gpios, maybe no use\n");
	}

	xc7022->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(xc7022->pwdn_gpio)) {
	  dev_info(dev, "Failed to get pwdn-gpios, maybe no use\n");
	}

	ret = xc7022_configure_regulators(xc7022);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	xc7022->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(xc7022->pinctrl)) {
		xc7022->pins_default =
			pinctrl_lookup_state(xc7022->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(xc7022->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		xc7022->pins_sleep =
			pinctrl_lookup_state(xc7022->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(xc7022->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&xc7022->mutex);

	sd = &xc7022->subdev;
	v4l2_i2c_subdev_init(sd, client, &xc7022_subdev_ops);
	ret = xc7022_initialize_controls(xc7022);
	if (ret)
		goto err_destroy_mutex;

	ret = xc7022_check_sensor_id(xc7022, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &xc7022_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	xc7022->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &xc7022->pad);
	if (ret < 0)
		goto err_power_off;
	
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(xc7022->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 xc7022->module_index, facing,
		 XC7022_NAME, dev_name(sd->dev));
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
	__xc7022_power_off(xc7022);
	free_gpio(xc7022);
//err_free_handler:
	v4l2_ctrl_handler_free(&xc7022->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&xc7022->mutex);

	return ret;
}

static int xc7022_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7022 *xc7022 = to_xc7022(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&xc7022->ctrl_handler);
	mutex_destroy(&xc7022->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__xc7022_power_off(xc7022);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id xc7022_of_match[] = {
	{ .compatible = "firefly,xc7022" },
	{},
};
MODULE_DEVICE_TABLE(of, xc7022_of_match);
#endif

static const struct i2c_device_id xc7022_match_id[] = {
	{ "firefly,xc7022", 0 },
	{ },
};

static struct i2c_driver xc7022_i2c_driver = {
	.driver = {
		.name = "xc7022",
		.pm = &xc7022_pm_ops,
		.of_match_table = of_match_ptr(xc7022_of_match),
	},
	.probe		= &xc7022_probe,
	.remove		= &xc7022_remove,
	.id_table	= xc7022_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&xc7022_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&xc7022_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision xc7022 sensor driver");
MODULE_LICENSE("GPL v2");
