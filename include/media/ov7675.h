/*
 * A V4L2 driver for OmniVision OV7675 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Copyright 2010 Archos SA
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#ifndef _OV7675_H
#define _OV7675_H

/*
 * The 7675 sits on i2c with ID 0x21
 */
#define OV7675_I2C_ADDR		0x42 >> 1

#define OV7675_USE_XCLKA	0
#define OV7675_USE_XCLKB	1

/*
 * Our nominal (default) frame rate.
 */
#define OV7675_FRAME_RATE	30


struct ov7675_platform_data {
	int (*power_set) (struct v4l2_int_device *s, enum v4l2_power on);
	int (*ifparm) (struct v4l2_int_device *s, struct v4l2_ifparm *p);
	int (*priv_data_set) (struct v4l2_int_device *s, void *);
	u32 (*set_xclk)(struct v4l2_int_device *s, u32 xclkfreq);
};

#endif /* _OV7675_H */
