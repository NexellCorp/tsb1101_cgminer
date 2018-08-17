/*
 * generic SPI functions
 *
 * Copyright 2013, 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "spi-context.h"

#include "logging.h"
#include "miner.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h>

//static int fd_gpio;
static unsigned int spi_rd_speed;
struct spi_ctx *spi_init(struct spi_config *config)
{
	char dev_fname[PATH_MAX];
	struct spi_ctx *ctx;

	if (config == NULL)
		return NULL;

	sprintf(dev_fname, SPI_DEVICE_TEMPLATE, config->bus, config->cs_line);

	int fd = open(dev_fname, O_RDWR);
	if (fd < 0) {
		applog(LOG_ERR, "SPI: Can not open SPI device %s", dev_fname);
		return NULL;
	}

	spi_rd_speed = config->speed;
	if ((ioctl(fd, SPI_IOC_WR_MODE, &config->mode) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_MODE, &config->mode) < 0) ||
	    (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &config->speed) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &config->speed) < 0)) {
		applog(LOG_ERR, "SPI: ioctl error on SPI device %s", dev_fname);
		close(fd);
		return NULL;
	}

	ctx = malloc(sizeof(*ctx));
	assert(ctx != NULL);

	ctx->fd = fd;
#ifndef	USE_TSB1101
	ctx->config = *config;
#else
	memcpy(&ctx->config, config, sizeof(struct spi_config));
#endif
	applog(LOG_WARNING, "SPI '%s': mode=%hhu, bits=%hhu, speed=%u, fd=0x%08x",
	       dev_fname, ctx->config.mode, ctx->config.bits,
	       ctx->config.speed, fd);

//	fd_gpio = open("/sys/class/gpio/export", O_WRONLY);
//	write(fd_gpio, "42", 3);	// reset
//	close(fd_gpio);
//	fd_gpio = open("/sys/class/gpio/gpio42/direction", O_WRONLY);
//	write(fd_gpio, "out", 4);	// oon
//	close(fd_gpio);
//	fd_gpio = open("/sys/class/gpio/gpio42/value", O_WRONLY);
//	write(fd_gpio, "0", 2);
	return ctx;
}

extern void spi_exit(struct spi_ctx *ctx)
{
//	close(fd_gpio);
	if (NULL == ctx)
		return;

	close(ctx->fd);
	free(ctx);
}

extern bool spi_transfer(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len)
{
	struct spi_ioc_transfer xfr;
	int ret;

	if(len&0x3) {
		applog(LOG_ERR, "SPI: length must be 4bytes align, %d is not allowed\n", len);
		return -1;
	}
	if (rxbuf != NULL)
		memset(rxbuf, 0xff, len);

	ret = len;

	xfr.tx_buf = (unsigned long)txbuf;
	xfr.rx_buf = (unsigned long)rxbuf;
	xfr.len = len;
	xfr.speed_hz = ctx->config.speed;
	xfr.delay_usecs = ctx->config.delay;
	xfr.bits_per_word = ctx->config.bits;
	xfr.cs_change = 1;
	xfr.tx_nbits = 0;
	xfr.rx_nbits = 0;
	xfr.pad = 0;

//	write(fd_gpio, "1", 2);

	ret = ioctl(ctx->fd, SPI_IOC_MESSAGE(1), &xfr);
//	write(fd_gpio, "0", 2);
	if (ret < 1) {
		applog(LOG_ERR, "SPI: ioctl error on SPI device: %d", ret);
	}

	return ret > 0;
}

extern bool spi_transfer_x20(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len)
{
	struct spi_ioc_transfer xfr;
	int ret;

	if(len&0x3) {
		applog(LOG_ERR, "SPI: length must be 4bytes align, %d is not allowed\n", len);
		return -1;
	}
	if (rxbuf != NULL)
		memset(rxbuf, 0xff, len);

	ret = len;

	xfr.tx_buf = (unsigned long)txbuf;
	xfr.rx_buf = (unsigned long)rxbuf;
	xfr.len = len;
	xfr.speed_hz = ctx->config.speed*20;
	xfr.delay_usecs = ctx->config.delay;
	xfr.bits_per_word = ctx->config.bits;
	xfr.cs_change = 1;
	xfr.tx_nbits = 0;
	xfr.rx_nbits = 0;
	xfr.pad = 0;

//	write(fd_gpio, "1", 2);

	ret = ioctl(ctx->fd, SPI_IOC_MESSAGE(1), &xfr);
//	write(fd_gpio, "0", 2);
	if (ret < 1) {
		applog(LOG_ERR, "SPIx20: ioctl error on SPI device: %d", ret);
	}

	return ret > 0;
}

extern bool spi_transfer_x20_a(struct spi_ctx *ctx, 
		struct spi_ioc_transfer *xfr, int num)
{
	int ret, ii;

//	write(fd_gpio, "1", 2);

	ret = ioctl(ctx->fd, SPI_IOC_MESSAGE(num), xfr);
//	write(fd_gpio, "0", 2);
	if (ret < 1) {
		applog(LOG_ERR, "SPIx20_a: ioctl error on SPI device: %d", ret);
	}

	return ret > 0;
}
