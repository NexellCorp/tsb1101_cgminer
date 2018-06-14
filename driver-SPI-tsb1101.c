/*
 * cgminer SPI driver for TSB1101 devices
 *
 * Copyright 2018 Jinyong, Lee <justin@nexell.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "tsb1101-common.h"

static struct spi_ctx *spi[MAX_SPI_PORT];
static int spi_idx = 0;

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);

	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}

/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 *
 * TODO: to be removed after bring up / test phase
 */
#define COOLDOWN_MS (30 * 1000)
/* if after this number of retries a chip is still inaccessible, disable it */
#define DISABLE_CHIP_FAIL_THRESHOLD	3


enum TSB1101_command {
	SPI_CMD_READ_ID			= 0x00,
	SPI_CMD_AUTO_ADDRESS	= 0x01,
	SPI_CMD_RUN_BIST		= 0x02,
	SPI_CMD_READ_BIST		= 0x03,
	SPI_CMD_RESET			= 0x04,
	SPI_CMD_SET_PLL			= 0x05,
	SPI_CMD_READ_PLL		= 0x06,
	SPI_CMD_WRITE_PARM		= 0x07,
	SPI_CMD_WRITE_NONCE		= 0x09,
	SPI_CMD_READ_JOB_ID		= 0x0C,
	SPI_CMD_READ_RESULT		= 0x0D,
	SPI_CMD_SET_CONTROL		= 0x12,
};

/*
 * for now, we have one global config, defaulting values:
 * - ref_clk 16MHz / sys_clk 800MHz
 * - 2000 kHz SPI clock
 */
struct tsb1101_config_options tsb1101_config_options = {
	.pll = 16000, .udiv = 3, .spi_clk_khz = 2000,
};

/* override values with --bitmine-tsb1101-options ref:sys:spi: - use 0 for default */
static struct tsb1101_config_options *parsed_config_options;

/********** temporary helper for hexdumping SPI traffic */
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(LOG_DEBUG, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

static void flush_spi(struct tsb1101_chain *tsb1101)
{
	memset(tsb1101->spi_tx, 0, 64);
	spi_transfer(tsb1101->spi_ctx, tsb1101->spi_tx, tsb1101->spi_rx, 64);
}


/********** upper layer SPI functions */
static uint8_t *exec_cmd(struct tsb1101_chain *tsb1101,
			  uint8_t cmd, uint8_t chip_id,
			  uint8_t *data, uint8_t len,
			  uint8_t resp_len)
{
	int tx_len = 2 + len + resp_len;
	memset(tsb1101->spi_tx, 0, tx_len);
	tsb1101->spi_tx[0] = cmd;
	tsb1101->spi_tx[1] = chip_id;

	if (data != NULL)
		memcpy(tsb1101->spi_tx + 2, data, len);

	assert(spi_transfer(tsb1101->spi_ctx, tsb1101->spi_tx, tsb1101->spi_rx, tx_len));
	hexdump("send: TX", tsb1101->spi_tx, tx_len);
	hexdump("send: RX", tsb1101->spi_rx, tx_len);

	return (tsb1101->spi_rx + 2 + len);
}

static const uint8_t golden_midstat[256/8] = {
	0x22, 0x8e, 0xa4, 0x73, 0x2a, 0x3c, 0x9b, 0xa8, 
	0x60, 0xc0, 0x09, 0xcd, 0xa7, 0x25, 0x2b, 0x91, 
	0x61, 0xa5, 0xe7, 0x5e, 0xc8, 0xc5, 0x82, 0xa5, 
	0xf1, 0x06, 0xab, 0xb3, 0xaf, 0x41, 0xf7, 0x90
};
static const uint8_t golden_data[96/8] = {
	0x21, 0x94, 0x26, 0x1a, 0x93, 0x95, 0xe6, 0x4d, 0xbe, 0xd1, 0x71, 0x15
};
static uint8_t golden_nonce[64/8] = {
	0x0e, 0x33, 0x33, 0x7a, 0x0e, 0x33, 0x33, 0x7a
};
static uint8_t golden_hash[256/8] = {
	0x00, 0x00, 0x00, 0x00, 0x89, 0xF8, 0x5C, 0xEA, 
	0x5E, 0x26, 0xD5, 0x6E, 0x48, 0x99, 0x89, 0x2E, 
	0x08, 0xAF, 0xF4, 0x2F, 0x11, 0xAE, 0x01, 0x8C, 
	0xA8, 0x6B, 0xD5, 0x4B, 0xF7, 0xEE, 0x96, 0x8F
};
static uint8_t golden_param[360/8] = {
	/* midstat */
	0x22, 0x8e, 0xa4, 0x73, 0x2a, 0x3c, 0x9b, 0xa8, 
	0x60, 0xc0, 0x09, 0xcd, 0xa7, 0x25, 0x2b, 0x91, 
	0x61, 0xa5, 0xe7, 0x5e, 0xc8, 0xc5, 0x82, 0xa5, 
	0xf1, 0x06, 0xab, 0xb3, 0xaf, 0x41, 0xf7, 0x90,
	/* data */
	0x21, 0x94, 0x26, 0x1a, 0x93, 0x95, 0xe6, 0x4d, 0xbe, 0xd1, 0x71, 0x15,
	/* TODO: select */
	0x00,
};
/********** tsb1101 SPI commands */
static uint8_t *cmd_BIST_BCAST(struct tsb1101_chain *tsb1101)
{
	uint8_t *ret;

	ret = exec_cmd(tsb1101, SPI_CMD_WRITE_PARM, 0x00, golden_param, 360/8, 2);
	if (ret == NULL || ret[1] != SPI_CMD_WRITE_PARM) {
		applog(LOG_ERR, "%d: SPI_CMD_WRITE_PARM failed", tsb1101->chain_id);
		return NULL;
	}

	ret = exec_cmd(tsb1101, SPI_CMD_WRITE_NONCE, 0x00, golden_nonce, 8, 2);
	if (ret == NULL || ret[1] != SPI_CMD_WRITE_NONCE) {
		applog(LOG_ERR, "%d: SPI_CMD_WRITE_NONCE failed", tsb1101->chain_id);
		return NULL;
	}

	ret = exec_cmd(tsb1101, SPI_CMD_RUN_BIST, 0x00, golden_hash, 256/8, 0);
	if (ret == NULL || ret[1] != SPI_CMD_RUN_BIST) {
		applog(LOG_ERR, "%d: SPI_CMD_RUN_BIST failed", tsb1101->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_RESET_BCAST(struct tsb1101_chain *tsb1101, uint8_t strategy)
{
	uint8_t *ret = exec_cmd(tsb1101, SPI_CMD_RESET, 0x00, NULL, 0, 2);
	if (ret == NULL || (ret[0] != SPI_CMD_RESET && tsb1101->num_chips != 0)) {
		applog(LOG_ERR, "%d: cmd_RESET_BCAST failed", tsb1101->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_READ_RESULT_BCAST(struct tsb1101_chain *tsb1101)
{
	int tx_len = 6;
	memset(tsb1101->spi_tx, 0, tx_len);
	tsb1101->spi_tx[0] = SPI_CMD_READ_JOB_ID;

	assert(spi_transfer(tsb1101->spi_ctx, tsb1101->spi_tx, tsb1101->spi_rx, tx_len));
	hexdump("send: TX", tsb1101->spi_tx, tx_len);
	hexdump("send: RX", tsb1101->spi_rx, tx_len);

	uint8_t *scan = tsb1101->spi_rx;
	scan+=2;
	return scan;
}
static uint8_t *cmd_READ_RESULT(struct tsb1101_chain *tsb1101, uint8_t chip_id)
{
	int tx_len = 6;
	memset(tsb1101->spi_tx, 0, tx_len);
	tsb1101->spi_tx[0] = SPI_CMD_READ_RESULT;

	assert(spi_transfer(tsb1101->spi_ctx, tsb1101->spi_tx, tsb1101->spi_rx, tx_len));
	hexdump("send: TX", tsb1101->spi_tx, tx_len);
	hexdump("send: RX", tsb1101->spi_rx, tx_len);

	uint8_t *scan = tsb1101->spi_rx;
	scan+=2;
	return scan;
}

static uint8_t *cmd_READ_ID(struct tsb1101_chain *tsb1101, uint8_t chip)
{
	uint8_t *ret = exec_cmd(tsb1101, SPI_CMD_READ_ID, chip, NULL, 0, 4);
	if (ret == NULL || ret[0] != chip) {
		applog(LOG_ERR, "%d: cmd_READ_ID chip %d failed",
		       tsb1101->chain_id, chip);
		return NULL;
	}
	memcpy(tsb1101->spi_rx, ret, 4);
	return ret;
}
static uint8_t *cmd_WRITE_JOB(struct tsb1101_chain *tsb1101, uint8_t chip_id,
			      uint8_t *job)
{
	/* ensure we push the SPI command to the last chip in chain */
	int tx_len = WRITE_JOB_LENGTH + 2;
	memcpy(tsb1101->spi_tx, job, WRITE_JOB_LENGTH);
	memset(tsb1101->spi_tx + WRITE_JOB_LENGTH, 0, tx_len - WRITE_JOB_LENGTH);

	assert(spi_transfer(tsb1101->spi_ctx, tsb1101->spi_tx, tsb1101->spi_rx, tx_len));
	hexdump("send: TX", tsb1101->spi_tx, tx_len);
	hexdump("send: RX", tsb1101->spi_rx, tx_len);

	tx_len = 2 + 8 + 2;
	uint32_t *start_nonce = (uint32_t *)(tsb1101->spi_tx+2+4);
	uint32_t *  end_nonce = (uint32_t *)(tsb1101->spi_tx+2  );
	memset(tsb1101->spi_tx, 0, tx_len);
	memset(tsb1101->spi_rx, 0, tx_len);
	*start_nonce = 0;
	  *end_nonce = 0xffffffff;
	tsb1101->spi_tx[1] = SPI_CMD_WRITE_NONCE;
	tsb1101->spi_tx[0] = chip_id;
	assert(spi_transfer(tsb1101->spi_ctx, tsb1101->spi_tx, tsb1101->spi_rx, tx_len));
	hexdump("send: TX", tsb1101->spi_tx, tx_len);
	hexdump("send: RX", tsb1101->spi_rx, tx_len);

	uint8_t *ret = tsb1101->spi_rx + 2 + 8;
	if (ret[0] != tsb1101->spi_tx[0] || ret[1] != tsb1101->spi_tx[1]){
		applog(LOG_ERR, "%d: cmd_WRITE_JOB failed: "
			"0x%02x%02x/0x%02x%02x", tsb1101->chain_id,
			ret[0], ret[1], tsb1101->spi_tx[0], tsb1101->spi_tx[1]);
		return NULL;
	}
	return ret;
}

/********** tsb1101 low level functions */
static bool set_pll_config(struct tsb1101_chain *tsb1101, int chip_id,
			   int pll, int udiv)
{
	int ii;
	uint8_t *ret;
	int cid = tsb1101->chain_id;

	// SPI_CMD_SET_PLL
	for(ii=1; ii<=tsb1101->num_chips; ii++) {
		ret = exec_cmd(tsb1101, SPI_CMD_SET_PLL, ii, (uint8_t *)&pll, 4, 2);
		if( (ret[1] != SPI_CMD_SET_PLL) || (ret[1] != ii) ) {
			applog(LOG_WARNING, "%d: error in SET_PLL of %d(0x%x)CHIP", cid, ii, ii);
		}
	}

	for(ii=1; ii<=tsb1101->num_chips; ii++) {
		ret = exec_cmd(tsb1101, SPI_CMD_READ_PLL, ii, NULL, 0, 2);
		if(ret[1] != SPI_CMD_READ_PLL) {
			applog(LOG_WARNING, "%d: error in READ_PLL", cid);
			return 0;
		}
		if(ret[0]&1)
			applog(LOG_WARNING, "%d: PLL locked %d(0x%x)CHIP", cid, ii, ii);
		else
			applog(LOG_WARNING, "%d: PLL not locked %d(0x%x)CHIP", cid, ii, ii);
	}

	ret = exec_cmd(tsb1101, SPI_CMD_SET_CONTROL, 0, (uint8_t *)&udiv, 2, 2);
	if( (ret[1] != SPI_CMD_SET_CONTROL) || (ret[0] != 0) ) {
		applog(LOG_WARNING, "%d: error in SET_CONTROL", cid);
		return 0;
	}

	return true;
}

static bool check_chip(struct tsb1101_chain *tsb1101, int i)
{
	int chip_id = i + 1;
	int cid = tsb1101->chain_id;
	uint8_t *ret;

	ret = exec_cmd(tsb1101, SPI_CMD_READ_BIST, chip_id, NULL, 0, 2);
	if(ret[1]&1) cgsleep_ms(200);
	ret = exec_cmd(tsb1101, SPI_CMD_READ_BIST, chip_id, NULL, 0, 2);
	if(ret[1]&1) {
		applog(LOG_WARNING, "%d: error in READ_BIST", cid);
		return false;
	}

	tsb1101->chips[i].num_cores = ret[0];
	applog(LOG_WARNING, "%d: Found chip %d with %d active cores",
	       cid, chip_id, tsb1101->chips[i].num_cores);
	tsb1101->num_cores += tsb1101->chips[i].num_cores;

	return true;
}

/*
 * RUN_BIST works only once after HW reset, on subsequent calls it
 * returns 0 as number of chips.
 */
static int chain_detect(struct tsb1101_chain *tsb1101)
{
	int tx_len, ii;
	uint8_t dummy[32];
	int cid = tsb1101->chain_id;
	uint8_t *ret;

	// SPI_CMD_RESET
//	ret = exec_cmd(tsb1101, SPI_CMD_RESET, 0x00, NULL, 0, 2);
//	if( (ret[1] != SPI_CMD_RESET) || (ret[0] != 0x00) ) {
//		applog(LOG_WARNING, "%d: error in reset all", cid);
//		return 0;
//	}

	// SPI_CMD_AUTO_ADDRESS
	ret = exec_cmd(tsb1101, SPI_CMD_AUTO_ADDRESS, 0x00, dummy, 32, 2);
	if(ret[0] != 0) {
		applog(LOG_WARNING, "%d: error in AUTO_ADDRESS", cid);
		return 0;
	}
	tsb1101->num_chips = ret[1];

	for(ii=1; ii<=tsb1101->num_chips; ii++) {
		// SPI_CMD_READ_ID
		ret = exec_cmd(tsb1101, SPI_CMD_READ_ID, ii, NULL, 0, 4);
		if(ret[1] != SPI_CMD_READ_ID) {
			applog(LOG_WARNING, "%d: error in READ_ID(%d;0x%x)", cid, ii, ii);
			ii--;
			break;
		}
		if(ret[0] != ii) {
			applog(LOG_WARNING, "%d: error2 in READ_ID(%d;0x%x)", cid, ii, ii);
			ii--;
			break;
		}
	}
	tsb1101->num_active_chips = ii;

	applog(LOG_WARNING, "%d: detected %d chips", cid, tsb1101->num_chips);

	return tsb1101->num_chips;
}

/********** disable / re-enable related section (temporary for testing) */
static int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
}

static bool is_chip_disabled(struct tsb1101_chain *tsb1101, uint8_t chip_id)
{
	struct tsb1101_chip *chip = &tsb1101->chips[chip_id - 1];
	return chip->disabled || chip->cooldown_begin != 0;
}

/* check and disable chip, remember time */
static void disable_chip(struct tsb1101_chain *tsb1101, uint8_t chip_id)
{
	// TODO
	flush_spi(tsb1101);
	struct tsb1101_chip *chip = &tsb1101->chips[chip_id - 1];
	int cid = tsb1101->chain_id;
	if (is_chip_disabled(tsb1101, chip_id)) {
		applog(LOG_WARNING, "%d: chip %d already disabled",
		       cid, chip_id);
		return;
	}
	applog(LOG_WARNING, "%d: temporary disabling chip %d", cid, chip_id);
	chip->cooldown_begin = get_current_ms();
}

/* check if disabled chips can be re-enabled */
void check_disabled_chips(struct tsb1101_chain *tsb1101)
{
	int i;
	int cid = tsb1101->chain_id;
	for (i = 0; i < tsb1101->num_active_chips; i++) {
		int chip_id = i + 1;
		struct tsb1101_chip *chip = &tsb1101->chips[i];
		if (!is_chip_disabled(tsb1101, chip_id))
			continue;
		/* do not re-enable fully disabled chips */
		if (chip->disabled)
			continue;
		if (chip->cooldown_begin + COOLDOWN_MS > get_current_ms())
			continue;
		if (!cmd_READ_ID(tsb1101, chip_id)) {
			chip->fail_count++;
			applog(LOG_WARNING, "%d: chip %d not yet working - %d",
			       cid, chip_id, chip->fail_count);
			if (chip->fail_count > DISABLE_CHIP_FAIL_THRESHOLD) {
				applog(LOG_WARNING,
				       "%d: completely disabling chip %d at %d",
				       cid, chip_id, chip->fail_count);
				chip->disabled = true;
				tsb1101->num_cores -= chip->num_cores;
				continue;
			}
			/* restart cooldown period */
			chip->cooldown_begin = get_current_ms();
			continue;
		}
		applog(LOG_WARNING, "%d: chip %d is working again",
		       cid, chip_id);
		chip->cooldown_begin = 0;
		chip->fail_count = 0;
	}
}

/********** job creation and result evaluation */
uint32_t get_diff(double diff)
{
	uint32_t n_bits;
	int shift = 29;
	double f = (double) 0x0000ffff / diff;
	while (f < (double) 0x00008000) {
		shift--;
		f *= 256.0;
	}
	while (f >= (double) 0x00800000) {
		shift++;
		f /= 256.0;
	}
	n_bits = (int) f + (shift << 24);
	return n_bits;
}

static uint8_t *create_job(uint8_t chip_id, uint8_t job_id, struct work *work)
{
	static uint8_t job[WRITE_JOB_LENGTH] = {
		/* command */
		0x00, SPI_CMD_WRITE_PARM,
		/* midstate */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* wdata */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		/* start nonce */
		0x00, 0x00, 0x00, 0x00,
		/* difficulty 1 */
		0xff, 0xff, 0x00, 0x1d,
		/* end nonce */
		0xff, 0xff, 0xff, 0xff,
	};
	uint8_t *midstate = work->midstate;
	uint8_t *wdata = work->data + 64;

	uint32_t *p1 = (uint32_t *) &job[34];
	uint32_t *p2 = (uint32_t *) wdata;

	job[0] = chip_id;

	swab256(job + 2, midstate);
	p1[0] = bswap_32(p2[0]);
	p1[1] = bswap_32(p2[1]);
	p1[2] = bswap_32(p2[2]);
	// TODO: write to target
	return job;
}

/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct tsb1101_chain *tsb1101, uint8_t chip_id, struct work *work,
		     uint8_t queue_states)
{
	int cid = tsb1101->chain_id;
	struct tsb1101_chip *chip = &tsb1101->chips[chip_id - 1];
	bool retval = false;

	int job_id = chip->last_queued_id + 1;

	applog(LOG_INFO, "%d: queuing chip %d with job_id %d, state=0x%02x",
	       cid, chip_id, job_id, queue_states);
	if (job_id == (queue_states & 0x0f) || job_id == (queue_states >> 4))
		applog(LOG_WARNING, "%d: job overlap: %d, 0x%02x",
		       cid, job_id, queue_states);

	if (chip->work[chip->last_queued_id] != NULL) {
		work_completed(tsb1101->cgpu, chip->work[chip->last_queued_id]);
		chip->work[chip->last_queued_id] = NULL;
		retval = true;
	}
	uint8_t *jobdata = create_job(chip_id, job_id, work);
	if (!cmd_WRITE_JOB(tsb1101, chip_id, jobdata)) {
		/* give back work */
		work_completed(tsb1101->cgpu, work);

		applog(LOG_ERR, "%d: failed to set work for chip %d.%d",
		       cid, chip_id, job_id);
		disable_chip(tsb1101, chip_id);
	} else {
		chip->work[chip->last_queued_id] = work;
		chip->last_queued_id++;
		chip->last_queued_id &= 3;
	}
	return retval;
}

static bool get_nonce(struct tsb1101_chain *tsb1101, uint8_t *nonce,
		      uint8_t *chip, uint8_t *job_id)
{
	uint8_t *ret = cmd_READ_RESULT_BCAST(tsb1101);
	if (ret == NULL)
		return false;
	if ((ret[1]&0x7) == 0) {
		applog(LOG_DEBUG, "%d: output queue empty", tsb1101->chain_id);
		return false;
	}
	if ((ret[1]&(1<<1)) == 0) {
		applog(LOG_DEBUG, "%d: job %x: output of nonce", tsb1101->chain_id, ret[2]);
		// clear interrupt
		*job_id = ret[2];
		*chip = ret[0];
		ret = cmd_READ_RESULT(tsb1101, *chip);
		return false;
	}
	if ((ret[1]&1) == 0) {
		applog(LOG_DEBUG, "%d: job 0x%x: golden nonce", tsb1101->chain_id, ret[2]);
		// clear interrupt
		*job_id = ret[2];
		*chip = ret[0];
		ret = cmd_READ_RESULT(tsb1101, *chip);
		memcpy(nonce, ret, 4);
		return true;
	}
	applog(LOG_DEBUG, "%d: job %x: unknown status: 0x%x", tsb1101->chain_id, ret[2], ret[1]);
	// clear interrupt
	*job_id = ret[2];
	*chip = ret[0];
	ret = cmd_READ_RESULT(tsb1101, *chip);
	return false;
}

/* reset input work queues in chip chain */
static bool abort_work(struct tsb1101_chain *tsb1101)
{
	/* drop jobs already queued: reset strategy 0xed */
	return cmd_RESET_BCAST(tsb1101, 0xed);
}

/********** driver interface */
void exit_tsb1101_chain(struct tsb1101_chain *tsb1101)
{
	if (tsb1101 == NULL)
		return;
	free(tsb1101->chips);
	tsb1101->chips = NULL;
	tsb1101->spi_ctx = NULL;
	free(tsb1101);
}

struct tsb1101_chain *init_tsb1101_chain(struct spi_ctx *ctx, int chain_id)
{
	int i;
	struct tsb1101_chain *tsb1101 = malloc(sizeof(*tsb1101));
	assert(tsb1101 != NULL);

	applog(LOG_DEBUG, "%d: TSB1101 init chain", chain_id);
	memset(tsb1101, 0, sizeof(*tsb1101));
	tsb1101->spi_ctx = ctx;
	tsb1101->chain_id = chain_id;

	tsb1101->num_chips = chain_detect(tsb1101);
	if (tsb1101->num_chips == 0)
		goto failure;

	applog(LOG_WARNING, "spidev%d.%d: %d: Found %d TSB1101 chips",
	       tsb1101->spi_ctx->config.bus, tsb1101->spi_ctx->config.cs_line,
	       tsb1101->chain_id, tsb1101->num_chips);

	if (!set_pll_config(tsb1101, 0, tsb1101_config_options.pll,
			    tsb1101_config_options.udiv))
		goto failure;

	tsb1101->num_active_chips = tsb1101->num_chips;

	tsb1101->chips = calloc(tsb1101->num_active_chips, sizeof(struct tsb1101_chip));
	assert (tsb1101->chips != NULL);

	if (!cmd_BIST_BCAST(tsb1101))
		goto failure;

	for (i = 0; i < tsb1101->num_active_chips; i++)
		check_chip(tsb1101, i);

	applog(LOG_WARNING, "%d: found %d chips with total %d active cores",
	       tsb1101->chain_id, tsb1101->num_active_chips, tsb1101->num_cores);

	mutex_init(&tsb1101->lock);
	INIT_LIST_HEAD(&tsb1101->active_wq.head);

	return tsb1101;

failure:
	exit_tsb1101_chain(tsb1101);
	return NULL;
}

static bool detect_single_chain(struct spi_ctx *ctx)
{
//	board_selector = (struct board_selector*)&dummy_board_selector;
	applog(LOG_WARNING, "TSB1101: checking single chain");
	struct tsb1101_chain *tsb1101 = init_tsb1101_chain(ctx, 0);
	if (tsb1101 == NULL)
		return false;

	struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
	assert(cgpu != NULL);

	memset(cgpu, 0, sizeof(*cgpu));
	cgpu->drv = &tsb1101_drv;
	cgpu->name = "TSB1101.SingleChain";
	cgpu->threads = 1;

	cgpu->device_data = tsb1101;

	tsb1101->cgpu = cgpu;
	add_cgpu(cgpu);
	applog(LOG_WARNING, "Detected single TSB1101 chain with %d chips / %d cores",
	       tsb1101->num_active_chips, tsb1101->num_cores);
	return true;
}

/* Probe SPI channel and register chip chain */
void tsb1101_detect(bool hotplug)
{
	/* no hotplug support for SPI */
	if (hotplug)
		return;

	if(spi_idx>=MAX_SPI_PORT) return;

	/* parse tsb1101-options */
	if (opt_tsb1101_options != NULL && parsed_config_options == NULL) {
		int spi_clk = 0;
		int sys_clk_mhz = 0;
		int udiv = 0;

		sscanf(opt_tsb1101_options, "%d:%d:%d",
		       &spi_clk, &sys_clk_mhz, &udiv);
		if (spi_clk != 0)
			tsb1101_config_options.spi_clk_khz = spi_clk;
		if (sys_clk_mhz == 100)
			tsb1101_config_options.pll = 0x000c4004;
		else //if (sys_clk_mhz == 500)
			tsb1101_config_options.pll = 0x000a5004;
		if (udiv != 0)
			tsb1101_config_options.udiv = udiv;

		/* config options are global, scan them once */
		parsed_config_options = &tsb1101_config_options;
	}
	applog(LOG_DEBUG, "TSB1101 detect");

	/* register global SPI context */
	struct spi_config cfg = default_spi_config;
	cfg.mode = SPI_MODE_1;
	cfg.speed = tsb1101_config_options.spi_clk_khz * 1000;
	cfg.bus = spi_idx;
	spi[spi_idx] = spi_init(&cfg);
	if (spi[spi_idx] == NULL)
		return;

	if (detect_single_chain(spi[spi_idx]))
		return;
	/* release SPI context if no TSB1101 products found */
	spi_exit(spi[spi_idx]);

	spi_idx++;
}

#define TEMP_UPDATE_INT_MS	2000
static int64_t tsb1101_scanwork(struct thr_info *thr)
{
	int i;
	struct cgpu_info *cgpu = thr->cgpu;
	struct tsb1101_chain *tsb1101 = cgpu->device_data;
	int32_t nonce_ranges_processed = 0;

	if (tsb1101->num_cores == 0) {
		cgpu->deven = DEV_DISABLED;
		return 0;
	}
//	board_selector->select(tsb1101->chain_id);

	applog(LOG_DEBUG, "TSB1101 running scanwork");

	uint32_t nonce;
	uint8_t chip_id;
	uint8_t job_id;
	bool work_updated = false;

	mutex_lock(&tsb1101->lock);

	if (tsb1101->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms()) {
//		tsb1101->temp = board_selector->get_temp(0);
		tsb1101->last_temp_time = get_current_ms();
	}
	int cid = tsb1101->chain_id;
	/* poll queued results */
	while (true) {
		if (!get_nonce(tsb1101, (uint8_t*)&nonce, &chip_id, &job_id))
			break;
		nonce = bswap_32(nonce);
		work_updated = true;
		if (chip_id < 1 || chip_id > tsb1101->num_active_chips) {
			applog(LOG_WARNING, "%d: wrong chip_id %d",
			       cid, chip_id);
			continue;
		}
		if (job_id < 1 && job_id > 4) {
			applog(LOG_WARNING, "%d: chip %d: result has wrong "
			       "job_id %d", cid, chip_id, job_id);
			flush_spi(tsb1101);
			continue;
		}

		struct tsb1101_chip *chip = &tsb1101->chips[chip_id - 1];
		struct work *work = chip->work[job_id - 1];
		if (work == NULL) {
			/* already been flushed => stale */
			applog(LOG_WARNING, "%d: chip %d: stale nonce 0x%08x",
			       cid, chip_id, nonce);
			chip->stales++;
			continue;
		}
		if (!submit_nonce(thr, work, nonce)) {
			applog(LOG_WARNING, "%d: chip %d: invalid nonce 0x%08x",
			       cid, chip_id, nonce);
			chip->hw_errors++;
			/* add a penalty of a full nonce range on HW errors */
			nonce_ranges_processed--;
			continue;
		}
		applog(LOG_DEBUG, "YEAH: %d: chip %d / job_id %d: nonce 0x%08x",
		       cid, chip_id, job_id, nonce);
		chip->nonces_found++;
	}

	/* check for completed works */
	for (i = tsb1101->num_active_chips; i > 0; i--) {
		uint8_t c = i;
		if (is_chip_disabled(tsb1101, c))
			continue;
		if (!cmd_READ_ID(tsb1101, c)) {
			disable_chip(tsb1101, c);
			continue;
		}
		uint8_t qstate = tsb1101->spi_rx[1] & 3;
		uint8_t qbuff = tsb1101->spi_rx[2];
		struct work *work;
		struct tsb1101_chip *chip = &tsb1101->chips[i - 1];
		switch(qstate) {
		case 4:
		case 3:
			continue;
		case 2:
		case 1:
		case 0:
			work_updated = true;

			work = wq_dequeue(&tsb1101->active_wq);
			if (work == NULL) {
				applog(LOG_INFO, "%d: chip %d: work underflow",
				       cid, c);
				break;
			}
			if (set_work(tsb1101, c, work, qbuff)) {
				chip->nonce_ranges_done++;
				nonce_ranges_processed++;
			}
			applog(LOG_DEBUG, "%d: chip %d: job done: %d/%d/%d/%d",
			       cid, c,
			       chip->nonce_ranges_done, chip->nonces_found,
			       chip->hw_errors, chip->stales);
			break;
		}
	}
	check_disabled_chips(tsb1101);
	mutex_unlock(&tsb1101->lock);

//	board_selector->release();

	if (nonce_ranges_processed < 0)
		nonce_ranges_processed = 0;

	if (nonce_ranges_processed != 0) {
		applog(LOG_DEBUG, "%d, nonces processed %d",
		       cid, nonce_ranges_processed);
	}
	/* in case of no progress, prevent busy looping */
	if (!work_updated)
		cgsleep_ms(40);

	return (int64_t)nonce_ranges_processed << 32;
}


/* queue two work items per chip in chain */
static bool tsb1101_queue_full(struct cgpu_info *cgpu)
{
	struct tsb1101_chain *tsb1101 = cgpu->device_data;
	int queue_full = false;

	mutex_lock(&tsb1101->lock);
	applog(LOG_DEBUG, "%d, TSB1101 running queue_full: %d/%d",
	       tsb1101->chain_id, tsb1101->active_wq.num_elems, tsb1101->num_active_chips);

	if (tsb1101->active_wq.num_elems >= tsb1101->num_active_chips * 2)
		queue_full = true;
	else
		wq_enqueue(&tsb1101->active_wq, get_queued(cgpu));

	mutex_unlock(&tsb1101->lock);

	return queue_full;
}

static void tsb1101_flush_work(struct cgpu_info *cgpu)
{
	struct tsb1101_chain *tsb1101 = cgpu->device_data;
	int cid = tsb1101->chain_id;
//	board_selector->select(cid);

	applog(LOG_DEBUG, "%d: TSB1101 running flushwork", cid);

	int i;

	mutex_lock(&tsb1101->lock);
	/* stop chips hashing current work */
	if (!abort_work(tsb1101)) {
		applog(LOG_ERR, "%d: failed to abort work in chip chain!", cid);
	}
	/* flush the work chips were currently hashing */
	for (i = 0; i < tsb1101->num_active_chips; i++) {
		int j;
		struct tsb1101_chip *chip = &tsb1101->chips[i];
		for (j = 0; j < 4; j++) {
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;
			applog(LOG_DEBUG, "%d: flushing chip %d, work %d: 0x%p",
			       cid, i, j + 1, work);
			work_completed(cgpu, work);
			chip->work[j] = NULL;
		}
		chip->last_queued_id = 0;
	}
	/* flush queued work */
	applog(LOG_DEBUG, "%d: flushing queued work...", cid);
	while (tsb1101->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&tsb1101->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	mutex_unlock(&tsb1101->lock);

//	board_selector->release();
}

static void tsb1101_get_statline_before(char *buf, size_t len,
				   struct cgpu_info *cgpu)
{
	struct tsb1101_chain *tsb1101 = cgpu->device_data;
	char temp[10];
	if (tsb1101->temp != 0)
		snprintf(temp, 9, "%2dC", tsb1101->temp);
	tailsprintf(buf, len, " %2d:%2d/%3d %s",
		    tsb1101->chain_id, tsb1101->num_active_chips, tsb1101->num_cores,
		    tsb1101->temp == 0 ? "   " : temp);
}

struct device_drv tsb1101_drv = {
	.drv_id = DRIVER_tsb1101,
	.dname = "TSB1101",
	.name = "TSB1101",
	.drv_detect = tsb1101_detect,

	.hash_work = hash_queued_work,
	.scanwork = tsb1101_scanwork,
	.queue_full = tsb1101_queue_full,
	.flush_work = tsb1101_flush_work,
	.get_statline_before = tsb1101_get_statline_before,
};
