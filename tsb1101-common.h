#ifndef TSB1101_COMMON_H
#define TSB1101_COMMON_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/********** work queue */
struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};

/********** chip and chain context structures */
/* the WRITE_JOB command is the largest (2 bytes command, 56 bytes payload) */
#define WRITE_JOB_LENGTH	(360/8+16/8)//58
#define MAX_CHAIN_LENGTH	64
/*
 * For commands to traverse the chain, we need to issue dummy writes to
 * keep SPI clock running. To reach the last chip in the chain, we need to
 * write the command, followed by chain-length words to pass it through the
 * chain and another chain-length words to get the ACK back to host
 */
#define MAX_CMD_LENGTH		(360/8+16/8)	// param + command

struct tsb1101_chip {
	int num_cores;
	int last_queued_id;
	struct work *work[4];
	/* stats */
	int hw_errors;
	int stales;
	int nonces_found;
	int nonce_ranges_done;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
	/* number of consecutive failures to access the chip */
	int fail_count;
	/* mark chip disabled, do not try to re-enable it */
	bool disabled;
};

struct tsb1101_chain {
	int chain_id;
	struct cgpu_info *cgpu;
//	struct mcp4x *trimpot;
	int num_chips;
	int num_cores;
	int num_active_chips;
	int chain_skew;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *spi_ctx;
	struct tsb1101_chip *chips;
	pthread_mutex_t lock;

	struct work_queue active_wq;

	/* mark chain disabled, do not try to re-enable it */
	bool disabled;
	uint8_t temp;
	int last_temp_time;
};

#define MAX_SPI_PORT	3
struct tsb1101_board {
	int board_id;
	int num_chains;
	struct tsb1101_chain *chain[MAX_SPI_PORT];
};

/********** config paramters */
struct tsb1101_config_options {
	int spi_clk_khz;
	int pll;
	int udiv;
	/* limit chip chain to this number of chips (testing only) */
	int override_chip_num;
};

/* global configuration instance */
extern struct tsb1101_config_options tsb1101_config_options;

#endif /* TSB1101_COMMON_H */
