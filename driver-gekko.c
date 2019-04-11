#include "driver-gekko.h"
#include "crc.h"
#include "compat.h"
#include <unistd.h>

static bool compac_prepare(struct thr_info *thr);
static uint8_t dev_init_count[0xffff] = {0};
static uint8_t *init_count;
static uint32_t stat_len;
static uint32_t chip_max;

uint32_t bmcrc(unsigned char *ptr, uint32_t len)
{
	unsigned char c[5] = {1, 1, 1, 1, 1};
	uint32_t i, c1, ptr_idx = 0;

	for (i = 0; i < len; i++) {
		c1 = c[1];
		c[1] = c[0];
		c[0] = c[4] ^ ((ptr[ptr_idx] & (0x80 >> (i % 8))) ? 1 : 0);
		c[4] = c[3];
		c[3] = c[2];
		c[2] = c1 ^ c[0];

		if (((i + 1) % 8) == 0)
			ptr_idx++;
	}
	return (c[4] * 0x10) | (c[3] * 0x08) | (c[2] * 0x04) | (c[1] * 0x02) | (c[0] * 0x01);
}

void dumpbuffer(struct cgpu_info *compac, int LOG_LEVEL, char *note, unsigned char *ptr, uint32_t len)
{
	struct COMPAC_INFO *info = compac->device_data;
	if (opt_log_output || LOG_LEVEL <= opt_log_level) {
		char str[2048];
		const char * hex = "0123456789ABCDEF";
		char * pout = str;
		int i = 0;

		for(; i < 768 && i < len - 1; ++i){
			*pout++ = hex[(*ptr>>4)&0xF];
			*pout++ = hex[(*ptr++)&0xF];
			if (i % 42 == 41) {
				*pout = 0;
				pout = str;
				applog(LOG_LEVEL, "%i: %s %s: %s", compac->cgminer_id, compac->drv->name, note, str);
			} else {
				*pout++ = ':';
			}
		}
		*pout++ = hex[(*ptr>>4)&0xF];
		*pout++ = hex[(*ptr)&0xF];
		*pout = 0;
		applog(LOG_LEVEL, "%d: %s %d - %s: %s", compac->cgminer_id, compac->drv->name, compac->device_id, note, str);

	}
}

static int compac_micro_send(struct cgpu_info *compac, uint8_t cmd, uint8_t channel, uint8_t value)
{
	struct COMPAC_INFO *info = compac->device_data;
	int bytes = 1;
	int read_bytes = 1;
	int micro_temp;
	uint8_t temp;
	unsigned short usb_val;
	char null[255];

	// synchronous : safe to run in the listen thread.
	if (!info->micro_found) {
		return 0;
	}

	// Baud Rate : 500,000

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF3; // low byte: bitmask - 1111 0011 - CB1(HI), CB0(HI)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	cgsleep_ms(2);
	//usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, 0x06, (FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);

	info->cmd[0] = cmd | channel;
	info->cmd[1] = value;

	if (value != 0x00 || cmd == M2_SET_VCORE) {
		bytes = 2;
	}

	usb_read_timeout(compac, (char *)info->rx, 255, &read_bytes, 1, C_GETRESULTS);

	dumpbuffer(compac, LOG_INFO, "(micro) TX", info->cmd, bytes);
	usb_write(compac, info->cmd, bytes, &read_bytes, C_REQUESTRESULTS);

	memset(info->rx, 0, info->rx_len);
	usb_read_timeout(compac, (char *)info->rx, 1, &read_bytes, 5, C_GETRESULTS);

	if (read_bytes > 0) {
		dumpbuffer(compac, LOG_INFO, "(micro) RX", info->rx, read_bytes);
		switch (cmd) {
			case 0x20:
				temp = info->rx[0];
				micro_temp = 32 + 1.8 * temp;
				if (micro_temp != info->micro_temp) {
					info->micro_temp = micro_temp;
					applog(LOG_WARNING, "%d: %s %d - micro temp changed to %d°C / %.1f°F", compac->cgminer_id, compac->drv->name, compac->device_id, temp, info->micro_temp);
				}
				break;
			default:
				break;
		}
	}

	// Restore Baud Rate
	//usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, (info->bauddiv + 1), (FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF2; // low byte: bitmask - 1111 0010 - CB1(HI), CB0(LO)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	cgsleep_ms(2);

	return read_bytes;

}

static void compac_send(struct cgpu_info *compac, unsigned char *req_tx, uint32_t bytes, uint32_t crc_bits)
{
	struct COMPAC_INFO *info = compac->device_data;
	int read_bytes = 1;
	int read_wait = 0;
	int i;

	//leave original buffer intact
	for (i = 0; i < bytes; i++) {
		info->cmd[i] = req_tx[i];
	}
	info->cmd[bytes - 1] |= bmcrc(req_tx, crc_bits);

	cgsleep_ms(1);

	int log_level = (bytes < info->task_len) ? LOG_INFO : LOG_INFO;

	dumpbuffer(compac, LOG_INFO, "TX", info->cmd, bytes);
	usb_write(compac, info->cmd, bytes, &read_bytes, C_REQUESTRESULTS);
}

static void compac_send_chain_inactive(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	int i;

	applog(LOG_INFO,"%d: %s %d - sending chain inactive for %d chip(s)", compac->cgminer_id, compac->drv->name, compac->device_id, info->chips);
	if (info->asic_type == BM1387) {
		unsigned char buffer[5] = {0x55, 0x05, 0x00, 0x00, 0x00};
		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);; // chain inactive
		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);; // chain inactive
		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);; // chain inactive
		for (i = 0; i < info->chips; i++) {
			buffer[0] = 0x41;
			buffer[1] = 0x05;
			buffer[2] = (0x100 / info->chips) * i;
			compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);;
		}

		cgsleep_ms(10);
		unsigned char baudrate[] = { 0x58, 0x09, 0x00, 0x1C, 0x00, 0x20, 0x07, 0x00, 0x19 };
		info->bauddiv = 0x01; // 1.5Mbps baud.
		baudrate[6] = info->bauddiv;
		compac_send(compac, (char *)baudrate, sizeof(baudrate), 8 * sizeof(baudrate) - 8);
		cgsleep_ms(10);
		usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, (info->bauddiv + 1), (FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);
		cgsleep_ms(10);

		unsigned char gateblk[9] = {0x58, 0x09, 0x00, 0x1C, 0x40, 0x20, 0x99, 0x80, 0x01};
		gateblk[6] = 0x80 | info->bauddiv;
		compac_send(compac, (char *)gateblk, sizeof(gateblk), 8 * sizeof(gateblk) - 8);; // chain inactive
	} else if (info->asic_type == BM1384) {
		unsigned char buffer[] = {0x85, 0x00, 0x00, 0x00};
		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5); // chain inactive
		for (i = 0; i < info->chips; i++) {
			buffer[0] = 0x01;
			buffer[1] = (0x100 / info->chips) * i;
			compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
		}
		buffer[0] = 0x86; // GATEBLK
		buffer[1] = 0x00;
		buffer[2] = 0x9a; // 0x80 | 0x1a;
		//compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
	}

	if (info->mining_state == MINER_CHIP_COUNT_OK) {
		applog(LOG_INFO, "%d: %s %d - open cores", compac->cgminer_id, compac->drv->name, compac->device_id);
		info->zero_check = 0;
		info->task_hcn = 0;
		info->mining_state = MINER_OPEN_CORE;
	}
}

static void compac_update_rates(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct ASIC_INFO *asic;
	float average_frequency = 0;
	int i;

	for (i = 0; i < info->chips; i++) {
		asic = &info->asics[i];
		asic->hashrate = asic->frequency * info->cores * 1000000;
		asic->fullscan_ms = 1000.0 * 0xffffffffull / asic->hashrate;
		average_frequency += asic->frequency;
	}

	average_frequency = average_frequency / info->chips;
	if (average_frequency != info->frequency) {
		applog(LOG_INFO,"%d: %s %d - frequency updated %.2fMHz -> %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency, average_frequency);
		info->frequency = average_frequency;
		info->wu_max = 0;
	}

	info->hashrate = info->chips * info->frequency * info->cores * 1000000;
	info->fullscan_ms = 1000.0 * 0xffffffffull / info->hashrate;
	info->scanhash_ms = bound(info->fullscan_ms / 2, 1, 100);
	info->ticket_mask = bound(pow(2, ceil(log(info->hashrate / (2.0 * 0xffffffffull)) / log(2))) - 1, 0, 4000);
	info->ticket_mask = (info->asic_type == BM1387) ? 0 : info->ticket_mask;
	info->difficulty = info->ticket_mask + 1;
	info->wu = 0.0139091 * info->cores * info->chips * info->frequency;
}

static void compac_set_frequency_single(struct cgpu_info *compac, float frequency, int asic_id)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct ASIC_INFO *asic = &info->asics[asic_id];
	uint32_t i, r, r1, r2, r3, p1, p2, pll;

	if (info->asic_type == BM1387) {
		unsigned char buffer[] = {0x48, 0x09, 0x00, 0x0C, 0x00, 0x50, 0x02, 0x41, 0x00};   //250MHz -- osc of 25MHz
		frequency = bound(frequency, 50, 900);
		frequency = ceil(100 * (frequency) / 625.0) * 6.25;

		if (frequency < 400) {
			buffer[7] = 0x41;
			buffer[5] = (frequency * 8) / 25;
		} else {
			buffer[7] = 0x21;
			buffer[5] = (frequency * 4) / 25;
		}
		buffer[2] = (0x100 / info->chips) * asic_id;

		//asic->frequency = frequency;
		applog(LOG_INFO, "%d: %s %d - setting chip[%d] frequency %.2fMHz -> %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, asic_id, asic->frequency, frequency);
		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);

		//unsigned char gateblk[9] = {0x48, 0x09, 0x00, 0x1C, 0x40, 0x20, 0x99, 0x80, 0x01};
		//gateblk[6] = 0x80 | info->bauddiv;
		//gateblk[2] = (0x100 / info->chips) * id;
		//compac_send(compac, (char *)gateblk, sizeof(gateblk), 8 * sizeof(gateblk) - 8);; // chain inactive

	}
}

static void compac_set_frequency(struct cgpu_info *compac, float frequency)
{
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t i, r, r1, r2, r3, p1, p2, pll;

	if (info->asic_type == BM1387) {
		unsigned char buffer[] = {0x58, 0x09, 0x00, 0x0C, 0x00, 0x50, 0x02, 0x41, 0x00};   //250MHz -- osc of 25MHz
		frequency = bound(frequency, 50, 900);
		frequency = ceil(100 * (frequency) / 625.0) * 6.25;

		if (frequency < 400) {
			buffer[7] = 0x41;
			buffer[5] = (frequency * 8) / 25;
		} else {
			buffer[7] = 0x21;
			buffer[5] = (frequency * 4) / 25;
		}
/*
		} else {
			buffer[7] = 0x11;
			buffer[5] = (frequency * 2) / 25;
		}
*/

		//applog(LOG_WARNING, "%d: %s %d - setting frequency to %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, frequency);
		//compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);

		for (i = 0; i < info->chips; i++) {
			compac_set_frequency_single(compac, frequency, i);
		}

		info->frequency = frequency;
		info->last_frequency_ping = (struct timeval){0};

	} else if (info->asic_type == BM1384) {
		unsigned char buffer[] = {0x82, 0x0b, 0x83, 0x00};

		frequency = bound(frequency, 6, 500);
		frequency = ceil(100 * (frequency) / 625.0) * 6.25;

		info->frequency = frequency;

		r = floor(log(info->frequency/25) / log(2));

		r1 = 0x0785 - r;
		r2 = 0x200 / pow(2, r);
		r3 = 25 * pow(2, r);

		p1 = r1 + r2 * (info->frequency - r3) / 6.25;
		p2 = p1 * 2 + (0x7f + r);

		pll = ((uint32_t)(info->frequency) % 25 == 0 ? p1 : p2);

		if (info->frequency < 100) {
			pll = 0x0783 - 0x80 * (100 - info->frequency) / 6.25;
		}

		buffer[1] = (pll >> 8) & 0xff;
		buffer[2] = (pll) & 0xff;

		applog(LOG_INFO, "%d: %s %d - setting frequency to %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, frequency);
		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
		buffer[0] = 0x84;
		buffer[1] = 0x00;
		buffer[2] = 0x00;
//		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
		buffer[2] = 0x04;
		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
	}
	compac_update_rates(compac);
}

static void compac_update_work(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	int i;

	for (i = 0; i < JOB_MAX; i++) {
		info->active_work[i] = false;
	}
	info->update_work = 1;
}

static void compac_flush_buffer(struct cgpu_info *compac)
{
	int read_bytes = 1;
	unsigned char resp[32];

	while (read_bytes) {
		usb_read_timeout(compac, (char *)resp, 32, &read_bytes, 1, C_REQUESTRESULTS);
	}
}

static void compac_flush_work(struct cgpu_info *compac)
{
	compac_flush_buffer(compac);
	compac_update_work(compac);
}

static void compac_toggle_reset(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	unsigned short usb_val;

	applog(LOG_INFO,"%d: %s %d - Toggling ASIC nRST to reset", compac->cgminer_id, compac->drv->name, compac->device_id);

	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_RESET, info->interface, C_RESET);
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_DATA, FTDI_VALUE_DATA_BTS, info->interface, C_SETDATA);
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, FTDI_VALUE_BAUD_BTS, (FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_FLOW, FTDI_VALUE_FLOW, info->interface, C_SETFLOW);

	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_PURGE_TX, info->interface, C_PURGETX);
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_PURGE_RX, info->interface, C_PURGERX);

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF2; // low byte: bitmask - 1111 0010 - CB1(HI)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	cgsleep_ms(30);

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF0; // low byte: bitmask - 1111 0000 - CB1(LO)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	cgsleep_ms(30);

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF2; // low byte: bitmask - 1111 0010 - CB1(HI)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	cgsleep_ms(200);

	cgtime(&info->last_reset);
}

static uint64_t compac_check_nonce(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t nonce = (info->rx[3] << 0) | (info->rx[2] << 8) | (info->rx[1] << 16) | (info->rx[0] << 24);

	uint32_t hwe = compac->hw_errors;
	uint32_t job_id, i;
	uint64_t hashes = 0;
	struct timeval now;

	if (info->asic_type == BM1387) {
		job_id = info->rx[5] & 0xff;
	} else if (info->asic_type == BM1384) {
		job_id = info->rx[4] ^ 0x80;
	}

	if (job_id > info->max_job_id || (abs(info->job_id - job_id) > 3 && abs(info->max_job_id - job_id + info->job_id) > 3)) {
		return hashes;
	}

	if (!info->active_work[job_id] &&
		!(job_id > 0 && info->active_work[job_id - 1]) &&
		!(job_id > 1 && info->active_work[job_id - 2]) &&
		!(job_id > 2 && info->active_work[job_id - 3])) {
		return hashes;
	}

	cgtime(&now);

	info->nonces++;
	info->nonceless = 0;
	if (nonce == info->prev_nonce) {
		applog(LOG_INFO, "%d: %s %d - Duplicate Nonce : %08x @ %02x [%02x %02x %02x %02x %02x %02x %02x]", compac->cgminer_id, compac->drv->name, compac->device_id, nonce, job_id,
			info->rx[0], info->rx[1], info->rx[2], info->rx[3], info->rx[4], info->rx[5], info->rx[6]);
		info->dups++;
		int asic_id = floor(info->rx[0] / (0x100 / info->chips));
		struct ASIC_INFO *asic = &info->asics[asic_id];
		asic->dups++;

		if (info->dups == 1) {
			info->mining_state = MINER_MINING_DUPS;
		}
		return hashes;
	} else {
		info->dups = 0;
	}

	hashes = info->difficulty * 0xffffffffull;
	info->prev_nonce = nonce;

	applog(LOG_INFO, "%d: %s %d - Device reported nonce: %08x @ %02x", compac->cgminer_id, compac->drv->name, compac->device_id, nonce, job_id);

	struct work *work = info->work[job_id];
	bool active_work = info->active_work[job_id];
	int midnum = 0;
	if (!opt_gekko_noboost && info->vmask) {
		// force check last few nonces by [job_id - 1]
		if (info->asic_type == BM1387) {
			for (i = 0; i <= 3; i++) {
				if (job_id >= i) {
					if (info->active_work[job_id - i]) {
						work = info->work[job_id - i];
						active_work = info->active_work[job_id - i];
						if (active_work && work) {
							work->micro_job_id = pow(2, i);
							memcpy(work->data, &(work->pool->vmask_001[work->micro_job_id]), 4);
							if (test_nonce(work, nonce)) {
								midnum = i;
								if (i > 0) {
									info->boosted = true;
								}
								break;
							}
						}
					}
				}
			}
		}
	}

	if (!active_work || !work) {
		return hashes;
	}

	work->device_diff = info->difficulty;

	if (submit_nonce(info->thr, work, nonce)) {
		int asic_id = floor(info->rx[0] / (0x100 / info->chips));
		struct ASIC_INFO *asic = &info->asics[asic_id];
		cgtime(&asic->last_nonce);
		cgtime(&info->last_nonce);

		if (midnum > 0) {
			applog(LOG_INFO, "%d: %s %d - AsicBoost nonce found : midstate%d", compac->cgminer_id, compac->drv->name, compac->device_id, midnum);
		}

		info->accepted++;
		info->failing = false;
	} else {
		if (hwe != compac->hw_errors) {
			cgtime(&info->last_hwerror);
			compac_flush_buffer(compac);
		}
	}

	return hashes;
}

static void busy_work(struct COMPAC_INFO *info)
{
	memset(info->task, 0, info->task_len);

	if (info->asic_type == BM1387) {
		info->task[0] = 0x21;
		info->task[1] = info->task_len;
		info->task[2] = info->job_id & 0xff;
		info->task[3] = ((!opt_gekko_noboost && info->vmask) ? 0x04 : 0x01);
		memset(info->task + 8, 0xff, 12);

		unsigned short crc = crc16_false(info->task, info->task_len - 2);
		info->task[info->task_len - 2] = (crc >> 8) & 0xff;
		info->task[info->task_len - 1] = crc & 0xff;
	} else if (info->asic_type == BM1384) {
		if (info->mining_state == MINER_MINING) {
			info->task[39] = info->ticket_mask & 0xff;
			stuff_msb(info->task + 40, info->task_hcn);
		}
		info->task[51] = info->job_id & 0xff;
	}
}

static void init_task(struct COMPAC_INFO *info)
{
	struct work *work = info->work[info->job_id];

	memset(info->task, 0, info->task_len);

	if (info->asic_type == BM1387) {
		info->task[0] = 0x21;
		info->task[1] = info->task_len;
		info->task[2] = info->job_id & 0xff;
		info->task[3] = ((!opt_gekko_noboost && info->vmask) ? 0x04 : 0x01);

		if (info->mining_state == MINER_MINING) {
			stuff_reverse(info->task + 8, work->data + 64, 12);
			stuff_reverse(info->task + 20, work->midstate, 32);
			if (!opt_gekko_noboost && info->vmask) {
				stuff_reverse(info->task + 20 + 32, work->midstate1, 32);
				stuff_reverse(info->task + 20 + 32 + 32, work->midstate2, 32);
				stuff_reverse(info->task + 20 + 32 + 32 + 32, work->midstate3, 32);
			}
		} else {
			memset(info->task + 8, 0xff, 12);
		}
		unsigned short crc = crc16_false(info->task, info->task_len - 2);
		info->task[info->task_len - 2] = (crc >> 8) & 0xff;
		info->task[info->task_len - 1] = crc & 0xff;
	} else if (info->asic_type == BM1384) {
		if (info->mining_state == MINER_MINING) {
			stuff_reverse(info->task, work->midstate, 32);
			stuff_reverse(info->task + 52, work->data + 64, 12);
			info->task[39] = info->ticket_mask & 0xff;
			stuff_msb(info->task + 40, info->task_hcn);
		}
		info->task[51] = info->job_id & 0xff;
	}
}

static void *compac_mine(void *object)
{
	struct cgpu_info *compac = (struct cgpu_info *)object;
	struct COMPAC_INFO *info = compac->device_data;
	struct work *work = NULL;
	struct work *old_work = NULL;

	struct timeval now;
	struct timeval last_frequency_report;
	struct sched_param param;
	int i, read_bytes, sleep_ms, policy, ret_nice, ping_itr;
	uint32_t err = 0;
	uint64_t hashes = 0;
	uint64_t max_task_wait = 0;
	char str_frequency[1024];
	float wait_factor = ((!opt_gekko_noboost && info->vmask && info->asic_type == BM1387) ? 2.25 : 0.75);

	bool adjustable = 0;

#ifndef WIN32
	ret_nice = nice(-15);
#else /* WIN32 */
	pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy);
	pthread_setschedparam(pthread_self(), policy, &param);
	ret_nice = param.sched_priority;
#endif /* WIN32 */
	applog(LOG_INFO, "%d: %s %d - work thread niceness (%d)", compac->cgminer_id, compac->drv->name, compac->device_id, ret_nice);

	max_task_wait = bound(wait_factor * info->fullscan_ms, 1, 3 * info->fullscan_ms);
	sleep_ms = bound(ceil(max_task_wait/8.0), 1, 200);

	while (info->mining_state != MINER_SHUTDOWN) {
		cgtime(&now);

		if (info->chips == 0 || compac->deven == DEV_DISABLED || compac->usbinfo.nodev || info->mining_state != MINER_MINING) {
			cgsleep_ms(10);
		} else if (info->update_work || (ms_tdiff(&now, &info->last_task) > max_task_wait)) {
			uint64_t hashrate_15, hashrate_5m, hashrate_1m, hashrate_li, hashrate_tm;
			double dev_runtime, wu;
			float frequency_computed;
			bool low_eff = 0;

			info->update_work = 0;

			max_task_wait = bound(wait_factor * info->fullscan_ms, 1, 3 * info->fullscan_ms);
			sleep_ms = bound(ceil(max_task_wait/100.0), 1, 200);

			dev_runtime = cgpu_runtime(compac);
			wu = compac->diff1 / dev_runtime * 60;

			if (wu > info->wu_max) {
				info->wu_max = wu;
				cgtime(&info->last_wu_increase);
			}

			hashrate_li = (double)compac->rolling * 1000000ull;
			hashrate_1m = (double)compac->rolling1 * 1000000ull;
			hashrate_5m = (double)compac->rolling5 * 1000000ull;
			hashrate_15 = (double)compac->rolling15 * 1000000ull;
			hashrate_tm = (double)compac->total_mhashes / dev_runtime * 1000000ull;;

			info->eff_li = 100.0 * (1.0 * hashrate_li / info->hashrate);
			info->eff_1m = 100.0 * (1.0 * hashrate_1m / info->hashrate);
			info->eff_5m = 100.0 * (1.0 * hashrate_5m / info->hashrate);
			info->eff_15 = 100.0 * (1.0 * hashrate_15 / info->hashrate);
			info->eff_wu = 100.0 * (1.0 * wu / info->wu);
			info->eff_tm = 100.0 * (1.0 * hashrate_tm / info->hashrate);

			info->eff_li = (info->eff_li > 100) ? 100 : info->eff_li;
			info->eff_1m = (info->eff_1m > 100) ? 100 : info->eff_1m;
			info->eff_5m = (info->eff_5m > 100) ? 100 : info->eff_5m;
			info->eff_15 = (info->eff_15 > 100) ? 100 : info->eff_15;
			info->eff_wu = (info->eff_wu > 100) ? 100 : info->eff_wu;
			info->eff_tm = (info->eff_tm > 100) ? 100 : info->eff_tm;

			frequency_computed = ((hashrate_5m / 1000000.0) / info->cores) / info->chips;
			if (frequency_computed > info->frequency_computed && frequency_computed < 900) {
				info->frequency_computed = frequency_computed;
			}

			info->eff_gs = (((info->eff_tm > info->eff_1m) ? info->eff_tm : info->eff_1m) * 3 + info->eff_li) / 4;

			if (info->eff_5m > 10.0 && info->eff_15 < opt_gekko_tune_down && info->eff_15 < opt_gekko_tune_down && info->eff_5m < opt_gekko_tune_down && info->eff_wu < opt_gekko_tune_down) {
				low_eff = 1;
			}

			if (ms_tdiff(&now, &info->monitor_time) > MS_SECOND_5 && ms_tdiff(&now, &info->last_frequency_adjust) > MS_SECOND_5) {
				for (i = 0; i < info->chips; i++) {
					struct ASIC_INFO *asic = &info->asics[i];
					if (ms_tdiff(&now, &asic->last_nonce) > asic->fullscan_ms * 60 * info->chips || asic->dups > 3) {
						float new_frequency = info->frequency_requested;

						if (info->nononce_reset < 3) {
							// Capture failure high/low frequencies using first three resets
							if ((info->frequency - 6.25) > info->frequency_fail_high) {
								info->frequency_fail_high = (info->frequency - 6.25);
							}
							if ((info->frequency - 6.25) < info->frequency_fail_low) {
								info->frequency_fail_low = (info->frequency - 6.25);
							}
							applog(LOG_WARNING,"%d: %s %d - asic plateau: (%d/3) %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->nononce_reset + 1, info->frequency_fail_high);
						} else {
							if (info->frequency_fail_high > info->frequency) {
								if (ms_tdiff(&now, &info->last_frequency_adjust) > MS_MINUTE_30) {
									// Been running for 30 minutes, possible plateau
									// Move requested frequency closer to plateau value
									info->frequency_fail_high = info->frequency;
								}
							} else {
								if (ms_tdiff(&now, &asic->state_change_time) < MS_MINUTE_30) {
									// Step back frequency
									info->frequency_fail_high -= 6.25;
								}
							}
							if (info->frequency_fail_high < info->frequency_computed) {
								info->frequency_fail_high = info->frequency_computed;
							}
							new_frequency = info->frequency_fail_high;
							new_frequency = ceil(100 * (new_frequency) / 625.0) * 6.25;
						}
						asic->last_state = asic->state;
						asic->state = ASIC_HALFDEAD;
						cgtime(&asic->state_change_time);
						cgtime(&info->monitor_time);
						if (asic->dups > 3) {
							applog(LOG_WARNING,"%d: %s %d - duplicate nonces from chip[%d]", compac->cgminer_id, compac->drv->name, compac->device_id, i);
							asic->dups = 0;
						} else {
							applog(LOG_WARNING,"%d: %s %d - missing nonces from chip[%d]", compac->cgminer_id, compac->drv->name, compac->device_id, i);
						}
						if (new_frequency != info->frequency_requested) {
							applog(LOG_WARNING,"%d: %s %d - no nonce: target frequency %.2fMHz -> %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency_requested, new_frequency);
							info->frequency_requested = new_frequency;
						}
						info->nononce_reset++;
						info->mining_state = MINER_RESET;
						break;
					}
				}
			}

			if (low_eff && ms_tdiff(&now, &info->last_frequency_adjust) > MS_HOUR_1 && ms_tdiff(&now, &info->last_wu_increase) > MS_MINUTE_30 && ms_tdiff(&now, &info->last_pool_lost) > MS_MINUTE_10) {
				float new_frequency = info->frequency - 6.25;
				if (new_frequency < info->frequency_computed) {
					new_frequency = info->frequency_computed;
				}
				new_frequency = ceil(100 * (new_frequency) / 625.0) * 6.25;

				cgtime(&info->last_frequency_adjust);
				cgtime(&info->monitor_time);

				applog(LOG_WARNING,"%d: %s %d - low eff: (1m)%.1f (5m)%.1f (15m)%.1f (WU)%.1f  - [%.1f]", compac->cgminer_id, compac->drv->name, compac->device_id, info->eff_1m, info->eff_5m, info->eff_15, info->eff_wu, opt_gekko_tune_down);

				if (new_frequency == info->frequency) {
					info->mining_state = MINER_RESET;
					continue;
				}

				applog(LOG_WARNING,"%d: %s %d - low eff: target frequency %.2fMHz -> %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency_requested, new_frequency);
				info->frequency_requested = new_frequency;
				info->frequency_fail_high = new_frequency;

				for (i = 0; i < info->chips; i++)
				{
					struct ASIC_INFO *asic = &info->asics[i];
					asic->frequency_requested = info->frequency_requested;
				}
			}

			if (ms_tdiff(&now, &info->last_frequency_ping) > MS_SECOND_5 || (info->frequency_of != info->chips && ms_tdiff(&now, &info->last_frequency_ping) > 50)) {

				info->frequency_of--;

				if (info->frequency_of == (info->chips - 1) && info->eff_gs >= opt_gekko_tune_up)
					adjustable = 1;

				if (info->frequency_of < 0) {
					info->frequency_of = info->chips;
					ping_itr = (ping_itr + 1) % 2;
					adjustable = 0;
				} else {
					struct ASIC_INFO *asic = &info->asics[info->frequency_of];
					if (ping_itr == 1 && asic->frequency != asic->frequency_requested) {
						float new_frequency;
						if (asic->frequency < asic->frequency_requested) {
							new_frequency = asic->frequency + opt_gekko_step_freq;
							if (new_frequency > asic->frequency_requested) {
								new_frequency = asic->frequency_requested;
							}
							if (new_frequency < info->frequency_start) {
								new_frequency = info->frequency_start;
							}
							if (!adjustable) {
								new_frequency = asic->frequency;
								if (info->frequency_of == info->chips)
									applog(LOG_INFO,"%d: %s %d - pending frequency change - waiting for hashrate to catch up", compac->cgminer_id, compac->drv->name, compac->device_id);
							}
						} else {
							new_frequency = asic->frequency_requested;
						}
						if (asic->frequency != new_frequency) {
							cgtime(&info->monitor_time);
							if (info->asic_type == BM1387) {
								compac_set_frequency_single(compac, new_frequency, info->frequency_of);
							} else if (info->asic_type == BM1384) {
								if (info->frequency_of == 0) {
									compac_set_frequency(compac, new_frequency);
									compac_send_chain_inactive(compac);
								}
							}
						}
					}

					if (info->asic_type == BM1387) {
						unsigned char buffer[] = {0x44, 0x05, 0x00, 0x0C, 0x00};  // PLL_PARAMETER
						buffer[2] = (0x100 / info->chips) * info->frequency_of;
						compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);
						cgtime(&info->last_frequency_ping);
					} else if (info->asic_type == BM1384) {
						unsigned char buffer[] = {0x04, 0x00, 0x04, 0x00};
						buffer[1] = (0x100 / info->chips) * info->frequency_of;
						compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
						cgtime(&info->last_frequency_ping);
					}
				}
			}

			work = get_queued(compac);

			if (work) {
				info->job_id = (info->job_id + 1) % (info->max_job_id - 3);
				old_work = info->work[info->job_id];
				info->work[info->job_id] = work;
				info->active_work[info->job_id] = 1;
				info->vmask = work->pool->vmask;
				if (info->asic_type == BM1387) {
					if (!opt_gekko_noboost && info->vmask) {
						info->task_len = 150;
					} else {
						info->task_len = 54;
					}
				}
				init_task(info);
			} else {
				struct pool *cp;
				cp = current_pool();
				busy_work(info);
				info->busy_work++;
				if (!cp->stratum_active)
					cgtime(&info->last_pool_lost);
				cgtime(&info->monitor_time);
			}

			err = usb_write(compac, (char *)info->task, info->task_len, &read_bytes, C_SENDWORK);
			//dumpbuffer(compac, LOG_WARNING, "TASK.TX", info->task, info->task_len);
			if (err != LIBUSB_SUCCESS) {
				applog(LOG_WARNING,"%d: %s %d - usb failure (%d)", compac->cgminer_id, compac->drv->name, compac->device_id, err);
				info->mining_state = MINER_RESET;
				continue;
			}
			if (read_bytes != info->task_len) {
				if (ms_tdiff(&now, &info->last_write_error) > (5 * 1000)) {
					applog(LOG_WARNING,"%d: %s %d - usb write error [%d:%d]", compac->cgminer_id, compac->drv->name, compac->device_id, read_bytes, info->task_len);
					cgtime(&info->last_write_error);
				}
			}

			sched_yield();
			if (old_work) {
				mutex_lock(&info->lock);
				work_completed(compac, old_work);
				mutex_unlock(&info->lock);
				old_work = NULL;
			}

			info->task_ms = (info->task_ms * 9 + ms_tdiff(&now, &info->last_task)) / 10;
			cgtime(&info->last_task);
		}
		cgsleep_ms(sleep_ms);
	}
}

static void *compac_handle_rx(void *object, int read_bytes)
{
	struct cgpu_info *compac = (struct cgpu_info *)object;
	struct COMPAC_INFO *info = compac->device_data;
	struct ASIC_INFO *asic;
	int crc_ok, cmd_resp, i;
	struct timeval now;

	cgtime(&now);

	cmd_resp = (info->rx[read_bytes - 1] <= 0x1F && bmcrc(info->rx, 8 * read_bytes - 5) == info->rx[read_bytes - 1]) ? 1 : 0;

	int log_level = (cmd_resp) ? LOG_INFO : LOG_INFO;
	dumpbuffer(compac, log_level, "RX", info->rx, read_bytes);

	if (cmd_resp && info->rx[0] == 0x80) {
		float frequency;
		cgtime(&info->last_frequency_report);

		if (info->asic_type == BM1387 && (info->rx[2] == 0 || (info->rx[3] >> 4) == 0 || (info->rx[3] & 0x0f) == 0)) {
			dumpbuffer(compac, LOG_WARNING, "RX", info->rx, read_bytes);
			applog(LOG_WARNING,"%d: %s %d - invalid frequency report", compac->cgminer_id, compac->drv->name, compac->device_id);
		} else {
			if (info->asic_type == BM1387) {
				frequency = 25.0 * info->rx[1] / (info->rx[2] * (info->rx[3] >> 4) * (info->rx[3] & 0x0f));
			} else if (info->asic_type == BM1384) {
				frequency = (info->rx[1] + 1) * 6.25 / (1 + info->rx[2] & 0x0f) * pow(2, (3 - info->rx[3])) + ((info->rx[2] >> 4) * 6.25);
			}

			if (info->frequency_of != info->chips) {
				asic = &info->asics[info->frequency_of];
				cgtime(&asic->last_frequency_reply);
				if (frequency != asic->frequency) {
					if (frequency < asic->frequency && frequency != asic->frequency_requested) {
						applog(LOG_INFO,"%d: %s %d - chip[%d] reported frequency at %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency_of, frequency);
					} else {
						applog(LOG_INFO,"%d: %s %d - chip[%d] reported new frequency of %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency_of, frequency);
					}
					asic->frequency = frequency;
					info->report = 1;

				} else {
					applog(LOG_INFO,"%d: %s %d - chip[%d] reported frequency of %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency_of, frequency);
				}
			} else {
				applog(LOG_INFO,"%d: %s %d - [-1] reported frequency of %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, frequency);
				if (frequency != info->frequency) {
					info->frequency = frequency;
				}
			}

			compac_update_rates(compac);
		}
	}

	switch (info->mining_state) {
		case MINER_CHIP_COUNT:
		case MINER_CHIP_COUNT_XX:
			if (cmd_resp && info->rx[0] == 0x13) {
				struct ASIC_INFO *asic = &info->asics[info->chips];
				asic->frequency = info->frequency_default;
				asic->frequency_requested = info->frequency_requested;
				cgtime(&asic->last_nonce);
				info->chips++;
				info->mining_state = MINER_CHIP_COUNT_XX;
				compac_update_rates(compac);
			}
			break;
		case MINER_OPEN_CORE:
			if ((info->rx[0] == 0x72 && info->rx[1] == 0x03 && info->rx[2] == 0xEA && info->rx[3] == 0x83) ||
				(info->rx[0] == 0xE1 && info->rx[0] == 0x6B && info->rx[0] == 0xF8 && info->rx[0] == 0x09)) {
				//open core nonces = healthy chips.
				info->zero_check++;
			}
			break;
		case MINER_MINING:
			if (!cmd_resp) {
				sched_yield();
				mutex_lock(&info->lock);
				info->hashes += compac_check_nonce(compac);
				mutex_unlock(&info->lock);
			}
			break;
		default:
			break;
	}
}

static void *compac_listen(void *object)
{
	struct cgpu_info *compac = (struct cgpu_info *)object;
	struct COMPAC_INFO *info = compac->device_data;
	struct timeval now;
	unsigned char rx[BUFFER_MAX];
	unsigned char *prx = rx;
	int read_bytes, cmd_resp, i, pos, rx_bytes;
	uint32_t err = 0;

	memset(rx, 0, BUFFER_MAX);
	memset(info->rx, 0, BUFFER_MAX);

	pos = 0;
	rx_bytes = 0;

	while (info->mining_state != MINER_SHUTDOWN) {

		cgtime(&now);

		if (info->mining_state == MINER_CHIP_COUNT) {
			if (info->asic_type == BM1387) {
				unsigned char buffer[] = {0x54, 0x05, 0x00, 0x00, 0x00};
				compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);
			} else if (info->asic_type == BM1384) {
				unsigned char buffer[] = {0x84, 0x00, 0x00, 0x00};
				compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
			}
			err = usb_read_timeout(compac, rx, BUFFER_MAX, &read_bytes, 1000, C_GETRESULTS);
			dumpbuffer(compac, LOG_INFO, "CMD.RX", rx, read_bytes);

			rx_bytes = read_bytes;
			info->mining_state = MINER_CHIP_COUNT_XX;
		} else {
			err = usb_read_timeout(compac, &rx[pos], info->rx_len, &read_bytes, 20, C_GETRESULTS);
			rx_bytes += read_bytes;
		}

		if (read_bytes > 0) {

			if (rx_bytes < info->rx_len) {
				applog(LOG_INFO, "%d: %s %d - Buffered %d bytes", compac->cgminer_id, compac->drv->name, compac->device_id, rx_bytes);
				dumpbuffer(compac, LOG_INFO, "Partial-RX", rx, rx_bytes);
				pos = rx_bytes;
				continue;
			}

			if (rx_bytes >= info->rx_len)
				cmd_resp = (rx[read_bytes - 1] <= 0x1F && bmcrc(prx, 8 * read_bytes - 5) == rx[read_bytes - 1]) ? 1 : 0;

			if (info->mining_state == MINER_CHIP_COUNT_XX || cmd_resp) {
				if (rx_bytes % info->rx_len == 2) {
					// fix up trial 1
					int shift = 0;
					int extra = 0;
					for (i = 0; i < (rx_bytes - shift); i++) {
						if (rx[i] == 0x01) {
							shift = 2;
							extra = rx[i + 1];
						}
						rx[i] = rx[i + shift];
					}
					rx_bytes -= shift;
					applog(LOG_INFO, "%d: %s %d - Extra Data - 0x01 0x%02x", compac->cgminer_id, compac->drv->name, compac->device_id, extra & 0xff);
				}
			}

			if (rx_bytes % info->rx_len != 0) {
				int n_read_bytes;
				pos = rx_bytes;
				err = usb_read_timeout(compac, &rx[pos], BUFFER_MAX - pos, &n_read_bytes, 1, C_GETRESULTS);
				rx_bytes += n_read_bytes;

				if (rx_bytes % info->rx_len != 0 && rx_bytes >= info->rx_len) {
					int extra_bytes = rx_bytes % info->rx_len;
					for (i = extra_bytes; i < rx_bytes; i++) {
						rx[i - extra_bytes] = rx[i];
					}
					rx_bytes -= extra_bytes;
					applog(LOG_INFO, "%d: %s %d - Fixing buffer alignment, dumping initial %d bytes", compac->cgminer_id, compac->drv->name, compac->device_id, extra_bytes);
				}
			}

			if (rx_bytes % info->rx_len == 0) {
				for (i = 0; i < rx_bytes; i += info->rx_len) {
					memcpy(info->rx, &rx[i], info->rx_len);
					compac_handle_rx(compac, info->rx_len);
				}
				pos = 0;
				rx_bytes = 0;
			}
		} else {

			if (rx_bytes > 0)
				applog(LOG_INFO, "%d: %s %d - Second read, no data dumping (c) %d bytes", compac->cgminer_id, compac->drv->name, compac->device_id, rx_bytes);

			pos = 0;
			rx_bytes = 0;

			// RX line is idle, let's squeeze in a command to the micro if needed.
			if (info->asic_type == BM1387) {
				if (ms_tdiff(&now, &info->last_micro_ping) > MS_SECOND_5 && ms_tdiff(&now, &info->last_task) > 1 && ms_tdiff(&now, &info->last_task) < 3) {
					compac_micro_send(compac, M1_GET_TEMP, 0x00, 0x00);
					cgtime(&info->last_micro_ping);
				}
			}

			switch (info->mining_state) {
				case MINER_CHIP_COUNT_XX:
					if (info->chips < info->expected_chips) {
						applog(LOG_INFO, "%d: %s %d - found %d/%d chip(s)", compac->cgminer_id, compac->drv->name, compac->device_id, info->chips, info->expected_chips);
						info->mining_state = MINER_RESET;
					} else {
						applog(LOG_INFO, "%d: %s %d - found %d chip(s)", compac->cgminer_id, compac->drv->name, compac->device_id, info->chips);
						if (info->chips > 0) {
							info->mining_state = MINER_CHIP_COUNT_OK;
							(*init_count) = 0;
						} else {
							info->mining_state = MINER_RESET;
						}
					}
					break;
				default:
					break;
			}
		}
	}

}

static bool compac_init(struct thr_info *thr)
{
	int i;
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;

	info->boosted = false;
	info->prev_nonce = 0;
	info->fail_count = 0;
	info->busy_work = 0;
	info->nononce_reset = 0;
	info->frequency = 200;
	info->frequency_default = 200;
	info->frequency_fail_high = 0;
	info->frequency_fail_low = 999;
	info->frequency_of = info->chips;
	info->scanhash_ms = 10;

	memset(info->rx, 0, BUFFER_MAX);
	memset(info->tx, 0, BUFFER_MAX);
	memset(info->cmd, 0, BUFFER_MAX);
	memset(info->end, 0, BUFFER_MAX);
	memset(info->task, 0, BUFFER_MAX);

	for (i = 0; i < JOB_MAX; i++) {
		info->active_work[i] = false;
		info->work[i] = NULL;
	}

	cgtime(&info->last_write_error);
	cgtime(&info->last_frequency_adjust);
	info->last_frequency_ping = (struct timeval){0};
	cgtime(&info->last_micro_ping);
	cgtime(&info->last_scanhash);
	cgtime(&info->last_reset);
	cgtime(&info->last_task);
	cgtime(&info->start_time);
	cgtime(&info->monitor_time);

	switch (info->ident) {
		case IDENT_BSC:
		case IDENT_GSC:
			info->frequency_requested = opt_gekko_gsc_freq;
			info->frequency_start = opt_gekko_start_freq;
			break;
		case IDENT_BSD:
		case IDENT_GSD:
			info->frequency_requested = opt_gekko_gsd_freq;
			info->frequency_start = opt_gekko_start_freq;
			break;
		case IDENT_BSE:
		case IDENT_GSE:
			info->frequency_requested = opt_gekko_gse_freq;
			info->frequency_start = opt_gekko_start_freq;
			break;
		case IDENT_GSH:
			info->frequency_requested = opt_gekko_gsh_freq;
			info->frequency_start = opt_gekko_start_freq;
			break;
		case IDENT_GSI:
			info->frequency_requested = opt_gekko_gsi_freq;
			info->frequency_start = opt_gekko_start_freq;
			break;
		default:
			info->frequency_requested = 200;
			info->frequency_start = info->frequency_requested;
			break;
	}
	if (info->frequency_start > info->frequency_requested) {
		info->frequency_start = info->frequency_requested;
	}
	info->frequency_requested = ceil(100 * (info->frequency_requested) / 625.0) * 6.25;
	info->frequency_start = ceil(100 * (info->frequency_start) / 625.0) * 6.25;

	if (!info->rthr.pth) {
		pthread_mutex_init(&info->lock, NULL);
		pthread_mutex_init(&info->wlock, NULL);
		pthread_mutex_init(&info->rlock, NULL);

		if (thr_info_create(&(info->rthr), NULL, compac_listen, (void *)compac)) {
			applog(LOG_ERR, "%d: %s %d - read thread create failed", compac->cgminer_id, compac->drv->name, compac->device_id);
			return false;
		} else {
			applog(LOG_INFO, "%d: %s %d - read thread created", compac->cgminer_id, compac->drv->name, compac->device_id);
		}
		pthread_detach(info->rthr.pth);

		cgsleep_ms(100);

		if (thr_info_create(&(info->wthr), NULL, compac_mine, (void *)compac)) {
			applog(LOG_ERR, "%d: %s %d - write thread create failed", compac->cgminer_id, compac->drv->name, compac->device_id);
			return false;
		} else {
			applog(LOG_INFO, "%d: %s %d - write thread created", compac->cgminer_id, compac->drv->name, compac->device_id);
		}

		pthread_detach(info->wthr.pth);
	}

	return true;
}

static int64_t compac_scanwork(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;

	struct timeval now;
	int read_bytes;
	uint32_t err = 0;
	uint64_t hashes = 0;

	if (info->chips == 0)
		cgsleep_ms(10);

	if (compac->usbinfo.nodev)
		return -1;

	sched_yield();
	cgtime(&now);

	switch (info->mining_state) {
		case MINER_INIT:
			cgsleep_ms(50);
			compac_flush_buffer(compac);
			info->chips = 0;
			info->ramping = 0;
			if (info->frequency_start > info->frequency_requested) {
				info->frequency_start = info->frequency_requested;
			}
			info->mining_state = MINER_CHIP_COUNT;
			return 0;
			break;
		case MINER_CHIP_COUNT:
			if (ms_tdiff(&now, &info->last_reset) > MS_SECOND_5) {
				applog(LOG_INFO, "%d: %s %d - found 0 chip(s)", compac->cgminer_id, compac->drv->name, compac->device_id);
				info->mining_state = MINER_RESET;
				return 0;
			}
			cgsleep_ms(10);
			break;
		case MINER_CHIP_COUNT_OK:
			cgsleep_ms(50);
			//x//compac_set_frequency(compac, info->frequency_start);
			compac_send_chain_inactive(compac);
			return 0;
			break;
		case MINER_OPEN_CORE:
			info->job_id = info->ramping % info->max_job_id;

			//info->task_hcn = (0xffffffff / info->chips) * (1 + info->ramping) / info->cores;
			init_task(info);
			dumpbuffer(compac, LOG_DEBUG, "RAMP", info->task, info->task_len);

			usb_write(compac, (char *)info->task, info->task_len, &read_bytes, C_SENDWORK);
			if (info->ramping > info->cores) {
				//info->job_id = 0;
				info->mining_state = MINER_OPEN_CORE_OK;
				info->task_hcn = (0xffffffff / info->chips);
				return 0;
			}

			info->ramping++;
			info->task_ms = (info->task_ms * 9 + ms_tdiff(&now, &info->last_task)) / 10;
			cgtime(&info->last_task);
			cgsleep_ms(10);
			return 0;
			break;
		case MINER_OPEN_CORE_OK:
			applog(LOG_INFO, "%d: %s %d - start work", compac->cgminer_id, compac->drv->name, compac->device_id);
			cgtime(&info->start_time);
			cgtime(&info->monitor_time);
			cgtime(&info->last_frequency_adjust);
			info->last_frequency_ping = (struct timeval){0};
			cgtime(&info->last_frequency_report);
			cgtime(&info->last_micro_ping);
			cgtime(&info->last_nonce);
			compac_flush_buffer(compac);
			info->mining_state = MINER_MINING;
			return 0;
			break;
		case MINER_MINING:
			break;
		case MINER_RESET:
			compac_flush_work(compac);
			if (info->asic_type == BM1387) {
				compac_toggle_reset(compac);
			} else if (info->asic_type == BM1384) {
				compac_set_frequency(compac, info->frequency_default);
				compac_send_chain_inactive(compac);
			}
			compac_prepare(thr);

			info->fail_count++;
			info->mining_state = MINER_INIT;
			cgtime(&info->last_reset);
			return 0;
			break;
		case MINER_MINING_DUPS:
			info->mining_state = MINER_MINING;
			//applog(LOG_WARNING, "%d: %s %d - mining dup nonces", compac->cgminer_id, compac->drv->name, compac->device_id);
			//if (ms_tdiff(&now, &info->last_dup_fix) > MS_SECOND_5) {
		//		applog(LOG_WARNING, "%d: %s %d - readdressing asics", compac->cgminer_id, compac->drv->name, compac->device_id);
			//	compac_send_chain_inactive(compac);
			//	cgtime(&info->last_dup_fix);
			//}

			/*   Handled by per chip checks
			if ((int)info->frequency == 200) {
				//possible terminus reset condition.
				//compac_set_frequency(compac, info->frequency);
				//compac_send_chain_inactive(compac);
				//cgtime(&info->last_frequency_adjust);
			} else {
				//check for reset condition
				//if (info->asic_type == BM1384) {
				//	unsigned char buffer[] = {0x84, 0x00, 0x04, 0x00};
				//	compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
				//}
				//cgtime(&info->last_frequency_ping);
			}
			*/
			break;
		default:
			break;
	}
	hashes = info->hashes;
	info->hashes -= hashes;
	cgsleep_ms(info->scanhash_ms);

	return hashes;
}

static struct cgpu_info *compac_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *compac;
	struct COMPAC_INFO *info;
	int err, i;
	bool exclude_me = 0;
	uint32_t baudrate = CP210X_DATA_BAUD;
	unsigned int bits = CP210X_BITS_DATA_8 | CP210X_BITS_PARITY_MARK;

	compac = usb_alloc_cgpu(&gekko_drv, 1);

	if (!usb_init(compac, dev, found)) {
		applog(LOG_INFO, "failed usb_init");
		compac = usb_free_cgpu(compac);
		return NULL;
	}

	info = cgcalloc(1, sizeof(struct COMPAC_INFO));
	compac->device_data = (void *)info;

	info->ident = usb_ident(compac);

	if (opt_gekko_gsc_detect || opt_gekko_gsd_detect || opt_gekko_gse_detect || opt_gekko_gsh_detect || opt_gekko_gsi_detect) {
		exclude_me  = (info->ident == IDENT_BSC && !opt_gekko_gsc_detect);
		exclude_me |= (info->ident == IDENT_GSC && !opt_gekko_gsc_detect);
		exclude_me |= (info->ident == IDENT_BSD && !opt_gekko_gsd_detect);
		exclude_me |= (info->ident == IDENT_GSD && !opt_gekko_gsd_detect);
		exclude_me |= (info->ident == IDENT_BSE && !opt_gekko_gse_detect);
		exclude_me |= (info->ident == IDENT_GSE && !opt_gekko_gse_detect);
		exclude_me |= (info->ident == IDENT_GSH && !opt_gekko_gsh_detect);
		exclude_me |= (info->ident == IDENT_GSI && !opt_gekko_gsi_detect);
	}

	if (opt_gekko_serial != NULL && (strstr(opt_gekko_serial, compac->usbdev->serial_string) == NULL)) {
		exclude_me = true;
	}

	if (exclude_me) {
		usb_uninit(compac);
		free(info);
		compac->device_data = NULL;
		return NULL;
	}

	switch (info->ident) {
		case IDENT_BSC:
		case IDENT_GSC:
		case IDENT_BSD:
		case IDENT_GSD:
		case IDENT_BSE:
		case IDENT_GSE:
			info->asic_type = BM1384;
			info->cores = 55;
			info->max_job_id = 0x1f;
			info->rx_len = 5;
			info->task_len = 64;
			info->tx_len = 4;
			info->healthy = 0.33;

			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_IFC_ENABLE, CP210X_VALUE_UART_ENABLE, info->interface, NULL, 0, C_ENABLE_UART);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_DATA, CP210X_VALUE_DATA, info->interface, NULL, 0, C_SETDATA);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_BAUD, 0, info->interface, &baudrate, sizeof (baudrate), C_SETBAUD);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_SET_LINE_CTL, bits, info->interface, NULL, 0, C_SETPARITY);
			break;
		case IDENT_GSH:
			info->asic_type = BM1387;
			info->expected_chips = 2;
			break;
		case IDENT_GSI:
			info->asic_type = BM1387;
			info->expected_chips = 12;
			break;
		default:
			quit(1, "%d: %s compac_detect_one() invalid %s ident=%d",
				compac->cgminer_id, compac->drv->dname, compac->drv->dname, info->ident);
	}

	switch (info->asic_type) {
		case BM1387:
			info->rx_len = 7;
			info->task_len = 54;
			info->cores = 114;
			info->max_job_id = 0x7f;
			info->healthy = 0.75;
			compac_toggle_reset(compac);
			break;
		default:
			break;
	}

	info->interface = usb_interface(compac);
	info->mining_state = MINER_INIT;

	applog(LOG_DEBUG, "Using interface %d", info->interface);

	if (!add_cgpu(compac))
		quit(1, "Failed to add_cgpu in compac_detect_one");

	update_usb_stats(compac);

	for (i = 0; i < 8; i++) {
		compac->unique_id[i] = compac->unique_id[i+3];
	}
	compac->unique_id[8] = 0;

	applog(LOG_WARNING, "%d: %s %d - %s (%s)", compac->cgminer_id, compac->drv->name, compac->device_id, compac->usbdev->prod_string, compac->unique_id);

	return compac;
}

static void compac_detect(bool __maybe_unused hotplug)
{
	usb_detect(&gekko_drv, compac_detect_one);
}

static bool compac_prepare(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	int i;
	int read_bytes = 1;
	bool miner_ok = true;
	int device = compac->usbinfo.bus_number * 0xff + compac->usbinfo.device_address;

	init_count = &dev_init_count[device];
	(*init_count)++;

	if ((*init_count) > 1) {
		applog(LOG_INFO, "%d: %s %d - init_count %d", compac->cgminer_id, compac->drv->name, compac->device_id, *init_count);
	}

	info->thr = thr;
	info->bauddiv = 0x19; // 115200
	//info->bauddiv = 0x0D; // 214286
	//info->bauddiv = 0x07; // 375000

	//Sanity check and abort to prevent miner thread from being created.
	if (info->asic_type == BM1387) {
		// Ping Micro
		info->micro_found = 0;
/*
		if (info->asic_type == BM1387) {
			info->vcore = bound(opt_gekko_gsh_vcore, 300, 810);
			info->micro_found = 1;
			if (!compac_micro_send(compac, M1_GET_TEMP, 0x00, 0x00)) {
				info->micro_found = 0;
				applog(LOG_INFO, "%d: %s %d - micro not found : dummy mode", compac->cgminer_id, compac->drv->name, compac->device_id);
			} else {
				uint8_t vcc = (info->vcore / 1000.0 - 0.3) / 0.002;
				applog(LOG_INFO, "%d: %s %d - requesting vcore of %dmV (%x)", compac->cgminer_id, compac->drv->name, compac->device_id, info->vcore, vcc);
				compac_micro_send(compac, M2_SET_VCORE, 0x00, vcc);   // Default 400mV
			}
		}
*/

	}

	if ((*init_count) != 0 && (*init_count) % 5 == 0) {
		if ((*init_count) >= 15) {
			compac->deven = DEV_DISABLED;
		} else {
			applog(LOG_INFO, "%d: %s %d - forcing usb_nodev()", compac->cgminer_id, compac->drv->name, compac->device_id);
			usb_nodev(compac);
		}
	} else if ((*init_count) > 1) {
		cgsleep_ms(MS_SECOND_5);
	}

	return true;
}

static void compac_statline(char *buf, size_t bufsiz, struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct timeval now;
	int i;

	char ab[2];
	char asic_stat[64];
	char asic_statline[512];
	char ms_stat[64];
	char eff_stat[64];
	uint32_t len = 0;

	memset(asic_statline, 0, 512);
	memset(asic_stat, 0, 64);
	memset(ms_stat, 0, 64);
	memset(eff_stat, 0, 64);

	if (info->chips == 0) {
		if ((*init_count) > 1) {
			sprintf(asic_statline, "found 0 chip(s)");
		}

		for (i = strlen(asic_statline); i < stat_len; i++)
			asic_statline[i] = ' ';

		tailsprintf(buf, bufsiz, "%s", asic_statline);
		return;
	}

	ab[0] = (info->boosted) ? '+' : 0;
	ab[1] = 0;

	if (info->chips > chip_max)
		chip_max = info->chips;

	cgtime(&now);

	if (opt_widescreen) {
		asic_stat[0] = '[';

		for (i = 1; i <= info->chips; i++) {
			struct ASIC_INFO *asic = &info->asics[i - 1];

			switch (asic->state) {
				case ASIC_HEALTHY:
					asic_stat[i] = 'o';
					break;
				case ASIC_HALFDEAD:
					asic_stat[i] = '-';
					break;
				case ASIC_ALMOST_DEAD:
					asic_stat[i] = '!';
					break;
				case ASIC_DEAD:
					asic_stat[i] = 'x';
					break;
			}

		}
		asic_stat[info->chips + 1] = ']';
		for (i = 1; i <= (chip_max - info->chips) + 1; i++)
			asic_stat[info->chips + 1 + i] = ' ';
	}

	sprintf(ms_stat, "(%d/%d/%d)", info->scanhash_ms, info->task_ms, info->fullscan_ms);

	uint8_t wuc = (ms_tdiff(&now, &info->last_wu_increase) > MS_MINUTE_1) ? 32 : 94;

	if (info->eff_gs >= 99.9 && info->eff_wu >= 98.9) {
		sprintf(eff_stat, "| 100.0%% WU:100%%");
	} else if (info->eff_wu >= 98.9) {
		sprintf(eff_stat, "| %5.1f%% WU:100%%", info->eff_gs);
	} else {
		sprintf(eff_stat, "| %5.1f%% WU:%c%2.0f%%", info->eff_gs, wuc, info->eff_wu);
	}


	if (info->asic_type == BM1387) {
		if (info->micro_found) {
			sprintf(asic_statline, "BM1387:%02d%-1s %.2fMHz P:%.2fMHz (%d/%d/%d/%.0fF)", info->chips, ab, info->frequency, info->frequency_computed, info->scanhash_ms, info->task_ms, info->fullscan_ms, info->micro_temp);
		} else {
			if (opt_log_output) {
				sprintf(asic_statline, "BM1387:%02d%-1s %.2fMHz %s%-13s", info->chips, ab, info->frequency, info->frequency_computed, asic_stat, ms_stat);
			} else {
				sprintf(asic_statline, "BM1387:%02d%-1s %.2fMHz P:%.2fMHz %s%s", info->chips, ab, info->frequency, info->frequency_computed, asic_stat, eff_stat);
			}
		}
	} else {
		if (opt_log_output) {
			sprintf(asic_statline, "BM1384:%02d  %.2fMHz P:%.2fMHz %s%-13s", info->chips, info->frequency, info->frequency_computed, asic_stat, ms_stat);
		} else {
			sprintf(asic_statline, "BM1384:%02d  %.2fMHz P:%.2fMHz %s%s", info->chips, info->frequency, info->frequency_computed, asic_stat, eff_stat);
		}
	}

	len = strlen(asic_statline);
	if (len > stat_len)
		stat_len = len;

	for (i = len; i < stat_len; i++)
		asic_statline[i] = ' ';

	tailsprintf(buf, bufsiz, "%s", asic_statline);
}

static struct api_data *compac_api_stats(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct api_data *root = NULL;

	root = api_add_int(root, "Nonces", &info->nonces, false);
	root = api_add_int(root, "Accepted", &info->accepted, false);

	//root = api_add_temp(root, "Temp", &info->micro_temp, false);

	return root;
}

static void compac_shutdown(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	applog(LOG_INFO, "%d: %s %d - shutting down", compac->cgminer_id, compac->drv->name, compac->device_id);
	if (!compac->usbinfo.nodev) {
		if (info->asic_type == BM1387) {
			compac_micro_send(compac, M2_SET_VCORE, 0x00, 0x00);   // 300mV
			compac_toggle_reset(compac);
		} else if (info->asic_type == BM1384 && info->frequency != info->frequency_default) {
			compac_set_frequency(compac, info->frequency_default);
			compac_send_chain_inactive(compac);
		}
	}
	info->mining_state = MINER_SHUTDOWN;
	pthread_join(info->rthr.pth, NULL); // Let thread close.
	pthread_join(info->wthr.pth, NULL); // Let thread close.
	PTH(thr) = 0L;
}

uint64_t bound(uint64_t value, uint64_t lower_bound, uint64_t upper_bound)
{
	if (value < lower_bound)
		return lower_bound;
	if (value > upper_bound)
		return upper_bound;
	return value;
}

void stuff_reverse(unsigned char *dst, unsigned char *src, uint32_t len)
{
	uint32_t i;
	for (i = 0; i < len; i++) {
		dst[i] = src[len - i - 1];
	}
}

void stuff_lsb(unsigned char *dst, uint32_t x)
{
	dst[0] = (x >>  0) & 0xff;
	dst[1] = (x >>  8) & 0xff;
	dst[2] = (x >> 16) & 0xff;
	dst[3] = (x >> 24) & 0xff;
}

void stuff_msb(unsigned char *dst, uint32_t x)
{
	dst[0] = (x >> 24) & 0xff;
	dst[1] = (x >> 16) & 0xff;
	dst[2] = (x >>  8) & 0xff;
	dst[3] = (x >>  0) & 0xff;
}

struct device_drv gekko_drv = {
	.drv_id              = DRIVER_gekko,
	.dname               = "GekkoScience",
	.name                = "GSX",
	.hash_work           = hash_queued_work,
	.get_api_stats       = compac_api_stats,
	.get_statline_before = compac_statline,
	.drv_detect          = compac_detect,
	.scanwork            = compac_scanwork,
	.flush_work          = compac_flush_work,
	.update_work         = compac_update_work,
	.thread_prepare      = compac_prepare,
	.thread_init         = compac_init,
	.thread_shutdown     = compac_shutdown,
};
