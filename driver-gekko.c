#include "driver-gekko.h"

static uint32_t compac_check_nonce(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t nonce = (info->work_rx[3] << 0) | (info->work_rx[2] << 8) | (info->work_rx[1] << 16) | (info->work_rx[0] << 24);

	uint64_t hashes = 0;
	uint32_t hwe = compac->hw_errors;
	struct timeval now;

	uint32_t job_id = info->work_rx[4] ^ 0x80;
	struct work *work = info->work[job_id];

	if (nonce == 0 || nonce == 0xffffffff || !work || job_id > MAX_JOBS) {
		return hashes;
	}

	info->nonces++;

	if (nonce == info->prev_nonce) {
		applog(LOG_WARNING, "Dup Nonce : %08x on %s %d", nonce, compac->drv->name, compac->device_id);
		return hashes;
	}

	hashes = info->difficulty * 0xffffffffull;

	info->prev_nonce = nonce;

	applog(LOG_INFO, "Device reported nonce: %08x @ %02x", nonce, info->work_rx[4]);

	cgtime(&now);
	cgtime(&info->last_nonce);

	if (submit_nonce(info->thr, work, nonce)) {
		info->accepted++;
	} else {
		if (hwe != compac->hw_errors) {
			cgtime(&info->last_hwerror);
		}
	}

	return hashes;
}

static void compac_update_work(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	info->update_work = 1;
}

static void compac_flush_work(struct cgpu_info *compac)
{
	int read_bytes;
	unsigned char resp[1];

	compac_update_work(compac);

	while (read_bytes) {
		usb_read_once_timeout(compac, (char *)resp, 1, &read_bytes, 10, C_BF1_GETINFO);
	}
}

static void init_ramp_task(struct COMPAC_INFO *info)
{
	memset(info->work_tx, 0, TX_TASK_SIZE);

	info->work_tx[40] = (info->ramp_hcn >> 24) & 0xff;
	info->work_tx[41] = (info->ramp_hcn >> 16) & 0xff;
	info->work_tx[42] = (info->ramp_hcn >> 8)  & 0xff;
	info->work_tx[43] = (info->ramp_hcn)       & 0xff;
	info->work_tx[51] = info->job_id;
}

static void init_task(struct COMPAC_INFO *info)
{
	struct work *work = info->work[info->job_id];
	uint32_t ticket_mask = (info->difficulty ? info->difficulty - 1 : 0);

	memset(info->work_tx, 0, TX_TASK_SIZE);

	stuff_reverse(info->work_tx, work->midstate, 32);
	stuff_reverse(info->work_tx + 52, work->data + 64, 12);

	info->work_tx[39] = ticket_mask & 0xff;
	info->work_tx[51] = info->job_id & 0xff;
}

static void chip_send(struct cgpu_info *compac, uint32_t b1, uint32_t b2, uint32_t b3, uint32_t b4)
{
	struct COMPAC_INFO *info = compac->device_data;

	unsigned char req_tx[4] = { b1, b2, b3, b4 };
	unsigned char resp_tx[5];
	int read_bytes = 1;
	int read_wait = 20;

	req_tx[3] |= crc5(req_tx, 27);

	if (req_tx[0] == 0x84 && req_tx[3] == 0x11)
		info->chips = 0;

	if (req_tx[0] == 0x84)
		read_wait = 1000;

	usb_write(compac, (char *)req_tx, 4, &read_bytes, C_REQUESTRESULTS);

	applog(LOG_INFO, "%s %d TX: %02x %02x %02x %02x", compac->drv->name, compac->device_id, req_tx[0], req_tx[1], req_tx[2], req_tx[3]);
	while (read_bytes) {
		usb_read_timeout(compac, (char *)resp_tx, RX_RESP_SIZE, &read_bytes, read_wait, C_GETRESULTS);

		if (read_bytes == RX_RESP_SIZE) {
			applog(LOG_INFO, "%s %d RX: %02x %02x %02x %02x %02x", compac->drv->name, compac->device_id, resp_tx[0], resp_tx[1], resp_tx[2], resp_tx[3], resp_tx[4]);
			if (req_tx[0] == 0x84 && req_tx[3] == 0x11)
				info->chips++;
		} else {
			applog(LOG_INFO, "%s %d RX bytes(%d): ", compac->drv->name, compac->device_id, read_bytes);
		}
	}
}

static void *compac_listen(void *object)
{
	struct cgpu_info *compac = (struct cgpu_info *)object;
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t err = 0;
	int read_bytes;

	while (info->shutdown == false)
	{
		if (compac->usbinfo.nodev)
			return false;

		if (info->active) {
			memset(info->work_rx, 0, RX_RESP_SIZE);
			err = usb_read_timeout(compac, (char *)info->work_rx, RX_RESP_SIZE, &read_bytes, 1000, C_GETRESULTS);
			if (read_bytes > 0) {
				applog(LOG_DEBUG,"rx: %02x %02x %02x %02x %02x", info->work_rx[0], info->work_rx[1], info->work_rx[2], info->work_rx[3], info->work_rx[4]);
			}
			if (read_bytes == RX_RESP_SIZE && info->work_rx[4] >= 0x80) {
				mutex_lock(&info->lock);
				info->hashes += compac_check_nonce(compac);
				mutex_unlock(&info->lock);
			}
			if (err < 0 && err != LIBUSB_ERROR_TIMEOUT) {
				applog(LOG_ERR, "%s %i: Comms error (rerr=%d amt=%d)", compac->drv->name, compac->device_id, err, read_bytes);
				dev_error(compac, REASON_DEV_COMMS_ERROR);
			}
		} else {
			cgsleep_ms(100);
		}
	}

	return false;
}

static int64_t compac_scanwork(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	struct timeval now;

	int read_bytes = 1;
	int i, cpu_yield;
	uint64_t hashes = 0;
	uint32_t err = 0;
	uint32_t hcn_max = 1.25 * info->hashrate * RAMP_MS / 1000;
	uint32_t max_task_wait = bound(info->fullscan_ms * 0.40, 5, 1000);

	if (info->ramping < RAMP_CT)
		max_task_wait = RAMP_MS;

	hashes = info->hashes;
	info->hashes -= hashes;

	cgtime(&now);
	info->scanhash_ms = (info->scanhash_ms * 9 + ms_tdiff(&now, &info->last_scanhash)) / 10;
	cgtime(&info->last_scanhash);

	if (info->update_work || (ms_tdiff(&now, &info->last_task) > max_task_wait)) {
		if (ms_tdiff(&now, &info->last_nonce) > MAX_IDLE) {
			if (info->failing) {
				if (ms_tdiff(&now, &info->last_nonce) > (MAX_IDLE * 2)) {
					applog(LOG_ERR, "%s %d: Device failed to respond to restart",
						   compac->drv->name, compac->device_id);
					return -1;
				}
			} else {
				applog(LOG_WARNING, "%s %d: No valid hashes for over 30 seconds, attempting to reset",
					   compac->drv->name, compac->device_id);
				usb_reset(compac);
				info->failing = true;
			}
		} else {
			info->job_id = (info->job_id + 1) % MAX_JOBS;

			if (info->update_work) {
				mutex_lock(&info->lock);
				for (i = 0; i < MAX_JOBS; i++) {
					if (info->work[i])
						free_work(info->work[i]);
					info->work[i] = NULL;
				}
				mutex_unlock(&info->lock);
				info->update_work = 0;
			}

			if (info->work[info->job_id] && info->work[info->job_id]->drv_rolllimit == 0) {
				free_work(info->work[info->job_id]);
				info->work[info->job_id] = NULL;
			}

			if (!info->work[info->job_id]) {
				info->work[info->job_id] = get_work(thr, thr->id);
			} else {
				info->work[info->job_id]->drv_rolllimit--;
				roll_work(info->work[info->job_id]);
			}

			if (info->ramping < RAMP_CT) {
				info->ramping++;
				info->ramp_hcn += hcn_max / RAMP_CT;
				info->ramp_hcn = bound(info->ramp_hcn, 0, 0xffffffff);

				init_ramp_task(info);

				cgtime(&info->last_nonce);
				hashes = info->hashrate * RAMP_MS / 1000;
			} else {
				info->active = true;
				init_task(info);
			}

			err = usb_write(compac, (char *)info->work_tx, TX_TASK_SIZE, &read_bytes, C_SENDWORK);
			if (err != LIBUSB_SUCCESS || read_bytes != TX_TASK_SIZE) {
				applog(LOG_WARNING,"%s %d: Write error", compac->drv->name, compac->device_id);
				return -1;
			}

			info->task_ms = (info->task_ms * 9 + ms_tdiff(&now, &info->last_task)) / 10;
			cgtime(&info->last_task);
		}
	}

	cpu_yield = bound(max_task_wait/10,1,100);
	cgsleep_ms(cpu_yield);
	return hashes;
}

static struct cgpu_info *compac_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *compac;
	struct COMPAC_INFO *info;

	compac = usb_alloc_cgpu(&gekko_drv, 1);

	if (!usb_init(compac, dev, found)) {
		applog(LOG_ERR, "failed usb_init");
		compac = usb_free_cgpu(compac);
		return NULL;
	}

	applog(LOG_INFO, "%s %d: Found at %s", compac->drv->name, compac->device_id, compac->device_path);

	info = cgcalloc(1, sizeof(struct COMPAC_INFO));
	compac->device_data = (void *)info;

	info->ident = usb_ident(compac);

	switch (info->ident) {
		case IDENT_GSC:
		case IDENT_GSD:
			break;
		default:
			quit(1, "%s compac_detect_one() invalid %s ident=%d",
				compac->drv->dname, compac->drv->dname, info->ident);
	}

	if (!add_cgpu(compac)) {
		usb_uninit(compac);
		free(info);
		return NULL;
	}

	update_usb_stats(compac);
	return compac;
}

static void compac_detect(bool __maybe_unused hotplug)
{
	usb_detect(&gekko_drv, compac_detect_one);
}

static void compac_set_frequency(struct cgpu_info *compac, float frequency)
{
	struct COMPAC_INFO *info = compac->device_data;

	uint32_t r, r1, r2, r3, p1, p2, pll, i, interval;
	unsigned char f[2];

	frequency = bound(frequency, 6, 500);
	frequency = ceil(100 * (frequency - 6.25) / 625.0) * 6.25 + 6.25;

	if (info->frequency == frequency)
		return;

	info->frequency = frequency;

	info->hashrate = info->frequency * info->chips * 55 * 1000000;
	info->fullscan_ms = 1000.0 * 0xffffffffull / info->hashrate;

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

	f[0] = (pll) & 0xff;
	f[1] = (pll >> 8) & 0xff;

	applog(LOG_WARNING,"%s %d: set frequency: %.2f [%02x %02x]",  compac->drv->name, compac->device_id, info->frequency, f[1], f[0]);

	chip_send(compac, 0x82, f[1], f[0], 0x00); // Set asic frequency

	chip_send(compac, 0x85, 0x00, 0x00, 0x00); // chain inactive
	for (i = 0; i < info->chips; i++) {
		interval = (0x100 / info->chips) * i;
		chip_send(compac, 0x01, interval, 0x00, 0x00);
	}
}

static bool compac_prepare(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t baudrate = CP210X_DATA_BAUD;
	unsigned int bits = CP210X_BITS_DATA_8 | CP210X_BITS_PARITY_MARK;
	uint32_t i;

	info->thr = thr;

	info->nonces = 0;
	info->ramping = 1;
	info->difficulty = 1;
	info->ramp_hcn = 0;
	info->hashes = 0;
	info->shutdown = false;

	cgtime(&info->start_time);
	cgtime(&info->last_scanhash);
	cgtime(&info->last_nonce);
	cgtime(&info->last_task);

	pthread_mutex_init(&info->lock, NULL);

	memset(info->work_rx, 0, RX_RESP_SIZE);

	for (i = 0; i < 9; i++)
		compac->unique_id[i] = compac->unique_id[i+3];

	info->interface = usb_interface(compac);

	usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_IFC_ENABLE, CP210X_VALUE_UART_ENABLE, info->interface, NULL, 0, C_ENABLE_UART);
	usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_DATA, CP210X_VALUE_DATA, info->interface, NULL, 0, C_SETDATA);
	usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_BAUD, 0, info->interface, &baudrate, sizeof (baudrate), C_SETBAUD);
	usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_SET_LINE_CTL, bits, info->interface, NULL, 0, C_SETPARITY);

	if (thr_info_create(&(info->rthr), NULL, compac_listen, (void *)compac)) {
		applog(LOG_ERR, "%s-%i: thread create failed", compac->drv->name, compac->device_id);
		return false;
	}
	pthread_detach(info->rthr.pth);

	return true;
}

static bool compac_init(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t baudrate = CP210X_DATA_BAUD * 2;  // Baud 230400
	float frequency = 100;

	chip_send(compac, 0x86, 0x10, 0x0D, 0x00); // Baud 230400
	usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_BAUD, 0, info->interface, &baudrate, sizeof (baudrate), C_SETBAUD);

	chip_send(compac, 0x84, 0x00, 0x04, 0x00); // Read reg  0x04
	chip_send(compac, 0x84, 0x00, 0x00, 0x00); // get chain reg0x0

	info->difficulty = bound(info->chips - 1, 1, info->chips);

	applog(LOG_WARNING,"found %d chip(s) on %s %d", info->chips, compac->drv->name, compac->device_id);

	switch (info->ident) {
		case IDENT_GSC:
			frequency = opt_gekko_gsc_freq;
			break;
		case IDENT_GSD:
			frequency = opt_gekko_gsd_freq;
			break;
		default:
			break;
	}

	compac_set_frequency(compac, frequency);
				
	return true;
}

static void compac_statline(char *buf, size_t bufsiz, struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	if (opt_log_output) {
		tailsprintf(buf, bufsiz, "COMPAC-%i %.2fMHz (%d/%d/%d/%d)", info->chips, info->frequency, info->scanhash_ms, info->task_ms, info->fullscan_ms, compac->hw_errors);
	} else {
		tailsprintf(buf, bufsiz, "COMPAC-%i %.2fMHz HW:%d", info->chips, info->frequency, compac->hw_errors);
	}
}

static struct api_data *compac_api_stats(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct api_data *root = NULL;

	root = api_add_int(root, "Nonces", &info->nonces, false);
	root = api_add_int(root, "Accepted", &info->accepted, false);

	return root;
}

static void compac_shutdown(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t baudrate = CP210X_DATA_BAUD;
	info->shutdown = true;
	compac_flush_work(compac);

	chip_send(compac, 0x86, 0x10, 0x1a, 0x00); // Baud 115200
	usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_BAUD, 0, info->interface, &baudrate, sizeof (baudrate), C_SETBAUD);
}

uint32_t crc5(unsigned char *ptr, uint32_t len)
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

void stuff_int32(unsigned char *dst, uint32_t x)
{
	dst[0] = (x >>  0) & 0xff;
	dst[1] = (x >>  8) & 0xff;
	dst[2] = (x >> 16) & 0xff;
	dst[3] = (x >> 24) & 0xff;
}

struct device_drv gekko_drv = {
	.drv_id              = DRIVER_gekko,
	.dname               = "GekkoScience",
	.name                = "GSX",
	.hash_work           = &hash_driver_work,
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
