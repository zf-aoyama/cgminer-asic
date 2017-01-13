#include "math.h"
#include "miner.h"
#include "usbutils.h"

#define TX_TASK_SIZE 64
#define RX_RESP_SIZE  5

#define RAMP_CT 200
#define RAMP_MS 25

#define MAX_CHIPS 0x1F

struct COMPAC_INFO {
	
	enum sub_ident ident;
	struct thr_info *thr;
	
	float frequency;

	uint32_t scanhash_ms;		 // Avg time(ms) inside scanhash loop
	uint32_t task_ms;			 // Avg time(ms) between task sent to device
	uint32_t fullscan_ms;		 // Estimated time(ms) for full nonce range
	uint64_t hashrate;           // Estimated hashrate = 55M x Chips x Frequency
	
	uint64_t ramp_hcn;           // HCN high watermark at ramping
	uint32_t prev_nonce;
	
	unsigned char work_tx[TX_TASK_SIZE];
	unsigned char work_rx[RX_RESP_SIZE];
	
	uint32_t difficulty;
	
	bool failing;
	
	int nonces;
	int accepted;
	uint32_t interface;
	uint32_t chips;
	uint32_t ramping;
	
	uint32_t update_work;
	uint32_t job_id;

	struct timeval start_time;
	struct timeval last_scanhash;
	struct timeval last_task;
	struct timeval last_nonce;
	struct timeval last_hwerror;
	struct timeval next_freq_tune;

	struct work *work[MAX_CHIPS];
	
};

void stuff_int32(unsigned char *dst, uint32_t x);
void stuff_reverse(unsigned char *dst, unsigned char *src, uint32_t len);
uint64_t bound(uint64_t value, uint64_t lower_bound, uint64_t upper_bound);
uint32_t crc5(char *ptr, uint32_t len);
