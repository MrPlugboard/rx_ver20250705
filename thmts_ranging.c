#include "thmts_ranging.h"

altds_twr_t altds_twr;
double tof;

uint8_t altds_dstwr_check(altds_twr_ptr altds_twr_p)
{
	uint8_t altds_completed = 0;
	if(altds_twr_p->poll_rx_time != 0 &&
			altds_twr_p->poll_tx_time != 0 &&
			altds_twr_p->poll2_rx_time != 0 &&
			altds_twr_p->poll2_tx_time != 0 &&
			altds_twr_p->resp_rx_time != 0 &&
			altds_twr_p->resp_tx_time != 0){
		altds_completed = 1;
    }
    return altds_completed;
}

void altds_dstwr_clear(altds_twr_ptr altds_twr_p)
{
    if (altds_twr_p != NULL) {
        memset(altds_twr_p, 0, sizeof(altds_twr_t));
    }
}

double altds_dstwr_compute(altds_twr_ptr altds_twr_p)
{
	double tof = 0;
	uint64_t round1, round2, reply1, reply2 = 0;
	round1 = timestamp_substract(altds_twr_p->resp_rx_time,altds_twr_p->poll_tx_time);
	round2 = timestamp_substract(altds_twr_p->poll2_rx_time,altds_twr_p->resp_tx_time);
	reply1 = timestamp_substract(altds_twr_p->resp_tx_time,altds_twr_p->poll_rx_time);
	reply2 = timestamp_substract(altds_twr_p->poll2_tx_time,altds_twr_p->resp_rx_time);
	tof = ( (double)(round1 * round2 - reply1 * reply2) ) / ( (double)(round1 + round2 + reply1 + reply2) ) * C_speed / 124.8 / 64 / 8 / 1000000;
	return tof;
}
