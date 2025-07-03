#ifndef THMTS_RANGING_H
#define THMTS_RANGING_H

#include <stdint.h>
#include <stddef.h>
#include "thmts_bb_config.h"
#define Node_num 2
#define C_speed 299792458

#pragma pack(1)
typedef struct{
    uint64_t poll_tx_time;     // T1 - 发起方发送 Poll 的时间戳    *
    uint64_t poll_rx_time;     // T2 - 响应方接收 Poll 的时间戳    *
    uint64_t resp_tx_time;     // T3 - 响应方发送 Response 的时间戳 *
    uint64_t resp_rx_time;     // T4 - 发起方接收 Response 的时间戳
    uint64_t poll2_tx_time;    // T5 - 发起方发送 Roll2 的时间戳   *
    uint64_t poll2_rx_time;    // T6 - 响应方接收 Roll2 的时间戳   *
    uint8_t  poll_cnt;
} altds_twr_t;
#pragma pack()
typedef altds_twr_t* altds_twr_ptr;

extern altds_twr_t altds_twr;
extern double tof;

uint8_t altds_dstwr_check(altds_twr_ptr altds_twr_p);
double altds_dstwr_compute(altds_twr_ptr altds_twr_p);
void altds_dstwr_clear(altds_twr_ptr altds_twr_p);

#endif
