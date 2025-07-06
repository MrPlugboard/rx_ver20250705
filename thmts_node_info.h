//  节点信息结构体

#ifndef THMTS_NODE_INFO_H
#define THMTS_NODE_INFO_H

// incldue necessary headers
#include <stdint.h>

// 节点信息结构体
typedef struct
{
	uint16_t dev_id;
	uint16_t group_id;
	uint16_t slot_id;				// for tx
	uint16_t slot_rx_id;            // for rx 65535: 除发射外，均进行接收 ，其他则只在特定时隙接收，

	uint16_t role;					// 0 : master, 1: slave
	uint16_t state;					// 0 : idle, 1 : search, 2 : connected

	uint16_t uwb_tx_busy;
	uint16_t uwb_rx_busy;

	uint64_t ts_into_timer;		// 进入timer中断是uwb-bb的time stamp
	uint64_t ts_curr_rmark;

	int32_t arr_adjust;
	int32_t tcnt_adjust;
	uint32_t arr_adjust_flag;
	//调整钟差标志    0:  初始值  1：收到数据，计算出来第一次时间同步结果   2：进入第一次调整状态
	//只对从节点有效  3:  第一次调整完成之后的正常工作状态                 4：时钟偏差超过10us（78个clk），需要调整ARR，调整完成回到状态3
	uint8_t slot_mac[50];

} THMTS_NodeInfo_t;

// 全局节点变量声明（extern）
extern THMTS_NodeInfo_t node;

#define NODE_STATE_IDLE 0
#define NODE_STATE_SEARCH 1
#define NODE_STATE_CONNECTED 2
#define NODE_STATE_SELFLOOP 3

#define  NODE_ROLE_ANCHOR_MASTER				0
#define  NODE_ROLE_ANCHOR_SLAVE_NORMAL  		1
#define  NODE_ROLE_ANCHOR_SLAVE_BACKUP  		2
#define  NODE_ROLE_TAG								3
#define  NODE_ROLE_SELFTESTER_IN					4
#define  NODE_ROLE_SELFTESTER_OUT					5


#endif // THMT_NODE_INFO_H
