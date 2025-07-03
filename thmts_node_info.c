#include "thmts_node_info.h"
#include "thmts_config.h"


#ifdef MASTER_MODE
	// 节点模式设置为主节点
	THMTS_NodeInfo_t node = {
			.dev_id = 0,
			.group_id = 0,
			.slot_id = 0,
			.slot_rx_id = 1,
			.role = 0,
			.state = 0,
			.ts_into_timer = -1,
			.ts_curr_rmark = -1,
			.uwb_tx_busy = 0,
			.uwb_rx_busy = 0,
			.arr_adjust = 0,
			.arr_adjust_flag = 0
	};
#endif

#ifdef SLAVE_MODE
//	// 节点模式设置为从节点
//	THMTS_NodeInfo_t node = {
//			.dev_id = 0,
//			.group_id = 0,
//			.slot_id = 1,
////			.slot_id = 10,    //不让从节点发射信号
//			.slot_rx_id = 0,
//			.role = 1,
//			.state = 0,
//			.ts_into_timer = -1,
//			.ts_curr_rmark = -1,
//			.uwb_tx_busy = 0,
//			.uwb_rx_busy = 0,
//			.arr_adjust = 0,
//			.arr_adjust_flag = 0
//	};
#endif

	// 节点模式默认设置为从节点
	THMTS_NodeInfo_t node = {
			.dev_id = 0,
			.group_id = 0,
			.slot_id = 1,
			.slot_rx_id = 0,
			.role = 1,
			.state = 0,
			.ts_into_timer = -1,
			.ts_curr_rmark = -1,
			.uwb_tx_busy = 0,
			.uwb_rx_busy = 0,
			.arr_adjust = 0,
			.arr_adjust_flag = 0
	};
