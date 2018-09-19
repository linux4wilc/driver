/* SPDX-License-Identifier: GPL-2.0 */
/*
* Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
* All rights reserved.
*/

#ifndef WILC_WLAN_CFG_H
#define WILC_WLAN_CFG_H

struct wilc_cfg_byte {
	u16 id;
	u16 val;
};

struct wilc_cfg_hword {
	u16 id;
	u16 val;
};

struct wilc_cfg_word {
	u32 id;
	u32 val;
};

struct wilc_cfg_str {
	u16 id;
	u8 *str;
};

struct wilc_cfg_bin {
	u16 id;
	u8 *bin;
};

struct wilc;
int wilc_wlan_cfg_set_wid(struct wilc_vif *vif, u8 *frame, u32 offset, u16 id,
			  u8 *buf, int size);
int wilc_wlan_cfg_get_wid(u8 *frame, u32 offset, u16 id);
int wilc_wlan_cfg_get_wid_value(struct wilc_vif *vif, u16 wid, u8 *buffer,
				u32 buffer_size);
void wilc_wlan_cfg_indicate_rx(struct wilc *wilc, u8 *frame, int size,
			       struct wilc_cfg_rsp *rsp);
int wilc_wlan_cfg_init(void);

#endif
