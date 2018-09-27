/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#ifndef WILC_LINUX_WLAN_H
#define WILC_LINUX_WLAN_H
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#include "wilc_wfi_netdevice.h"
#include "wilc_wlan_if.h"

#define IP_STATE_OBTAINING						1
#define IP_STATE_OBTAINED						2
#define IP_STATE_GO_ASSIGNING					3
#define IP_STATE_DEFAULT							4

extern int wait_for_recovery;

void handle_pwrsave_during_obtainingIP(struct wilc_vif *vif, uint8_t state);
void store_power_save_current_state(struct wilc_vif *vif, bool val);
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
void clear_during_ip(struct timer_list *t);
#else
void clear_during_ip(unsigned long arg);
#endif

struct net_device *wilc_get_if_netdev(struct wilc *wilc, uint8_t ifc);
struct host_if_drv *wilc_get_drv_handler_by_ifc(struct wilc *wilc, uint8_t ifc);

#if KERNEL_VERSION(3, 14, 0) > LINUX_VERSION_CODE
static inline void ether_addr_copy(u8 *dst, const u8 *src)
{
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	*(u32 *)dst = *(const u32 *)src;
	*(u16 *)(dst + 4) = *(const u16 *)(src + 4);
#else
	u16 *a = (u16 *)dst;
	const u16 *b = (const u16 *)src;

	a[0] = b[0];
	a[1] = b[1];
	a[2] = b[2];
#endif
}

static inline bool ether_addr_equal_unaligned(const u8 *addr1, const u8 *addr2)
{
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	return ether_addr_equal(addr1, addr2);
#else
	return memcmp(addr1, addr2, ETH_ALEN) == 0;
#endif
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0) */

int wilc_bt_power_up(struct wilc *wilc, int source);
int wilc_bt_power_down(struct wilc *wilc, int source);
void wilc_wfi_monitor_rx(struct wilc_vif *vif, u8 *buff, u32 size);

#endif /* WILC_LINUX_WLAN_H */
