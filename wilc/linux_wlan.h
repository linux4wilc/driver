/*
 * Atmel WILC 802.11 b/g/n driver
 *
 * Copyright (c) 2015 Atmel Corportation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef WILC_LINUX_WLAN_H
#define WILC_LINUX_WLAN_H
#include <linux/types.h>
#include "wilc_wfi_netdevice.h"
#include "wilc_wlan_if.h"
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#define IP_STATE_OBTAINING						1
#define IP_STATE_OBTAINED						2
#define IP_STATE_GO_ASSIGNING					3
#define IP_STATE_DEFAULT							4


void handle_pwrsave_during_obtainingIP(struct wilc_vif *vif, uint8_t state);
void store_power_save_current_state(struct wilc_vif *vif, bool val);
void clear_duringIP(unsigned long data);



struct net_device* wilc_get_if_netdev(struct wilc *wilc, uint8_t ifc);
struct host_if_drv * wilc_get_drv_handler_by_ifc(struct wilc *wilc, uint8_t ifc);

#endif /* WILC_LINUX_WLAN_H */
