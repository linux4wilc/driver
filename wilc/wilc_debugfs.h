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

#ifndef WILC_DEBUGFS_H
#define WILC_DEBUGFS_H

#include <linux/kern_levels.h>

#define GENERIC_DBG	  		BIT(0)
#define HOSTAPD_DBG       	BIT(1)
#define HOSTINF_DBG	  		BIT(2)
#define CORECONFIG_DBG  	BIT(3)
#define CFG80211_DBG      	BIT(4)
#define INT_DBG		  		BIT(5)
#define TX_DBG		 		BIT(6)
#define RX_DBG		 		BIT(7)
#define TCP_ENH	  			BIT(8)
#define INIT_DBG	  	  	BIT(9)
#define PWRDEV_DBG	  		BIT(10)
#define DBG_REGION_ALL		(BIT(11)-1)

extern atomic_t WILC_DEBUG_REGION;

#define PRINT_D(netdev, region,format,...)	do{ if(atomic_read(&WILC_DEBUG_REGION)&(region))\
	netdev_dbg(netdev, "DBG [%s: %d] "format,__FUNCTION__,__LINE__, ##__VA_ARGS__);}while(0)
							
#define PRINT_INFO(netdev, region, format,...) do{ if(atomic_read(&WILC_DEBUG_REGION)&(region))\
	netdev_info(netdev, "INFO [%s]"format,__FUNCTION__, ##__VA_ARGS__);}while(0)

#define PRINT_WRN(netdev, region, format,...) do{ if(atomic_read(&WILC_DEBUG_REGION)&(region))\
	netdev_warn(netdev, "WRN [%s: %d]"format,__FUNCTION__,__LINE__, ##__VA_ARGS__);}while(0)

#define PRINT_ER(netdev, format,...) do{ netdev_err(netdev, "ERR [%s: %d] "format,\
	__FUNCTION__,__LINE__, ##__VA_ARGS__);}while(0)

int wilc_debugfs_init(void);
void wilc_debugfs_remove(void);
#endif /* WILC_DEBUGFS_H */
