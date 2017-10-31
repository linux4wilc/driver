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

#ifndef LINUX_WLAN_COMMON_H
#define LINUX_WLAN_COMMON_H


enum debug_region{
	Generic_debug = 0,
	Hostapd_debug,
	Hostinf_debug,
	CFG80211_debug,
	Coreconfig_debug,
	Interrupt_debug,
	TX_debug,
	RX_debug,
	Lock_debug,
	Tcp_enhance,
	Spin_debug,
	
	Init_debug,
	Bus_debug,
	Mem_debug,
	Firmware_debug,
	PwrDev_debug,
	COMP = 0xFFFFFFFF,
};

#define GENERIC_DBG	  		(1<<Generic_debug)
#define HOSTAPD_DBG       	(1<<Hostapd_debug)
#define HOSTINF_DBG	  		(1<<Hostinf_debug)
#define CORECONFIG_DBG  	(1<<Coreconfig_debug)
#define CFG80211_DBG      	(1<<CFG80211_debug)
#define INT_DBG		  		(1<<Interrupt_debug)
#define TX_DBG		 		(1<<TX_debug)
#define RX_DBG		 		(1<<RX_debug)
#define LOCK_DBG	  		(1<<Lock_debug)
#define TCP_ENH	  			(1<<Tcp_enhance)

#define SPIN_DEBUG 			(1<<Spin_debug)

#define INIT_DBG	  	  		(1<<Init_debug)
#define BUS_DBG		  		(1<<Bus_debug)
#define MEM_DBG		  		(1<<Mem_debug)
#define FIRM_DBG	  		(1<<Firmware_debug)
#define PWRDEV_DBG	  		(1<<PwrDev_debug)

#define REGION	 INIT_DBG|GENERIC_DBG|CFG80211_DBG | FIRM_DBG | HOSTAPD_DBG | PWRDEV_DBG

#define DEBUG	    1
#define INFO        0
#define WRN         0
#define PRINT_D(region,...)	do{ if(DEBUG == 1 && ((REGION)&(region))){printk("DBG [%s: %d]",__FUNCTION__,__LINE__);\
							printk(__VA_ARGS__);}}while(0)
							
#define PRINT_INFO(region,...) do{ if(INFO == 1 && ((REGION)&(region))){printk("INFO [%s]",__FUNCTION__);\
							printk(__VA_ARGS__);}}while(0)

#define PRINT_WRN(region,...) do{ if(WRN == 1 && ((REGION)&(region))){printk("WRN [%s: %d]",__FUNCTION__,__LINE__);\
							printk(__VA_ARGS__);}}while(0)

#define PRINT_ER(...)	do{ printk("ERR [%s: %d]",__FUNCTION__,__LINE__);\
							printk(__VA_ARGS__);}while(0)
#endif /* LINUX_WLAN_COMMON_H */
