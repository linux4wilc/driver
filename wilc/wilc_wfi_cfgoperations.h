/* SPDX-License-Identifier: GPL-2.0 */
/*!
 *  @file	wilc_wfi_cfgoperations.h
 *  @brief	Definitions for the network module
 *  @author	syounan
 *  @sa		wilc_oswrapper.h top level OS wrapper file
 *  @date	31 Aug 2010
 *  @version	1.0
 */
#ifndef NM_WFI_CFGOPERATIONS
#define NM_WFI_CFGOPERATIONS
#include "wilc_wfi_netdevice.h"

#define NO_ENCRYPT		0

struct wireless_dev *wilc_create_wiphy(struct net_device *net, struct device *dev);
void wilc_free_wiphy(struct net_device *net);
int wilc_deinit_host_int(struct net_device *net);
int wilc_init_host_int(struct net_device *net);
void WILC_WFI_monitor_rx(struct wilc_vif *vif, u8 *buff, u32 size);
int WILC_WFI_deinit_mon_interface(void);
struct net_device *WILC_WFI_init_mon_interface(const char *name, struct net_device *real_dev);
void wilc_mgmt_frame_register(struct wiphy *wiphy, struct wireless_dev *wdev,
			      u16 frame_type, bool reg);
void wilc_sysfs_init(struct wilc_vif *vif1, struct wilc_vif *vif2);
void wilc_sysfs_exit(void);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)
void remove_network_from_shadow(struct timer_list *t);
#else
void remove_network_from_shadow(unsigned long arg);
#endif

#endif
