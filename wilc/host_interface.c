// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#include <linux/etherdevice.h>

#include "wilc_wfi_netdevice.h"
#include "linux_wlan.h"
#include "wilc_wfi_cfgoperations.h"

#define HOST_IF_SCAN_TIMEOUT                    4000
#define HOST_IF_CONNECT_TIMEOUT                 9500

#define FALSE_FRMWR_CHANNEL			100

#define REAL_JOIN_REQ				0

/* Generic success will return 0 */
#define WILC_SUCCESS		0	/* Generic success */

/* Negative numbers to indicate failures */
/* Generic Fail */
#define	WILC_FAIL		-100
/* Busy with another operation*/
#define	WILC_BUSY		-101
/* A given argument is invalid*/
#define	WILC_INVALID_ARGUMENT	-102
/* An API request would violate the Driver state machine
 * (i.e. to start PID while not camped)
 */
#define	WILC_INVALID_STATE	-103
/* In copy operations if the copied data is larger than the allocated buffer*/
#define	WILC_BUFFER_OVERFLOW	-104
/* null pointer is passed or used */
#define WILC_NULL_PTR		-105
#define	WILC_EMPTY		-107
#define WILC_FULL		-108
#define	WILC_TIMEOUT		-109
/* The required operation have been canceled by the user*/
#define WILC_CANCELED		-110
/* The Loaded file is corruped or having an invalid format */
#define WILC_INVALID_FILE	-112
/* Cant find the file to load */
#define WILC_NOT_FOUND		-113
#define WILC_NO_MEM		-114
#define WILC_UNSUPPORTED_VERSION -115
#define WILC_FILE_EOF		-116

struct send_buffered_eap {
	wilc_frmw_to_linux_t frmw_to_linux;
	free_eap_buf_param eap_buf_param;
	u8 *buff;
	unsigned int size;
	unsigned int pkt_offset;
	void *user_arg;
};

struct rcvd_async_info {
	u8 *buffer;
	u32 len;
};

struct set_multicast {
	bool enabled;
	u32 cnt;
	u8 *mc_list;
};

struct host_if_wowlan_trigger {
	u8 wowlan_trigger;
};

struct bt_coex_mode {
	u8 bt_coex;
};

struct host_if_set_ant {
	u8 mode;
	u8 antenna1;
	u8 antenna2;
	u8 gpio_mode;
};

struct del_all_sta {
	u8 assoc_sta;
	u8 mac[WILC_MAX_NUM_STA][ETH_ALEN];
};

struct wilc_op_mode {
	__le32 mode;
};

struct wilc_reg_frame {
	bool reg;
	u8 reg_id;
	__le32 frame_type;
} __packed;

struct wilc_drv_handler {
	__le32 handler;
	u8 mode;
} __packed;

struct wilc_wep_key {
	u8 index;
	u8 key_len;
	u8 key[0];
} __packed;

struct wilc_sta_wpa_ptk {
	u8 mac_addr[ETH_ALEN];
	u8 key_len;
	u8 key[0];
} __packed;

struct wilc_ap_wpa_ptk {
	u8 mac_addr[ETH_ALEN];
	u8 index;
	u8 key_len;
	u8 key[0];
} __packed;

struct wilc_gtk_key {
	u8 mac_addr[ETH_ALEN];
	u8 rsc[8];
	u8 index;
	u8 key_len;
	u8 key[0];
} __packed;

union message_body {
	struct rcvd_net_info net_info;
	struct rcvd_async_info async_info;
	struct set_multicast multicast_info;
	struct remain_ch remain_on_ch;
	char *data;
	struct send_buffered_eap send_buff_eap;
	struct host_if_set_ant set_ant;
	struct host_if_wowlan_trigger wow_trigger;
	struct bt_coex_mode bt_coex_mode;
};

struct host_if_msg {
	union message_body body;
	struct wilc_vif *vif;
	struct work_struct work;
	void (*fn)(struct work_struct *ws);
	struct completion work_comp;
	bool is_sync;
};

struct join_bss_param {
	enum bss_types bss_type;
	u8 dtim_period;
	u16 beacon_period;
	u16 cap_info;
	u8 bssid[6];
	char ssid[MAX_SSID_LEN];
	u8 ssid_len;
	u8 supp_rates[MAX_RATES_SUPPORTED + 1];
	u8 ht_capable;
	u8 wmm_cap;
	u8 uapsd_cap;
	bool rsn_found;
	u8 rsn_grp_policy;
	u8 mode_802_11i;
	u8 rsn_pcip_policy[3];
	u8 rsn_auth_policy[3];
	u8 rsn_cap[2];
	u32 tsf;
	u8 noa_enabled;
	u8 opp_enabled;
	u8 ct_window;
	u8 cnt;
	u8 idx;
	u8 duration[4];
	u8 interval[4];
	u8 start_time[4];
};

static struct host_if_drv *terminated_handle;
static struct mutex hif_deinit_lock;

/* 'msg' should be free by the caller for syc */
static struct host_if_msg*
wilc_alloc_work(struct wilc_vif *vif, void (*work_fun)(struct work_struct *),
		bool is_sync)
{
	struct host_if_msg *msg;

	if (!work_fun)
		return ERR_PTR(-EINVAL);

	msg = kzalloc(sizeof(*msg), GFP_ATOMIC);
	if (!msg)
		return ERR_PTR(-ENOMEM);
	msg->fn = work_fun;
	msg->vif = vif;
	msg->is_sync = is_sync;
	if (is_sync)
		init_completion(&msg->work_comp);

	return msg;
}

static int wilc_enqueue_work(struct host_if_msg *msg)
{
	INIT_WORK(&msg->work, msg->fn);

	if (!msg->vif || !msg->vif->wilc || !msg->vif->wilc->hif_workqueue)
		return -EINVAL;

	if (!queue_work(msg->vif->wilc->hif_workqueue, &msg->work))
		return -EINVAL;

	return 0;
}

/* The idx starts from 0 to (NUM_CONCURRENT_IFC - 1), but 0 index used as
 * special purpose in wilc device, so we add 1 to the index to starts from 1.
 * As a result, the returned index will be 1 to NUM_CONCURRENT_IFC.
 */
int wilc_get_vif_idx(struct wilc_vif *vif)
{
	return vif->idx + 1;
}

/* We need to minus 1 from idx which is from wilc device to get real index
 * of wilc->vif[], because we add 1 when pass to wilc device in the function
 * wilc_get_vif_idx.
 * As a result, the index should be between 0 and (NUM_CONCURRENT_IFC - 1).
 */
static struct wilc_vif *wilc_get_vif_from_idx(struct wilc *wilc, int idx)
{
	int index = idx - 1;

	if (index < 0 || index >= WILC_NUM_CONCURRENT_IFC)
		return NULL;

	return wilc->vif[index];
}

void filter_shadow_scan(struct wilc_priv *priv, u8 *ch_freq_list,
			u8 ch_list_len)
{
	int i;
	int ch_index;
	int j;
	struct network_info *net_info;

	if (ch_list_len == 0)
		return;

	for (i = 0; i < priv->scanned_cnt;) {
		net_info = &priv->scanned_shadow[i];

		for (ch_index = 0; ch_index < ch_list_len; ch_index++)
			if (net_info->ch == (ch_freq_list[ch_index] + 1))
				break;

		/* filter only un-matched channels */
		if (ch_index != ch_list_len) {
			i++;
			continue;
		}

		kfree(net_info->ies);
		net_info->ies = NULL;

		kfree(net_info->join_params);
		net_info->join_params = NULL;

		for (j = i; j < priv->scanned_cnt - 1; j++)
			priv->scanned_shadow[j] = priv->scanned_shadow[j + 1];

		priv->scanned_cnt--;
	}
}

static void handle_send_buffered_eap(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);
	struct wilc_vif *vif = msg->vif;
	struct send_buffered_eap *hif_buff_eap = &msg->body.send_buff_eap;

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Sending bufferd eapol to WPAS\n");
	if (!hif_buff_eap->buff)
		goto out;

	if (hif_buff_eap->frmw_to_linux)
		hif_buff_eap->frmw_to_linux(vif, hif_buff_eap->buff,
					    hif_buff_eap->size,
					    hif_buff_eap->pkt_offset,
					    PKT_STATUS_BUFFERED);
	if (hif_buff_eap->eap_buf_param)
		hif_buff_eap->eap_buf_param(hif_buff_eap->user_arg);

	if (hif_buff_eap->buff != NULL) {
		kfree(hif_buff_eap->buff);
		hif_buff_eap->buff = NULL;
	}

out:
	kfree(msg);
}

int wilc_scan(struct wilc_vif *vif, u8 scan_source, u8 scan_type,
	      u8 *ch_freq_list, u8 ch_list_len, const u8 *ies,
	      size_t ies_len, wilc_scan_result scan_result, void *user_arg,
	      struct hidden_network *hidden_net)
{
	struct wiphy *wiphy = vif->ndev->ieee80211_ptr->wiphy;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	int result = 0;
	struct wid wid_list[5];
	u32 index = 0;
	u32 i;
	u8 *buffer;
	u8 valuesize = 0;
	u8 *hdn_ntwk_wid_val = NULL;
	struct host_if_drv *hif_drv = vif->hif_drv;
	struct host_if_drv *hif_drv_p2p = get_drv_hndl_by_ifc(vif->wilc,
							      WILC_P2P_IFC);
	struct host_if_drv *hif_drv_wlan = get_drv_hndl_by_ifc(vif->wilc,
							       WILC_WLAN_IFC);

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Setting SCAN params\n");
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Scanning: In [%d] state\n",
		   hif_drv->hif_state);

	if (hif_drv_p2p != NULL) {
		if (hif_drv_p2p->hif_state != HOST_IF_IDLE &&
		    hif_drv_p2p->hif_state != HOST_IF_CONNECTED) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Don't scan. P2P_IFC is in state [%d]\n",
				   hif_drv_p2p->hif_state);
			result = -EBUSY;
			goto error;
		}
	}

	if (hif_drv_wlan != NULL) {
		if (hif_drv_wlan->hif_state != HOST_IF_IDLE &&
		    hif_drv_wlan->hif_state != HOST_IF_CONNECTED) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Don't scan. WLAN_IFC is in state [%d]\n",
				   hif_drv_wlan->hif_state);
			result = -EBUSY;
			goto error;
		}
	}
	if (vif->connecting) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Don't do scan in (CONNECTING) state\n");
		result = -EBUSY;
		goto error;
	}
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	if (vif->obtaining_ip) {
		PRINT_ER(vif->ndev, "Don't do obss scan\n");
		result = -EBUSY;
		goto error;
	}
#endif

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Setting SCAN params\n");
	hif_drv->usr_scan_req.ch_cnt = 0;

	wid_list[index].id = WID_SSID_PROBE_REQ;
	wid_list[index].type = WID_STR;

	for (i = 0; i < hidden_net->n_ssids; i++)
		valuesize += ((hidden_net->net_info[i].ssid_len) + 1);
	hdn_ntwk_wid_val = kmalloc(valuesize + 1, GFP_KERNEL);
	wid_list[index].val = hdn_ntwk_wid_val;
	if (wid_list[index].val) {
		buffer = wid_list[index].val;

		*buffer++ = hidden_net->n_ssids;

		PRINT_INFO(vif->ndev, HOSTINF_DBG,
			   "In Handle_ProbeRequest number of ssid %d\n",
			 hidden_net->n_ssids);
		for (i = 0; i < hidden_net->n_ssids; i++) {
			*buffer++ = hidden_net->net_info[i].ssid_len;
			memcpy(buffer, hidden_net->net_info[i].ssid,
			       hidden_net->net_info[i].ssid_len);
			buffer += hidden_net->net_info[i].ssid_len;
		}

		wid_list[index].size = (s32)(valuesize + 1);
		index++;
	}

	wid_list[index].id = WID_INFO_ELEMENT_PROBE;
	wid_list[index].type = WID_BIN_DATA;
	wid_list[index].val = (s8 *)ies;
	wid_list[index].size = ies_len;
	index++;

	wid_list[index].id = WID_SCAN_TYPE;
	wid_list[index].type = WID_CHAR;
	wid_list[index].size = sizeof(char);
	wid_list[index].val = (s8 *)&scan_type;
	index++;

	wid_list[index].id = WID_SCAN_CHANNEL_LIST;
	wid_list[index].type = WID_BIN_DATA;

	if (ch_freq_list && ch_list_len > 0) {
		for (i = 0; i < ch_list_len; i++) {
			if (ch_freq_list[i] > 0)
				ch_freq_list[i] -= 1;
		}
	}

	wid_list[index].val = ch_freq_list;
	wid_list[index].size = ch_list_len;
	index++;

	wid_list[index].id = WID_START_SCAN_REQ;
	wid_list[index].type = WID_CHAR;
	wid_list[index].size = sizeof(char);
	wid_list[index].val = (s8 *)&scan_source;
	index++;

    /*
     * Remove APs from shadow scan list which are
     * not in the requested scan channels list
     */
	filter_shadow_scan(priv, ch_freq_list, ch_list_len);

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, wid_list,
				      index,
				      wilc_get_vif_idx(vif));

	if (result) {
		PRINT_ER(vif->ndev, "Failed to send scan parameters\n");
		goto error;
	} else {
		hif_drv->usr_scan_req.scan_result = scan_result;
		hif_drv->usr_scan_req.arg = user_arg;
		hif_drv->scan_timer_vif = vif;
		PRINT_INFO(vif->ndev, HOSTINF_DBG,
			   ">> Starting the SCAN timer\n");
#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
		hif_drv->scan_timer.data = (unsigned long)hif_drv;
#endif
		mod_timer(&hif_drv->scan_timer,
			  jiffies + msecs_to_jiffies(HOST_IF_SCAN_TIMEOUT));
	}

error:
	kfree(hidden_net->net_info);
	kfree(hdn_ntwk_wid_val);

	return result;
}

s32 handle_scan_done(struct wilc_vif *vif, enum scan_event evt)
{
	s32 result = 0;
	u8 abort_running_scan;
	struct wid wid;
	struct host_if_drv *hif_drv = vif->hif_drv;
	struct user_scan_req *scan_req;
	u8 null_bssid[6] = {0};

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "handling scan done\n");

	if (!hif_drv) {
		PRINT_ER(vif->ndev, "hif driver is NULL\n");
		return result;
	}

	if (evt == SCAN_EVENT_DONE) {
		if (memcmp(hif_drv->assoc_bssid, null_bssid, ETH_ALEN) == 0)
			hif_drv->hif_state = HOST_IF_IDLE;
		else
			hif_drv->hif_state = HOST_IF_CONNECTED;
	} else if (evt == SCAN_EVENT_ABORTED) {
		PRINT_INFO(vif->ndev, GENERIC_DBG, "Abort running scan\n");
		abort_running_scan = 1;
		wid.id = WID_ABORT_RUNNING_SCAN;
		wid.type = WID_CHAR;
		wid.val = (s8 *)&abort_running_scan;
		wid.size = sizeof(char);

		result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
					      wilc_get_vif_idx(vif));

		if (result) {
			PRINT_ER(vif->ndev, "Failed to set abort running\n");
			result = -EFAULT;
		}
	}

	scan_req = &hif_drv->usr_scan_req;
	if (scan_req->scan_result) {
		scan_req->scan_result(evt, NULL, scan_req->arg, NULL);
		scan_req->scan_result = NULL;
	}

	return result;
}

static int wilc_send_connect_wid(struct wilc_vif *vif)
{
	int result = 0;
	struct wid wid_list[8];
	u32 wid_cnt = 0, dummyval = 0;
	u8 *cur_byte = NULL;
	struct host_if_drv *hif_drv = vif->hif_drv;
	struct user_conn_req *conn_attr = &hif_drv->usr_conn_req;
	struct join_bss_param *bss_param = hif_drv->usr_conn_req.param;
	struct host_if_drv *hif_drv_p2p = get_drv_hndl_by_ifc(vif->wilc,
							      WILC_P2P_IFC);
	struct host_if_drv *hif_drv_wlan = get_drv_hndl_by_ifc(vif->wilc,
							       WILC_WLAN_IFC);

	if (hif_drv_p2p != NULL) {
		if (hif_drv_p2p->hif_state == HOST_IF_SCANNING) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Don't scan. P2P_IFC is in state [%d]\n",
			 hif_drv_p2p->hif_state);
			 result = -EFAULT;
			goto error;
		}
	}
	if (hif_drv_wlan != NULL) {
		if (hif_drv_wlan->hif_state == HOST_IF_SCANNING) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Don't scan. WLAN_IFC is in state [%d]\n",
			 hif_drv_wlan->hif_state);
			result = -EFAULT;
			goto error;
		}
	}

	wid_list[wid_cnt].id = WID_SUCCESS_FRAME_COUNT;
	wid_list[wid_cnt].type = WID_INT;
	wid_list[wid_cnt].size = sizeof(u32);
	wid_list[wid_cnt].val = (s8 *)(&(dummyval));
	wid_cnt++;

	wid_list[wid_cnt].id = WID_RECEIVED_FRAGMENT_COUNT;
	wid_list[wid_cnt].type = WID_INT;
	wid_list[wid_cnt].size = sizeof(u32);
	wid_list[wid_cnt].val = (s8 *)(&(dummyval));
	wid_cnt++;

	wid_list[wid_cnt].id = WID_FAILED_COUNT;
	wid_list[wid_cnt].type = WID_INT;
	wid_list[wid_cnt].size = sizeof(u32);
	wid_list[wid_cnt].val = (s8 *)(&(dummyval));
	wid_cnt++;

	wid_list[wid_cnt].id = WID_INFO_ELEMENT_ASSOCIATE;
	wid_list[wid_cnt].type = WID_BIN_DATA;
	wid_list[wid_cnt].val = conn_attr->ies;
	wid_list[wid_cnt].size = conn_attr->ies_len;
	wid_cnt++;

	wid_list[wid_cnt].id = WID_11I_MODE;
	wid_list[wid_cnt].type = WID_CHAR;
	wid_list[wid_cnt].size = sizeof(char);
	wid_list[wid_cnt].val = (s8 *)&conn_attr->security;
	wid_cnt++;

	PRINT_D(vif->ndev, HOSTINF_DBG, "Encrypt Mode = %x\n",
		conn_attr->security);
	wid_list[wid_cnt].id = WID_AUTH_TYPE;
	wid_list[wid_cnt].type = WID_CHAR;
	wid_list[wid_cnt].size = sizeof(char);
	wid_list[wid_cnt].val = (s8 *)&conn_attr->auth_type;
	wid_cnt++;

	PRINT_D(vif->ndev, HOSTINF_DBG, "Authentication Type = %x\n",
		conn_attr->auth_type);
	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Connecting to network of SSID %s on channel %d\n",
		 conn_attr->ssid, conn_attr->ch);

	wid_list[wid_cnt].id = WID_JOIN_REQ_EXTENDED;
	wid_list[wid_cnt].type = WID_STR;
	wid_list[wid_cnt].size = 112;
	wid_list[wid_cnt].val = kmalloc(wid_list[wid_cnt].size, GFP_KERNEL);

	if (!wid_list[wid_cnt].val) {
		result = -EFAULT;
		goto error;
	}

	cur_byte = wid_list[wid_cnt].val;

	if (conn_attr->ssid) {
		memcpy(cur_byte, conn_attr->ssid, conn_attr->ssid_len);
		cur_byte[conn_attr->ssid_len] = '\0';
	}
	cur_byte += MAX_SSID_LEN;
	*(cur_byte++) = WILC_FW_BSS_TYPE_INFRA;

	if (conn_attr->ch >= 1 && conn_attr->ch <= 14) {
		*(cur_byte++) = conn_attr->ch;
	} else {
		PRINT_ER(vif->ndev, "Channel out of range\n");
		*(cur_byte++) = 0xFF;
	}
	put_unaligned_le16(bss_param->cap_info, cur_byte);
	cur_byte += 2;
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "* Cap Info %0x*\n",
		   (*(cur_byte - 2) | ((*(cur_byte - 1)) << 8)));

	if (conn_attr->bssid)
		memcpy(cur_byte, conn_attr->bssid, 6);
	cur_byte += 6;

	if (conn_attr->bssid)
		memcpy(cur_byte, conn_attr->bssid, 6);
	cur_byte += 6;

	put_unaligned_le16(bss_param->beacon_period, cur_byte);
	cur_byte += 2;
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "* Beacon Period %d*\n",
		   *(cur_byte - 2) | ((*(cur_byte - 1)) << 8));
	*(cur_byte++)  =  bss_param->dtim_period;
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "* DTIM Period %d*\n",
		   *(cur_byte - 1));

	memcpy(cur_byte, bss_param->supp_rates, MAX_RATES_SUPPORTED + 1);
	cur_byte += (MAX_RATES_SUPPORTED + 1);

	*(cur_byte++)  =  bss_param->wmm_cap;
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "* wmm cap%d*\n", *(cur_byte - 1));
	*(cur_byte++)  = bss_param->uapsd_cap;

	*(cur_byte++)  = bss_param->ht_capable;
	conn_attr->ht_capable = bss_param->ht_capable;

	*(cur_byte++)  =  bss_param->rsn_found;
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "* rsn found %d*\n",
		   *(cur_byte - 1));
	*(cur_byte++)  =  bss_param->rsn_grp_policy;
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "* rsn group policy %0x*\n",
		   *(cur_byte - 1));
	*(cur_byte++) =  bss_param->mode_802_11i;
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "* mode_802_11i %d*\n",
		   *(cur_byte - 1));
	memcpy(cur_byte, bss_param->rsn_pcip_policy,
	       sizeof(bss_param->rsn_pcip_policy));
	cur_byte += sizeof(bss_param->rsn_pcip_policy);

	memcpy(cur_byte, bss_param->rsn_auth_policy,
	       sizeof(bss_param->rsn_auth_policy));
	cur_byte += sizeof(bss_param->rsn_auth_policy);

	memcpy(cur_byte, bss_param->rsn_cap, sizeof(bss_param->rsn_cap));
	cur_byte += sizeof(bss_param->rsn_cap);

	*(cur_byte++) = REAL_JOIN_REQ;
	*(cur_byte++) = bss_param->noa_enabled;

	if (bss_param->noa_enabled) {
		PRINT_INFO(vif->ndev, HOSTINF_DBG, "NOA present\n");
		put_unaligned_le32(bss_param->tsf, cur_byte);
		cur_byte += 4;

		*(cur_byte++) = bss_param->idx;
		*(cur_byte++) = bss_param->opp_enabled;

		if (bss_param->opp_enabled)
			*(cur_byte++) = bss_param->ct_window;

		*(cur_byte++) = bss_param->cnt;

		memcpy(cur_byte, bss_param->duration,
		       sizeof(bss_param->duration));
		cur_byte += sizeof(bss_param->duration);

		memcpy(cur_byte, bss_param->interval,
		       sizeof(bss_param->interval));
		cur_byte += sizeof(bss_param->interval);

		memcpy(cur_byte, bss_param->start_time,
		       sizeof(bss_param->start_time));
		cur_byte += sizeof(bss_param->start_time);
	} else {
		PRINT_INFO(vif->ndev, HOSTINF_DBG, "NOA not present\n");
	}

	cur_byte = wid_list[wid_cnt].val;
	wid_cnt++;

	PRINT_INFO(vif->ndev, GENERIC_DBG, "send HOST_IF_WAITING_CONN_RESP\n");

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, wid_list,
				      wid_cnt,
				      wilc_get_vif_idx(vif));
	if (result) {
		PRINT_ER(vif->ndev, "failed to send config packet\n");
		kfree(cur_byte);
		goto error;
	} else {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "set HOST_IF_WAITING_CONN_RESP\n");
		hif_drv->hif_state = HOST_IF_WAITING_CONN_RESP;
	}
	kfree(cur_byte);
	return 0;

error:
	kfree(conn_attr->bssid);
	conn_attr->bssid = NULL;

	kfree(conn_attr->ssid);
	conn_attr->ssid = NULL;

	kfree(conn_attr->ies);
	conn_attr->ies = NULL;

	return result;
}

static void handle_connect_timeout(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);
	struct wilc_vif *vif = msg->vif;
	int result;
	struct connect_info info;
	struct wid wid;
	u16 dummy_reason_code = 0;
	struct host_if_drv *hif_drv = vif->hif_drv;

	if (!hif_drv) {
		PRINT_ER(vif->ndev, "hif driver is NULL\n");
		goto out;
	}

	hif_drv->hif_state = HOST_IF_IDLE;

	memset(&info, 0, sizeof(struct connect_info));

	if (hif_drv->usr_conn_req.conn_result) {
		if (hif_drv->usr_conn_req.bssid) {
			memcpy(info.bssid,
			       hif_drv->usr_conn_req.bssid, 6);
		}

		if (hif_drv->usr_conn_req.ies) {
			info.req_ies_len = hif_drv->usr_conn_req.ies_len;
			info.req_ies = kmemdup(hif_drv->usr_conn_req.ies,
					       hif_drv->usr_conn_req.ies_len,
					       GFP_KERNEL);
			if (!info.req_ies)
				goto out;
		}

		hif_drv->usr_conn_req.conn_result(EVENT_CONN_RESP,
						  &info,
						  WILC_MAC_STATUS_DISCONNECTED,
						  NULL,
						  hif_drv->usr_conn_req.arg);

		kfree(info.req_ies);
		info.req_ies = NULL;
	} else {
		PRINT_ER(vif->ndev, "conn_result is NULL\n");
	}

	wid.id = WID_DISCONNECT;
	wid.type = WID_CHAR;
	wid.val = (s8 *)&dummy_reason_code;
	wid.size = sizeof(char);

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Sending disconnect request\n");
	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to send disconect\n");

	hif_drv->usr_conn_req.ssid_len = 0;
	kfree(hif_drv->usr_conn_req.ssid);
	hif_drv->usr_conn_req.ssid = NULL;
	kfree(hif_drv->usr_conn_req.bssid);
	hif_drv->usr_conn_req.bssid = NULL;
	hif_drv->usr_conn_req.ies_len = 0;
	kfree(hif_drv->usr_conn_req.ies);
	hif_drv->usr_conn_req.ies = NULL;

out:
	kfree(msg);
}

static void host_int_fill_join_bss_param(struct join_bss_param *param, u8 *ies,
					 u16 *out_index, u8 *pcipher_tc,
					 u8 *auth_total_cnt, u32 tsf_lo,
					 u8 *rates_no)
{
	u8 ext_rates_no;
	u16 offset;
	u8 pcipher_cnt;
	u8 auth_cnt;
	u8 i, j;
	u16 index = *out_index;

	if (ies[index] == WLAN_EID_SUPP_RATES) {
		*rates_no = ies[index + 1];
		param->supp_rates[0] = *rates_no;
		index += 2;

		for (i = 0; i < *rates_no; i++)
			param->supp_rates[i + 1] = ies[index + i];

		index += *rates_no;
	} else if (ies[index] == WLAN_EID_EXT_SUPP_RATES) {
		ext_rates_no = ies[index + 1];
		if (ext_rates_no > (MAX_RATES_SUPPORTED - *rates_no))
			param->supp_rates[0] = MAX_RATES_SUPPORTED;
		else
			param->supp_rates[0] += ext_rates_no;
		index += 2;
		for (i = 0; i < (param->supp_rates[0] - *rates_no); i++)
			param->supp_rates[*rates_no + i + 1] = ies[index + i];

		index += ext_rates_no;
	} else if (ies[index] == WLAN_EID_HT_CAPABILITY) {
		param->ht_capable = true;
		index += ies[index + 1] + 2;
	} else if ((ies[index] == WLAN_EID_VENDOR_SPECIFIC) &&
		   (ies[index + 2] == 0x00) && (ies[index + 3] == 0x50) &&
		   (ies[index + 4] == 0xF2) && (ies[index + 5] == 0x02) &&
		   ((ies[index + 6] == 0x00) || (ies[index + 6] == 0x01)) &&
		   (ies[index + 7] == 0x01)) {
		param->wmm_cap = true;

		if (ies[index + 8] & BIT(7))
			param->uapsd_cap = true;
		index += ies[index + 1] + 2;
	} else if ((ies[index] == WLAN_EID_VENDOR_SPECIFIC) &&
		 (ies[index + 2] == 0x50) && (ies[index + 3] == 0x6f) &&
		 (ies[index + 4] == 0x9a) &&
		 (ies[index + 5] == 0x09) && (ies[index + 6] == 0x0c)) {
		u16 p2p_cnt;

		param->tsf = tsf_lo;
		param->noa_enabled = 1;
		param->idx = ies[index + 9];

		if (ies[index + 10] & BIT(7)) {
			param->opp_enabled = 1;
			param->ct_window = ies[index + 10];
		} else {
			param->opp_enabled = 0;
		}

		param->cnt = ies[index + 11];
		p2p_cnt = index + 12;

		memcpy(param->duration, ies + p2p_cnt, 4);
		p2p_cnt += 4;

		memcpy(param->interval, ies + p2p_cnt, 4);
		p2p_cnt += 4;

		memcpy(param->start_time, ies + p2p_cnt, 4);

		index += ies[index + 1] + 2;
	} else if ((ies[index] == WLAN_EID_RSN) ||
		 ((ies[index] == WLAN_EID_VENDOR_SPECIFIC) &&
		  (ies[index + 2] == 0x00) &&
		  (ies[index + 3] == 0x50) && (ies[index + 4] == 0xF2) &&
		  (ies[index + 5] == 0x01))) {
		u16 rsn_idx = index;

		if (ies[rsn_idx] == WLAN_EID_RSN) {
			param->mode_802_11i = 2;
		} else {
			if (param->mode_802_11i == 0)
				param->mode_802_11i = 1;
			rsn_idx += 4;
		}

		rsn_idx += 7;
		param->rsn_grp_policy = ies[rsn_idx];
		rsn_idx++;
		offset = ies[rsn_idx] * 4;
		pcipher_cnt = (ies[rsn_idx] > 3) ? 3 : ies[rsn_idx];
		rsn_idx += 2;

		i = *pcipher_tc;
		j = 0;
		for (; i < (pcipher_cnt + *pcipher_tc) && i < 3; i++, j++) {
			u8 *policy =  &param->rsn_pcip_policy[i];

			*policy = ies[rsn_idx + ((j + 1) * 4) - 1];
		}

		*pcipher_tc += pcipher_cnt;
		rsn_idx += offset;

		offset = ies[rsn_idx] * 4;

		auth_cnt = (ies[rsn_idx] > 3) ? 3 : ies[rsn_idx];
		rsn_idx += 2;
		i = *auth_total_cnt;
		j = 0;
		for (; i < (*auth_total_cnt + auth_cnt); i++, j++) {
			u8 *policy =  &param->rsn_auth_policy[i];

			*policy = ies[rsn_idx + ((j + 1) * 4) - 1];
		}

		*auth_total_cnt += auth_cnt;
		rsn_idx += offset;

		if (ies[index] == WLAN_EID_RSN) {
			param->rsn_cap[0] = ies[rsn_idx];
			param->rsn_cap[1] = ies[rsn_idx + 1];
			rsn_idx += 2;
		}
		param->rsn_found = true;
		index += ies[index + 1] + 2;
	} else {
		index += ies[index + 1] + 2;
	}

	*out_index = index;
}

static void *host_int_parse_join_bss_param(struct network_info *info)
{
	struct join_bss_param *param;
	u16 index = 0;
	u8 rates_no = 0;
	u8 pcipher_total_cnt = 0;
	u8 auth_total_cnt = 0;

	param = kzalloc(sizeof(*param), GFP_KERNEL);
	if (!param)
		return NULL;

	param->dtim_period = info->dtim_period;
	param->beacon_period = info->beacon_period;
	param->cap_info = info->cap_info;
	memcpy(param->bssid, info->bssid, 6);
	memcpy((u8 *)param->ssid, info->ssid, info->ssid_len + 1);
	param->ssid_len = info->ssid_len;
	memset(param->rsn_pcip_policy, 0xFF, 3);
	memset(param->rsn_auth_policy, 0xFF, 3);

	while (index < info->ies_len)
		host_int_fill_join_bss_param(param, info->ies, &index,
					     &pcipher_total_cnt,
					     &auth_total_cnt, info->tsf_lo,
					     &rates_no);

	return (void *)param;
}

static inline u8 *get_bssid(struct ieee80211_mgmt *mgmt)
{
	if (ieee80211_has_fromds(mgmt->frame_control))
		return mgmt->sa;
	else if (ieee80211_has_tods(mgmt->frame_control))
		return mgmt->da;
	else
		return mgmt->bssid;
}

static s32 wilc_parse_network_info(struct wilc_vif *vif, u8 *msg_buffer,
				   struct network_info **ret_network_info)
{
	struct network_info *info;
	struct ieee80211_mgmt *mgt;
	u8 *wid_val, *msa, *ies;
	u16 wid_len, rx_len, ies_len;
	u8 msg_type;
	size_t offset;
	const u8 *ch_elm, *tim_elm, *ssid_elm;

	msg_type = msg_buffer[0];
	if ('N' != msg_type)
		return -EFAULT;

	wid_len = get_unaligned_le16(&msg_buffer[6]);
	wid_val = &msg_buffer[8];

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->rssi = wid_val[0];

	msa = &wid_val[1];
	mgt = (struct ieee80211_mgmt *)&wid_val[1];
	rx_len = wid_len - 1;

	if (ieee80211_is_probe_resp(mgt->frame_control)) {
		info->cap_info = le16_to_cpu(mgt->u.probe_resp.capab_info);
		info->beacon_period = le16_to_cpu(mgt->u.probe_resp.beacon_int);
		info->tsf = le64_to_cpu(mgt->u.probe_resp.timestamp);
		info->tsf_lo = (u32)info->tsf;
		offset = offsetof(struct ieee80211_mgmt, u.probe_resp.variable);
	} else if (ieee80211_is_beacon(mgt->frame_control)) {
		info->cap_info = le16_to_cpu(mgt->u.beacon.capab_info);
		info->beacon_period = le16_to_cpu(mgt->u.beacon.beacon_int);
		info->tsf = le64_to_cpu(mgt->u.beacon.timestamp);
		info->tsf_lo = (u32)info->tsf;
		offset = offsetof(struct ieee80211_mgmt, u.beacon.variable);
	} else {
		/* only process probe response and beacon frame */
		kfree(info);
		return -EIO;
	}

	PRINT_INFO(vif->ndev, CORECONFIG_DBG, "TSF :%x\n", info->tsf_lo);

	ether_addr_copy(info->bssid, get_bssid(mgt));

	ies = mgt->u.beacon.variable;
	ies_len = rx_len - offset;
	if (ies_len <= 0) {
		kfree(info);
		return -EIO;
	}

	info->ies = kmemdup(ies, ies_len, GFP_KERNEL);
	if (!info->ies) {
		kfree(info);
		return -ENOMEM;
	}

	info->ies_len = ies_len;

	ssid_elm = cfg80211_find_ie(WLAN_EID_SSID, ies, ies_len);
	if (ssid_elm) {
		info->ssid_len = ssid_elm[1];
		if (info->ssid_len <= IEEE80211_MAX_SSID_LEN)
			memcpy(info->ssid, ssid_elm + 2, info->ssid_len);
		else
			info->ssid_len = 0;
	}

	ch_elm = cfg80211_find_ie(WLAN_EID_DS_PARAMS, ies, ies_len);
	if (ch_elm && ch_elm[1] > 0)
		info->ch = ch_elm[2];

	tim_elm = cfg80211_find_ie(WLAN_EID_TIM, ies, ies_len);
	if (tim_elm && tim_elm[1] >= 2)
		info->dtim_period = tim_elm[3];

	*ret_network_info = info;

	return 0;
}

static void handle_rcvd_ntwrk_info(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);
	struct wilc_vif *vif = msg->vif;
	struct rcvd_net_info *rcvd_info = &msg->body.net_info;
	u32 i;
	bool found;
	struct network_info *info = NULL;
	void *params;
	struct host_if_drv *hif_drv = vif->hif_drv;
	struct user_scan_req *scan_req = &hif_drv->usr_scan_req;
	int ret;

	found = true;
	PRINT_D(vif->ndev, HOSTINF_DBG, "Handling received network info\n");

	if (!scan_req->scan_result)
		goto done;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "State: Scanning, parsing network information received\n");
	ret = wilc_parse_network_info(vif, rcvd_info->buffer, &info);
	if (ret || !info || !scan_req->scan_result) {
		PRINT_ER(vif->ndev, "info or scan result NULL\n");
		goto done;
	}

	for (i = 0; i < scan_req->ch_cnt; i++) {
		if (memcmp(scan_req->net_info[i].bssid, info->bssid, 6) == 0) {
			if (info->rssi <= scan_req->net_info[i].rssi) {
				PRINT_INFO(vif->ndev, HOSTINF_DBG,
					   "Network previously discovered\n");
				goto done;
			} else {
				scan_req->net_info[i].rssi = info->rssi;
				found = false;
				break;
			}
		}
	}

	if (found) {
		PRINT_INFO(vif->ndev, HOSTINF_DBG, "New network found\n");
		if (scan_req->ch_cnt < MAX_NUM_SCANNED_NETWORKS) {
			scan_req->net_info[scan_req->ch_cnt].rssi = info->rssi;

			memcpy(scan_req->net_info[scan_req->ch_cnt].bssid,
			       info->bssid, 6);

			scan_req->ch_cnt++;

			info->new_network = true;
			params = host_int_parse_join_bss_param(info);

			scan_req->scan_result(SCAN_EVENT_NETWORK_FOUND, info,
					       scan_req->arg, params);
		} else {
			PRINT_WRN(vif->ndev, HOSTINF_DBG,
				  "Discovered networks exceeded max. limit\n");
		}
	} else {
		info->new_network = false;
		scan_req->scan_result(SCAN_EVENT_NETWORK_FOUND, info,
				      scan_req->arg, NULL);
	}

done:
	kfree(rcvd_info->buffer);
	rcvd_info->buffer = NULL;

	if (info) {
		kfree(info->ies);
		kfree(info);
	}

	kfree(msg);
}

static void host_int_get_assoc_res_info(struct wilc_vif *vif,
					u8 *assoc_resp_info,
					u32 max_assoc_resp_info_len,
					u32 *rcvd_assoc_resp_info_len)
{
	int result;
	struct wid wid;

	wid.id = WID_ASSOC_RES_INFO;
	wid.type = WID_STR;
	wid.val = assoc_resp_info;
	wid.size = max_assoc_resp_info_len;

	result = wilc_send_config_pkt(vif, WILC_GET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result) {
		*rcvd_assoc_resp_info_len = 0;
		PRINT_ER(vif->ndev, "Failed to send association response\n");
		return;
	}

	*rcvd_assoc_resp_info_len = wid.size;
}

static inline void host_int_free_user_conn_req(struct host_if_drv *hif_drv)
{
	hif_drv->usr_conn_req.ssid_len = 0;
	kfree(hif_drv->usr_conn_req.ssid);
	hif_drv->usr_conn_req.ssid = NULL;
	kfree(hif_drv->usr_conn_req.bssid);
	hif_drv->usr_conn_req.bssid = NULL;
	hif_drv->usr_conn_req.ies_len = 0;
	kfree(hif_drv->usr_conn_req.ies);
	hif_drv->usr_conn_req.ies = NULL;
}

static s32 wilc_parse_assoc_resp_info(u8 *buffer, u32 buffer_len,
				      struct connect_info *ret_conn_info)
{
	u8 *ies;
	u16 ies_len;
	struct assoc_resp *res = (struct assoc_resp *)buffer;

	ret_conn_info->status = le16_to_cpu(res->status_code);
	if (ret_conn_info->status == WLAN_STATUS_SUCCESS) {
		ies = &buffer[sizeof(*res)];
		ies_len = buffer_len - sizeof(*res);

		ret_conn_info->resp_ies = kmemdup(ies, ies_len, GFP_KERNEL);
		if (!ret_conn_info->resp_ies)
			return -ENOMEM;

		ret_conn_info->resp_ies_len = ies_len;
	}

	return 0;
}

static inline void host_int_parse_assoc_resp_info(struct wilc_vif *vif,
						  u8 mac_status)
{
	struct connect_info conn_info;
	struct host_if_drv *hif_drv = vif->hif_drv;

	memset(&conn_info, 0, sizeof(struct connect_info));

	if (mac_status == WILC_MAC_STATUS_CONNECTED) {
		u32 assoc_resp_info_len;

		memset(hif_drv->assoc_resp, 0, WILC_MAX_ASSOC_RESP_FRAME_SIZE);

		host_int_get_assoc_res_info(vif, hif_drv->assoc_resp,
					    WILC_MAX_ASSOC_RESP_FRAME_SIZE,
					    &assoc_resp_info_len);

		PRINT_D(vif->ndev, HOSTINF_DBG,
			"Received association response = %d\n",
			assoc_resp_info_len);
		if (assoc_resp_info_len != 0) {
			s32 err = 0;

			PRINT_INFO(vif->ndev, HOSTINF_DBG,
				   "Parsing association response\n");
			err = wilc_parse_assoc_resp_info(hif_drv->assoc_resp,
							 assoc_resp_info_len,
							 &conn_info);
			if (err)
				PRINT_ER(vif->ndev,
					 "wilc_parse_assoc_resp_info() returned error %d\n",
					 err);
		}
	}

	if (hif_drv->usr_conn_req.bssid) {
		memcpy(conn_info.bssid, hif_drv->usr_conn_req.bssid, 6);

		if (mac_status == WILC_MAC_STATUS_CONNECTED &&
		    conn_info.status == WLAN_STATUS_SUCCESS) {
			memcpy(hif_drv->assoc_bssid,
			       hif_drv->usr_conn_req.bssid, ETH_ALEN);
		}
	}

	if (hif_drv->usr_conn_req.ies) {
		conn_info.req_ies = kmemdup(hif_drv->usr_conn_req.ies,
					    hif_drv->usr_conn_req.ies_len,
					    GFP_KERNEL);
		if (conn_info.req_ies)
			conn_info.req_ies_len = hif_drv->usr_conn_req.ies_len;
	}

	del_timer(&hif_drv->connect_timer);
	hif_drv->usr_conn_req.conn_result(EVENT_CONN_RESP,
					  &conn_info, mac_status, NULL,
					  hif_drv->usr_conn_req.arg);

	if (mac_status == WILC_MAC_STATUS_CONNECTED &&
	    conn_info.status == WLAN_STATUS_SUCCESS) {
		PRINT_INFO(vif->ndev, HOSTINF_DBG,
			   "MAC status : CONNECTED and Connect Status : Successful\n");
		hif_drv->hif_state = HOST_IF_CONNECTED;

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		handle_pwrsave_for_IP(vif, IP_STATE_OBTAINING);
#endif
	} else {
		PRINT_INFO(vif->ndev, HOSTINF_DBG,
			   "MAC status : %d and Connect Status : %d\n",
			   mac_status, conn_info.status);
		hif_drv->hif_state = HOST_IF_IDLE;
	}

	kfree(conn_info.resp_ies);
	conn_info.resp_ies = NULL;

	kfree(conn_info.req_ies);
	conn_info.req_ies = NULL;
	host_int_free_user_conn_req(hif_drv);
}

static inline void host_int_handle_disconnect(struct wilc_vif *vif)
{
	struct disconnect_info disconn_info;
	struct host_if_drv *hif_drv = vif->hif_drv;
	wilc_connect_result conn_result = hif_drv->usr_conn_req.conn_result;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Received WILC_MAC_STATUS_DISCONNECTED from the FW\n");
	memset(&disconn_info, 0, sizeof(struct disconnect_info));

	if (hif_drv->usr_scan_req.scan_result) {
		PRINT_INFO(vif->ndev, HOSTINF_DBG,
			   "\n\n<< Abort the running OBSS Scan >>\n\n");
		del_timer(&hif_drv->scan_timer);
		handle_scan_done(vif, SCAN_EVENT_ABORTED);
	}

	disconn_info.reason = 0;
	disconn_info.ie = NULL;
	disconn_info.ie_len = 0;

	if (conn_result) {
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		handle_pwrsave_for_IP(vif, IP_STATE_DEFAULT);
#endif

		conn_result(EVENT_DISCONN_NOTIF,
			    NULL, 0, &disconn_info, hif_drv->usr_conn_req.arg);
	} else {
		PRINT_ER(vif->ndev, "Connect result NULL\n");
	}

	eth_zero_addr(hif_drv->assoc_bssid);

	host_int_free_user_conn_req(hif_drv);
	hif_drv->hif_state = HOST_IF_IDLE;
}

static void handle_rcvd_gnrl_async_info(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);
	struct wilc_vif *vif = msg->vif;
	struct rcvd_async_info *rcvd_info = &msg->body.async_info;
	u8 msg_type;
	u8 mac_status;
	u8 mac_status_reason_code;
	u8 mac_status_additional_info;
	struct host_if_drv *hif_drv = vif->hif_drv;

	if (!rcvd_info->buffer) {
		netdev_err(vif->ndev, "buffer is NULL\n");
		goto free_msg;
	}

	if (!hif_drv) {
		PRINT_ER(vif->ndev, "hif driver is NULL\n");
		goto free_rcvd_info;
	}
	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "Current State = %d,Received state = %d\n",
		   hif_drv->hif_state,
		   rcvd_info->buffer[7]);

	if (hif_drv->hif_state == HOST_IF_WAITING_CONN_RESP ||
	    hif_drv->hif_state == HOST_IF_CONNECTED ||
	    hif_drv->usr_scan_req.scan_result) {
		if (!hif_drv->usr_conn_req.conn_result) {
			PRINT_ER(vif->ndev, "conn_result is NULL\n");
			goto free_rcvd_info;
		}

		msg_type = rcvd_info->buffer[0];

		if ('I' != msg_type) {
			PRINT_ER(vif->ndev, "Received Message incorrect.\n");
			goto free_rcvd_info;
		}

		mac_status  = rcvd_info->buffer[7];
		mac_status_reason_code = rcvd_info->buffer[8];
		mac_status_additional_info = rcvd_info->buffer[9];
		PRINT_INFO(vif->ndev, HOSTINF_DBG,
			   "Received MAC status= %d Reason= %d Info = %d\n",
			   mac_status, mac_status_reason_code,
			   mac_status_additional_info);
		if (hif_drv->hif_state == HOST_IF_WAITING_CONN_RESP) {
			host_int_parse_assoc_resp_info(vif, mac_status);
		} else if ((mac_status == WILC_MAC_STATUS_DISCONNECTED) &&
			   (hif_drv->hif_state == HOST_IF_CONNECTED)) {
			host_int_handle_disconnect(vif);
		} else if ((mac_status == WILC_MAC_STATUS_DISCONNECTED) &&
			   (hif_drv->usr_scan_req.scan_result)) {
			PRINT_WRN(vif->ndev, HOSTINF_DBG,
				  "Received WILC_MAC_STATUS_DISCONNECTED. Abort the running Scan");
			del_timer(&hif_drv->scan_timer);
			if (hif_drv->usr_scan_req.scan_result)
				handle_scan_done(vif, SCAN_EVENT_ABORTED);
		}
	}

free_rcvd_info:
	kfree(rcvd_info->buffer);
	rcvd_info->buffer = NULL;

free_msg:
	kfree(msg);
}

int wilc_disconnect(struct wilc_vif *vif)
{
	struct wid wid;
	struct host_if_drv *hif_drv = vif->hif_drv;
	struct disconnect_info disconn_info;
	struct user_scan_req *scan_req;
	struct user_conn_req *conn_req;
	int result;
	u16 dummy_reason_code = 0;
	struct host_if_drv *hif_drv_p2p = get_drv_hndl_by_ifc(vif->wilc,
							      WILC_P2P_IFC);
	struct host_if_drv *hif_drv_wlan = get_drv_hndl_by_ifc(vif->wilc,
							       WILC_WLAN_IFC);

	if (hif_drv_wlan != NULL) {
		if (hif_drv_wlan->hif_state == HOST_IF_SCANNING) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Abort Scan. WLAN_IFC is in state [%d]\n",
				   hif_drv_wlan->hif_state);
			del_timer(&hif_drv_wlan->scan_timer);
			handle_scan_done(vif, SCAN_EVENT_ABORTED);
		}
	}
	if (hif_drv_p2p != NULL) {
		if (hif_drv_p2p->hif_state == HOST_IF_SCANNING) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Abort Scan. P2P_IFC is in state [%d]\n",
				   hif_drv_p2p->hif_state);
			del_timer(&hif_drv_p2p->scan_timer);
			handle_scan_done(vif, SCAN_EVENT_ABORTED);
		}
	}
	wid.id = WID_DISCONNECT;
	wid.type = WID_CHAR;
	wid.val = (s8 *)&dummy_reason_code;
	wid.size = sizeof(char);

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Sending disconnect request\n");

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	handle_pwrsave_for_IP(vif, IP_STATE_DEFAULT);
#endif

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));

	if (result) {
		PRINT_ER(vif->ndev, "Failed to send dissconect\n");
		return -ENOMEM;
	}

	memset(&disconn_info, 0, sizeof(struct disconnect_info));

	disconn_info.reason = 0;
	disconn_info.ie = NULL;
	disconn_info.ie_len = 0;
	scan_req = &hif_drv->usr_scan_req;
	conn_req = &hif_drv->usr_conn_req;

	if (scan_req->scan_result) {
		del_timer(&hif_drv->scan_timer);
		scan_req->scan_result(SCAN_EVENT_ABORTED, NULL, scan_req->arg,
				      NULL);
		scan_req->scan_result = NULL;
	}

	if (conn_req->conn_result) {
		if (hif_drv->hif_state == HOST_IF_WAITING_CONN_RESP) {
			struct connect_info connect;

			PRINT_INFO(vif->ndev, HOSTINF_DBG,
				   "supplicant requested disconnection\n");
			memset(&connect, 0, sizeof(struct connect_info));
			del_timer(&hif_drv->connect_timer);
			if (conn_req->bssid != NULL)
				memcpy(connect.bssid, conn_req->bssid, 6);
			if (conn_req->ies != NULL) {
				connect.req_ies_len = conn_req->ies_len;
				connect.req_ies = kmalloc(conn_req->ies_len,
							  GFP_ATOMIC);
				memcpy(connect.req_ies,
				       conn_req->ies,
				       conn_req->ies_len);
			}
			conn_req->conn_result(EVENT_CONN_RESP,
					      &connect,
					      WILC_MAC_STATUS_DISCONNECTED,
					      NULL, conn_req->arg);

			if (connect.req_ies != NULL) {
				kfree(connect.req_ies);
				connect.req_ies = NULL;
			}

		} else if (hif_drv->hif_state == HOST_IF_CONNECTED) {
			conn_req->conn_result(EVENT_DISCONN_NOTIF,
					      NULL, 0, &disconn_info,
					      conn_req->arg);
		}
	} else {
		PRINT_ER(vif->ndev, "conn_result = NULL\n");
	}

	hif_drv->hif_state = HOST_IF_IDLE;

	eth_zero_addr(hif_drv->assoc_bssid);

	conn_req->ssid_len = 0;
	kfree(conn_req->ssid);
	conn_req->ssid = NULL;
	kfree(conn_req->bssid);
	conn_req->bssid = NULL;
	conn_req->ies_len = 0;
	kfree(conn_req->ies);
	conn_req->ies = NULL;

	return 0;
}

void wilc_resolve_disconnect_aberration(struct wilc_vif *vif)
{
	if (!vif->hif_drv)
		return;
	if (vif->hif_drv->hif_state == HOST_IF_WAITING_CONN_RESP ||
	    vif->hif_drv->hif_state == HOST_IF_CONNECTING) {
		PRINT_INFO(vif->ndev, HOSTINF_DBG,
			   "\n\n<< correcting Supplicant state machine >>\n\n");
		wilc_disconnect(vif);
	}
}

int wilc_get_statistics(struct wilc_vif *vif, struct rf_info *stats)
{
	struct wid wid_list[5];
	u32 wid_cnt = 0, result;

	wid_list[wid_cnt].id = WID_LINKSPEED;
	wid_list[wid_cnt].type = WID_CHAR;
	wid_list[wid_cnt].size = sizeof(char);
	wid_list[wid_cnt].val = (s8 *)&stats->link_speed;
	wid_cnt++;

	wid_list[wid_cnt].id = WID_RSSI;
	wid_list[wid_cnt].type = WID_CHAR;
	wid_list[wid_cnt].size = sizeof(char);
	wid_list[wid_cnt].val = (s8 *)&stats->rssi;
	wid_cnt++;

	wid_list[wid_cnt].id = WID_SUCCESS_FRAME_COUNT;
	wid_list[wid_cnt].type = WID_INT;
	wid_list[wid_cnt].size = sizeof(u32);
	wid_list[wid_cnt].val = (s8 *)&stats->tx_cnt;
	wid_cnt++;

	wid_list[wid_cnt].id = WID_RECEIVED_FRAGMENT_COUNT;
	wid_list[wid_cnt].type = WID_INT;
	wid_list[wid_cnt].size = sizeof(u32);
	wid_list[wid_cnt].val = (s8 *)&stats->rx_cnt;
	wid_cnt++;

	wid_list[wid_cnt].id = WID_FAILED_COUNT;
	wid_list[wid_cnt].type = WID_INT;
	wid_list[wid_cnt].size = sizeof(u32);
	wid_list[wid_cnt].val = (s8 *)&stats->tx_fail_cnt;
	wid_cnt++;

	result = wilc_send_config_pkt(vif, WILC_GET_CFG, wid_list,
				      wid_cnt,
				      wilc_get_vif_idx(vif));

	if (result) {
		PRINT_ER(vif->ndev, "Failed to send scan parameters\n");
		return result;
	}

	if (stats->link_speed > TCP_ACK_FILTER_LINK_SPEED_THRESH &&
	    stats->link_speed != DEFAULT_LINK_SPEED) {
		PRINT_INFO(vif->ndev, HOSTINF_DBG, "Enable TCP filter\n");
		wilc_enable_tcp_ack_filter(vif, true);
	} else if (stats->link_speed != DEFAULT_LINK_SPEED) {
		PRINT_INFO(vif->ndev, HOSTINF_DBG, "Disable TCP filter %d\n",
			   stats->link_speed);
		wilc_enable_tcp_ack_filter(vif, false);
	}

	return result;
}

static void handle_get_statistics(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);
	struct wilc_vif *vif = msg->vif;
	struct rf_info *stats = (struct rf_info *)msg->body.data;

	wilc_get_statistics(vif, stats);
	kfree(msg);
}

static void wilc_hif_pack_sta_param(struct wilc_vif *vif, u8 *cur_byte,
				    const u8 *mac,
				    struct station_parameters *params)
{
	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Packing STA params\n");
	ether_addr_copy(cur_byte, mac);
	cur_byte +=  ETH_ALEN;

	put_unaligned_le16(params->aid, cur_byte);
	cur_byte += 2;

	*cur_byte++ = params->supported_rates_len;
	if (params->supported_rates_len > 0)
		memcpy(cur_byte, params->supported_rates,
		       params->supported_rates_len);
	cur_byte += params->supported_rates_len;

	if (params->ht_capa) {
		*cur_byte++ = true;
		memcpy(cur_byte, &params->ht_capa,
		       sizeof(struct ieee80211_ht_cap));
	} else {
		*cur_byte++ = false;
	}
	cur_byte += sizeof(struct ieee80211_ht_cap);

	put_unaligned_le16(params->sta_flags_mask, cur_byte);
	cur_byte += 2;
	put_unaligned_le16(params->sta_flags_set, cur_byte);
}

static int handle_remain_on_chan(struct wilc_vif *vif,
				 struct remain_ch *hif_remain_ch)
{
	int result;
	u8 remain_on_chan_flag;
	struct wid wid;
	struct host_if_drv *hif_drv = vif->hif_drv;
	struct host_if_drv *hif_drv_p2p = get_drv_hndl_by_ifc(vif->wilc,
							      WILC_P2P_IFC);
	struct host_if_drv *hif_drv_wlan = get_drv_hndl_by_ifc(vif->wilc,
							       WILC_WLAN_IFC);

	if (!hif_drv) {
		PRINT_ER(vif->ndev, "Driver is null\n");
		return -EFAULT;
	}

	if (!hif_drv->remain_on_ch_pending) {
		hif_drv->remain_on_ch.arg = hif_remain_ch->arg;
		hif_drv->remain_on_ch.expired = hif_remain_ch->expired;
		hif_drv->remain_on_ch.ready = hif_remain_ch->ready;
		hif_drv->remain_on_ch.ch = hif_remain_ch->ch;
		hif_drv->remain_on_ch.id = hif_remain_ch->id;
	} else {
		hif_remain_ch->ch = hif_drv->remain_on_ch.ch;
	}

	if (hif_drv_p2p != NULL) {
		if (hif_drv_p2p->hif_state == HOST_IF_SCANNING) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "IFC busy scanning P2P_IFC state %d\n",
				   hif_drv_p2p->hif_state);
			hif_drv->remain_on_ch_pending = 1;
			result = -EBUSY;
			goto error;
		} else if ((hif_drv_p2p->hif_state != HOST_IF_IDLE) &&
		(hif_drv_p2p->hif_state != HOST_IF_CONNECTED)) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "IFC busy connecting. P2P_IFC state %d\n",
				   hif_drv_p2p->hif_state);
			result = -EBUSY;
			goto error;
		}
	}
	if (hif_drv_wlan != NULL) {
		if (hif_drv_wlan->hif_state == HOST_IF_SCANNING) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "IFC busy scanning. WLAN_IFC state %d\n",
				   hif_drv_wlan->hif_state);
			hif_drv->remain_on_ch_pending = 1;
			result = -EBUSY;
			goto error;
		} else if ((hif_drv_wlan->hif_state != HOST_IF_IDLE) &&
		(hif_drv_wlan->hif_state != HOST_IF_CONNECTED)) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "IFC busy connecting. WLAN_IFC %d\n",
				   hif_drv_wlan->hif_state);
			result = -EBUSY;
			goto error;
		}
	}

	if (vif->connecting) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Don't do scan in (CONNECTING) state\n");
		result = -EBUSY;
		goto error;
	}
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	if (vif->obtaining_ip) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Don't obss scan until IP adresss is obtained\n");
		result = -EBUSY;
		goto error;
	}
#endif

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Setting channel :%d\n",
		   hif_remain_ch->ch);
	remain_on_chan_flag = true;
	wid.id = WID_REMAIN_ON_CHAN;
	wid.type = WID_STR;
	wid.size = 2;
	wid.val = kmalloc(wid.size, GFP_KERNEL);
	if (!wid.val) {
		result = -ENOMEM;
		goto error;
	}

	wid.val[0] = remain_on_chan_flag;
	wid.val[1] = (s8)hif_remain_ch->ch;

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	kfree(wid.val);
	if (result != 0)
		PRINT_ER(vif->ndev, "Failed to set remain on channel\n");

	hif_drv->hif_state = HOST_IF_P2P_LISTEN;
error:

	hif_drv->remain_on_ch_timer_vif = vif;
#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
	hif_drv->remain_on_ch_timer.data = (unsigned long)hif_drv;
#endif
	mod_timer(&hif_drv->remain_on_ch_timer,
		  jiffies + msecs_to_jiffies(hif_remain_ch->duration));

	if (hif_drv->remain_on_ch.ready)
		hif_drv->remain_on_ch.ready(hif_drv->remain_on_ch.arg);

	if (hif_drv->remain_on_ch_pending)
		hif_drv->remain_on_ch_pending = 0;

	return result;
}

static void handle_listen_state_expired(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);
	struct wilc_vif *vif = msg->vif;
	struct remain_ch *hif_remain_ch = &msg->body.remain_on_ch;
	u8 remain_on_chan_flag;
	struct wid wid;
	int result;
	struct host_if_drv *hif_drv = vif->hif_drv;
	u8 null_bssid[6] = {0};

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "CANCEL REMAIN ON CHAN\n");

	if (hif_drv->hif_state == HOST_IF_P2P_LISTEN) {
		remain_on_chan_flag = false;
		wid.id = WID_REMAIN_ON_CHAN;
		wid.type = WID_STR;
		wid.size = 2;
		wid.val = kmalloc(wid.size, GFP_KERNEL);

		if (!wid.val) {
			PRINT_ER(vif->ndev, "Failed to allocate memory\n");
			goto free_msg;
		}

		wid.val[0] = remain_on_chan_flag;
		wid.val[1] = FALSE_FRMWR_CHANNEL;

		result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
					      wilc_get_vif_idx(vif));
		kfree(wid.val);
		if (result != 0) {
			PRINT_ER(vif->ndev, "Failed to set remain channel\n");
			goto free_msg;
		}

		if (hif_drv->remain_on_ch.expired)
			hif_drv->remain_on_ch.expired(hif_drv->remain_on_ch.arg,
						      hif_remain_ch->id);

		if (memcmp(hif_drv->assoc_bssid, null_bssid, ETH_ALEN) == 0)
			hif_drv->hif_state = HOST_IF_IDLE;
		else
			hif_drv->hif_state = HOST_IF_CONNECTED;
	} else {
		PRINT_D(vif->ndev, GENERIC_DBG,  "Not in listen state\n");
	}

free_msg:
	kfree(msg);
}

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
static void listen_timer_cb(struct timer_list *t)
#else
static void listen_timer_cb(unsigned long arg)
#endif
{
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	struct host_if_drv *hif_drv = from_timer(hif_drv, t,
						      remain_on_ch_timer);
#else
	struct host_if_drv *hif_drv = (struct host_if_drv *)arg;
#endif
	struct wilc_vif *vif = hif_drv->remain_on_ch_timer_vif;
	int result;
	struct host_if_msg *msg;

	del_timer(&vif->hif_drv->remain_on_ch_timer);

	msg = wilc_alloc_work(vif, handle_listen_state_expired, false);
	if (IS_ERR(msg))
		return;

	msg->body.remain_on_ch.id = vif->hif_drv->remain_on_ch.id;

	result = wilc_enqueue_work(msg);
	if (result) {
		PRINT_ER(vif->ndev, "wilc_mq_send fail\n");
		kfree(msg);
	}
}

static void handle_set_mcast_filter(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);
	struct wilc_vif *vif = msg->vif;
	struct set_multicast *hif_set_mc = &msg->body.multicast_info;
	int result;
	struct wid wid;
	u8 *cur_byte;

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Setup Multicast Filter\n");

	wid.id = WID_SETUP_MULTICAST_FILTER;
	wid.type = WID_BIN;
	wid.size = sizeof(struct set_multicast) + (hif_set_mc->cnt * ETH_ALEN);
	wid.val = kmalloc(wid.size, GFP_KERNEL);
	if (!wid.val)
		goto error;

	cur_byte = wid.val;
	*cur_byte++ = (hif_set_mc->enabled & 0xFF);
	*cur_byte++ = 0;
	*cur_byte++ = 0;
	*cur_byte++ = 0;

	*cur_byte++ = (hif_set_mc->cnt & 0xFF);
	*cur_byte++ = ((hif_set_mc->cnt >> 8) & 0xFF);
	*cur_byte++ = ((hif_set_mc->cnt >> 16) & 0xFF);
	*cur_byte++ = ((hif_set_mc->cnt >> 24) & 0xFF);

	if (hif_set_mc->cnt > 0 && hif_set_mc->mc_list)
		memcpy(cur_byte, hif_set_mc->mc_list,
		       ((hif_set_mc->cnt) * ETH_ALEN));

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to send setup multicast\n");

error:
	kfree(hif_set_mc->mc_list);
	kfree(wid.val);
	kfree(msg);
}

void wilc_set_wowlan_trigger(struct wilc_vif *vif, u8 wowlan_trigger)
{
	int ret;
	struct wid wid;

	wid.id = WID_WOWLAN_TRIGGER;
	wid.type = WID_CHAR;
	wid.val = &wowlan_trigger;
	wid.size = sizeof(s8);

	ret = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				   wilc_get_vif_idx(vif));

	if (ret)
		PRINT_ER(vif->ndev,
			 "Failed to send wowlan trigger config packet\n");
}

static void handle_scan_timer(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);
	int ret;

	PRINT_INFO(msg->vif->ndev, HOSTINF_DBG, "handling scan timer\n");
	ret = handle_scan_done(msg->vif, SCAN_EVENT_ABORTED);
	if (ret)
		PRINT_ER(msg->vif->ndev, "Failed to handle scan done\n");
	kfree(msg);
}

static void handle_scan_complete(struct work_struct *work)
{
	struct host_if_msg *msg = container_of(work, struct host_if_msg, work);

	del_timer(&msg->vif->hif_drv->scan_timer);
	PRINT_INFO(msg->vif->ndev, HOSTINF_DBG, "scan completed\n");

	handle_scan_done(msg->vif, SCAN_EVENT_DONE);

	if (msg->vif->hif_drv->remain_on_ch_pending)
		handle_remain_on_chan(msg->vif,
				      &msg->vif->hif_drv->remain_on_ch);
	kfree(msg);
}

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
static void timer_scan_cb(struct timer_list *t)
#else
static void timer_scan_cb(unsigned long arg)
#endif
{
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	struct host_if_drv *hif_drv = from_timer(hif_drv, t, scan_timer);
#else
	struct host_if_drv *hif_drv = (struct host_if_drv *)arg;
#endif
	struct wilc_vif *vif = hif_drv->scan_timer_vif;
	struct host_if_msg *msg;
	int result;

	msg = wilc_alloc_work(vif, handle_scan_timer, false);
	if (IS_ERR(msg))
		return;

	result = wilc_enqueue_work(msg);
	if (result)
		kfree(msg);
}

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
static void timer_connect_cb(struct timer_list *t)
#else
static void timer_connect_cb(unsigned long arg)
#endif
{
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	struct host_if_drv *hif_drv = from_timer(hif_drv, t, connect_timer);
#else
	struct host_if_drv *hif_drv = (struct host_if_drv *)arg;
#endif
	struct wilc_vif *vif = hif_drv->connect_timer_vif;
	struct host_if_msg *msg;
	int result;

	msg = wilc_alloc_work(vif, handle_connect_timeout, false);
	if (IS_ERR(msg))
		return;

	result = wilc_enqueue_work(msg);
	if (result)
		kfree(msg);
}

signed int wilc_send_buffered_eap(struct wilc_vif *vif,
				  wilc_frmw_to_linux_t frmw_to_linux,
				  free_eap_buf_param eap_buf_param,
				  u8 *buff, unsigned int size,
				  unsigned int pkt_offset,
				  void *user_arg)
{
	int result;
	struct host_if_msg *msg;

	if (!vif || !frmw_to_linux || !eap_buf_param)
		return -EFAULT;

	msg = wilc_alloc_work(vif, handle_send_buffered_eap, false);
	if (IS_ERR(msg))
		return PTR_ERR(msg);
	msg->body.send_buff_eap.frmw_to_linux = frmw_to_linux;
	msg->body.send_buff_eap.eap_buf_param = eap_buf_param;
	msg->body.send_buff_eap.size = size;
	msg->body.send_buff_eap.pkt_offset = pkt_offset;
	msg->body.send_buff_eap.buff = kmalloc(size + pkt_offset,
						  GFP_ATOMIC);
	memcpy(msg->body.send_buff_eap.buff, buff, size + pkt_offset);
	msg->body.send_buff_eap.user_arg = user_arg;

	result = wilc_enqueue_work(msg);
	if (result) {
		PRINT_ER(vif->ndev, "enqueue work failed\n");
		kfree(msg->body.send_buff_eap.buff);
		kfree(msg);
	}
	return result;
}

int wilc_remove_wep_key(struct wilc_vif *vif, u8 index)
{
	struct wid wid;
	int result;

	wid.id = WID_REMOVE_WEP_KEY;
	wid.type = WID_STR;
	wid.size = sizeof(char);
	wid.val = &index;

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev,
			 "Failed to send remove wep key config packet\n");
	return result;
}

int wilc_set_wep_default_keyid(struct wilc_vif *vif, u8 index)
{
	struct wid wid;
	int result;

	wid.id = WID_KEY_ID;
	wid.type = WID_CHAR;
	wid.size = sizeof(char);
	wid.val = &index;
	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev,
			 "Failed to send wep default key config packet\n");

	return result;
}

int wilc_add_wep_key_bss_sta(struct wilc_vif *vif, const u8 *key, u8 len,
			     u8 index)
{
	struct wid wid;
	int result;
	struct wilc_wep_key *wep_key;

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Handling WEP key\n");
	wid.id = WID_ADD_WEP_KEY;
	wid.type = WID_STR;
	wid.size = sizeof(*wep_key) + len;
	wep_key = kzalloc(wid.size, GFP_KERNEL);
	if (!wep_key) {
		PRINT_ER(vif->ndev, "No buffer to send Key\n");
		return -ENOMEM;
	}
	wid.val = (u8 *)wep_key;

	wep_key->index = index;
	wep_key->key_len = len;
	memcpy(wep_key->key, key, len);

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		netdev_err(vif->ndev,
			   "Failed to add wep key config packet\n");

	kfree(wep_key);
	return result;
}

int wilc_add_wep_key_bss_ap(struct wilc_vif *vif, const u8 *key, u8 len,
			    u8 index, u8 mode, enum authtype auth_type)
{
	struct wid wid_list[3];
	int result;
	struct wilc_wep_key *wep_key;

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "Handling WEP key index: %d\n",
		   index);
	wid_list[0].id = WID_11I_MODE;
	wid_list[0].type = WID_CHAR;
	wid_list[0].size = sizeof(char);
	wid_list[0].val = &mode;

	wid_list[1].id = WID_AUTH_TYPE;
	wid_list[1].type = WID_CHAR;
	wid_list[1].size = sizeof(char);
	wid_list[1].val = (s8 *)&auth_type;

	wid_list[2].id = WID_WEP_KEY_VALUE;
	wid_list[2].type = WID_STR;
	wid_list[2].size = sizeof(*wep_key) + len;
	wep_key = kzalloc(wid_list[2].size, GFP_KERNEL);
	if (!wep_key) {
		PRINT_ER(vif->ndev, "No buffer to send Key\n");
		return -ENOMEM;
	}

	wid_list[2].val = (u8 *)wep_key;

	wep_key->index = index;
	wep_key->key_len = len;
	memcpy(wep_key->key, key, len);
	result = wilc_send_config_pkt(vif, WILC_SET_CFG, wid_list,
				      ARRAY_SIZE(wid_list),
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev,
			 "Failed to add wep ap key config packet\n");

	kfree(wep_key);
	return result;
}

int wilc_add_ptk(struct wilc_vif *vif, const u8 *ptk, u8 ptk_key_len,
		 const u8 *mac_addr, const u8 *rx_mic, const u8 *tx_mic,
		 u8 mode, u8 cipher_mode, u8 index)
{
	int result = 0;
	u8 t_key_len = ptk_key_len + RX_MIC_KEY_LEN + TX_MIC_KEY_LEN;

	if (mode == WILC_AP_MODE) {
		struct wid wid_list[2];
		struct wilc_ap_wpa_ptk *key_buf;

		wid_list[0].id = WID_11I_MODE;
		wid_list[0].type = WID_CHAR;
		wid_list[0].size = sizeof(char);
		wid_list[0].val = (s8 *)&cipher_mode;

		key_buf = kzalloc(sizeof(*key_buf) + t_key_len, GFP_KERNEL);
		if (!key_buf) {
			PRINT_ER(vif->ndev,
				 "NO buffer to keep Key buffer - AP\n");
			return -ENOMEM;
		}
		ether_addr_copy(key_buf->mac_addr, mac_addr);
		key_buf->index = index;
		key_buf->key_len = t_key_len;
		memcpy(&key_buf->key[0], ptk, ptk_key_len);

		if (rx_mic)
			memcpy(&key_buf->key[ptk_key_len], rx_mic,
			       RX_MIC_KEY_LEN);

		if (tx_mic)
			memcpy(&key_buf->key[ptk_key_len + RX_MIC_KEY_LEN],
			       tx_mic, TX_MIC_KEY_LEN);

		wid_list[1].id = WID_ADD_PTK;
		wid_list[1].type = WID_STR;
		wid_list[1].size = sizeof(*key_buf) + t_key_len;
		wid_list[1].val = (u8 *)key_buf;
		result = wilc_send_config_pkt(vif, WILC_SET_CFG, wid_list,
					      ARRAY_SIZE(wid_list),
					      wilc_get_vif_idx(vif));
		kfree(key_buf);
	} else if (mode == WILC_STATION_MODE) {
		struct wid wid;
		struct wilc_sta_wpa_ptk *key_buf;

		key_buf = kzalloc(sizeof(*key_buf) + t_key_len, GFP_KERNEL);
		if (!key_buf) {
			PRINT_ER(vif->ndev,
				 "No buffer to keep Key buffer - Station\n");
			return -ENOMEM;
		}

		ether_addr_copy(key_buf->mac_addr, mac_addr);
		key_buf->key_len = t_key_len;
		memcpy(&key_buf->key[0], ptk, ptk_key_len);

		if (rx_mic)
			memcpy(&key_buf->key[ptk_key_len], rx_mic,
			       RX_MIC_KEY_LEN);

		if (tx_mic)
			memcpy(&key_buf->key[ptk_key_len + RX_MIC_KEY_LEN],
			       tx_mic, TX_MIC_KEY_LEN);

		wid.id = WID_ADD_PTK;
		wid.type = WID_STR;
		wid.size = sizeof(*key_buf) + t_key_len;
		wid.val = (s8 *)key_buf;
		result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
					      wilc_get_vif_idx(vif));
		kfree(key_buf);
	}

	return result;
}

int wilc_add_rx_gtk(struct wilc_vif *vif, const u8 *rx_gtk, u8 gtk_key_len,
		    u8 index, u32 key_rsc_len, const u8 *key_rsc,
		    const u8 *rx_mic, const u8 *tx_mic, u8 mode,
		    u8 cipher_mode)
{
	int result = 0;
	struct wilc_gtk_key *gtk_key;
	int t_key_len = gtk_key_len + RX_MIC_KEY_LEN + TX_MIC_KEY_LEN;

	gtk_key = kzalloc(sizeof(*gtk_key) + t_key_len, GFP_KERNEL);
	if (!gtk_key) {
		PRINT_ER(vif->ndev, "No buffer to send GTK Key\n");
		return -ENOMEM;
	}

	/* fill bssid value only in station mode */
	if (mode == WILC_STATION_MODE &&
	    vif->hif_drv->hif_state == HOST_IF_CONNECTED)
		memcpy(gtk_key->mac_addr, vif->hif_drv->assoc_bssid, ETH_ALEN);

	if (key_rsc)
		memcpy(gtk_key->rsc, key_rsc, 8);
	gtk_key->index = index;
	gtk_key->key_len = t_key_len;
	memcpy(&gtk_key->key[0], rx_gtk, gtk_key_len);

	if (rx_mic)
		memcpy(&gtk_key->key[gtk_key_len], rx_mic, RX_MIC_KEY_LEN);

	if (tx_mic)
		memcpy(&gtk_key->key[gtk_key_len + RX_MIC_KEY_LEN],
		       tx_mic, TX_MIC_KEY_LEN);

	if (mode == WILC_AP_MODE) {
		struct wid wid_list[2];

		wid_list[0].id = WID_11I_MODE;
		wid_list[0].type = WID_CHAR;
		wid_list[0].size = sizeof(char);
		wid_list[0].val = (s8 *)&cipher_mode;

		wid_list[1].id = WID_ADD_RX_GTK;
		wid_list[1].type = WID_STR;
		wid_list[1].size = sizeof(*gtk_key) + t_key_len;
		wid_list[1].val = (u8 *)gtk_key;

		result = wilc_send_config_pkt(vif, WILC_SET_CFG, wid_list,
					      ARRAY_SIZE(wid_list),
					      wilc_get_vif_idx(vif));
		kfree(gtk_key);
	} else if (mode == WILC_STATION_MODE) {
		struct wid wid;

		wid.id = WID_ADD_RX_GTK;
		wid.type = WID_STR;
		wid.size = sizeof(*gtk_key) + t_key_len;
		wid.val = (u8 *)gtk_key;
		result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
					      wilc_get_vif_idx(vif));
		kfree(gtk_key);
	}

	return result;
}

int wilc_set_pmkid_info(struct wilc_vif *vif, struct wilc_pmkid_attr *pmkid)
{
	struct wid wid;
	int result;

	wid.id = WID_PMKID_INFO;
	wid.type = WID_STR;
	wid.size = (pmkid->numpmkid * sizeof(struct wilc_pmkid)) + 1;
	wid.val = (u8 *)pmkid;

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));

	return result;
}

int wilc_get_mac_address(struct wilc_vif *vif, u8 *mac_addr)
{
	int result;
	struct wid wid;

	wid.id = WID_MAC_ADDR;
	wid.type = WID_STR;
	wid.size = ETH_ALEN;
	wid.val = mac_addr;

	result = wilc_send_config_pkt(vif, WILC_GET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		netdev_err(vif->ndev, "Failed to get mac address\n");

	return result;
}

int wilc_set_mac_address(struct wilc_vif *vif, u8 *mac_addr)
{
	struct wid wid;
	int result;

	wid.id = WID_MAC_ADDR;
	wid.type = WID_STR;
	wid.size = ETH_ALEN;
	wid.val = mac_addr;

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to set mac address\n");

	return result;
}

int wilc_set_join_req(struct wilc_vif *vif, u8 *bssid, const u8 *ssid,
		      size_t ssid_len, const u8 *ies, size_t ies_len,
		      wilc_connect_result connect_result, void *user_arg,
		      u8 security, enum authtype auth_type,
		      u8 channel, void *join_params)
{
	int result;
	struct host_if_drv *hif_drv = vif->hif_drv;
	struct user_conn_req *con_info = &hif_drv->usr_conn_req;

	if (!hif_drv || !connect_result) {
		PRINT_ER(vif->ndev, "hif driver or connect result is NULL\n");
		return -EFAULT;
	}

	if (!join_params) {
		PRINT_ER(vif->ndev, "joinparams is NULL\n");
		return -EFAULT;
	}

	if (hif_drv->usr_scan_req.scan_result) {
		PRINT_ER(vif->ndev, "%s: Scan in progress\n", __func__);
		return -EBUSY;
	}

	con_info->security = security;
	con_info->auth_type = auth_type;
	con_info->ch = channel;
	con_info->conn_result = connect_result;
	con_info->arg = user_arg;
	con_info->param = join_params;

	if (bssid) {
		con_info->bssid = kmemdup(bssid, 6, GFP_KERNEL);
		if (!con_info->bssid)
			return -ENOMEM;
	}

	if (ssid) {
		con_info->ssid_len = ssid_len;
		con_info->ssid = kmemdup(ssid, ssid_len, GFP_KERNEL);
		if (!con_info->ssid) {
			result = -ENOMEM;
			goto free_bssid;
		}
	}

	if (ies) {
		con_info->ies_len = ies_len;
		con_info->ies = kmemdup(ies, ies_len, GFP_KERNEL);
		if (!con_info->ies) {
			result = -ENOMEM;
			goto free_ssid;
		}
	}

	result = wilc_send_connect_wid(vif);
	if (result) {
		PRINT_ER(vif->ndev, "Failed to send connect wid\n");
		goto free_ies;
	}

#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
	hif_drv->connect_timer.data = (unsigned long)hif_drv;
#endif
	hif_drv->connect_timer_vif = vif;
	mod_timer(&hif_drv->connect_timer,
		  jiffies + msecs_to_jiffies(HOST_IF_CONNECT_TIMEOUT));

	return 0;

free_ies:
	kfree(con_info->ies);

free_ssid:
	kfree(con_info->ssid);

free_bssid:
	kfree(con_info->bssid);

	return result;
}

int wilc_set_mac_chnl_num(struct wilc_vif *vif, u8 channel)
{
	struct wid wid;
	int result;

	wid.id = WID_CURRENT_CHANNEL;
	wid.type = WID_CHAR;
	wid.size = sizeof(char);
	wid.val = &channel;

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to set channel\n");

	return result;
}

int wilc_set_wfi_drv_handler(struct wilc_vif *vif, int index, u8 mode,
			     u8 ifc_id)
{
	struct wid wid;
	struct host_if_drv *hif_drv = vif->hif_drv;
	int result;
	struct wilc_drv_handler drv;

	wid.id = WID_SET_DRV_HANDLER;
	wid.type = WID_STR;
	wid.size = sizeof(drv);
	wid.val = (u8 *)&drv;

	drv.handler = cpu_to_le32(index);
	drv.mode = (ifc_id | (mode << 1));

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      hif_drv->driver_handler_id);
	if (result)
		PRINT_ER(vif->ndev, "Failed to set driver handler\n");

	return result;
}

int wilc_set_operation_mode(struct wilc_vif *vif, u32 mode)
{
	struct wid wid;
	struct wilc_op_mode op_mode;
	int result;

	wid.id = WID_SET_OPERATION_MODE;
	wid.type = WID_INT;
	wid.size = sizeof(op_mode);
	wid.val = (u8 *)&op_mode;

	op_mode.mode = cpu_to_le32(mode);

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to set operation mode\n");

	return result;
}

s32 wilc_get_inactive_time(struct wilc_vif *vif, const u8 *mac, u32 *out_val)
{
	struct wid wid;
	s32 result;

	wid.id = WID_SET_STA_MAC_INACTIVE_TIME;
	wid.type = WID_STR;
	wid.size = ETH_ALEN;
	wid.val = kzalloc(wid.size, GFP_KERNEL);
	if (!wid.val) {
		PRINT_ER(vif->ndev, "Failed to allocate buffer\n");
		return -ENOMEM;
	}

	ether_addr_copy(wid.val, mac);
	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	kfree(wid.val);
	if (result) {
		PRINT_ER(vif->ndev, "Failed to set inactive mac\n");
		return result;
	}

	wid.id = WID_GET_INACTIVE_TIME;
	wid.type = WID_INT;
	wid.val = (s8 *)out_val;
	wid.size = sizeof(u32);
	result = wilc_send_config_pkt(vif, WILC_GET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to get inactive time\n");

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Getting inactive time : %d\n",
		   *out_val);

	return result;
}

int wilc_get_rssi(struct wilc_vif *vif, s8 *rssi_level)
{
	struct wid wid;
	int result;

	if (!rssi_level) {
		PRINT_ER(vif->ndev, "RSS pointer value is null\n");
		return -EFAULT;
	}

	wid.id = WID_RSSI;
	wid.type = WID_CHAR;
	wid.size = sizeof(char);
	wid.val = rssi_level;
	result = wilc_send_config_pkt(vif, WILC_GET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		netdev_err(vif->ndev, "Failed to get RSSI value\n");

	return result;
}

int wilc_get_stats_async(struct wilc_vif *vif, struct rf_info *stats)
{
	int result;
	struct host_if_msg *msg;

	PRINT_INFO(vif->ndev, HOSTINF_DBG, " getting async statistics\n");
	msg = wilc_alloc_work(vif, handle_get_statistics, false);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	msg->body.data = (char *)stats;

	result = wilc_enqueue_work(msg);
	if (result) {
		PRINT_ER(vif->ndev, "enqueue work failed\n");
		kfree(msg);
		return result;
	}

	return result;
}

int wilc_hif_set_cfg(struct wilc_vif *vif, struct cfg_param_attr *param)
{
	struct wid wid_list[4];
	int i = 0;
	int result;

	if (param->flag & WILC_CFG_PARAM_RETRY_SHORT) {
		wid_list[i].id = WID_SHORT_RETRY_LIMIT;
		wid_list[i].val = (s8 *)&param->short_retry_limit;
		wid_list[i].type = WID_SHORT;
		wid_list[i].size = sizeof(u16);
		i++;
	}
	if (param->flag & WILC_CFG_PARAM_RETRY_LONG) {
		wid_list[i].id = WID_LONG_RETRY_LIMIT;
		wid_list[i].val = (s8 *)&param->long_retry_limit;
		wid_list[i].type = WID_SHORT;
		wid_list[i].size = sizeof(u16);
		i++;
	}
	if (param->flag & WILC_CFG_PARAM_FRAG_THRESHOLD) {
		wid_list[i].id = WID_FRAG_THRESHOLD;
		wid_list[i].val = (s8 *)&param->frag_threshold;
		wid_list[i].type = WID_SHORT;
		wid_list[i].size = sizeof(u16);
		i++;
	}
	if (param->flag & WILC_CFG_PARAM_RTS_THRESHOLD) {
		wid_list[i].id = WID_RTS_THRESHOLD;
		wid_list[i].val = (s8 *)&param->rts_threshold;
		wid_list[i].type = WID_SHORT;
		wid_list[i].size = sizeof(u16);
		i++;
	}

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, wid_list,
				      i, wilc_get_vif_idx(vif));

	return result;
}

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
static void get_periodic_rssi(struct timer_list *t)
#else
static void get_periodic_rssi(unsigned long arg)
#endif
{
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	struct wilc_vif *vif = from_timer(vif, t, periodic_rssi);
#else
	struct wilc_vif *vif = (struct wilc_vif *)arg;
#endif

	if (!vif->hif_drv) {
		PRINT_ER(vif->ndev, "hif driver is NULL\n");
		return;
	}

	if (vif->hif_drv->hif_state == HOST_IF_CONNECTED)
		wilc_get_stats_async(vif, &vif->periodic_stats);

	mod_timer(&vif->periodic_rssi, jiffies + msecs_to_jiffies(5000));
}

int wilc_init(struct net_device *dev, struct host_if_drv **hif_drv_handler)
{
	struct host_if_drv *hif_drv;
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc = vif->wilc;
	int i;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Initializing host interface for client %d\n",
		   wilc->clients_count + 1);

	hif_drv  = kzalloc(sizeof(*hif_drv), GFP_KERNEL);
	if (!hif_drv) {
		PRINT_ER(dev, "hif driver is NULL\n");
		return -ENOMEM;
	}
	*hif_drv_handler = hif_drv;
	for (i = 0; i <= wilc->vif_num; i++)
		if (dev == wilc->vif[i]->ndev) {
			wilc->vif[i]->hif_drv = hif_drv;
			hif_drv->driver_handler_id = i + 1;
			break;
		}

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	vif->obtaining_ip = false;
#endif

	if (wilc->clients_count == 0)
		mutex_init(&hif_deinit_lock);

	#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
		timer_setup(&vif->periodic_rssi, get_periodic_rssi, 0);
	#else
		setup_timer(&vif->periodic_rssi, get_periodic_rssi,
			    (unsigned long)vif);
	#endif
		mod_timer(&vif->periodic_rssi,
			  jiffies + msecs_to_jiffies(5000));

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	timer_setup(&hif_drv->scan_timer, timer_scan_cb, 0);
	timer_setup(&hif_drv->connect_timer, timer_connect_cb, 0);
	timer_setup(&hif_drv->remain_on_ch_timer, listen_timer_cb, 0);
#else
	setup_timer(&hif_drv->scan_timer, timer_scan_cb, 0);
	setup_timer(&hif_drv->connect_timer, timer_connect_cb, 0);
	setup_timer(&hif_drv->remain_on_ch_timer, listen_timer_cb, 0);
#endif

	hif_drv->hif_state = HOST_IF_IDLE;

	hif_drv->p2p_timeout = 0;

	wilc->clients_count++;

	return 0;
}

int wilc_deinit(struct wilc_vif *vif)
{
	int result = 0;
	struct host_if_drv *hif_drv = vif->hif_drv;

	if (!hif_drv) {
		PRINT_ER(vif->ndev, "hif driver is NULL\n");
		return -EFAULT;
	}

	mutex_lock(&hif_deinit_lock);

	terminated_handle = hif_drv;
	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "De-initializing host interface for client %d\n",
		   vif->wilc->clients_count);

	del_timer_sync(&hif_drv->scan_timer);
	del_timer_sync(&hif_drv->connect_timer);
	del_timer_sync(&vif->periodic_rssi);
	del_timer_sync(&hif_drv->remain_on_ch_timer);

	wilc_set_wfi_drv_handler(vif, 0, 0, 0);

	if (hif_drv->usr_scan_req.scan_result) {
		hif_drv->usr_scan_req.scan_result(SCAN_EVENT_ABORTED, NULL,
						  hif_drv->usr_scan_req.arg,
						  NULL);
		hif_drv->usr_scan_req.scan_result = NULL;
	}

	hif_drv->hif_state = HOST_IF_IDLE;

	kfree(hif_drv);

	vif->wilc->clients_count--;
	terminated_handle = NULL;
	mutex_unlock(&hif_deinit_lock);
	return result;
}

void wilc_network_info_received(struct wilc *wilc, u8 *buffer, u32 length)
{
	int result;
	struct host_if_msg *msg;
	int id;
	struct host_if_drv *hif_drv;
	struct wilc_vif *vif;

	id = buffer[length - 4];
	id |= (buffer[length - 3] << 8);
	id |= (buffer[length - 2] << 16);
	id |= (buffer[length - 1] << 24);
	vif = wilc_get_vif_from_idx(wilc, id);
	if (!vif)
		return;
	hif_drv = vif->hif_drv;

	if (!hif_drv || hif_drv == terminated_handle) {
		PRINT_ER(vif->ndev, "driver not init[%p]\n", hif_drv);
		return;
	}

	msg = wilc_alloc_work(vif, handle_rcvd_ntwrk_info, false);
	if (IS_ERR(msg))
		return;

	msg->body.net_info.len = length;
	msg->body.net_info.buffer = kmemdup(buffer, length, GFP_KERNEL);
	if (!msg->body.net_info.buffer) {
		kfree(msg);
		return;
	}

	result = wilc_enqueue_work(msg);
	if (result) {
		PRINT_ER(vif->ndev, "message parameters (%d)\n", result);
		kfree(msg->body.net_info.buffer);
		kfree(msg);
	}
}

void wilc_gnrl_async_info_received(struct wilc *wilc, u8 *buffer, u32 length)
{
	int result;
	struct host_if_msg *msg;
	int id;
	struct host_if_drv *hif_drv;
	struct wilc_vif *vif;

	mutex_lock(&hif_deinit_lock);

	id = buffer[length - 4];
	id |= (buffer[length - 3] << 8);
	id |= (buffer[length - 2] << 16);
	id |= (buffer[length - 1] << 24);
	vif = wilc_get_vif_from_idx(wilc, id);
	if (!vif) {
		mutex_unlock(&hif_deinit_lock);
		return;
	}
	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "General asynchronous info packet received\n");

	hif_drv = vif->hif_drv;

	if (!hif_drv || hif_drv == terminated_handle) {
		PRINT_ER(vif->ndev, "hif driver is NULL\n");
		mutex_unlock(&hif_deinit_lock);
		return;
	}

	if (!hif_drv->usr_conn_req.conn_result) {
		PRINT_ER(vif->ndev, "there is no current Connect Request\n");
		mutex_unlock(&hif_deinit_lock);
		return;
	}

	msg = wilc_alloc_work(vif, handle_rcvd_gnrl_async_info, false);
	if (IS_ERR(msg)) {
		mutex_unlock(&hif_deinit_lock);
		return;
	}

	msg->body.async_info.len = length;
	msg->body.async_info.buffer = kmemdup(buffer, length, GFP_KERNEL);
	if (!msg->body.async_info.buffer) {
		kfree(msg);
		mutex_unlock(&hif_deinit_lock);
		return;
	}

	result = wilc_enqueue_work(msg);
	if (result) {
		PRINT_ER(vif->ndev, "enqueue work failed\n");
		kfree(msg->body.async_info.buffer);
		kfree(msg);
	}

	mutex_unlock(&hif_deinit_lock);
}

void wilc_scan_complete_received(struct wilc *wilc, u8 *buffer, u32 length)
{
	int result;
	int id;
	struct host_if_drv *hif_drv;
	struct wilc_vif *vif;

	id = buffer[length - 4];
	id |= buffer[length - 3] << 8;
	id |= buffer[length - 2] << 16;
	id |= buffer[length - 1] << 24;
	vif = wilc_get_vif_from_idx(wilc, id);
	if (!vif)
		return;
	hif_drv = vif->hif_drv;
	PRINT_INFO(vif->ndev, GENERIC_DBG, "Scan notification received\n");

	if (!hif_drv || hif_drv == terminated_handle) {
		PRINT_ER(vif->ndev, "hif driver is NULL\n");
		return;
	}

	if (hif_drv->usr_scan_req.scan_result) {
		struct host_if_msg *msg;

		msg = wilc_alloc_work(vif, handle_scan_complete, false);
		if (IS_ERR(msg))
			return;

		result = wilc_enqueue_work(msg);
		if (result) {
			PRINT_ER(vif->ndev, "enqueue work failed\n");
			kfree(msg);
		}
	}
}

int wilc_remain_on_channel(struct wilc_vif *vif, u32 session_id,
			   u32 duration, u16 chan,
			   wilc_remain_on_chan_expired expired,
			   wilc_remain_on_chan_ready ready,
			   void *user_arg)
{
	struct remain_ch roc;
	int result;

	PRINT_INFO(vif->ndev, CFG80211_DBG, "%s called\n", __func__);
	roc.ch = chan;
	roc.expired = expired;
	roc.ready = ready;
	roc.arg = user_arg;
	roc.duration = duration;
	roc.id = session_id;
	result = handle_remain_on_chan(vif, &roc);
	if (result)
		PRINT_ER(vif->ndev, "%s: failed to set remain on channel\n",
			 __func__);

	return result;
}

int wilc_listen_state_expired(struct wilc_vif *vif, u32 session_id)
{
	int result;
	struct host_if_msg *msg;
	struct host_if_drv *hif_drv = vif->hif_drv;

	if (!hif_drv) {
		PRINT_ER(vif->ndev, "hif driver is NULL\n");
		return -EFAULT;
	}

	del_timer(&hif_drv->remain_on_ch_timer);

	msg = wilc_alloc_work(vif, handle_listen_state_expired, false);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	msg->body.remain_on_ch.id = session_id;

	result = wilc_enqueue_work(msg);
	if (result) {
		PRINT_ER(vif->ndev, "enqueue work failed\n");
		kfree(msg);
	}

	return result;
}

void wilc_frame_register(struct wilc_vif *vif, u16 frame_type, bool reg)
{
	struct wid wid;
	int result;
	struct wilc_reg_frame reg_frame;

	wid.id = WID_REGISTER_FRAME;
	wid.type = WID_STR;
	wid.size = sizeof(reg_frame);
	wid.val = (u8 *)&reg_frame;

	memset(&reg_frame, 0x0, sizeof(reg_frame));
	reg_frame.reg = reg;

	switch (frame_type) {
	case IEEE80211_STYPE_ACTION:
		PRINT_INFO(vif->ndev, HOSTINF_DBG, "ACTION\n");
		reg_frame.reg_id = WILC_FW_ACTION_FRM_IDX;
		break;

	case IEEE80211_STYPE_PROBE_REQ:
		PRINT_INFO(vif->ndev, HOSTINF_DBG, "PROBE REQ\n");
		reg_frame.reg_id = WILC_FW_PROBE_REQ_IDX;
		break;

	default:
		PRINT_INFO(vif->ndev, HOSTINF_DBG, "Not valid frame type\n");
		break;
	}
	reg_frame.frame_type = cpu_to_le16(frame_type);
	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to frame register\n");
}

int wilc_add_beacon(struct wilc_vif *vif, u32 interval, u32 dtim_period,
		    struct cfg80211_beacon_data *params)
{
	struct wid wid;
	int result;
	u8 *cur_byte;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Setting adding beacon\n");

	wid.id = WID_ADD_BEACON;
	wid.type = WID_BIN;
	wid.size = params->head_len + params->tail_len + 16;
	wid.val = kzalloc(wid.size, GFP_KERNEL);
	if (!wid.val) {
		PRINT_ER(vif->ndev, "Failed to allocate buffer\n");
		return -ENOMEM;
	}

	cur_byte = wid.val;
	put_unaligned_le32(interval, cur_byte);
	cur_byte += 4;
	put_unaligned_le32(dtim_period, cur_byte);
	cur_byte += 4;
	put_unaligned_le32(params->head_len, cur_byte);
	cur_byte += 4;

	if (params->head_len > 0)
		memcpy(cur_byte, params->head, params->head_len);
	cur_byte += params->head_len;

	put_unaligned_le32(params->tail_len, cur_byte);
	cur_byte += 4;

	if (params->tail_len > 0)
		memcpy(cur_byte, params->tail, params->tail_len);

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to send add beacon\n");

	kfree(wid.val);

	return result;
}

int wilc_del_beacon(struct wilc_vif *vif)
{
	int result;
	struct wid wid;
	u8 del_beacon = 0;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Setting deleting beacon message queue params\n");

	wid.id = WID_DEL_BEACON;
	wid.type = WID_CHAR;
	wid.size = sizeof(char);
	wid.val = &del_beacon;
	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to send delete beacon\n");

	return result;
}

int wilc_add_station(struct wilc_vif *vif, const u8 *mac,
		     struct station_parameters *params)
{
	struct wid wid;
	int result;
	u8 *cur_byte;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Setting adding station message queue params\n");

	wid.id = WID_ADD_STA;
	wid.type = WID_BIN;
	wid.size = WILC_ADD_STA_LENGTH + params->supported_rates_len;
	wid.val = kmalloc(wid.size, GFP_KERNEL);
	if (!wid.val)
		return -ENOMEM;

	cur_byte = wid.val;
	wilc_hif_pack_sta_param(vif, cur_byte, mac, params);

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result != 0)
		PRINT_ER(vif->ndev, "Failed to send add station\n");

	kfree(wid.val);

	return result;
}

int wilc_del_station(struct wilc_vif *vif, const u8 *mac_addr)
{
	struct wid wid;
	int result;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Setting deleting station message queue params\n");

	wid.id = WID_REMOVE_STA;
	wid.type = WID_BIN;
	wid.size = ETH_ALEN;
	wid.val = kzalloc(wid.size, GFP_KERNEL);
	if (!wid.val) {
		PRINT_ER(vif->ndev, "Failed to allocate buffer\n");
		return -ENOMEM;
	}

	if (!mac_addr)
		eth_broadcast_addr(wid.val);
	else
		ether_addr_copy(wid.val, mac_addr);

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to del station\n");

	kfree(wid.val);

	return result;
}

int wilc_del_allstation(struct wilc_vif *vif, u8 mac_addr[][ETH_ALEN])
{
	struct wid wid;
	int result;
	int i;
	u8 assoc_sta = 0;
	struct del_all_sta del_sta;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Setting deauthenticating station message queue params\n");
	memset(&del_sta, 0x0, sizeof(del_sta));
	for (i = 0; i < WILC_MAX_NUM_STA; i++) {
		if (!is_zero_ether_addr(mac_addr[i])) {
			PRINT_INFO(vif->ndev,
				   CFG80211_DBG, "BSSID = %x%x%x%x%x%x\n",
				   mac_addr[i][0], mac_addr[i][1],
				   mac_addr[i][2], mac_addr[i][3],
				   mac_addr[i][4], mac_addr[i][5]);
			assoc_sta++;
			ether_addr_copy(del_sta.mac[i], mac_addr[i]);
		}
	}
	if (!assoc_sta) {
		PRINT_INFO(vif->ndev, CFG80211_DBG, "NO ASSOCIATED STAS\n");
		return 0;
	}
	del_sta.assoc_sta = assoc_sta;

	wid.id = WID_DEL_ALL_STA;
	wid.type = WID_STR;
	wid.size = (assoc_sta * ETH_ALEN) + 1;
	wid.val = (u8 *)&del_sta;

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to send delete all station\n");

	return result;
}

int wilc_edit_station(struct wilc_vif *vif, const u8 *mac,
		      struct station_parameters *params)
{
	struct wid wid;
	int result;
	u8 *cur_byte;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Setting editing station message queue params\n");

	wid.id = WID_EDIT_STA;
	wid.type = WID_BIN;
	wid.size = WILC_ADD_STA_LENGTH + params->supported_rates_len;
	wid.val = kmalloc(wid.size, GFP_KERNEL);
	if (!wid.val)
		return -ENOMEM;

	cur_byte = wid.val;
	wilc_hif_pack_sta_param(vif, cur_byte, mac, params);

	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to send edit station\n");

	kfree(wid.val);
	return result;
}

int wilc_set_power_mgmt(struct wilc_vif *vif, bool enabled, u32 timeout)
{
	struct wid wid;
	int result;
	s8 power_mode;

	if (wilc_wlan_get_num_conn_ifcs(vif->wilc) == 2 && enabled)
		return 0;

	PRINT_INFO(vif->ndev, HOSTINF_DBG, "\n\n>> Setting PS to %d <<\n\n",
		   enabled);
	if (enabled)
		power_mode = WILC_FW_MIN_FAST_PS;
	else
		power_mode = WILC_FW_NO_POWERSAVE;

	wid.id = WID_POWER_MANAGEMENT;
	wid.val = &power_mode;
	wid.size = sizeof(char);
	result = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				      wilc_get_vif_idx(vif));
	if (result)
		PRINT_ER(vif->ndev, "Failed to send power management\n");
	else
		store_power_save_current_state(vif, power_mode);

	return result;
}

int wilc_setup_multicast_filter(struct wilc_vif *vif, bool enabled,
				u32 count, u8 *mc_list)
{
	int result;
	struct host_if_msg *msg;

	PRINT_INFO(vif->ndev, HOSTINF_DBG,
		   "Setting Multicast Filter params\n");
	msg = wilc_alloc_work(vif, handle_set_mcast_filter, false);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	msg->body.multicast_info.enabled = enabled;
	msg->body.multicast_info.cnt = count;
	msg->body.multicast_info.mc_list = mc_list;

	result = wilc_enqueue_work(msg);
	if (result) {
		PRINT_ER(vif->ndev, "enqueue work failed\n");
		kfree(msg);
	}
	return result;
}

int wilc_set_tx_power(struct wilc_vif *vif, u8 tx_power)
{
	int ret;
	struct wid wid;

	wid.id = WID_TX_POWER;
	wid.type = WID_CHAR;
	wid.val = &tx_power;
	wid.size = sizeof(char);

	ret = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				   wilc_get_vif_idx(vif));
	return ret;
}

int wilc_get_tx_power(struct wilc_vif *vif, u8 *tx_power)
{
	int ret;
	struct wid wid;

	wid.id = WID_TX_POWER;
	wid.type = WID_CHAR;
	wid.val = tx_power;
	wid.size = sizeof(char);

	ret = wilc_send_config_pkt(vif, WILC_GET_CFG, &wid, 1,
				   wilc_get_vif_idx(vif));
	return ret;
}

bool is_valid_gpio(struct wilc_vif *vif, u8 gpio)
{
	switch (vif->wilc->chip) {
	case WILC_1000:
		if (gpio == 0 || gpio == 1 || gpio == 4 || gpio == 6)
			return true;
		else
			return false;
	case WILC_3000:
		if (gpio == 0 || gpio == 3 || gpio == 4 ||
		    (gpio >= 17 && gpio <= 20))
			return true;
		else
			return false;
	default:
		return false;
	}
}

int wilc_set_antenna(struct wilc_vif *vif, u8 mode)
{
	struct wid wid;
	int ret;
	struct sysfs_attr_group *attr_syfs_p = &vif->attr_sysfs;
	struct host_if_set_ant set_ant;

	set_ant.mode = mode;

	if (attr_syfs_p->ant_swtch_mode == ANT_SWTCH_INVALID_GPIO_CTRL) {
		PRINT_ER(vif->ndev, "Ant switch GPIO mode is invalid.\n");
		PRINT_ER(vif->ndev, "Set it using /sys/wilc/ant_swtch_mode\n");
		return WILC_FAIL;
	}

	if (is_valid_gpio(vif, attr_syfs_p->antenna1)) {
		set_ant.antenna1 = attr_syfs_p->antenna1;
	} else {
		PRINT_ER(vif->ndev, "Invalid GPIO%d\n", attr_syfs_p->antenna1);
		return WILC_FAIL;
	}

	if (attr_syfs_p->ant_swtch_mode == ANT_SWTCH_DUAL_GPIO_CTRL) {
		if ((attr_syfs_p->antenna2 != attr_syfs_p->antenna1) &&
		    is_valid_gpio(vif, attr_syfs_p->antenna2)) {
			set_ant.antenna2 = attr_syfs_p->antenna2;
		} else {
			PRINT_ER(vif->ndev, "Invalid GPIO %d\n",
				 attr_syfs_p->antenna2);
			return WILC_FAIL;
		}
	}

	set_ant.gpio_mode = attr_syfs_p->ant_swtch_mode;

	wid.id = WID_ANTENNA_SELECTION;
	wid.type = WID_BIN;
	wid.val = (u8 *)&set_ant;
	wid.size = sizeof(struct host_if_set_ant);
	if (attr_syfs_p->ant_swtch_mode == ANT_SWTCH_SNGL_GPIO_CTRL)
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "set antenna %d on GPIO %d\n", set_ant.mode,
			   set_ant.antenna1);
	else if (attr_syfs_p->ant_swtch_mode == ANT_SWTCH_DUAL_GPIO_CTRL)
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "set antenna %d on GPIOs %d and %d\n",
			   set_ant.mode, set_ant.antenna1,
			   set_ant.antenna2);

	ret = wilc_send_config_pkt(vif, WILC_SET_CFG, &wid, 1,
				   wilc_get_vif_idx(vif));
	if (ret)
		PRINT_ER(vif->ndev, "Failed to set antenna mode\n");

	return ret;
}
