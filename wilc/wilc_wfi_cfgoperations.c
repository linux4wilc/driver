// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#include <linux/etherdevice.h>

#include "wilc_wfi_cfgoperations.h"
#include "linux_wlan.h"

#define ACTION_CAT_ID			24
#define ACTION_SUBTYPE_ID		25
#define P2P_PUB_ACTION_SUBTYPE		30

#define ACTION_FRAME			0xd0
#define GO_INTENT_ATTR_ID		0x04
#define CHANLIST_ATTR_ID		0x0b
#define OPERCHAN_ATTR_ID		0x11
#define PUB_ACTION_ATTR_ID		0x04
#define P2PELEM_ATTR_ID			0xdd

#define GO_NEG_REQ			0x00
#define GO_NEG_RSP			0x01
#define GO_NEG_CONF			0x02
#define P2P_INV_REQ			0x03
#define P2P_INV_RSP			0x04
#define PUBLIC_ACT_VENDORSPEC		0x09
#define GAS_INITIAL_REQ			0x0a
#define GAS_INITIAL_RSP			0x0b

#define INVALID_CHANNEL			0

static const struct ieee80211_txrx_stypes
	wilc_wfi_cfg80211_mgmt_types[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_STATION] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
	[NL80211_IFTYPE_AP] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			BIT(IEEE80211_STYPE_DISASSOC >> 4) |
			BIT(IEEE80211_STYPE_AUTH >> 4) |
			BIT(IEEE80211_STYPE_DEAUTH >> 4) |
			BIT(IEEE80211_STYPE_ACTION >> 4)
	},
	[NL80211_IFTYPE_P2P_CLIENT] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_DISASSOC >> 4) |
			BIT(IEEE80211_STYPE_AUTH >> 4) |
			BIT(IEEE80211_STYPE_DEAUTH >> 4)
	}
};

static const struct wiphy_wowlan_support wowlan_support = {
	.flags = WIPHY_WOWLAN_ANY
};

struct p2p_mgmt_data {
	int size;
	u8 *buff;
};

static const u8 p2p_oui[] = {0x50, 0x6f, 0x9A, 0x09};
static const u8 p2p_vendor_spec[] = {0xdd, 0x05, 0x00, 0x08, 0x40, 0x03};

static void cfg_scan_result(enum scan_event scan_event,
			    struct wilc_rcvd_net_info *info, void *user_void)
{
	struct wilc_priv *priv = user_void;

	if (!priv->cfg_scanning)
		return;

	if (scan_event == SCAN_EVENT_NETWORK_FOUND) {
		s32 freq;
		struct ieee80211_channel *channel;
		struct cfg80211_bss *bss;
		struct wiphy *wiphy = priv->dev->ieee80211_ptr->wiphy;

		if (!wiphy || !info)
			return;

		freq = ieee80211_channel_to_frequency((s32)info->ch,
						      NL80211_BAND_2GHZ);
		channel = ieee80211_get_channel(wiphy, freq);
		if (!channel)
			return;

		PRINT_D(priv->dev, CFG80211_DBG,
			"Network Info:: CHANNEL Frequency: %d, RSSI: %d,\n",
			freq, ((s32)info->rssi * 100));

		bss = cfg80211_inform_bss_frame(wiphy, channel, info->mgmt,
						info->frame_len,
						(s32)info->rssi * 100,
						GFP_KERNEL);
		if (!bss)
			cfg80211_put_bss(wiphy, bss);
	} else if (scan_event == SCAN_EVENT_DONE) {
		PRINT_INFO(priv->dev, CFG80211_DBG, "Scan Done[%p]\n",
			   priv->dev);
		mutex_lock(&priv->scan_req_lock);

		if (priv->scan_req) {
#if KERNEL_VERSION(4, 7, 0) <= LINUX_VERSION_CODE
			struct cfg80211_scan_info info = {
				.aborted = false,
			};

			cfg80211_scan_done(priv->scan_req, &info);
#else
			cfg80211_scan_done(priv->scan_req, false);
#endif
			priv->cfg_scanning = false;
			priv->scan_req = NULL;
		}
		mutex_unlock(&priv->scan_req_lock);
	} else if (scan_event == SCAN_EVENT_ABORTED) {
		mutex_lock(&priv->scan_req_lock);

		PRINT_INFO(priv->dev, CFG80211_DBG, "Scan Aborted\n");
		if (priv->scan_req) {
#if KERNEL_VERSION(4, 7, 0) <= LINUX_VERSION_CODE
			struct cfg80211_scan_info info = {
				.aborted = false,
			};
			cfg80211_scan_done(priv->scan_req, &info);
#else
			cfg80211_scan_done(priv->scan_req, false);
#endif

			priv->cfg_scanning = false;
			priv->scan_req = NULL;
		}
		mutex_unlock(&priv->scan_req_lock);
	}
}

static void cfg_connect_result(enum conn_event conn_disconn_evt,
			       u8 mac_status, void *priv_data)
{
	struct wilc_priv *priv = priv_data;
	struct net_device *dev = priv->dev;
	struct wilc_vif *vif = netdev_priv(dev);
	struct host_if_drv *wfi_drv = priv->hif_drv;
	struct wilc_conn_info *conn_info = &wfi_drv->conn_info;

	vif->connecting = false;

	if (conn_disconn_evt == EVENT_CONN_RESP) {
		u16 connect_status = conn_info->status;

		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Connection response received=%d connect_stat[%d]\n",
			   mac_status, connect_status);
		if (mac_status == WILC_MAC_STATUS_DISCONNECTED &&
		    connect_status == WLAN_STATUS_SUCCESS) {
			connect_status = WLAN_STATUS_UNSPECIFIED_FAILURE;
			wilc_wlan_set_bssid(priv->dev, NULL, WILC_STATION_MODE);

			if (vif->iftype != WILC_CLIENT_MODE)
				vif->wilc->sta_ch = INVALID_CHANNEL;

			PRINT_ER(dev, "Unspecified failure\n");
		}

		if (connect_status == WLAN_STATUS_SUCCESS) {
			PRINT_ER(dev,
				"Connection Successful: BSSID: %x%x%x%x%x%x\n",
				conn_info->bssid[0], conn_info->bssid[1],
				conn_info->bssid[2], conn_info->bssid[3],
				conn_info->bssid[4], conn_info->bssid[5]);
			memcpy(priv->associated_bss, conn_info->bssid,
			       ETH_ALEN);
		}

		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Association request info elements length = %d\n",
			   conn_info->req_ies_len);
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Association response info elements length = %d\n",
			   conn_info->resp_ies_len);
		cfg80211_connect_result(dev, conn_info->bssid,
					conn_info->req_ies,
					conn_info->req_ies_len,
					conn_info->resp_ies,
					conn_info->resp_ies_len, connect_status,
					GFP_KERNEL);
	} else if (conn_disconn_evt == EVENT_DISCONN_NOTIF) {
		u16 reason = 0;
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		vif->obtaining_ip = false;
#endif
		PRINT_ER(vif->ndev,
			 "Received WILC_MAC_STATUS_DISCONNECTED dev [%p]\n",
			 priv->dev);
		priv->p2p.local_random = 0x01;
		priv->p2p.recv_random = 0x00;
		priv->p2p.is_wilc_ie = false;
		eth_zero_addr(priv->associated_bss);
		wilc_wlan_set_bssid(priv->dev, NULL, WILC_STATION_MODE);

		if (vif->iftype != WILC_CLIENT_MODE) {
			vif->wilc->sta_ch = INVALID_CHANNEL;
		} else {
			if (wfi_drv->ifc_up)
				reason = 3;
			else
				reason = 1;
		}
#if KERNEL_VERSION(4, 2, 0) > LINUX_VERSION_CODE
		cfg80211_disconnected(dev, reason, NULL, 0, GFP_KERNEL);
#else
		cfg80211_disconnected(dev, reason, NULL, 0, false, GFP_KERNEL);
#endif
	}
}

static int set_channel(struct wiphy *wiphy,
		       struct cfg80211_chan_def *chandef)
{
	u32 channelnum = 0;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);
	int result = 0;

	channelnum = ieee80211_frequency_to_channel(chandef->chan->center_freq);
	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Setting channel %d with frequency %d\n",
		   channelnum, chandef->chan->center_freq);

	vif->wilc->op_ch = channelnum;
	result = wilc_set_mac_chnl_num(vif, channelnum);

	if (result != 0)
		PRINT_ER(priv->dev, "Error in setting channel %d\n",
			 channelnum);

	return result;
}

static inline int wilc_wfi_cfg_alloc_fill_ssid(struct wilc_vif *vif,
			     struct cfg80211_scan_request *request,
			     struct wilc_probe_ssid *search)
{
	int i;
	int slot_id = 0;

	search->ssid_info = kcalloc(request->n_ssids,
				    sizeof(*search->ssid_info), GFP_KERNEL);
	if (!search->ssid_info)
		goto out;

	search->n_ssids = request->n_ssids;

	for (i = 0; i < request->n_ssids; i++) {
		if (request->ssids[i].ssid_len > 0) {
			struct wilc_probe_ssid_info *info;

			info = &search->ssid_info[slot_id];
			info->ssid = kmemdup(request->ssids[i].ssid,
					     request->ssids[i].ssid_len,
					     GFP_KERNEL);
			if (!info->ssid)
				goto out_free;

			info->ssid_len = request->ssids[i].ssid_len;
			slot_id++;
		} else {
			search->n_ssids -= 1;
		}
	}
	return 0;

out_free:

	for (i = 0; i < slot_id; i++)
		kfree(search->ssid_info[i].ssid);

	kfree(search->ssid_info);
out:

	return -ENOMEM;
}

static int scan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);
	u32 i;
	int ret = 0;
	u8 scan_ch_list[WILC_MAX_NUM_SCANNED_CH];
	struct wilc_probe_ssid probe_ssid;

	if (request->n_channels > WILC_MAX_NUM_SCANNED_CH) {
		PRINT_ER(priv->dev, "Requested scanned channels over\n");
		return -EINVAL;
	}

	priv->scan_req = request;
	priv->cfg_scanning = true;
	for (i = 0; i < request->n_channels; i++) {
		u16 freq = request->channels[i]->center_freq;

		scan_ch_list[i] = (u8)ieee80211_frequency_to_channel(freq);
		PRINT_D(vif->ndev, CFG80211_DBG,
			"ScanChannel List[%d] = %d",
			i, scan_ch_list[i]);
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Requested num of channel %d\n",
		   request->n_channels);
	PRINT_INFO(vif->ndev, CFG80211_DBG, "Scan Request IE len =  %d\n",
		   request->ie_len);
	PRINT_INFO(vif->ndev, CFG80211_DBG, "Number of SSIDs %d\n",
		   request->n_ssids);
	if (wilc_wfi_cfg_alloc_fill_ssid(vif, request, &probe_ssid)) {
		ret = -ENOMEM;
		goto out;
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Trigger Scan Request\n");
	ret = wilc_scan(vif, WILC_FW_USER_SCAN, WILC_FW_ACTIVE_SCAN,
			scan_ch_list, request->n_channels,
			(const u8 *)request->ie, request->ie_len,
			cfg_scan_result, (void *)priv, &probe_ssid);

out:
	if (ret) {
		priv->scan_req = NULL;
		priv->cfg_scanning = false;
		PRINT_WRN(vif->ndev, CFG80211_DBG,
			  "Device is busy: Error(%d)\n", ret);
	}

	return ret;
}

static int connect(struct wiphy *wiphy, struct net_device *dev,
		   struct cfg80211_connect_params *sme)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);
	struct host_if_drv *wfi_drv = priv->hif_drv;
	int ret;
	u32 i;
	u8 security = WILC_FW_SEC_NO;
	enum authtype auth_type = WILC_FW_AUTH_ANY;
	u32 cipher_group;
	struct cfg80211_bss *bss;
	void *join_params;
	u8 ch;

	vif->connecting = true;

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Connecting to SSID [%s] on netdev [%p] host if [%x]\n",
		   sme->ssid, dev, (u32)priv->hif_drv);

	if (vif->iftype == WILC_CLIENT_MODE)
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Connected to Direct network,OBSS disabled\n");

	PRINT_D(vif->ndev, CFG80211_DBG, "Required SSID= %s\n, AuthType= %d\n",
		sme->ssid, sme->auth_type);

	memset(priv->wep_key, 0, sizeof(priv->wep_key));
	memset(priv->wep_key_len, 0, sizeof(priv->wep_key_len));

	PRINT_D(vif->ndev, CFG80211_DBG, "sme->crypto.wpa_versions=%x\n",
		sme->crypto.wpa_versions);
	PRINT_D(vif->ndev, CFG80211_DBG, "sme->crypto.cipher_group=%x\n",
		sme->crypto.cipher_group);
	PRINT_D(vif->ndev, CFG80211_DBG, "sme->crypto.n_ciphers_pairwise=%d\n",
		sme->crypto.n_ciphers_pairwise);
	for (i = 0; i < sme->crypto.n_ciphers_pairwise; i++)
		PRINT_D(vif->ndev, CORECONFIG_DBG,
			"sme->crypto.ciphers_pairwise[%d]=%x\n", i,
			sme->crypto.ciphers_pairwise[i]);

	cipher_group = sme->crypto.cipher_group;
	if (cipher_group != 0) {
		PRINT_INFO(vif->ndev, CORECONFIG_DBG,
			   ">> sme->crypto.wpa_versions: %x\n",
			   sme->crypto.wpa_versions);
		if (cipher_group == WLAN_CIPHER_SUITE_WEP40) {
			security = WILC_FW_SEC_WEP;
			PRINT_D(vif->ndev, CFG80211_DBG,
				"WEP Default Key Idx = %d\n", sme->key_idx);

			for (i = 0; i < sme->key_len; i++)
				PRINT_D(vif->ndev, CORECONFIG_DBG,
				"WEP Key Value[%d] = %d\n", i, sme->key[i]);

			priv->wep_key_len[sme->key_idx] = sme->key_len;
			memcpy(priv->wep_key[sme->key_idx], sme->key,
			       sme->key_len);

			wilc_set_wep_default_keyid(vif, sme->key_idx);
			wilc_add_wep_key_bss_sta(vif, sme->key, sme->key_len,
						 sme->key_idx);
		} else if (cipher_group == WLAN_CIPHER_SUITE_WEP104) {
			security = WILC_FW_SEC_WEP_EXTENDED;

			priv->wep_key_len[sme->key_idx] = sme->key_len;
			memcpy(priv->wep_key[sme->key_idx], sme->key,
			       sme->key_len);

			wilc_set_wep_default_keyid(vif, sme->key_idx);
			wilc_add_wep_key_bss_sta(vif, sme->key, sme->key_len,
						 sme->key_idx);
		} else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2) {
			if (cipher_group == WLAN_CIPHER_SUITE_TKIP)
				security = WILC_FW_SEC_WPA2_TKIP;
			else
				security = WILC_FW_SEC_WPA2_AES;
		} else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_1) {
			if (cipher_group == WLAN_CIPHER_SUITE_TKIP)
				security = WILC_FW_SEC_WPA_TKIP;
			else
				security = WILC_FW_SEC_WPA_AES;
		} else {
			ret = -ENOTSUPP;
			PRINT_ER(dev, "Unsupported cipher\n");
			goto out_error;
		}
	}

	if ((sme->crypto.wpa_versions & NL80211_WPA_VERSION_1) ||
	    (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)) {
		for (i = 0; i < sme->crypto.n_ciphers_pairwise; i++) {
			u32 ciphers_pairwise = sme->crypto.ciphers_pairwise[i];

			if (ciphers_pairwise == WLAN_CIPHER_SUITE_TKIP)
				security |= WILC_FW_TKIP;
			else
				security |= WILC_FW_AES;
		}
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Adding key with cipher group %x\n",
		   cipher_group);

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Authentication Type = %d\n",
		   sme->auth_type);
	switch (sme->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
		PRINT_INFO(vif->ndev, CFG80211_DBG, "In OPEN SYSTEM\n");
		auth_type = WILC_FW_AUTH_OPEN_SYSTEM;
		break;

	case NL80211_AUTHTYPE_SHARED_KEY:
		auth_type = WILC_FW_AUTH_SHARED_KEY;
		PRINT_INFO(vif->ndev, CFG80211_DBG, "In SHARED KEY\n");
		break;

	default:
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Automatic Authentication type= %d\n",
			   sme->auth_type);
		break;
	}

	if (sme->crypto.n_akm_suites) {
		if (sme->crypto.akm_suites[0] == WLAN_AKM_SUITE_8021X)
			auth_type = WILC_FW_AUTH_IEEE8021;
	}

	if (wfi_drv->usr_scan_req.scan_result) {
		netdev_err(vif->ndev, "%s: Scan in progress\n", __func__);
		ret = -EBUSY;
		goto out_error;
	}

#if KERNEL_VERSION(4, 1, 0) > LINUX_VERSION_CODE
	bss = cfg80211_get_bss(wiphy, sme->channel, sme->bssid, sme->ssid,
			       sme->ssid_len, WLAN_CAPABILITY_ESS,
			       WLAN_CAPABILITY_ESS);
#else
	bss = cfg80211_get_bss(wiphy, sme->channel, sme->bssid, sme->ssid,
			       sme->ssid_len, IEEE80211_BSS_TYPE_ANY,
			       IEEE80211_PRIVACY(sme->privacy));
#endif
	if (!bss) {
		ret = -EINVAL;
		goto out_error;
	}
	if (ether_addr_equal_unaligned(vif->bssid, bss->bssid)) {
		ret = -EALREADY;
		goto out_put_bss;
	}

	join_params = wilc_parse_join_bss_param(bss, &sme->crypto);
	if (!join_params) {
		PRINT_ER(vif->ndev, "%s: failed to construct join param\n",
			 __func__);
		ret = -EINVAL;
		goto out_put_bss;
	}
	ch = ieee80211_frequency_to_channel(bss->channel->center_freq);
	PRINT_D(vif->ndev, CFG80211_DBG, "Required Channel = %d\n", ch);
	vif->wilc->op_ch = ch;
	if (vif->iftype != WILC_CLIENT_MODE)
		vif->wilc->sta_ch = ch;

	wilc_wlan_set_bssid(dev, bss->bssid, WILC_STATION_MODE);

	wfi_drv->conn_info.security = security;
	wfi_drv->conn_info.auth_type = auth_type;
	wfi_drv->conn_info.ch = ch;
	wfi_drv->conn_info.conn_result = cfg_connect_result;
	wfi_drv->conn_info.arg = priv;
	wfi_drv->conn_info.param = join_params;

	ret = wilc_set_join_req(vif, bss->bssid, sme->ie, sme->ie_len);
	if (ret) {
		PRINT_ER(dev, "wilc_set_join_req(): Error(%d)\n", ret);
		ret = -ENOENT;
		if (vif->iftype != WILC_CLIENT_MODE)
			vif->wilc->sta_ch = INVALID_CHANNEL;
		wilc_wlan_set_bssid(dev, NULL, WILC_STATION_MODE);
		wfi_drv->conn_info.conn_result = NULL;
		kfree(join_params);
		goto out_put_bss;
	}
	kfree(join_params);
	cfg80211_put_bss(wiphy, bss);
	return 0;

out_put_bss:
	cfg80211_put_bss(wiphy, bss);

out_error:
	vif->connecting = false;
	return ret;
}

static int disconnect(struct wiphy *wiphy, struct net_device *dev,
		      u16 reason_code)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);
	struct wilc *wilc = vif->wilc;
	struct host_if_drv *wfi_drv;
	int ret;

	vif->connecting = false;

	if (!wilc)
		return -EIO;
	wfi_drv = (struct host_if_drv *)priv->hif_drv;
	if (vif->iftype != WILC_CLIENT_MODE)
		wilc->sta_ch = INVALID_CHANNEL;
	wilc_wlan_set_bssid(priv->dev, NULL, WILC_STATION_MODE);

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Disconnecting with reason code(%d)\n", reason_code);
	priv->p2p.local_random = 0x01;
	priv->p2p.recv_random = 0x00;
	priv->p2p.is_wilc_ie = false;
	wfi_drv->p2p_timeout = 0;

	ret = wilc_disconnect(vif);
	if (ret != 0) {
		PRINT_ER(priv->dev, "Error in disconnecting (%d)\n", ret);
		ret = -EINVAL;
	}

	return ret;
}

static inline void wilc_wfi_cfg_copy_wep_info(struct wilc_priv *priv,
					      u8 key_index,
					      struct key_params *params)
{
	priv->wep_key_len[key_index] = params->key_len;
	memcpy(priv->wep_key[key_index], params->key, params->key_len);
}

static int wilc_wfi_cfg_allocate_wpa_entry(struct wilc_priv *priv, u8 idx)
{
	if (!priv->wilc_gtk[idx]) {
		priv->wilc_gtk[idx] = kzalloc(sizeof(*priv->wilc_gtk[idx]),
					      GFP_KERNEL);
		if (!priv->wilc_gtk[idx])
			return -ENOMEM;
	}

	if (!priv->wilc_ptk[idx]) {
		priv->wilc_ptk[idx] = kzalloc(sizeof(*priv->wilc_ptk[idx]),
					      GFP_KERNEL);
		if (!priv->wilc_ptk[idx])
			return -ENOMEM;
	}

	return 0;
}

static int wilc_wfi_cfg_copy_wpa_info(struct wilc_wfi_key *key_info,
				      struct key_params *params)
{
	kfree(key_info->key);

	key_info->key = kmemdup(params->key, params->key_len, GFP_KERNEL);
	if (!key_info->key)
		return -ENOMEM;

	kfree(key_info->seq);

	if (params->seq_len > 0) {
		key_info->seq = kmemdup(params->seq, params->seq_len,
					GFP_KERNEL);
		if (!key_info->seq)
			return -ENOMEM;
	}

	key_info->cipher = params->cipher;
	key_info->key_len = params->key_len;
	key_info->seq_len = params->seq_len;

	return 0;
}

static int add_key(struct wiphy *wiphy, struct net_device *netdev, u8 key_index,
		   bool pairwise,
		   const u8 *mac_addr, struct key_params *params)

{
	int ret = 0, keylen = params->key_len;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	const u8 *rx_mic = NULL;
	const u8 *tx_mic = NULL;
	u8 mode = WILC_FW_SEC_NO;
	u8 op_mode;
	int i;
	struct wilc_vif *vif = netdev_priv(netdev);

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Adding key with cipher suite = %x\n", params->cipher);
	PRINT_INFO(vif->ndev, CFG80211_DBG, "%x %x %d\n", (u32)wiphy,
		   (u32)netdev, key_index);
	PRINT_INFO(vif->ndev, CFG80211_DBG, "key %x %x %x\n", params->key[0],
		   params->key[1],
		   params->key[2]);
	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		if (priv->wdev->iftype == NL80211_IFTYPE_AP) {
			wilc_wfi_cfg_copy_wep_info(priv, key_index, params);

			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Adding AP WEP Default key Idx = %d\n",
				   key_index);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Adding AP WEP Key len= %d\n",
				   params->key_len);

			for (i = 0; i < params->key_len; i++)
				PRINT_INFO(vif->ndev, CFG80211_DBG,
					   "WEP AP key val[%d] = %x\n", i,
					   params->key[i]);

			if (params->cipher == WLAN_CIPHER_SUITE_WEP40)
				mode = WILC_FW_SEC_WEP;
			else
				mode = WILC_FW_SEC_WEP_EXTENDED;

			ret = wilc_add_wep_key_bss_ap(vif, params->key,
						      params->key_len,
						      key_index, mode,
						      WILC_FW_AUTH_OPEN_SYSTEM);
			break;
		}
		if (memcmp(params->key, priv->wep_key[key_index],
			   params->key_len)) {
			wilc_wfi_cfg_copy_wep_info(priv, key_index, params);

			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Adding WEP Default key Idx = %d\n",
				   key_index);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Adding WEP Key length = %d\n",
				   params->key_len);
			ret = wilc_add_wep_key_bss_sta(vif, params->key,
						       params->key_len,
						       key_index);
		}

		break;

	case WLAN_CIPHER_SUITE_TKIP:
	case WLAN_CIPHER_SUITE_CCMP:
		if (priv->wdev->iftype == NL80211_IFTYPE_AP ||
		    priv->wdev->iftype == NL80211_IFTYPE_P2P_GO) {
			struct wilc_wfi_key *key;

			ret = wilc_wfi_cfg_allocate_wpa_entry(priv, key_index);
			if (ret)
				return -ENOMEM;

			if (params->key_len > 16 &&
			    params->cipher == WLAN_CIPHER_SUITE_TKIP) {
				tx_mic = params->key + 24;
				rx_mic = params->key + 16;
				keylen = params->key_len - 16;
			}

			if (!pairwise) {
				if (params->cipher == WLAN_CIPHER_SUITE_TKIP)
					mode = WILC_FW_SEC_WPA_TKIP;
				else
					mode = WILC_FW_SEC_WPA2_AES;

				priv->wilc_groupkey = mode;

				key = priv->wilc_gtk[key_index];
			} else {
				PRINT_D(vif->ndev, CFG80211_DBG,
					"STA Address: %x%x%x%x%x\n",
					mac_addr[0], mac_addr[1], mac_addr[2],
					mac_addr[3], mac_addr[4]);
				if (params->cipher == WLAN_CIPHER_SUITE_TKIP)
					mode = WILC_FW_SEC_WPA_TKIP;
				else
					mode = (priv->wilc_groupkey |
						WILC_FW_AES);

				key = priv->wilc_ptk[key_index];
			}
			ret = wilc_wfi_cfg_copy_wpa_info(key, params);
			if (ret)
				return -ENOMEM;

			op_mode = WILC_AP_MODE;
		} else {
			if (params->key_len > 16 &&
			    params->cipher == WLAN_CIPHER_SUITE_TKIP) {
				rx_mic = params->key + 24;
				tx_mic = params->key + 16;
				keylen = params->key_len - 16;
			}

			op_mode = WILC_STATION_MODE;
		}

		if (!pairwise)
			ret = wilc_add_rx_gtk(vif, params->key, keylen,
					      key_index, params->seq_len,
					      params->seq, rx_mic, tx_mic,
					      op_mode, mode);
		else
			ret = wilc_add_ptk(vif, params->key, keylen, mac_addr,
					   rx_mic, tx_mic, op_mode, mode,
					   key_index);

		break;

	default:
		PRINT_ER(netdev, "Unsupported cipher\n");
		ret = -ENOTSUPP;
	}

	return ret;
}

static int del_key(struct wiphy *wiphy, struct net_device *netdev,
		   u8 key_index,
		   bool pairwise,
		   const u8 *mac_addr)
{
	int ret = 0;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(netdev);
	struct wilc *wl = vif->wilc;

	if (netdev == wl->vif[0]->ndev) {
		if (priv->wilc_gtk[key_index]) {
			kfree(priv->wilc_gtk[key_index]->key);
			priv->wilc_gtk[key_index]->key = NULL;
			kfree(priv->wilc_gtk[key_index]->seq);
			priv->wilc_gtk[key_index]->seq = NULL;

			kfree(priv->wilc_gtk[key_index]);
			priv->wilc_gtk[key_index] = NULL;
		}

		if (priv->wilc_ptk[key_index]) {
			kfree(priv->wilc_ptk[key_index]->key);
			priv->wilc_ptk[key_index]->key = NULL;
			kfree(priv->wilc_ptk[key_index]->seq);
			priv->wilc_ptk[key_index]->seq = NULL;
			kfree(priv->wilc_ptk[key_index]);
			priv->wilc_ptk[key_index] = NULL;
		}
	}

	if (key_index <= 3 && priv->wep_key_len[key_index]) {
		memset(priv->wep_key[key_index], 0,
		       priv->wep_key_len[key_index]);
		priv->wep_key_len[key_index] = 0;
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Removing WEP key with index = %d\n",
			   key_index);
		ret = wilc_remove_wep_key(vif, key_index);
	}

	return ret;
}

static int get_key(struct wiphy *wiphy, struct net_device *netdev, u8 key_index,
		   bool pairwise, const u8 *mac_addr, void *cookie,
		   void (*callback)(void *cookie, struct key_params *))
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct  key_params key_params;
	struct wilc_vif *vif = netdev_priv(netdev);

	if (!pairwise) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Getting group key idx: %x\n", key_index);
		key_params.key = priv->wilc_gtk[key_index]->key;
		key_params.cipher = priv->wilc_gtk[key_index]->cipher;
		key_params.key_len = priv->wilc_gtk[key_index]->key_len;
		key_params.seq = priv->wilc_gtk[key_index]->seq;
		key_params.seq_len = priv->wilc_gtk[key_index]->seq_len;
	} else {
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Getting pairwise key\n");
		key_params.key = priv->wilc_ptk[key_index]->key;
		key_params.cipher = priv->wilc_ptk[key_index]->cipher;
		key_params.key_len = priv->wilc_ptk[key_index]->key_len;
		key_params.seq = priv->wilc_ptk[key_index]->seq;
		key_params.seq_len = priv->wilc_ptk[key_index]->seq_len;
	}

	callback(cookie, &key_params);

	return 0;
}

static int set_default_key(struct wiphy *wiphy, struct net_device *netdev,
			   u8 key_index, bool unicast, bool multicast)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	wilc_set_wep_default_keyid(vif, key_index);

	return 0;
}

#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
static int get_station(struct wiphy *wiphy, struct net_device *dev,
		       const u8 *mac, struct station_info *sinfo)
#else
static int get_station(struct wiphy *wiphy, struct net_device *dev,
		       u8 *mac, struct station_info *sinfo)
#endif
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc = vif->wilc;
	u32 i = 0;
	u32 associatedsta = ~0;
	u32 inactive_time = 0;

	if (vif->iftype == WILC_AP_MODE || vif->iftype == WILC_GO_MODE) {
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Getting station parameters\n");
		for (i = 0; i < NUM_STA_ASSOCIATED; i++) {
			if (!(memcmp(mac,
				     priv->assoc_stainfo.sta_associated_bss[i],
				     ETH_ALEN))) {
				associatedsta = i;
				break;
			}
		}

		if (associatedsta == ~0) {
			PRINT_ER(dev, "sta required is not associated\n");
			return -ENOENT;
		}

#if KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE
		sinfo->filled |= BIT(NL80211_STA_INFO_INACTIVE_TIME);
#else
		sinfo->filled |= STATION_INFO_INACTIVE_TIME;
#endif

		wilc_get_inactive_time(vif, mac, &inactive_time);
		sinfo->inactive_time = 1000 * inactive_time;
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Inactive time %d\n",
			   sinfo->inactive_time);
	} else if (vif->iftype == WILC_STATION_MODE) {
		struct rf_info stats;

		if (!wilc->initialized) {
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "driver not initialized\n");
			return -EBUSY;
		}
		wilc_get_statistics(vif, &stats);
#if KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE
		sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL) |
			      BIT(NL80211_STA_INFO_RX_PACKETS) |
			      BIT(NL80211_STA_INFO_TX_PACKETS) |
			      BIT(NL80211_STA_INFO_TX_FAILED) |
			      BIT(NL80211_STA_INFO_TX_BITRATE);
#else
		sinfo->filled |= STATION_INFO_SIGNAL |
			      STATION_INFO_RX_PACKETS |
			      STATION_INFO_TX_PACKETS |
			      STATION_INFO_TX_FAILED |
			      STATION_INFO_TX_BITRATE;
#endif
		sinfo->signal = stats.rssi;
		sinfo->rx_packets = stats.rx_cnt;
		sinfo->tx_packets = stats.tx_cnt + stats.tx_fail_cnt;
		sinfo->tx_failed = stats.tx_fail_cnt;
		sinfo->txrate.legacy = stats.link_speed * 10;

		if (stats.link_speed > TCP_ACK_FILTER_LINK_SPEED_THRESH &&
		    stats.link_speed != DEFAULT_LINK_SPEED)
			wilc_enable_tcp_ack_filter(vif, true);
		else if (stats.link_speed != DEFAULT_LINK_SPEED)
			wilc_enable_tcp_ack_filter(vif, false);

		PRINT_INFO(vif->ndev, CORECONFIG_DBG,
			   "*** stats[%d][%d][%d][%d][%d]\n", sinfo->signal,
			   sinfo->rx_packets, sinfo->tx_packets,
			   sinfo->tx_failed, sinfo->txrate.legacy);
	}
	return 0;
}

static int change_bss(struct wiphy *wiphy, struct net_device *dev,
		      struct bss_parameters *params)
{
	PRINT_INFO(dev, CFG80211_DBG, "Changing Bss parametrs\n");
	return 0;
}

static int set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	int ret;
	struct cfg_param_attr cfg_param_val;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	cfg_param_val.flag = 0;
	PRINT_INFO(vif->ndev, CFG80211_DBG, "Setting Wiphy params\n");

	if (changed & WIPHY_PARAM_RETRY_SHORT) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Setting WIPHY_PARAM_RETRY_SHORT %d\n",
			   wiphy->retry_short);
			cfg_param_val.flag  |= WILC_CFG_PARAM_RETRY_SHORT;
		cfg_param_val.short_retry_limit = wiphy->retry_short;
	}
	if (changed & WIPHY_PARAM_RETRY_LONG) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Setting WIPHY_PARAM_RETRY_LONG %d\n",
			   wiphy->retry_long);
			cfg_param_val.flag |= WILC_CFG_PARAM_RETRY_LONG;
		cfg_param_val.long_retry_limit = wiphy->retry_long;
	}
	if (changed & WIPHY_PARAM_FRAG_THRESHOLD) {
		if (wiphy->frag_threshold > 255 &&
		    wiphy->frag_threshold < 7937) {
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Setting WIPHY_PARAM_FRAG_THRESHOLD %d\n",
				   wiphy->frag_threshold);
			cfg_param_val.flag |= WILC_CFG_PARAM_RETRY_LONG;
			cfg_param_val.frag_threshold = wiphy->frag_threshold;
		} else {
			PRINT_ER(vif->ndev,
				 "Fragmentation threshold out of range\n");
			return -EINVAL;
		}
	}

	if (changed & WIPHY_PARAM_RTS_THRESHOLD) {
		if (wiphy->rts_threshold > 255) {
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Setting WIPHY_PARAM_RTS_THRESHOLD %d\n",
				   wiphy->rts_threshold);
			cfg_param_val.flag |= WILC_CFG_PARAM_RTS_THRESHOLD;
			cfg_param_val.rts_threshold = wiphy->rts_threshold;
		} else {
			PRINT_ER(vif->ndev, "RTS threshold out of range\n");
			return -EINVAL;
		}
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Setting CFG params in the host interface\n");
	ret = wilc_hif_set_cfg(vif, &cfg_param_val);
	if (ret)
		PRINT_ER(priv->dev, "Error in setting WIPHY PARAMS\n");

	return ret;
}

static int set_pmksa(struct wiphy *wiphy, struct net_device *netdev,
		     struct cfg80211_pmksa *pmksa)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);
	u32 i;
	int ret = 0;
	u8 flag = 0;

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Setting PMKSA\n");

	for (i = 0; i < priv->pmkid_list.numpmkid; i++)	{
		if (!memcmp(pmksa->bssid, priv->pmkid_list.pmkidlist[i].bssid,
			    ETH_ALEN)) {
			flag = PMKID_FOUND;
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "PMKID already exists\n");
			break;
		}
	}
	if (i < WILC_MAX_NUM_PMKIDS) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Setting PMKID in private structure\n");
		memcpy(priv->pmkid_list.pmkidlist[i].bssid, pmksa->bssid,
		       ETH_ALEN);
		memcpy(priv->pmkid_list.pmkidlist[i].pmkid, pmksa->pmkid,
		       WLAN_PMKID_LEN);
		if (!(flag == PMKID_FOUND))
			priv->pmkid_list.numpmkid++;
	} else {
		PRINT_ER(netdev, "Invalid PMKID index\n");
		ret = -EINVAL;
	}

	if (!ret) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Setting pmkid in the host interface\n");
		ret = wilc_set_pmkid_info(vif, &priv->pmkid_list);
	}
	return ret;
}

static int del_pmksa(struct wiphy *wiphy, struct net_device *netdev,
		     struct cfg80211_pmksa *pmksa)
{
	u32 i;
	int ret = 0;
	struct wilc_priv *priv = wiphy_priv(wiphy);

	PRINT_INFO(netdev, CFG80211_DBG, "Deleting PMKSA keys\n");

	for (i = 0; i < priv->pmkid_list.numpmkid; i++)	{
		if (!memcmp(pmksa->bssid, priv->pmkid_list.pmkidlist[i].bssid,
			    ETH_ALEN)) {
			PRINT_INFO(netdev, CFG80211_DBG,
				   "Reseting PMKID values\n");
			memset(&priv->pmkid_list.pmkidlist[i], 0,
			       sizeof(struct wilc_pmkid));
			break;
		}
	}

	if (i < priv->pmkid_list.numpmkid && priv->pmkid_list.numpmkid > 0) {
		for (; i < (priv->pmkid_list.numpmkid - 1); i++) {
			memcpy(priv->pmkid_list.pmkidlist[i].bssid,
			       priv->pmkid_list.pmkidlist[i + 1].bssid,
			       ETH_ALEN);
			memcpy(priv->pmkid_list.pmkidlist[i].pmkid,
			       priv->pmkid_list.pmkidlist[i + 1].pmkid,
			       WLAN_PMKID_LEN);
		}
		priv->pmkid_list.numpmkid--;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int flush_pmksa(struct wiphy *wiphy, struct net_device *netdev)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);

	PRINT_INFO(netdev, CFG80211_DBG, "Flushing  PMKID key values\n");
	memset(&priv->pmkid_list, 0, sizeof(struct wilc_pmkid_attr));

	return 0;
}

static inline void wilc_wfi_cfg_parse_ch_attr(struct wilc_vif *vif, u8 *buf,
					      u8 ch_list_attr_idx,
					      u8 op_ch_attr_idx, u8 sta_ch)
{
	int i = 0;
	int j = 0;

	if (ch_list_attr_idx) {
		u8 limit = ch_list_attr_idx + 3 + buf[ch_list_attr_idx + 1];

		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Modify channel list attribute [%d]\n",
			   sta_ch);
		for (i = ch_list_attr_idx + 3; i < limit; i++) {
			if (buf[i] == 0x51) {
				for (j = i + 2; j < ((i + 2) + buf[i + 1]); j++)
					buf[j] = sta_ch;
				break;
			}
		}
	}

	if (op_ch_attr_idx) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Modify operating channel attribute %d\n",
			   sta_ch);
		buf[op_ch_attr_idx + 6] = 0x51;
		buf[op_ch_attr_idx + 7] = sta_ch;
	}
}

static void wilc_wfi_cfg_parse_rx_action(struct wilc_vif *vif, u8 *buf,
					 u32 len, u8 sta_ch, bool p2p_mode)
{
	u32 index = 0;
	u8 op_channel_attr_index = 0;
	u8 channel_list_attr_index = 0;

	while (index < len) {
		if (buf[index] == GO_INTENT_ATTR_ID) {
			if (!p2p_mode)
				buf[index + 3] = (buf[index + 3]  & 0x01) |
						 (0x0f << 1);
			else
				buf[index + 3] = (buf[index + 3]  & 0x01) |
						 (0x00 << 1);
		}
		if (buf[index] ==  CHANLIST_ATTR_ID)
			channel_list_attr_index = index;
		else if (buf[index] ==  OPERCHAN_ATTR_ID)
			op_channel_attr_index = index;
		index += buf[index + 1] + 3;
	}
	if (sta_ch != INVALID_CHANNEL)
		wilc_wfi_cfg_parse_ch_attr(vif, buf, channel_list_attr_index,
					   op_channel_attr_index, sta_ch);
}

static void wilc_wfi_cfg_parse_tx_action(struct wilc_vif *vif, u8 *buf,
					 u32 len, bool oper_ch, u8 sta_ch,
					 u8 p2p_mode)
{
	u32 index = 0;
	u8 op_channel_attr_index = 0;
	u8 channel_list_attr_index = 0;

	while (index < len) {
		if (buf[index] == GO_INTENT_ATTR_ID) {
			if (!p2p_mode)
				buf[index + 3] = (buf[index + 3]  & 0x01) |
						 (0x00 << 1);
			else
				buf[index + 3] = (buf[index + 3]  & 0x01) |
						 (0x0f << 1);
			break;
		}

		if (buf[index] ==  CHANLIST_ATTR_ID)
			channel_list_attr_index = index;
		else if (buf[index] ==  OPERCHAN_ATTR_ID)
			op_channel_attr_index = index;
		index += buf[index + 1] + 3;
	}
	if (sta_ch != INVALID_CHANNEL && oper_ch)
		wilc_wfi_cfg_parse_ch_attr(vif, buf, channel_list_attr_index,
					   op_channel_attr_index, sta_ch);
}

static void wilc_wfi_cfg_parse_rx_vendor_spec(struct wilc_priv *priv, u8 *buff,
					      u32 size)
{
	int i;
	u8 subtype;
	struct wilc_vif *vif = netdev_priv(priv->dev);

	subtype = buff[P2P_PUB_ACTION_SUBTYPE];
	if ((subtype == GO_NEG_REQ || subtype == GO_NEG_RSP) &&
	    !priv->p2p.is_wilc_ie) {
		for (i = P2P_PUB_ACTION_SUBTYPE; i < size; i++) {
			if (!memcmp(p2p_vendor_spec, &buff[i], 6)) {
				priv->p2p.recv_random = buff[i + 6];
				priv->p2p.is_wilc_ie = true;
				PRINT_INFO(vif->ndev, GENERIC_DBG,
					   "WILC Vendor specific IE:%02x\n",
					   priv->p2p.recv_random);
				break;
			}
		}
	}

	if (priv->p2p.local_random <= priv->p2p.recv_random) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "PEER WILL BE GO LocaRand=%02x RecvRand %02x\n",
			   priv->p2p.local_random, priv->p2p.recv_random);
		return;
	}

	if (subtype == GO_NEG_REQ || subtype == GO_NEG_RSP ||
	    subtype == P2P_INV_REQ || subtype == P2P_INV_RSP) {
		for (i = P2P_PUB_ACTION_SUBTYPE + 2; i < size; i++) {
			if (buff[i] == P2PELEM_ATTR_ID &&
			    !(memcmp(p2p_oui, &buff[i + 2], 4))) {
				bool p2p_mode = vif->attr_sysfs.p2p_mode;

				wilc_wfi_cfg_parse_rx_action(vif, &buff[i + 6],
							     size - (i + 6),
							     vif->wilc->sta_ch,
							     p2p_mode);
				break;
			}
		}
	}
}

void wilc_wfi_p2p_rx(struct net_device *dev, u8 *buff, u32 size)
{
	struct wilc_priv *priv = wiphy_priv(dev->ieee80211_ptr->wiphy);
	struct host_if_drv *wfi_drv = priv->hif_drv;
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wl = vif->wilc;
	u32 header, pkt_offset;
	s32 freq;
	__le16 fc;

	memcpy(&header, (buff - HOST_HDR_OFFSET), HOST_HDR_OFFSET);

	pkt_offset = GET_PKT_OFFSET(header);

	fc = ((struct ieee80211_hdr *)buff)->frame_control;
	if (pkt_offset & IS_MANAGMEMENT_CALLBACK) {
		bool ack = false;

		if (ieee80211_is_probe_resp(fc) ||
		    pkt_offset & IS_MGMT_STATUS_SUCCES)
			ack = true;

		cfg80211_mgmt_tx_status(priv->wdev, priv->tx_cookie, buff, size,
					ack, GFP_KERNEL);
		return;
	}

	PRINT_D(vif->ndev, GENERIC_DBG, "Rx Frame Type:%x\n", fc);

#if KERNEL_VERSION(4, 7, 0) <= LINUX_VERSION_CODE
	freq = ieee80211_channel_to_frequency(wl->op_ch, NL80211_BAND_2GHZ);
 #else
	freq = ieee80211_channel_to_frequency(wl->op_ch, IEEE80211_BAND_2GHZ);
 #endif
	if (!ieee80211_is_action(fc)) {
		cfg80211_rx_mgmt(priv->wdev, freq, 0, buff, size, 0);
		return;
	}

	PRINT_D(vif->ndev, GENERIC_DBG,
		   "Rx Action Frame Type: %x %x\n",
		   buff[ACTION_SUBTYPE_ID],
		   buff[P2P_PUB_ACTION_SUBTYPE]);
	if (priv->cfg_scanning &&
	    time_after_eq(jiffies, (unsigned long)wfi_drv->p2p_timeout)) {
		PRINT_WRN(dev, GENERIC_DBG, "Receiving action wrong ch\n");
		return;
	}
	if (buff[ACTION_CAT_ID] == PUB_ACTION_ATTR_ID) {
		u8 subtype = buff[P2P_PUB_ACTION_SUBTYPE];

		switch (buff[ACTION_SUBTYPE_ID]) {
		case GAS_INITIAL_REQ:
			PRINT_D(vif->ndev, GENERIC_DBG,
				   "GAS INITIAL REQ %x\n",
				   buff[ACTION_SUBTYPE_ID]);
			break;

		case GAS_INITIAL_RSP:
			PRINT_D(vif->ndev, GENERIC_DBG,
				   "GAS INITIAL RSP %x\n",
				   buff[ACTION_SUBTYPE_ID]);
			break;

		case PUBLIC_ACT_VENDORSPEC:
			if (!memcmp(p2p_oui, &buff[ACTION_SUBTYPE_ID + 1], 4))
				wilc_wfi_cfg_parse_rx_vendor_spec(priv, buff,
								  size);

			if ((subtype == GO_NEG_REQ || subtype == GO_NEG_RSP) &&
			    priv->p2p.is_wilc_ie)
				size -= 7;

			break;

		default:
			PRINT_WRN(dev, GENERIC_DBG,
				   "Not handled action frame type:%x\n",
				   buff[ACTION_SUBTYPE_ID]);
			break;
		}
	}

	cfg80211_rx_mgmt(priv->wdev, freq, 0, buff, size, 0);
}

static void wilc_wfi_mgmt_tx_complete(void *priv, int status)
{
	struct p2p_mgmt_data *pv_data = priv;

	kfree(pv_data->buff);
	kfree(pv_data);
}

static void wilc_wfi_remain_on_channel_expired(void *data, u64 cookie)
{
	struct wilc_priv *priv = data;
	struct wilc_wfi_p2p_listen_params *params = &priv->remain_on_ch_params;

	if (cookie != priv->remain_on_ch_params.listen_cookie) {
		PRINT_INFO(priv->dev, GENERIC_DBG,
			   "Received cookies didn't match received[%llu] Expected[%llu]\n",
			   cookie, priv->remain_on_ch_params.listen_cookie);
		return;
	}

	priv->p2p_listen_state = false;

	cfg80211_remain_on_channel_expired(priv->wdev, cookie,
					   params->listen_ch, GFP_KERNEL);
}

static int remain_on_channel(struct wiphy *wiphy,
			     struct wireless_dev *wdev,
			     struct ieee80211_channel *chan,
			     unsigned int duration, u64 *cookie)
{
	int ret = 0;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);
	u64 id;

	PRINT_INFO(vif->ndev, GENERIC_DBG, "Remaining on channel [%d]\n",
		   chan->hw_value);

	if (wdev->iftype == NL80211_IFTYPE_AP) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Required while in AP mode\n");
		return ret;
	}

	id = ++priv->inc_roc_cookie;
	if (id == 0)
		id = ++priv->inc_roc_cookie;

	ret = wilc_remain_on_channel(vif, id, duration, chan->hw_value,
				     wilc_wfi_remain_on_channel_expired,
				     (void *)priv);
	if (ret)
		return ret;

	vif->wilc->op_ch = chan->hw_value;
	priv->remain_on_ch_params.listen_ch = chan;
	priv->remain_on_ch_params.listen_cookie = id;
	*cookie = id;
	priv->remain_on_ch_params.listen_duration = duration;
	priv->p2p_listen_state = true;

	cfg80211_ready_on_channel(wdev, *cookie, chan, duration, GFP_KERNEL);

#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
	vif->hif_drv->remain_on_ch_timer.data = (unsigned long)vif->hif_drv;
#endif
	mod_timer(&vif->hif_drv->remain_on_ch_timer,
		  jiffies + msecs_to_jiffies(duration));

	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "Remaining on duration [%d] [%llu]\n",
		   duration, priv->remain_on_ch_params.listen_cookie);

	return ret;
}

static int cancel_remain_on_channel(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    u64 cookie)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "cookie received[%llu] expected[%llu]\n",
		   cookie, priv->remain_on_ch_params.listen_cookie);
	if (cookie != priv->remain_on_ch_params.listen_cookie)
		return -ENOENT;

	return wilc_listen_state_expired(vif, cookie);
}

#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
static void wilc_wfi_cfg_tx_vendor_spec(struct wilc_priv *priv,
					struct p2p_mgmt_data *mgmt_tx,
					struct cfg80211_mgmt_tx_params *params,
					u8 iftype, u32 buf_len)
#else
static void wilc_wfi_cfg_tx_vendor_spec(struct wilc_priv *priv,
					struct p2p_mgmt_data *mgmt_tx,
					const u8 *buf, size_t len,
					u8 iftype, u32 buf_len)
#endif
{
#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
	const u8 *buf = params->buf;
	size_t len = params->len;
#endif
	struct wilc_vif *vif = netdev_priv(priv->dev);
	u32 i;
	u8 subtype = buf[P2P_PUB_ACTION_SUBTYPE];

	if (subtype == GO_NEG_REQ || subtype == GO_NEG_RSP) {
		if (priv->p2p.local_random == 1 &&
		    priv->p2p.recv_random < priv->p2p.local_random) {
			get_random_bytes(&priv->p2p.local_random, 1);
			priv->p2p.local_random++;
		}
	}

	if (priv->p2p.local_random <= priv->p2p.recv_random ||
	    !(subtype == GO_NEG_REQ || subtype == GO_NEG_RSP ||
	      subtype == P2P_INV_REQ || subtype == P2P_INV_RSP))
		return;

	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "LOCAL WILL BE GO LocaRand=%02x RecvRand %02x\n",
		   priv->p2p.local_random, priv->p2p.recv_random);
	for (i = P2P_PUB_ACTION_SUBTYPE + 2; i < len; i++) {
		if (buf[i] == P2PELEM_ATTR_ID &&
		    !memcmp(p2p_oui, &buf[i + 2], 4)) {
			bool oper_ch = false;
			u8 *tx_buff = &mgmt_tx->buff[i + 6];

			if (subtype == P2P_INV_REQ || subtype == P2P_INV_RSP)
				oper_ch = true;

			wilc_wfi_cfg_parse_tx_action(vif, tx_buff,
						     len - (i + 6), oper_ch,
						     vif->wilc->sta_ch,
						     vif->attr_sysfs.p2p_mode);
			break;
		}
	}

	if (subtype != P2P_INV_REQ && subtype != P2P_INV_RSP) {
		int vendor_spec_len = sizeof(p2p_vendor_spec);

		memcpy(&mgmt_tx->buff[len], p2p_vendor_spec,
		       vendor_spec_len);
		mgmt_tx->buff[len + vendor_spec_len] = priv->p2p.local_random;
		mgmt_tx->size = buf_len;
	}
}

#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
static int mgmt_tx(struct wiphy *wiphy,
		   struct wireless_dev *wdev,
		   struct cfg80211_mgmt_tx_params *params,
		   u64 *cookie)
#else
static int mgmt_tx(struct wiphy *wiphy,
		   struct wireless_dev *wdev,
		   struct ieee80211_channel *chan, bool offchan,
		   unsigned int wait, const u8 *buf, size_t len,
		   bool no_cck, bool dont_wait_for_ack, u64 *cookie)
#endif
{
#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
	struct ieee80211_channel *chan = params->chan;
	unsigned int wait = params->wait;
	const u8 *buf = params->buf;
	size_t len = params->len;
#endif
	const struct ieee80211_mgmt *mgmt;
	struct p2p_mgmt_data *mgmt_tx;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct host_if_drv *wfi_drv = priv->hif_drv;
	struct wilc_vif *vif = netdev_priv(wdev->netdev);
	u32 buf_len = len + sizeof(p2p_vendor_spec) +
		      sizeof(priv->p2p.local_random);
	int ret = 0;

	*cookie = prandom_u32();
	priv->tx_cookie = *cookie;
	mgmt = (const struct ieee80211_mgmt *)buf;

	if (!ieee80211_is_mgmt(mgmt->frame_control)) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "This function transmits only management frames\n");
		goto out;
	}

	mgmt_tx = kmalloc(sizeof(*mgmt_tx), GFP_KERNEL);
	if (!mgmt_tx) {
		PRINT_ER(vif->ndev,
			 "Failed to allocate memory for mgmt_tx structure\n");
		return -ENOMEM;
	}

	mgmt_tx->buff = kmalloc(buf_len, GFP_KERNEL);
	if (!mgmt_tx->buff) {
		ret = -ENOMEM;
		PRINT_ER(vif->ndev,
			 "Failed to allocate memory for mgmt_tx buff\n");
		kfree(mgmt_tx);
		goto out;
	}

	memcpy(mgmt_tx->buff, buf, len);
	mgmt_tx->size = len;

	if (ieee80211_is_probe_resp(mgmt->frame_control)) {
		PRINT_INFO(vif->ndev, GENERIC_DBG, "TX: Probe Response\n");
		PRINT_INFO(vif->ndev, GENERIC_DBG, "Setting channel: %d\n",
			   chan->hw_value);
		wilc_set_mac_chnl_num(vif, chan->hw_value);
		vif->wilc->op_ch = chan->hw_value;
		goto out_txq_add_pkt;
	}

	if (!ieee80211_is_action(mgmt->frame_control))
		goto out_txq_add_pkt;

	PRINT_INFO(vif->ndev, GENERIC_DBG, "ACTION FRAME:%x\n",
		   (u16)mgmt->frame_control);
	if (buf[ACTION_CAT_ID] == PUB_ACTION_ATTR_ID) {
		if (buf[ACTION_SUBTYPE_ID] != PUBLIC_ACT_VENDORSPEC ||
		    buf[P2P_PUB_ACTION_SUBTYPE] != GO_NEG_CONF) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Setting channel: %d\n",
				   chan->hw_value);
			wilc_set_mac_chnl_num(vif,
					      chan->hw_value);
			vif->wilc->op_ch = chan->hw_value;
		}
		switch (buf[ACTION_SUBTYPE_ID]) {
		case GAS_INITIAL_REQ:
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "GAS INITIAL REQ %x\n",
				   buf[ACTION_SUBTYPE_ID]);
			break;

		case GAS_INITIAL_RSP:
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "GAS INITIAL RSP %x\n",
				   buf[ACTION_SUBTYPE_ID]);
			break;

		case PUBLIC_ACT_VENDORSPEC:
			if (!memcmp(p2p_oui, &buf[ACTION_SUBTYPE_ID + 1], 4))
			#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
				wilc_wfi_cfg_tx_vendor_spec(priv, mgmt_tx,
							    params,
							    vif->iftype,
							    buf_len);
			#else
				wilc_wfi_cfg_tx_vendor_spec(priv, mgmt_tx, buf,
							    len, vif->iftype,
							    buf_len);
			#endif
			else
				PRINT_INFO(vif->ndev, GENERIC_DBG,
					   "Not a P2P public action frame\n");

			break;

		default:
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Not handled action frame type:%x\n",
				   buf[ACTION_SUBTYPE_ID]);
			break;
		}
	}

	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "TX: ACTION FRAME Type:%x : Chan:%d\n",
		   buf[ACTION_SUBTYPE_ID], chan->hw_value);
	wfi_drv->p2p_timeout = (jiffies + msecs_to_jiffies(wait));

out_txq_add_pkt:

	txq_add_mgmt_pkt(wdev->netdev, mgmt_tx,	 mgmt_tx->buff, mgmt_tx->size,
			 wilc_wfi_mgmt_tx_complete);

out:

	return ret;
}

static int mgmt_tx_cancel_wait(struct wiphy *wiphy,
			       struct wireless_dev *wdev,
			       u64 cookie)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct host_if_drv *wfi_drv = priv->hif_drv;

	wfi_drv->p2p_timeout = jiffies;

	if (!priv->p2p_listen_state) {
		struct wilc_wfi_p2p_listen_params *params;

		params = &priv->remain_on_ch_params;

		cfg80211_remain_on_channel_expired(priv->wdev,
						   params->listen_cookie,
						   params->listen_ch,
						   GFP_KERNEL);
	}

	return 0;
}

void wilc_mgmt_frame_register(struct wiphy *wiphy, struct wireless_dev *wdev,
			      u16 frame_type, bool reg)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->wdev->netdev);
	struct wilc *wl = vif->wilc;

	if (!frame_type)
		return;

	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "Frame registering Frame Type: %x: Boolean: %d\n",
		   frame_type, reg);
	switch (frame_type) {
	case IEEE80211_STYPE_PROBE_REQ:
		vif->frame_reg[0].type = frame_type;
		vif->frame_reg[0].reg = reg;
		break;

	case IEEE80211_STYPE_ACTION:
		vif->frame_reg[1].type = frame_type;
		vif->frame_reg[1].reg = reg;
		break;

	default:
		break;
	}

	if (!wl->initialized) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Return since mac is closed\n");
		return;
	}
	wilc_frame_register(vif, frame_type, reg);
}

static int set_cqm_rssi_config(struct wiphy *wiphy, struct net_device *dev,
			       s32 rssi_thold, u32 rssi_hyst)
{
	PRINT_INFO(dev, CFG80211_DBG, "Setting CQM RSSi Function\n");
	return 0;
}

static int dump_station(struct wiphy *wiphy, struct net_device *dev,
			int idx, u8 *mac, struct station_info *sinfo)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);
	int ret;

	if (idx != 0)
		return -ENOENT;

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Dumping station information\n");

	ret = wilc_get_rssi(vif, &sinfo->signal);
	if (ret)
		return ret;

#if KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE
	sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL);
#else
	sinfo->filled |= STATION_INFO_SIGNAL;
#endif

	return 0;
}

static int set_power_mgmt(struct wiphy *wiphy, struct net_device *dev,
			  bool enabled, int timeout)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	if (!priv->hif_drv) {
		PRINT_ER(dev, "hif driver is NULL\n");
		return -EIO;
	}

	/* Can't set PS during obtaining IP */
	if (vif->obtaining_ip == true) {
		PRINT_ER(dev,
			 "Device obtaining IP, Power Managment will be handled after IP Obtained\n");
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Save the Current state of the PS = %d\n", enabled);

		/* Save the current status of the PS */
		store_power_save_current_state(vif, enabled);

		return 0;
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   " Power save Enabled= %d , TimeOut = %d\n", enabled,
		   timeout);

	wilc_set_power_mgmt(vif, enabled, timeout);

	return 0;
}

#if KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE
static int change_virtual_intf(struct wiphy *wiphy, struct net_device *dev,
			       enum nl80211_iftype type,
			       struct vif_params *params)
#else
static int change_virtual_intf(struct wiphy *wiphy, struct net_device *dev,
			       enum nl80211_iftype type, u32 *flags,
			       struct vif_params *params)
#endif
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wl = vif->wilc;
	struct net_device *net_device_1 = wilc_get_if_netdev(wl, WILC_P2P_IFC);
	struct net_device *net_device_2 = wilc_get_if_netdev(wl, WILC_WLAN_IFC);
	struct wilc_vif *vif_1 = netdev_priv(net_device_1);
	struct wilc_vif *vif_2 = netdev_priv(net_device_2);

	PRINT_INFO(vif->ndev, HOSTAPD_DBG,
		   "In Change virtual interface function\n");
	PRINT_INFO(vif->ndev, HOSTAPD_DBG,
		   "Wireless interface name =%s\n", dev->name);
	priv->p2p.local_random = 0x01;
	priv->p2p.recv_random = 0x00;
	priv->p2p.is_wilc_ie = false;
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "Changing virtual interface, enable scan\n");
	handle_pwrsave_for_IP(vif, IP_STATE_DEFAULT);
#endif

	switch (type) {
	case NL80211_IFTYPE_STATION:
		vif->connecting = false;
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_STATION\n");
		dev->ieee80211_ptr->iftype = type;
		priv->wdev->iftype = type;
		vif->monitor_flag = 0;
		vif->iftype = WILC_STATION_MODE;
		wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
					 WILC_STATION_MODE, vif->ifc_id);
		wilc_set_operation_mode(vif, WILC_STATION_MODE);

		memset(priv->assoc_stainfo.sta_associated_bss, 0,
		       WILC_MAX_NUM_STA * ETH_ALEN);

		wilc_set_power_mgmt(vif_1, 1, 0);
		wilc_set_power_mgmt(vif_2, 1, 0);
		break;

	case NL80211_IFTYPE_P2P_CLIENT:
		vif->connecting = false;
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_P2P_CLIENT\n");
		dev->ieee80211_ptr->iftype = type;
		priv->wdev->iftype = type;
		vif->monitor_flag = 0;
		vif->iftype = WILC_CLIENT_MODE;
		wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
					 WILC_STATION_MODE, vif->ifc_id);
		wilc_set_operation_mode(vif, WILC_STATION_MODE);

		wilc_set_power_mgmt(vif_1, 0, 0);
		wilc_set_power_mgmt(vif_2, 0, 0);
		break;

	case NL80211_IFTYPE_AP:
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_AP\n");
		dev->ieee80211_ptr->iftype = type;
		priv->wdev->iftype = type;
		vif->iftype = WILC_AP_MODE;
		if (wl->initialized) {
			wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
						 WILC_AP_MODE, vif->ifc_id);
			wilc_set_operation_mode(vif, WILC_AP_MODE);
			wilc_set_power_mgmt(vif_1, 0, 0);
			wilc_set_power_mgmt(vif_2, 0, 0);
		}
		break;

	case NL80211_IFTYPE_P2P_GO:
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_GO\n");
		PRINT_INFO(vif->ndev, GENERIC_DBG, "start duringIP timer\n");

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		handle_pwrsave_for_IP(vif, IP_STATE_GO_ASSIGNING);
#endif
		dev->ieee80211_ptr->iftype = type;
		priv->wdev->iftype = type;
		vif->iftype = WILC_GO_MODE;
		wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
					WILC_AP_MODE, vif->ifc_id);
		wilc_set_operation_mode(vif, WILC_AP_MODE);
		wilc_set_power_mgmt(vif_1, 0, 0);
		wilc_set_power_mgmt(vif_2, 0, 0);
		break;
	case NL80211_IFTYPE_MONITOR:
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_MONITOR\n");
		dev->ieee80211_ptr->iftype = type;
		dev->type = ARPHRD_IEEE80211_RADIOTAP;
		priv->wdev->iftype = type;
		vif->iftype = WILC_MONITOR_MODE;
		if (wl->initialized) {
			wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
						 WILC_MONITOR_MODE,
						 vif->ifc_id);

			wilc_set_operation_mode(vif, WILC_MONITOR_MODE);
			wilc_set_power_mgmt(vif_1, 0, 0);
			wilc_set_power_mgmt(vif_2, 0, 0);
		}
		break;

	default:
		PRINT_ER(dev, "Unknown interface type= %d\n", type);
		return -EINVAL;
	}

	return 0;
}

static int start_ap(struct wiphy *wiphy, struct net_device *dev,
		    struct cfg80211_ap_settings *settings)
{
	struct wilc_vif *vif = netdev_priv(dev);
	int ret;

	PRINT_INFO(vif->ndev, HOSTAPD_DBG, "Starting ap\n");

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Interval= %d\n DTIM period= %d\n Head length= %d Tail length= %d\n",
		   settings->beacon_interval, settings->dtim_period,
		   settings->beacon.head_len, settings->beacon.tail_len);
	ret = set_channel(wiphy, &settings->chandef);

	if (ret != 0)
		PRINT_ER(dev, "Error in setting channel\n");

	wilc_wlan_set_bssid(dev, dev->dev_addr, WILC_AP_MODE);
	wilc_set_power_mgmt(vif, 0, 0);

	return wilc_add_beacon(vif, settings->beacon_interval,
			       settings->dtim_period, &settings->beacon);
}

static int change_beacon(struct wiphy *wiphy, struct net_device *dev,
			 struct cfg80211_beacon_data *beacon)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	PRINT_INFO(vif->ndev, HOSTAPD_DBG, "Setting beacon\n");

	return wilc_add_beacon(vif, 0, 0, beacon);
}

static int stop_ap(struct wiphy *wiphy, struct net_device *dev)
{
	int ret;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Deleting beacon\n");

	wilc_wlan_set_bssid(dev, NULL, WILC_AP_MODE);

	ret = wilc_del_beacon(vif);

	if (ret)
		PRINT_ER(dev, "Host delete beacon fail\n");

	return ret;
}

#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
static int add_station(struct wiphy *wiphy, struct net_device *dev,
		       const u8 *mac, struct station_parameters *params)
#else
static int add_station(struct wiphy *wiphy, struct net_device *dev,
		       u8 *mac, struct station_parameters *params)
#endif
{
	int ret = 0;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(dev);
	u8 *assoc_bss = priv->assoc_stainfo.sta_associated_bss[params->aid];

	if (vif->iftype == WILC_AP_MODE || vif->iftype == WILC_GO_MODE) {
		memcpy(assoc_bss, mac, ETH_ALEN);

		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Adding station parameters %d\n", params->aid);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "BSSID = %x%x%x%x%x%x\n",
			   assoc_bss[0], assoc_bss[1], assoc_bss[2],
			   assoc_bss[3], assoc_bss[4], assoc_bss[5]);
		PRINT_INFO(vif->ndev, HOSTAPD_DBG, "ASSOC ID = %d\n",
			   params->aid);
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Number of supported rates = %d\n",
			   params->supported_rates_len);

		PRINT_INFO(vif->ndev, CFG80211_DBG, "IS HT supported = %d\n",
			   (!params->ht_capa) ? false : true);

		if (params->ht_capa) {
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Capability Info = %d\n",
				   params->ht_capa->cap_info);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "AMPDU Params = %d\n",
				   params->ht_capa->ampdu_params_info);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "HT Extended params= %d\n",
				   params->ht_capa->extended_ht_cap_info);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Tx Beamforming Cap= %d\n",
				   params->ht_capa->tx_BF_cap_info);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Antenna selection info = %d\n",
				   params->ht_capa->antenna_selection_info);
		}

		PRINT_INFO(vif->ndev, CFG80211_DBG, "Flag Mask = %d\n",
			   params->sta_flags_mask);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Flag Set = %d\n",
			   params->sta_flags_set);
		ret = wilc_add_station(vif, (const u8 *)mac, params);
		if (ret)
			PRINT_ER(dev, "Host add station fail\n");
	}

	return ret;
}

#if KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE
static int del_station(struct wiphy *wiphy, struct net_device *dev,
		       struct station_del_parameters *params)
#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
static int del_station(struct wiphy *wiphy, struct net_device *dev,
		       const u8 *mac)
#else
static int del_station(struct wiphy *wiphy, struct net_device *dev,
		       u8 *mac)
#endif
{
#if KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE
	const u8 *mac = params->mac;
#endif
	int ret = 0;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(dev);
	struct sta_info *info;

	if (!(vif->iftype == WILC_AP_MODE || vif->iftype == WILC_GO_MODE))
		return ret;

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Deleting station\n");

	info = &priv->assoc_stainfo;

	if (!mac) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "All associated stations\n");
		ret = wilc_del_allstation(vif, info->sta_associated_bss);
	} else {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "With mac address: %x%x%x%x%x%x\n",
			   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}

	ret = wilc_del_station(vif, mac);
	if (ret)
		PRINT_ER(dev, "Host delete station fail\n");

	return ret;
}

#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
static int change_station(struct wiphy *wiphy, struct net_device *dev,
			  const u8 *mac, struct station_parameters *params)
#else
static int change_station(struct wiphy *wiphy, struct net_device *dev,
			  u8 *mac, struct station_parameters *params)
#endif
{
	int ret = 0;
	struct wilc_vif *vif = netdev_priv(dev);

	PRINT_D(vif->ndev, CFG80211_DBG, "Change station paramters\n");

	if (vif->iftype == WILC_AP_MODE || vif->iftype == WILC_GO_MODE) {
		PRINT_INFO(vif->ndev, CFG80211_DBG, "BSSID = %x%x%x%x%x%x\n",
			  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "ASSOC ID = %d\n",
			   params->aid);
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Number of supported rates = %d\n",
			   params->supported_rates_len);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "IS HT supported = %d\n",
			   (!params->ht_capa) ? false : true);
		if (params->ht_capa) {
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Capability Info = %d\n",
				   params->ht_capa->cap_info);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "AMPDU Params = %d\n",
				   params->ht_capa->ampdu_params_info);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "HT Extended params= %d\n",
				   params->ht_capa->extended_ht_cap_info);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Tx Beamforming Cap= %d\n",
				   params->ht_capa->tx_BF_cap_info);
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Antenna selection info = %d\n",
				   params->ht_capa->antenna_selection_info);
		}
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Flag Mask = %d\n",
			   params->sta_flags_mask);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Flag Set = %d\n",
			   params->sta_flags_set);
		ret = wilc_edit_station(vif, (const u8 *)mac, params);
		if (ret)
			PRINT_ER(dev, "Host edit station fail\n");
	}
	return ret;
}

#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
static struct wireless_dev *add_virtual_intf(struct wiphy *wiphy,
					     const char *name,
					     unsigned char name_assign_type,
					     enum nl80211_iftype type,
					     struct vif_params *params)
#elif KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE
static struct wireless_dev *add_virtual_intf(struct wiphy *wiphy,
					     const char *name,
					     unsigned char name_assign_type,
					     enum nl80211_iftype type,
					     u32 *flags,
					     struct vif_params *params)
#elif KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
static struct wireless_dev *add_virtual_intf(struct wiphy *wiphy,
					     const char *name,
					     enum nl80211_iftype type,
					     u32 *flags,
					     struct vif_params *params)
#else
static struct wireless_dev *add_virtual_intf(struct wiphy *wiphy,
					     char *name,
					     enum nl80211_iftype type,
					     u32 *flags,
					     struct vif_params *params)
#endif
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->wdev->netdev);
	struct net_device *new_ifc;

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Adding monitor interface[%p]\n",
		   priv->wdev->netdev);

	if (type == NL80211_IFTYPE_MONITOR) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Initializing mon ifc virtual device driver\n");
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Adding monitor interface[%p]\n", vif->ndev);
		new_ifc = wilc_wfi_init_mon_interface(vif->wilc, name,
						      vif->ndev);
		if (new_ifc) {
			PRINT_INFO(vif->ndev, CFG80211_DBG,
			"Setting monitor flag in private structure\n");
			vif = netdev_priv(priv->wdev->netdev);
			vif->monitor_flag = 1;
		} else {
			PRINT_ER(vif->ndev,
				 "Error in initializing monitor interface\n");
		}
	}
	return priv->wdev;
}

static int del_virtual_intf(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);

	PRINT_INFO(priv->dev, HOSTAPD_DBG, "Deleting virtual interface\n");
	return 0;
}

static int wilc_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);

	if (!wow)
		PRINT_INFO(priv->dev, GENERIC_DBG,
			   "No wake up triggers defined\n");
	else if (wow->any == 0)
		PRINT_INFO(priv->dev, GENERIC_DBG,
			   "The only supported wake up trigger (any) is not set\n");

	return 0;
}

static int wilc_resume(struct wiphy *wiphy)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	PRINT_INFO(vif->ndev, GENERIC_DBG, "cfg resume\n");
	return 0;
}

static void wilc_set_wakeup(struct wiphy *wiphy, bool enabled)
{
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	PRINT_INFO(vif->ndev, GENERIC_DBG, "cfg set wake up = %d\n", enabled);
	wilc_set_wowlan_trigger(vif, (u8)enabled);
}

static int set_tx_power(struct wiphy *wiphy, struct wireless_dev *wdev,
			enum nl80211_tx_power_setting type, int mbm)
{
	int ret;
	s32 tx_power = MBM_TO_DBM(mbm);
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Setting tx power %d\n", tx_power);
	if (tx_power < 0)
		tx_power = 0;
	else if (tx_power > 18)
		tx_power = 18;
	ret = wilc_set_tx_power(vif, tx_power);
	if (ret)
		PRINT_ER(vif->ndev, "Failed to set tx power\n");

	return ret;
}

static int get_tx_power(struct wiphy *wiphy, struct wireless_dev *wdev,
			int *dbm)
{
	int ret;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);
	struct wilc *wl = vif->wilc;

	/* If firmware is not started, return. */
	if (!wl->initialized)
		return -EIO;
	*dbm = 0;
	ret = wilc_get_tx_power(vif, (u8 *)dbm);
	if (ret)
		PRINT_ER(vif->ndev, "Failed to get tx power\n");

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Got tx power %d\n", *dbm);

	return ret;
}

static int set_antenna(struct wiphy *wiphy, u32 tx_ant, u32 rx_ant)
{
	int ret;
	struct wilc_priv *priv = wiphy_priv(wiphy);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Select antenna mode %d\n", tx_ant);
	if (!tx_ant || !rx_ant)
		return -EINVAL;

	ret = wilc_set_antenna(vif, (u8)(tx_ant-1));
	if (ret)
		PRINT_ER(vif->ndev, "Failed to set tx antenna\n");

	return ret;
}

static const struct cfg80211_ops wilc_cfg80211_ops = {
	.set_monitor_channel = set_channel,
	.scan = scan,
	.connect = connect,
	.disconnect = disconnect,
	.add_key = add_key,
	.del_key = del_key,
	.get_key = get_key,
	.set_default_key = set_default_key,
	.add_virtual_intf = add_virtual_intf,
	.del_virtual_intf = del_virtual_intf,
	.change_virtual_intf = change_virtual_intf,

	.start_ap = start_ap,
	.change_beacon = change_beacon,
	.stop_ap = stop_ap,
	.add_station = add_station,
	.del_station = del_station,
	.change_station = change_station,
	.get_station = get_station,
	.dump_station = dump_station,
	.change_bss = change_bss,
	.set_wiphy_params = set_wiphy_params,

	.set_pmksa = set_pmksa,
	.del_pmksa = del_pmksa,
	.flush_pmksa = flush_pmksa,
	.remain_on_channel = remain_on_channel,
	.cancel_remain_on_channel = cancel_remain_on_channel,
	.mgmt_tx_cancel_wait = mgmt_tx_cancel_wait,
	.mgmt_tx = mgmt_tx,
	.mgmt_frame_register = wilc_mgmt_frame_register,
	.set_power_mgmt = set_power_mgmt,
	.set_cqm_rssi_config = set_cqm_rssi_config,

	.suspend = wilc_suspend,
	.resume = wilc_resume,
	.set_wakeup = wilc_set_wakeup,
	.set_tx_power = set_tx_power,
	.get_tx_power = get_tx_power,
	.set_antenna = set_antenna,
};

static struct wireless_dev *wilc_wfi_cfg_alloc(struct net_device *net)
{
	struct wireless_dev *wdev;

	PRINT_INFO(net, CFG80211_DBG, "Allocating wireless device\n");
	wdev = kzalloc(sizeof(*wdev), GFP_KERNEL);
	if (!wdev) {
		PRINT_ER(net, "Cannot allocate wireless device\n");
		goto out;
	}

	wdev->wiphy = wiphy_new(&wilc_cfg80211_ops, sizeof(struct wilc_priv));
	if (!wdev->wiphy) {
		PRINT_ER(net, "Cannot allocate wiphy\n");
		goto free_mem;
	}

	return wdev;

free_mem:
	kfree(wdev);
out:
	return NULL;
}

struct wireless_dev *wilc_create_wiphy(struct net_device *net,
				       struct device *dev)
{
	struct wilc_priv *priv;
	struct wireless_dev *wdev;
	int ret;

	PRINT_INFO(net, CFG80211_DBG, "Registering wifi device\n");
	wdev = wilc_wfi_cfg_alloc(net);
	if (!wdev) {
		PRINT_ER(net, "wiphy new allocate failed\n");
		return NULL;
	}

	priv = wdev_priv(wdev);
	priv->wdev = wdev;

	memcpy(priv->bitrates, wilc_bitrates, sizeof(wilc_bitrates));
	memcpy(priv->channels, wilc_2ghz_channels, sizeof(wilc_2ghz_channels));
	priv->band.bitrates = priv->bitrates;
	priv->band.n_bitrates = ARRAY_SIZE(priv->bitrates);
	priv->band.channels = priv->channels;
	priv->band.n_channels = ARRAY_SIZE(wilc_2ghz_channels);

	priv->band.ht_cap.ht_supported = 1;
	priv->band.ht_cap.cap |= (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);
	priv->band.ht_cap.mcs.rx_mask[0] = 0xff;
	priv->band.ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_8K;
	priv->band.ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;

#if KERNEL_VERSION(4, 7, 0) <= LINUX_VERSION_CODE
	wdev->wiphy->bands[NL80211_BAND_2GHZ] = &priv->band;
#else
	wdev->wiphy->bands[IEEE80211_BAND_2GHZ] = &priv->band;
#endif

	wdev->wiphy->max_scan_ssids = WILC_MAX_NUM_PROBED_SSID;
#if KERNEL_VERSION(3, 11, 0) <= LINUX_VERSION_CODE
	wdev->wiphy->wowlan = &wowlan_support;
#else
	wdev->wiphy->wowlan = wowlan_support;
#endif
	wdev->wiphy->max_num_pmkids = WILC_MAX_NUM_PMKIDS;
	PRINT_D(net, CFG80211_DBG, "Max number of PMKIDs = %d\n",
		wdev->wiphy->max_num_pmkids);
	wdev->wiphy->max_scan_ie_len = 1000;
	wdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	memcpy(priv->cipher_suites, wilc_cipher_suites,
	       sizeof(wilc_cipher_suites));
	wdev->wiphy->cipher_suites = priv->cipher_suites;
	wdev->wiphy->n_cipher_suites = ARRAY_SIZE(wilc_cipher_suites);
	wdev->wiphy->available_antennas_tx = 0x3;
	wdev->wiphy->available_antennas_rx = 0x3;
	wdev->wiphy->mgmt_stypes = wilc_wfi_cfg80211_mgmt_types;

	wdev->wiphy->max_remain_on_channel_duration = 500;
	wdev->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
					BIT(NL80211_IFTYPE_AP) |
					BIT(NL80211_IFTYPE_MONITOR) |
					BIT(NL80211_IFTYPE_P2P_GO) |
					BIT(NL80211_IFTYPE_P2P_CLIENT);
	wdev->wiphy->flags |= WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL;
	wdev->iftype = NL80211_IFTYPE_STATION;

	PRINT_D(net, CFG80211_DBG,
		"Max scan ids= %d,Max scan IE len= %d,Signal Type= %d,Interface Modes= %d,Interface Type= %d\n",
		wdev->wiphy->max_scan_ssids, wdev->wiphy->max_scan_ie_len,
		wdev->wiphy->signal_type, wdev->wiphy->interface_modes,
		wdev->iftype);

	set_wiphy_dev(wdev->wiphy, dev);

	ret = wiphy_register(wdev->wiphy);
	if (ret) {
		PRINT_ER(net, "Cannot register wiphy device\n");
		wiphy_free(wdev->wiphy);
		kfree(wdev);
		return NULL;
	}

	priv->dev = net;
	return wdev;
}

int wilc_init_host_int(struct net_device *net)
{
	int ret;
	struct wilc_priv *priv = wdev_priv(net->ieee80211_ptr);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	PRINT_INFO(net, INIT_DBG, "Host[%p][%p]\n", net, net->ieee80211_ptr);

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	timer_setup(&vif->during_ip_timer, clear_during_ip, 0);
	#endif
	timer_setup(&priv->eap_buff_timer, eap_buff_timeout, 0);
#else
	#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	setup_timer(&vif->during_ip_timer, clear_during_ip, 0);
	#endif
	setup_timer(&priv->eap_buff_timer, eap_buff_timeout, 0);
#endif

	priv->p2p_listen_state = false;

	mutex_init(&priv->scan_req_lock);
	ret = wilc_init(net, &priv->hif_drv);
	if (ret)
		PRINT_ER(net, "Error while initializing hostinterface\n");

	return ret;
}

void wilc_deinit_host_int(struct net_device *net)
{
	int ret;
	struct wilc_priv *priv = wdev_priv(net->ieee80211_ptr);
	struct wilc_vif *vif = netdev_priv(priv->dev);

	priv->p2p_listen_state = false;

	mutex_destroy(&priv->scan_req_lock);
	ret = wilc_deinit(vif);

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	del_timer_sync(&vif->during_ip_timer);
#endif
	del_timer_sync(&priv->eap_buff_timer);

	if (ret)
		PRINT_ER(net, "Error while deinitializing host interface\n");
}

void wilc_free_wiphy(struct net_device *net)
{
	PRINT_INFO(net, CFG80211_DBG, "Unregistering wiphy\n");
	if (!net) {
		PRINT_INFO(net, INIT_DBG, "net_device is NULL\n");
		return;
	}

	if (!net->ieee80211_ptr) {
		PRINT_INFO(net, INIT_DBG, "ieee80211_ptr is NULL\n");
		return;
	}

	if (!net->ieee80211_ptr->wiphy) {
		PRINT_INFO(net, INIT_DBG, "wiphy is NULL\n");
		return;
	}

	wiphy_unregister(net->ieee80211_ptr->wiphy);

	PRINT_INFO(net, INIT_DBG, "Freeing wiphy\n");
	wiphy_free(net->ieee80211_ptr->wiphy);
	kfree(net->ieee80211_ptr);
}
