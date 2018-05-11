// SPDX-License-Identifier: GPL-2.0
#include "wilc_wfi_cfgoperations.h"
#include "linux_wlan.h"

#define ENCRYPT_ENABLED		BIT(0)
#define WEP			BIT(1)
#define WEP_EXTENDED		BIT(2)
#define WPA			BIT(3)
#define WPA2			BIT(4)
#define AES			BIT(5)
#define TKIP			BIT(6)

#define FRAME_TYPE_ID			0
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

#define nl80211_SCAN_RESULT_EXPIRE	(3 * HZ)
#define SCAN_RESULT_EXPIRE		(40 * HZ)

static const u32 cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

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

#define TCP_ACK_FILTER_LINK_SPEED_THRESH	54
#define DEFAULT_LINK_SPEED			72

#define GET_PKT_OFFSET(a) (((a) >> 22) & 0x1ff)

static struct network_info last_scanned_shadow[MAX_NUM_SCANNED_NETWORKS_SHADOW];
static u32 last_scanned_cnt;
static u8 op_ifcs;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
#define CHAN2G(_channel, _freq, _flags) {	 \
		.band             = IEEE80211_BAND_2GHZ, \
		.center_freq      = (_freq),		 \
		.hw_value         = (_channel),		 \
		.flags            = (_flags),		 \
		.max_antenna_gain = 0,			 \
		.max_power        = 30,			 \
}				
#else
#define CHAN2G(_channel, _freq, _flags) {	 \
		.band             = NL80211_BAND_2GHZ, \
		.center_freq      = (_freq),		 \
		.hw_value         = (_channel),		 \
		.flags            = (_flags),		 \
		.max_antenna_gain = 0,			 \
		.max_power        = 30,			 \
}
#endif

static struct ieee80211_channel ieee80211_2ghz_channels[] = {
	CHAN2G(1,  2412, 0),
	CHAN2G(2,  2417, 0),
	CHAN2G(3,  2422, 0),
	CHAN2G(4,  2427, 0),
	CHAN2G(5,  2432, 0),
	CHAN2G(6,  2437, 0),
	CHAN2G(7,  2442, 0),
	CHAN2G(8,  2447, 0),
	CHAN2G(9,  2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0),
};

#define RATETAB_ENT(_rate, _hw_value, _flags) {	\
		.bitrate  = (_rate),			\
		.hw_value = (_hw_value),		\
		.flags    = (_flags),			\
}

static struct ieee80211_rate ieee80211_bitrates[] = {
	RATETAB_ENT(10,  0,  0),
	RATETAB_ENT(20,  1,  0),
	RATETAB_ENT(55,  2,  0),
	RATETAB_ENT(110, 3,  0),
	RATETAB_ENT(60,  9,  0),
	RATETAB_ENT(90,  6,  0),
	RATETAB_ENT(120, 7,  0),
	RATETAB_ENT(180, 8,  0),
	RATETAB_ENT(240, 9,  0),
	RATETAB_ENT(360, 10, 0),
	RATETAB_ENT(480, 11, 0),
	RATETAB_ENT(540, 12, 0),
};

struct p2p_mgmt_data {
	int size;
	u8 *buff;
};

static u8 wlan_channel = INVALID_CHANNEL;
static u8 curr_channel;
static u8 p2p_oui[] = {0x50, 0x6f, 0x9A, 0x09};
static u8 p2p_local_random = 0x01;
static u8 p2p_recv_random;
static u8 p2p_vendor_spec[] = {0xdd, 0x05, 0x00, 0x08, 0x40, 0x03};
static bool wilc_ie;

static struct ieee80211_supported_band wilc_band_2ghz = {
	.channels = ieee80211_2ghz_channels,
	.n_channels = ARRAY_SIZE(ieee80211_2ghz_channels),
	.bitrates = ieee80211_bitrates,
	.n_bitrates = ARRAY_SIZE(ieee80211_bitrates),
};

#define AGING_TIME	(9 * 1000)

void clear_shadow_scan(struct wilc_vif *vif)
{
	int i;

	if (op_ifcs == 0)
		return;

	del_timer_sync(&vif->wilc->aging_timer);
	PRINT_D(vif->ndev, CORECONFIG_DBG, "destroy aging timer\n");

	for (i = 0; i < last_scanned_cnt; i++) {
		if (last_scanned_shadow[i].ies) {
			kfree(last_scanned_shadow[i].ies);
			last_scanned_shadow[i].ies = NULL;
		}

		kfree(last_scanned_shadow[i].join_params);
		last_scanned_shadow[i].join_params = NULL;
	}
	last_scanned_cnt = 0;
}

void filter_shadow_scan(void* pUserVoid, u8 *ch_freq_list, u8 ch_list_len)
{
	struct WILC_WFI_priv* priv;
	int i;
	int ch_index;
	int j;
 	priv = (struct WILC_WFI_priv*)pUserVoid;

	if(ch_list_len > 0) {
		for(i = 0;i < last_scanned_cnt;) {
			for(ch_index=0;ch_index < ch_list_len;ch_index++) 				
				if(last_scanned_shadow[i].ch == (ch_freq_list[ch_index] + 1))
					break;

			/* filter only un-matched channels */
			if (ch_index == ch_list_len){
				if (last_scanned_shadow[i].ies){
					kfree(last_scanned_shadow[i].ies);
					last_scanned_shadow[i].ies = NULL;
				}

				kfree(last_scanned_shadow[i].join_params);
				last_scanned_shadow[i].join_params = NULL;

				for(j=i;(j<last_scanned_cnt-1);j++)
					last_scanned_shadow[j] = last_scanned_shadow[j+1];

				last_scanned_cnt--;
				continue;
			}
			i++;
		}
	}
}

static u32 get_rssi_avg(struct network_info *network_info)
{
	u8 i;
	int rssi_v = 0;
	u8 num_rssi = (network_info->rssi_history.full) ?
		       NUM_RSSI : (network_info->rssi_history.index);

	for (i = 0; i < num_rssi; i++)
		rssi_v += network_info->rssi_history.samples[i];

	rssi_v /= num_rssi;
	return rssi_v;
}

static void refresh_scan(struct wilc_priv *priv, bool direct_scan)
{
	struct wiphy *wiphy = priv->dev->ieee80211_ptr->wiphy;
	int i;

	for (i = 0; i < last_scanned_cnt; i++) {
		struct network_info *network_info;
		s32 freq;
		struct ieee80211_channel *channel;
		int rssi;
		struct cfg80211_bss *bss;

		network_info = &last_scanned_shadow[i];

		if (!network_info)
			continue;

		if (!memcmp("DIRECT-", network_info->ssid, 7) && !direct_scan)
			continue;

	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0)
		freq = ieee80211_channel_to_frequency((s32)network_info->ch,
						      NL80211_BAND_2GHZ);
	#else
		freq = ieee80211_channel_to_frequency((s32)network_info->ch,
											  IEEE80211_BAND_2GHZ);
	#endif
		channel = ieee80211_get_channel(wiphy, freq);
		rssi = get_rssi_avg(network_info);
		bss = cfg80211_inform_bss(wiphy,
					  channel,
				#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
					  CFG80211_BSS_FTYPE_UNKNOWN,
				#endif
					  network_info->bssid,
					  network_info->tsf_hi,
					  network_info->cap_info,
					  network_info->beacon_period,
					  (const u8 *)network_info->ies,
					  (size_t)network_info->ies_len,
					  (s32)rssi * 100,
					  GFP_KERNEL);
		cfg80211_put_bss(wiphy, bss);
	}
}

static void reset_shadow_found(void)
{
	int i;

	for (i = 0; i < last_scanned_cnt; i++)
		last_scanned_shadow[i].found = 0;
}

static void update_scan_time(void)
{
	int i;

	for (i = 0; i < last_scanned_cnt; i++)
		last_scanned_shadow[i].time_scan = jiffies;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)
void remove_network_from_shadow(struct timer_list *t)
#else
void remove_network_from_shadow(unsigned long arg)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)
	struct wilc* wilc = from_timer(wilc, t, aging_timer);
#else
	struct wilc* wilc = (struct wilc*)arg;
#endif
	struct wilc_vif* vif = wilc->aging_timer_vif;
	unsigned long now = jiffies;
	int i, j;

	for (i = 0; i < last_scanned_cnt; i++) {
		if (!time_after(now, last_scanned_shadow[i].time_scan +
				(unsigned long)(SCAN_RESULT_EXPIRE)))
			continue;
	
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Network expired in ScanShadow: %s\n",
			   last_scanned_shadow[i].ssid);
		kfree(last_scanned_shadow[i].ies);
		last_scanned_shadow[i].ies = NULL;

		kfree(last_scanned_shadow[i].join_params);

		for (j = i; (j < last_scanned_cnt - 1); j++)
			last_scanned_shadow[j] = last_scanned_shadow[j + 1];

		last_scanned_cnt--;
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Number of cached networks: %d\n",
		   last_scanned_cnt);

	if (last_scanned_cnt != 0) {
	#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
		wilc->aging_timer.data = (unsigned long) wilc;
	#endif
		wilc->aging_timer_vif = vif;
		mod_timer(&wilc->aging_timer,
			  jiffies + msecs_to_jiffies(AGING_TIME));
	}
	else {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "No need to restart Aging timer\n");
	}
}

static int is_network_in_shadow(struct network_info *nw_info, struct wilc_priv *priv)
{
	struct wilc* wilc;
	struct wilc_vif* vif;
	struct net_device *dev;
	int state = -1;
	int i;

	dev = priv->dev;
	vif = netdev_priv(dev);
	wilc = vif->wilc;

	if (last_scanned_cnt == 0) {
		PRINT_INFO(priv->dev, CFG80211_DBG, "Starting Aging timer\n");
	#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
		wilc->aging_timer.data = (unsigned long) wilc;
	#endif
		wilc->aging_timer_vif = vif;
		mod_timer(&wilc->aging_timer,
			  jiffies + msecs_to_jiffies(AGING_TIME));
		state = -1;
	} else {
		for (i = 0; i < last_scanned_cnt; i++) {
			if (memcmp(last_scanned_shadow[i].bssid,
				   nw_info->bssid, 6) == 0) {
				state = i;
				break;
			}
		}
	}
	return state;
}

static void add_network_to_shadow(struct network_info *nw_info,
				  struct wilc_priv *priv, void *join_params)
{
	int ap_found = is_network_in_shadow(nw_info, priv);
	u32 ap_index = 0;
	u8 rssi_index = 0;
	struct network_info *shadow_nw_info;

	if (last_scanned_cnt >= MAX_NUM_SCANNED_NETWORKS_SHADOW) {
		PRINT_INFO(priv->dev, CFG80211_DBG,
			   "Shadow network reached its maximum limit\n");
		return;
	}
	if (ap_found == -1) {
		ap_index = last_scanned_cnt;
		last_scanned_cnt++;
	} else {
		ap_index = ap_found;
	}
	shadow_nw_info = &last_scanned_shadow[ap_index];
	rssi_index = shadow_nw_info->rssi_history.index;
	shadow_nw_info->rssi_history.samples[rssi_index++] = nw_info->rssi;
	if (rssi_index == NUM_RSSI) {
		rssi_index = 0;
		shadow_nw_info->rssi_history.full = true;
	}
	shadow_nw_info->rssi_history.index = rssi_index;
	shadow_nw_info->rssi = nw_info->rssi;
	shadow_nw_info->cap_info = nw_info->cap_info;
	shadow_nw_info->ssid_len = nw_info->ssid_len;
	memcpy(shadow_nw_info->ssid, nw_info->ssid, nw_info->ssid_len);
	memcpy(shadow_nw_info->bssid, nw_info->bssid, ETH_ALEN);
	shadow_nw_info->beacon_period = nw_info->beacon_period;
	shadow_nw_info->dtim_period = nw_info->dtim_period;
	shadow_nw_info->ch = nw_info->ch;
	shadow_nw_info->tsf_hi = nw_info->tsf_hi;
	if (ap_found != -1)
		kfree(shadow_nw_info->ies);
	shadow_nw_info->ies = kmemdup(nw_info->ies, nw_info->ies_len,
				      GFP_KERNEL);
	if (shadow_nw_info->ies)
		shadow_nw_info->ies_len = nw_info->ies_len;
	else
		shadow_nw_info->ies_len = 0;
	shadow_nw_info->time_scan = jiffies;
	shadow_nw_info->time_scan_cached = jiffies;
	shadow_nw_info->found = 1;
	if (ap_found != -1)
		kfree(shadow_nw_info->join_params);
	shadow_nw_info->join_params = join_params;
}

static void cfg_scan_result(enum scan_event scan_event,
			    struct network_info *network_info,
			    void *user_void, void *join_params)
{
	struct wilc_priv *priv;
	struct wiphy *wiphy;
	s32 freq;
	struct ieee80211_channel *channel;
	struct cfg80211_bss *bss = NULL;

	priv = user_void;
	if (!priv->cfg_scanning)
		return;

	if (scan_event == SCAN_EVENT_NETWORK_FOUND) {
		wiphy = priv->dev->ieee80211_ptr->wiphy;

		if (!wiphy || !network_info)
			return;

		if (wiphy->signal_type == CFG80211_SIGNAL_TYPE_UNSPEC &&
		    (((s32)network_info->rssi * 100) < 0 ||
		    ((s32)network_info->rssi * 100) > 100))
			return;

		freq = ieee80211_channel_to_frequency((s32)network_info->ch,
						      NL80211_BAND_2GHZ);
		channel = ieee80211_get_channel(wiphy, freq);

		if (!channel)
			return;
		PRINT_D(priv->dev, CFG80211_DBG,
			"Network Info:: CHANNEL Frequency: %d, RSSI: %d, CapabilityInfo: %d, BeaconPeriod: %d\n",
			channel->center_freq,
			((s32)network_info->rssi * 100),
			network_info->cap_info,
			network_info->beacon_period);

		if (network_info->new_network) {
			if (priv->rcvd_ch_cnt >= MAX_NUM_SCANNED_NETWORKS) {
				PRINT_ER(priv->dev,
					 "Discovered networks exceeded the max limit\n");
				return;
			}
			
			PRINT_INFO(priv->dev, CFG80211_DBG,
				    "Network %s found\n",
				    network_info->ssid);
			priv->rcvd_ch_cnt++;

			add_network_to_shadow(network_info, priv, join_params);

			if (memcmp("DIRECT-", network_info->ssid, 7))
				return;

			bss = cfg80211_inform_bss(wiphy,
						  channel,
					#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
						  CFG80211_BSS_FTYPE_UNKNOWN,
					#endif
						  network_info->bssid,
						  network_info->tsf_hi,
						  network_info->cap_info,
						  network_info->beacon_period,
						  (const u8 *)network_info->ies,
						  (size_t)network_info->ies_len,
						  (s32)network_info->rssi * 100,
						  GFP_KERNEL);
			cfg80211_put_bss(wiphy, bss);
		} else {
			u32 i;

			for (i = 0; i < priv->rcvd_ch_cnt; i++) {
				if (memcmp(last_scanned_shadow[i].bssid,
					   network_info->bssid, 6) == 0)
					break;
			}

			if (i >= priv->rcvd_ch_cnt)
				return;

			PRINT_INFO(priv->dev, CFG80211_DBG,
				   "Update RSSI of %s\n",
				   last_scanned_shadow[i].ssid);
			last_scanned_shadow[i].rssi = network_info->rssi;
			last_scanned_shadow[i].time_scan = jiffies;
		}
	} else if (scan_event == SCAN_EVENT_DONE) {
		PRINT_INFO(priv->dev, CFG80211_DBG, "Scan Done[%p]\n",
			   priv->dev);
		PRINT_INFO(priv->dev, CFG80211_DBG, "Refreshing Scan ...\n");
		refresh_scan(priv, false);

		if (priv->rcvd_ch_cnt > 0)
			PRINT_INFO(priv->dev, CFG80211_DBG,
				   "%d Network(s) found\n", priv->rcvd_ch_cnt);
		else
			PRINT_INFO(priv->dev, CFG80211_DBG,
				   "No networks found\n");
		mutex_lock(&priv->scan_req_lock);

		if (priv->scan_req) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0)
			struct cfg80211_scan_info info = {
				.aborted = false,
			};

			cfg80211_scan_done(priv->scan_req, &info);
#else
			cfg80211_scan_done(priv->scan_req, false);
#endif
			priv->rcvd_ch_cnt = 0;
			priv->cfg_scanning = false;
			priv->scan_req = NULL;
		}
		mutex_unlock(&priv->scan_req_lock);
	} else if (scan_event == SCAN_EVENT_ABORTED) {
		mutex_lock(&priv->scan_req_lock);

		PRINT_INFO(priv->dev, CFG80211_DBG, "Scan Aborted \n");
		if (priv->scan_req) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0)
			struct cfg80211_scan_info info = {
				.aborted = false,
			};
			cfg80211_scan_done(priv->scan_req, &info);
#else
			cfg80211_scan_done(priv->scan_req, false);
#endif

			update_scan_time();
			refresh_scan(priv, false);
			priv->cfg_scanning = false;
			priv->scan_req = NULL;
		}
		mutex_unlock(&priv->scan_req_lock);
	}
}

static inline bool wilc_wfi_cfg_scan_time_expired(int i)
{
	unsigned long now = jiffies;

	if (time_after(now, last_scanned_shadow[i].time_scan_cached +
		       (unsigned long)(nl80211_SCAN_RESULT_EXPIRE - (1 * HZ))))
		return true;
	else
		return false;
}

int wilc_connecting;

static void cfg_connect_result(enum conn_event conn_disconn_evt,
			       struct connect_info *conn_info,
			       u8 mac_status,
			       struct disconnect_info *disconn_info,
			       void *priv_data)
{
	struct wilc_priv *priv;
	struct net_device *dev;
	struct host_if_drv *wfi_drv;
	u8 null_bssid[ETH_ALEN] = {0};
	struct wilc *wl;
	struct wilc_vif *vif;

	wilc_connecting = 0;

	priv = priv_data;
	dev = priv->dev;
	vif = netdev_priv(dev);
	wl = vif->wilc;
	wfi_drv = (struct host_if_drv *)priv->hif_drv;

	if (conn_disconn_evt == CONN_DISCONN_EVENT_CONN_RESP) {
		u16 connect_status;

		connect_status = conn_info->status;

		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Connection response received = %d\n",
			   mac_status);
		if (mac_status == MAC_STATUS_DISCONNECTED &&
		    conn_info->status == SUCCESSFUL_STATUSCODE) {
			connect_status = WLAN_STATUS_UNSPECIFIED_FAILURE;
			wilc_wlan_set_bssid(priv->dev, null_bssid,
					    STATION_MODE);
			eth_zero_addr(wilc_connected_ssid);

			if (!wfi_drv->p2p_connect)
				wlan_channel = INVALID_CHANNEL;

			PRINT_ER(dev, "Unspecified failure\n");
		}

		if (connect_status == WLAN_STATUS_SUCCESS) {
			bool scan_refresh = false;
			u32 i;

			PRINT_D(vif->ndev, CFG80211_DBG,
				"Connection Successful: BSSID: %x%x%x%x%x%x\n",
				conn_info->bssid[0], conn_info->bssid[1],
				conn_info->bssid[2], conn_info->bssid[3],
				conn_info->bssid[4], conn_info->bssid[5]);
			memcpy(priv->associated_bss, conn_info->bssid,
			       ETH_ALEN);

			for (i = 0; i < last_scanned_cnt; i++) {
				if (memcmp(last_scanned_shadow[i].bssid,
					   conn_info->bssid,
					   ETH_ALEN) == 0) {
					if (wilc_wfi_cfg_scan_time_expired(i))
						scan_refresh = true;

					break;
				}
			}

			if (scan_refresh)
				refresh_scan(priv, true);
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
	} else if (conn_disconn_evt == CONN_DISCONN_EVENT_DISCONN_NOTIF)    {
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		wilc_optaining_ip = false;
#endif
		PRINT_ER(vif->ndev,
			 "Received MAC_STATUS_DISCONNECTED from firmware with reason %d on dev [%p]\n",
			 disconn_info->reason, priv->dev);
		p2p_local_random = 0x01;
		p2p_recv_random = 0x00;
		wilc_ie = false;
		eth_zero_addr(priv->associated_bss);
		wilc_wlan_set_bssid(priv->dev, null_bssid, STATION_MODE);
		eth_zero_addr(wilc_connected_ssid);

		if (!wfi_drv->p2p_connect)
			wlan_channel = INVALID_CHANNEL;
		if (wfi_drv->IFC_UP && dev == wl->vif[1]->ndev)
			disconn_info->reason = 3;
		else if (!wfi_drv->IFC_UP && dev == wl->vif[1]->ndev)
			disconn_info->reason = 1;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,2,0)
		cfg80211_disconnected(dev, disconn_info->reason,
				      disconn_info->ie, disconn_info->ie_len,
				      GFP_KERNEL);
#else
		cfg80211_disconnected(dev, disconn_info->reason,
				      disconn_info->ie, disconn_info->ie_len,
				      false, GFP_KERNEL);
#endif
	}
}

static int set_channel(struct wiphy *wiphy,
		       struct cfg80211_chan_def *chandef)
{
	u32 channelnum = 0;
	struct wilc_priv *priv;
	int result = 0;
	struct wilc_vif *vif;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);

	channelnum = ieee80211_frequency_to_channel(chandef->chan->center_freq);
	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Setting channel %d with frequency %d\n",
		   channelnum, chandef->chan->center_freq);

	curr_channel = channelnum;
	result = wilc_set_mac_chnl_num(vif, channelnum);

	if (result != 0)
		PRINT_ER(priv->dev, "Error in setting channel %d\n", channelnum);

	return result;
}

static inline int wilc_wfi_cfg_alloc_fill_ssid(struct wilc_vif *vif, 
			     struct cfg80211_scan_request *request,
			     struct hidden_network *ntwk)
{
	int i;
	int slot_id = 0;

	ntwk->net_info = kcalloc(request->n_ssids, sizeof(*ntwk->net_info),
				 GFP_KERNEL);
	if (!ntwk->net_info)
		goto out;

	ntwk->n_ssids = request->n_ssids;

	for (i = 0; i < request->n_ssids; i++) {
		if (request->ssids[i].ssid_len > 0) {
			struct hidden_net_info *info = &ntwk->net_info[slot_id];

			info->ssid = kmemdup(request->ssids[i].ssid,
					     request->ssids[i].ssid_len,
					     GFP_KERNEL);
			if (!info->ssid)
				goto out_free;

			info->ssid_len = request->ssids[i].ssid_len;
			slot_id++;
		} else {
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Received one NULL SSID\n");
			ntwk->n_ssids -= 1;
		}
	}
	return 0;

out_free:

	for (i = 0; i < slot_id ; i++)
		kfree(ntwk->net_info[i].ssid);

	kfree(ntwk->net_info);
out:

	return -ENOMEM;
}

static int scan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
{
	struct wilc_priv *priv;
	u32 i;
	s32 ret = 0;
	u8 scan_ch_list[MAX_NUM_SCANNED_NETWORKS];
	struct hidden_network hidden_ntwk;
	struct wilc_vif *vif;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);

	priv->scan_req = request;

	priv->rcvd_ch_cnt = 0;

	reset_shadow_found();

	priv->cfg_scanning = true;
	if (request->n_channels > MAX_NUM_SCANNED_NETWORKS) {
		PRINT_ER(priv->dev, "Requested scanned channels over\n");
		return -EINVAL;
	}

	for (i = 0; i < request->n_channels; i++) {
		u16 freq = request->channels[i]->center_freq;
		scan_ch_list[i] = (u8)ieee80211_frequency_to_channel(freq);
		PRINT_D(vif->ndev, CFG80211_DBG,
			"ScanChannel List[%d] = %d",
			i, scan_ch_list[i]);
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Requested num of scan channel %d\n",
		   request->n_channels);
	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Scan Request IE len =  %d\n",
		   request->ie_len);
	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Number of SSIDs %d\n",
		   request->n_ssids);
	if (request->n_ssids >= 1) {
		if (wilc_wfi_cfg_alloc_fill_ssid(vif, request,
						 &hidden_ntwk))
			return -ENOMEM;

		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Trigger Scan Request\n");
		ret = wilc_scan(vif, USER_SCAN, ACTIVE_SCAN,
				scan_ch_list,
				request->n_channels,
				(const u8 *)request->ie,
				request->ie_len, cfg_scan_result,
				(void *)priv, &hidden_ntwk);
	} else {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Trigger Scan Request\n");
		ret = wilc_scan(vif, USER_SCAN, ACTIVE_SCAN,
				scan_ch_list,
				request->n_channels,
				(const u8 *)request->ie,
				request->ie_len, cfg_scan_result,
				(void *)priv, NULL);
	}

	if (ret != 0) {
		ret = -EBUSY;
		PRINT_WRN(vif->ndev, CFG80211_DBG,
			  "Device is busy: Error(%d)\n", ret);
	}

	return ret;
}

static int connect(struct wiphy *wiphy, struct net_device *dev,
		   struct cfg80211_connect_params *sme)
{
	s32 ret = 0;
	u32 i;
	u32 sel_bssi_idx = UINT_MAX;
	u8 security = NO_ENCRYPT;
	enum AUTHTYPE auth_type = ANY;
	u32 cipher_group;
	struct wilc_priv *priv;
	struct host_if_drv *wfi_drv;
	struct network_info *nw_info = NULL;
	struct wilc_vif *vif;

	wilc_connecting = 1;
	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);
	wfi_drv = (struct host_if_drv *)priv->hif_drv;

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Connecting to SSID [%s] on netdev [%p] host if [%x]\n",
		   sme->ssid,dev, (u32)priv->hif_drv);
	if (!(strncmp(sme->ssid, "DIRECT-", 7))) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Connected to Direct network,OBSS disabled\n");
		wfi_drv->p2p_connect = 1;
	} else {
		wfi_drv->p2p_connect = 0;
	}
	PRINT_D(vif->ndev, CFG80211_DBG, "Required SSID= %s\n, AuthType= %d\n",
		sme->ssid, sme->auth_type);

	for (i = 0; i < last_scanned_cnt; i++) {
		if (sme->ssid_len == last_scanned_shadow[i].ssid_len &&
		    memcmp(last_scanned_shadow[i].ssid,
			   sme->ssid,
			   sme->ssid_len) == 0) {
			PRINT_D(vif->ndev, CFG80211_DBG,
				"Network with required SSID is found %s\n",
				sme->ssid);
			if (!sme->bssid) {
				PRINT_D(vif->ndev, CFG80211_DBG,
					"BSSID is not passed from the user\n");
				if (sel_bssi_idx == UINT_MAX ||
				    last_scanned_shadow[i].rssi >
				    last_scanned_shadow[sel_bssi_idx].rssi)
					sel_bssi_idx = i;
			} else {
				if (memcmp(last_scanned_shadow[i].bssid,
					   sme->bssid,
					   ETH_ALEN) == 0) {
					PRINT_D(vif->ndev, CFG80211_DBG,
						"BSSID is passed from the user and matched\n");
					sel_bssi_idx = i;
					break;
				}
			}
		}
	}

	if (sel_bssi_idx < last_scanned_cnt) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Required bss is in scan results\n");
		nw_info = &last_scanned_shadow[sel_bssi_idx];
		PRINT_D(vif->ndev, CFG80211_DBG,
			"network BSSID to be associated: %x%x%x%x%x%x\n",
			nw_info->bssid[0], nw_info->bssid[1],
			nw_info->bssid[2], nw_info->bssid[3],
			nw_info->bssid[4], nw_info->bssid[5]);
	} else {
		ret = -ENOENT;
		wilc_connecting = 0;
		if (last_scanned_cnt == 0)
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "No Scan results yet\n");
		else
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "Required bss not in scan results: Error(%d)\n",
				   ret);
		return ret;
	}

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
	if (cipher_group != NO_ENCRYPT) {
		PRINT_INFO(vif->ndev, CORECONFIG_DBG,
			   ">> sme->crypto.wpa_versions: %x\n",
			   sme->crypto.wpa_versions);
		if (cipher_group == WLAN_CIPHER_SUITE_WEP40) {
			security = ENCRYPT_ENABLED | WEP;
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
		} else if (cipher_group == WLAN_CIPHER_SUITE_WEP104)   {
			security = ENCRYPT_ENABLED | WEP | WEP_EXTENDED;

			priv->wep_key_len[sme->key_idx] = sme->key_len;
			memcpy(priv->wep_key[sme->key_idx], sme->key,
			       sme->key_len);

			wilc_set_wep_default_keyid(vif, sme->key_idx);
			wilc_add_wep_key_bss_sta(vif, sme->key, sme->key_len,
						 sme->key_idx);
		} else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)   {
			if (cipher_group == WLAN_CIPHER_SUITE_TKIP)
				security = ENCRYPT_ENABLED | WPA2 | TKIP;
			else
				security = ENCRYPT_ENABLED | WPA2 | AES;
		} else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_1)   {
			if (cipher_group == WLAN_CIPHER_SUITE_TKIP)
				security = ENCRYPT_ENABLED | WPA | TKIP;
			else
				security = ENCRYPT_ENABLED | WPA | AES;
		} else {
			ret = -ENOTSUPP;
			PRINT_ER(dev, "Not supported cipher\n");
			wilc_connecting = 0;
			return ret;
		}
	}

	if ((sme->crypto.wpa_versions & NL80211_WPA_VERSION_1) ||
	    (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)) {
		for (i = 0; i < sme->crypto.n_ciphers_pairwise; i++) {
			u32 ciphers_pairwise = sme->crypto.ciphers_pairwise[i];
			if (ciphers_pairwise == WLAN_CIPHER_SUITE_TKIP)
				security = security | TKIP;
			else
				security = security | AES;
		}
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG,"Adding key with cipher group %x\n",
		   cipher_group);

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Authentication Type = %d\n",
		   sme->auth_type);
	switch (sme->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
		PRINT_INFO(vif->ndev, CFG80211_DBG, "In OPEN SYSTEM\n");
		auth_type = OPEN_SYSTEM;
		break;

	case NL80211_AUTHTYPE_SHARED_KEY:
		auth_type = SHARED_KEY;
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
			auth_type = IEEE8021;
	}

	PRINT_D(vif->ndev, CFG80211_DBG, "Required Channel = %d\n",
		nw_info->ch);
	curr_channel = nw_info->ch;

	if (!wfi_drv->p2p_connect)
		wlan_channel = nw_info->ch;

	wilc_wlan_set_bssid(dev, nw_info->bssid, STATION_MODE);

	ret = wilc_set_join_req(vif, nw_info->bssid, sme->ssid,
				sme->ssid_len, sme->ie, sme->ie_len,
				cfg_connect_result, (void *)priv,
				security, auth_type,
				nw_info->ch,
				nw_info->join_params);
	if (ret != 0) {
		PRINT_ER(dev, "wilc_set_join_req(): Error(%d)\n", ret);
		ret = -ENOENT;
		wilc_connecting = 0;
		return ret;
	}

	return ret;
}

static int disconnect(struct wiphy *wiphy, struct net_device *dev,
		      u16 reason_code)
{
	s32 ret = 0;
	struct wilc_priv *priv;
	struct host_if_drv *wfi_drv;
	struct wilc_vif *vif;
	struct wilc *wilc;
	u8 null_bssid[ETH_ALEN] = {0};

	wilc_connecting = 0;
	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);
	wilc = vif->wilc;

	if (!wilc)
		return -EIO;
	wfi_drv = (struct host_if_drv *)priv->hif_drv;
	if (!wfi_drv->p2p_connect)
		wlan_channel = INVALID_CHANNEL;
	wilc_wlan_set_bssid(priv->dev, null_bssid, STATION_MODE);

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Disconnecting with reason code(%d)\n", reason_code);
	p2p_local_random = 0x01;
	p2p_recv_random = 0x00;
	wilc_ie = false;
	wfi_drv->p2p_timeout = 0;

	ret = wilc_disconnect(vif, reason_code);
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
	s32 ret = 0, keylen = params->key_len;
	struct wilc_priv *priv;
	const u8 *rx_mic = NULL;
	const u8 *tx_mic = NULL;
	u8 mode = NO_ENCRYPT;
	u8 op_mode;
	struct wilc_vif *vif;
	int i;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(netdev);

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Adding key with cipher suite = %x\n", params->cipher);
	PRINT_INFO(vif->ndev, CFG80211_DBG,"%x %x %d\n",(u32)wiphy,
		   (u32)netdev, key_index);
	PRINT_INFO(vif->ndev, CFG80211_DBG,"key %x %x %x\n",params->key[0],
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
				mode = ENCRYPT_ENABLED | WEP;
			else
				mode = ENCRYPT_ENABLED | WEP | WEP_EXTENDED;

			ret = wilc_add_wep_key_bss_ap(vif, params->key,
						      params->key_len,
						      key_index, mode,
						      OPEN_SYSTEM);
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
					mode = ENCRYPT_ENABLED | WPA | TKIP;
				else
					mode = ENCRYPT_ENABLED | WPA2 | AES;

				priv->wilc_groupkey = mode;

				key = priv->wilc_gtk[key_index];
			} else {
				PRINT_D(vif->ndev, CFG80211_DBG,
					"STA Address: %x%x%x%x%x\n",
					mac_addr[0], mac_addr[1], mac_addr[2],
					mac_addr[3], mac_addr[4]);
				if (params->cipher == WLAN_CIPHER_SUITE_TKIP)
					mode = ENCRYPT_ENABLED | WPA | TKIP;
				else
					mode = priv->wilc_groupkey | AES;

				key = priv->wilc_ptk[key_index];
			}
			ret = wilc_wfi_cfg_copy_wpa_info(key, params);
			if (ret)
				return -ENOMEM;

			op_mode = AP_MODE;
		} else {
			if (params->key_len > 16 &&
			    params->cipher == WLAN_CIPHER_SUITE_TKIP) {
				rx_mic = params->key + 24;
				tx_mic = params->key + 16;
				keylen = params->key_len - 16;
			}

			op_mode = STATION_MODE;
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
		PRINT_ER(netdev, "Not supported cipher: Error(%d)\n", ret);
		ret = -ENOTSUPP;
	}

	return ret;
}

static int del_key(struct wiphy *wiphy, struct net_device *netdev,
		   u8 key_index,
		   bool pairwise,
		   const u8 *mac_addr)
{
	struct wilc_priv *priv;
	struct wilc *wl;
	struct wilc_vif *vif;
	int ret = 0;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(netdev);
	wl = vif->wilc;

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
	struct wilc_priv *priv;
	struct  key_params key_params;
	struct wilc_vif *vif;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(netdev);

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
	struct wilc_priv *priv;
	struct wilc_vif *vif;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);

	wilc_set_wep_default_keyid(vif, key_index);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
static int get_station(struct wiphy *wiphy, struct net_device *dev,
		       const u8 *mac, struct station_info *sinfo)
#else
static int get_station(struct wiphy *wiphy, struct net_device *dev,
		       u8 *mac, struct station_info *sinfo)
#endif
{
	struct wilc_priv *priv;
	struct wilc_vif *vif;
	struct wilc *wilc;
	u32 i = 0;
	u32 associatedsta = ~0;
	u32 inactive_time = 0;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(dev);
	wilc = vif->wilc;

	if (vif->iftype == AP_MODE || vif->iftype == GO_MODE) {
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
		sinfo->filled |= BIT(NL80211_STA_INFO_INACTIVE_TIME);
#else
		sinfo->filled |= STATION_INFO_INACTIVE_TIME;
#endif

		wilc_get_inactive_time(vif, mac, &inactive_time);
		sinfo->inactive_time = 1000 * inactive_time;
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Inactive time %d\n",
			   sinfo->inactive_time);
	} else if (vif->iftype == STATION_MODE) {
		struct rf_info stats;

		if (!wilc->initialized) {
			PRINT_INFO(vif->ndev, CFG80211_DBG,
				   "driver not initialized\n");
			return -EBUSY;
		}
		wilc_get_statistics(vif, &stats);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
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
			wilc_enable_tcp_ack_filter(true);
		else if (stats.link_speed != DEFAULT_LINK_SPEED)
			wilc_enable_tcp_ack_filter(false);

		PRINT_INFO(vif->ndev, CORECONFIG_DBG,
			   "*** stats[%d][%d][%d][%d][%d]\n",sinfo->signal,
			   sinfo->rx_packets,sinfo->tx_packets,
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
	s32 ret = 0;
	struct cfg_param_attr cfg_param_val;
	struct wilc_priv *priv;
	struct wilc_vif *vif;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);

	cfg_param_val.flag = 0;
	PRINT_INFO(vif->ndev, CFG80211_DBG, "Setting Wiphy params\n");

	if (changed & WIPHY_PARAM_RETRY_SHORT) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Setting WIPHY_PARAM_RETRY_SHORT %d\n",
			   wiphy->retry_short);
		cfg_param_val.flag  |= RETRY_SHORT;
		cfg_param_val.short_retry_limit = wiphy->retry_short;
	}
	if (changed & WIPHY_PARAM_RETRY_LONG) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Setting WIPHY_PARAM_RETRY_LONG %d\n",
			   wiphy->retry_long);
		cfg_param_val.flag |= RETRY_LONG;
		cfg_param_val.long_retry_limit = wiphy->retry_long;
	}
	if (changed & WIPHY_PARAM_FRAG_THRESHOLD) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Setting WIPHY_PARAM_FRAG_THRESHOLD %d\n",
			   wiphy->frag_threshold);
		cfg_param_val.flag |= FRAG_THRESHOLD;
		cfg_param_val.frag_threshold = wiphy->frag_threshold;
	}

	if (changed & WIPHY_PARAM_RTS_THRESHOLD) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Setting WIPHY_PARAM_RTS_THRESHOLD %d\n",
			   wiphy->rts_threshold);
		cfg_param_val.flag |= RTS_THRESHOLD;
		cfg_param_val.rts_threshold = wiphy->rts_threshold;
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
	u32 i;
	s32 ret = 0;
	u8 flag = 0;
	struct wilc_vif *vif;
	struct wilc_priv *priv = wiphy_priv(wiphy);

	vif = netdev_priv(priv->dev);
	
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
		       PMKID_LEN);
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
	s32 ret = 0;

	struct wilc_priv *priv = wiphy_priv(wiphy);

	PRINT_INFO(netdev, CFG80211_DBG, "Deleting PMKSA keys\n");

	for (i = 0; i < priv->pmkid_list.numpmkid; i++)	{
		if (!memcmp(pmksa->bssid, priv->pmkid_list.pmkidlist[i].bssid,
			    ETH_ALEN)) {
			PRINT_INFO(netdev, CFG80211_DBG,
				   "Reseting PMKID values\n");
			memset(&priv->pmkid_list.pmkidlist[i], 0,
			       sizeof(struct host_if_pmkid));
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
			       PMKID_LEN);
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
	memset(&priv->pmkid_list, 0, sizeof(struct host_if_pmkid_attr));

	return 0;
}

static inline void wilc_wfi_cfg_parse_ch_attr(struct wilc_vif *vif, u8 *buf,
					      u8 ch_list_attr_idx,
					      u8 op_ch_attr_idx)
{
	int i = 0;
	int j = 0;

	if (ch_list_attr_idx) {
		u8 limit = ch_list_attr_idx + 3 + buf[ch_list_attr_idx + 1];

		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Modify channel list attribute\n");
		for (i = ch_list_attr_idx + 3; i < limit; i++) {
			if (buf[i] == 0x51) {
				for (j = i + 2; j < ((i + 2) + buf[i + 1]); j++)
					buf[j] = wlan_channel;
				break;
			}
		}
	}

	if (op_ch_attr_idx) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Modify operating channel attribute\n");
		buf[op_ch_attr_idx + 6] = 0x51;
		buf[op_ch_attr_idx + 7] = wlan_channel;
	}
}

static void wilc_wfi_cfg_parse_rx_action(struct wilc_vif *vif, u8 * buf,
					 u32 len, bool p2p_mode)
{
	u32 index = 0;

	u8 op_channel_attr_index = 0;
	u8 channel_list_attr_index = 0;

	while (index < len) {
		if (buf[index] == GO_INTENT_ATTR_ID) {
			if(!p2p_mode)
				buf[index + 3] = (buf[index + 3]  & 0x01) | (0x0f << 1);
			else
				buf[index + 3] = (buf[index + 3]  & 0x01) | (0x00 << 1);
		}
		if (buf[index] ==  CHANLIST_ATTR_ID)
			channel_list_attr_index = index;
		else if (buf[index] ==  OPERCHAN_ATTR_ID)
			op_channel_attr_index = index;
		index += buf[index + 1] + 3;
	}
	if (wlan_channel != INVALID_CHANNEL)
		wilc_wfi_cfg_parse_ch_attr(vif, buf, channel_list_attr_index,
					   op_channel_attr_index);
}

static void wilc_wfi_cfg_parse_tx_action(struct wilc_vif *vif, u8 * buf,
					 u32 len,bool oper_ch, u8 p2p_mode)
{
	u32 index = 0;

	u8 op_channel_attr_index = 0;
	u8 channel_list_attr_index = 0;

	while (index < len) {
		if (buf[index] == GO_INTENT_ATTR_ID) {
			if(!p2p_mode)
				buf[index + 3] = (buf[index + 3]  & 0x01) | (0x00 << 1);
			else
				buf[index + 3] = (buf[index + 3]  & 0x01) | (0x0f << 1);
			break;
		}

		if (buf[index] ==  CHANLIST_ATTR_ID)
			channel_list_attr_index = index;
		else if (buf[index] ==  OPERCHAN_ATTR_ID)
			op_channel_attr_index = index;
		index += buf[index + 1] + 3;
	}
	if (wlan_channel != INVALID_CHANNEL && oper_ch)
		wilc_wfi_cfg_parse_ch_attr(vif, buf, channel_list_attr_index,
					   op_channel_attr_index);
}

static void wilc_wfi_cfg_parse_rx_vendor_spec(struct wilc_priv *priv, u8 *buff,
					      u32 size)
{
	int i;
	u8 subtype;
	struct wilc_vif *vif = netdev_priv(priv->dev);

	subtype = buff[P2P_PUB_ACTION_SUBTYPE];
	if ((subtype == GO_NEG_REQ || subtype == GO_NEG_RSP) && !wilc_ie) {
		for (i = P2P_PUB_ACTION_SUBTYPE; i < size; i++) {
			if (!memcmp(p2p_vendor_spec, &buff[i], 6)) {
				p2p_recv_random = buff[i + 6];
				wilc_ie = true;
				PRINT_INFO(vif->ndev, GENERIC_DBG,
					   "WILC Vendor specific IE:%02x\n",
					   p2p_recv_random);
				break;
			}
		}
	}

	if (p2p_local_random <= p2p_recv_random) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "PEER WILL BE GO LocaRand=%02x RecvRand %02x\n",
			   p2p_local_random, p2p_recv_random);
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
							     p2p_mode);
				break;
			}
		}
	}
}

void wilc_wfi_p2p_rx(struct net_device *dev, u8 *buff, u32 size)
{
	struct wilc_priv *priv;
	struct wilc_vif *vif;
	u32 header, pkt_offset;
	struct host_if_drv *wfi_drv;
	s32 s32Freq;

	vif = netdev_priv(dev);
	priv = wiphy_priv(dev->ieee80211_ptr->wiphy);
	wfi_drv = (struct host_if_drv *)priv->hif_drv;

	memcpy(&header, (buff - HOST_HDR_OFFSET), HOST_HDR_OFFSET);

	pkt_offset = GET_PKT_OFFSET(header);

	if (pkt_offset & IS_MANAGMEMENT_CALLBACK) {
		bool ack = false;

		if (buff[FRAME_TYPE_ID] == IEEE80211_STYPE_PROBE_RESP ||
		    pkt_offset & IS_MGMT_STATUS_SUCCES)
			ack = true;

		cfg80211_mgmt_tx_status(priv->wdev, priv->tx_cookie, buff, size,
					ack, GFP_KERNEL);
		return;
	}

	PRINT_D(vif->ndev, GENERIC_DBG, "Rx Frame Type:%x\n",
		   buff[FRAME_TYPE_ID]);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0)
	s32Freq = ieee80211_channel_to_frequency(curr_channel, NL80211_BAND_2GHZ);
 #else
	s32Freq = ieee80211_channel_to_frequency(curr_channel, IEEE80211_BAND_2GHZ);
 #endif
	if (!ieee80211_is_action(buff[FRAME_TYPE_ID])) {
		cfg80211_rx_mgmt(priv->wdev, s32Freq, 0, buff, size, 0);
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
			    wilc_ie)
				size -= 7;

			break;

		default:
			PRINT_WRN(dev, GENERIC_DBG, 
				   "NOT HANDLED PUBLIC ACTION FRAME TYPE:%x\n",
				   buff[ACTION_SUBTYPE_ID]);
			break;
		}
	}

	cfg80211_rx_mgmt(priv->wdev, s32Freq, 0, buff, size, 0);
}

static void wilc_wfi_mgmt_tx_complete(void *priv, int status)
{
	struct p2p_mgmt_data *pv_data = priv;

	kfree(pv_data->buff);
	kfree(pv_data);
}

static void wilc_wfi_remain_on_channel_ready(void *priv_data)
{
	struct wilc_priv *priv;

	priv = priv_data;

	PRINT_INFO(priv->dev, HOSTINF_DBG, "Remain on channel ready\n");
	priv->p2p_listen_state = true;

	cfg80211_ready_on_channel(priv->wdev,
				  priv->remain_on_ch_params.listen_cookie,
				  priv->remain_on_ch_params.listen_ch,
				  priv->remain_on_ch_params.listen_duration,
				  GFP_KERNEL);
}

static void wilc_wfi_remain_on_channel_expired(void *data, u32 session_id)
{
	struct wilc_priv *priv = data;
	struct wilc_wfi_p2p_listen_params *params = &priv->remain_on_ch_params;

	if (session_id != params->listen_session_id) {
		PRINT_INFO(priv->dev, GENERIC_DBG,
			   "Received ID 0x%x Expected ID 0x%x (No match)\n",
			   session_id,
			   priv->remain_on_ch_params.listen_session_id);
		return;
	}


	PRINT_INFO(priv->dev, GENERIC_DBG,
		   "Remain on channel expired\n");
	priv->p2p_listen_state = false;

	cfg80211_remain_on_channel_expired(priv->wdev, params->listen_cookie,
					   params->listen_ch, GFP_KERNEL);
}

static int remain_on_channel(struct wiphy *wiphy,
			     struct wireless_dev *wdev,
			     struct ieee80211_channel *chan,
			     unsigned int duration, u64 *cookie)
{
	s32 ret = 0;
	struct wilc_priv *priv;
	struct wilc_vif *vif;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);
	PRINT_INFO(vif->ndev, GENERIC_DBG, "Remaining on channel %d\n", chan->hw_value);

	if (wdev->iftype == NL80211_IFTYPE_AP) {
		PRINT_INFO(vif->ndev, GENERIC_DBG, "Required while in AP mode\n");
		return ret;
	}

	curr_channel = chan->hw_value;

	priv->remain_on_ch_params.listen_ch = chan;
	priv->remain_on_ch_params.listen_cookie = *cookie;
	priv->remain_on_ch_params.listen_duration = duration;
	priv->remain_on_ch_params.listen_session_id++;

	return wilc_remain_on_channel(vif,
				priv->remain_on_ch_params.listen_session_id,
				duration, chan->hw_value,
				wilc_wfi_remain_on_channel_expired,
				wilc_wfi_remain_on_channel_ready, (void *)priv);
}

static int cancel_remain_on_channel(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    u64 cookie)
{
	struct wilc_priv *priv;
	struct wilc_vif *vif;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);
	PRINT_INFO(vif->ndev, CFG80211_DBG, "Cancel remain on channel\n");

	return wilc_listen_state_expired(vif,
			priv->remain_on_ch_params.listen_session_id);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
static void wilc_wfi_cfg_tx_vendor_spec(struct wilc_vif *vif,
					struct p2p_mgmt_data *mgmt_tx,
					struct cfg80211_mgmt_tx_params *params,
					u8 iftype, u32 buf_len)
#else
static void wilc_wfi_cfg_tx_vendor_spec(struct wilc_vif *vif,
					struct p2p_mgmt_data *mgmt_tx,
					const u8 *buf, size_t len,
					u8 iftype, u32 buf_len)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
	const u8 *buf = params->buf;
	size_t len = params->len;
#endif
	u32 i;
	u8 subtype = buf[P2P_PUB_ACTION_SUBTYPE];

	if (subtype == GO_NEG_REQ || subtype == GO_NEG_RSP) {
		if (p2p_local_random == 1 &&
		    p2p_recv_random < p2p_local_random) {
			get_random_bytes(&p2p_local_random, 1);
			p2p_local_random++;
		}
	}

	if (p2p_local_random <= p2p_recv_random || !(subtype == GO_NEG_REQ ||
						     subtype == GO_NEG_RSP ||
						     subtype == P2P_INV_REQ ||
						     subtype == P2P_INV_RSP))
		return;

	PRINT_INFO(vif->ndev, GENERIC_DBG, 
		   "LOCAL WILL BE GO LocaRand=%02x RecvRand %02x\n",
		   p2p_local_random, p2p_recv_random);
	for (i = P2P_PUB_ACTION_SUBTYPE + 2; i < len; i++) {
		if (buf[i] == P2PELEM_ATTR_ID &&
		    !memcmp(p2p_oui, &buf[i + 2], 4)) {
			bool oper_ch = false;
			u8 *tx_buff = &mgmt_tx->buff[i + 6];

			if (subtype == P2P_INV_REQ || subtype == P2P_INV_RSP)
				oper_ch = true;
			
			wilc_wfi_cfg_parse_tx_action(vif, tx_buff,
						     len - (i + 6), oper_ch,
						     vif->attr_sysfs.p2p_mode);
			break;
		}
	}

	if (subtype != P2P_INV_REQ && subtype != P2P_INV_RSP) {
		int vendor_spec_len = sizeof(p2p_vendor_spec);

		memcpy(&mgmt_tx->buff[len], p2p_vendor_spec,
		       vendor_spec_len);
		mgmt_tx->buff[len + vendor_spec_len] = p2p_local_random;
		mgmt_tx->size = buf_len;
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
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
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
	struct ieee80211_channel *chan = params->chan;
	unsigned int wait = params->wait;
	const u8 *buf = params->buf;
	size_t len = params->len;
#endif
	const struct ieee80211_mgmt *mgmt;
	struct p2p_mgmt_data *mgmt_tx;
	struct wilc_priv *priv;
	struct host_if_drv *wfi_drv;
	struct wilc_vif *vif;
	u32 buf_len = len + sizeof(p2p_vendor_spec) + sizeof(p2p_local_random);
	int ret = 0;

	vif = netdev_priv(wdev->netdev);
	priv = wiphy_priv(wiphy);
	wfi_drv = (struct host_if_drv *)priv->hif_drv;

	*cookie = (unsigned long)buf;
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
		curr_channel = chan->hw_value;
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
			curr_channel = chan->hw_value;
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
			#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
				wilc_wfi_cfg_tx_vendor_spec(vif, mgmt_tx,
							    params,
							    vif->iftype,
							    buf_len);
			#else
				wilc_wfi_cfg_tx_vendor_spec(vif, mgmt_tx, buf,
							    len, vif->iftype,
							    buf_len);
			#endif
			else
				PRINT_INFO(vif->ndev, GENERIC_DBG, "Not a P2P public action frame\n");

			break;

		default:
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "NOT HANDLED PUBLIC ACTION FRAME TYPE:%x\n",
				   buf[ACTION_SUBTYPE_ID]);
			break;
		}
	}

	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "TX: ACTION FRAME Type:%x : Chan:%d\n",
		   buf[ACTION_SUBTYPE_ID], chan->hw_value);
	wfi_drv->p2p_timeout = (jiffies + msecs_to_jiffies(wait));

out_txq_add_pkt:

	wilc_wlan_txq_add_mgmt_pkt(wdev->netdev, mgmt_tx,
				   mgmt_tx->buff, mgmt_tx->size,
				   wilc_wfi_mgmt_tx_complete);

out:

	return ret;
}

static int mgmt_tx_cancel_wait(struct wiphy *wiphy,
			       struct wireless_dev *wdev,
			       u64 cookie)
{
	struct wilc_priv *priv;
	struct host_if_drv *wfi_drv;

	priv = wiphy_priv(wiphy);
	wfi_drv = (struct host_if_drv *)priv->hif_drv;
	PRINT_INFO(priv->dev, CFG80211_DBG, "Tx Cancel wait :%lu\n", jiffies);
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
	struct wilc_priv *priv;
	struct wilc_vif *vif;
	struct wilc *wl;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->wdev->netdev);
	wl = vif->wilc;

	if (!frame_type)
		return;

	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "Frame registering Frame Type: %x: Boolean: %d\n",
		   frame_type,reg);
	switch (frame_type) {
	case PROBE_REQ:
		vif->frame_reg[0].type = frame_type;
		vif->frame_reg[0].reg = reg;
		break;

	case ACTION:
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
	struct wilc_priv *priv;
	struct wilc_vif *vif;

	if (idx != 0)
		return -ENOENT;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Dumping station information\n");

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
	sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL);
#else
	sinfo->filled |= STATION_INFO_SIGNAL;
#endif

	wilc_get_rssi(vif, &sinfo->signal);

	return 0;
}

static int set_power_mgmt(struct wiphy *wiphy, struct net_device *dev,
			  bool enabled, int timeout)
{
	struct wilc_priv *priv;
	struct wilc_vif *vif;

	if (!wiphy)
		return -ENOENT;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);
	if (!priv->hif_drv) {
		PRINT_ER(dev, "Driver is NULL\n");
		return -EIO;
	}

	/* Can't set PS during obtaining IP */
	if (wilc_optaining_ip == true)
	{
		PRINT_ER(dev, "Device obtaining IP, Power Managment will be handled after IP Obtained\n");
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Save the Current state of the PS = %d\n", enabled);

		/* Save the current status of the PS */
		store_power_save_current_state(vif, enabled);
			
		return 0;
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG, " Power save Enabled= %d , TimeOut = %d\n", enabled, timeout);

	if (wilc_enable_ps)
		wilc_set_power_mgmt(vif, enabled, timeout);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
static int change_virtual_intf(struct wiphy *wiphy, struct net_device *dev,
			       enum nl80211_iftype type,
			       struct vif_params *params)
#else
static int change_virtual_intf(struct wiphy *wiphy, struct net_device *dev,
			       enum nl80211_iftype type, u32 *flags, 
			       struct vif_params *params)
#endif
{
	struct wilc_priv *priv;
	struct wilc_vif *vif;
	struct wilc_vif *vif_1;
	struct wilc_vif *vif_2;
	struct wilc *wl;
	struct net_device *net_device_1;
	struct net_device *net_device_2;
	struct wilc_priv* priv_1;
	struct wilc_priv* priv_2;

	vif = netdev_priv(dev);
	priv = wiphy_priv(wiphy);
	wl = vif->wilc;
	net_device_1 = wilc_get_if_netdev(wl, P2P_IFC);
	net_device_2 = wilc_get_if_netdev(wl, WLAN_IFC);
	priv_1 = wdev_priv(net_device_1->ieee80211_ptr);
	priv_2 = wdev_priv(net_device_2->ieee80211_ptr);
	vif_1 = netdev_priv(net_device_1);
	vif_2 = netdev_priv(net_device_2);
	
	PRINT_INFO(vif->ndev, HOSTAPD_DBG,
		   "In Change virtual interface function\n");
	PRINT_INFO(vif->ndev, HOSTAPD_DBG,
		   "Wireless interface name =%s\n", dev->name);
	p2p_local_random = 0x01;
	p2p_recv_random = 0x00;
	wilc_ie = false;
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	PRINT_INFO(vif->ndev, GENERIC_DBG,
		   "Changing virtual interface, enable scan\n");
	handle_pwrsave_during_obtainingIP(vif, IP_STATE_DEFAULT);
#endif

	switch (type) {
	case NL80211_IFTYPE_STATION:
		wilc_connecting = 0;
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_STATION\n");
		dev->ieee80211_ptr->iftype = type;
		priv->wdev->iftype = type;
		vif->monitor_flag = 0;
		vif->iftype = STATION_MODE;
		wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
					 STATION_MODE, vif->ifc_id);
		wilc_set_operation_mode(vif, STATION_MODE);

		memset(priv->assoc_stainfo.sta_associated_bss, 0,
		       MAX_NUM_STA * ETH_ALEN);

		wilc_enable_ps = true;
		wilc_set_power_mgmt(vif_1, 1, 0);
		wilc_set_power_mgmt(vif_2, 1, 0);
		break;

	case NL80211_IFTYPE_P2P_CLIENT:
		wilc_connecting = 0;
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_P2P_CLIENT\n");
		dev->ieee80211_ptr->iftype = type;
		priv->wdev->iftype = type;
		vif->monitor_flag = 0;
		vif->iftype = CLIENT_MODE;
		wilc_enable_ps = false;
		wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
					 STATION_MODE, vif->ifc_id);
		wilc_set_operation_mode(vif, STATION_MODE);

		wilc_set_power_mgmt(vif_1, 0, 0);
		wilc_set_power_mgmt(vif_2, 0, 0);
		break;

	case NL80211_IFTYPE_AP:
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_AP\n");
		dev->ieee80211_ptr->iftype = type;
		priv->wdev->iftype = type;
		vif->iftype = AP_MODE;
		wilc_enable_ps = false;
		if (wl->initialized) {
			wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
						 AP_MODE, vif->ifc_id);
			wilc_set_operation_mode(vif, AP_MODE);
			wilc_set_power_mgmt(vif_1, 0, 0);
			wilc_set_power_mgmt(vif_2, 0, 0);
		}
		break;

	case NL80211_IFTYPE_P2P_GO:
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Interface type = NL80211_IFTYPE_GO\n");
		PRINT_INFO(vif->ndev, GENERIC_DBG, "start duringIP timer\n");

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		handle_pwrsave_during_obtainingIP(vif, IP_STATE_GO_ASSIGNING);
#endif
		dev->ieee80211_ptr->iftype = type;
		priv->wdev->iftype = type;
		vif->iftype = GO_MODE;
		wilc_set_wfi_drv_handler(vif, wilc_get_vif_idx(vif),
						 AP_MODE, vif->ifc_id);
		wilc_set_operation_mode(vif, AP_MODE);
		wilc_enable_ps = false;
		wilc_set_power_mgmt(vif_1, 0, 0);
		wilc_set_power_mgmt(vif_2, 0, 0);
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
	struct cfg80211_beacon_data *beacon = &settings->beacon;
	s32 ret = 0;
	struct wilc_vif *vif;

	vif = netdev_priv(dev);
	PRINT_INFO(vif->ndev, HOSTAPD_DBG,"Starting ap\n");

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Interval= %d\n DTIM period= %d\n Head length= %d Tail length= %d\n",
		   settings->beacon_interval, settings->dtim_period,
		   beacon->head_len,beacon->tail_len);
	ret = set_channel(wiphy, &settings->chandef);

	if (ret != 0)
		PRINT_ER(dev, "Error in setting channel\n");

	wilc_wlan_set_bssid(dev,vif->src_addr,AP_MODE);
	wilc_set_power_mgmt(vif, 0, 0);

	return wilc_add_beacon(vif, settings->beacon_interval,
				   settings->dtim_period, beacon->head_len,
				   (u8 *)beacon->head, beacon->tail_len,
				   (u8 *)beacon->tail);
}

static int change_beacon(struct wiphy *wiphy, struct net_device *dev,
			 struct cfg80211_beacon_data *beacon)
{
	struct wilc_priv *priv;
	struct wilc_vif *vif;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);
	PRINT_INFO(vif->ndev, HOSTAPD_DBG, "Setting beacon\n");

	return wilc_add_beacon(vif, 0, 0, beacon->head_len,
				   (u8 *)beacon->head, beacon->tail_len,
				   (u8 *)beacon->tail);
}

static int stop_ap(struct wiphy *wiphy, struct net_device *dev)
{
	s32 ret = 0;
	struct wilc_priv *priv;
	struct wilc_vif *vif;
	u8 null_bssid[ETH_ALEN] = {0};

	if (!wiphy)
		return -EFAULT;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->dev);
	PRINT_INFO(vif->ndev, CFG80211_DBG, "Deleting beacon\n");

	wilc_wlan_set_bssid(dev, null_bssid, AP_MODE);

	ret = wilc_del_beacon(vif);

	if (ret)
		PRINT_ER(dev, "Host delete beacon fail\n");

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
static int add_station(struct wiphy *wiphy, struct net_device *dev,
		       const u8 *mac, struct station_parameters *params)
#else
static int add_station(struct wiphy *wiphy, struct net_device *dev,
		       u8 *mac, struct station_parameters *params)
#endif
{
	s32 ret = 0;
	struct wilc_priv *priv;
	struct add_sta_param sta_params = { {0} };
	struct wilc_vif *vif;

	if (!wiphy)
		return -EFAULT;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(dev);

	if (vif->iftype == AP_MODE || vif->iftype == GO_MODE) {
		memcpy(sta_params.bssid, mac, ETH_ALEN);
		memcpy(priv->assoc_stainfo.sta_associated_bss[params->aid], mac,
		       ETH_ALEN);
		sta_params.aid = params->aid;
		sta_params.rates_len = params->supported_rates_len;
		sta_params.rates = params->supported_rates;

		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Adding station parameters %d\n", params->aid);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "BSSID = %x%x%x%x%x%x\n",
			   priv->assoc_stainfo.sta_associated_bss[params->aid][0],
			   priv->assoc_stainfo.sta_associated_bss[params->aid][1],
			   priv->assoc_stainfo.sta_associated_bss[params->aid][2],
			   priv->assoc_stainfo.sta_associated_bss[params->aid][3],
			   priv->assoc_stainfo.sta_associated_bss[params->aid][4],
			   priv->assoc_stainfo.sta_associated_bss[params->aid][5]);
		PRINT_INFO(vif->ndev, HOSTAPD_DBG, "ASSOC ID = %d\n",
			   sta_params.aid);
		PRINT_INFO(vif->ndev, HOSTAPD_DBG,
			   "Number of supported rates = %d\n",
			   sta_params.rates_len);
		if (!params->ht_capa) {
			sta_params.ht_supported = false;
		} else {
			sta_params.ht_supported = true;
			sta_params.ht_capa = *params->ht_capa;
		}

		sta_params.flags_mask = params->sta_flags_mask;
		sta_params.flags_set = params->sta_flags_set;

		PRINT_INFO(vif->ndev, CFG80211_DBG, "IS HT supported = %d\n",
			   sta_params.ht_supported);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Capability Info = %d\n",
			   sta_params.ht_capa.cap_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "AMPDU Params = %d\n",
			   sta_params.ht_capa.ampdu_params_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "HT Extended params= %d\n",
			   sta_params.ht_capa.extended_ht_cap_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Tx Beamforming Cap= %d\n",
			   sta_params.ht_capa.tx_BF_cap_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Antenna selection info = %d\n",
			   sta_params.ht_capa.antenna_selection_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Flag Mask = %d\n",
			   sta_params.flags_mask);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Flag Set = %d\n",
			   sta_params.flags_set);
		ret = wilc_add_station(vif, &sta_params);
		if (ret)
			PRINT_ER(dev, "Host add station fail\n");
	}

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
static int del_station(struct wiphy *wiphy, struct net_device *dev,
		       struct station_del_parameters *params)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
static int del_station(struct wiphy *wiphy, struct net_device *dev,
		       const u8 *mac)
#else
static int del_station(struct wiphy *wiphy, struct net_device *dev,
		       u8 *mac)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	const u8 *mac = params->mac;
#endif
	s32 ret = 0;
	struct wilc_priv *priv;
	struct wilc_vif *vif;
	struct sta_info *info;

	if (!wiphy)
		return -EFAULT;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(dev);

	if (!(vif->iftype == AP_MODE || vif->iftype == GO_MODE))
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
static int change_station(struct wiphy *wiphy, struct net_device *dev,
			  const u8 *mac, struct station_parameters *params)
#else
static int change_station(struct wiphy *wiphy, struct net_device *dev,
			  u8 *mac, struct station_parameters *params)
#endif
{
	s32 ret = 0;
	struct add_sta_param sta_params = { {0} };
	struct wilc_vif *vif;

	PRINT_D(vif->ndev, CFG80211_DBG, "Change station paramters\n");

	if (!wiphy)
		return -EFAULT;

	vif = netdev_priv(dev);

	if (vif->iftype == AP_MODE || vif->iftype == GO_MODE) {
		memcpy(sta_params.bssid, mac, ETH_ALEN);
		sta_params.aid = params->aid;
		sta_params.rates_len = params->supported_rates_len;
		sta_params.rates = params->supported_rates;

		PRINT_INFO(vif->ndev, CFG80211_DBG, "BSSID = %x%x%x%x%x%x\n",
			  sta_params.bssid[0], sta_params.bssid[1],
			  sta_params.bssid[2], sta_params.bssid[3],
			  sta_params.bssid[4],
			  sta_params.bssid[5]);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "ASSOC ID = %d\n",
			   sta_params.aid);
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Number of supported rates = %d\n",
			   sta_params.rates_len);
		if (!params->ht_capa) {
			sta_params.ht_supported = false;
		} else {
			sta_params.ht_supported = true;
			sta_params.ht_capa = *params->ht_capa;
		}

		sta_params.flags_mask = params->sta_flags_mask;
		sta_params.flags_set = params->sta_flags_set;

		PRINT_INFO(vif->ndev, CFG80211_DBG, "IS HT supported = %d\n",
			   sta_params.ht_supported);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Capability Info = %d\n",
			   sta_params.ht_capa.cap_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "AMPDU Params = %d\n",
			   sta_params.ht_capa.ampdu_params_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "HT Extended params= %d\n",
			   sta_params.ht_capa.extended_ht_cap_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Tx Beamforming Cap= %d\n",
			   sta_params.ht_capa.tx_BF_cap_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Antenna selection info = %d\n",
			   sta_params.ht_capa.antenna_selection_info);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Flag Mask = %d\n",
			   sta_params.flags_mask);
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Flag Set = %d\n",
			   sta_params.flags_set);
		ret = wilc_edit_station(vif, &sta_params);
		if (ret)
			PRINT_ER(dev, "Host edit station fail\n");
	}
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
static struct wireless_dev *add_virtual_intf(struct wiphy *wiphy,
					     const char *name,
					     unsigned char name_assign_type,
					     enum nl80211_iftype type,
					     struct vif_params *params)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
static struct wireless_dev *add_virtual_intf(struct wiphy *wiphy,
					     const char *name,
					     unsigned char name_assign_type,
					     enum nl80211_iftype type,
					     u32 *flags,
					     struct vif_params *params)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
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
	struct wilc_vif *vif;
	struct wilc_priv *priv;
	struct net_device *new_ifc = NULL;

	priv = wiphy_priv(wiphy);
	vif = netdev_priv(priv->wdev->netdev);

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Adding monitor interface[%p]\n",
		   priv->wdev->netdev);

	if (type == NL80211_IFTYPE_MONITOR) {
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Initializing mon ifc virtual device driver\n");
		PRINT_INFO(vif->ndev, CFG80211_DBG, "Adding monitor interface[%p]\n", vif->ndev);
		new_ifc = wilc_wfi_init_mon_interface(name, vif->ndev);
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

	if(!wow)
		PRINT_INFO(priv->dev, GENERIC_DBG, "No wake up triggers defined\n");
	else if(wow->any == 0)
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
	host_int_set_wowlan_trigger(vif,(u8)enabled);
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
	struct wilc *wl;

	wl = vif->wilc;

	/* If firmware is not started, return. */
	if (!wl->initialized)
		return -EIO;
	*dbm=0;
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

	PRINT_INFO(vif->ndev, CFG80211_DBG,"Select antenna mode %d\n",tx_ant);
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
		goto _fail_;
	}

	wdev->wiphy = wiphy_new(&wilc_cfg80211_ops, sizeof(struct wilc_priv));
	if (!wdev->wiphy) {
		PRINT_ER(net, "Cannot allocate wiphy\n");
		goto _fail_mem_;
	}

	wilc_band_2ghz.ht_cap.ht_supported = 1;
	wilc_band_2ghz.ht_cap.cap |= (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);
	wilc_band_2ghz.ht_cap.mcs.rx_mask[0] = 0xff;
	wilc_band_2ghz.ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_8K;
	wilc_band_2ghz.ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	wdev->wiphy->bands[NL80211_BAND_2GHZ] = &wilc_band_2ghz;
#else
	wdev->wiphy->bands[IEEE80211_BAND_2GHZ] = &wilc_band_2ghz;
#endif
	return wdev;

_fail_mem_:
	kfree(wdev);
_fail_:
	return NULL;
}

struct wireless_dev *wilc_create_wiphy(struct net_device *net,
				       struct device *dev)
{
	struct wilc_priv *priv;
	struct wireless_dev *wdev;
	s32 ret = 0;

	PRINT_INFO(net, CFG80211_DBG, "Registering wifi device\n");
	wdev = wilc_wfi_cfg_alloc(net);
	if (!wdev) {
		PRINT_ER(net, "wiphy new allocate failed\n");
		return NULL;
	}

	priv = wdev_priv(wdev);
	priv->wdev = wdev;
	wdev->wiphy->max_scan_ssids = MAX_NUM_PROBED_SSID;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,11,0)
	wdev->wiphy->wowlan = &wowlan_support;
#else
	wdev->wiphy->wowlan = wowlan_support;
#endif
	wdev->wiphy->max_num_pmkids = WILC_MAX_NUM_PMKIDS;
	PRINT_D(net, CFG80211_DBG, "Max number of PMKIDs = %d\n",
		wdev->wiphy->max_num_pmkids);
	wdev->wiphy->max_scan_ie_len = 1000;
	wdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wdev->wiphy->cipher_suites = cipher_suites;
	wdev->wiphy->n_cipher_suites = ARRAY_SIZE(cipher_suites);
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
		wdev->wiphy->max_scan_ssids, wdev->wiphy->max_scan_ie_len, wdev->wiphy->signal_type,
		wdev->wiphy->interface_modes, wdev->iftype);

	set_wiphy_dev(wdev->wiphy, dev);

	ret = wiphy_register(wdev->wiphy);
	if (ret)
		PRINT_ER(net, "Cannot register wiphy device\n");
	else
		PRINT_INFO(net, CFG80211_DBG, "Successful Registering\n");

	priv->dev = net;
	return wdev;
}

int wilc_init_host_int(struct net_device *net)
{
	int ret = 0;
	struct wilc_priv *priv;

	PRINT_INFO(net, INIT_DBG, "Host[%p][%p]\n", net, net->ieee80211_ptr);
	priv = wdev_priv(net->ieee80211_ptr);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)
	#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	timer_setup(&priv->during_ip_timer, clear_during_ip, 0);
	#endif
	timer_setup(&priv->eap_buff_timer, eap_buff_timeout, 0);
#else
	#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	setup_timer(&priv->during_ip_timer, clear_during_ip, 0);
	#endif
	setup_timer(&priv->eap_buff_timer, eap_buff_timeout, 0);
#endif
	op_ifcs++;

	priv->auto_rate_adjusted = false;

	priv->p2p_listen_state = false;

	mutex_init(&priv->scan_req_lock);
	ret = wilc_init(net, &priv->hif_drv);
	if (ret)
		PRINT_ER(net, "Error while initializing hostinterface\n");

	return ret;
}

int wilc_deinit_host_int(struct net_device *net)
{
	int ret = 0;
	struct wilc_vif *vif;
	struct wilc_priv *priv;

	priv = wdev_priv(net->ieee80211_ptr);
	vif = netdev_priv(priv->dev);

	priv->auto_rate_adjusted = false;

	priv->p2p_listen_state = false;

	op_ifcs--;

	mutex_destroy(&priv->scan_req_lock);
	ret = wilc_deinit(vif);

	clear_shadow_scan(vif);
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	del_timer_sync(&priv->during_ip_timer);
#endif
	del_timer_sync(&priv->eap_buff_timer);

	if (ret)
		PRINT_ER(net, "Error while deinitializing host interface\n");

	return ret;
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
