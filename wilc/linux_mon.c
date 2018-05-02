// SPDX-License-Identifier: GPL-2.0
#include "wilc_wfi_cfgoperations.h"
#include "wilc_wlan_if.h"
#include "wilc_wlan.h"

struct wilc_wfi_radiotap_hdr {
	struct ieee80211_radiotap_header hdr;
	u8 rate;
} __packed;

struct wilc_wfi_radiotap_cb_hdr {
	struct ieee80211_radiotap_header hdr;
	u8 rate;
	u8 dump;
	u16 tx_flags;
} __packed;

static struct net_device *wilc_wfi_mon; /* global monitor netdev */

static u8 srcadd[6];
static u8 bssid[6];

#define IEEE80211_RADIOTAP_F_TX_RTS	0x0004  /* used rts/cts handshake */
#define IEEE80211_RADIOTAP_F_TX_FAIL	0x0001  /* failed due to excessive*/
#define GET_PKT_OFFSET(a) (((a) >> 22) & 0x1ff)

void WILC_WFI_monitor_rx(struct wilc_vif *vif, u8 *buff, u32 size)
{
	u32 header, pkt_offset;
	struct sk_buff *skb = NULL;
	struct wilc_wfi_radiotap_hdr *hdr;
	struct wilc_wfi_radiotap_cb_hdr *cb_hdr;

	PRINT_D(vif->ndev, HOSTAPD_DBG,
		"In monitor interface receive function\n");
	if (!wilc_wfi_mon)
		return;

	if (!netif_running(wilc_wfi_mon)) {
		PRINT_D(vif->ndev, HOSTAPD_DBG,
			"Monitor interface already RUNNING\n");
		return;
	}

	/* Get WILC header */
	memcpy(&header, (buff - HOST_HDR_OFFSET), HOST_HDR_OFFSET);
	/*
	 * The packet offset field contain info about what type of management
	 * the frame we are dealing with and ack status
	 */
	pkt_offset = GET_PKT_OFFSET(header);

	if (pkt_offset & IS_MANAGMEMENT_CALLBACK) {
		/* hostapd callback mgmt frame */

		skb = dev_alloc_skb(size + sizeof(struct wilc_wfi_radiotap_cb_hdr));
		if (!skb) {
			PRINT_D(vif->ndev, HOSTAPD_DBG,
				"Monitor if : No memory to allocate skb");
			return;
		}
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,0)
		skb_put_data(skb, buff, size);

		cb_hdr = skb_push(skb, sizeof(*cb_hdr));
	#else
		memcpy(skb_put(skb, size), buff, size);
		cb_hdr = (struct wilc_wfi_radiotap_cb_hdr *)skb_push(skb, sizeof(*cb_hdr));
	#endif
		memset(cb_hdr, 0, sizeof(struct wilc_wfi_radiotap_cb_hdr));

		cb_hdr->hdr.it_version = 0; /* PKTHDR_RADIOTAP_VERSION; */

		cb_hdr->hdr.it_len = cpu_to_le16(sizeof(struct wilc_wfi_radiotap_cb_hdr));

		cb_hdr->hdr.it_present = cpu_to_le32(
				(1 << IEEE80211_RADIOTAP_RATE) |
				(1 << IEEE80211_RADIOTAP_TX_FLAGS));

		cb_hdr->rate = 5; /* txrate->bitrate / 5; */

		if (pkt_offset & IS_MGMT_STATUS_SUCCES)	{
			/* success */
			cb_hdr->tx_flags = IEEE80211_RADIOTAP_F_TX_RTS;
		} else {
			cb_hdr->tx_flags = IEEE80211_RADIOTAP_F_TX_FAIL;
		}

	} else {
		skb = dev_alloc_skb(size + sizeof(struct wilc_wfi_radiotap_hdr));

		if (!skb) {
			PRINT_D(vif->ndev, HOSTAPD_DBG,
				"Monitor if : No memory to allocate skb");
			return;
		}
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,0)
		skb_put_data(skb, buff, size);
		hdr = skb_push(skb, sizeof(*hdr));
	#else
		memcpy(skb_put(skb, size), buff, size);
		hdr = (struct wilc_wfi_radiotap_hdr *)skb_push(skb, 
							       sizeof(*hdr));
	#endif	
		memset(hdr, 0, sizeof(struct wilc_wfi_radiotap_hdr));
		hdr->hdr.it_version = 0; /* PKTHDR_RADIOTAP_VERSION; */
		hdr->hdr.it_len = cpu_to_le16(sizeof(struct wilc_wfi_radiotap_hdr));
		PRINT_D(vif->ndev, HOSTAPD_DBG,
			"Radiotap len %d\n", hdr->hdr.it_len);
		hdr->hdr.it_present = cpu_to_le32
				(1 << IEEE80211_RADIOTAP_RATE); /* | */
		PRINT_D(vif->ndev, HOSTAPD_DBG,"Presentflags %d\n",
			hdr->hdr.it_present);
		hdr->rate = 5; /* txrate->bitrate / 5; */
	}

	skb->dev = wilc_wfi_mon;
	skb_reset_mac_header(skb);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb->pkt_type = PACKET_OTHERHOST;
	skb->protocol = htons(ETH_P_802_2);
	memset(skb->cb, 0, sizeof(skb->cb));

	netif_rx(skb);
}

struct tx_complete_mon_data {
	int size;
	void *buff;
};

static void mgmt_tx_complete(void *priv, int status)
{
	struct tx_complete_mon_data *pv_data = priv;
	u8 *buf =  pv_data->buff;

	if (status == 1) {
		if (buf[0] == 0x10 || buf[0] == 0xb0)
		PRINT_INFO(wilc_wfi_mon, HOSTAPD_DBG, 
			   "Packet sent Size = %d Add = %p.\n",
			   pv_data->size, pv_data->buff);
	} else {
		PRINT_INFO(wilc_wfi_mon, HOSTAPD_DBG,
			   "Couldn't send packet Size = %d Add = %p.\n",
			   pv_data->size,pv_data->buff);
	}
	/*
	 * in case of fully hosting mode, the freeing will be done
	 * in response to the cfg packet
	 */
	kfree(pv_data->buff);

	kfree(pv_data);
}

static int mon_mgmt_tx(struct net_device *dev, const u8 *buf, size_t len)
{
	struct tx_complete_mon_data *mgmt_tx = NULL;

	if (!dev) {
		PRINT_ER(dev, "ERROR: dev == NULL\n");
		return -EFAULT;
	}

	netif_stop_queue(dev);
	mgmt_tx = kmalloc(sizeof(*mgmt_tx), GFP_ATOMIC);
	if (!mgmt_tx) {
		PRINT_ER(dev,
			 "Failed to allocate memory for mgmt_tx structure\n");
		return -ENOMEM;
	}

	mgmt_tx->buff = kmemdup(buf, len, GFP_ATOMIC);
	if (!mgmt_tx->buff) {
		kfree(mgmt_tx);
		return -ENOMEM;
	}

	mgmt_tx->size = len;

	wilc_wlan_txq_add_mgmt_pkt(dev, mgmt_tx, mgmt_tx->buff, mgmt_tx->size,
				   mgmt_tx_complete);

	netif_wake_queue(dev);
	return 0;
}

static netdev_tx_t WILC_WFI_mon_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	u32 rtap_len, ret = 0;
	struct WILC_WFI_mon_priv  *mon_priv;

	if (!wilc_wfi_mon)
		return -EFAULT;

	mon_priv = netdev_priv(wilc_wfi_mon);
	if (!mon_priv) {
		PRINT_ER(dev, "Monitor interface private structure is NULL\n");
		return -EFAULT;
	}
	rtap_len = ieee80211_get_radiotap_len(skb->data);
	if (skb->len < rtap_len) {
		PRINT_ER(dev, "Error in radiotap header\n");
		return -1;
	}

	skb_pull(skb, rtap_len);

	skb->dev = mon_priv->real_ndev;

	PRINT_D(dev, HOSTAPD_DBG,"Skipping the radiotap header\n");
	PRINT_D(dev, HOSTAPD_DBG,"SKB netdevice name = %s\n", skb->dev->name);
	PRINT_D(dev, HOSTAPD_DBG,"MONITOR real dev name = %s\n",
		mon_priv->real_ndev->name);
	/* Identify if Ethernet or MAC header (data or mgmt) */
	memcpy(srcadd, &skb->data[10], 6);
	memcpy(bssid, &skb->data[16], 6);
	/* if source address and bssid fields are equal>>Mac header */
	/*send it to mgmt frames handler */
	if (!(memcmp(srcadd, bssid, 6))) {
		ret = mon_mgmt_tx(mon_priv->real_ndev, skb->data, skb->len);
		if (ret)
			PRINT_ER(dev, "fail to mgmt tx\n");
		dev_kfree_skb(skb);
	} else {
		ret = wilc_mac_xmit(skb, mon_priv->real_ndev);
	}

	return ret;
}

static const struct net_device_ops wilc_wfi_netdev_ops = {
	.ndo_start_xmit         = WILC_WFI_mon_xmit,

};

struct net_device *WILC_WFI_init_mon_interface(const char *name,
					       struct net_device *real_dev)
{
	struct WILC_WFI_mon_priv *priv;

	/*If monitor interface is already initialized, return it*/
	if (wilc_wfi_mon)
		return wilc_wfi_mon;

	wilc_wfi_mon = alloc_etherdev(sizeof(struct WILC_WFI_mon_priv));
	if (!wilc_wfi_mon) {
		PRINT_ER(real_dev, "failed to allocate memory\n");
		return NULL;
	}
	wilc_wfi_mon->type = ARPHRD_IEEE80211_RADIOTAP;
	strncpy(wilc_wfi_mon->name, name, IFNAMSIZ);
	wilc_wfi_mon->name[IFNAMSIZ - 1] = 0;
	wilc_wfi_mon->netdev_ops = &wilc_wfi_netdev_ops;

	if (register_netdevice(wilc_wfi_mon)) {
		PRINT_ER(real_dev, "register_netdevice failed \n");
		return NULL;
	}
	priv = netdev_priv(wilc_wfi_mon);
	if (!priv) {
		PRINT_ER(real_dev, "private structure is NULL\n");
		return NULL;
	}

	priv->real_ndev = real_dev;

	return wilc_wfi_mon;
}

int WILC_WFI_deinit_mon_interface(void)
{
	bool rollback_lock = false;

	if (wilc_wfi_mon) {
		PRINT_INFO(wilc_wfi_mon, HOSTAPD_DBG,
			   "In Deinit monitor interface\n");
		PRINT_INFO(wilc_wfi_mon, HOSTAPD_DBG, "Locking RTNL\n");
		if (rtnl_is_locked()) {
			rtnl_unlock();
			rollback_lock = true;
		}
		PRINT_INFO(wilc_wfi_mon, HOSTAPD_DBG, "Unregister netdev\n");
		unregister_netdev(wilc_wfi_mon);

		if (rollback_lock) {
			rtnl_lock();
			rollback_lock = false;
		}
		wilc_wfi_mon = NULL;
	}
	return 0;
}
