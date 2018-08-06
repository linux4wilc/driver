// SPDX-License-Identifier: GPL-2.0
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/ip.h>

#include "wilc_wfi_netdevice.h"
#include "wilc_wlan_cfg.h"
#include "linux_wlan.h"

#define WAKUP_TRAILS_TIMEOUT		(10000)

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,12,21)
#define list_next_entry(pos, member) \
	list_entry((pos)->member.next, typeof(*(pos)), member)
#endif

extern void wilc_frmw_to_linux(struct wilc_vif *vif, u8 *buff, u32 size,
				u32 pkt_offset, u8 status);
extern void wilc_wfi_monitor_rx(struct wilc_vif *vif, u8 *buff, u32 size);

void acquire_bus(struct wilc *wilc, enum bus_acquire acquire, int source)
{
	mutex_lock(&wilc->hif_cs);
	if (acquire == ACQUIRE_AND_WAKEUP)
		chip_wakeup(wilc, source);
}

void release_bus(struct wilc *wilc, enum bus_release release, int source)
{
	if (release == RELEASE_ALLOW_SLEEP)
		chip_allow_sleep(wilc, source);
	mutex_unlock(&wilc->hif_cs);
}

uint8_t reset_bus(struct wilc *wilc)
{
	uint8_t ret = 0;
	if(wilc->io_type == HIF_SPI)
		return wilc->hif_func->hif_reset(wilc);
	return ret;
}

static void wilc_wlan_txq_remove(struct wilc *wilc, u8 q_num,
				 struct txq_entry_t *tqe)
{
	list_del(&tqe->list);
	wilc->txq_entries -= 1;
	wilc->txq[q_num].count--;
}

static struct txq_entry_t *
wilc_wlan_txq_remove_from_head(struct net_device *dev, u8 q_num)
{
	struct txq_entry_t *tqe = NULL;
	unsigned long flags;
	struct wilc_vif *vif;
	struct wilc *wilc;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	spin_lock_irqsave(&wilc->txq_spinlock, flags);
	if (!list_empty(&wilc->txq[q_num].txq_head.list)) {
		tqe = list_first_entry(&wilc->txq[q_num].txq_head.list,
				       struct txq_entry_t, list);
		list_del(&tqe->list);
		wilc->txq_entries -= 1;
		wilc->txq[q_num].count--;
	}
	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);
	return tqe;
}

static void wilc_wlan_txq_add_to_tail(struct net_device *dev, u8 q_num,
				      struct txq_entry_t *tqe)
{
	unsigned long flags;
	struct wilc_vif *vif;
	struct wilc *wilc;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	spin_lock_irqsave(&wilc->txq_spinlock, flags);
	list_add_tail(&tqe->list, &wilc->txq[q_num].txq_head.list);
	wilc->txq_entries += 1;
	wilc->txq[q_num].count++;
	PRINT_INFO(vif->ndev, TX_DBG, "Number of entries in TxQ = %d\n",
		   wilc->txq_entries);

	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);

	PRINT_INFO(vif->ndev, TX_DBG, "Wake the txq_handling\n");
	complete(&wilc->txq_event);
}

static int wilc_wlan_txq_add_to_head(struct wilc_vif *vif, u8 q_num,
				     struct txq_entry_t *tqe)
{
	unsigned long flags;
	struct wilc *wilc = vif->wilc;

	mutex_lock(&wilc->txq_add_to_head_cs);

	spin_lock_irqsave(&wilc->txq_spinlock, flags);
	list_add(&tqe->list, &wilc->txq[q_num].txq_head.list);
	wilc->txq_entries += 1;
	wilc->txq[q_num].count++;
	PRINT_INFO(vif->ndev, TX_DBG,"Number of entries in TxQ = %d\n",wilc->txq_entries);

	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);
	mutex_unlock(&wilc->txq_add_to_head_cs);
	complete(&wilc->txq_event);
	PRINT_INFO(vif->ndev, TX_DBG, "Wake up the txq_handler\n");

	return 0;
}

struct ack_session_info;
struct ack_session_info {
	u32 seq_num;
	u32 bigger_ack_num;
	u16 src_port;
	u16 dst_port;
	u16 status;
};

struct pending_acks_info {
	u32 ack_num;
	u32 session_index;
	struct txq_entry_t  *txqe;
};

#define NOT_TCP_ACK			(-1)

#define MAX_TCP_SESSION		25
#define MAX_PENDING_ACKS		256
static struct ack_session_info ack_session_info[2 * MAX_TCP_SESSION];
static struct pending_acks_info pending_acks_info[MAX_PENDING_ACKS];

static u32 pending_base;
static u32 tcp_session;
static u32 pending_acks;

static inline int add_tcp_session(struct wilc_vif *vif, u32 src_prt,
				  u32 dst_prt, u32 seq)
{
	if (tcp_session < 2 * MAX_TCP_SESSION) {
		ack_session_info[tcp_session].seq_num = seq;
		ack_session_info[tcp_session].bigger_ack_num = 0;
		ack_session_info[tcp_session].src_port = src_prt;
		ack_session_info[tcp_session].dst_port = dst_prt;
		tcp_session++;
		PRINT_INFO(vif->ndev, TCP_ENH, "TCP Session %d to Ack %d\n",
			   tcp_session, seq);
	}
	return 0;
}

static inline int update_tcp_session(struct wilc_vif *vif, u32 index, u32 ack)
{
	if (index < 2 * MAX_TCP_SESSION &&
	    ack > ack_session_info[index].bigger_ack_num)
		ack_session_info[index].bigger_ack_num = ack;
	return 0;
}

static inline int add_tcp_pending_ack(struct wilc_vif *vif, u32 ack,
				      u32 session_index,
				      struct txq_entry_t *txqe)
{
	u32 i = pending_base + pending_acks;

	if (i < MAX_PENDING_ACKS) {
		pending_acks_info[i].ack_num = ack;
		pending_acks_info[i].txqe = txqe;
		pending_acks_info[i].session_index = session_index;
		txqe->tcp_pending_ack_idx = i;
		pending_acks++;
	}
	return 0;
}

static inline void tcp_process(struct net_device *dev, struct txq_entry_t *tqe)
{
	void *buffer = tqe->buffer;
	const struct ethhdr *eth_hdr_ptr = buffer;
	int i;
	unsigned long flags;
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc = vif->wilc;

	spin_lock_irqsave(&wilc->txq_spinlock, flags);

	if (eth_hdr_ptr->h_proto == htons(ETH_P_IP)) {
		const struct iphdr *ip_hdr_ptr = buffer + ETH_HLEN;

		if (ip_hdr_ptr->protocol == IPPROTO_TCP) {
			const struct tcphdr *tcp_hdr_ptr;
			u32 IHL, total_length, data_offset;

			IHL = ip_hdr_ptr->ihl << 2;
			tcp_hdr_ptr = buffer + ETH_HLEN + IHL;
			total_length = ntohs(ip_hdr_ptr->tot_len);

			data_offset = tcp_hdr_ptr->doff << 2;
			if (total_length == (IHL + data_offset)) {
				u32 seq_no, ack_no;

				seq_no = ntohl(tcp_hdr_ptr->seq);
				ack_no = ntohl(tcp_hdr_ptr->ack_seq);
				for (i = 0; i < tcp_session; i++) {
					u32 j = ack_session_info[i].seq_num;

					if (i < 2 * MAX_TCP_SESSION &&
					    j == seq_no) {
						update_tcp_session(vif, i,
								   ack_no);
						break;
					}
				}
				if (i == tcp_session)
					add_tcp_session(vif, 0, 0, seq_no);

				add_tcp_pending_ack(vif, ack_no, i, tqe);
			}
		}
	}
	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);
}

static int wilc_wlan_txq_filter_dup_tcp_ack(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wilc;
	u32 i = 0;
	u32 dropped = 0;
	unsigned long flags;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	spin_lock_irqsave(&wilc->txq_spinlock, flags);
	for (i = pending_base; i < (pending_base + pending_acks); i++) {
		u32 session_index;
		u32 bigger_ack_num;

		if (i >= MAX_PENDING_ACKS)
			break;

		session_index = pending_acks_info[i].session_index;

		if (session_index >= 2 * MAX_TCP_SESSION)
			break;

		bigger_ack_num = ack_session_info[session_index].bigger_ack_num;

		if (pending_acks_info[i].ack_num < bigger_ack_num) {
			struct txq_entry_t *tqe;

			PRINT_INFO(vif->ndev, TCP_ENH, "DROP ACK: %u\n",
				   pending_acks_info[i].ack_num);
			tqe = pending_acks_info[i].txqe;
			if (tqe) {
				wilc_wlan_txq_remove(wilc, tqe->q_num, tqe);
				tqe->status = 1;
				if (tqe->tx_complete_func)
					tqe->tx_complete_func(tqe->priv,
							      tqe->status);
				kfree(tqe);
				dropped++;
			}
		}
	}
	pending_acks = 0;
	tcp_session = 0;

	if (pending_base == 0)
		pending_base = MAX_TCP_SESSION;
	else
		pending_base = 0;

	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);

	while (dropped > 0) {
		if(!wait_for_completion_timeout(&wilc->txq_event,
						msecs_to_jiffies(1)))
			PRINT_ER(vif->ndev, "completion timedout\n");
		dropped--;
	}

	return 1;
}

static struct net_device *get_if_handler(struct wilc *wilc, u8 *mac_header)
{
	u8 *bssid, *bssid1;
	int i = 0;
	struct net_device *mon_netdev = NULL;

	bssid = mac_header + 10;
	bssid1 = mac_header + 4;
	for (i = 0; i <= wilc->vif_num; i++) {
		if (wilc->vif[i]->iftype == STATION_MODE)
			if (ether_addr_equal_unaligned(bssid,
						       wilc->vif[i]->bssid))
				return wilc->vif[i]->ndev;
		if (wilc->vif[i]->iftype == AP_MODE)
			if (ether_addr_equal_unaligned(bssid1,
						       wilc->vif[i]->bssid))
				return wilc->vif[i]->ndev;
		if (wilc->vif[i]->iftype == MONITOR_MODE)
			mon_netdev = wilc->vif[i]->ndev;
	}

	if (!mon_netdev)
		PRINT_WRN(wilc->vif[0]->ndev, GENERIC_DBG, "Invalid handle\n");
	return mon_netdev;
}

static bool ack_filter_enabled;

void wilc_enable_tcp_ack_filter(bool value)
{
	ack_filter_enabled = value;
}

static int wilc_wlan_txq_add_cfg_pkt(struct wilc_vif *vif, u8 *buffer,
				     u32 buffer_size)
{
	struct txq_entry_t *tqe;
	struct wilc *wilc = vif->wilc;

	PRINT_INFO(vif->ndev, TX_DBG, "Adding config packet ...\n");
	if (wilc->quit) {
		PRINT_INFO(vif->ndev, TX_DBG, "Return due to clear function\n");
		complete(&wilc->cfg_event);
		return 0;
	}

	if (!(wilc->initialized)) {
		PRINT_INFO(vif->ndev, TX_DBG, "wilc not initialized\n");
		complete(&wilc->cfg_event);
		return 0;
	}
	tqe = kmalloc(sizeof(*tqe), GFP_KERNEL);
	if (!tqe) {
		complete(&wilc->cfg_event);
		return 0;
	}
	tqe->type = WILC_CFG_PKT;
	tqe->buffer = buffer;
	tqe->buffer_size = buffer_size;
	tqe->tx_complete_func = NULL;
	tqe->priv = NULL;
	tqe->q_num = AC_VO_Q;
	tqe->tcp_pending_ack_idx = NOT_TCP_ACK;

	PRINT_INFO(vif->ndev, TX_DBG,
		   "Adding the config packet at the Queue tail\n");

	if (wilc_wlan_txq_add_to_head(vif, AC_VO_Q, tqe)) {
		complete(&wilc->cfg_event);
		kfree(tqe);
		return 0;
	}

	return 1;
}

static void ac_q_limit(struct wilc *wilc, u8 ac, u16 *q_limit)
{
	static u8 buffer[AC_BUFFER_SIZE];
	static u16 end_index;
	static bool initialized;
	static u16 cnt[NQUEUES];
	u8 factors[NQUEUES] = {1, 1, 1, 1};
	static u16 sum;
	u16 i;
	unsigned long flags;

	spin_lock_irqsave(&wilc->txq_spinlock, flags);
	if (!initialized) {
		for (i = 0; i < AC_BUFFER_SIZE; i++)
			buffer[i] = i % NQUEUES;

		for (i = 0; i < NQUEUES; i++) {
			cnt[i] = AC_BUFFER_SIZE * factors[i] / NQUEUES;
			sum += cnt[i];
		}
		end_index = AC_BUFFER_SIZE - 1;
		initialized = 1;
	}

	cnt[buffer[end_index]] -= factors[buffer[end_index]];
	cnt[ac] += factors[ac];
	sum += (factors[ac] - factors[buffer[end_index]]);

	buffer[end_index] = ac;
	if (end_index > 0)
		end_index--;
	else
		end_index = AC_BUFFER_SIZE - 1;

	for (i = 0; i < NQUEUES; i++){
		if(!sum)
			q_limit[i] = 1;
		else
			q_limit[i] = (cnt[i] * FLOW_CONTROL_UPPER_THRESHOLD / sum) + 1;
	}
	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);
	return;
}

static inline u8 ac_classify(struct wilc *wilc, struct txq_entry_t *tqe)
{
	u8 *eth_hdr_ptr;
	u8 *buffer = tqe->buffer;
	u8 ac;
	u16 h_proto;
	unsigned long flags;

	spin_lock_irqsave(&wilc->txq_spinlock, flags);

	eth_hdr_ptr = &buffer[0];
	h_proto = ntohs(*((unsigned short *)&eth_hdr_ptr[12]));
	if (h_proto == ETH_P_IP) {
		u8 *ip_hdr_ptr;
		u32 IHL, DSCP;

		ip_hdr_ptr = &buffer[ETHERNET_HDR_LEN];
		IHL = (ip_hdr_ptr[0] & 0xf) << 2;
		DSCP = (ip_hdr_ptr[1] & 0xfc);

		switch (DSCP) {
		case 0x20:
		case 0x40:
		case 0x08:
			ac = AC_BK_Q;
			break;
		case 0x80:
		case 0xA0:
		case 0x28:
			ac = AC_VI_Q;
			break;
		case 0xC0:
		case 0xd0:
		case 0xE0:
		case 0x88:
		case 0xB8:
			ac = AC_VO_Q;
			break;
		default:
			ac = AC_BE_Q;
			break;
		}
	} else {
		ac  = AC_BE_Q;
	}

	tqe->q_num = ac;
	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);

	return ac;
}

static inline int ac_balance(u8 *count, u8 *ratio)
{
	u8 i, max_count = 0;

	if (!count || !ratio)
		return -1;

	for (i = 0; i < NQUEUES; i++)
		if (count[i] > max_count)
			max_count = count[i];

	for (i = 0; i < NQUEUES; i++)
		ratio[i] = max_count - count[i];

	return 0;
}

static inline void ac_pkt_count(u32 reg, u8 *pkt_count)
{
	pkt_count[AC_BK_Q] = (reg & 0x000000fa) >> BK_AC_COUNT_POS;
	pkt_count[AC_BE_Q] = (reg & 0x0000fe00) >> BE_AC_COUNT_POS;
	pkt_count[AC_VI_Q] = (reg & 0x00fe0000) >> VI_AC_COUNT_POS;
	pkt_count[AC_VO_Q] = (reg & 0xfe000000) >> VO_AC_COUNT_POS;
}

static inline u8 ac_change(struct wilc *wilc, u8 *ac)
{
	do {
		if (wilc->txq[*ac].acm == 0)
			return 0;
		(*ac)++;
	} while (*ac < NQUEUES);
	return 1;
}

static inline void ac_acm_bit(struct wilc *wilc, u32 reg)
{
	wilc->txq[AC_BK_Q].acm = (reg & 0x00000002) >> BK_AC_ACM_STAT_POS;
	wilc->txq[AC_BE_Q].acm = (reg & 0x00000100) >> BE_AC_ACM_STAT_POS;
	wilc->txq[AC_VI_Q].acm = (reg & 0x00010000) >> VI_AC_ACM_STAT_POS;
	wilc->txq[AC_VO_Q].acm = (reg & 0x01000000) >> VO_AC_ACM_STAT_POS;
}

int wilc_wlan_txq_add_net_pkt(struct net_device *dev, void *priv, u8 *buffer,
			      u32 buffer_size, wilc_tx_complete_func_t func)
{
	struct txq_entry_t *tqe;
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc;
	u8 q_num;
	u16 q_limit[NQUEUES] = {0, 0, 0, 0};

	if(!vif){
		pr_info("%s vif is NULL\n", __func__);
		return -1;
	}

	wilc = vif->wilc;

	if (wilc->quit) {
		PRINT_INFO(vif->ndev, TX_DBG,
			   "drv is quitting, return from net_pkt\n");
		func(priv, 0);
		return 0;
	}

	if (!(wilc->initialized)) {
		PRINT_INFO(vif->ndev, TX_DBG,
			   "not_init, return from net_pkt\n");
		func(priv, 0);
		return 0;
	}

	tqe = kmalloc(sizeof(*tqe), GFP_KERNEL);

	if (!tqe) {
		PRINT_INFO(vif->ndev, TX_DBG,
			   "malloc failed, return from net_pkt\n");
		func(priv, 0);
		return 0;
	}
	tqe->type = WILC_NET_PKT;
	tqe->buffer = buffer;
	tqe->buffer_size = buffer_size;
	tqe->tx_complete_func = func;
	tqe->priv = priv;

	q_num = ac_classify(wilc, tqe);
	if (ac_change(wilc, &q_num)) {
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "No suitable non-ACM queue\n");
		kfree(tqe);
		return 0;
	}
	ac_q_limit(wilc, q_num, q_limit);

	if ((q_num == AC_VO_Q && wilc->txq[q_num].count <= q_limit[AC_VO_Q]) ||
	    (q_num == AC_VI_Q && wilc->txq[q_num].count <= q_limit[AC_VI_Q]) ||
	    (q_num == AC_BE_Q && wilc->txq[q_num].count <= q_limit[AC_BE_Q]) ||
	    (q_num == AC_BK_Q && wilc->txq[q_num].count <= q_limit[AC_BK_Q])) {
		PRINT_INFO(vif->ndev, TX_DBG,
			   "Adding mgmt packet at the Queue tail\n");
		tqe->tcp_pending_ack_idx = NOT_TCP_ACK;
		if (ack_filter_enabled)
			tcp_process(dev, tqe);
		wilc_wlan_txq_add_to_tail(dev, q_num, tqe);
	} else {
		tqe->status = 0;
		if (tqe->tx_complete_func)
			tqe->tx_complete_func(tqe->priv, tqe->status);
		kfree(tqe);
	}

	return wilc->txq_entries;
}

int wilc_wlan_txq_add_mgmt_pkt(struct net_device *dev, void *priv, u8 *buffer,
			       u32 buffer_size, wilc_tx_complete_func_t func)
{
	struct txq_entry_t *tqe;
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc;

	wilc = vif->wilc;

	if (wilc->quit) {
		PRINT_INFO(vif->ndev, TX_DBG, "drv is quitting\n");
		func(priv, 0);
		return 0;
	}

	if (!(wilc->initialized)) {
		PRINT_INFO(vif->ndev, TX_DBG, "wilc not_init\n");
		func(priv, 0);
		return 0;
	}
	tqe = kmalloc(sizeof(*tqe), GFP_KERNEL);

	if (!tqe) {
		PRINT_INFO(vif->ndev, TX_DBG, "Queue malloc failed\n");
		func(priv, 0);
		return 0;
	}
	tqe->type = WILC_MGMT_PKT;
	tqe->buffer = buffer;
	tqe->buffer_size = buffer_size;
	tqe->tx_complete_func = func;
	tqe->priv = priv;
	tqe->q_num = AC_BE_Q;
	tqe->tcp_pending_ack_idx = NOT_TCP_ACK;

	PRINT_INFO(vif->ndev, TX_DBG, "Adding Mgmt packet to Queue tail\n");
	wilc_wlan_txq_add_to_tail(dev, AC_BE_Q, tqe);
	return 1;
}

static struct txq_entry_t *wilc_wlan_txq_get_first(struct wilc *wilc, u8 q_num)
{
	struct txq_entry_t *tqe = NULL;
	unsigned long flags;

	spin_lock_irqsave(&wilc->txq_spinlock, flags);
	if(!list_empty(&wilc->txq[q_num].txq_head.list))
		tqe = list_first_entry(&wilc->txq[q_num].txq_head.list,
				       struct txq_entry_t, list);

	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);

	return tqe;
}

static struct txq_entry_t *wilc_wlan_txq_get_next(struct wilc *wilc,
						  struct txq_entry_t *tqe,
						  u8 q_num)
{
	unsigned long flags;

	spin_lock_irqsave(&wilc->txq_spinlock, flags);
	if(!list_is_last(&tqe->list, &wilc->txq[q_num].txq_head.list))
		tqe = list_next_entry(tqe, list);
	else
		tqe = NULL;

	spin_unlock_irqrestore(&wilc->txq_spinlock, flags);

	return tqe;
}

static void wilc_wlan_rxq_add(struct wilc *wilc, struct rxq_entry_t *rqe)
{
	struct wilc_vif *vif = wilc->vif[0];
	
	if (wilc->quit)
		return;

	mutex_lock(&wilc->rxq_cs);
	list_add_tail(&rqe->list, &wilc->rxq_head.list);
	PRINT_INFO(vif->ndev, RX_DBG,"Added to RX queue\n");
	mutex_unlock(&wilc->rxq_cs);
}

static struct rxq_entry_t *wilc_wlan_rxq_remove(struct wilc *wilc)
{
	struct wilc_vif *vif = wilc->vif[0];
	struct rxq_entry_t *rqe = NULL;
	
	PRINT_INFO(vif->ndev, RX_DBG,"Getting rxQ element\n");

	mutex_lock(&wilc->rxq_cs);
	if (!list_empty(&wilc->rxq_head.list)) {
		rqe = list_first_entry(&wilc->rxq_head.list, struct rxq_entry_t,
				       list);
		list_del(&rqe->list);
	} else {
		PRINT_INFO(vif->ndev, RX_DBG,"Nothing to get from Q\n");
	}
	mutex_unlock(&wilc->rxq_cs);

	return rqe;
}

void chip_allow_sleep(struct wilc *wilc, int source)
{
	u32 reg = 0;

	if (((source == PWR_DEV_SRC_WIFI) &&
	    (wilc->keep_awake[PWR_DEV_SRC_BT] == true)) ||
	    ((source == PWR_DEV_SRC_BT) &&
	    (wilc->keep_awake[PWR_DEV_SRC_WIFI] == true))) {
		pr_warn("Another device is preventing allow sleep operation. request source is %s\n",
			  (source == PWR_DEV_SRC_WIFI ? "Wifi" : "BT"));
	} else {
		int ret;
		if(wilc->chip == WILC_1000) {
			u32 wakeup_reg, wakeup_bit;
			u32 to_host_from_fw_reg, to_host_from_fw_bit;
			u32 from_host_to_fw_reg, from_host_to_fw_bit;
			u32 trials =100;
			
			if(wilc->io_type == HIF_SDIO ||
				wilc->io_type == HIF_SDIO_GPIO_IRQ) {
				wakeup_reg = 0xf0;
				wakeup_bit = BIT(0);
				from_host_to_fw_reg = 0xfa;
				from_host_to_fw_bit = BIT(0);
				to_host_from_fw_reg = 0xfc;
				to_host_from_fw_bit = BIT(0);		
			} else {
				wakeup_reg = 0x1;
				wakeup_bit = BIT(1);
				from_host_to_fw_reg = 0x0b;
				from_host_to_fw_bit = BIT(0);
				to_host_from_fw_reg = 0xfc;
				to_host_from_fw_bit = BIT(0);
			}

			while(trials--){
				ret = wilc->hif_func->hif_read_reg(wilc, to_host_from_fw_reg,&reg);
				if(!ret) goto _fail_;
				if((reg & to_host_from_fw_bit) == 0)
					break;	
			}
			if(!trials)
				pr_warn("FW not responding\n");

			/* Clear bit 1 */
			ret = wilc->hif_func->hif_read_reg(wilc, wakeup_reg, &reg);
			if(!ret) goto _fail_;
			if(reg & wakeup_bit)
			{
				reg &=~wakeup_bit;
				ret = wilc->hif_func->hif_write_reg(wilc, wakeup_reg, reg);
				if(!ret) goto _fail_;
			}

			ret = wilc->hif_func->hif_read_reg(wilc, from_host_to_fw_reg, &reg);
			if(!ret) goto _fail_;
			if(reg & from_host_to_fw_bit)
			{
				reg &= ~from_host_to_fw_bit;
				ret = wilc->hif_func->hif_write_reg(wilc, from_host_to_fw_reg, reg);
				if(!ret) goto _fail_;
			}
		} else {
			if(wilc->io_type == HIF_SDIO ||
				wilc->io_type == HIF_SDIO_GPIO_IRQ) {
				wilc->hif_func->hif_read_reg(wilc, 0xf0, &reg);
				wilc->hif_func->hif_write_reg(wilc, 0xf0, reg & ~BIT(0));
			} else {
				wilc->hif_func->hif_read_reg(wilc, 0x1, &reg);
				wilc->hif_func->hif_write_reg(wilc, 0x1, reg & ~BIT(1));
			}
		}
	}

	wilc->keep_awake[source] = false;
_fail_:
	return;
}

void chip_wakeup_wilc1000(struct wilc *wilc, int source)
{
	u32 ret = 0;
	u32 reg = 0, clk_status_val = 0, trials = 0;
	u32 wakeup_reg, wakeup_bit;
	u32 clk_status_reg, clk_status_bit;
	u32 to_host_from_fw_reg, to_host_from_fw_bit;
	u32 from_host_to_fw_reg, from_host_to_fw_bit;
	
	if(wilc->io_type == HIF_SDIO ||
		wilc->io_type == HIF_SDIO_GPIO_IRQ) {
		wakeup_reg = 0xf0;
		clk_status_reg = 0xf1;
		wakeup_bit = BIT(0);
		clk_status_bit = BIT(0);
		from_host_to_fw_reg = 0xfa;
		from_host_to_fw_bit = BIT(0);
		to_host_from_fw_reg = 0xfc;
		to_host_from_fw_bit = BIT(0);		
	} else {
		wakeup_reg = 0x1;
		clk_status_reg = 0x0f;
		wakeup_bit = BIT(1);
		clk_status_bit = BIT(2);
		from_host_to_fw_reg = 0x0b;
		from_host_to_fw_bit = BIT(0);
		to_host_from_fw_reg = 0xfc;
		to_host_from_fw_bit = BIT(0);
	}

	ret = wilc->hif_func->hif_read_reg(wilc, from_host_to_fw_reg, &reg);
	if(!ret) goto _fail_;
	
	if(!(reg & from_host_to_fw_bit)) {
		/*USE bit 0 to indicate host wakeup*/
		ret = wilc->hif_func->hif_write_reg(wilc, from_host_to_fw_reg, reg|from_host_to_fw_bit);
		if(!ret) goto _fail_;
	}
	
	ret = wilc->hif_func->hif_read_reg(wilc, wakeup_reg, &reg);
	if(!ret) goto _fail_;
	/* Set bit 1 */
	if(!(reg & wakeup_bit)) {
		ret = wilc->hif_func->hif_write_reg(wilc, wakeup_reg, reg | wakeup_bit);
		if(!ret) goto _fail_;	
	}

	do {
		ret = wilc->hif_func->hif_read_reg(wilc, clk_status_reg, &clk_status_val);
		if(!ret) {
			pr_err("Bus error (5).%d %x\n",ret,clk_status_val);
			goto _fail_;
		}
		if(clk_status_val & clk_status_bit) {
			break;
		}
		//nm_bsp_sleep(2);
		trials++;
		if(trials > WAKUP_TRAILS_TIMEOUT) {
			pr_err("Failed to wakup the chip\n");
			ret = -1;
			goto _fail_;
		}
	} while(1);

	if(wilc_get_chipid(wilc, false) < 0x1002b0) {
		uint32_t val32;
		/* Enable PALDO back right after wakeup */
		wilc->hif_func->hif_read_reg(wilc, 0x1e1c , &val32);
		val32 |= BIT(6);
		wilc->hif_func->hif_write_reg(wilc, 0x1e1c , val32);

		wilc->hif_func->hif_read_reg(wilc, 0x1e9c , &val32);
		val32 |= BIT(6);
		wilc->hif_func->hif_write_reg(wilc, 0x1e9c , val32);
	}
	/*workaround sometimes spi fail to read clock regs after reading/writing clockless registers*/
	reset_bus(wilc);

_fail_:
	return;
}

void chip_wakeup_wilc3000(struct wilc *wilc, int source)
{
	u32 wakeup_reg_val, clk_status_reg_val, trials = 0;
	u32 wakeup_register, wakeup_bit;
	u32 clk_status_register, clk_status_bit;
	int wake_seq_trials = 5;

	if(wilc->io_type == HIF_SDIO ||
		wilc->io_type == HIF_SDIO_GPIO_IRQ) {
		wakeup_register = 0xf0;
		clk_status_register = 0xf0;
		wakeup_bit = BIT(0);
		clk_status_bit = BIT(4);
	} else {
		wakeup_register = 0x1;
		clk_status_register = 0x13;
		wakeup_bit = BIT(1);
		clk_status_bit = BIT(2);
	}
		
	wilc->hif_func->hif_read_reg(wilc, wakeup_register, &wakeup_reg_val);
	do {
		wilc->hif_func->hif_write_reg(wilc, wakeup_register, wakeup_reg_val | wakeup_bit);
		/* Check the clock status */
		wilc->hif_func->hif_read_reg(wilc, clk_status_register, &clk_status_reg_val);

		/*
		 * in case of clocks off, wait 1ms, and check it again.
		 * if still off, wait for another 1ms, for a total wait of 3ms.
		 * If still off, redo the wake up sequence
		 */
		while (((clk_status_reg_val & clk_status_bit) == 0) &&
		       (((++trials) % 3) == 0)) {
			/* Wait for the chip to stabilize*/
			usleep_range(1000, 1000);

			/*
			 * Make sure chip is awake. This is an extra step that can be removed
			 * later to avoid the bus access overhead
			 * g_wlan.hif_func.hif_read_reg(0xf0, &clk_status_reg_val);
			 */
			wilc->hif_func->hif_read_reg(wilc, clk_status_register,
						      &clk_status_reg_val);

		}
		/* in case of failure, Reset the wakeup bit to introduce a new edge on the next loop */
		if ((clk_status_reg_val & clk_status_bit) == 0) {
			dev_warn(wilc->dev, "clocks still OFF. Retrying\n");
			wilc->hif_func->hif_write_reg(wilc, wakeup_register,
						      wakeup_reg_val & (~wakeup_bit));
		}
	} while (((clk_status_reg_val & clk_status_bit) == 0)
		 && (wake_seq_trials-- > 0));
	if(!wake_seq_trials)
		dev_err(wilc->dev, "clocks still OFF. Wake up failed\n");
	wilc->keep_awake[source] = true;
}

void chip_wakeup(struct wilc *wilc, int source)
{
	if(wilc->chip == WILC_1000)
		chip_wakeup_wilc1000(wilc, source);
	else
		chip_wakeup_wilc3000(wilc, source);
}

void host_wakeup_notify(struct wilc *wilc, int source)
{
	acquire_bus(wilc, ACQUIRE_ONLY,source);
	if (wilc->chip == WILC_1000)
		wilc->hif_func->hif_write_reg(wilc, 0x10b0, 1);
	else
		wilc->hif_func->hif_write_reg(wilc, 0x10c0, 1);
	release_bus(wilc, RELEASE_ONLY, source);
}

void host_sleep_notify(struct wilc *wilc, int source)
{
	acquire_bus(wilc, ACQUIRE_ONLY,source);
	if (wilc->chip == WILC_1000)
		wilc->hif_func->hif_write_reg(wilc, 0x10ac, 1);
	else
		wilc->hif_func->hif_write_reg(wilc, 0x10bc, 1);
	release_bus(wilc, RELEASE_ONLY,source);
}

static u8 ac_fw_count[NQUEUES] = {0, 0, 0, 0};
int wilc_wlan_handle_txq(struct net_device *dev, u32 *txq_count)
{
	int i, entries = 0;
	u8 k, ac;
	u32 sum;
	u32 reg;
	u8 ac_desired_ratio[NQUEUES] = {0, 0, 0, 0};
	u8 ac_preserve_ratio[NQUEUES] = {1, 1, 1, 1};
	u8 *num_pkts_to_add;
	u8 vmm_entries_ac[WILC_VMM_TBL_SIZE];
	u8 *txb;
	u32 offset = 0;
	bool max_size_over = 0, ac_exist = 0;
	int vmm_sz = 0;
	struct txq_entry_t *tqe_q[NQUEUES];
	int ret = 0;
	int counter;
	int timeout;
	u32 vmm_table[WILC_VMM_TBL_SIZE];
	u8 ac_pkt_num_to_chip[NQUEUES] = {0, 0, 0, 0};
	struct wilc_vif *vif;
	struct wilc *wilc;
	const struct wilc_hif_func *func;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	txb = wilc->tx_buffer;
	if (!wilc->txq_entries) {
		*txq_count = 0;
		return 0;
	}

	if (wilc->quit)
		goto out;
	if (ac_balance(ac_fw_count, ac_desired_ratio))
		return -1;
	
	mutex_lock(&wilc->txq_add_to_head_cs);
	wilc_wlan_txq_filter_dup_tcp_ack(dev);

	PRINT_INFO(vif->ndev, TX_DBG,"Getting the head of the TxQ\n");
	for(ac = 0; ac < NQUEUES; ac++)
		tqe_q[ac] = wilc_wlan_txq_get_first(wilc, ac);
	i = 0;
	sum = 0;
	max_size_over = 0;
	num_pkts_to_add = ac_desired_ratio;
	do {
		ac_exist = 0;
		for(ac = 0; (ac < NQUEUES) && (!max_size_over); ac++) {
			if (tqe_q[ac]) {
				ac_exist = 1;
				for(k = 0; (k < num_pkts_to_add[ac]) && (!max_size_over) && tqe_q[ac]; k++) {
					if (i < (WILC_VMM_TBL_SIZE - 1)) {
						if (tqe_q[ac]->type == WILC_CFG_PKT)
							vmm_sz = ETH_CONFIG_PKT_HDR_OFFSET;
						else if (tqe_q[ac]->type == WILC_NET_PKT)
							vmm_sz = ETH_ETHERNET_HDR_OFFSET;
						else
							vmm_sz = HOST_HDR_OFFSET;

						vmm_sz += tqe_q[ac]->buffer_size;
						PRINT_INFO(vif->ndev, TX_DBG,"VMM Size before alignment = %d\n",vmm_sz);
						if (vmm_sz & 0x3)
							vmm_sz = (vmm_sz + 4) & ~0x3;

						if ((sum + vmm_sz) > LINUX_TX_SIZE) {
							max_size_over = 1;
							break;
						}
						PRINT_INFO(vif->ndev, TX_DBG,"VMM Size AFTER alignment = %d\n",vmm_sz);
						vmm_table[i] = vmm_sz / 4;
						PRINT_INFO(vif->ndev, TX_DBG,"VMMTable entry size = %d\n",vmm_table[i]);
						if (tqe_q[ac]->type == WILC_CFG_PKT) {
							vmm_table[i] |= BIT(10);
							PRINT_INFO(vif->ndev, TX_DBG,"VMMTable entry changed for CFG packet = %d\n",vmm_table[i]);
						}
						vmm_table[i] = cpu_to_le32(vmm_table[i]);
						vmm_entries_ac[i] = ac;

						i++;
						sum += vmm_sz;
						PRINT_INFO(vif->ndev, TX_DBG,"sum = %d\n",sum);
						tqe_q[ac] = wilc_wlan_txq_get_next(wilc, tqe_q[ac], ac);
					} else {
						max_size_over = 1;
						break;
					}
				}
			}
		}
		num_pkts_to_add = ac_preserve_ratio;
	} while (!max_size_over && ac_exist);

	if (i == 0) {
		PRINT_INFO(vif->ndev, TX_DBG,"Nothing in TX-Q\n");
		goto out;
	} 
	vmm_table[i] = 0x0;

	acquire_bus(wilc, ACQUIRE_AND_WAKEUP, PWR_DEV_SRC_WIFI);
	counter = 0;
	func = wilc->hif_func;
	do {
		ret = func->hif_read_reg(wilc, WILC_HOST_TX_CTRL, &reg);
		if (!ret) {
			PRINT_ER(vif->ndev, "fail read reg vmm_tbl_entry..\n");
			break;
		}
		if ((reg & 0x1) == 0){
			ac_pkt_count(reg, ac_fw_count);
			ac_acm_bit(wilc, reg);
			break;
		}

		counter++;
		if (counter > 200) {
			counter = 0;
			PRINT_INFO(vif->ndev, TX_DBG,
				    "Looping in tx ctrl , force quit\n");
			ret = func->hif_write_reg(wilc, WILC_HOST_TX_CTRL, 0);
			break;
		}
	} while (!wilc->quit);

	if (!ret)
		goto out_release_bus;

	timeout = 200;
	do {
		ret = func->hif_block_tx(wilc,
					 WILC_VMM_TBL_RX_SHADOW_BASE,
					 (u8 *)vmm_table,
					 ((i + 1) * 4));
		if (!ret) {
			PRINT_ER(vif->ndev, "ERR block TX of VMM table.\n");
			break;
		}
		
		if (wilc->chip == WILC_1000) {
			ret = wilc->hif_func->hif_write_reg(wilc,
							    WILC_HOST_VMM_CTL,
							    0x2);
			if (!ret) {
				PRINT_ER(vif->ndev, 
					  "fail write reg host_vmm_ctl..\n");
				break;
			}

			do {
				ret = func->hif_read_reg(wilc,
						      WILC_HOST_VMM_CTL,
						      &reg);
				if (!ret)
					break;
				if ((reg >> 2) & 0x1) {
					entries = ((reg >> 3) & 0x3f);
					break;
				}
			} while (--timeout);
		} else {
			ret = func->hif_write_reg(wilc,
					      WILC_HOST_VMM_CTL,
					      0);
			if (!ret) {
				PRINT_ER(vif->ndev, 
					  "fail write reg host_vmm_ctl..\n");
				break;
			}
			/* interrupt firmware */
			ret = func->hif_write_reg(wilc,
					      WILC_INTERRUPT_CORTUS_0,
					      1);
			if (!ret) {
				PRINT_ER(vif->ndev,
					  "fail write reg WILC_INTERRUPT_CORTUS_0..\n");
				break;
			}

			do {
				ret = func->hif_read_reg(wilc,
						      WILC_INTERRUPT_CORTUS_0,
						      &reg);
				if (!ret) {
					PRINT_ER(vif->ndev,
						  "fail read reg WILC_INTERRUPT_CORTUS_0..\n");
					break;
				}
				if (reg == 0) {
					// Get the entries

					ret = func->hif_read_reg(wilc,
							      WILC_HOST_VMM_CTL,
							      &reg);
					if (!ret) {
						PRINT_ER(vif->ndev,
							  "fail read reg host_vmm_ctl..\n");
						break;
					}
					entries = ((reg >> 3) & 0x3f);
					break;
				}
			} while (--timeout);
		}
		if (timeout <= 0) {
			ret = func->hif_write_reg(wilc, WILC_HOST_VMM_CTL, 0x0);
			break;
		}

		if (!ret)
			break;

		if (entries == 0) {
			PRINT_INFO(vif->ndev, TX_DBG,
				   "no buffer in the chip (reg: %08x), retry later [[ %d, %x ]] \n",
				   reg, i, vmm_table[i-1]);
			ret = func->hif_read_reg(wilc, WILC_HOST_TX_CTRL, &reg);
			if (!ret) {
				PRINT_ER(vif->ndev,
					  "fail read reg WILC_HOST_TX_CTRL..\n");
				break;
			}
			reg &= ~BIT(0);
			ret = func->hif_write_reg(wilc, WILC_HOST_TX_CTRL, reg);
			if (!ret) {
				PRINT_ER(vif->ndev,
					  "fail write reg WILC_HOST_TX_CTRL..\n");
				break;
			}
			break;
		}
		break;
	} while (1);

	if (!ret)
		goto out_release_bus;

	if (entries == 0) {
		ret = WILC_TX_ERR_NO_BUF;
		goto out_release_bus;
	}

	release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
	schedule();
	offset = 0;
	i = 0;
	do {
		struct txq_entry_t *tqe;
		u32 header, buffer_offset;

		tqe = wilc_wlan_txq_remove_from_head(dev, vmm_entries_ac[i]);
		ac_pkt_num_to_chip[vmm_entries_ac[i]]++;
		if (!tqe)
			break;
		if (vmm_table[i] == 0)
			break;

		vmm_table[i] = cpu_to_le32(vmm_table[i]);
		vmm_sz = (vmm_table[i] & 0x3ff);
		vmm_sz *= 4;
		header = (tqe->type << 31) |
			 (tqe->buffer_size << 15) |
			 vmm_sz;
		if (tqe->type == WILC_MGMT_PKT)
			header |= BIT(30);
		else
			header &= ~BIT(30);

		header = cpu_to_le32(header);
		memcpy(&txb[offset], &header, 4);
		if (tqe->type == WILC_CFG_PKT) {
			buffer_offset = ETH_CONFIG_PKT_HDR_OFFSET;
		} else if (tqe->type == WILC_NET_PKT) {
			char *bssid = ((struct tx_complete_data *)(tqe->priv))->bssid;
			int prio = tqe->q_num;

			buffer_offset = ETH_ETHERNET_HDR_OFFSET;
			memcpy(&txb[offset + 4], &prio, sizeof(prio));
			memcpy(&txb[offset + 8], bssid, 6);
		} else {
			buffer_offset = HOST_HDR_OFFSET;
		}

		memcpy(&txb[offset + buffer_offset],
		       tqe->buffer, tqe->buffer_size);
		offset += vmm_sz;
		i++;
		tqe->status = 1;
		if (tqe->tx_complete_func)
			tqe->tx_complete_func(tqe->priv,
					      tqe->status);
		if (tqe->tcp_pending_ack_idx != NOT_TCP_ACK &&
		    tqe->tcp_pending_ack_idx < MAX_PENDING_ACKS)
			pending_acks_info[tqe->tcp_pending_ack_idx].txqe = NULL;
		kfree(tqe);
	} while (--entries);
	for(i = 0; i < NQUEUES; i++)
		ac_fw_count[i] += ac_pkt_num_to_chip[i];

	acquire_bus(wilc, ACQUIRE_AND_WAKEUP, PWR_DEV_SRC_WIFI);

	ret = func->hif_clear_int_ext(wilc, ENABLE_TX_VMM);
	if (!ret) {
		PRINT_ER(vif->ndev, "fail start tx VMM ...\n");
		goto out_release_bus;
	}

	ret = func->hif_block_tx_ext(wilc, 0, txb, offset);
	if(!ret)
		PRINT_ER(vif->ndev, "fail block tx ext...\n");

out_release_bus:
	release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
	schedule();

out:
	mutex_unlock(&wilc->txq_add_to_head_cs);

	PRINT_INFO(vif->ndev, TX_DBG,"THREAD: Exiting txq\n");
	*txq_count = wilc->txq_entries;
	if(ret == 1)
		cfg_packet_timeout = 0;
	return ret;
}

static void wilc_wlan_handle_rx_buff(struct wilc *wilc, u8 *buffer, int size)
{
	int offset = 0;
	u32 header;
	u32 pkt_len, pkt_offset, tp_len;
	int is_cfg_packet;
	u8 *buff_ptr;
	struct wilc_vif *vif = wilc->vif[0];

	do {
		PRINT_INFO(vif->ndev, RX_DBG, "Handling rx buffer\n");
		buff_ptr = buffer + offset;
		memcpy(&header, buff_ptr, 4);
		header = cpu_to_le32(header);
		PRINT_INFO(vif->ndev, RX_DBG,
			   "Header = %04x - Offset = %d\n",header,offset);

		is_cfg_packet = (header >> 31) & 0x1;
		pkt_offset = (header >> 22) & 0x1ff;
		tp_len = (header >> 11) & 0x7ff;
		pkt_len = header & 0x7ff;

		if (pkt_len == 0 || tp_len == 0) {
			PRINT_INFO(vif->ndev, RX_DBG, 
				   "Data corrupted %d, %d\n",
				   pkt_len, tp_len);
			break;
		}

		if (is_cfg_packet) {
			struct wilc_cfg_rsp rsp;

			buff_ptr += pkt_offset;

			wilc_wlan_cfg_indicate_rx(wilc, buff_ptr, pkt_len,
						  &rsp);
			if (rsp.type == WILC_CFG_RSP) {
				PRINT_INFO(vif->ndev, RX_DBG,
					   "cfg_seq %d rsp.seq %d\n",
					   wilc->cfg_seq_no, rsp.seq_no);

				if (wilc->cfg_seq_no == rsp.seq_no)
					complete(&wilc->cfg_event);

			} else if (rsp.type == WILC_CFG_RSP_STATUS) {
				wilc_mac_indicate(wilc);
			}
		} else if (pkt_offset & IS_MANAGMEMENT) {
			 pkt_offset &= ~(IS_MANAGMEMENT |
					 IS_MANAGMEMENT_CALLBACK |
					 IS_MGMT_STATUS_SUCCES);
			 buff_ptr += HOST_HDR_OFFSET;
			 wilc_wfi_mgmt_rx(wilc, buff_ptr, pkt_len);
		} else {
			struct net_device *wilc_netdev;
			
			wilc_netdev = get_if_handler(wilc, buffer);
			if (!wilc_netdev) {
				PRINT_ER(vif->ndev,
					 "wilc_netdev in wilc is NULL");
				return;
			 }

			 vif = netdev_priv(wilc_netdev);

			 if (vif->iftype == MONITOR_MODE)
			 /* packet received on monitor interface */
			 wilc_wfi_monitor_rx(vif, buffer, size);
			 else if (pkt_len > 0)				 
				 wilc_frmw_to_linux(vif, buff_ptr,
							pkt_len,
							pkt_offset,
							PKT_STATUS_NEW);
		}

		offset += tp_len;
		if (offset >= size)
			break;
	} while (1);
}

static void wilc_wlan_handle_rxq(struct wilc *wilc)
{
	int size;
	u8 *buffer;
	struct rxq_entry_t *rqe;
	struct wilc_vif *vif = wilc->vif[0];

	do {
		if (wilc->quit) {
			PRINT_INFO(vif->ndev, RX_DBG,
				   "Quitting. Exit handle RX queue\n");
			complete(&wilc->cfg_event);
			break;
		}
		rqe = wilc_wlan_rxq_remove(wilc);
		if (!rqe) {
			PRINT_INFO(vif->ndev, RX_DBG,
				   "nothing in RX queue\n");
			break;
		}

		buffer = rqe->buffer;
		size = rqe->buffer_size;
		PRINT_INFO(vif->ndev, RX_DBG,
			   "rxQ entery Size = %d - Address = %p\n",
			   size,buffer);

		wilc_wlan_handle_rx_buff(wilc, buffer, size);

		kfree(rqe);
	} while (1);

	PRINT_INFO(vif->ndev, RX_DBG,"THREAD: Exiting RX thread \n");
}

static void wilc_unknown_isr_ext(struct wilc *wilc)
{
	wilc->hif_func->hif_clear_int_ext(wilc, 0);
}

static void wilc_wlan_handle_isr_ext(struct wilc *wilc, u32 int_status)
{
	u32 offset = wilc->rx_buffer_offset;
	u8 *buffer = NULL;
	u32 size;
	u32 retries = 0;
	int ret = 0;
	struct rxq_entry_t *rqe;
	struct wilc_vif *vif = wilc->vif[0];

	size = (int_status & 0x7fff) << 2;

	while (!size && retries < 10) {
		PRINT_ER(vif->ndev,
			 "RX Size equal zero Trying to read it again\n");
		wilc->hif_func->hif_read_size(wilc, &size);
		size = (size & 0x7fff) << 2;
		retries++;
	}

	if (size <= 0)
		return;

	if (LINUX_RX_SIZE - offset < size)
		offset = 0;

	buffer = &wilc->rx_buffer[offset];

	wilc->hif_func->hif_clear_int_ext(wilc, DATA_INT_CLR | ENABLE_RX_VMM);

	ret = wilc->hif_func->hif_block_rx_ext(wilc, 0, buffer, size);
	if (!ret) {
		PRINT_ER(vif->ndev, "fail block rx\n");
		return;
	}

	offset += size;
	wilc->rx_buffer_offset = offset;
	rqe = kmalloc(sizeof(*rqe), GFP_KERNEL);
	if (!rqe)
		return;

	rqe->buffer = buffer;
	rqe->buffer_size = size;
	PRINT_INFO(vif->ndev, RX_DBG,
		   "rxq entery Size= %d Address= %p\n",
		   rqe->buffer_size, rqe->buffer);
	wilc_wlan_rxq_add(wilc, rqe);
	wilc_wlan_handle_rxq(wilc);
}

void wilc_handle_isr(struct wilc *wilc)
{
	u32 int_status;
	struct wilc_vif *vif = wilc->vif[0];

	acquire_bus(wilc, ACQUIRE_AND_WAKEUP, PWR_DEV_SRC_WIFI);
	wilc->hif_func->hif_read_int(wilc, &int_status);

	if (int_status & DATA_INT_EXT)
		wilc_wlan_handle_isr_ext(wilc, int_status);

	if (!(int_status & (ALL_INT_EXT))) {
		PRINT_WRN(vif->ndev, TX_DBG, ">> UNKNOWN_INTERRUPT - 0x%08x\n",
			  int_status);
		wilc_unknown_isr_ext(wilc);
	}

	release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
}

int wilc_wlan_firmware_download(struct wilc *wilc, const u8 *buffer,
				u32 buffer_size)
{
	u32 offset;
	u32 addr, size, size2, blksz;
	u8 *dma_buffer;
	int ret = 0;
	u32 reg = 0;
	struct wilc_vif *vif = wilc->vif[0];

	blksz = BIT(12);

	dma_buffer = kmalloc(blksz, GFP_KERNEL);
	if (!dma_buffer) {
		PRINT_ER(vif->ndev,
			 "Can't allocate buffer for fw download IO error\n");
		return -EIO;
	}

	offset = 0;
	PRINT_INFO(vif->ndev, INIT_DBG, "Downloading firmware size = %d\n",
		   buffer_size);

	acquire_bus(wilc, ACQUIRE_AND_WAKEUP, PWR_DEV_SRC_WIFI);

	wilc->hif_func->hif_read_reg(wilc, WILC_GLB_RESET_0, &reg);
	reg &= ~(1ul << 10);
	ret = wilc->hif_func->hif_write_reg(wilc, WILC_GLB_RESET_0, reg);
	wilc->hif_func->hif_read_reg(wilc, WILC_GLB_RESET_0, &reg);
	if ((reg & (1ul << 10)) != 0)
		PRINT_ER(vif->ndev, "Failed to reset Wifi CPU\n");

	release_bus(wilc, RELEASE_ONLY, PWR_DEV_SRC_WIFI);
	do {
		memcpy(&addr, &buffer[offset], 4);
		memcpy(&size, &buffer[offset + 4], 4);
		addr = cpu_to_le32(addr);
		size = cpu_to_le32(size);
		acquire_bus(wilc, ACQUIRE_AND_WAKEUP, PWR_DEV_SRC_WIFI);
		offset += 8;
		while (((int)size) && (offset < buffer_size)) {
			if (size <= blksz)
				size2 = size;
			else
				size2 = blksz;

			memcpy(dma_buffer, &buffer[offset], size2);
			ret = wilc->hif_func->hif_block_tx(wilc, addr,
							   dma_buffer, size2);
			if (!ret)
				break;

			addr += size2;
			offset += size2;
			size -= size2;
		}
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);

		if (!ret) {
			ret = -EIO;
			PRINT_ER(vif->ndev, "Bus error\n");
			goto fail;
		}
		PRINT_INFO(vif->ndev, INIT_DBG, "Offset = %d\n", offset);
	} while (offset < buffer_size);

fail:

	kfree(dma_buffer);

	return (ret < 0) ? ret : 0;
}

int wilc_wlan_start(struct wilc *wilc)
{
	u32 reg = 0;
	int ret;
	struct wilc_vif *vif = wilc->vif[0];

	if (wilc->io_type == HIF_SDIO ||
	    wilc->io_type == HIF_SDIO_GPIO_IRQ)
		reg |= BIT(3);
	else if (wilc->io_type == HIF_SPI)
		reg = 1;

	acquire_bus(wilc, ACQUIRE_AND_WAKEUP,PWR_DEV_SRC_WIFI);
	ret = wilc->hif_func->hif_write_reg(wilc, WILC_VMM_CORE_CFG, reg);
	if (!ret) {
		PRINT_ER(vif->ndev,
			 "[wilc start]: fail write reg vmm_core_cfg...\n");
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
		return -EIO;
	}
	reg = 0;
	if (wilc->io_type == HIF_SDIO_GPIO_IRQ)
		reg |= WILC_HAVE_SDIO_IRQ_GPIO;

	if(wilc->chip == WILC_3000)
		reg |= WILC_HAVE_SLEEP_CLK_SRC_RTC;

	ret = wilc->hif_func->hif_write_reg(wilc, WILC_GP_REG_1, reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "[wilc start]: fail write WILC_GP_REG_1...\n");
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
		return -EIO;
	}

	wilc->hif_func->hif_sync_ext(wilc, NUM_INT_EXT);


	wilc->hif_func->hif_read_reg(wilc, WILC_GLB_RESET_0, &reg);
	if ((reg & BIT(10)) == BIT(10)) {
		reg &= ~BIT(10);
		wilc->hif_func->hif_write_reg(wilc, WILC_GLB_RESET_0, reg);
		wilc->hif_func->hif_read_reg(wilc, WILC_GLB_RESET_0, &reg);
	}

	reg |= BIT(10);
	ret = wilc->hif_func->hif_write_reg(wilc, WILC_GLB_RESET_0, reg);
	wilc->hif_func->hif_read_reg(wilc, WILC_GLB_RESET_0, &reg);

	if (ret >= 0)
		wilc->initialized = 1;
	else
		wilc->initialized = 0;
	release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);

	return (ret < 0) ? ret : 0;
}

int wilc_wlan_stop(struct wilc *wilc)
{
	u32 reg = 0;
	int ret;
	u8 timeout = 10;
	struct wilc_vif *vif = wilc->vif[0];

	acquire_bus(wilc, ACQUIRE_AND_WAKEUP, PWR_DEV_SRC_WIFI);

	/* Clear Wifi mode*/
	ret = wilc->hif_func->hif_read_reg(wilc, WILC_GLOBAL_MODE_CONTROL, &reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "Error while reading reg\n");
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
		return ret;
	}
	
	reg &= ~BIT(0);
	ret = wilc->hif_func->hif_write_reg(wilc, WILC_GLOBAL_MODE_CONTROL, reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "Error while writing reg\n");
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
		return ret;
	}

	/* Configure the power sequencer to ignore WIFI sleep signal on making chip
		sleep decision */
	ret = wilc->hif_func->hif_read_reg(wilc, WILC_PWR_SEQ_MISC_CTRL, &reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "Error while reading reg\n");
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
		return ret;
	}
	
	reg &= ~BIT(28);
	ret = wilc->hif_func->hif_write_reg(wilc, WILC_PWR_SEQ_MISC_CTRL, reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "Error while writing reg\n");
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
		return ret;
	}

	ret = wilc->hif_func->hif_read_reg(wilc, WILC_GLB_RESET_0, &reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "Error while reading reg\n");
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
		return ret;
	}

	reg &= ~BIT(10);
	ret = wilc->hif_func->hif_write_reg(wilc, WILC_GLB_RESET_0, reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "Error while writing reg\n");
		release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
		return ret;
	}

	do {
		ret = wilc->hif_func->hif_read_reg(wilc,
						   WILC_GLB_RESET_0, &reg);
		if (!ret) {
			PRINT_ER(vif->ndev, "Error while reading reg\n");
			release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);
			return ret;
		}
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "Read RESET Reg %x : Retry%d\n", reg, timeout);
		if ((reg & BIT(10))) {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Bit 10 not reset : Retry %d\n", timeout);
			reg &= ~BIT(10);
			ret = wilc->hif_func->hif_write_reg(wilc,
							    WILC_GLB_RESET_0,
							    reg);
			timeout--;
		} else {
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Bit 10 reset after : Retry %d\n", timeout);
			ret = wilc->hif_func->hif_read_reg(wilc,
							   WILC_GLB_RESET_0,
							   &reg);
			if (!ret) {
				PRINT_ER(vif->ndev, "Error while reading reg\n");
				release_bus(wilc, RELEASE_ALLOW_SLEEP,
					    PWR_DEV_SRC_WIFI);
				return ret;
			}
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "Read RESET Reg %x : Retry%d\n", reg,
				   timeout);
			break;
		}

	} while (timeout);

	if (wilc->chip == WILC_1000) {
		reg = (BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(8) | BIT(9) |
		       BIT(26) | BIT(29) | BIT(30) | BIT(31));
	} else {
		reg = (BIT(0) | BIT(2) | BIT(3) | BIT(8) | BIT(9) |
		       BIT(20) | BIT(26) | BIT(29) | BIT(30) | BIT(31));
	}

	wilc->hif_func->hif_read_reg(wilc, WILC_FW_HOST_COMM, &reg);
	reg = BIT(0);

	ret = wilc->hif_func->hif_write_reg(wilc, WILC_FW_HOST_COMM, reg);

	release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);

	return ret;
}

void wilc_wlan_cleanup(struct net_device *dev)
{
	struct txq_entry_t *tqe;
	struct rxq_entry_t *rqe;
	u8 ac;
	struct wilc_vif *vif;
	struct wilc *wilc;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	wilc->quit = 1;
	for (ac = 0; ac < NQUEUES; ac++) {
		do {
			tqe = wilc_wlan_txq_remove_from_head(dev, ac);
			if (!tqe)
				break;
			if (tqe->tx_complete_func)
				tqe->tx_complete_func(tqe->priv, 0);
			kfree(tqe);
		} while (1);
	}

	do {
		rqe = wilc_wlan_rxq_remove(wilc);
		if (!rqe)
			break;
		kfree(rqe);
	} while (1);

	kfree(wilc->rx_buffer);
	wilc->rx_buffer = NULL;
	kfree(wilc->tx_buffer);
	wilc->tx_buffer = NULL;
}

static int wilc_wlan_cfg_commit(struct wilc_vif *vif, int type,
				u32 drv_handler)
{
	struct wilc *wilc = vif->wilc;
	struct wilc_cfg_frame *cfg = &wilc->cfg_frame;
	int total_len = wilc->cfg_frame_offset + 4 + DRIVER_HANDLER_SIZE;
	int seq_no = wilc->cfg_seq_no % 256;
	int driver_handler = (u32)drv_handler;

	if (type == WILC_CFG_SET)
		cfg->wid_header[0] = 'W';
	else
		cfg->wid_header[0] = 'Q';
	cfg->wid_header[1] = seq_no;
	cfg->wid_header[2] = (u8)total_len;
	cfg->wid_header[3] = (u8)(total_len >> 8);
	cfg->wid_header[4] = (u8)driver_handler;
	cfg->wid_header[5] = (u8)(driver_handler >> 8);
	cfg->wid_header[6] = (u8)(driver_handler >> 16);
	cfg->wid_header[7] = (u8)(driver_handler >> 24);
	wilc->cfg_seq_no = seq_no;

	if (!wilc_wlan_txq_add_cfg_pkt(vif, &cfg->wid_header[0], total_len))
		return -1;

	return 0;
}

int wilc_wlan_cfg_set(struct wilc_vif *vif, int start, u16 wid, u8 *buffer,
		      u32 buffer_size, int commit, u32 drv_handler)
{
	u32 offset;
	int ret_size;
	struct wilc *wilc = vif->wilc;

	if (wilc->cfg_frame_in_use)
		return 0;

	if (start)
		wilc->cfg_frame_offset = 0;

	offset = wilc->cfg_frame_offset;
	ret_size = wilc_wlan_cfg_set_wid(vif, wilc->cfg_frame.frame, offset,
					 wid, buffer, buffer_size);
	offset += ret_size;
	wilc->cfg_frame_offset = offset;

	if (!commit)
		return ret_size;

	PRINT_INFO(vif->ndev, TX_DBG,
		   "[WILC]PACKET Commit with sequence number%d\n",
		   wilc->cfg_seq_no);
	wilc->cfg_frame_in_use = 1;

	if (wilc_wlan_cfg_commit(vif, WILC_CFG_SET, drv_handler))
		ret_size = 0;

	if (!wait_for_completion_timeout(&wilc->cfg_event,
					 msecs_to_jiffies(CFG_PKTS_TIMEOUT))) {
		PRINT_ER(vif->ndev, "Set Timed Out\n");
		ret_size = 0;
	}

	wilc->cfg_frame_in_use = 0;
	wilc->cfg_frame_offset = 0;
	wilc->cfg_seq_no += 1;

	return ret_size;
}

int wilc_wlan_cfg_get(struct wilc_vif *vif, int start, u16 wid, int commit,
		      u32 drv_handler)
{
	u32 offset;
	int ret_size;
	struct wilc *wilc = vif->wilc;

	if (wilc->cfg_frame_in_use)
		return 0;

	if (start)
		wilc->cfg_frame_offset = 0;

	offset = wilc->cfg_frame_offset;
	ret_size = wilc_wlan_cfg_get_wid(wilc->cfg_frame.frame, offset, wid);
	offset += ret_size;
	wilc->cfg_frame_offset = offset;

	if (!commit)
		return ret_size;

	wilc->cfg_frame_in_use = 1;

	if (wilc_wlan_cfg_commit(vif, WILC_CFG_QUERY, drv_handler))
		ret_size = 0;

	if (!wait_for_completion_timeout(&wilc->cfg_event,
					 msecs_to_jiffies(CFG_PKTS_TIMEOUT))) {
		PRINT_INFO(vif->ndev, TX_DBG, "Get Timed Out\n");
		ret_size = 0;
	}
	PRINT_INFO(vif->ndev, TX_DBG, "Get Response received\n");
	wilc->cfg_frame_in_use = 0;
	wilc->cfg_frame_offset = 0;
	wilc->cfg_seq_no += 1;

	return ret_size;
}

int wilc_wlan_cfg_get_val(struct wilc_vif *vif, u16 wid, u8 *buffer,
			  u32 buffer_size)
{
	return wilc_wlan_cfg_get_wid_value(vif, wid, buffer, buffer_size);
}
unsigned int cfg_packet_timeout = 0;
extern int wait_for_recovery;
int wilc_send_config_pkt(struct wilc_vif *vif, u8 mode, struct wid *wids,
			 u32 count, u32 drv)
{
	int i;
	int ret = 0;

	if(wait_for_recovery){
		PRINT_INFO(vif->ndev, CORECONFIG_DBG,
			   "Host interface is suspended\n");
		while(wait_for_recovery)
			msleep(300);
		PRINT_INFO(vif->ndev, CORECONFIG_DBG,
			   "Host interface is resumed\n");
	}

	if (mode == GET_CFG) {
		for (i = 0; i < count; i++) {
			PRINT_D(vif->ndev, CORECONFIG_DBG,
				"Sending CFG packet [%d][%d]\n",!i,
				(i == count - 1));
			if (!wilc_wlan_cfg_get(vif, !i,
					       wids[i].id,
					       (i == count - 1),
					       drv)) {
				ret = -ETIMEDOUT;
				PRINT_ER(vif->ndev, "Get Timed out\n");
				break;
			}
		}
		for (i = 0; i < count; i++) {
			wids[i].size = wilc_wlan_cfg_get_val(vif, wids[i].id,
							     wids[i].val,
							     wids[i].size);
		}
	} else if (mode == SET_CFG) {
		for (i = 0; i < count; i++) {
			PRINT_INFO(vif->ndev, CORECONFIG_DBG,
				   "Sending config SET PACKET WID:%x\n",
				   wids[i].id);
			if (!wilc_wlan_cfg_set(vif, !i,
					       wids[i].id,
					       wids[i].val,
					       wids[i].size,
					       (i == count - 1),
					       drv)) {
				ret = -ETIMEDOUT;
				PRINT_ER(vif->ndev, "Set Timed out\n");
				break;
			}
		}
	}
	cfg_packet_timeout = (ret < 0) ? cfg_packet_timeout + 1 : 0;
	return ret;
}

static u32 init_chip(struct net_device *dev)
{
	u32 chipid;
	u32 reg, ret = 0;
	struct wilc_vif *vif;
	struct wilc *wilc;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	acquire_bus(wilc, ACQUIRE_AND_WAKEUP,PWR_DEV_SRC_WIFI);

	chipid = wilc_get_chipid(wilc, true);

	ret = wilc->hif_func->hif_read_reg(wilc, 0x1118, &reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "fail read reg 0x1118\n");
		goto end;
	}

	reg |= BIT(0);
	ret = wilc->hif_func->hif_write_reg(wilc, 0x1118, reg);
	if (!ret) {
		PRINT_ER(vif->ndev, "fail write reg 0x1118\n");
		goto end;
	}
	ret = wilc->hif_func->hif_write_reg(wilc, 0xc0000, 0x71);
	if (!ret) {
		PRINT_ER(vif->ndev, "fail write reg 0xc0000 ...\n");
		goto end;
	}

	if(wilc->chip == WILC_3000) {
		ret = wilc->hif_func->hif_read_reg(wilc, 0x207ac, &reg);
		PRINT_INFO(vif->ndev, INIT_DBG, "Bootrom sts = %x\n", reg);
		ret = wilc->hif_func->hif_write_reg(wilc, 0x4f0000,
						    0x71);
		if (!ret) {
			PRINT_ER(vif->ndev, "fail write reg 0x4f0000 ...\n");
			goto end;
		}
	}

end:
	release_bus(wilc, RELEASE_ALLOW_SLEEP, PWR_DEV_SRC_WIFI);

	return ret;
}

u32 wilc_get_chipid(struct wilc *wilc, bool update)
{
	static u32 chipid;
	int ret;
	u32 tempchipid = 0;

	if (chipid == 0 || update) {
		ret = wilc->hif_func->hif_read_reg(wilc, 0x3b0000,
						     &tempchipid);
		if (!ret) 
			pr_err("[wilc start]: fail read reg 0x3b0000\n");
		if (!ISWILC3000(tempchipid)) {
			wilc->hif_func->hif_read_reg(wilc, 0x1000,
						     &tempchipid);
			if (!ISWILC1000(tempchipid)) {
				chipid = 0;
				return chipid;
			}
			if(tempchipid < 0x1003a0){
				pr_err("WILC1002 isn't suported %x\n", chipid);
				chipid = 0;
				return chipid;
			}
		}
		chipid = tempchipid;
	}

	return chipid;
}

int wilc_wlan_init(struct net_device *dev)
{
	int ret = 0;
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc;

	wilc = vif->wilc;

	wilc->quit = 0;
	
	PRINT_INFO(vif->ndev, INIT_DBG,"Initializing WILC_Wlan\n");

	if(!wilc->hif_func->hif_is_init()) {
		acquire_bus(wilc, ACQUIRE_ONLY , PWR_DEV_SRC_WIFI);
		if (!wilc->hif_func->hif_init(wilc, false)) {
			ret = -EIO;
			release_bus(wilc, RELEASE_ONLY, PWR_DEV_SRC_WIFI);
			goto fail;
		}
		release_bus(wilc, RELEASE_ONLY, PWR_DEV_SRC_WIFI);
	}

	if (!wilc_wlan_cfg_init()) {
		ret = -ENOBUFS;
		goto fail;
	}

	if (!wilc->tx_buffer)
		wilc->tx_buffer = kmalloc(LINUX_TX_SIZE, GFP_KERNEL);

	if (!wilc->tx_buffer) {
		ret = -ENOBUFS;
		PRINT_ER(vif->ndev, "Can't allocate Tx Buffer");
		goto fail;
	}

	if (!wilc->rx_buffer)
		wilc->rx_buffer = kmalloc(LINUX_RX_SIZE, GFP_KERNEL);
	PRINT_D(vif->ndev, TX_DBG, "g_wlan.rx_buffer =%p\n", wilc->rx_buffer);
	if (!wilc->rx_buffer) {
		ret = -ENOBUFS;
		PRINT_ER(vif->ndev, "Can't allocate Rx Buffer");
		goto fail;
	}

	if (!init_chip(dev)) {
		ret = -EIO;
		goto fail;
	}

	return 1;

fail:

	kfree(wilc->rx_buffer);
	wilc->rx_buffer = NULL;
	kfree(wilc->tx_buffer);
	wilc->tx_buffer = NULL;

	return ret;
}
