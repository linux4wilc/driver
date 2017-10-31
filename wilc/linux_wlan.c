#include "wilc_wfi_cfgoperations.h"
#include "wilc_wlan_if.h"
#include "wilc_wlan.h"

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <linux/kthread.h>
#include <linux/firmware.h>

#include <linux/init.h>
#include <linux/netdevice.h>
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
#include <linux/inetdevice.h>
#endif /* DISABLE_PWRSAVE_AND_SCAN_DURING_IP */
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/mutex.h>
#include <linux/completion.h>

#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/pm_runtime.h>

#include "linux_wlan.h"

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
bool g_ignore_PS_state = false;
#define duringIP_TIME		15000
extern struct timer_list wilc_during_ip_timer;

void handle_pwrsave_during_obtainingIP(struct wilc_vif *vif, uint8_t state)
{

	switch(state)
	{	
	case IP_STATE_OBTAINING:
		wilc_optaining_ip = true;

		/* Set this flag to avoid storing the disabled case of PS which occurs duringIP */
		g_ignore_PS_state = true;

		wilc_set_power_mgmt(vif, 0, 0);

		/* Start the DuringIPTimer */
		wilc_during_ip_timer.data = (uint32_t)vif;
		mod_timer(&wilc_during_ip_timer, (jiffies + msecs_to_jiffies(20000)));

		break;
	
	case IP_STATE_OBTAINED:
		wilc_optaining_ip = false;

		/* Recover PS previous state */
		if(wilc_enable_ps == true)
		{
			wilc_set_power_mgmt(vif, vif->pwrsave_current_state, 0);
		}

		del_timer(&wilc_during_ip_timer);	

		break;
	
	case IP_STATE_GO_ASSIGNING:

		/* Set the wilc_optaining_ip flag */
		wilc_optaining_ip = true;

		/* Start the DuringIPTimer */
		wilc_during_ip_timer.data = (unsigned long)vif;
		mod_timer(&wilc_during_ip_timer, (jiffies + msecs_to_jiffies(duringIP_TIME)));

		break;
	
	default: //IP_STATE_DEFAULT

		/* Clear the wilc_optaining_ip flag */
		wilc_optaining_ip = false;

		/* Stop the DuringIPTimer */
		del_timer(&wilc_during_ip_timer);

		/* Disable PS */
		if(vif)
		{
			wilc_set_power_mgmt(vif, 0, 0);
		}

		break;
	}
}

void set_obtaining_IP_flag(bool val)
{
	wilc_optaining_ip = val;
}

void store_power_save_current_state(struct wilc_vif *vif, bool val)
{
	if(g_ignore_PS_state)
	{
		g_ignore_PS_state = false;
		return;
	}
	vif->pwrsave_current_state = val;
}

void clear_duringIP(unsigned long arg)
{
	struct wilc_vif *vif = (struct wilc_vif *)arg;
	
	netdev_err(vif->ndev, "Unable to Obtain IP\n");

	/* Clear the wilc_optaining_ip flag */
	wilc_optaining_ip = false;

	netdev_info(vif->ndev, "Recover the state of the PS = %d\n", vif->pwrsave_current_state);

	/* Recover PS previous state */
	if(wilc_enable_ps == true)
	{
		wilc_set_power_mgmt(vif, vif->pwrsave_current_state, 0);
	}
}

static int dev_state_ev_handler(struct notifier_block *this,
				unsigned long event, void *ptr);

static struct notifier_block g_dev_notifier = {
	.notifier_call = dev_state_ev_handler
};
#endif /* DISABLE_PWRSAVE_AND_SCAN_DURING_IP */

static int wlan_deinit_locks(struct net_device *dev);
static void wlan_deinitialize_threads(struct net_device *dev);

void frmw_to_linux(struct wilc *wilc, u8 *buff, u32 size, u32 pkt_offset, u8
		   status);
static void linux_wlan_tx_complete(void *priv, int status);
static int  mac_init_fn(struct net_device *ndev);
static struct net_device_stats *mac_stats(struct net_device *dev);
static int  mac_ioctl(struct net_device *ndev, struct ifreq *req, int cmd);
static int wilc_mac_open(struct net_device *ndev);
static int wilc_mac_close(struct net_device *ndev);
static void wilc_set_multicast_list(struct net_device *dev);

bool wilc_enable_ps = true;

static const struct net_device_ops wilc_netdev_ops = {
	.ndo_init = mac_init_fn,
	.ndo_open = wilc_mac_open,
	.ndo_stop = wilc_mac_close,
	.ndo_start_xmit = wilc_mac_xmit,
	.ndo_do_ioctl = mac_ioctl,
	.ndo_get_stats = mac_stats,
	.ndo_set_rx_mode  = wilc_set_multicast_list,
};

int debug_running = false;
int recovery_on = 0;
int wait_for_recovery = 0;
static int debug_thread(void *arg)
{
	struct wilc_vif *vif;
	struct wilc *wl;
	struct wilc_priv *priv;
	struct net_device *dev = arg;
	signed long timeout;
	struct host_if_drv *hif_drv;
	int i = 0;
	
	vif = netdev_priv(dev);

	priv = wiphy_priv(vif->ndev->ieee80211_ptr->wiphy);
	hif_drv = (struct host_if_drv *)priv->hif_drv;

	if(!vif)
		return -1;

	wl = vif->wilc;
	if(!wl)
		return -1;

	complete(&wl->debug_thread_started);

	while(1){
		if (wl->initialized) {
			if (wait_for_completion_timeout(&wl->debug_thread_started, msecs_to_jiffies(6000))) {
				while (!kthread_should_stop())
					schedule();

				return 0;
			}

			if (debug_running) {
				netdev_info(dev, "*** Debug Thread Running ***\n");
				if(cfg_packet_timeout >= 5){
					netdev_info(dev, "<Recover>\n");
					cfg_packet_timeout = 0;
					timeout = 10;
					recovery_on = 1;
					wait_for_recovery = 1;
					for (i = 0; i < NUM_CONCURRENT_IFC; i++)
						wilc_mac_close(wl->vif[i]->ndev);
					for (i = NUM_CONCURRENT_IFC; i > 0; i--)
						while (wilc_mac_open(wl->vif[i - 1]->ndev) && --timeout)
							msleep(100);

					if(hif_drv->hif_state == HOST_IF_CONNECTED) {
						struct disconnect_info strDisconnectNotifInfo;

						memset(&strDisconnectNotifInfo, 0, sizeof(struct disconnect_info));
						if (hif_drv->usr_scan_req.scan_result) {
							del_timer(&hif_drv->scan_timer);
							Handle_ScanDone(vif, SCAN_EVENT_ABORTED);
						}
						strDisconnectNotifInfo.reason = 0;
						strDisconnectNotifInfo.ie = NULL;
						strDisconnectNotifInfo.ie_len = 0;

						if (hif_drv->usr_conn_req.conn_result) {
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP

							handle_pwrsave_during_obtainingIP(vif, IP_STATE_DEFAULT);
#endif

							hif_drv->usr_conn_req.conn_result(CONN_DISCONN_EVENT_DISCONN_NOTIF,
											  NULL,
											  0,
											  &strDisconnectNotifInfo,
											  hif_drv->usr_conn_req.arg);
						} else {
							netdev_err(vif->ndev, "Connect result NULL\n");
						}
						eth_zero_addr(hif_drv->assoc_bssid);

						hif_drv->usr_conn_req.ssid_len = 0;
						kfree(hif_drv->usr_conn_req.ssid);
						hif_drv->usr_conn_req.ssid = NULL;
						kfree(hif_drv->usr_conn_req.bssid);
						hif_drv->usr_conn_req.bssid = NULL;
						hif_drv->usr_conn_req.ies_len = 0;
						kfree(hif_drv->usr_conn_req.ies);
						hif_drv->usr_conn_req.ies = NULL;

						if (join_req != NULL && join_req_vif == vif) {
							kfree(join_req);
							join_req = NULL;
						}
						if (info_element != NULL && join_req_vif == vif) {
							kfree(info_element);
							info_element = NULL;
						}

						hif_drv->hif_state = HOST_IF_IDLE;
						scan_while_connected = false;

					}
					recovery_on = 0;
				}
			}
		} else {
			msleep(1000);
		}
	}
	return 0;
}

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
static int dev_state_ev_handler(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct in_ifaddr *dev_iface = ptr;
	struct wilc_priv *priv;
	struct host_if_drv *hif_drv;
	struct net_device *dev;
	u8 *ip_addr_buf;
	struct wilc_vif *vif;
	u8 null_ip[4] = {0};

	if (!dev_iface || !dev_iface->ifa_dev || !dev_iface->ifa_dev->dev)
		return NOTIFY_DONE;

	if (memcmp(dev_iface->ifa_label, IFC_0, 5) &&
	    memcmp(dev_iface->ifa_label, IFC_1, 4))
		return NOTIFY_DONE;

	dev  = (struct net_device *)dev_iface->ifa_dev->dev;
	if (!dev->ieee80211_ptr || !dev->ieee80211_ptr->wiphy)
		return NOTIFY_DONE;

	priv = wiphy_priv(dev->ieee80211_ptr->wiphy);
	if (!priv)
		return NOTIFY_DONE;

	hif_drv = (struct host_if_drv *)priv->hif_drv;
	vif = netdev_priv(dev);
	if (!vif || !hif_drv)
		return NOTIFY_DONE;

	switch (event) {
	case NETDEV_UP:
		if (vif->iftype == STATION_MODE || vif->iftype == CLIENT_MODE) {
			hif_drv->IFC_UP = 1;

			handle_pwrsave_during_obtainingIP(vif, IP_STATE_OBTAINED);
		}
		netdev_dbg(dev, "[%s] Up IP\n", dev_iface->ifa_label);

		ip_addr_buf = (char *)&dev_iface->ifa_address;
		netdev_dbg(dev, "IP add=%d:%d:%d:%d\n",
			   ip_addr_buf[0], ip_addr_buf[1],
			   ip_addr_buf[2], ip_addr_buf[3]);
		break;

	case NETDEV_DOWN:
		if (vif->iftype == STATION_MODE || vif->iftype == CLIENT_MODE) {
			hif_drv->IFC_UP = 0;
			handle_pwrsave_during_obtainingIP(vif, IP_STATE_DEFAULT);
		}

		
		wilc_resolve_disconnect_aberration(vif);

		netdev_dbg(dev, "[%s] Down IP\n", dev_iface->ifa_label);

		ip_addr_buf = null_ip;
		netdev_dbg(dev, "IP add=%d:%d:%d:%d\n",
			   ip_addr_buf[0], ip_addr_buf[1],
			   ip_addr_buf[2], ip_addr_buf[3]);
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}
#endif /* DISABLE_PWRSAVE_AND_SCAN_DURING_IP */

static irqreturn_t isr_uh_routine(int irq, void *user_data)
{
	struct wilc_vif *vif;
	struct wilc *wilc;
	struct net_device *dev = user_data;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	if (wilc->close) {
		netdev_err(dev, "Can't handle UH interrupt\n");
		return IRQ_HANDLED;
	}
	return IRQ_WAKE_THREAD;
}

static irqreturn_t isr_bh_routine(int irq, void *userdata)
{
	struct wilc_vif *vif;
	struct wilc *wilc;
	struct net_device *dev = userdata;

	vif = netdev_priv(userdata);
	wilc = vif->wilc;

	if (wilc->close) {
		netdev_err(dev, "Can't handle BH interrupt\n");
		return IRQ_HANDLED;
	}

	wilc_handle_isr(wilc);

	return IRQ_HANDLED;
}

static int init_irq(struct net_device *dev)
{
	int ret = 0;
	struct wilc_vif *vif;
	struct wilc *wl;

	vif = netdev_priv(dev);
	wl = vif->wilc;

	if ((gpio_request(wl->gpio, "WILC_INTR") == 0) &&
	    (gpio_direction_input(wl->gpio) == 0)) {
		wl->dev_irq_num = gpio_to_irq(wl->gpio);
	} else {
		ret = -1;
		netdev_err(dev, "could not obtain gpio for WILC_INTR\n");
	}

	if (ret != -1 && request_threaded_irq(wl->dev_irq_num,
					      isr_uh_routine,
					      isr_bh_routine,
					      IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					      "WILC_IRQ", dev) < 0) {
		netdev_err(dev, "Failed to request IRQ GPIO: %d\n", wl->gpio);
		gpio_free(wl->gpio);
		ret = -1;
	} else {
		netdev_info(dev,
			   "IRQ request succeeded IRQ-NUM= %d on GPIO: %d\n",
			   wl->dev_irq_num, wl->gpio);
	}

	return ret;
}

static void deinit_irq(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wilc;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	/* Deinitialize IRQ */
	if (wilc->dev_irq_num) {
		free_irq(wilc->dev_irq_num, wilc);
		gpio_free(wilc->gpio);
	}
}

void wilc_mac_indicate(struct wilc *wilc, int flag)
{
	int status;

	if (flag == WILC_MAC_INDICATE_STATUS) {
		wilc_wlan_cfg_get_val(WID_STATUS,
				      (unsigned char *)&status, 4);
		if (wilc->mac_status == WILC_MAC_STATUS_INIT) {
			wilc->mac_status = status;
			complete(&wilc->sync_event);
		} else {
			wilc->mac_status = status;
		}
	}
}

void free_eap_buff_params(void *vp)
{
	struct wilc_priv *priv;

	priv = (struct wilc_priv *)vp;

	if(priv->buffered_eap) {
		if(priv->buffered_eap->buff) {
			kfree(priv->buffered_eap->buff);
			priv->buffered_eap->buff = NULL;
		}
		kfree(priv->buffered_eap);
		priv->buffered_eap = NULL;
	}
}

void eap_buff_timeout(unsigned long user)
{
    u8 null_bssid[ETH_ALEN] = {0};
    static u8 timeout = 5;
    int status = -1; 
    struct wilc_priv *priv;
    struct wilc_vif *vif ;
	
    priv = (struct wilc_priv *)user;
    vif = netdev_priv(priv->dev);
    if (!(memcmp(priv->au8AssociatedBss, null_bssid, ETH_ALEN)) && (timeout-- > 0)) {
            eap_buff_timer.data = (unsigned long)user;
            mod_timer(&eap_buff_timer,(jiffies + msecs_to_jiffies(10)));
            return;
    }
    del_timer(&eap_buff_timer);
    timeout = 5;

    status = wilc_send_buffered_eap(vif,
                                    frmw_to_linux,
                                    free_eap_buff_params,
                                    priv->buffered_eap->buff,
                                    priv->buffered_eap->size,
                                    priv->buffered_eap->pkt_offset,
                                    (void *)priv);
	if (status)
		netdev_err(vif->ndev, "Failed so send buffered eap\n");
}



static struct net_device *get_if_handler(struct wilc *wilc, u8 *mac_header)
{
	u8 *bssid, *bssid1;
	int i = 0;

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
	}

	return NULL;
}

int wilc_wlan_set_bssid(struct net_device *wilc_netdev, u8 *bssid, u8 mode)
{
	struct wilc_vif *vif = netdev_priv(wilc_netdev);
	struct wilc *wilc;
	u8 i = 0;

	wilc = vif->wilc;

	for (i = 0; i <= wilc->vif_num; i++){
		if (wilc_netdev == wilc->vif[i]->ndev) {
			memcpy(wilc->vif[i]->bssid, bssid, 6);
			wilc->vif[i]->iftype = mode;
		}
	}
	return 0;
}

int wilc_wlan_get_num_conn_ifcs(struct wilc *wilc)
{
	u8 i = 0;
	u8 null_bssid[6] = {0};
	u8 ret_val = 0;

	for (i = 0; i <= wilc->vif_num; i++)
		if (memcmp(wilc->vif[i]->bssid, null_bssid, 6))
			ret_val++;

	return ret_val;
}
EXPORT_SYMBOL_GPL(wilc_wlan_get_num_conn_ifcs);

struct net_device* wilc_get_if_netdev(struct wilc *wilc, uint8_t ifc)
{
	return wilc->vif[ifc]->ndev;
}

struct host_if_drv * wilc_get_drv_handler_by_ifc(struct wilc *wilc, uint8_t ifc)
{
	return wilc->vif[ifc]->hif_drv;
}

#define TX_BACKOFF_WEIGHT_INCR_STEP (1)
#define TX_BACKOFF_WEIGHT_DECR_STEP (1)
#define TX_BACKOFF_WEIGHT_MAX (0)
#define TX_BACKOFF_WEIGHT_MIN (0)
#define TX_BACKOFF_WEIGHT_UNIT_MS (1)


static int linux_wlan_txq_task(void *vp)
{
	int ret, txq_count;
	struct wilc_vif *vif;
	struct wilc *wl;
	struct net_device *dev = vp;
	int backoff_weight = TX_BACKOFF_WEIGHT_MIN;
	signed long timeout;

	vif = netdev_priv(dev);
	wl = vif->wilc;

	complete(&wl->txq_thread_started);
	while (1) {
		wait_for_completion(&wl->txq_event);

		if (wl->close) {
			complete(&wl->txq_thread_started);

			while (!kthread_should_stop())
				schedule();
			break;
		}
		do {
			ret = wilc_wlan_handle_txq(dev, &txq_count);
			if (txq_count < FLOW_CONTROL_LOWER_THRESHOLD) {
				if (netif_queue_stopped(wl->vif[0]->ndev))
					netif_wake_queue(wl->vif[0]->ndev);
				if (netif_queue_stopped(wl->vif[1]->ndev))
					netif_wake_queue(wl->vif[1]->ndev);
			}

			if (ret == WILC_TX_ERR_NO_BUF) {
				timeout = msecs_to_jiffies(TX_BACKOFF_WEIGHT_UNIT_MS << backoff_weight);
				do {
					/* Back off from sending packets for some time. */
					/* schedule_timeout will allow RX task to run and free buffers.*/
					/*Setting state to TASK_INTERRUPTIBLE will put the thread back to CPU*/
					/*running queue when it's signaled even if 'timeout' isn't elapsed.*/
					/*This gives faster chance for reserved SK buffers to be freed*/
					set_current_state(TASK_INTERRUPTIBLE);
					timeout = schedule_timeout(timeout);
					} while(/*timeout*/0);
				backoff_weight += TX_BACKOFF_WEIGHT_INCR_STEP;
				if (backoff_weight > TX_BACKOFF_WEIGHT_MAX)
					backoff_weight = TX_BACKOFF_WEIGHT_MAX;
			} else if (backoff_weight > TX_BACKOFF_WEIGHT_MIN) {
				backoff_weight -= TX_BACKOFF_WEIGHT_DECR_STEP;
				if (backoff_weight < TX_BACKOFF_WEIGHT_MIN)
					backoff_weight = TX_BACKOFF_WEIGHT_MIN;
			}

		} while (ret == WILC_TX_ERR_NO_BUF && !wl->close);
	}
	return 0;
}

int wilc_wlan_get_firmware(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wilc;
	int chip_id, ret = 0;
	const struct firmware *wilc_firmware;
	char *firmware;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	chip_id = wilc_get_chipid(wilc, false);

	if (chip_id == 0x3000d0) {
		netdev_info(dev, "Detect chip WILC3000\n");
		firmware = FIRMWARE_WILC3000_WIFI;
	}
	else if (chip_id == 0x1003a0) {
		netdev_info(dev, "Detect chip WILC1000\n");
		firmware = FIRMWARE_WILC1000_WIFi;
	} else {
		return -1;
	}

	netdev_info(dev, "loading firmware %s\n", firmware);

	if (!(&vif->ndev->dev)) {
		netdev_err(dev, "Dev  is NULL\n");
		goto _fail_;
	}

	if (request_firmware(&wilc_firmware, firmware, wilc->dev) != 0) {
		netdev_err(dev, "%s - firmware not available\n", firmware);
		ret = -1;
		goto _fail_;
	}

	wilc->firmware = wilc_firmware;

_fail_:

	return ret;
}

static int linux_wlan_start_firmware(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wilc;
	int ret = 0;

	vif = netdev_priv(dev);
	wilc = vif->wilc;
#ifdef PREVENT_SDIO_HOST_FROM_SUSPEND
	func = dev_to_sdio_func(wilc->dev);
	pm_runtime_get_sync(mmc_dev(func->card->host));
#endif

	ret = wilc_wlan_start(wilc);
	if (ret < 0) {
		netdev_err(dev, "Failed to start Firmware\n");
		return ret;
	}

	if (!wait_for_completion_timeout(&wilc->sync_event,
					 msecs_to_jiffies(1000)))
		return -ETIME;

	return 0;
}

static int wilc_firmware_download(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wilc;
	int ret = 0;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	if (!wilc->firmware) {
		netdev_err(dev, "Firmware buffer is NULL\n");
		ret = -ENOBUFS;
	}

	ret = wilc_wlan_firmware_download(wilc, wilc->firmware->data,
					  wilc->firmware->size);
	if (ret < 0)
		goto _fail_;

	netdev_dbg(dev, "Download Succeeded\n");
	
_fail_:
	release_firmware(wilc->firmware);
	wilc->firmware = NULL;

	return ret;
}

static int linux_wlan_init_test_config(struct net_device *dev,
				       struct wilc_vif *vif)
{
	unsigned char c_val[64];
	struct wilc *wilc = vif->wilc;
	struct wilc_priv *priv;
	struct host_if_drv *hif_drv;

	netdev_dbg(dev, "Start configuring Firmware\n");
	priv = wiphy_priv(dev->ieee80211_ptr->wiphy);
	hif_drv = (struct host_if_drv *)priv->hif_drv;
	netdev_dbg(dev, "Host = %p\n", hif_drv);
	wilc_get_chipid(wilc, false);

	*(int *)c_val = (unsigned int)vif->iftype;

	if (!wilc_wlan_cfg_set(vif, 1, WID_SET_OPERATION_MODE, c_val, 4, 0, 0))
		goto _fail_;

	c_val[0] = 0;
	if (!wilc_wlan_cfg_set(vif, 0, WID_PC_TEST_MODE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = INFRASTRUCTURE;
	if (!wilc_wlan_cfg_set(vif, 0, WID_BSS_TYPE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = RATE_AUTO;
	if (!wilc_wlan_cfg_set(vif, 0, WID_CURRENT_TX_RATE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = G_MIXED_11B_2_MODE;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11G_OPERATING_MODE, c_val, 1, 0,
			       0))
		goto _fail_;

	c_val[0] = G_AUTO_PREAMBLE;
	if (!wilc_wlan_cfg_set(vif, 0, WID_PREAMBLE, c_val, 1, 0, 0))
		goto _fail_;
	
	c_val[0] = AUTO_PROT;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_PROT_MECH, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = ACTIVE_SCAN;
	if (!wilc_wlan_cfg_set(vif, 0, WID_SCAN_TYPE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = SITE_SURVEY_OFF;
	if (!wilc_wlan_cfg_set(vif, 0, WID_SITE_SURVEY, c_val, 1, 0, 0))
		goto _fail_;

	*((int *)c_val) = 0xffff;
	if (!wilc_wlan_cfg_set(vif, 0, WID_RTS_THRESHOLD, c_val, 2, 0, 0))
		goto _fail_;

	*((int *)c_val) = 2346;
	if (!wilc_wlan_cfg_set(vif, 0, WID_FRAG_THRESHOLD, c_val, 2, 0, 0))
		goto _fail_;

	c_val[0] = 0;
	if (!wilc_wlan_cfg_set(vif, 0, WID_BCAST_SSID, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = 1;
	if (!wilc_wlan_cfg_set(vif, 0, WID_QOS_ENABLE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = NO_POWERSAVE;
	if (!wilc_wlan_cfg_set(vif, 0, WID_POWER_MANAGEMENT, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = NO_ENCRYPT;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11I_MODE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = OPEN_SYSTEM;
	if (!wilc_wlan_cfg_set(vif, 0, WID_AUTH_TYPE, c_val, 1, 0, 0))
		goto _fail_;

	strcpy(c_val, "123456790abcdef1234567890");
	if (!wilc_wlan_cfg_set(vif, 0, WID_WEP_KEY_VALUE, c_val,
			       (strlen(c_val) + 1), 0, 0))
		goto _fail_;

	strcpy(c_val, "12345678");
	if (!wilc_wlan_cfg_set(vif, 0, WID_11I_PSK, c_val, (strlen(c_val)), 0,
			       0))
		goto _fail_;

	strcpy(c_val, "password");
	if (!wilc_wlan_cfg_set(vif, 0, WID_1X_KEY, c_val, (strlen(c_val) + 1),
			       0, 0))
		goto _fail_;

	c_val[0] = 192;
	c_val[1] = 168;
	c_val[2] = 1;
	c_val[3] = 112;
	if (!wilc_wlan_cfg_set(vif, 0, WID_1X_SERV_ADDR, c_val, 4, 0, 0))
		goto _fail_;

	c_val[0] = 3;
	if (!wilc_wlan_cfg_set(vif, 0, WID_LISTEN_INTERVAL, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = 3;
	if (!wilc_wlan_cfg_set(vif, 0, WID_DTIM_PERIOD, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = NORMAL_ACK;
	if (!wilc_wlan_cfg_set(vif, 0, WID_ACK_POLICY, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = 0;
	if (!wilc_wlan_cfg_set(vif, 0, WID_USER_CONTROL_ON_TX_POWER, c_val, 1,
			       0, 0))
		goto _fail_;

	c_val[0] = 48;
	if (!wilc_wlan_cfg_set(vif, 0, WID_TX_POWER_LEVEL_11A, c_val, 1, 0,
			       0))
		goto _fail_;

	c_val[0] = 28;
	if (!wilc_wlan_cfg_set(vif, 0, WID_TX_POWER_LEVEL_11B, c_val, 1, 0,
			       0))
		goto _fail_;

	*((int *)c_val) = 100;
	if (!wilc_wlan_cfg_set(vif, 0, WID_BEACON_INTERVAL, c_val, 2, 0, 0))
		goto _fail_;

	c_val[0] = REKEY_DISABLE;
	if (!wilc_wlan_cfg_set(vif, 0, WID_REKEY_POLICY, c_val, 1, 0, 0))
		goto _fail_;

	*((int *)c_val) = 84600;
	if (!wilc_wlan_cfg_set(vif, 0, WID_REKEY_PERIOD, c_val, 4, 0, 0))
		goto _fail_;

	*((int *)c_val) = 500;
	if (!wilc_wlan_cfg_set(vif, 0, WID_REKEY_PACKET_COUNT, c_val, 4, 0,
			       0))
		goto _fail_;

	c_val[0] = 1;
	if (!wilc_wlan_cfg_set(vif, 0, WID_SHORT_SLOT_ALLOWED, c_val, 1, 0,
			       0))
		goto _fail_;

	c_val[0] = G_SELF_CTS_PROT;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_ERP_PROT_TYPE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = 1;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_ENABLE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = HT_MIXED_MODE;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_OPERATING_MODE, c_val, 1, 0,
			       0))
		goto _fail_;

	c_val[0] = 1;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_TXOP_PROT_DISABLE, c_val, 1, 0,
			       0))
		goto _fail_;

	c_val[0] = DETECT_PROTECT_REPORT;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_OBSS_NONHT_DETECTION, c_val, 1,
			       0, 0))
		goto _fail_;

	c_val[0] = RTS_CTS_NONHT_PROT;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_HT_PROT_TYPE, c_val, 1, 0, 0))
		goto _fail_;

	c_val[0] = 0;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_RIFS_PROT_ENABLE, c_val, 1, 0,
			       0))
		goto _fail_;
	
	c_val[0] = 7;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_CURRENT_TX_MCS, c_val, 1, 0,
			       0))
		goto _fail_;

	c_val[0] = 1;
	if (!wilc_wlan_cfg_set(vif, 0, WID_11N_IMMEDIATE_BA_ENABLED, c_val, 1,
			       1, 0))
		goto _fail_;

	return 0;

_fail_:
	return -1;
}

void wilc_wlan_deinitialize(struct net_device *dev)
{
	int ret;
	struct wilc_vif *vif;
	struct wilc *wl;

	vif = netdev_priv(dev);
	wl = vif->wilc;

	if (wl->initialized) {
		netdev_info(dev, "Deinitializing wilc...\n");

		if (!wl) {
			netdev_err(dev, "wl is NULL\n");
			return;
		}

		if(wl->hif_func->disable_interrupt) {
			mutex_lock(&wl->hif_cs);
			wl->hif_func->disable_interrupt(wl);
			mutex_unlock(&wl->hif_cs);
		}

		if (&wl->txq_event)
			complete(&wl->txq_event);

		wlan_deinitialize_threads(dev);
		deinit_irq(dev);

		ret = wilc_wlan_stop(wl);
		if(ret == 0)
			PRINT_ER("failed in wlan_stop\n");

		wilc_wlan_cleanup(dev);

		wlan_deinit_locks(dev);

		wl->initialized = false;

		netdev_info(dev, "wilc deinitialization Done\n");
	} else {
		netdev_info(dev, "wilc is not initialized\n");
	}
}

static int wlan_init_locks(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wl;

	vif = netdev_priv(dev);
	wl = vif->wilc;

	mutex_init(&wl->rxq_cs);

	spin_lock_init(&wl->txq_spinlock);
	mutex_init(&wl->txq_add_to_head_cs);

	init_completion(&wl->txq_event);
	init_completion(&wl->cfg_event);
	init_completion(&wl->sync_event);
	init_completion(&wl->txq_thread_started);
	init_completion(&wl->debug_thread_started);

	return 0;
}

static int wlan_deinit_locks(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wilc;

	vif = netdev_priv(dev);
	wilc = vif->wilc;
	

	if (&wilc->hif_cs)
		mutex_destroy(&wilc->hif_cs);
	
	if (&wilc->rxq_cs)
		mutex_destroy(&wilc->rxq_cs);

	return 0;
}

static int wlan_initialize_threads(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wilc;

	vif = netdev_priv(dev);
	wilc = vif->wilc;

	wilc->txq_thread = kthread_run(linux_wlan_txq_task, (void *)dev,
				     "K_TXQ_TASK");
	if (IS_ERR(wilc->txq_thread)) {
		netdev_err(dev, "couldn't create TXQ thread\n");
		wilc->close = 1;
		return PTR_ERR(wilc->txq_thread);
	}
	wait_for_completion(&wilc->txq_thread_started);

	if (!debug_running) {
		wilc->debug_thread = kthread_run(debug_thread,(void *)dev,
			"WILC_DEBUG");
		if(IS_ERR(wilc->debug_thread)){
			netdev_err(dev, "couldn't create debug thread\n");
			wilc->close = 1;
			kthread_stop(wilc->txq_thread);
			return PTR_ERR(wilc->debug_thread);
		}
		debug_running = true;
		wait_for_completion(&wilc->debug_thread_started);
	}

	return 0;
}

static void wlan_deinitialize_threads(struct net_device *dev)
{
	struct wilc_vif *vif;
	struct wilc *wl;

	vif = netdev_priv(dev);
	wl = vif->wilc;
	
	if (!recovery_on) {
		debug_running = false;
		if (&wl->debug_thread_started)
			complete(&wl->debug_thread_started);
		if (wl->debug_thread) {
			kthread_stop(wl->debug_thread);
			wl->debug_thread = NULL;
		}
	}

	wl->close = 1;

	if (&wl->txq_event)
		complete(&wl->txq_event);

	if (wl->txq_thread) {
		kthread_stop(wl->txq_thread);
		wl->txq_thread = NULL;
	}
}

int wilc_wlan_initialize(struct net_device *dev, struct wilc_vif *vif)
{
	int ret = 0;
	struct wilc *wl = vif->wilc;

	if (!wl->initialized) {
		wl->mac_status = WILC_MAC_STATUS_INIT;
		wl->close = 0;
		wl->initialized = 0;

		wlan_init_locks(dev);
		ret = wilc_wlan_init(dev);
		if (ret < 0) {
			netdev_err(dev, "Initializing WILC_Wlan FAILED\n");
			ret = -EIO;
			goto _fail_locks_;
		}

		if (wl->gpio >= 0 && init_irq(dev)) {
			ret = -EIO;
			goto _fail_locks_;
		}

		ret = wlan_initialize_threads(dev);
		if (ret < 0) {
			netdev_err(dev, "Initializing Threads FAILED\n");
			ret = -EIO;
			goto _fail_wilc_wlan_;
		}

		if (!wl->dev_irq_num &&
		    wl->hif_func->enable_interrupt &&
		    wl->hif_func->enable_interrupt(wl)) {
			netdev_err(dev, "couldn't initialize IRQ\n");
			ret = -EIO;
			goto _fail_irq_init_;
		}

		if (wilc_wlan_get_firmware(dev)) {
			netdev_err(dev, "Can't get firmware\n");
			ret = -EIO;
			goto _fail_irq_enable_;
		}

		ret = wilc_firmware_download(dev);
		if (ret < 0) {
			netdev_err(dev, "Failed to download firmware\n");
			ret = -EIO;
			goto _fail_irq_enable_;
		}

		ret = linux_wlan_start_firmware(dev);
		if (ret < 0) {
			netdev_err(dev, "Failed to start firmware\n");
			ret = -EIO;
			goto _fail_irq_enable_;
		}

		if (wilc_wlan_cfg_get(vif, 1, WID_FIRMWARE_VERSION, 1, 0)) {
			int size;
			char firmware_ver[50];

			size = wilc_wlan_cfg_get_val(WID_FIRMWARE_VERSION,
						     firmware_ver,
						     sizeof(firmware_ver));
			firmware_ver[size] = '\0';
			netdev_info(dev, "WILC Firmware Ver = %s\n", firmware_ver);
		}
		ret = linux_wlan_init_test_config(dev, vif);

		if (ret < 0) {
			netdev_err(dev, "Failed to configure firmware\n");
			ret = -EIO;
			goto _fail_fw_start_;
		}

		wl->initialized = true;
		return 0;

_fail_fw_start_:
		wilc_wlan_stop(wl);

_fail_irq_enable_:
		if (!wl->dev_irq_num &&
		    wl->hif_func->disable_interrupt)
			wl->hif_func->disable_interrupt(wl);
_fail_irq_init_:
		if (wl->dev_irq_num)
			deinit_irq(dev);

		wlan_deinitialize_threads(dev);
_fail_wilc_wlan_:
		wilc_wlan_cleanup(dev);
_fail_locks_:
		wlan_deinit_locks(dev);
		netdev_err(dev, "WLAN Iinitialization FAILED\n");
	} else {
		netdev_dbg(dev, "wilc already initialized\n");
	}
	return ret;
}

static int mac_init_fn(struct net_device *ndev)
{
	netif_start_queue(ndev);
	netif_stop_queue(ndev);

	return 0;
}

static int wilc_mac_open(struct net_device *ndev)
{
	struct wilc_vif *vif;

	unsigned char mac_add[ETH_ALEN] = {0};
	int ret = 0;
	struct wilc *wl;

	vif = netdev_priv(ndev);
	wl = vif->wilc;

	if (!wl || !wl->dev) {
		netdev_err(ndev, "device not ready\n");
		return -ENODEV;
	}

	netdev_dbg(ndev, "MAC OPEN[%p] %s\n",ndev, ndev->name);

	if(!recovery_on){
		ret = wilc_init_host_int(ndev);
		if (ret < 0) {
			netdev_err(ndev, "Failed to initialize host interface\n");
			return ret;
		}
	}

	ret = wilc_wlan_initialize(ndev, vif);
	if (ret < 0) {
		netdev_err(ndev, "Failed to initialize wilc\n");
		if(!recovery_on)
			wilc_deinit_host_int(ndev);
		return ret;
	}

	wait_for_recovery = 0;
	wilc_set_wfi_drv_handler(vif,
				 wilc_get_vif_idx(vif),
				 vif->iftype, vif->ifc_id);
	wilc_set_operation_mode(vif, vif->iftype);
	wilc_get_mac_address(vif, mac_add);
	netdev_dbg(ndev, "Mac address: %pM\n", mac_add);
	memcpy(vif->src_addr, mac_add, ETH_ALEN);

	memcpy(ndev->dev_addr, vif->src_addr, ETH_ALEN);

	if (!is_valid_ether_addr(ndev->dev_addr)) {
		netdev_err(ndev, "Wrong MAC address\n");
		wilc_deinit_host_int(ndev);
		wilc_wlan_deinitialize(ndev);
		return -EINVAL;
	}

	wilc_mgmt_frame_register(vif->ndev->ieee80211_ptr->wiphy,
				 vif->ndev->ieee80211_ptr,
				 vif->frame_reg[0].type,
				 vif->frame_reg[0].reg);
	wilc_mgmt_frame_register(vif->ndev->ieee80211_ptr->wiphy,
				 vif->ndev->ieee80211_ptr,
				 vif->frame_reg[1].type,
				 vif->frame_reg[1].reg);
#if defined(ANT_SWTCH_DUAL_GPIO_CTRL) || defined(ANT_SWTCH_SNGL_GPIO_CTRL)
	wilc_set_antenna(priv->hif_drv,DIVERSITY);
#endif
	netif_wake_queue(ndev);
	wl->open_ifcs++;
	vif->mac_opened = 1;
	return 0;
}

static struct net_device_stats *mac_stats(struct net_device *dev)
{
	struct wilc_vif *vif = netdev_priv(dev);

	return &vif->netstats;
}

static void wilc_set_multicast_list(struct net_device *dev)
{
	struct netdev_hw_addr *ha;
	struct wilc_vif *vif;
	int i = 0;

	vif = netdev_priv(dev);

	if (dev->flags & IFF_PROMISC)
		return;

	if ((dev->flags & IFF_ALLMULTI) ||
	    (dev->mc.count) > WILC_MULTICAST_TABLE_SIZE) {
		wilc_setup_multicast_filter(vif, false, 0);
		return;
	}

	if ((dev->mc.count) == 0) {
		wilc_setup_multicast_filter(vif, true, 0);
		return;
	}

	netdev_for_each_mc_addr(ha, dev) {
		memcpy(wilc_multicast_mac_addr_list[i], ha->addr, ETH_ALEN);
		netdev_dbg(dev, "Entry[%d]: %x:%x:%x:%x:%x:%x\n", i,
			   wilc_multicast_mac_addr_list[i][0],
			   wilc_multicast_mac_addr_list[i][1],
			   wilc_multicast_mac_addr_list[i][2],
			   wilc_multicast_mac_addr_list[i][3],
			   wilc_multicast_mac_addr_list[i][4],
			   wilc_multicast_mac_addr_list[i][5]);
		i++;
	}

	wilc_setup_multicast_filter(vif, true, (dev->mc.count));
}

static void linux_wlan_tx_complete(void *priv, int status)
{
	struct tx_complete_data *pv_data = priv;

	dev_kfree_skb(pv_data->skb);
	kfree(pv_data);
}

int wilc_mac_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct wilc_vif *vif;
	struct tx_complete_data *tx_data = NULL;
	int queue_count;
	char *udp_buf;
	struct iphdr *ih;
	struct ethhdr *eth_h;
	struct wilc *wilc;

	vif = netdev_priv(ndev);
	wilc = vif->wilc;

	if (skb->dev != ndev) {
		netdev_err(ndev, "Packet not destined to this device\n");
		return 0;
	}

	tx_data = kmalloc(sizeof(*tx_data), GFP_ATOMIC);
	if (!tx_data) {
		netdev_err(ndev, "Failed to allocate memory for tx_data structure\n");
		dev_kfree_skb(skb);
		netif_wake_queue(ndev);
		return 0;
	}

	tx_data->buff = skb->data;
	tx_data->size = skb->len;
	tx_data->skb  = skb;

	eth_h = (struct ethhdr *)(skb->data);
	if (eth_h->h_proto == (0x8e88))
		netdev_info(ndev," EAPOL transmitted\n");

	ih = (struct iphdr *)(skb->data + sizeof(struct ethhdr));

	udp_buf = (char *)ih + sizeof(struct iphdr);
	if ((udp_buf[1] == 68 && udp_buf[3] == 67) ||
	    (udp_buf[1] == 67 && udp_buf[3] == 68))
		netdev_dbg(ndev, "DHCP Message transmitted, type:%x %x %x\n",
			   udp_buf[248], udp_buf[249], udp_buf[250]);

	vif->netstats.tx_packets++;
	vif->netstats.tx_bytes += tx_data->size;

	tx_data->bssid = wilc->vif[vif->idx]->bssid;
	queue_count = wilc_wlan_txq_add_net_pkt(ndev, (void *)tx_data,
						tx_data->buff, tx_data->size,
						linux_wlan_tx_complete);

	if (queue_count > FLOW_CONTROL_UPPER_THRESHOLD) {
		netif_stop_queue(wilc->vif[0]->ndev);
		netif_stop_queue(wilc->vif[1]->ndev);
	}

	return 0;
}

static int wilc_mac_close(struct net_device *ndev)
{
	struct wilc_priv *priv;
	struct wilc_vif *vif;
	struct host_if_drv *hif_drv;
	struct wilc *wl;

	vif = netdev_priv(ndev);

	if (!vif || !vif->ndev || !vif->ndev->ieee80211_ptr ||
	    !vif->ndev->ieee80211_ptr->wiphy) {
		netdev_err(ndev, "vif = NULL\n");
		return 0;
	}

	priv = wiphy_priv(vif->ndev->ieee80211_ptr->wiphy);
	wl = vif->wilc;

	if (!priv) {
		netdev_err(ndev, "priv = NULL\n");
		return 0;
	}

	hif_drv = (struct host_if_drv *)priv->hif_drv;

	netdev_dbg(ndev, "Mac close\n");

	if (!wl) {
		netdev_err(ndev, "wilc = NULL\n");
		return 0;
	}

	if (!hif_drv)	{
		netdev_err(ndev, "pstrWFIDrv = NULL\n");
		return 0;
	}

	if ((wl->open_ifcs) > 0) {
		wl->open_ifcs--;
	} else {
		netdev_err(ndev, "ERROR: MAC close called while number of opened interfaces is zero\n");
		return 0;
	}

	if (vif->ndev) {
		netif_stop_queue(vif->ndev);

	if(!recovery_on)
		wilc_deinit_host_int(vif->ndev);
	}

	if (wl->open_ifcs == 0) {
		netdev_dbg(ndev, "Deinitializing wilc\n");
		wl->close = 1;
		wilc_wlan_deinitialize(ndev);
		WILC_WFI_deinit_mon_interface();
	}

	vif->mac_opened = 0;

	return 0;
}

static int mac_ioctl(struct net_device *ndev, struct ifreq *req, int cmd)
{
	u8 *buff = NULL;
	s8 rssi;
	u32 size = 0;
	struct wilc_vif *vif;
	s32 ret = 0;
	struct wilc *wilc;

	vif = netdev_priv(ndev);
	wilc = vif->wilc;

	if (!wilc->initialized)
		return 0;

	switch (cmd) {
	case SIOCSIWPRIV:
	{
		struct iwreq *wrq = (struct iwreq *)req;

		size = wrq->u.data.length;

		if (size && wrq->u.data.pointer) {
			buff = memdup_user(wrq->u.data.pointer,
					   wrq->u.data.length);
			if (IS_ERR(buff))
				return PTR_ERR(buff);

			if (strncasecmp(buff, "RSSI", size) == 0) {
				ret = wilc_get_rssi(vif, &rssi);
				if (ret)
					netdev_err(ndev, "Failed to send get rssi param's message queue ");

				netdev_info(ndev, "RSSI :%d\n", rssi);

				rssi += 5;

				snprintf(buff, size, "rssi %d", rssi);

				if (copy_to_user(wrq->u.data.pointer, buff, size)) {
					netdev_err(ndev, "failed to copy\n");
					ret = -EFAULT;
					goto done;
				}
			}
		}
	}
	break;

	default:
	{
		netdev_info(ndev, "Command - %d - has been received\n", cmd);
		ret = -EOPNOTSUPP;
		goto done;
	}
	}

done:

	kfree(buff);

	return ret;
}

void frmw_to_linux(struct wilc *wilc, u8 *buff, u32 size, u32 pkt_offset, u8
		   status)
{
	unsigned int frame_len = 0;
	int stats;
	unsigned char *buff_to_send = NULL;
	struct sk_buff *skb;
	struct net_device *wilc_netdev;
	struct wilc_vif *vif;
	struct wilc_priv *priv;
	u8 null_bssid[ETH_ALEN] = {0};

	if (!wilc)
		return;

	wilc_netdev = get_if_handler(wilc, buff);
	if (!wilc_netdev){
		return;
	}
	buff += pkt_offset;
	vif = netdev_priv(wilc_netdev);
	priv = wiphy_priv(vif->ndev->ieee80211_ptr->wiphy);

	if (size > 0) {
		frame_len = size;
		buff_to_send = buff;

		if((status == PKT_STATUS_NEW)
			&&((buff_to_send[12] == 0x88 && buff_to_send[13] == 0x8e))
			&&(vif->iftype == STATION_MODE || vif->iftype == CLIENT_MODE)
			&&(!(memcmp(priv->au8AssociatedBss, null_bssid, ETH_ALEN)))) {

			if(!priv->buffered_eap) {
				priv->buffered_eap = kmalloc(sizeof(struct
								    wilc_buffered_eap),
							     GFP_ATOMIC);
				if(priv->buffered_eap) {
					priv->buffered_eap->buff = NULL;
					priv->buffered_eap->size = 0;
					priv->buffered_eap->pkt_offset = 0;
				} else {
					netdev_err(wilc_netdev, "failed to alloc priv->buffered_eap\n");
					return;
				}
			} else {
				kfree(priv->buffered_eap->buff );
			}
			priv->buffered_eap->buff = kmalloc(size + pkt_offset,
							   GFP_ATOMIC);
			priv->buffered_eap->size = size;
			priv->buffered_eap->pkt_offset = pkt_offset;
			memcpy(priv->buffered_eap->buff, buff -
			       pkt_offset, size + pkt_offset);
                       eap_buff_timer.data = (unsigned long) priv;
                       mod_timer(&eap_buff_timer,(jiffies +
                                                  msecs_to_jiffies(10))) ;
			return;
		}
		skb = dev_alloc_skb(frame_len);
		if (!skb){
       	 	netdev_err(wilc_netdev, "Low memory - packet droped\n");
			return;
		}

		if (wilc == NULL || wilc_netdev == NULL)
			netdev_err(wilc_netdev, "wilc_netdev in wilc is NULL");
		skb->dev = wilc_netdev;
		if (skb->dev == NULL)
			netdev_err(wilc_netdev, "skb->dev is NULL\n");
		memcpy(skb_put(skb, frame_len), buff_to_send, frame_len);

		skb->protocol = eth_type_trans(skb, wilc_netdev);
		vif->netstats.rx_packets++;
		vif->netstats.rx_bytes += frame_len;
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		stats = netif_rx(skb);
		netdev_dbg(wilc_netdev, "netif_rx ret value is: %d\n", stats);
	} else {
		netdev_err(wilc_netdev, "Discard sending packet with len = %d\n", size);
	}
}

void WILC_WFI_mgmt_rx(struct wilc *wilc, u8 *buff, u32 size)
{
	int i = 0;
	struct wilc_vif *vif;

	for (i = 0; i <= wilc->vif_num; i++) {
		vif = netdev_priv(wilc->vif[i]->ndev);
		if (vif->monitor_flag) {
			WILC_WFI_monitor_rx(buff, size);
			return;
		}
	}

	vif = netdev_priv(wilc->vif[1]->ndev);
	if ((buff[0] == vif->frame_reg[0].type && vif->frame_reg[0].reg) ||
	    (buff[0] == vif->frame_reg[1].type && vif->frame_reg[1].reg))
		WILC_WFI_p2p_rx(wilc->vif[1]->ndev, buff, size);
}

void wilc_netdev_cleanup(struct wilc *wilc)
{
	int i;

	if (wilc && wilc->firmware) {
		release_firmware(wilc->firmware);
		wilc->firmware = NULL;
	}

	if (wilc && (wilc->vif[0]->ndev || wilc->vif[1]->ndev)) {
		for (i = 0; i < NUM_CONCURRENT_IFC; i++)
			if (wilc->vif[i]->ndev)
				if (wilc->vif[i]->mac_opened)
					wilc_mac_close(wilc->vif[i]->ndev);

		for (i = 0; i < NUM_CONCURRENT_IFC; i++) {
			if(wilc->vif[i] && wilc->vif[i]->ndev) {
				unregister_netdev(wilc->vif[i]->ndev);
				wilc_free_wiphy(wilc->vif[i]->ndev);
			}
			free_netdev(wilc->vif[i]->ndev);
		}
	}

	#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		unregister_inetaddr_notifier(&g_dev_notifier);
	#endif

	kfree(wilc);
	p2p_sysfs_exit();
}
EXPORT_SYMBOL_GPL(wilc_netdev_cleanup);

int wilc_netdev_init(struct wilc **wilc, struct device *dev, int io_type,
		     int gpio, const struct wilc_hif_func *ops)
{
	int i, ret;
	struct wilc_vif *vif;
	struct net_device *ndev;
	struct wilc *wl;
	struct wireless_dev *wdev;

	wl = kzalloc(sizeof(*wl), GFP_KERNEL);
	if (!wl)
		return -ENOMEM;

	*wilc = wl;

	wl->io_type = io_type;
	wl->gpio = gpio;
	wl->hif_func = ops;

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	register_inetaddr_notifier(&g_dev_notifier);
#endif

	for (i = 0; i < NUM_CONCURRENT_IFC; i++) {
		ndev = alloc_etherdev(sizeof(struct wilc_vif));
		if (!ndev)
			return -ENOMEM;

		vif = netdev_priv(ndev);
		memset(vif, 0, sizeof(struct wilc_vif));

		if (i == 0)
			strcpy(ndev->name, "wlan%d");
		else
			strcpy(ndev->name, "p2p%d");

		wl->vif_num = i;
		vif->idx = wl->vif_num;
		vif->wilc = *wilc;
		vif->ndev = ndev;
		wl->vif[i] = vif;


		ndev->netdev_ops = &wilc_netdev_ops;

		wdev = wilc_create_wiphy(ndev, dev);

		if (dev)
			SET_NETDEV_DEV(ndev, dev);

		if (!wdev) {
			netdev_err(ndev, "Can't register WILC Wiphy\n");
			return -1;
		}

		vif->ndev->ieee80211_ptr = wdev;
		vif->ndev->ml_priv = vif;
		wdev->netdev = vif->ndev;
		vif->netstats.rx_packets = 0;
		vif->netstats.tx_packets = 0;
		vif->netstats.rx_bytes = 0;
		vif->netstats.tx_bytes = 0;

		ret = register_netdev(ndev);
		if (ret) {
			netdev_err(ndev, "Device couldn't be registered - %s\n",
			       ndev->name);
			return ret;
		}

		vif->iftype = STATION_MODE;
		vif->mac_opened = 0;
	}
	p2p_sysfs_init(vif);

	return 0;
}
EXPORT_SYMBOL_GPL(wilc_netdev_init);

static void wilc_wlan_power(int power)
{
	if (gpio_request(GPIO_NUM_CHIP_EN, "CHIP_EN") == 0 &&
	    gpio_request(GPIO_NUM_RESET, "RESET") == 0) {
		gpio_direction_output(GPIO_NUM_CHIP_EN, 0);
		gpio_direction_output(GPIO_NUM_RESET, 0);
		if (power) {
			gpio_set_value(GPIO_NUM_CHIP_EN, 1);
			mdelay(5);
			gpio_set_value(GPIO_NUM_RESET, 1);
		} else {
			gpio_set_value(GPIO_NUM_RESET, 0);
			gpio_set_value(GPIO_NUM_CHIP_EN, 0);
		}
		gpio_free(GPIO_NUM_CHIP_EN);
		gpio_free(GPIO_NUM_RESET);
	}
}

void wilc_wlan_power_on_sequence(void)
{
	wilc_wlan_power(0);
	wilc_wlan_power(1);
}
EXPORT_SYMBOL_GPL(wilc_wlan_power_on_sequence);

void wilc_wlan_power_off_sequence(void)
{
	wilc_wlan_power(0);
}
EXPORT_SYMBOL_GPL(wilc_wlan_power_off_sequence);

MODULE_LICENSE("GPL");
