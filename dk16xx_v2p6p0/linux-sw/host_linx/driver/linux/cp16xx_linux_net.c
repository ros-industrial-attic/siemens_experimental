#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>

#include <linux/in.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/tcp.h>

#include <linux/in6.h>
#include <asm/atomic.h>
#include <asm/checksum.h>

#include "../../layer2lib/src/l2eth_base.c"
#include "../../layer2lib/src/l2eth_buf_mgmt.c"
#include "../../layer2lib/src/l2eth_user.c"

static int timeout = 5;
module_param(timeout, int, S_IRUGO);
MODULE_PARM_DESC(timeout, "watchdog timeout for network device");

/**
 *  Private part of the device
 **/
struct cp16xx_net_card_data {
    struct net_device_stats stats;

    L2ETH_UINT32 handle;          /* handle number of current session with cp16xx */
    L2ETH_UINT32 mclist_size;     /* the maximal size of multicast list */
    L2ETH_MODE mode;
    L2ETH_UINT8 infobuffer[114];  /* buffer for l2eth_get_information */
                                  /* take the max length of multicast list, 19 * 6 */
    int sbuffer;                  /* keep track of the number of available send buffer */
    L2_KERNEL_SPINLOCK clock;              /* for mutual exclusive access to this structure, nonsleep */
    L2ETH_MUTEX cmutex;           /* for mutual exclusive access to this structure */
    L2ETH_SEMAPHORE online;       /* for mode complete callback of online */
    L2ETH_SEMAPHORE offline;      /* for mode complete callback of offline */
    L2ETH_UINT32 error;           /* for error code of mode complete callback */

    L2_KERNEL_SPINLOCK rlock;     /* lock for allocate and free packets */
    L2_KERNEL_SPINLOCK slock;     /* lock for return packets */

    struct net_device *ndev;
};

/* static array of all net_device pointers */
static struct cp16xx_net_card_data *cp16xx_net_cards[MAX_CP16XX_DEVICES];
/* spinlock protects cp16xx_net_cards array */
static DPR_SPINLOCK cp16xx_net_cards_lock;

static void cp16xx_net_init_global_data(void)
{
    memset(cp16xx_net_cards, 0, sizeof(cp16xx_net_cards));
    DPR_SPINLOCK_INIT(cp16xx_net_cards_lock);
}

static void cp16xx_net_uninit_global_data(void)
{
    DPR_SPINLOCK_UNINIT(cp16xx_net_cards_lock);
}

struct cp16xx_net_card_data *cp16xx_net_card_ref_from_handle(L2ETH_UINT32 handle)
{
    int i;
    DPR_SPINLOCK_FLAGS flags;

    /* queue private data in global static array of this */
    DPR_SPINLOCK_LOCK(cp16xx_net_cards_lock, flags);
    for(i = 0; i < MAX_CP16XX_DEVICES; i++) {
        if(cp16xx_net_cards[i] && cp16xx_net_cards[i]->handle == handle) {
            break;
        }
    }
    DPR_SPINLOCK_UNLOCK(cp16xx_net_cards_lock, flags);

    if(i >= MAX_CP16XX_DEVICES) {
        return NULL;
    }

    return cp16xx_net_cards[i];
}

static struct cp16xx_net_card_data *cp16xx_net_card_ref_add(struct cp16xx_net_card_data *priv)
{
    int i;
    DPR_SPINLOCK_FLAGS flags;

    /* queue private data in global static array of this */
    DPR_SPINLOCK_LOCK(cp16xx_net_cards_lock, flags);
    for(i = 0; i < MAX_CP16XX_DEVICES; i++) {
        if(!cp16xx_net_cards[i]) {
            cp16xx_net_cards[i] = priv;
            break;
        }
    }
    DPR_SPINLOCK_UNLOCK(cp16xx_net_cards_lock, flags);

    if(i >= MAX_CP16XX_DEVICES) {
        DPRLIBERRMSG("too much CPs\n");
        return NULL;
    }

    return cp16xx_net_cards[i];
}

static void cp16xx_net_card_ref_del(struct cp16xx_net_card_data *priv)
{
    int i;
    DPR_SPINLOCK_FLAGS flags;

    DPR_SPINLOCK_LOCK(cp16xx_net_cards_lock, flags);
    for(i = 0; i < MAX_CP16XX_DEVICES; i++) {
        if(cp16xx_net_cards[i] == priv) {
            cp16xx_net_cards[i] = NULL;
            break;
        }
    }
    DPR_SPINLOCK_UNLOCK(cp16xx_net_cards_lock, flags);
}

L2_KERNEL_SPINLOCK *l2_read_spinlock(L2ETH_UINT32 handle)
{
    struct cp16xx_net_card_data *priv;

    priv = cp16xx_net_card_ref_from_handle(handle);
    if(priv)
        return &priv->rlock;

    return NULL;
}

L2_KERNEL_SPINLOCK *l2_write_spinlock(L2ETH_UINT32 handle)
{
    struct cp16xx_net_card_data *priv;

    priv = cp16xx_net_card_ref_from_handle(handle);
    if(priv)
        return &priv->slock;

    return NULL;
}

/**
 *  Helper inline function to get information via l2
 **/
inline L2ETH_UINT32 cp_get_information(L2ETH_OID oid, u8 *infobuffer,
    L2ETH_QUERY *query, L2ETH_UINT32 handle)
{
  query->Oid = oid;
  query->pBuffer = infobuffer;
  query->BufferLength = getMaxBufferLength(oid);
  memset(query->pBuffer, 0, query->BufferLength);

  return l2eth_get_information(handle, query);
}

/**
 *  cp_open - Interface Activation Routine
 *
 *  cp_open is called when the interface is activated
 *  by means of "ifconfig" or anything else
 **/
int cp_open(struct net_device *dev)
{
    struct cp16xx_net_card_data *priv;
    L2ETH_UINT32 result;
    L2ETH_QUERY query;
    unsigned long carrier;
    int ret = 0;

    priv = netdev_priv(dev);
    if(L2ETH_MUTEX_LOCK(priv->cmutex))
        return -ERESTARTSYS;

    if(priv->mode == L2ETH_ONLINE)
        goto carrier;

    /* bring the card online */
    DPRLIBLOGMSG("going to bring the card online\n");
    result = l2eth_set_mode(priv->handle, L2ETH_ONLINE); /* bring the interface online */

    switch(result) {
    case L2ETH_OK:
        DPRLIBLOGMSG("succeeded calling to set the interface online\n");
        /* allow users to interrupt if wait too long for the call back function */
        if(L2ETH_SEM_WAIT_INTERRUPTIBLE(priv->online)) {
            ret = -ERESTARTSYS;
            goto end;
        } else {
        /* no error, everything is fine */
        if(priv->error == L2ETH_OK) {
            priv->mode = L2ETH_ONLINE;
            netif_start_queue(dev);
            ret = 0;
            break;
        } else {
            ret = priv->error;
            goto end;
        }
        }
    default:
        DPRLIBERRMSG("failed with bringing the interface online with error 0x%8x\n", result);
        ret = -EIO;
        break;
    }

    /* check the media conntion status */
carrier:
    result = cp_get_information(L2ETH_OID_MEDIA_CONNECT_STATUS, priv->infobuffer,
        &query, priv->handle);
    if(result != L2ETH_OK)
        ret = -EIO;
    else {
        memcpy(&carrier, priv->infobuffer, getMaxBufferLength(L2ETH_OID_MEDIA_CONNECT_STATUS));
        DPRLIBLOGMSG("the media connection status is now %lu\n", carrier);

        /* since the interpretation of the  value of priv->connected is
        not yet clear, the following part is commented out */
        /*if(carrier)
        netif_carrier_on(dev);
        else
        netif_carrier_off(dev);
        */
        netif_carrier_on(dev);  /* delete this line later when uncomment code above */
    }

end:
    L2ETH_MUTEX_UNLOCK(priv->cmutex);
    return ret;
}

/**
 *  cp_release - Interface Deactivation Routine
 *
 *  cp_release is called when the interface is deactivated
 *  by means of "ifconfig" or anything else
 **/
int cp_release(struct net_device *dev)
{
    struct cp16xx_net_card_data *priv;
    L2ETH_UINT32 result;
    int ret = 0;

    priv = netdev_priv(dev);
    if(L2ETH_MUTEX_LOCK(priv->cmutex))
        return -ERESTARTSYS;

    /* if the interface is already offline, return successfully */
    if(priv->mode == L2ETH_OFFLINE) {
        L2ETH_MUTEX_UNLOCK(priv->cmutex);
        return 0;
    }

    result = l2eth_set_mode(priv->handle, L2ETH_OFFLINE); /* bring the interface offline */
    switch(result) {
    case L2ETH_OK:
        DPRLIBLOGMSG("succeeded calling to set the interface offline\n");
        if(L2ETH_SEM_WAIT_INTERRUPTIBLE(priv->offline)) {
            ret = -ERESTARTSYS;
            break;
        } else {
            if(priv->error == L2ETH_OK) {
                priv->mode = L2ETH_OFFLINE;
                netif_stop_queue(dev);
                ret = 0;
                break;
            } else {
                ret = priv->error;
                break;
            }
        }
    default:
        DPRLIBERRMSG("failed with bringing the interface offline with error 0x%8x\n", result);
        ret = -EIO;
        break;
    }

    L2ETH_MUTEX_UNLOCK(priv->cmutex);
    return ret;
}

/**
 *  cp_config - Change card configurations(past on by ifconfig)
 *
 *  cp_config doesn't do a lot becuase only few things could be configured
 *  via l2 interface
 **/
int cp_config(struct net_device *dev, struct ifmap *map)
{
    DPRLIBLOGMSG("cp_config called \n");
    if(dev->flags & IFF_UP)
        return -EBUSY;
    return 0;
}

/**
 *  cp_change_mtu - Change the size of max transfer unix
 **/
int cp_change_mtu(struct net_device *dev, int new_mtu)
{
    struct cp16xx_net_card_data *priv;
    u8 mtu[4] = {0, 0, 0, 0};
    L2ETH_UINT32 result;
    int ret;

    L2ETH_QUERY query = {
        .Oid = L2ETH_OID_MAXIMUM_FRAME_SIZE,
        .pBuffer = mtu,
        .BufferLength = 4,
    };

    priv = netdev_priv(dev);

    if(new_mtu < ETH_ZLEN || new_mtu > ETH_DATA_LEN)
        return -EINVAL;
    memcpy((mtu + (4 - sizeof(int))), &new_mtu, sizeof(int));

    DPRLIBLOGMSG("going to change mtu to %d\n", *((L2ETH_UINT32 *)mtu));
    if(L2ETH_MUTEX_LOCK(priv->cmutex))
        return -ERESTARTSYS;
    result = l2eth_set_information(priv->handle, &query);

    if(result != L2ETH_OK) {
        DPRLIBERRMSG("failed to set mtu on the card with error 0x%8x\n", result);
        ret = -EIO;
    } else {
        dev->mtu = new_mtu;
        ret = 0;
    }

    L2ETH_MUTEX_UNLOCK(priv->cmutex);
    return ret;
}

/**
 *  cp_set_multicast_list - set multicast list for the card
 **/
void cp_set_multicast_list(struct net_device *dev)
{
}

/**
 *  cp_l2_tx_callback - Callback function for L2 interface
 *
 *  the allocated l2 packet will be set free here and
 *  statistic will be updated
 **/
void cp_l2_tx_callback(L2ETH_UINT32 handle, L2ETH_PACKET *packet, L2ETH_UINT32 error)
{
    struct cp16xx_net_card_data *priv;
    struct net_device *dev;

    priv = cp16xx_net_card_ref_from_handle(handle);
    dev = priv->ndev;

    DPRLIBLOGMSG("Packet send completed, in callback\n");
    if(l2eth_free_packet(priv->handle, packet) != L2ETH_OK)
        DPRLIBERRMSG("Unable to free L2ETH_PACKET\n");
    else {
        DPRLIBLOGMSG("Succeeded to free L2ETH_PACKET\n");
        L2_KERNEL_SPINLOCK_LOCK_IRQ_OFF(priv->clock);
        priv->sbuffer++;
        if(netif_queue_stopped(dev)) {
            if(priv->sbuffer >= (MAX_ETHERNET_SEND_FRAME_COUNT / 4)) {
                netif_wake_queue(dev);
                DPRLIBLOGMSG("Have more buffer now, wake the queue\n");
            }
        }
        L2_KERNEL_SPINLOCK_UNLOCK_IRQ_ON(priv->clock);
    }

    if(error != L2ETH_OK)
        priv->stats.tx_errors++;
    else
        priv->stats.tx_packets++;

    if(!netif_carrier_ok(dev))
        priv->stats.tx_carrier_errors += 1;

    priv->stats.tx_bytes += packet->DataLength;
}

/**
 *  io control
 **/
int cp_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    DPRLIBLOGMSG("ioctl called\n");
    return 0;
}

/**
 *  cp__tx -  Hand over a packet to L2 interface.
 *
 *  cp_tx would be called by the network subsystem if a packet comes through
 *  the socket which is bound to this device
 **/
int cp_tx(struct sk_buff *skb, struct net_device *dev)
{
    struct cp16xx_net_card_data *priv;
    int len;
    L2ETH_PACKET *packet;
    L2ETH_UINT32 result;

    priv = netdev_priv(dev);

    if(priv->mode == L2ETH_OFFLINE) {
        DPRLIBLOGMSG("cp_tx called while the interface is offline\n");
        return -1;
    }

    dev->trans_start = jiffies;

    /**
    *  Request an L2ETH_PACKET from l2 interface
    **/
    DPRLIBLOGMSG("going to allocate a packet for sending\n");
    result = l2eth_allocate_packet(priv->handle, &packet);

    if(result != L2ETH_OK) {
        DPRLIBERRMSG("L2ETH_PACKET allocation for send failed with 0x%8x\n", result);
        dev_kfree_skb(skb);
        priv->stats.tx_dropped += 1;
        return -EIO;
    }

    /* the last piece has been allocated, tell the kernel to stop sending */
    L2_KERNEL_SPINLOCK_LOCK_IRQ_OFF(priv->clock);
    if((--(priv->sbuffer)) == 0) {
        netif_stop_queue(dev);
        DPRLIBLOGMSG("No more buffer, stop the sending queue\n");
    }
    L2_KERNEL_SPINLOCK_UNLOCK_IRQ_ON(priv->clock);

    /**
    *  The data field of a skb must reach the minimal length.
    *  In case it doesn't, extra paddings must be appended
    *
    **/
    if(skb->len < ETH_ZLEN)
        len = ETH_ZLEN;
    else
        len = skb->len;

    packet->DataLength = len;
    memcpy(packet->pBuffer, skb->data, skb->len );

    result = l2eth_send(priv->handle, packet);
    dev_kfree_skb(skb);

    if(result != L2ETH_OK) {
        DPRLIBERRMSG("l2eth_send ended with 0x%8x\n", result);
        priv->stats.tx_errors += 1;
        return -EIO;
    }

    return 0;
}

/**
 *  cp_rx - Receive a packet
 *
 *  cp_rx retrieves, encapsulates and passes a packet to upper levels
 **/
void cp_rx(struct net_device *dev, L2ETH_PACKET *packet)
{
    struct sk_buff *skb;
    struct cp16xx_net_card_data *priv;

    DPRLIBLOGMSG("cp_rx called\n");

    priv = netdev_priv(dev);
    /**
    *  Allocate a new socket buffer, if the allocation fails
    *  increase the number of dropped packats in the stats
    *  structure by 1.
    **/
    DPRLIBLOGMSG("address of the packet buffer is 0x%lx\n", (unsigned long)packet->pBuffer);
    DPRLIBLOGMSG("length of the packet is %u\n", packet->DataLength);
    skb = dev_alloc_skb(packet->DataLength + 2);
    if (!skb) {
        if (printk_ratelimit())
        printk(KERN_DEBUG "cp rx: low on mem - packet dropped\n");
        priv->stats.rx_dropped++;
        goto out;
    }
    skb_reserve(skb, 2); /* align on 16 bytes boundary */
    memcpy(skb_put(skb, packet->DataLength), packet->pBuffer, packet->DataLength);

    /**
    *  Write metadata, and pass it to the upper level
    **/
    skb->dev = dev;
    skb->protocol = eth_type_trans(skb, dev);
    netif_rx(skb);

    /**
    *  Update statistics for the device
    **/
    priv->stats.rx_packets++;
    priv->stats.rx_bytes += packet->DataLength;

out:
    l2eth_return_packet(priv->handle, packet);
    return;
}

/**
 *  cp_l2_rx_callback - Callback function for L2 interface
 *
 *  cp_l2_rx_callback maps handle to net_device and passes the L2 packet
 *  to cp_rx
 **/
int cp_l2_rx_callback(L2ETH_UINT32 handle, L2ETH_PACKET *packet)
{
    struct cp16xx_net_card_data *priv;
    struct net_device *dev;

    DPRLIBLOGMSG("Receive callbackup function  called with handle 0x%x\n", handle);

    priv = cp16xx_net_card_ref_from_handle(handle);
    dev = priv->ndev;

    cp_rx(dev, packet);

    /* as a driver, all packets are for me */
    return 1;
}

/**
 *  cp_tx_timeout - Deal with a transmit timeout
 **/
void cp_tx_timeout(struct net_device *dev)
{
    struct cp16xx_net_card_data *priv;

    priv = netdev_priv(dev);
    priv->stats.tx_errors++;
    return;
}

/**
 *  cp_stats - Return statistics to the caller
 **/
struct net_device_stats *cp_stats(struct net_device *dev)
{
    struct cp16xx_net_card_data *priv;

    priv = netdev_priv(dev);
    return &priv->stats;
}

/**
 *  cp_l2_mode_complete_callback - Callback function for L2 interface
 **/
void cp_l2_mode_complete_callback(L2ETH_UINT32 handle, L2ETH_MODE mode, L2ETH_UINT32 error)
{
    struct cp16xx_net_card_data *priv;

    priv = cp16xx_net_card_ref_from_handle(handle);

    DPRLIBLOGMSG("cp_l2_mode_complete_callback called, the current mode is %d\n", mode);
    priv->error = error;
    if (mode == L2ETH_ONLINE) {
        L2ETH_SEM_POST(priv->online);
        return;
    } else {
        L2ETH_SEM_POST(priv->offline);
        return;
    }
}

/**
 * cp_l2_status_callback - Callback function for L2 interface
 *
 * only the media connection status is taken into account
 **/
void cp_l2_status_callback(L2ETH_UINT32 handle, L2ETH_OID oid)
{
    struct cp16xx_net_card_data *priv;
    struct net_device *dev;

    DPRLIBLOGMSG("Receive cp_l2_status_callback function  called with handle 0x%x\n", handle);

    priv = cp16xx_net_card_ref_from_handle(handle);
    dev = priv->ndev;

    if(oid == L2ETH_OID_MEDIA_CONNECT_STATUS) {
        DPRLIBLOGMSG("cp_l2_status_callback called, oid %d changed\n", oid);
        if(!netif_carrier_ok(dev)) {
            DPRLIBLOGMSG("the carrier is now on\n");
            netif_carrier_on(dev);
        } else {
            DPRLIBLOGMSG("the carrier is now off\n");
            netif_carrier_off(dev);
        }
    }
    return;
}

/**
 *  cp16xx_net_card_setup - The setup function
 *
 *  cp16xx_net_card_setup is invoked by register_netdev()
 **/
void cp16xx_net_card_setup(struct net_device *dev)
{
    struct cp16xx_net_card_data *priv;

    DPRLIBLOGMSG("cp16xx_net_card_setup called to do some initialization work\n");

    /**
    *  Assign fields in dev, using ether_setup() and
    *  some hand assignments
    **/
    ether_setup(dev);

    dev->open               = cp_open;
    dev->stop               = cp_release;
    dev->set_multicast_list = cp_set_multicast_list;
    dev->change_mtu         = cp_change_mtu;
    dev->set_config         = cp_config;
    dev->hard_start_xmit    = cp_tx;
    dev->do_ioctl           = cp_ioctl;
    dev->get_stats          = cp_stats;
    dev->tx_timeout         = cp_tx_timeout;
    dev->watchdog_timeo     = timeout;

    priv = netdev_priv(dev);

    memset(priv, 0, sizeof(struct cp16xx_net_card_data));
    priv->sbuffer = MAX_ETHERNET_SEND_FRAME_COUNT;
    DPR_SPINLOCK_INIT(priv->clock);
    L2ETH_MUTEX_CREATE_UNLOCKED(priv->cmutex);
    L2ETH_SEM_CREATE(priv->online);
    L2ETH_SEM_CREATE(priv->offline);
    priv->mode = L2ETH_OFFLINE;
    L2_KERNEL_SPINLOCK_INIT(priv->rlock);
    L2_KERNEL_SPINLOCK_INIT(priv->slock);
    priv->ndev = dev;

    netif_carrier_off(dev);        /* consider the carrier is off initially */
}

/**
 *  cp16xx_net_card_uninit - Driver Module  Exit Cleanup Function
 *
 **/
static void cp16xx_net_card_uninit(struct cp16xx_card_data *card)
{
    struct net_device *dev;
    struct cp16xx_net_card_data *priv;

    dev = (struct net_device *)card->os_info.net_device;

    if(dev) {
        priv = netdev_priv(dev);

        cp16xx_net_card_ref_del(priv);

        /**
        *  Unregister the network with the kernel
        **/
        unregister_netdev(dev);

        /**
        *  Unregister the driver with l2 interface
        *  if it's registered
        **/
        l2eth_close(priv->handle);

        free_netdev(dev);

        card->os_info.net_device = NULL;
    }
    return;
}

/**
 *  cp16xx_net_card_init - Driver instance registration function
 *
 **/
int cp16xx_net_card_init(struct cp16xx_card_data *card)
{
    int ret;
    struct net_device *dev;
    struct cp16xx_net_card_data *priv;
    L2ETH_UINT32 result, handle;
    L2ETH_QUERY query;

    /* Allocate the device and set it's name */
    dev = alloc_netdev(sizeof(struct cp16xx_net_card_data), "cp16xx_net", cp16xx_net_card_setup);
    if (dev == NULL) {
        ret = -ENOMEM;
        goto out;
    }

    memset(dev->dev_addr, 0, getMaxBufferLength(L2ETH_OID_PERMANENT_ADDRESS));

    /* Init the l2 part of the device */
    priv = netdev_priv(dev);

    /* open the l2 interface */
    result = l2eth_open((card->cp_index + 1), cp_l2_rx_callback, cp_l2_tx_callback, cp_l2_status_callback,
        cp_l2_mode_complete_callback, &handle);

    if (result == L2ETH_OK) {
        priv->handle = handle;
    } else {
        DPRLIBERRMSG("failed to open the l2 interface with error 0x%x\n", result);
        ret = -EIO;
        goto out;
    }

    /* to get the mac address of the card */
    result = cp_get_information(L2ETH_OID_PERMANENT_ADDRESS, priv->infobuffer,
        &query, handle);
    if(result != L2ETH_OK) {
        DPRLIBERRMSG("failed to readout the MAC address from card with error 0x%x\n", result);
        ret = -EIO;
        goto out;
    }
    memcpy(dev->dev_addr, priv->infobuffer, getMaxBufferLength(L2ETH_OID_PERMANENT_ADDRESS));

    /* to get the maximum size of the multicast list */
    result = cp_get_information(L2ETH_OID_MAXIMUM_LIST_SIZE, priv->infobuffer,
        &query, handle);
    if(result != L2ETH_OK) {
        DPRLIBERRMSG("failed to read out the maximum size of the multicast list from card\
        with error 0x%x\n", result);
        ret = -EIO;
        goto out;
    }
    memcpy(&(priv->mclist_size), query.pBuffer, getMaxBufferLength(L2ETH_OID_MAXIMUM_LIST_SIZE));
    DPRLIBLOGMSG("maximum size of the multicast list is %d\n", priv->mclist_size);

    /* Everything is ready, register the device */
    if (!(ret = register_netdev(dev))) {
        DPRLIBLOGMSG("Successfully registered device\n");

        card->os_info.net_device = dev;
        cp16xx_net_card_ref_add(priv);
    }

out:
    if (ret) {
        DPRLIBERRMSG("driver initialization failed with error %d\n", ret);
        if(dev) {
            priv = netdev_priv(dev);
            if(priv->handle)
                l2eth_close(priv->handle);
            free_netdev(dev);
        }
    }
    return ret;
}
