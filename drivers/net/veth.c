/*
 *  drivers/net/veth.c
 *
 *  Copyright (C) 2007 OpenVZ http://openvz.org, SWsoft Inc
 *
 * Author: Pavel Emelianov <xemul@openvz.org>
 * Ethtool interface from: Eric W. Biederman <ebiederm@xmission.com>
 *
 */

#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/u64_stats_sync.h>

#include <net/rtnetlink.h>
#include <net/dst.h>
#include <net/xfrm.h>
#include <linux/veth.h>
#include <linux/module.h>
#include <linux/tpacket4.h>

#define DRV_NAME	"veth"
#define DRV_VERSION	"1.0"

struct pcpu_vstats {
	u64			packets;
	u64			bytes;
	struct u64_stats_sync	syncp;
};

struct veth_priv {
	struct net_device __rcu	*peer;
	atomic64_t		dropped;
	unsigned		requested_headroom;
	struct tp4_queue        *tp4q_rx;
	struct tp4_queue        *tp4q_tx;
	struct napi_struct      *napi;
	struct tpacket4_desc    *descs_rx;
	struct tpacket4_desc    *descs_tx;
	bool                    tp4_zerocopy;
};

/*
 * ethtool interface
 */

static struct {
	const char string[ETH_GSTRING_LEN];
} ethtool_stats_keys[] = {
	{ "peer_ifindex" },
};

static int veth_get_link_ksettings(struct net_device *dev,
				   struct ethtool_link_ksettings *cmd)
{
	cmd->base.speed		= SPEED_10000;
	cmd->base.duplex	= DUPLEX_FULL;
	cmd->base.port		= PORT_TP;
	cmd->base.autoneg	= AUTONEG_DISABLE;
	return 0;
}

static void veth_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
}

static void veth_get_strings(struct net_device *dev, u32 stringset, u8 *buf)
{
	switch(stringset) {
	case ETH_SS_STATS:
		memcpy(buf, &ethtool_stats_keys, sizeof(ethtool_stats_keys));
		break;
	}
}

static int veth_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(ethtool_stats_keys);
	default:
		return -EOPNOTSUPP;
	}
}

static void veth_get_ethtool_stats(struct net_device *dev,
		struct ethtool_stats *stats, u64 *data)
{
	struct veth_priv *priv = netdev_priv(dev);
	struct net_device *peer = rtnl_dereference(priv->peer);

	data[0] = peer ? peer->ifindex : 0;
}

static const struct ethtool_ops veth_ethtool_ops = {
	.get_drvinfo		= veth_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_strings		= veth_get_strings,
	.get_sset_count		= veth_get_sset_count,
	.get_ethtool_stats	= veth_get_ethtool_stats,
	.get_link_ksettings	= veth_get_link_ksettings,
};

static netdev_tx_t veth_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct veth_priv *priv = netdev_priv(dev);
	struct net_device *rcv;
	int length = skb->len;

	/* Drop packets from stack if we are in zerocopy mode. */
	if (unlikely(priv->tp4_zerocopy)) {
		consume_skb(skb);
		return NETDEV_TX_OK;
	}

	rcu_read_lock();
	rcv = rcu_dereference(priv->peer);
	if (unlikely(!rcv)) {
		kfree_skb(skb);
		goto drop;
	}

	if (likely(dev_forward_skb(rcv, skb) == NET_RX_SUCCESS)) {
		struct pcpu_vstats *stats = this_cpu_ptr(dev->vstats);

		u64_stats_update_begin(&stats->syncp);
		stats->bytes += length;
		stats->packets++;
		u64_stats_update_end(&stats->syncp);
	} else {
drop:
		atomic64_inc(&priv->dropped);
	}
	rcu_read_unlock();
	return NETDEV_TX_OK;
}

static int veth_tp4_xmit(struct net_device *netdev)
{
	struct veth_priv *priv = netdev_priv(netdev);

	local_bh_disable();
	napi_schedule(priv->napi);
	local_bh_enable();

	return NETDEV_TX_OK;
}

static int veth_napi_poll(struct napi_struct *napi, int budget)
{
	struct net_device *netdev = napi->dev;
	struct pcpu_vstats *stats = this_cpu_ptr(netdev->vstats);
	struct veth_priv *priv_rcv, *priv = netdev_priv(netdev);
	struct tp4_queue *tp4q_rx, *tp4q_tx = priv->tp4q_tx;
	int ndescs_rx = 0, ndescs_tx;
	struct net_device *rcv;
	int length = 0;
	int err;
	int i;

	rcu_read_lock();
	rcv = rcu_dereference(priv->peer);
	if (unlikely(!rcv)) {
		tp4q_return_error(tp4q_tx, EAGAIN);
		goto exit;
	}

	priv_rcv = netdev_priv(rcv);
	if (unlikely(!priv_rcv->tp4_zerocopy)) {
		tp4q_return_error(tp4q_tx, EAGAIN);
		goto exit;
	}

	/* To make sure we do not read the tp4_queue pointers
	 * before the other process has enabled zerocopy
	 */
	smp_rmb();

	tp4q_rx = priv_rcv->tp4q_rx;

	ndescs_tx = tp4q_dequeue(tp4q_tx, priv->descs_tx, budget);
	if (ndescs_tx == 0)
		goto exit;

	ndescs_rx = tp4q_dequeue(tp4q_rx, priv->descs_rx, ndescs_tx);

	for (i = 0; i < ndescs_rx; i++) {
		struct tpacket4_desc *desc_rx = &priv->descs_rx[i];
		struct tpacket4_desc *desc_tx = &priv->descs_tx[i];
		struct tpacket4_hdr *hdr_tx, *hdr_rx;
		u64 addr;
		u32 size;

		hdr_tx = tp4q_get_validated_header(tp4q_tx, desc_tx->addr);
		if (!hdr_tx) {
			tp4q_set_error(desc_tx, EBADF);
			tp4q_set_error(desc_rx, EAGAIN);
			atomic64_inc(&priv->dropped);
			continue;
		}

		hdr_rx = tp4q_get_header(tp4q_rx, desc_rx->addr);
		if (!hdr_rx) {
			tp4q_set_error(desc_tx, EAGAIN);
			tp4q_set_error(desc_rx, EBADF);
			atomic64_inc(&priv->dropped);
			continue;
		}

		size = hdr_tx->data_end - hdr_tx->data;
		if (tp4q_rx->umem != tp4q_tx->umem) {
			/* TODO: Right now, only copy if it fits! In
			 * the future copy multiple buffers into Rx if
			 * needed (jumbo frame support).
			 */
			if (size <= tp4q_max_data_size(tp4q_rx)) {
				/* The processes do not share packet
				 * buffer memory.  Copy data between
				 * the two packet buffers
				 */
				tp4q_write_header(tp4q_rx, hdr_rx, size);

				memcpy(tp4q_get_data(tp4q_rx, hdr_rx),
				       tp4q_get_data(tp4q_tx, hdr_tx), size);
			} else {
				tp4q_set_error(desc_tx, EFBIG);
				tp4q_set_error(desc_rx, EFBIG);
				atomic64_inc(&priv->dropped);
				continue;
			}
			addr = desc_rx->addr;
		} else {
			/* Shared packet buffer. Just copy the address. */
			addr = desc_tx->addr;
		}

		tp4q_write_desc(desc_rx, addr, size);
		length += desc_tx->len;
	}

	if (ndescs_tx > ndescs_rx) {
		for (i = ndescs_rx; i < ndescs_tx; i++)
			tp4q_set_error(&priv->descs_tx[i], EAGAIN);
		atomic64_add(ndescs_tx - ndescs_rx, &priv->dropped);
	}

	/* Return buffers to user space */
	err = tp4q_enqueue(tp4q_rx, priv->descs_rx, ndescs_rx);
	WARN_ON(err);
	err = tp4q_enqueue(tp4q_tx, priv->descs_tx, ndescs_tx);
	WARN_ON(err);

	u64_stats_update_begin(&stats->syncp);
	stats->bytes += length;
	stats->packets += ndescs_rx;
	u64_stats_update_end(&stats->syncp);

exit:
	rcu_read_unlock();
	if (ndescs_rx < NAPI_POLL_WEIGHT)
		napi_complete_done(priv->napi, 0);
	return ndescs_rx;
}

/*
 * general routines
 */

static u64 veth_stats_one(struct pcpu_vstats *result, struct net_device *dev)
{
	struct veth_priv *priv = netdev_priv(dev);
	int cpu;

	result->packets = 0;
	result->bytes = 0;
	for_each_possible_cpu(cpu) {
		struct pcpu_vstats *stats = per_cpu_ptr(dev->vstats, cpu);
		u64 packets, bytes;
		unsigned int start;

		do {
			start = u64_stats_fetch_begin_irq(&stats->syncp);
			packets = stats->packets;
			bytes = stats->bytes;
		} while (u64_stats_fetch_retry_irq(&stats->syncp, start));
		result->packets += packets;
		result->bytes += bytes;
	}
	return atomic64_read(&priv->dropped);
}

static void veth_get_stats64(struct net_device *dev,
			     struct rtnl_link_stats64 *tot)
{
	struct veth_priv *priv = netdev_priv(dev);
	struct net_device *peer;
	struct pcpu_vstats one;

	tot->tx_dropped = veth_stats_one(&one, dev);
	tot->tx_bytes = one.bytes;
	tot->tx_packets = one.packets;

	rcu_read_lock();
	peer = rcu_dereference(priv->peer);
	if (peer) {
		tot->rx_dropped = veth_stats_one(&one, peer);
		tot->rx_bytes = one.bytes;
		tot->rx_packets = one.packets;
	}
	rcu_read_unlock();
}

/* fake multicast ability */
static void veth_set_multicast_list(struct net_device *dev)
{
}

static int veth_open(struct net_device *dev)
{
	struct veth_priv *priv = netdev_priv(dev);
	struct net_device *peer = rtnl_dereference(priv->peer);

	if (!peer)
		return -ENOTCONN;

	if (peer->flags & IFF_UP) {
		netif_carrier_on(dev);
		netif_carrier_on(peer);
	}
	return 0;
}

static int veth_close(struct net_device *dev)
{
	struct veth_priv *priv = netdev_priv(dev);
	struct net_device *peer = rtnl_dereference(priv->peer);

	netif_carrier_off(dev);
	if (peer)
		netif_carrier_off(peer);

	return 0;
}

static int is_valid_veth_mtu(int mtu)
{
	return mtu >= ETH_MIN_MTU && mtu <= ETH_MAX_MTU;
}

static int veth_dev_init(struct net_device *dev)
{
	dev->vstats = netdev_alloc_pcpu_stats(struct pcpu_vstats);
	if (!dev->vstats)
		return -ENOMEM;
	return 0;
}

static void veth_dev_free(struct net_device *dev)
{
	free_percpu(dev->vstats);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void veth_poll_controller(struct net_device *dev)
{
	/* veth only receives frames when its peer sends one
	 * Since it's a synchronous operation, we are guaranteed
	 * never to have pending data when we poll for it so
	 * there is nothing to do here.
	 *
	 * We need this though so netpoll recognizes us as an interface that
	 * supports polling, which enables bridge devices in virt setups to
	 * still use netconsole
	 */
}
#endif	/* CONFIG_NET_POLL_CONTROLLER */

static int veth_get_iflink(const struct net_device *dev)
{
	struct veth_priv *priv = netdev_priv(dev);
	struct net_device *peer;
	int iflink;

	rcu_read_lock();
	peer = rcu_dereference(priv->peer);
	iflink = peer ? peer->ifindex : 0;
	rcu_read_unlock();

	return iflink;
}

static void veth_set_rx_headroom(struct net_device *dev, int new_hr)
{
	struct veth_priv *peer_priv, *priv = netdev_priv(dev);
	struct net_device *peer;

	if (new_hr < 0)
		new_hr = 0;

	rcu_read_lock();
	peer = rcu_dereference(priv->peer);
	if (unlikely(!peer))
		goto out;

	peer_priv = netdev_priv(peer);
	priv->requested_headroom = new_hr;
	new_hr = max(priv->requested_headroom, peer_priv->requested_headroom);
	dev->needed_headroom = new_hr;
	peer->needed_headroom = new_hr;

out:
	rcu_read_unlock();
}

static int veth_tp4_disable(struct net_device *netdev,
			    struct tp4_netdev_parms *params)
{
	struct veth_priv *priv_rcv, *priv = netdev_priv(netdev);
	struct net_device *rcv;

	priv->tp4_zerocopy = false;

	/* Make sure other process sees zero copy as off before starting
	 * to turn things off
	 */
	smp_wmb();

	napi_disable(priv->napi);
	netif_napi_del(priv->napi);

	rcu_read_lock();
	rcv = rcu_dereference(priv->peer);
	if (!rcv) {
		WARN_ON(!rcv);
		goto exit;
	}
	priv_rcv = netdev_priv(rcv);

	if (priv_rcv->tp4_zerocopy) {
		/* Wait for other thread to complete
		 * before removing tp4 queues
		 */
		napi_synchronize(priv_rcv->napi);
	}
exit:
	rcu_read_unlock();

	tp4q_disable(&netdev->dev, priv->tp4q_rx);
	tp4q_disable(&netdev->dev, priv->tp4q_tx);
	kfree(priv->napi);
	kfree(priv->descs_rx);
	kfree(priv->descs_tx);

	return 0;
}

static int veth_tp4_enable(struct net_device *netdev,
			   struct tp4_netdev_parms *params)
{
	struct veth_priv *priv = netdev_priv(netdev);
	int err;

	priv->napi = kzalloc(sizeof(*priv->napi), GFP_KERNEL);
	if (!priv->napi)
		return -ENOMEM;

	netif_napi_add(netdev, priv->napi, veth_napi_poll,
		       NAPI_POLL_WEIGHT);

	err = tp4q_enable(&netdev->dev, params->tp4q_rx, DMA_NONE);
	if (err)
		goto rxq_err;

	err = tp4q_enable(&netdev->dev, params->tp4q_tx, DMA_NONE);
	if (err)
		goto txq_err;

	priv->descs_rx = kmalloc_array(NAPI_POLL_WEIGHT,
				       sizeof(*priv->descs_rx), GFP_KERNEL);
	if (!priv->descs_rx) {
		err = -ENOMEM;
		goto descs_rx_err;
	}

	priv->descs_tx = kmalloc_array(NAPI_POLL_WEIGHT,
				       sizeof(*priv->descs_tx), GFP_KERNEL);
	if (!priv->descs_tx) {
		err = -ENOMEM;
		goto descs_tx_err;
	}

	priv->tp4q_rx = params->tp4q_rx;
	priv->tp4q_tx = params->tp4q_tx;

	/* Make sure other process sees queues initialized before enabling
	 * zerocopy mode
	 */
	smp_wmb();
	priv->tp4_zerocopy = true;
	napi_enable(priv->napi);

	return 0;

descs_tx_err:
	kfree(priv->descs_rx);
descs_rx_err:
	tp4q_disable(&netdev->dev, params->tp4q_tx);
txq_err:
	tp4q_disable(&netdev->dev, params->tp4q_rx);
rxq_err:
	netif_napi_del(priv->napi);
	kfree(priv->napi);
	return err;
}

static int veth_tp4_zerocopy(struct net_device *netdev,
			     struct tp4_netdev_parms *params)
{
	switch (params->command) {
	case TP4_ENABLE:
		return veth_tp4_enable(netdev, params);

	case TP4_DISABLE:
		return veth_tp4_disable(netdev, params);

	default:
		return -ENOTSUPP;
	}
}

static const struct net_device_ops veth_netdev_ops = {
	.ndo_init            = veth_dev_init,
	.ndo_open            = veth_open,
	.ndo_stop            = veth_close,
	.ndo_start_xmit      = veth_xmit,
	.ndo_get_stats64     = veth_get_stats64,
	.ndo_set_rx_mode     = veth_set_multicast_list,
	.ndo_set_mac_address = eth_mac_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= veth_poll_controller,
#endif
	.ndo_get_iflink		= veth_get_iflink,
	.ndo_features_check	= passthru_features_check,
	.ndo_set_rx_headroom	= veth_set_rx_headroom,
	.ndo_tp4_zerocopy	= veth_tp4_zerocopy,
	.ndo_tp4_xmit           = veth_tp4_xmit,
};

#define VETH_FEATURES (NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_HW_CSUM | \
		       NETIF_F_RXCSUM | NETIF_F_SCTP_CRC | NETIF_F_HIGHDMA | \
		       NETIF_F_GSO_SOFTWARE | NETIF_F_GSO_ENCAP_ALL | \
		       NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX | \
		       NETIF_F_HW_VLAN_STAG_TX | NETIF_F_HW_VLAN_STAG_RX )

static void veth_setup(struct net_device *dev)
{
	ether_setup(dev);

	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
	dev->priv_flags |= IFF_NO_QUEUE;
	dev->priv_flags |= IFF_PHONY_HEADROOM;

	dev->netdev_ops = &veth_netdev_ops;
	dev->ethtool_ops = &veth_ethtool_ops;
	dev->features |= NETIF_F_LLTX;
	dev->features |= VETH_FEATURES;
	dev->vlan_features = dev->features &
			     ~(NETIF_F_HW_VLAN_CTAG_TX |
			       NETIF_F_HW_VLAN_STAG_TX |
			       NETIF_F_HW_VLAN_CTAG_RX |
			       NETIF_F_HW_VLAN_STAG_RX);
	dev->needs_free_netdev = true;
	dev->priv_destructor = veth_dev_free;
	dev->max_mtu = ETH_MAX_MTU;

	dev->hw_features = VETH_FEATURES;
	dev->hw_enc_features = VETH_FEATURES;
	dev->mpls_features = NETIF_F_HW_CSUM | NETIF_F_GSO_SOFTWARE;
}

/*
 * netlink interface
 */

static int veth_validate(struct nlattr *tb[], struct nlattr *data[],
			 struct netlink_ext_ack *extack)
{
	if (tb[IFLA_ADDRESS]) {
		if (nla_len(tb[IFLA_ADDRESS]) != ETH_ALEN)
			return -EINVAL;
		if (!is_valid_ether_addr(nla_data(tb[IFLA_ADDRESS])))
			return -EADDRNOTAVAIL;
	}
	if (tb[IFLA_MTU]) {
		if (!is_valid_veth_mtu(nla_get_u32(tb[IFLA_MTU])))
			return -EINVAL;
	}
	return 0;
}

static struct rtnl_link_ops veth_link_ops;

static int veth_newlink(struct net *src_net, struct net_device *dev,
			struct nlattr *tb[], struct nlattr *data[],
			struct netlink_ext_ack *extack)
{
	int err;
	struct net_device *peer;
	struct veth_priv *priv;
	char ifname[IFNAMSIZ];
	struct nlattr *peer_tb[IFLA_MAX + 1], **tbp;
	unsigned char name_assign_type;
	struct ifinfomsg *ifmp;
	struct net *net;

	/*
	 * create and register peer first
	 */
	if (data != NULL && data[VETH_INFO_PEER] != NULL) {
		struct nlattr *nla_peer;

		nla_peer = data[VETH_INFO_PEER];
		ifmp = nla_data(nla_peer);
		err = rtnl_nla_parse_ifla(peer_tb,
					  nla_data(nla_peer) + sizeof(struct ifinfomsg),
					  nla_len(nla_peer) - sizeof(struct ifinfomsg),
					  NULL);
		if (err < 0)
			return err;

		err = veth_validate(peer_tb, NULL, extack);
		if (err < 0)
			return err;

		tbp = peer_tb;
	} else {
		ifmp = NULL;
		tbp = tb;
	}

	if (ifmp && tbp[IFLA_IFNAME]) {
		nla_strlcpy(ifname, tbp[IFLA_IFNAME], IFNAMSIZ);
		name_assign_type = NET_NAME_USER;
	} else {
		snprintf(ifname, IFNAMSIZ, DRV_NAME "%%d");
		name_assign_type = NET_NAME_ENUM;
	}

	net = rtnl_link_get_net(src_net, tbp);
	if (IS_ERR(net))
		return PTR_ERR(net);

	peer = rtnl_create_link(net, ifname, name_assign_type,
				&veth_link_ops, tbp);
	if (IS_ERR(peer)) {
		put_net(net);
		return PTR_ERR(peer);
	}

	if (!ifmp || !tbp[IFLA_ADDRESS])
		eth_hw_addr_random(peer);

	if (ifmp && (dev->ifindex != 0))
		peer->ifindex = ifmp->ifi_index;

	err = register_netdevice(peer);
	put_net(net);
	net = NULL;
	if (err < 0)
		goto err_register_peer;

	netif_carrier_off(peer);

	err = rtnl_configure_link(peer, ifmp);
	if (err < 0)
		goto err_configure_peer;

	/*
	 * register dev last
	 *
	 * note, that since we've registered new device the dev's name
	 * should be re-allocated
	 */

	if (tb[IFLA_ADDRESS] == NULL)
		eth_hw_addr_random(dev);

	if (tb[IFLA_IFNAME])
		nla_strlcpy(dev->name, tb[IFLA_IFNAME], IFNAMSIZ);
	else
		snprintf(dev->name, IFNAMSIZ, DRV_NAME "%%d");

	err = register_netdevice(dev);
	if (err < 0)
		goto err_register_dev;

	netif_carrier_off(dev);

	/*
	 * tie the deviced together
	 */

	priv = netdev_priv(dev);
	rcu_assign_pointer(priv->peer, peer);
	priv->tp4_zerocopy = false;

	priv = netdev_priv(peer);
	rcu_assign_pointer(priv->peer, dev);
	priv->tp4_zerocopy = false;
	return 0;

err_register_dev:
	/* nothing to do */
err_configure_peer:
	unregister_netdevice(peer);
	return err;

err_register_peer:
	free_netdev(peer);
	return err;
}

static void veth_dellink(struct net_device *dev, struct list_head *head)
{
	struct veth_priv *priv;
	struct net_device *peer;

	priv = netdev_priv(dev);
	peer = rtnl_dereference(priv->peer);

	/* Note : dellink() is called from default_device_exit_batch(),
	 * before a rcu_synchronize() point. The devices are guaranteed
	 * not being freed before one RCU grace period.
	 */
	RCU_INIT_POINTER(priv->peer, NULL);
	unregister_netdevice_queue(dev, head);

	if (peer) {
		priv = netdev_priv(peer);
		RCU_INIT_POINTER(priv->peer, NULL);
		unregister_netdevice_queue(peer, head);
	}
}

static const struct nla_policy veth_policy[VETH_INFO_MAX + 1] = {
	[VETH_INFO_PEER]	= { .len = sizeof(struct ifinfomsg) },
};

static struct net *veth_get_link_net(const struct net_device *dev)
{
	struct veth_priv *priv = netdev_priv(dev);
	struct net_device *peer = rtnl_dereference(priv->peer);

	return peer ? dev_net(peer) : dev_net(dev);
}

static struct rtnl_link_ops veth_link_ops = {
	.kind		= DRV_NAME,
	.priv_size	= sizeof(struct veth_priv),
	.setup		= veth_setup,
	.validate	= veth_validate,
	.newlink	= veth_newlink,
	.dellink	= veth_dellink,
	.policy		= veth_policy,
	.maxtype	= VETH_INFO_MAX,
	.get_link_net	= veth_get_link_net,
};

/*
 * init/fini
 */

static __init int veth_init(void)
{
	return rtnl_link_register(&veth_link_ops);
}

static __exit void veth_exit(void)
{
	rtnl_link_unregister(&veth_link_ops);
}

module_init(veth_init);
module_exit(veth_exit);

MODULE_DESCRIPTION("Virtual Ethernet Tunnel");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_RTNL_LINK(DRV_NAME);
