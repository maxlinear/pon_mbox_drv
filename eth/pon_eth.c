/******************************************************************************
 *
 *  Copyright (c) 2020 - 2024 MaxLinear, Inc.
 *  Copyright (C) 2017 - 2020 Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 ******************************************************************************/

/** This driver uses the datapath library to send and receive data packets from
 *  the PON IP.
 *  One hardware net device is automatically registered when a PON IP devices,
 *  was probed through device tree. This device will not be able to transmit or
 *  receive any data packets.
 *  For each T-CONT a device of type tcont has to be created on top
 *  of the PON device. This T-CONT device takes the T-CONT index or the
 *  T-CONT ID, if the T-CONT ID is given, it will fetch the Index from the
 *  PON FW.
 *  For each GEM port an interface of type gem must be created.
 *  This interface gets the GEM port index or the GEM port ID, if
 *  only the GEM port ID is given, it will fetch the index from the PON FW.
 *  For upstream traffic to flow the GEM port has to be connected to a T-CONT
 *  interface. This will apply the settings to the dp library.
 *  Traffic can only be send and received on the GEM port interface.
 *
 *  This is an example configuration done with the ip command:
 *
 *  ip link add link pon0 tcont1033 type tcont id 1033
 *  ip link add link pon0 gem1123 type gem id 1123 traffic_type eth dir \
 *          ds enc dis
 *  ip link set gem1123 type gem tcont tcont1033 dir bi
 *  ip link add link pon0 pmapper1 type pmapper
 *  ip link set pmapper1 type pmapper pcp 0 gem1123 pcp 1 gem1123 pcpdef 1
 *  ip link set pmapper1 type pmapper mode dscp
 *  ip link set pmapper1 type pmapper dscp 1 0 dscp 4 3 dscp 5 1 dscp 6 4
 *
 */

#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <net/datapath_api.h>
#include <net/datapath_br_domain.h>
#include <net/datapath_api_qos.h>
#include <net/dsfield.h>
#include <net/rtnetlink.h>
#include <net/switchdev.h>
#if IS_ENABLED(CONFIG_QOS_TC)
#include <net/qos_tc.h>
#endif

#include <pon/pon_ip_msg.h>
#include <pon/pon_mbox_ikm.h>
#include "pon_eth.h"
#include "pon_eth_iphost.h"

#ifdef EXTRA_VERSION
#define pon_eth_extra_ver_str "." EXTRA_VERSION
#else
#define pon_eth_extra_ver_str ""
#endif

/** what string support, version string */
const char pon_eth_whatversion[] = "@(#)MaxLinear PON network driver, version "
				    __stringify(PACKAGE_VERSION)
				    pon_eth_extra_ver_str;

/** Maximum number of TX queues each T-Cont can have */
#define PON_TCONT_TX_QUEUES		8
/** Maximum number of TX and RX queues to the CPU */
#define PON_CPU_QUEUES			8

#define QUEUE_LOOKUP_MODE_FLOW_ID_TC_4BIT 0
#define QUEUE_LOOKUP_MODE_SUBIF_ID 1
#define QUEUE_LOOKUP_MODE_SUBIF_ID_TC_4BIT 2
#define QUEUE_LOOKUP_MODE_SUBIF_ID_TC_3BIT 3

/* OUI Extended Ethertype (IEEE Std 802) */
#define ETH_P_OUI		0x88B7

/* Common structure for PON net devices */
struct ltq_pon_net_common {
	u32 ethtool_flags;
	#define ETHTOOL_FLAG_BP_CPU_ENABLE	BIT(0)
};

/* ethtool flag for pon0 interface */
#define ETHTOOL_FLAG_CLASS_SELECT_SPCP_DSCP BIT(0)

/* Use default PCP value for DSCP bit */
#define PON_PMAPPER_DSCP_DEFAULT 0xff

/** This represents the IEEE 802.1p mapper configuration.
 */
struct ltq_pon_net_pmapper {
	struct ltq_pon_net_common common;
	struct list_head node;
	struct net_device *ndev;
	struct ltq_pon_net_hw *hw;
	struct net *src_net;
	enum DP_PMAP_MODE mode;
	u8 pcpdef;
	struct ltq_pon_net_gem *gem[DP_PMAP_PCP_NUM];
	u8 dscp_map[DP_PMAP_DSCP_NUM];
	u32 subif[DP_PMAP_PCP_NUM];
	u32 subif_def;
	struct rtnl_link_stats64 stats;
};

/** This represents the T-CONT or allocation ID configuration with its index.
 * The qos_idx points to the hardware traffic container instance which
 * handles the upstream timing for a specific allocation, identified by its
 * alloc_id number.
 */
struct ltq_pon_net_tcont {
	struct list_head node;
	struct net_device *ndev;
	struct ltq_pon_net_hw *hw;
	bool alloc_id_valid;
	u32 alloc_id;
	u16 alloc_idx;
	int queue_id;
	struct list_head gem_list;
	/** This value identifies the dequeue port from which the data for
	 *  for the Allocation ID shall be pulled by hardware.
	 */
	u16 qos_idx;
	/** This link reference identifies a specific linking of an allocation
	 *  ID to an allocation's hardware index.
	 */
	u32 alloc_link_ref;
	/** Identifies OMCI T-CONT */
	bool tcont_omci;
};

/** This represents GEM port configuration with its index. */
struct ltq_pon_net_gem {
	struct ltq_pon_net_common common;
	struct list_head node;
	struct list_head tcont_node;
	struct net_device *ndev;
	/* Network  device registered to dp, changes when added to pmapper */
	struct net_device *dp_ndev;
	struct ltq_pon_net_hw *hw;
	struct net *src_net;
	u16 gem_idx;
	u32 gem_id;
	u16 gem_max_size;
	u8 gem_traffic_type;
	u8 gem_dir;
	u8 gem_enc;
	/* true when GEM port is also configured in the PON FW */
	bool valid;
	/* DP registered */
	bool dp_connected;
	/* DP update required */
	bool dp_update;
	/* Identifies multicast GEM */
	bool multicast;
	struct ltq_pon_net_tcont *tcont;
	struct ltq_pon_net_pmapper *pmapper;
	struct rtnl_link_stats64 stats;
	/** This link reference identifies a specific linking of an allocation
	 *  ID to an allocation's hardware index.
	 */
	u32 alloc_link_ref;
};

/** Maximum number of allocations which are supported by the PON IP hardware */
#define PON_ALLOC_MAX 64

/** Maximum number of gems which are supported by the PON IP hardware */
#define PON_GEM_MAX 256

/** Alloc IDX mask.
 *  The lower 8 bit from ALLOC_LINK_REF value are always identical to the
 *  ALLOC_IDX value
 */
#define ALLOC_IDX_MASK 0xFF

/** This array identifies which QOS Indexes are currently used by
 *  T-CONT netdevices.
 */
static bool g_qos_idx_used[PON_ALLOC_MAX];

/** This array identifies which GEM Indexes are currently used by
 *  GEM netdevices.
 */
static bool g_gem_idx_used[PON_GEM_MAX];

/** This array is a shadow storage of qos idx per alloc idx.
 *  TODO: Remove in PONRTSYS-9459.
 */
static u8 g_alloc_idx_to_qos_idx[PON_ALLOC_MAX];

/** This pointer identifies PON netdevice private data used for firmware
 *  event handling.
 */
static struct ltq_pon_net_hw *g_hw;

static int ltq_pon_net_gem_disconnect(struct ltq_pon_net_gem *gem,
				      struct ltq_pon_net_tcont *tcont);
static int ltq_pon_net_pmapper_gem_remove(struct ltq_pon_net_pmapper *pmapper,
					  int idx);
static int ltq_pon_net_dp_gem_update(struct ltq_pon_net_gem *gem,
				     struct net_device *master);
static void ltq_pon_net_tcont_gems_activate(struct ltq_pon_net_tcont *tcont);

/** The hardware interface open callback.
 * This interface does not support traffic and is only used for management.
 */
int ltq_pon_net_mdev_open(struct net_device *hw_ndev)
{
	return -EBUSY;
}

/** The hardware interface transmit callback.
 * This interface does not support traffic and is only used for management.
 */
static netdev_tx_t ltq_pon_net_mdev_start_xmit(struct sk_buff *skb,
					       struct net_device *hw_ndev)
{
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

#if IS_ENABLED(CONFIG_QOS_TC)
#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE)
static int ltq_pon_net_mdev_setup_tc(struct net_device *hw_ndev, u32 handle,
				      __be16 protocol, struct tc_to_netdev *tc)
{
	return qos_tc_setup(hw_ndev, handle, protocol, tc, 0, 0);
}
#else
static int ltq_pon_net_mdev_setup_tc(struct net_device *hw_ndev,
				       enum tc_setup_type type,
				       void *type_data)
{
	return qos_tc_setup(hw_ndev, type, type_data, 0, 0);
}
#endif
#endif

#define SFF_8079_ADDR_MODE	92
#define SFF_8472_IMPLEMENTED	BIT(6)
static int get_module_info(struct net_device *dev,
			   struct ethtool_modinfo *modinfo)
{
	u8 addr_mode = 0;
	int ret;

	ret = pon_sfp_read(0, SFF_8079_ADDR_MODE,
			   &addr_mode, sizeof(addr_mode));
	if (ret == sizeof(addr_mode) && (addr_mode & SFF_8472_IMPLEMENTED)) {
		/* We have an SFP which supports a revision of SFF-8472 */
		modinfo->type = ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
	} else {
		/* We have an SFP, but it does not support SFF-8472 */
		modinfo->type = ETH_MODULE_SFF_8079;
		modinfo->eeprom_len = ETH_MODULE_SFF_8079_LEN;
	}

	return 0;
}

static int get_module_eeprom(struct net_device *dev,
			     struct ethtool_eeprom *ee,
			     u8 *data)
{
	int i = 0;
	u32 offset[2];
	u32 len[2];
	int ret = 0;

	if (ee->len == 0)
		return -EINVAL;

	if (ee->offset < ETH_MODULE_SFF_8079_LEN) {
		if (ee->offset + ee->len > ETH_MODULE_SFF_8079_LEN) {
			offset[0] = ee->offset;
			len[0] = ETH_MODULE_SFF_8079_LEN - offset[0];
			offset[1] = 0;
			len[1] = ee->len - len[0];
		} else {
			offset[0] = ee->offset;
			len[0] = ee->len;
			offset[1] = 0;
			len[1] = 0;
		}
	} else {
		offset[0] = ETH_MODULE_SFF_8079_LEN;
		len[0] = 0;
		offset[1] = ee->offset - ETH_MODULE_SFF_8079_LEN;
		len[1] = ee->len;
	}

	for (i = 0; i < 2; i++) {
		if (!len[i])
			continue;
		ret = pon_sfp_read(i, offset[i],
				   &data[offset[i] + (256 * i) - ee->offset],
				   len[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static u32 get_mdev_priv_flags(struct net_device *dev)
{
	struct ltq_pon_net_hw *hw = netdev_priv(dev);

	return hw->ethtool_flags;
}

static int set_mdev_priv_flags(struct net_device *dev, u32 flags)
{
	GSW_return_t ret = 0;
	GSW_QoS_portCfg_t qosPortCfg = {0, };
	struct ltq_pon_net_hw *hw = netdev_priv(dev);
	struct core_ops *gsw_handle = NULL;

	if (!(flags ^ hw->ethtool_flags))
		return 0;

	gsw_handle = gsw_get_swcore_ops(0);
	if (!gsw_handle) {
		pr_err("%s failed for device: %s - no gsw handle\n",
		       __func__, dev->name);
		return -EINVAL;
	}

	qosPortCfg.nPortId = hw->port_id;
	if (flags & ETHTOOL_FLAG_CLASS_SELECT_SPCP_DSCP)
		qosPortCfg.eClassMode = GSW_QOS_CLASS_SELECT_SPCP_DSCP;
	else
		qosPortCfg.eClassMode = GSW_QOS_CLASS_SELECT_NO;

	ret = gsw_handle->gsw_qos_ops.QoS_PortCfgSet(gsw_handle,
						     &qosPortCfg);
	if (ret != GSW_statusOk) {
		pr_err("%s: QoS_PortCfgSet failed for port %d: %d\n",
		       __func__, qosPortCfg.nPortId, ret);
		return ret;
	}
	hw->ethtool_flags = flags & ETHTOOL_FLAG_CLASS_SELECT_SPCP_DSCP;

	return 0;
}

static const char priv_mdev_flags_strings[][ETH_GSTRING_LEN] = {
	"qos_class_select_spcp_dscp",
};

#define PRIV_MDEV_FLAGS_STR_LEN ARRAY_SIZE(priv_mdev_flags_strings)

static void get_mdev_strings(struct net_device *netdev, u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_PRIV_FLAGS:
		for (i = 0; i < PRIV_MDEV_FLAGS_STR_LEN; i++) {
			memcpy(data, priv_mdev_flags_strings[i],
			       ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_STATS:
		dp_net_dev_get_ss_stat_strings(netdev, data);
		break;
	default:
		break;
	}
}

static int get_mdev_stringset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_PRIV_FLAGS:
		return PRIV_MDEV_FLAGS_STR_LEN;
	case ETH_SS_STATS:
		return dp_net_dev_get_ss_stat_strings_count(netdev);
	default:
		return -EOPNOTSUPP;
	}
}

static void add_ani_archived_stats(struct net_device *netdev, u64 *data,
				   int count)
{
	int i;
	struct ltq_pon_net_hw *hw = netdev_priv(netdev);

	if (!hw->num_ani_archived_stats)
		return;

	if (hw->num_ani_archived_stats != count) {
		netdev_err(netdev,
			   "%s: unexpected number of ANI archived counters\n",
			   __func__);
		return;
	}

	for (i = 0; i < hw->num_ani_archived_stats; i++)
		data[i] += hw->ani_archived_stats[i];
}

static void get_mdev_ethtool_stats(struct net_device *netdev,
				   struct ethtool_stats *stats, u64 *data)
{
	struct ltq_pon_net_hw *hw = netdev_priv(netdev);
	struct ltq_pon_net_gem *in_ele;
	u64 *stat;
	int i;

	memset(data, 0, stats->n_stats * sizeof(*data));

	stat = kcalloc(stats->n_stats, sizeof(*stat), GFP_KERNEL);
	if (!stat)
		return;

	list_for_each_entry(in_ele, &hw->gem_list, node) {
		int count;

		/*
		 * Getting a different number of counters is a sign that
		 * something changed in the DP and summing these
		 * counters by indices would not make sense.
		 */
		count = dp_net_dev_get_ss_stat_strings_count(in_ele->ndev);
		if (count != stats->n_stats) {
			netdev_err(netdev, "%s: unexpected number of counters for %s (expected %d, got %d)\n",
				   __func__, in_ele->ndev->name,
				   stats->n_stats, count);
			continue;
		}

		memset(stat, 0, stats->n_stats * sizeof(*stat));
		dp_net_dev_get_ethtool_stats(in_ele->ndev, stats, stat);
		for (i = 0; i < stats->n_stats; i++)
			data[i] += stat[i];
	}

	add_ani_archived_stats(netdev, data, stats->n_stats);

	kfree(stat);
}

/** Ethtool ops to provide access to the SFP module EEPROM devices
 *  and private flags.
 *  Only provided on the base (pon0) interface.
 */
static const struct ethtool_ops mdev_ethtool_ops = {
	.get_module_info	= get_module_info,
	.get_module_eeprom	= get_module_eeprom,
	.get_strings		= get_mdev_strings,
	.get_sset_count		= get_mdev_stringset_count,
	.get_priv_flags		= get_mdev_priv_flags,
	.set_priv_flags		= set_mdev_priv_flags,
	.get_ethtool_stats	= get_mdev_ethtool_stats,
};

/** Net dev ops for the hardware interface.
 * This is only used for management of the interfaces. Other GPON interfaces
 * can be added to the interface as links.
 */
static const struct net_device_ops ltq_pon_net_mdev_ops = {
	.ndo_open		= ltq_pon_net_mdev_open,
	.ndo_start_xmit		= ltq_pon_net_mdev_start_xmit,
#if IS_ENABLED(CONFIG_QOS_TC)
	.ndo_setup_tc		= ltq_pon_net_mdev_setup_tc,
#endif
};

static u32 get_priv_flags(struct net_device *dev)
{
	struct ltq_pon_net_common *np = netdev_priv(dev);

	return np->ethtool_flags;
}

static int set_priv_flags(struct net_device *dev, u32 flags)
{
	struct ltq_pon_net_common *np = netdev_priv(dev);
	struct dp_bp_attr bp_conf = {0};
	int ret = 0;

	bp_conf.dev = dev;

	if (flags & ETHTOOL_FLAG_BP_CPU_ENABLE) {
		bp_conf.en = true;
		ret = dp_set_bp_attr(&bp_conf, 0);
		if (ret != 0)
			return ret;
		np->ethtool_flags |= ETHTOOL_FLAG_BP_CPU_ENABLE;
	} else {
		bp_conf.en = false;
		ret = dp_set_bp_attr(&bp_conf, 0);
		if (ret != 0)
			return ret;
		np->ethtool_flags &= ~ETHTOOL_FLAG_BP_CPU_ENABLE;
	}

	return ret;
}

static const char priv_flags_strings[][ETH_GSTRING_LEN] = {
	"bp_to_cpu_enable",
};

#define PRIV_FLAGS_STR_LEN ARRAY_SIZE(priv_flags_strings)

static void get_strings(struct net_device *netdev, u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_PRIV_FLAGS:
		for (i = 0; i < PRIV_FLAGS_STR_LEN; i++) {
			memcpy(data, priv_flags_strings[i],
			       ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_STATS:
		dp_net_dev_get_ss_stat_strings(netdev, data);
		break;
	default:
		break;
	}
}

static int get_stringset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_PRIV_FLAGS:
		return PRIV_FLAGS_STR_LEN;
	case ETH_SS_STATS:
		return dp_net_dev_get_ss_stat_strings_count(netdev);
	default:
		return -EOPNOTSUPP;
	}
}

static void get_ethtool_stats(struct net_device *netdev,
			      struct ethtool_stats *stats, u64 *data)
{
	dp_net_dev_get_ethtool_stats(netdev, stats, data);
}

/* Structure of the ether tool operation */
static const struct ethtool_ops ethtool_ops = {
	.get_strings		= get_strings,
	.get_sset_count		= get_stringset_count,
	.get_priv_flags		= get_priv_flags,
	.set_priv_flags		= set_priv_flags,
	.get_ethtool_stats	= get_ethtool_stats,
};

/** The GEM port interface open callback. */
static int ltq_pon_net_gem_open(struct net_device *gem_ndev)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);

	if (!gem->dp_connected && gem->ndev == gem->dp_ndev)
		return -EIO;

	/* Enable packet receiving in datapath */
	if (dp_rx_enable(gem_ndev, gem_ndev->name, DP_RX_ENABLE) !=
	    DP_SUCCESS) {
		netdev_err(gem_ndev, "dp_rx enable failed\n");
		return -EIO;
	}

	return 0;
}

/** The GEM port interface stop callback. */
static int ltq_pon_net_gem_stop(struct net_device *gem_ndev)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);

	if (!gem->dp_connected && gem->ndev == gem->dp_ndev)
		return -EIO;

	/* Disable packet receiving in datapath */
	if (dp_rx_enable(gem_ndev, gem_ndev->name, DP_RX_DISABLE) !=
	    DP_SUCCESS) {
		netdev_err(gem_ndev, "dp_rx disable failed\n");
		return -EIO;
	}

	return 0;
}

static void ltq_pon_net_gem_uninit(struct net_device *gem_ndev)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);
	struct ltq_pon_net_pmapper *pmapper = gem->pmapper;
	struct ltq_pon_net_hw *hw = gem->hw;
	int i;

	list_del(&gem->node);

	if (pmapper) {
		for (i = 0; i < DP_PMAP_PCP_NUM; i++) {
			if (pmapper->gem[i] == gem)
				ltq_pon_net_pmapper_gem_remove(pmapper, i);
		}
	}

	if (gem->dp_connected)
		ltq_pon_net_gem_disconnect(gem, gem->tcont);
	dev_put(hw->ndev);
}

/** Transmit one data packet with the dp lib, using the subif based on the
 * GEM port.
 */
static netdev_tx_t ltq_pon_net_gem_start_xmit(struct sk_buff *skb,
					      struct net_device *gem_ndev)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);
	int32_t ret;
	uint32_t dp_flags = 0;
	dp_subif_t dp_subif;
	unsigned int len = skb->len;
	const struct ethhdr *eth = eth_hdr(skb);

	/* OMCI GEM and trying to send packet with wrong protocol?
	 * We cannot use skb->protocol here, as this is not set for raw sockets,
	 * which are used for the OMCI communication.
	 */
	if (gem->gem_idx == 0 && eth->h_proto != htons(ETH_P_OUI)) {
		netdev_dbg(gem_ndev, "packet with eth_type 0x%04X dropped\n",
			   ntohs(eth->h_proto));
		gem->stats.tx_dropped++;
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	/* only do padding on non-OMCI packets */
	if (gem->gem_idx) {
		if (skb_put_padto(skb, ETH_ZLEN)) {
			gem->stats.tx_dropped++;
			return NETDEV_TX_OK;
		}
	}

	dp_subif.port_id = gem->hw->port_id;
	dp_subif.subif = gem->gem_idx;

	ltq_pon_net_set_skd_dw(skb, dp_subif.port_id, dp_subif.subif);

	if (gem_ndev->features & NETIF_F_HW_CSUM)
		dp_flags |= DP_TX_CAL_CHKSUM;

	ret = dp_xmit(gem_ndev, &dp_subif, skb, skb->len, dp_flags);
	if (!ret) {
		gem->stats.tx_packets++;
		gem->stats.tx_bytes += len;
	} else {
		gem->stats.tx_dropped++;
	}

	return NETDEV_TX_OK;
}

static int ltq_pon_net_gem_eth_change_mtu(
			struct net_device *gem_ndev, int new_mtu)
{
	int ret;

	if (new_mtu == gem_ndev->mtu)
		return 0;

#if (KERNEL_VERSION(4, 10, 0) <= LINUX_VERSION_CODE)
	if (new_mtu < gem_ndev->min_mtu) {
		netdev_err(gem_ndev,
			"%s Invalid MTU %d requested, hw min %d\n",
			__func__, new_mtu, gem_ndev->min_mtu);
		return -EINVAL;
	}

	if (gem_ndev->max_mtu > 0 && new_mtu > gem_ndev->max_mtu) {
		netdev_err(gem_ndev,
			"%s Invalid MTU %d requested, hw max %d\n",
			__func__, new_mtu, gem_ndev->max_mtu);
		return -EINVAL;
	}
#endif

	ret = dp_set_mtu_size(gem_ndev, (u32)new_mtu);
	if (ret != DP_SUCCESS) {
		netdev_err(gem_ndev,
			"%s new_mtu = %d failed (dp_set_mtu_size=%d)\n",
			__func__, new_mtu, ret);
		return -EINVAL;
	}
	gem_ndev->mtu = new_mtu;

	return 0;
}

#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
static struct
rtnl_link_stats64 *ltq_pon_net_gem_get_stats(struct net_device *gem_ndev,
					     struct rtnl_link_stats64 *storage)
#else
static void ltq_pon_net_gem_get_stats(struct net_device *gem_ndev,
				      struct rtnl_link_stats64 *storage)
#endif
{
#ifndef CONFIG_DPM_DATAPATH_MIB
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);

	*storage = gem->stats;
#else
	dp_get_netif_stats(gem_ndev, NULL, storage, 0);
#endif

#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
	return storage;
#else
	return;
#endif
}

/** Returns the ifindex of the PON HW netdev interface,
 *  for the given GEM port.
 */
static int ltq_pon_net_gem_get_iflink(const struct net_device *gem_ndev)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);
	struct net_device *hw_ndev = gem->hw->ndev;

	return hw_ndev->ifindex;
}

#if IS_ENABLED(CONFIG_QOS_TC)
#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE)
static int ltq_pon_net_gem_setup_tc(struct net_device *gem_ndev, u32 handle,
				      __be16 protocol, struct tc_to_netdev *tc)
{
	return qos_tc_setup(gem_ndev, handle, protocol, tc, -1, -1);
}
#else
static int ltq_pon_net_gem_setup_tc(struct net_device *gem_ndev,
				       enum tc_setup_type type,
				       void *type_data)
{
	return qos_tc_setup(gem_ndev, type, type_data, -1, -1);
}
#endif
#endif

/** The per GEM port net ops. */
static const struct net_device_ops ltq_pon_net_gem_ops = {
	.ndo_open		= ltq_pon_net_gem_open,
	.ndo_stop		= ltq_pon_net_gem_stop,
	.ndo_uninit		= ltq_pon_net_gem_uninit,
	.ndo_start_xmit		= ltq_pon_net_gem_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= ltq_pon_net_gem_eth_change_mtu,
	.ndo_get_stats64	= ltq_pon_net_gem_get_stats,
	.ndo_get_iflink		= ltq_pon_net_gem_get_iflink,
#if IS_ENABLED(CONFIG_QOS_TC)
	.ndo_setup_tc		= ltq_pon_net_gem_setup_tc,
#endif
#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE) /* removed in 4.14 */
	.ndo_bridge_setlink	= switchdev_port_bridge_setlink,
	.ndo_bridge_getlink	= switchdev_port_bridge_getlink,
#endif
};

/** The T-CONT interface open callback.
 * This interface does not support traffic and is only used for management.
 */
static int ltq_pon_net_tcont_open(struct net_device *hw_ndev)
{
	return -EBUSY;
}

/** Returns the ifindex of the PON IP HW interface for the given T-CONT. */
static int ltq_pon_net_tcont_get_iflink(const struct net_device *tcont_ndev)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);
	struct net_device *hw_ndev = tcont->hw->ndev;

	return hw_ndev->ifindex;
}

#if IS_ENABLED(CONFIG_QOS_TC)
#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE)
static int ltq_pon_net_tcont_setup_tc(struct net_device *tcont_ndev, u32 handle,
				      __be16 protocol, struct tc_to_netdev *tc)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);

	return qos_tc_setup(tcont_ndev, handle, protocol, tc,
				tcont->hw->port_id, tcont->qos_idx);
}
#else
static int ltq_pon_net_tcont_setup_tc(struct net_device *tcont_ndev,
				       enum tc_setup_type type,
				       void *type_data)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);

	return qos_tc_setup(tcont_ndev, type, type_data,
				tcont->hw->port_id, tcont->qos_idx);
}
#endif
#endif

/** The per T-CONT net ops. */
static const struct net_device_ops ltq_pon_net_tcont_ops = {
	.ndo_open		= ltq_pon_net_tcont_open,
	.ndo_start_xmit		= ltq_pon_net_mdev_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_iflink		= ltq_pon_net_tcont_get_iflink,
#if IS_ENABLED(CONFIG_QOS_TC)
	.ndo_setup_tc		= ltq_pon_net_tcont_setup_tc,
#endif
};

/** The NetLink messages for the PON device configuration. */
enum {
	IFLA_PON_UNSPEC,
	IFLA_PON_QUEUE_LOOKUP_MODE,
	IFLA_PON_MAX
};

static const struct nla_policy pon_net_pon_cfg_nl_policy[IFLA_PON_MAX] = {
	[IFLA_PON_QUEUE_LOOKUP_MODE] = { .type = NLA_U8 },
};

/** Returns the maximum size of a PON device configuration NetLink message. */
static size_t pon_net_pon_cfg_nl_getsize(const struct net_device *pon_ndev)
{
	return nla_total_size(sizeof(u8));	/* IFLA_PON_QUEUE_LOOKUP_MODE */
}

static int pon_net_pon_cfg_validate(struct ltq_pon_net_hw *pon,
				    struct nlattr *data[])
{
	u8 queue_lookup_mode = QUEUE_LOOKUP_MODE_SUBIF_ID;

	if (data[IFLA_PON_QUEUE_LOOKUP_MODE])
		queue_lookup_mode
			= nla_get_u8(data[IFLA_PON_QUEUE_LOOKUP_MODE]);

	if (queue_lookup_mode != QUEUE_LOOKUP_MODE_SUBIF_ID &&
	    queue_lookup_mode != QUEUE_LOOKUP_MODE_SUBIF_ID_TC_4BIT &&
	    queue_lookup_mode != QUEUE_LOOKUP_MODE_SUBIF_ID_TC_3BIT) {
		netdev_err(pon->ndev, "invalid queue lookup mode: %i\n",
			   queue_lookup_mode);
		return -EINVAL;
	}

	pon->queue_lookup_mode = queue_lookup_mode;

	return 0;
}

/** Fills the NetLink message with the configuration of PON device. */
static int pon_net_pon_cfg_fill_info(struct sk_buff *skb,
				     const struct net_device *pon_ndev)
{
	struct ltq_pon_net_hw *pon = netdev_priv(pon_ndev);

	if (nla_put_u8(skb, IFLA_PON_QUEUE_LOOKUP_MODE, pon->queue_lookup_mode))
		return -ENOBUFS;

	return 0;
}

#if (KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE)
static int pon_net_pon_cfg_changelink(struct net_device *pon_ndev,
				      struct nlattr *tb[],
				      struct nlattr *data[])
#else
static int pon_net_pon_cfg_changelink(struct net_device *pon_ndev,
				      struct nlattr *tb[],
				      struct nlattr *data[],
				      struct netlink_ext_ack *extack)
#endif
{
	struct ltq_pon_net_hw *pon = netdev_priv(pon_ndev);
	int err;
	struct dp_q_map q_map = {
		.dp_port = pon->port_id,
	};

	if (!data) {
		netdev_err(pon_ndev, "Invalid data\n");
		return -EINVAL;
	}

	err = pon_net_pon_cfg_validate(pon, data);
	if (err)
		return err;

	err = dp_lookup_mode_cfg(pon->dev->id, pon->queue_lookup_mode,
				 &q_map, DP_CQM_LU_MODE_SET);
	if (err != DP_SUCCESS)
		return -EIO;

	return err;
}

/** Rtnl ops to make changes in PON devices. */
static struct rtnl_link_ops pon_net_pon_cfg_rtnl __read_mostly = {
	.kind		= "pon",
	.priv_size	= sizeof(struct ltq_pon_net_hw),

	.get_size	= pon_net_pon_cfg_nl_getsize,
	.policy		= pon_net_pon_cfg_nl_policy,
	.fill_info	= pon_net_pon_cfg_fill_info,

	.changelink	= pon_net_pon_cfg_changelink,
	.maxtype	= IFLA_PON_MAX,
};

/** Currently nothing to do when opening the IEEE 802.1p mapper. */
static int ltq_pon_net_pmapper_open(struct net_device *pmapper_ndev)
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);
	struct ltq_pon_net_gem *gem;
	int i, err;

	if (!pmapper->hw) {
		netdev_err(pmapper_ndev, "Invalid/missing pmapper->hw!\n");
		return -EIO;
	}

	/* Enable packet receiving in datapath */
	for (i = 0; i < ARRAY_SIZE(pmapper->gem); i++) {
		gem = pmapper->gem[i];
		if (!gem)
			continue;

		err = dp_rx_enable(gem->ndev, pmapper_ndev->name,
				   DP_RX_ENABLE);
		if (err != DP_SUCCESS) {
			netdev_err(gem->ndev, "dp_rx_enable failed\n");
			return -EIO;
		}
	}

	return 0;
}

static int ltq_pon_net_pmapper_stop(struct net_device *pmapper_ndev)
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);
	struct ltq_pon_net_gem *gem;
	int i, err;

	if (!pmapper->hw) {
		netdev_err(pmapper_ndev, "Invalid/missing pmapper->hw!\n");
		return -EIO;
	}

	/* Disable packet receiving in datapath */
	for (i = 0; i < ARRAY_SIZE(pmapper->gem); i++) {
		gem = pmapper->gem[i];
		if (!gem)
			continue;

		err = dp_rx_enable(gem->ndev, pmapper_ndev->name,
				   DP_RX_DISABLE);
		if (err != DP_SUCCESS) {
			netdev_err(gem->ndev, "dp_rx disable failed\n");
			return -EIO;
		}
	}

	return 0;
}

static void ltq_pon_net_pmapper_uninit(struct net_device *pmapper_ndev)
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);
	struct ltq_pon_net_hw *hw = pmapper->hw;
	int i;

	/* As a workaround do the deregistration from the datapath library
	 * here and not in the dellink callback, because uninit is called
	 * after all the tc callbacks to clean up the queue were done.
	 */
	for (i = 0; i < DP_PMAP_PCP_NUM; i++) {
		if (pmapper->gem[i])
			ltq_pon_net_pmapper_gem_remove(pmapper, i);
	}

	/* Remove the IEEE 802.1p mapper from the PON IP hardware interface. */
	list_del(&pmapper->node);

	dev_put(hw->ndev);
}

/*
 * Returns the sub interface based on the VLAN priority tag from the
 * given skb.
 */
static u32 ltq_pmapper_subif_vlan(struct ltq_pon_net_pmapper *pmapper,
				  struct sk_buff *skb)
{
	u16 vlan_tci;
	u32 pcp;
	int err;

	if (!skb_vlan_tagged(skb))
		return pmapper->subif_def;

	err = vlan_get_tag(skb, &vlan_tci);
	if (err)
		return pmapper->subif_def;

	pcp = (vlan_tci & VLAN_PRIO_MASK) >> VLAN_PRIO_SHIFT;
	if (pcp >= ARRAY_SIZE(pmapper->subif))
		return pmapper->subif_def;

	return pmapper->subif[pcp];
}

/* Returns the sub interface based on the given dscp value. */
static u32 ltq_pmapper_subif_dscp_val(struct ltq_pon_net_pmapper *pmapper,
				      u8 dscp)
{
	u8 pcp;

	if (dscp >= ARRAY_SIZE(pmapper->dscp_map))
		return pmapper->subif_def;

	pcp = pmapper->dscp_map[dscp];
	if (pcp == PON_PMAPPER_DSCP_DEFAULT)
		return pmapper->subif_def;

	if (pcp >= ARRAY_SIZE(pmapper->subif))
		return pmapper->subif_def;

	return pmapper->subif[pcp];
}

/* Returns the sub interface based on the dscp value from the given skb. */
static u32 ltq_pmapper_subif_dscp(struct ltq_pon_net_pmapper *pmapper,
				  struct sk_buff *skb)
{
	u8 dscp;

	switch (skb->protocol) {
	case htons(ETH_P_IP):
		dscp = (ipv4_get_dsfield(ip_hdr(skb)) & 0xfc) >> 2;
		return ltq_pmapper_subif_dscp_val(pmapper, dscp);
	case htons(ETH_P_IPV6):
		dscp = (ipv6_get_dsfield(ipv6_hdr(skb)) & 0xfc) >> 2;
		return ltq_pmapper_subif_dscp_val(pmapper, dscp);
	default:
		return pmapper->subif_def;
	}
}

/** Transmit one data packet with the dp lib. */
static netdev_tx_t
ltq_pon_net_pmapper_start_xmit(struct sk_buff *skb,
			       struct net_device *pmapper_ndev)
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);
	int32_t ret;
	uint32_t dp_flags = 0;
	unsigned int len = skb->len;
	dp_subif_t dp_subif;

	if (skb_put_padto(skb, ETH_ZLEN)) {
		pmapper->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	dp_subif.port_id = pmapper->hw->port_id;

	switch (pmapper->mode) {
	case DP_PMAP_PCP:
		dp_subif.subif = ltq_pmapper_subif_vlan(pmapper, skb);
		break;
	case DP_PMAP_DSCP:
		if (skb_vlan_tagged(skb))
			dp_subif.subif = ltq_pmapper_subif_vlan(pmapper, skb);
		else
			dp_subif.subif = ltq_pmapper_subif_dscp(pmapper, skb);
		break;
	case DP_PMAP_DSCP_ONLY:
		dp_subif.subif = ltq_pmapper_subif_dscp(pmapper, skb);
		break;
	default:
		pmapper->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	if (dp_subif.subif == DP_PMAPPER_DISCARD_CTP) {
		pmapper->stats.tx_dropped++;
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	ltq_pon_net_set_skd_dw(skb, dp_subif.port_id, dp_subif.subif);

	if (pmapper_ndev->features & NETIF_F_HW_CSUM)
		dp_flags |= DP_TX_CAL_CHKSUM;

	ret = dp_xmit(pmapper_ndev, &dp_subif, skb, skb->len, dp_flags);
	if (!ret) {
		pmapper->stats.tx_packets++;
		pmapper->stats.tx_bytes += len;
	} else {
		pmapper->stats.tx_dropped++;
	}

	return NETDEV_TX_OK;
}

#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
static struct rtnl_link_stats64 *
ltq_pon_net_pmapper_get_stats(struct net_device *pmapper_ndev,
			      struct rtnl_link_stats64 *storage)
{
#ifndef CONFIG_DPM_DATAPATH_MIB
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);

	*storage = pmapper->stats;
#else
	dp_get_netif_stats(pmapper_ndev, NULL, storage, 0);
#endif
	return storage;
}
#else
static void
ltq_pon_net_pmapper_get_stats(struct net_device *pmapper_ndev,
			      struct rtnl_link_stats64 *storage)
{
#ifndef CONFIG_DPM_DATAPATH_MIB
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);

	*storage = pmapper->stats;
#else
	dp_get_netif_stats(pmapper_ndev, NULL, storage, 0);
#endif
}
#endif

/** Receives one package, this is called by the datapath library. */
static int32_t ltq_pon_net_dp_rx(struct net_device *ndev,
				 struct net_device *txif,
				 struct sk_buff *skb, int32_t len)
{
	struct rtnl_link_stats64 *stats;
	struct ltq_pon_net_hw *hw = NULL;

	if (WARN_ON(!skb))
		return -1;

	if (WARN_ON(!ndev)) {
		dev_kfree_skb_any(skb);
		return -1;
	}

	/* check if this is a pon master device */
	if (ndev->netdev_ops->ndo_open == &ltq_pon_net_gem_open) {
		struct ltq_pon_net_gem *gem = netdev_priv(ndev);

		stats = &gem->stats;
		hw = gem->hw;
	} else if (ndev->netdev_ops->ndo_open == &ltq_pon_net_pmapper_open) {
		struct ltq_pon_net_pmapper *pmapper = netdev_priv(ndev);

		stats = &pmapper->stats;
		hw = pmapper->hw;
	} else {
		netdev_err(ndev, "received data on unknown device\n");
		dev_kfree_skb_any(skb);
		return -1;
	}

	if (WARN_ON(!hw)) {
		dev_kfree_skb_any(skb);
		return -1;
	}

	if (hw->soc_data->pmac_remove) {
		/* Remove PMAC to DMA header */
		len -= DP_MAX_PMAC_LEN;
		skb_pull(skb, DP_MAX_PMAC_LEN);
	}

	skb->dev = ndev;
	skb->protocol = eth_type_trans(skb, ndev);

	if (ndev->features & NETIF_F_RXCSUM)
		skb->ip_summed = CHECKSUM_UNNECESSARY;

	netif_rx(skb);
	stats->rx_packets++;
	stats->rx_bytes += len;

	return 0;
}

/* desc_1->field.session_id  is not available in the LGM DMA descriptor */
#if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_SOC_LGM)
/**
 * Returns the Linux interface index for the GEM port of the given skb.
 * This only works for skbs received on a pmapper.
 *
 * Returns:
 *    Positive number: Linux Interface index of GEM port
 *    -ENXIO: Not part of a pmapper or can not find GEM port in pmapper
 *    -ENOENT: no private data
 */
int ltq_pon_net_gem_get(struct sk_buff *skb)
{
	struct net_device *pmapper_ndev;
	struct ltq_pon_net_pmapper *pmapper;
	struct ltq_pon_net_gem *gem;
	u32 subif;
	int i;
	struct dma_rx_desc_1 *desc_1 = (struct dma_rx_desc_1 *)&skb->DW1;

	pmapper_ndev = skb->dev;
	if (pmapper_ndev == NULL) {
		pr_err("Missing pmapper_ndev!\n");
		return -ENODEV;
	}
	/* Check if this is really a pmapper from this driver */
	if (pmapper_ndev->netdev_ops->ndo_open != ltq_pon_net_pmapper_open) {
		netdev_err(pmapper_ndev, "Invalid ndo_open! Not ltq_pon_net_pmapper_open!\n");
		return -ENXIO;
	}

	pmapper = netdev_priv(pmapper_ndev);
	if (!pmapper) {
		netdev_err(pmapper_ndev, "Invalid/missing pmapper!\n");
		return -ENOENT;
	}

	/* The lowest 8 bits contain the sub interface ID */
	subif = desc_1->field.session_id & 0xff;
	BUILD_BUG_ON(ARRAY_SIZE(pmapper->gem) != ARRAY_SIZE(pmapper->subif));
	for (i = 0; i < ARRAY_SIZE(pmapper->gem); i++) {
		if (pmapper->subif[i] != subif)
			continue;

		gem = pmapper->gem[i];
		if (!gem || !gem->ndev)
			continue;
		return gem->ndev->ifindex;
	}

	return -ENODEV;
}
EXPORT_SYMBOL(ltq_pon_net_gem_get);
#endif

/** Returns the ifindex of the PON IP HW interface for the given
 * IEEE 802.1p mapper.
 */
static int ltq_pon_net_pmapper_get_iflink(const struct net_device *pmapper_ndev)
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);
	struct net_device *hw_ndev = pmapper->hw->ndev;

	return hw_ndev->ifindex;
}

/** The IEEE 802.1p mapper net ops.
 * These contain the functions to really send packets.
 */
static const struct net_device_ops ltq_pon_net_pmapper_ops = {
	.ndo_open		= ltq_pon_net_pmapper_open,
	.ndo_stop		= ltq_pon_net_pmapper_stop,
	.ndo_uninit		= ltq_pon_net_pmapper_uninit,
	.ndo_start_xmit		= ltq_pon_net_pmapper_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	 /* TODO: switch config needs changes to activate jumbo frames */
	.ndo_get_stats64	= ltq_pon_net_pmapper_get_stats,
	.ndo_get_iflink		= ltq_pon_net_pmapper_get_iflink,
#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE) /* removed in 4.14 */
	.ndo_bridge_setlink	= switchdev_port_bridge_setlink,
	.ndo_bridge_getlink	= switchdev_port_bridge_getlink,
#endif
#if IS_ENABLED(CONFIG_QOS_TC)
	.ndo_setup_tc		= ltq_pon_net_gem_setup_tc,
#endif
};

/** DP callbacks, minimal needed to receive packets */
dp_cb_t ltq_pon_net_dp_cb = {
	.rx_fn		= ltq_pon_net_dp_rx,
};

/** DP registration and allocation for GPON modes
 */
static int ltq_pon_dp_gpon_register(struct ltq_pon_net_hw *hw)
{
	struct dp_port_data port_data = {0,};
	struct module *owner = hw->dev->driver->owner;
	struct dp_dev_data dp_data = {0};
	u32 flags = DP_F_GPON;
	int32_t err;

	port_data.flag_ops = DP_F_DATA_RESV_Q | DP_F_DATA_RESV_SCH;
	if (!hw->dp_flags.with_tx_fcs)
		port_data.flag_ops |= DP_F_DATA_FCS_DISABLE;
	if (hw->dp_flags.with_rx_fcs)
		port_data.flag_ops |= DP_F_REM_FCS;

	/* TODO: read these values from device tree */
	/* Reserve 64 schedulers and 64 queues in the central datapath
	 * system for PON.
	 */
	port_data.num_resv_q = DEFAULT_NUM_OF_QUEUES;
	port_data.num_resv_sched = DEFAULT_NUM_OF_SCHEDULERS;

	hw->port_id = dp_alloc_port_ext(hw->dev->id, owner, NULL, hw->dev_id,
					hw->dev_id, NULL, &port_data, flags);

	dev_dbg(hw->dev, "dp_alloc_port_ext(inst: %i, dev_port: %i, num_resv_q: %i, num_resv_sched: %i): %i\n",
		hw->dev->id, hw->dev_id, port_data.num_resv_q,
		port_data.num_resv_sched, hw->port_id);

	if (hw->port_id == DP_FAILURE) {
		dev_err(hw->dev, "dp_alloc_port_ext failed\n");
		return -ENODEV;
	}

	dp_data.max_ctp = hw->max_ctps;
	dp_data.max_gpid = hw->max_gpid;

	err = dp_register_dev_ext(hw->dev->id, owner, hw->port_id,
				  &ltq_pon_net_dp_cb, &dp_data, 0);
	dev_dbg(hw->dev, "dp_register_dev_ext(inst: %i, port_id: %i): %i\n",
		hw->dev->id, hw->port_id, err);

	if (err != DP_SUCCESS) {
		dev_err(hw->dev, "dp_register_dev_ext failed");
		dp_alloc_port_ext(hw->dev->id, owner, NULL, hw->dev->id,
				  hw->port_id, NULL, NULL, DP_F_DEREGISTER);
		return -ENODEV;
	}

	return 0;
}

/*
 * Registers the port in DP manager based on given mode.
 *
 * This can be called multiple times and will reject unsupported mode changes.
 */
int ltq_pon_net_register_dp_port(struct ltq_pon_net_hw *hw, enum pon_mode mode)
{
	int err;

	switch (mode) {
	case PON_MODE_UNKNOWN:
		/* we ignore the unknown mode */
		break;
	case PON_MODE_984_GPON:
	case PON_MODE_987_XGPON:
	case PON_MODE_9807_XGSPON:
	case PON_MODE_989_NGPON2_2G5:
	case PON_MODE_989_NGPON2_10G:
		/* Skip when we are already in GPON mode */
		if (hw->dp_mode_registered == DP_F_GPON)
			return 0;
		if (hw->dp_mode_registered) {
			dev_err(hw->dev, "Can not switch from %i to GPON\n",
				hw->dp_mode_registered);
			return -ENOTSUPP;
		}
		err = ltq_pon_dp_gpon_register(hw);
		if (err)
			return err;
		hw->dp_mode_registered = DP_F_GPON;
		break;
	case PON_MODE_AON:
		dev_err(hw->dev, "Can not switch from %i to AON\n",
			hw->dp_mode_registered);
		return -ENOTSUPP;
	default:
		dev_err(hw->dev, "unsupported mode\n");
		return -ENOTSUPP;
	}

	return 0;
}

/** Probes the PON IP hardware and registers the PON IP hardware interface. */
static int ltq_pon_net_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *of_node = dev->of_node;
	struct ltq_pon_net_hw *hw;
	struct net_device *hw_ndev;
	const struct ltq_pon_net_soc_data *soc_data = NULL;
	enum pon_mode mode;
	int ret;

	soc_data = of_device_get_match_data(dev);
	if (!soc_data) {
		dev_err(dev, "No data found!\n");
		return -EINVAL;
	}

	hw_ndev = alloc_netdev_mq(sizeof(*hw), "pon%d", NET_NAME_UNKNOWN,
				  ether_setup, PON_CPU_QUEUES);
	if (!hw_ndev) {
		dev_err(dev, "Cannot allocate netdev memory!\n");
		return -ENOMEM;
	}

	hw = netdev_priv(hw_ndev);
	hw->dev = dev;
	hw->ndev = hw_ndev;
	hw->soc_data = soc_data;
	hw->queue_lookup_mode = QUEUE_LOOKUP_MODE_SUBIF_ID_TC_4BIT;
	INIT_LIST_HEAD(&hw->pmapper_list);
	INIT_LIST_HEAD(&hw->tcont_list);
	INIT_LIST_HEAD(&hw->gem_list);
	INIT_LIST_HEAD(&hw->iphost_list);
	spin_lock_init(&hw->iphost_lock);

	/* Do not use platform_get_resource() because reg is not a real
	 * address on the bus, but an identifier in some hardware.
	 */
	ret = of_property_read_u32(of_node, "reg", &hw->dev_id);
	if (ret) {
		/* ignore missing "reg", assume default instance 0 */
		dev_dbg(hw->dev, "can not get <reg> property: %i\n", ret);
		hw->dev_id = 0;
	}
	/* The PON-IP is able to strip the FCS, this reads the option from
	 * device tree to configure it.
	 * We store it also in mbox driver and provide it to userspace to
	 * configure the PON-IP via firmware-message in the same way.
	 * A mismatch will result in a not working interface.
	 */
	hw->dp_flags.with_rx_fcs =
		of_property_read_bool(of_node, "intel,with-rx-fcs");
	hw->dp_flags.with_tx_fcs =
		of_property_read_bool(of_node, "intel,with-tx-fcs");
	dev_dbg(hw->dev, "rx_fcs: %s tx_fcs: %s\n",
		hw->dp_flags.with_rx_fcs ? "yes" : "no",
		hw->dp_flags.with_tx_fcs ? "yes" : "no");
	pon_mbox_save_pon_dp_flags(&hw->dp_flags);

	ret = of_property_read_u32(of_node, "intel,max_ctps", &hw->max_ctps);
	if (ret) {
		dev_dbg(hw->dev, "can not get <max_ctps> property: %i\n", ret);
		hw->max_ctps = 0;
	}

	ret = of_property_read_u32(of_node, "intel,max_gpid", &hw->max_gpid);
	if (ret) {
		dev_dbg(hw->dev, "can not get <max_gpid> property: %i\n", ret);
		hw->max_gpid = 0;
	}

	hw->dp_mode_registered = 0;

	hw_ndev->netdev_ops = &ltq_pon_net_mdev_ops;
#if IS_ENABLED(CONFIG_QOS_TC)
	hw_ndev->features |= NETIF_F_HW_TC;
#endif
	hw_ndev->ethtool_ops = &mdev_ethtool_ops;

	hw_ndev->rtnl_link_ops = &pon_net_pon_cfg_rtnl;
	SET_NETDEV_DEV(hw_ndev, hw->dev);

	ret = register_netdev(hw_ndev);
	if (ret) {
		dev_err(dev, "Cannot register net device\n");
		free_netdev(hw_ndev);
		return ret;
	}

	platform_set_drvdata(pdev, hw);

	/*
	 * Get the current PON mode, at bootup this is probably
	 * PON_MODE_UNKNOWN, but when this driver gets loaded later or it
	 * gets reloaded after some time the pon mode could already be set.
	 */
	mode = pon_mbox_get_pon_mode();
	ret = ltq_pon_net_register_dp_port(hw, mode);
	if (ret)
		return ret;

	ret = ltq_pon_iphost_port_register(hw);
	if (ret) {
		dev_err(dev, "ltq_pon_iphost_port_register failed\n");
		unregister_netdev(hw->ndev);
		free_netdev(hw_ndev);
		return ret;
	}

	hw->num_ani_archived_stats = 0;
	hw->ani_archived_stats = NULL;

	/* Store PON netdevice private data structure for event usage */
	g_hw = hw;

	return 0;
}

/** Unregisters the PON IP hardware. */
static int ltq_pon_net_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct module *owner = dev->driver->owner;
	struct ltq_pon_net_hw *hw = platform_get_drvdata(pdev);

	if (!hw)
		return -ENODEV;

	kfree(hw->ani_archived_stats);

	ltq_pon_iphost_port_deregister(hw);

	unregister_netdev(hw->ndev);
	dp_register_dev_ext(dev->id, owner, hw->port_id, &ltq_pon_net_dp_cb,
			    NULL, DP_F_DEREGISTER);
	dev_dbg(dev, "dp_register_dev_ext(inst: %i, port_id: %i, DP_F_DEREGISTER)\n",
		dev->id, hw->port_id);
	dp_alloc_port_ext(dev->id, owner, NULL, hw->port_id, hw->port_id, NULL,
			  NULL, DP_F_DEREGISTER);
	dev_dbg(dev, "dp_alloc_port_ext(inst: %i, dev_port: %i, DP_F_DEREGISTER)\n",
		dev->id, hw->port_id);
	free_netdev(hw->ndev);

	/* Store PON netdevice private data structure for event usage as NULL */
	g_hw = NULL;

	return 0;
}

/** The NetLink messages for the GEM port configuration. */
enum {
	IFLA_PON_GEM_UNSPEC,
	IFLA_PON_GEM_IDX,
	IFLA_PON_GEM_ID,
	IFLA_PON_GEM_TCONT,
	IFLA_PON_GEM_TRAFFIC_TYPE,
	IFLA_PON_GEM_MAX_SIZE,
	IFLA_PON_GEM_DIR,
	IFLA_PON_GEM_ENC,
	IFLA_PON_GEM_MC,
	/* For debbuging purposes only */
	IFLA_PON_GEM_BP,
	IFLA_PON_GEM_CTP,
	IFLA_PON_GEM_MAX
};

static const struct nla_policy
ltq_pon_net_gem_nl_policy[IFLA_PON_GEM_MAX] = {
	[IFLA_PON_GEM_IDX]		= { .type = NLA_U16 },
	[IFLA_PON_GEM_ID]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_TCONT]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_MAX_SIZE]		= { .type = NLA_U16 },
	[IFLA_PON_GEM_TRAFFIC_TYPE]	= { .type = NLA_U8 },
	[IFLA_PON_GEM_DIR]		= { .type = NLA_U8 },
	[IFLA_PON_GEM_ENC]		= { .type = NLA_U8 },
	[IFLA_PON_GEM_MC]		= { .type = NLA_U8 },
	[IFLA_PON_GEM_BP]		= { .type = NLA_U16 },
	[IFLA_PON_GEM_CTP]		= { .type = NLA_U32 },
};

/** Returns the maximum size of a GEM port configuration NetLink message. */
static size_t ltq_pon_net_gem_nl_getsize(const struct net_device *gem_ndev)
{
	return nla_total_size(sizeof(u16)) +	/* IFLA_PON_GEM_IDX */
	       nla_total_size(sizeof(u32)) +	/* IFLA_PON_GEM_ID */
	       nla_total_size(sizeof(u32)) +	/* IFLA_PON_GEM_TCONT */
	       nla_total_size(sizeof(u16)) +	/* IFLA_PON_GEM_MAX_SIZE */
	       nla_total_size(sizeof(u8)) +	/* IFLA_PON_GEM_TRAFFIC_TYPE */
	       nla_total_size(sizeof(u8)) +	/* IFLA_PON_GEM_DIR */
	       nla_total_size(sizeof(u8)) +	/* IFLA_PON_GEM_ENC */
	       nla_total_size(sizeof(u8)) +	/* IFLA_PON_GEM_MC */
	       nla_total_size(sizeof(u16)) +	/* IFLA_PON_GEM_BP */
	       nla_total_size(sizeof(u32));	/* IFLA_PON_GEM_CTP */
}

#if IS_ENABLED(CONFIG_QOS_TC)
static int ltq_pon_get_qid(int tcont_idx, int port_id, int *q_id)
{
	struct dp_dequeue_res deq = {0};
	static struct dp_queue_res q_res[8];
	int ret;

	if (tcont_idx <= 0)
		return -1;

	deq.dp_port = port_id;
	deq.cqm_deq_idx = tcont_idx;
	deq.q_res = q_res;
	deq.q_res_size = ARRAY_SIZE(q_res);
	ret = dp_deq_port_res_get(&deq, 0);
	if (ret < 0) {
		pr_err("%s: res get failed\n", __func__);
		return -1;
	}

	*q_id = q_res[0].q_id;

	return 0;
}
#endif

static uint8_t ltq_pon_net_dp_domain_get(struct ltq_pon_net_gem *gem)
{
	if (gem->multicast) {
		netdev_dbg(gem->ndev, "GEM Multicast domain: %d\n",
			   DP_BR_DM_MC);
		return DP_BR_DM_MC;
	}

	if (gem->gem_dir == PONFW_GEM_PORT_ID_DIR_DS) {
		netdev_dbg(gem->ndev, "GEM Broadcast domain: %d\n",
			   DP_BR_DM_BC1);
		return DP_BR_DM_BC1;
	}

	netdev_dbg(gem->ndev, "GEM Unicast domain: %d\n",
		   DP_BR_DM_UC);
	return DP_BR_DM_UCA;
}

static void ltq_pon_net_dp_domain_reg(struct ltq_pon_net_gem *gem,
				      dp_subif_t *dp_subif,
				      struct dp_subif_data *data)
{
	data->flag_ops |= DP_SUBIF_BR_DOMAIN;
	data->domain_id = ltq_pon_net_dp_domain_get(gem);
	data->domain_members = DP_BR_DM_MEMBER(data->domain_id);
	/* Broadcast domain traffic can also go to Unicast and other broadcast
	 * domains netdevices.
	 */
	if (data->domain_id == DP_BR_DM_BC1) {
		data->domain_members |= DP_BR_DM_MEMBER(DP_BR_DM_UC);
		data->domain_members |= DP_BR_DM_MEMBER(DP_BR_DM_BC2);
	}
	if (data->domain_id == DP_BR_DM_UCA) {
		data->domain_members |= DP_BR_DM_MEMBER(DP_BR_DM_UC);
		data->domain_members |= DP_BR_DM_MEMBER(DP_BR_DM_IPH);
	}
}

static void ltq_pon_net_dp_domain_upd(struct ltq_pon_net_gem *gem,
				      struct dp_subif_upd_info *dp_subif)
{
	dp_subif->flags |= DP_SUBIF_BR_DOMAIN;
	dp_subif->new_domain_flag = true;
	dp_subif->new_domain_id = ltq_pon_net_dp_domain_get(gem);
	dp_subif->domain_members = DP_BR_DM_MEMBER(dp_subif->new_domain_id);
	/* Broadcast domain traffic can also go to Unicast and other broadcast
	 * domains netdevices.
	 */
	if (dp_subif->new_domain_id == DP_BR_DM_BC1) {
		dp_subif->domain_members |= DP_BR_DM_MEMBER(DP_BR_DM_UC);
		dp_subif->domain_members |= DP_BR_DM_MEMBER(DP_BR_DM_BC2);
	}
	if (dp_subif->new_domain_id == DP_BR_DM_UCA) {
		dp_subif->domain_members |= DP_BR_DM_MEMBER(DP_BR_DM_UC);
		dp_subif->domain_members |= DP_BR_DM_MEMBER(DP_BR_DM_IPH);
	}
}

/** Connect the GEM port interface with the given network device to the data
 * path library. This is just a wrapper around the dp_register_subif_ext()
 * function.
 */
static int ltq_pon_net_gem_connect(struct ltq_pon_net_gem *gem,
				   struct net_device *ndev)
{
	struct net_device *gem_ndev = gem->ndev;
	struct device *dev = gem->hw->dev;
	struct module *owner = dev->driver->owner;
	struct ltq_pon_net_tcont *tcont = gem->tcont;
	struct dp_subif_data data = {0,};
	dp_subif_t dp_subif = {0,};
	int32_t ret;

	dp_subif.subif = gem->gem_idx;
	dp_subif.port_id = gem->hw->port_id;
	/* Register the netdevs with learning disable as it conflicts with
	 * the VLAN flow forwarding. If enabled there is a time slot during
	 * which addresses are learned and traffic forwarding is delayed until
	 * the addresses aged out.
	 */
	data.mac_learn_disable = DP_MAC_LEARNING_DIS;

	if (tcont) {
#if IS_ENABLED(CONFIG_QOS_TC)
		int qid = 0;

		ret = ltq_pon_get_qid(tcont->qos_idx, dp_subif.port_id, &qid);
		if (!ret && qid > 0) {
			data.flag_ops |= DP_SUBIF_SPECIFIC_Q;
			data.q_id = qid;
		}
#endif
		data.deq_port_idx = tcont->qos_idx;
	} else {
		ndev->priv_flags |= IFF_NO_QUEUE;
	}

	if (gem_ndev != ndev)
		data.ctp_dev = gem_ndev;

	/* Disable the RX traffic if the interface is not up. We check if the
	 * used network device (ndev) is up, but we deactivate this on the
	 * network device for the ctp_dev to do this on the specific sub
	 * interface.
	 */
	data.flag_ops |= DP_SUBIF_RX_FLAG;
	data.rx_en_flag = netif_running(ndev);

	if (!(gem->common.ethtool_flags & ETHTOOL_FLAG_BP_CPU_ENABLE))
		data.flag_ops |= DP_SUBIF_BP_CPU_DISABLE;

#if (KERNEL_VERSION(4, 9, 51) <= LINUX_VERSION_CODE)
	/* When we remove a sub interface, all the bridge associations are also
	 * removed by the datapath library, when we add it again the net device
	 * is still in the bridge, but the datapath library does not know about
	 * it and does not configure hardware forwarding. Removing the net
	 * device from the bridge and adding them again work arounds the
	 * problem.
	 */
	if (netdev_has_any_upper_dev(ndev))
		netdev_warn(ndev, "Do not change interface inside of bridge, this will break HW forwarding!");
#endif /* kernel <= 4.9.51 */

	ltq_pon_net_dp_domain_reg(gem, &dp_subif, &data);

	dev_dbg(dev, "dp_register_subif_ext(inst: %i, ndev: %s, subif: %i, port_id: %i, deq_port_idx: %i, data.ctp_dev: %s)\n",
		dev->id, ndev->name, dp_subif.subif, dp_subif.port_id,
		data.deq_port_idx, data.ctp_dev ? data.ctp_dev->name : NULL);
	ret = dp_register_subif_ext(dev->id, owner, ndev, ndev->name,
				    &dp_subif, &data, 0);
	if (ret != DP_SUCCESS) {
		netdev_err(gem_ndev, "dp_register_subif_ext failed: %d\n", ret);
		return -EIO;
	}
	netdev_dbg(gem_ndev, "dp_register_subif_ext success\n");
	if (tcont)
		tcont->queue_id = data.q_id;

	gem->dp_ndev = ndev;
	gem->dp_connected = true;

	return 0;
}

/** Disconnect an interface from the datapath library again. */
static int ltq_pon_net_gem_disconnect(struct ltq_pon_net_gem *gem,
				      struct ltq_pon_net_tcont *tcont)
{
	struct net_device *gem_ndev = gem->ndev;
	struct device *dev = gem->hw->dev;
	struct module *owner = dev->driver->owner;
	dp_subif_t dp_subif = {0,};
	struct dp_subif_data data = {0,};
	int32_t ret;

	if (!gem->dp_connected)
		return 0;

	dev_close(gem_ndev);

	dp_subif.subif = gem->gem_idx;
	dp_subif.port_id = gem->hw->port_id;
	data.mac_learn_disable = DP_MAC_LEARNING_DIS;

	if (tcont) {
#if IS_ENABLED(CONFIG_QOS_TC)
		int qid = 0;

		ret = ltq_pon_get_qid(tcont->qos_idx, dp_subif.port_id, &qid);
		if (!ret && qid > 0) {
			data.flag_ops |= DP_SUBIF_SPECIFIC_Q;
			data.q_id = qid;
		}
#endif
		data.deq_port_idx = tcont->qos_idx;
	}

	if (gem_ndev != gem->dp_ndev)
		data.ctp_dev = gem_ndev;

	ret = dp_register_subif_ext(dev->id, owner, gem->dp_ndev,
				    gem->dp_ndev->name, &dp_subif, &data,
				    DP_F_DEREGISTER);
	dev_dbg(dev, "dp_register_subif_ext(inst: %i, ndev: %s, subif: %i, port_id: %i, deq_port_idx: %i, data.ctp_dev: %s, DP_F_DEREGISTER): %i\n",
		dev->id, gem->dp_ndev->name, dp_subif.subif, dp_subif.port_id,
		data.deq_port_idx, data.ctp_dev ? data.ctp_dev->name : NULL,
		ret);
	if (ret != DP_SUCCESS) {
		netdev_err(gem_ndev,
			   "dp_register_subif_ext unregister failed\n");
		return -EIO;
	}

	if (tcont && list_empty(&tcont->gem_list))
		tcont->queue_id = PON_ETH_QUEUE_UNASSIGNED;

	gem->dp_ndev = NULL;
	gem->dp_connected = false;

	return 0;
}

static int ltq_pon_net_tcont_omci_fw_get(struct ltq_pon_net_tcont *tcont,
					 uint32_t *alloc_id,
					 uint32_t *alloc_link_ref)
{
	struct ponfw_alloc_idx fw_alloc_input = {0};
	struct ponfw_alloc_idx fw_alloc_output = {0};
	int err;

	fw_alloc_input.alloc_idx = tcont->alloc_idx;

	err = pon_mbox_send(PONFW_ALLOC_IDX_CMD_ID, PONFW_READ,
			    &fw_alloc_input, PONFW_ALLOC_IDX_LENR,
			    &fw_alloc_output,
			    sizeof(fw_alloc_output));
	if (err < 0) {
		netdev_err(tcont->ndev,
			   "failed to read Allocation Index (%u): %i\n",
			   tcont->alloc_idx, err);
		return err;
	}

	if (err != sizeof(fw_alloc_output)) {
		err = -EINVAL;
		netdev_err(tcont->ndev,
			   "failed to get answer for Allocation Index read (%u): %i\n",
			   tcont->alloc_idx, err);
		return err;
	}

	/* We should not store this information for local management,
	 * because it is done by firmware.
	 */
	*alloc_id = fw_alloc_output.alloc_id;
	*alloc_link_ref = fw_alloc_output.alloc_link_ref;

	return 0;
}

static u16 ltq_pon_net_gem_idx_preassign(struct ltq_pon_net_gem *gem)
{
	u16 i;

	if (gem->gem_idx)
		return 0;

	/* GEM Index 0 is used by OMCI channel, so skip it */
	for (i = 1; i < ARRAY_SIZE(g_gem_idx_used); i++) {
		if (g_gem_idx_used[i] == true)
			continue;

		g_gem_idx_used[i] = true;
		gem->gem_idx = i;
		netdev_dbg(gem->ndev,
			   "GEM Index (%u) pre-assigned for GEM id (%u)\n",
			   gem->gem_idx, gem->gem_id);
		return 0;
	}

	netdev_err(gem->ndev,
		   "failed to pre-assign GEM Index for GEM id (%u)\n",
		   gem->gem_id);

	return -ENODATA;
}

static int ltq_pon_net_gem_fw_apply(struct ltq_pon_net_gem *gem,
				    struct ltq_pon_net_tcont *tcont)
{
	struct ponfw_gem_port_id fw_gem_input = {0};
	uint32_t alloc_id, alloc_link_ref;
	int err;

	if (gem->gem_traffic_type == PONFW_GEM_PORT_ID_TT_OMCI)
		return 0;

	/* Warmstart: if system is not in operational state we can only
	 * pretend to configure GEM with pre-assigned index.
	 */
	if (!g_hw->in_o5) {
		err = ltq_pon_net_gem_idx_preassign(gem);
		return err;
	}

	fw_gem_input.gem_port_id = gem->gem_id;

	if (tcont && tcont->alloc_link_ref) {
		fw_gem_input.alloc_id = tcont->alloc_id;
		fw_gem_input.alloc_link_ref = tcont->alloc_link_ref;
		fw_gem_input.dir = gem->gem_dir;
	} else if (tcont && tcont->tcont_omci) {
		err = ltq_pon_net_tcont_omci_fw_get(tcont,
				&alloc_id,
				&alloc_link_ref);
		if (err)
			return err;
		fw_gem_input.alloc_id = alloc_id;
		fw_gem_input.alloc_link_ref = alloc_link_ref;
		fw_gem_input.dir = gem->gem_dir;
	} else {
		fw_gem_input.dir = PONFW_GEM_PORT_ID_DIR_DS;
	}
	fw_gem_input.max_gem_size = gem->gem_max_size;
	fw_gem_input.tt = gem->gem_traffic_type;
	fw_gem_input.enc = gem->gem_enc;

	/* Warmstart: If the GEM port was not removed while leaving the
	 * operational state, we need to configure it to the same
	 * GEM port index value as before.
	 */
	if (gem->gem_idx) {
		fw_gem_input.use_idx = PONFW_GEM_PORT_ID_USE_IDX_EN;
		fw_gem_input.gem_port_idx = gem->gem_idx;
	}

	err = pon_mbox_send(PONFW_GEM_PORT_ID_CMD_ID, PONFW_WRITE,
			    &fw_gem_input, PONFW_GEM_PORT_ID_LENW,
			    NULL, 0);
	if (err < 0) {
		/* When the T-Cont and the GEM port are configured over OMCI
		 * before they are configured over PLOAM or the SW did not get
		 * the information about this over the mailbox yet, this
		 * message will return an error. This is no problem because we
		 * will get the LINK event from the PON FW later and then
		 * link them again.
		 */
		if (tcont && tcont->alloc_link_ref) {
			netdev_dbg(gem->ndev,
				   "failed to map GEM id (%u) for Alloc id (%i): %i, ignore this as Alloc ID is probably not yet configured over PLOAM\n",
				   gem->gem_id, tcont ? tcont->alloc_id : -1,
				   err);
			return 0;
		}
		netdev_err(gem->ndev,
			   "failed to map GEM id (%u) for Alloc id (%i): %i\n",
			   gem->gem_id, tcont ? tcont->alloc_id : -1, err);
		return err;
	}

	if (tcont && tcont->alloc_link_ref) {
		gem->alloc_link_ref = tcont->alloc_link_ref;
		netdev_dbg(gem->ndev,
			   "mapped GEM id (%u) for Alloc id (%i) with Version number (0x%x)\n",
			   gem->gem_id, tcont->alloc_id, gem->alloc_link_ref);
	}

	return 0;
}

static int ltq_pon_net_pmapper_gem_count(struct ltq_pon_net_pmapper *pmapper)
{
	int i;
	int count = 0;

	for (i = 0; i < DP_PMAP_PCP_NUM; i++) {
		if (pmapper->gem[i])
			count++;
	}

	return count;
}

/** Updates the pmapper mapping data in the datapath library. This should be
 * called every time some gem port configuration in the pmapper is changed.
 */
static int ltq_pon_net_pmapper_set_mapping(struct ltq_pon_net_pmapper *pmapper)
{
	struct dp_pmapper pmap = {0,};
	int i;
	bool set_pmapper = false;
	int err = 0;

	pmap.mode = pmapper->mode;
	if (pmapper->gem[pmapper->pcpdef])
		pmap.def_ctp = pmapper->gem[pmapper->pcpdef]->gem_idx;
	else
		pmap.def_ctp = DP_PMAPPER_DISCARD_CTP;

	for (i = 0; i < DP_PMAP_PCP_NUM; i++) {
		if (pmapper->gem[i]) {
			if (pmap.pcp_map[i] != pmapper->gem[i]->gem_idx) {
				pmap.pcp_map[i] = pmapper->gem[i]->gem_idx;
				set_pmapper = true;
			}
		} else {
			if (pmap.pcp_map[i] != DP_PMAPPER_DISCARD_CTP) {
				pmap.pcp_map[i] = DP_PMAPPER_DISCARD_CTP;
				set_pmapper = true;
			}
		}
	}

	for (i = 0; i < DP_PMAP_DSCP_NUM; i++) {
		u8 pcp_idx = pmapper->dscp_map[i];
		struct ltq_pon_net_gem *gem;

		if (pcp_idx == PON_PMAPPER_DSCP_DEFAULT) {
			pmap.dscp_map[i] = pmap.def_ctp;
			continue;
		}
		if (pcp_idx >= DP_PMAP_PCP_NUM)
			continue;
		gem = pmapper->gem[pcp_idx];

		if (!gem && pmap.dscp_map[i] != DP_PMAPPER_DISCARD_CTP) {
			pmap.dscp_map[i] = DP_PMAPPER_DISCARD_CTP;
			set_pmapper = true;
		} else if (gem && pmap.dscp_map[i] != gem->gem_idx) {
			pmap.dscp_map[i] = gem->gem_idx;
			set_pmapper = true;
		}
	}

	/* Only call dp_set_pmapper() if we have at least one GEM port */
	if (set_pmapper && ltq_pon_net_pmapper_gem_count(pmapper) >= 1) {
		err = dp_set_pmapper(pmapper->ndev, &pmap, 0);
		netdev_dbg(pmapper->ndev,
			"dp_set_pmapper: def_ctp: %i, pcp: [%i, %i, %i, %i, %i, %i, %i, %i] : %i\n",
			pmap.def_ctp,
			pmap.pcp_map[0], pmap.pcp_map[1],
			pmap.pcp_map[2], pmap.pcp_map[3],
			pmap.pcp_map[4], pmap.pcp_map[5],
			pmap.pcp_map[6], pmap.pcp_map[7], err);
		if (err != DP_SUCCESS)
			netdev_err(pmapper->ndev, "dp_set_pmapper failed\n");
	}
	return err;
}

/** Fills the NetLink message with the configuration of this GEM port. */
static int ltq_pon_net_gem_fill_info(struct sk_buff *skb,
				     const struct net_device *gem_ndev)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);
	struct ltq_pon_net_tcont *tcont = gem->tcont;
	dp_subif_t dp_subif;
	int ret;

	if (nla_put_u16(skb, IFLA_PON_GEM_IDX, gem->gem_idx))
		return -ENOBUFS;

	if (nla_put_u32(skb, IFLA_PON_GEM_ID, gem->gem_id))
		return -ENOBUFS;

	if (nla_put_u16(skb, IFLA_PON_GEM_MAX_SIZE, gem->gem_max_size))
		return -ENOBUFS;

	if (nla_put_u8(skb, IFLA_PON_GEM_TRAFFIC_TYPE, gem->gem_traffic_type))
		return -ENOBUFS;

	if (nla_put_u8(skb, IFLA_PON_GEM_DIR, gem->gem_dir))
		return -ENOBUFS;

	if (nla_put_u8(skb, IFLA_PON_GEM_ENC, gem->gem_enc))
		return -ENOBUFS;

	if (nla_put_u8(skb, IFLA_PON_GEM_MC, gem->multicast))
		return -ENOBUFS;

	if (tcont && tcont->ndev) {
		if (nla_put_u32(skb, IFLA_PON_GEM_TCONT, tcont->ndev->ifindex))
			return -ENOBUFS;
	} else {
		if (nla_put_u32(skb, IFLA_PON_GEM_TCONT, 0))
			return -ENOBUFS;
	}

	/* Do not handle error because this is an uncritical debugging attribute
	 * and we continue to not break the main function
	 */
	ret = dp_get_netif_subifid(gem->ndev, NULL, NULL, NULL, &dp_subif, 0);
	if (ret == DP_SUCCESS) {
		if (nla_put_u16(skb, IFLA_PON_GEM_BP, dp_subif.bport))
			return -ENOBUFS;
	}

	if (nla_put_u32(skb, IFLA_PON_GEM_CTP, dp_subif.ctp))
		return -ENOBUFS;

	return 0;
}

/** This is called to configure the newly created GEM port interface. The
 * interface itself is created by the Linux stack this just does the GEM port
 * specific initial configuration.
 */
static void ltq_pon_net_gem_setup(struct net_device *gem_ndev)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);

	ether_setup(gem_ndev);

#if (KERNEL_VERSION(4, 10, 0) <= LINUX_VERSION_CODE)
	gem_ndev->min_mtu = 14;
	gem_ndev->max_mtu = 9216;
#endif

	gem_ndev->netdev_ops = &ltq_pon_net_gem_ops;
	gem_ndev->ethtool_ops = &ethtool_ops;
#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
	gem_ndev->destructor = free_netdev;
#else
	gem_ndev->priv_destructor = free_netdev;
#endif

	eth_hw_addr_random(gem_ndev);

	gem_ndev->features = NETIF_F_SG;
#if IS_ENABLED(CONFIG_QOS_TC)
	gem_ndev->features |= NETIF_F_HW_TC;
#endif
	gem_ndev->hw_features = gem_ndev->features;
	gem_ndev->vlan_features = gem_ndev->features;

	INIT_LIST_HEAD(&gem->node);
	INIT_LIST_HEAD(&gem->tcont_node);
}

static int ltq_pon_net_gem_port_idx_get(struct ltq_pon_net_gem *gem)
{
	struct ponfw_gem_port_id fw_gem_input = {0};
	struct ponfw_gem_port_id fw_gem_output = {0};
	struct list_head *ele;
	struct ltq_pon_net_gem *gem_ele;
	int err;

	/* Do not change index if it was already assigned before. */
	if (gem->gem_idx)
		return 0;

	fw_gem_input.gem_port_id = gem->gem_id;

	err = pon_mbox_send(PONFW_GEM_PORT_ID_CMD_ID, PONFW_READ,
			    &fw_gem_input, PONFW_GEM_PORT_ID_LENR,
			    &fw_gem_output, sizeof(fw_gem_output));
	if (err != sizeof(fw_gem_output)) {
		netdev_err(gem->ndev,
			   "failed to get GEM idx for GEM id (%i): %i\n",
			   gem->gem_id, err);
		return err;
	}

	list_for_each(ele, &gem->hw->gem_list) {
		gem_ele = list_entry(ele, struct ltq_pon_net_gem, node);
		if (gem_ele->gem_idx == fw_gem_output.gem_port_idx &&
		    gem_ele != gem) {
			netdev_err(gem->ndev,
				   "shares the same GEM index (%i) as %s\n",
				   fw_gem_output.gem_port_idx,
				   netdev_name(gem_ele->ndev));
			return -EINVAL;
		}
	}

	gem->gem_idx = fw_gem_output.gem_port_idx;
	g_gem_idx_used[gem->gem_idx] = true;

	netdev_dbg(gem->ndev,
		   "GEM idx(%u) is assigned for GEM id (%i)\n",
		   gem->gem_idx, gem->gem_id);

	return 0;
}

static void ltq_pon_net_gem_delete(struct ltq_pon_net_gem *gem)
{
	struct ponfw_gem_port_id_remove fw_gem_input = {0};
	struct ponfw_gem_port_id_remove fw_gem_output = {0};
	int err;

	g_gem_idx_used[gem->gem_idx] = false;

	if (gem->gem_traffic_type == PONFW_GEM_PORT_ID_TT_OMCI)
		return;

	if (!g_hw->in_o5)
		return;

	fw_gem_input.gem_port_id = gem->gem_id;

	err = pon_mbox_send(PONFW_GEM_PORT_ID_REMOVE_CMD_ID, PONFW_WRITE,
			     &fw_gem_input, PONFW_GEM_PORT_ID_REMOVE_LEN,
			     &fw_gem_output, sizeof(fw_gem_output));
	if (err < 0) {
		netdev_err(gem->ndev, "failed to remove GEM port id (%i): %i\n",
			   gem->gem_id, err);
		return;
	}

	netdev_dbg(gem->ndev, "GEM id (%i) removed\n", gem->gem_id);
}

static int ltq_pon_net_gem_link_apply(struct ltq_pon_net_gem *gem,
				      struct nlattr *tb[],
				      struct nlattr *data[])
{
	u32 tcont_ndev_idx = 0;
	struct net_device *tcont_ndev;
	struct ltq_pon_net_tcont *tcont = NULL;
	struct ltq_pon_net_tcont *tcont_previous = NULL;
	const struct net_device_ops *tcont_ops;
	struct net_device *dp_ndev = gem->ndev;
	int err;

	if (data[IFLA_PON_GEM_TCONT]) {
		tcont_ndev_idx = nla_get_u32(data[IFLA_PON_GEM_TCONT]);

		/* First check if the tcont interface changed from the one
		 * which is already configured. If it changed adapt the network
		 * configuration and then deregister and register the network
		 * device on the datapath library.
		 */
		if (tcont_ndev_idx) {
			tcont_ndev = dev_get_by_index(gem->src_net,
						      tcont_ndev_idx);
			if (!tcont_ndev) {
				netdev_err(dp_ndev, "Invalid tcont_ndev!\n");
				return -ENODEV;
			}

			tcont_ops = tcont_ndev->netdev_ops;

			/* check if this is a pon master device */
			if (tcont_ops->ndo_open != &ltq_pon_net_tcont_open) {
				dev_put(tcont_ndev);
				netdev_err(dp_ndev, "Added interface (%s) is not a T-CONT\n",
						tcont_ndev ?
						netdev_name(tcont_ndev) : NULL);
				return -ENODEV;
			}
			tcont = netdev_priv(tcont_ndev);
			if (gem->tcont != tcont) {
				gem->dp_update = true;
				if (gem->tcont) {
					list_del(&gem->tcont_node);
					dev_put(gem->tcont->ndev);
				}
				tcont_previous = gem->tcont;
				gem->tcont = tcont;
				list_add(&gem->tcont_node, &tcont->gem_list);
			} else {
				/* The driver already references this device so
				 * no need to increase the ref counter again.
				 */
				dev_put(tcont_ndev);
			}
		} else {
			if (gem->tcont != NULL) {
				gem->dp_update = true;
				list_del(&gem->tcont_node);
				dev_put(gem->tcont->ndev);
				tcont_previous = gem->tcont;
				gem->tcont = NULL;
				/*
				 * TODO:
				 * Remove tc queues using this tcont and gem.
				 */
			}
		}
	}

	if (gem->dp_update) {
		gem->dp_update = false;

		if (gem->dp_connected) {
			/* In case the device was already connected, use the
			 * same device again to connect to the datapath library
			 * again. When this gem port is part of a pmapper this
			 * is the pmapper net device and not the gem net device.
			 */
			err = ltq_pon_net_dp_gem_update(gem,
				gem->pmapper ? gem->pmapper->ndev : NULL);
			if (err)
				goto err_upd_ndev;

			return 0;
		}

		err = ltq_pon_net_gem_connect(gem, dp_ndev);
		if (err)
			goto err_upd_ndev;
		if (gem->pmapper) {
			err = ltq_pon_net_pmapper_set_mapping(gem->pmapper);
			if (err)
				goto err_tcont_pmapper;
		}
	}

	return 0;

err_tcont_pmapper:
	ltq_pon_net_gem_disconnect(gem, gem->tcont);

err_upd_ndev:
	if (tcont_ndev_idx) {
		list_del(&gem->tcont_node);
		dev_put(tcont_ndev);
		gem->tcont = NULL;
	}
	if (tcont_previous) {
		gem->tcont = tcont_previous;
		list_add(&gem->tcont_node, &tcont_previous->gem_list);
		dev_hold(tcont_previous->ndev);
	}

	return err;
}

static int ltq_pon_net_gem_validate(struct ltq_pon_net_gem *gem,
				    struct nlattr *data[])
{

	u8 gem_traffic_type = gem->gem_traffic_type;
	u8 gem_dir = gem->gem_dir;
	u8 gem_enc = gem->gem_enc;
	/* It is sufficient to check if t-cont is set. */
	u32 tcont_ndev_idx = gem->tcont ? 1 : 0;

	if (data[IFLA_PON_GEM_TRAFFIC_TYPE])
		gem_traffic_type = nla_get_u8(data[IFLA_PON_GEM_TRAFFIC_TYPE]);

	if (data[IFLA_PON_GEM_DIR])
		gem_dir = nla_get_u8(data[IFLA_PON_GEM_DIR]);

	if (data[IFLA_PON_GEM_ENC])
		gem_enc = nla_get_u8(data[IFLA_PON_GEM_ENC]);

	if (data[IFLA_PON_GEM_TCONT])
		tcont_ndev_idx = nla_get_u32(data[IFLA_PON_GEM_TCONT]);

	if (gem_dir != PONFW_GEM_PORT_ID_DIR_US &&
	    gem_dir != PONFW_GEM_PORT_ID_DIR_DS &&
	    gem_dir != PONFW_GEM_PORT_ID_DIR_BI) {
		netdev_err(gem->ndev, "invalid GEM port direction: %i\n",
			   gem_dir);
		return -EINVAL;
	}

	if ((gem_dir == PONFW_GEM_PORT_ID_DIR_US ||
	     gem_dir == PONFW_GEM_PORT_ID_DIR_BI) &&
	    !tcont_ndev_idx) {
		netdev_err(gem->ndev,
			   "%i: US or BI direction only allowed with T-Cont\n",
			   gem_dir);
		return -EINVAL;
	}

	if (gem_enc > PONFW_GEM_PORT_ID_ENC_DSEN) {
		netdev_err(gem->ndev, "invalid GEM port encryption: %i\n",
			   gem_enc);
		return -EINVAL;
	}

	if (gem_traffic_type > PONFW_GEM_PORT_ID_TT_ETHM) {
		netdev_err(gem->ndev, "invalid GEM port traffic type: %i\n",
			   gem_traffic_type);
		return -EINVAL;
	}

	if (data[IFLA_PON_GEM_MAX_SIZE])
		gem->gem_max_size = nla_get_u16(data[IFLA_PON_GEM_MAX_SIZE]);

	if (data[IFLA_PON_GEM_TRAFFIC_TYPE])
		gem->gem_traffic_type =
			nla_get_u8(data[IFLA_PON_GEM_TRAFFIC_TYPE]);

	if (data[IFLA_PON_GEM_DIR])
		gem->gem_dir = nla_get_u8(data[IFLA_PON_GEM_DIR]);

	if (data[IFLA_PON_GEM_ENC])
		gem->gem_enc = nla_get_u8(data[IFLA_PON_GEM_ENC]);

	if (data[IFLA_PON_GEM_MC]) {
		gem->multicast = nla_get_u8(data[IFLA_PON_GEM_MC]);
		gem->dp_update = true;
	}

	return 0;
}

static int ltq_pon_net_gem_valid_id(struct net_device *ndev, u32 gem_id)
{
	enum pon_mode mode = pon_mbox_get_pon_mode();

	switch (mode) {
	case PON_MODE_984_GPON:
	case PON_MODE_989_NGPON2_2G5:
	case PON_MODE_989_NGPON2_10G:
		if (gem_id > 4095) {
			netdev_err(ndev, "id (%i) not in valid range [0 ... 4095]\n",
				   gem_id);
			return -EINVAL;
		}
		return 0;
	case PON_MODE_987_XGPON:
	case PON_MODE_9807_XGSPON:
		if (gem_id < 1021 || gem_id > 65534) {
			netdev_err(ndev, "id (%i) not in valid range [1021 ... 65534]\n",
				   gem_id);
			return -EINVAL;
		}
		return 0;
	case PON_MODE_UNKNOWN:
	case PON_MODE_AON:
	default:
		netdev_err(ndev, "system in unsupported pon mode\n");
		return -EINVAL;
	}
}

static int ltq_pon_net_gem_id_unique_check(struct ltq_pon_net_gem *gem)
{
	struct list_head *ele;
	struct ltq_pon_net_gem *gem_ele;

	list_for_each(ele, &gem->hw->gem_list) {
		gem_ele = list_entry(ele, struct ltq_pon_net_gem, node);
		/* Condition checks if the traffic type of compared GEM is
		 * TT_OMCI and the error is not returned even if GEM with
		 * such type already used the same ID. It is a workaround
		 * used because of lack of synchronization between the driver
		 * and FW. For GEM OMCI the ID in the PON Ethernet driver could
		 * be outdated and the PON FW could already use a different one.
		 */
		if (gem_ele->gem_id == gem->gem_id &&
		    gem_ele != gem &&
		    gem_ele->gem_traffic_type != PONFW_GEM_PORT_ID_TT_OMCI) {
			netdev_err(gem->ndev,
				   "GEM with id (%i) already exists: %s\n",
				   gem->gem_id,
				   netdev_name(gem_ele->ndev));
			return -EINVAL;
		}
	}

	return 0;
}

/** Creates a new GEM port on the Interface given in IFLA_LINK. */
#if (KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE)
static int ltq_pon_net_gem_newlink(struct net *src_net,
				   struct net_device *gem_ndev,
				   struct nlattr *tb[], struct nlattr *data[])
#else
static int ltq_pon_net_gem_newlink(struct net *src_net,
				   struct net_device *gem_ndev,
				   struct nlattr *tb[],
				   struct nlattr *data[],
				   struct netlink_ext_ack *extack)
#endif
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);
	struct ltq_pon_net_hw *hw;
	struct net_device *hw_ndev;
	int err;

	if (!tb[IFLA_LINK]) {
		netdev_err(gem_ndev, "Missing IFLA_LINK!\n");
		return -EINVAL;
	}
	hw_ndev = dev_get_by_index(src_net, nla_get_u32(tb[IFLA_LINK]));
	if (!hw_ndev) {
		netdev_err(gem_ndev, "Invalid hw_ndev!\n");
		return -ENODEV;
	}

	/* check if this is a pon master device */
	if (hw_ndev->netdev_ops->ndo_open != &ltq_pon_net_mdev_open) {
		netdev_err(gem_ndev, "Invalid pon device!\n");
		err = -ENODEV;
		goto err_put_dev;
	}

	hw = netdev_priv(hw_ndev);
	gem->hw = hw;
	gem->ndev = gem_ndev;
	gem->src_net = src_net;
	gem->valid = true;
	gem->multicast = false;
	gem->dp_update = true;

	if (hw->dp_mode_registered != DP_F_GPON) {
		netdev_err(gem_ndev, "system not in ITU PON mode!\n");
		err = -ENOTSUPP;
		goto err_put_dev;
	}

	if (!data || (!data[IFLA_PON_GEM_IDX] && !data[IFLA_PON_GEM_ID])) {
		netdev_err(gem_ndev,
			   "Invalid GEM id provided!\n");
		err = -EINVAL;
		goto err_put_dev;
	}

	err = ltq_pon_net_gem_validate(gem, data);
	if (err)
		goto err_put_dev;

	if (data[IFLA_PON_GEM_IDX]) {
		if (gem->gem_traffic_type != PONFW_GEM_PORT_ID_TT_OMCI) {
			netdev_err(gem_ndev,
				   "setting GEM index only allowed for OMCI\n");
			err = -EINVAL;
			goto err_put_dev;
		}
		gem->gem_idx = nla_get_u16(data[IFLA_PON_GEM_IDX]);
		if (gem->gem_idx) {
			netdev_err(gem_ndev,
				   "GEM Index is special argument for gem-omci, do not use!\n");
			err = -EINVAL;
			goto err_put_dev;
		}
		/* OMCI traffic needs the CPU port */
		gem->common.ethtool_flags |= ETHTOOL_FLAG_BP_CPU_ENABLE;
	}

	if (data[IFLA_PON_GEM_ID]) {
		gem->gem_id = nla_get_u32(data[IFLA_PON_GEM_ID]);

		err = ltq_pon_net_gem_valid_id(gem_ndev, gem->gem_id);
		if (err)
			goto err_put_dev;

		err = ltq_pon_net_gem_id_unique_check(gem);
		if (err)
			goto err_put_dev;

		err = ltq_pon_net_gem_fw_apply(gem, NULL);
		if (err)
			goto err_put_dev;

		/* The GEM index is needed for DP and assigned by the PON FW */
		err = ltq_pon_net_gem_port_idx_get(gem);
		if (err)
			goto err_gem_id_remove;
	}

	err = register_netdevice(gem_ndev);
	if (err) {
		netdev_err(gem_ndev, "gem_ndev registration error: %i\n", err);
		goto err_gem_id_remove;
	}

	err = ltq_pon_net_gem_link_apply(gem, tb, data);
	if (err)
		goto err_gem_unregister;

	/* Call it again to also apply the T-Cont */
	err = ltq_pon_net_gem_fw_apply(gem, gem->tcont);
	if (err) {
		netdev_err(gem_ndev, "Update PON FW failed: %i", err);
		goto err_gem_unlink;
	}

	gem_port_id_write_update(gem->gem_idx);

	list_add(&gem->node, &hw->gem_list);

	netif_carrier_on(gem_ndev);
	return 0;

err_gem_unlink:
	ltq_pon_net_gem_disconnect(gem, gem->tcont);
	if (gem->tcont) {
		list_del(&gem->tcont_node);
		dev_put(gem->tcont->ndev);
		gem->tcont = NULL;
	}
err_gem_unregister:
	unregister_netdevice(gem_ndev);
err_gem_id_remove:
	if (data[IFLA_PON_GEM_ID])
		ltq_pon_net_gem_delete(gem);
err_put_dev:
	dev_put(hw_ndev);

	return err;
}

#if (KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE)
static int ltq_pon_net_gem_changelink(struct net_device *gem_ndev,
				      struct nlattr *tb[],
				      struct nlattr *data[])
#else
static int ltq_pon_net_gem_changelink(struct net_device *gem_ndev,
				      struct nlattr *tb[],
				      struct nlattr *data[],
				      struct netlink_ext_ack *extack)
#endif
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);
	int err;

	if (!data || gem->gem_traffic_type == PONFW_GEM_PORT_ID_TT_OMCI) {
		netdev_err(gem_ndev, "Invalid data\n");
		return -EINVAL;
	}

	if (data[IFLA_PON_GEM_IDX] &&
	    nla_get_u16(data[IFLA_PON_GEM_IDX]) != gem->gem_idx) {
		netdev_err(gem_ndev, "Changing GEM index to %i not allowed\n",
			   nla_get_u16(data[IFLA_PON_GEM_IDX]));
		return -EINVAL;
	}

	if (data[IFLA_PON_GEM_ID] &&
	    nla_get_u32(data[IFLA_PON_GEM_ID]) != gem->gem_id) {
		netdev_err(gem_ndev, "Changing GEM ID to %i not allowed\n",
			   nla_get_u32(data[IFLA_PON_GEM_ID]));
		return -EINVAL;
	}

	err = ltq_pon_net_gem_validate(gem, data);
	if (err)
		return err;

	err = ltq_pon_net_gem_link_apply(gem, tb, data);
	if (err) {
		netdev_err(gem_ndev, "Applying t-cont failed: %i", err);
		return -EINVAL;
	}

	err = ltq_pon_net_gem_fw_apply(gem, gem->tcont);
	if (err)
		netdev_err(gem_ndev, "Update PON FW failed: %i", err);

	return err;
}

static void ltq_pon_net_mdev_archive_stats(struct net_device *netdev, u64 *data,
					   int count)
{
	int i;
	struct ltq_pon_net_hw *hw = netdev_priv(netdev);

	if (!hw->ani_archived_stats) {
		hw->ani_archived_stats =
		    kcalloc(count, sizeof(*hw->ani_archived_stats), GFP_KERNEL);
		if (!hw->ani_archived_stats)
			return;

		hw->num_ani_archived_stats = count;
	}

	if (hw->num_ani_archived_stats != count) {
		netdev_err(netdev, "%s: attempting to archive unexpected number of counters",
			   __func__);
		return;
	}

	for (i = 0; i < count; i++)
		hw->ani_archived_stats[i] += data[i];
}

static u64 *duplicate_ethtool_stats(struct net_device *netdev, int *out_count)
{
	struct ethtool_stats stats;
	int count;
	u64 *data;

	count = dp_net_dev_get_ss_stat_strings_count(netdev);
	if (count <= 0)
		return NULL;

	stats.cmd = ETHTOOL_GSTATS;
	stats.n_stats = count;

	data = kcalloc(count, sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	dp_net_dev_get_ethtool_stats(netdev, &stats, data);

	if (out_count)
		*out_count = count;

	return data;
}

static void archive_gem_port_stats(struct net_device *gem_ndev)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);
	u64 *stats;
	int num_stats;

	stats = duplicate_ethtool_stats(gem_ndev, &num_stats);
	if (!stats)
		return;

	ltq_pon_net_mdev_archive_stats(gem->hw->ndev, stats, num_stats);
	kfree(stats);
}

/**
 * Deletes one GEM port, deregistration from dp library is done
 * in the uninit callback later, because dellink is called before Linux
 * removes the tc queue configuration and this has to be done before we
 * remove the interface from DP manager.
 */
static void ltq_pon_net_gem_dellink(struct net_device *gem_ndev,
				    struct list_head *head)
{
	struct ltq_pon_net_gem *gem = netdev_priv(gem_ndev);
	struct ltq_pon_net_tcont *tcont = gem->tcont;

	if (gem->valid)
		netif_carrier_off(gem_ndev);
	gem_port_id_remove_update(gem->gem_idx);

	/* When a GEM port is deleted we should still count the packets on the
	 * pon0 counters. To achieve this we must store the accumulated counter
	 * values from all deleted GEM ports and then add them to the final
	 * pon0 counter values.
	 */
	archive_gem_port_stats(gem_ndev);

	ltq_pon_net_gem_delete(gem);

	if (tcont) {
		list_del(&gem->tcont_node);
		dev_put(tcont->ndev);
		gem->tcont = NULL;
	}

	unregister_netdevice_queue(gem_ndev, head);
}

/** Rtnl ops to create and delete GEM ports.
 */
static struct rtnl_link_ops ltq_pon_net_gem_rtnl __read_mostly = {
	.kind		= "gem",
	.priv_size	= sizeof(struct ltq_pon_net_gem),

	.get_size	= ltq_pon_net_gem_nl_getsize,
	.policy		= ltq_pon_net_gem_nl_policy,
	.fill_info	= ltq_pon_net_gem_fill_info,

	.newlink	= ltq_pon_net_gem_newlink,
	.changelink	= ltq_pon_net_gem_changelink,
	.dellink	= ltq_pon_net_gem_dellink,
	.setup		= ltq_pon_net_gem_setup,
	.maxtype	= IFLA_PON_GEM_MAX,
};

/** The NetLink messages the T-CONT configuration. */
enum {
	IFLA_PON_TCONT_UNSPEC,
	IFLA_PON_TCONT_IDX,
	IFLA_PON_TCONT_ID,
	IFLA_PON_TCONT_MAX
};

static const struct nla_policy
ltq_pon_net_tcont_nl_policy[IFLA_PON_TCONT_MAX] = {
	[IFLA_PON_TCONT_IDX]		= { .type = NLA_U16 },
	[IFLA_PON_TCONT_ID]		= { .type = NLA_U32 },
};

/** Returns the maximum size of a T-CONT configuration NetLink message */
static size_t ltq_pon_net_tcont_nl_getsize(const struct net_device *gem_ndev)
{
	return nla_total_size(sizeof(u16)) +	/* IFLA_PON_TCONT_IDX */
	       nla_total_size(sizeof(u32));	/* IFLA_PON_TCONT_ID */
}

/** Fills the NetLink message with the full configuration of this T-CONT. */
static int ltq_pon_net_tcont_fill_info(struct sk_buff *skb,
				       const struct net_device *tcont_ndev)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);

	if (nla_put_u16(skb, IFLA_PON_TCONT_IDX, tcont->qos_idx))
		return -ENOBUFS;

	if (nla_put_u32(skb, IFLA_PON_TCONT_ID, tcont->alloc_id))
		return -ENOBUFS;

	return 0;
}

/** This is called to configure the newly created T-CONT interface. The
 * interface itself is created by the Linux stack this just does the T-CONT
 * specific initial configuration.
 */
static void ltq_pon_net_tcont_setup(struct net_device *tcont_ndev)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);

	ether_setup(tcont_ndev);

	tcont_ndev->netdev_ops = &ltq_pon_net_tcont_ops;
#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
	tcont_ndev->destructor = free_netdev;
#else
	tcont_ndev->priv_destructor = free_netdev;
#endif

	eth_hw_addr_random(tcont_ndev);

	tcont_ndev->features = NETIF_F_SG;
#if IS_ENABLED(CONFIG_QOS_TC)
	tcont_ndev->features |= NETIF_F_HW_TC;
#endif
	tcont_ndev->hw_features = tcont_ndev->features;
	tcont_ndev->vlan_features = tcont_ndev->features;

	INIT_LIST_HEAD(&tcont->node);
	INIT_LIST_HEAD(&tcont->gem_list);
	tcont->queue_id = PON_ETH_QUEUE_UNASSIGNED;
}

/** Try to assign the QOS index that is already selected by the
 * T-CONT netdevice.
 */
static int ltq_pon_net_qos_idx_preassign(struct net_device *tcont_ndev)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);

	/* Forcing a QOS index is not possible if it is already in use. */
	if (g_qos_idx_used[tcont->qos_idx])
		return -EEXIST;

	g_qos_idx_used[tcont->qos_idx] = true;
	netdev_dbg(tcont_ndev,
		   "QOS Index (%u) pre-assigned for Alloc id (%u)\n",
		   tcont->qos_idx, tcont->alloc_id);
	return 0;
}

/** This function assign available QOS Index to provided TCONT netdevice. */
static int ltq_pon_net_qos_idx_assign(struct net_device *tcont_ndev)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);
	int i;

	/* QOS Index 0 is used by OMCI channel, so skip it */
	for (i = 1; i < ARRAY_SIZE(g_qos_idx_used); i++) {
		if (g_qos_idx_used[i])
			continue;

		g_qos_idx_used[i] = true;
		tcont->qos_idx = i;

		netdev_dbg(tcont_ndev,
			   "QOS Index (%u) assigned for Alloc id (%u)\n",
			   tcont->qos_idx, tcont->alloc_id);

		return 0;
	}

	netdev_err(tcont_ndev,
		   "failed to assign QOS Index for Alloc id (%u)\n",
		   tcont->alloc_id);

	return -ENODATA;
}

/** This function free QOS Index used by provided TCONT netdevice */
static void ltq_pon_net_qos_idx_free(struct net_device *tcont_ndev)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);

	/* If QOS Index was set to OMCI channel value 0, skip it */
	if (!tcont->qos_idx) {
		netdev_dbg(tcont_ndev,
			   "QOS Index free skipped for Alloc id (%u)\n",
			   tcont->alloc_id);
		return;
	}

	netdev_dbg(tcont_ndev,
		   "QOS Index (%u) freed for Alloc id (%u)\n",
		   tcont->qos_idx, tcont->alloc_id);

	g_qos_idx_used[tcont->qos_idx] = false;
	tcont->qos_idx = 0;
}

static int ltq_pon_net_qos_idx_link(struct net_device *tcont_ndev, bool read)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);
	struct ponfw_alloc_id_link fw_alloc_input = {0};
	struct ponfw_alloc_id_link fw_alloc_output = {0};
	int err;

	fw_alloc_input.alloc_id = tcont->alloc_id;
	fw_alloc_input.qos_idx = tcont->qos_idx;
	fw_alloc_input.ctr = read ? PONFW_ALLOC_ID_LINK_CTR_READ :
				    PONFW_ALLOC_ID_LINK_CTR_LINK;

	err = pon_mbox_send(PONFW_ALLOC_ID_LINK_CMD_ID, PONFW_READ,
			    &fw_alloc_input, PONFW_ALLOC_ID_LINK_LENR,
			    &fw_alloc_output,
			    sizeof(fw_alloc_output));
	if (err < 0) {
		/* ignore errors on read to handle not existing entries */
		if (!read)
			netdev_err(tcont_ndev,
				   "failed to link QOS Index (%u) for Alloc id (%i): %i\n",
				   tcont->qos_idx, tcont->alloc_id, err);
		return err;
	}

	if (err != sizeof(fw_alloc_output)) {
		err = -EINVAL;
		netdev_err(tcont_ndev,
			   "failed to get answer for link QOS Index (%u) for Alloc id (%i): %i\n",
			   tcont->qos_idx, tcont->alloc_id, err);
		return err;
	}

	tcont->alloc_idx = fw_alloc_output.alloc_link_ref & ALLOC_IDX_MASK;

	if (tcont->alloc_idx &&
	    fw_alloc_output.hw_status == PONFW_ALLOC_ID_LINK_HW_STATUS_LINKED) {
		tcont->alloc_link_ref = fw_alloc_output.alloc_link_ref;
#ifdef PONRTSYS_9459
		/* TODO: this will work only in future FW versions, see
		 * PONRTSYS-9441. Task for implementation is PONRTSYS-9459)
		 */
		tcont->qos_idx = fw_alloc_output.qos_idx;
#else
		if (read)
			tcont->qos_idx =
				g_alloc_idx_to_qos_idx[tcont->alloc_idx];
		else
			g_alloc_idx_to_qos_idx[tcont->alloc_idx] =
				tcont->qos_idx;
#endif
		netdev_dbg(tcont_ndev,
			   "%slinked QOS Index (%u) for Alloc id (%i) with Version number (0x%x)\n",
			   read ? "read " : "",
			   tcont->qos_idx, tcont->alloc_id,
			   tcont->alloc_link_ref);

		alloc_id_write_update(tcont->alloc_idx);
	}

	return 0;
}

static int ltq_pon_net_qos_idx_link_set(struct net_device *tcont_ndev)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);
	int err;

	/* Check if QOS Index is already linked to allocation */
	if (!tcont->alloc_id_valid || tcont->alloc_link_ref) {
		netdev_dbg(tcont_ndev,
			   "QOS Index link skipped for Alloc id (%u)\n",
			   tcont->alloc_id);
		return 0;
	}

	err = ltq_pon_net_qos_idx_link(tcont_ndev, false);
	if (err)
		return err;

	netdev_dbg(tcont_ndev,
		   "QOS Index (%u) link requested for Alloc id (%u)\n",
		   tcont->qos_idx, tcont->alloc_id);

	return 0;
}

static int ltq_pon_net_qos_idx_link_get(struct net_device *tcont_ndev)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);
	int err;

	/* Check if QOS Index is linked to allocation and we already know the
	 * QOS Index
	 */
	if (tcont->alloc_link_ref && tcont->qos_idx) {
		netdev_dbg(tcont_ndev,
			   "QOS Index link get skipped for Alloc id (%u)\n",
			   tcont->alloc_id);
		return 0;
	}

	err = ltq_pon_net_qos_idx_link(tcont_ndev, true);
	if (err)
		return err;

	/* no valid entry found, indicate with error */
	if (!tcont->alloc_link_ref)
		return -ENOENT;

	netdev_dbg(tcont_ndev,
		   "QOS Index (%u) already linked for Alloc id (%u) with Version number (0x%x)\n",
		   tcont->qos_idx, tcont->alloc_id, tcont->alloc_link_ref);

	return 0;
}

static void ltq_pon_net_ack_alloc_unlink(struct ponfw_alloc_id_unlink *fw_alloc)
{
	struct ponfw_alloc_id_unlink fw_alloc_input = {0};
	int err;

	fw_alloc_input.alloc_id = fw_alloc->alloc_id;
	fw_alloc_input.alloc_link_ref = fw_alloc->alloc_link_ref;
	err = pon_mbox_send(PONFW_ALLOC_ID_UNLINK_CMD_ID, PONFW_WRITE,
			    &fw_alloc_input, sizeof(fw_alloc_input),
			    NULL, 0);
	if (err < 0)
		pr_err("pon_eth: ALLOC_ID_UNLINK ack was rejected by FW: %i\n",
		       err);
}

static void ltq_pon_net_qos_idx_flush(struct ltq_pon_net_tcont *tcont)
{
	struct dp_qos_blk_flush_port flush_cfg = {0,};
	int err;

	flush_cfg.inst = 0;
	flush_cfg.dp_port = tcont->hw->port_id;
	flush_cfg.deq_port_idx = tcont->qos_idx;
	/* This will flush all queues connected to this dequeue port.
	 * This operation takes about 40 to 100ms per dequeue port on PRX300.
	 */
	err = dp_block_flush_port(&flush_cfg, DP_QFLUSH_FLAG_RESTORE_LOOKUP);
	if (err != DP_SUCCESS)
		netdev_err(tcont->ndev, "DP flush for deq_port_idx: %i failed\n",
			   tcont->qos_idx);
}

static void ltq_pon_net_qos_idx_unlink(struct net_device *tcont_ndev,
				       struct ponfw_alloc_id_unlink *fw_alloc)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);
	struct ponfw_alloc_id_unlink fw_alloc_input = {0};
	int err = 0;

	if (!tcont->alloc_link_ref) {
		if (!fw_alloc) {
			netdev_dbg(tcont_ndev,
				"QOS Index unlink skipped for Alloc id (%u)\n",
				tcont->alloc_id);
			return;
		}
		/* Here the SW thinks this T-CONT is not configured or
		 * linked in the PON FW, but the PON FW still sends us
		 * a unlink event. We have seen this condition in some
		 * tests, but we do not understand why we get into
		 * this situation. We should send an acknowledge anyway
		 * to not block this index in the PON FW. After the
		 * acknowledge the state of both systems should be
		 * in sync again.
		 */
		netdev_warn(tcont_ndev, "ALLOC_ID_UNLINK event for unlinked tcont, alloc_id: %i, alloc_link_ref: %i\n",
			    fw_alloc->alloc_id, fw_alloc->alloc_link_ref);
		ltq_pon_net_ack_alloc_unlink(fw_alloc);
		return;
	}

	fw_alloc_input.alloc_id = tcont->alloc_id;
	fw_alloc_input.alloc_link_ref = tcont->alloc_link_ref;

	/* We should only flush queues if we not in operational state
	 * as it would cause issues with traffic on operating T-CONTs.
	 */
	if (!g_hw->in_o5) {
		ltq_pon_net_qos_idx_flush(tcont);
	} else {
		/* We should send a unlink message to the PON FW only when this
		 * was requested in operational state, in non-operational state
		 * firmware already performed resources cleanup independently.
		 */
		err = pon_mbox_send(PONFW_ALLOC_ID_UNLINK_CMD_ID, PONFW_WRITE,
				    &fw_alloc_input, sizeof(fw_alloc_input),
				    NULL, 0);
		if (err < 0) {
			netdev_err(tcont_ndev,
				   "failed to unlink QOS Index (%u) for Alloc id (%i): %i\n",
				   tcont->qos_idx, tcont->alloc_id, err);
			return;
		}
	}
	if (tcont->alloc_idx)
		g_alloc_idx_to_qos_idx[tcont->alloc_idx] = 0;

	alloc_id_remove_update(tcont->alloc_idx);

	netdev_dbg(tcont_ndev,
		   "unlinked QOS Index (%u) for Alloc id (%i) with Version number (0x%x)\n",
		   tcont->qos_idx, tcont->alloc_id, tcont->alloc_link_ref);

	tcont->alloc_link_ref = 0;
}

static int ltq_pon_net_tcont_valid_id(struct net_device *ndev, u32 alloc_id)
{
	enum pon_mode mode = pon_mbox_get_pon_mode();

	switch (mode) {
	case PON_MODE_984_GPON:
	case PON_MODE_989_NGPON2_2G5:
	case PON_MODE_989_NGPON2_10G:
		if (alloc_id < 256 || alloc_id > 4095) {
			netdev_err(ndev, "id (%i) not in valid range [256 ... 4095]\n",
				   alloc_id);
			return -EINVAL;
		}
		return 0;
	case PON_MODE_987_XGPON:
	case PON_MODE_9807_XGSPON:
		if (alloc_id < 1024 || alloc_id > 16383) {
			netdev_err(ndev, "id (%i) not in valid range [1024 ... 16383]\n",
				   alloc_id);
			return -EINVAL;
		}
		return 0;
	case PON_MODE_UNKNOWN:
	case PON_MODE_AON:
		netdev_err(ndev, "system in unsupported pon mode\n");
		return -EINVAL;
	}
	return -EINVAL;
}

/** Creates a new T-CONT on the interface given in IFLA_LINK. */
#if (KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE)
static int ltq_pon_net_tcont_newlink(struct net *src_net,
				     struct net_device *tcont_ndev,
				     struct nlattr *tb[], struct nlattr *data[])
#else
static int ltq_pon_net_tcont_newlink(struct net *src_net,
				     struct net_device *tcont_ndev,
				     struct nlattr *tb[], struct nlattr *data[],
				     struct netlink_ext_ack *extack)
#endif
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);
	struct ltq_pon_net_hw *hw;
	struct net_device *hw_ndev;
	int err;

	if (!tb[IFLA_LINK]) {
		netdev_err(tcont_ndev, "Missing IFLA_LINK!\n");
		return -EINVAL;
	}

	hw_ndev = dev_get_by_index(src_net, nla_get_u32(tb[IFLA_LINK]));
	if (!hw_ndev) {
		netdev_err(tcont_ndev, "Invalid/missing netlink value hw_ndev\n");
		return -ENODEV;
	}

	/* check if this is a pon master device */
	if (hw_ndev->netdev_ops->ndo_open != &ltq_pon_net_mdev_open) {
		netdev_err(tcont_ndev, "Invalid pon device\n");
		err = -ENODEV;
		goto err_put_dev;
	}

	hw = netdev_priv(hw_ndev);

	if (hw->dp_mode_registered != DP_F_GPON) {
		netdev_err(tcont_ndev, "system not in ITU PON mode!\n");
		err = -ENOTSUPP;
		goto err_put_dev;
	}

	if (!data || (!data[IFLA_PON_TCONT_IDX] && !data[IFLA_PON_TCONT_ID])) {
		netdev_err(tcont_ndev,
			   "Invalid TCONT id provided!\n");
		err = -EINVAL;
		goto err_put_dev;
	}

	if (data[IFLA_PON_TCONT_IDX]) {
		tcont->alloc_idx = nla_get_u16(data[IFLA_PON_TCONT_IDX]);
		if (tcont->alloc_idx) {
			netdev_err(tcont_ndev,
				   "Alloc Index is special argument for tcont-omci, do not use!\n");
			err = -EINVAL;
			goto err_put_dev;
		}
		tcont->tcont_omci = true;
		alloc_id_write_update(tcont->alloc_idx);
	}

	if (data[IFLA_PON_TCONT_ID]) {
		tcont->alloc_id = nla_get_u32(data[IFLA_PON_TCONT_ID]);

		err = ltq_pon_net_tcont_valid_id(tcont_ndev, tcont->alloc_id);
		if (err)
			goto err_put_dev;

		/* check if the firmware has already a qos idx linked
		 * and try to reuse it.
		 */
		err = ltq_pon_net_qos_idx_link_get(tcont_ndev);
		if (!err) {
			err = ltq_pon_net_qos_idx_preassign(tcont_ndev);
			/* if this fails, the entry is already in use.
			 * Free it and request a new assignment below.
			 */
			if (err)
				ltq_pon_net_qos_idx_unlink(tcont_ndev, NULL);
			else
				tcont->alloc_id_valid = true;
		}

		if (!tcont->alloc_id_valid) {
			err = ltq_pon_net_qos_idx_assign(tcont_ndev);
			if (err)
				goto err_put_dev;

			tcont->alloc_id_valid = true;

			err = ltq_pon_net_qos_idx_link_set(tcont_ndev);
			if (err)
				goto err_put_dev_unassign;
		}
	}

	tcont->hw = hw;
	tcont->ndev = tcont_ndev;

	list_add(&tcont->node, &hw->tcont_list);

	err = register_netdevice(tcont_ndev);
	if (err)
		goto err_list_remove;

	return 0;

err_list_remove:
	list_del(&tcont->node);
	if (tcont->alloc_link_ref)
		ltq_pon_net_qos_idx_unlink(tcont_ndev, NULL);
err_put_dev_unassign:
	if (tcont->alloc_id_valid)
		ltq_pon_net_qos_idx_free(tcont_ndev);
err_put_dev:
	dev_put(hw_ndev);

	return err;
}

static int ltq_pon_net_gem_disconnect_tcont(struct ltq_pon_net_gem *gem)
{
	struct ltq_pon_net_tcont *tcont = gem->tcont;
	struct ltq_pon_net_pmapper *pmapper = gem->pmapper;

	if (tcont) {
		list_del(&gem->tcont_node);
		dev_put(tcont->ndev);
		gem->tcont = NULL;
		ltq_pon_net_gem_fw_apply(gem, NULL);
	}

	if (!gem->dp_connected)
		return 0;

	return ltq_pon_net_dp_gem_update(gem, pmapper ? pmapper->ndev : NULL);
}

/** Deletes one T-CONT and removes all subifs. */
static void ltq_pon_net_tcont_dellink(struct net_device *tcont_ndev,
				      struct list_head *head)
{
	struct ltq_pon_net_tcont *tcont = netdev_priv(tcont_ndev);
	struct ltq_pon_net_hw *hw = tcont->hw;
	struct ltq_pon_net_gem *gem;
	struct list_head *ele, *next;

	list_for_each_safe(ele, next, &tcont->gem_list) {
		gem = list_entry(ele, struct ltq_pon_net_gem, tcont_node);
		ltq_pon_net_gem_disconnect_tcont(gem);
	}

	ltq_pon_net_qos_idx_unlink(tcont_ndev, NULL);
	ltq_pon_net_qos_idx_free(tcont_ndev);

	list_del(&tcont->node);

	unregister_netdevice_queue(tcont_ndev, head);

	dev_put(hw->ndev);
}

static unsigned int ltq_pon_net_tcont_get_num_tx_queues(void)
{
	return PON_TCONT_TX_QUEUES;
}

/** Rtnl ops to create and delete T-CONT interface.
 */
static struct rtnl_link_ops ltq_pon_net_tcont_rtnl __read_mostly = {
	.kind		= "tcont",
	.priv_size	= sizeof(struct ltq_pon_net_tcont),

	.get_size	= ltq_pon_net_tcont_nl_getsize,
	.policy		= ltq_pon_net_tcont_nl_policy,
	.fill_info	= ltq_pon_net_tcont_fill_info,

	.newlink	= ltq_pon_net_tcont_newlink,
	.dellink	= ltq_pon_net_tcont_dellink,
	.setup		= ltq_pon_net_tcont_setup,
	.maxtype	= IFLA_PON_TCONT_MAX,
	.get_num_tx_queues = ltq_pon_net_tcont_get_num_tx_queues,
};

/** The NetLink messages the IEEE 802.1p mapper configuration. */
enum {
	IFLA_PON_PMAPPER_UNSPEC,
	IFLA_PON_PMAPPER_PCP_DEF,
	IFLA_PON_PMAPPER_PCP_0,
	IFLA_PON_PMAPPER_PCP_1,
	IFLA_PON_PMAPPER_PCP_2,
	IFLA_PON_PMAPPER_PCP_3,
	IFLA_PON_PMAPPER_PCP_4,
	IFLA_PON_PMAPPER_PCP_5,
	IFLA_PON_PMAPPER_PCP_6,
	IFLA_PON_PMAPPER_PCP_7,
	IFLA_PON_PMAPPER_MODE,
	IFLA_PON_PMAPPER_DSCP,
	/* For debbuging purposes only */
	IFLA_PON_PMAPPER_BP,
	IFLA_PON_GEM_CTP_0,
	IFLA_PON_GEM_CTP_1,
	IFLA_PON_GEM_CTP_2,
	IFLA_PON_GEM_CTP_3,
	IFLA_PON_GEM_CTP_4,
	IFLA_PON_GEM_CTP_5,
	IFLA_PON_GEM_CTP_6,
	IFLA_PON_GEM_CTP_7,
	IFLA_PON_PMAPPER_MAX
};

static const struct nla_policy
ltq_pon_net_pmapper_nl_policy[IFLA_PON_PMAPPER_MAX] = {
	[IFLA_PON_PMAPPER_PCP_DEF]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_PCP_0]	= { .type = NLA_U32 },
	[IFLA_PON_PMAPPER_PCP_1]	= { .type = NLA_U32 },
	[IFLA_PON_PMAPPER_PCP_2]	= { .type = NLA_U32 },
	[IFLA_PON_PMAPPER_PCP_3]	= { .type = NLA_U32 },
	[IFLA_PON_PMAPPER_PCP_4]	= { .type = NLA_U32 },
	[IFLA_PON_PMAPPER_PCP_5]	= { .type = NLA_U32 },
	[IFLA_PON_PMAPPER_PCP_6]	= { .type = NLA_U32 },
	[IFLA_PON_PMAPPER_PCP_7]	= { .type = NLA_U32 },
	[IFLA_PON_PMAPPER_MODE]		= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP]		= { .type = NLA_NESTED },
	[IFLA_PON_PMAPPER_BP]		= { .type = NLA_U16 },
	[IFLA_PON_GEM_CTP_0]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_CTP_1]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_CTP_2]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_CTP_3]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_CTP_4]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_CTP_5]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_CTP_6]		= { .type = NLA_U32 },
	[IFLA_PON_GEM_CTP_7]		= { .type = NLA_U32 },
};

enum {
	IFLA_PON_PMAPPER_DSCP_UNSPEC,
	IFLA_PON_PMAPPER_DSCP_0,
	IFLA_PON_PMAPPER_DSCP_1,
	IFLA_PON_PMAPPER_DSCP_2,
	IFLA_PON_PMAPPER_DSCP_3,
	IFLA_PON_PMAPPER_DSCP_4,
	IFLA_PON_PMAPPER_DSCP_5,
	IFLA_PON_PMAPPER_DSCP_6,
	IFLA_PON_PMAPPER_DSCP_7,
	IFLA_PON_PMAPPER_DSCP_8,
	IFLA_PON_PMAPPER_DSCP_9,
	IFLA_PON_PMAPPER_DSCP_10,
	IFLA_PON_PMAPPER_DSCP_11,
	IFLA_PON_PMAPPER_DSCP_12,
	IFLA_PON_PMAPPER_DSCP_13,
	IFLA_PON_PMAPPER_DSCP_14,
	IFLA_PON_PMAPPER_DSCP_15,
	IFLA_PON_PMAPPER_DSCP_16,
	IFLA_PON_PMAPPER_DSCP_17,
	IFLA_PON_PMAPPER_DSCP_18,
	IFLA_PON_PMAPPER_DSCP_19,
	IFLA_PON_PMAPPER_DSCP_20,
	IFLA_PON_PMAPPER_DSCP_21,
	IFLA_PON_PMAPPER_DSCP_22,
	IFLA_PON_PMAPPER_DSCP_23,
	IFLA_PON_PMAPPER_DSCP_24,
	IFLA_PON_PMAPPER_DSCP_25,
	IFLA_PON_PMAPPER_DSCP_26,
	IFLA_PON_PMAPPER_DSCP_27,
	IFLA_PON_PMAPPER_DSCP_28,
	IFLA_PON_PMAPPER_DSCP_29,
	IFLA_PON_PMAPPER_DSCP_30,
	IFLA_PON_PMAPPER_DSCP_31,
	IFLA_PON_PMAPPER_DSCP_32,
	IFLA_PON_PMAPPER_DSCP_33,
	IFLA_PON_PMAPPER_DSCP_34,
	IFLA_PON_PMAPPER_DSCP_35,
	IFLA_PON_PMAPPER_DSCP_36,
	IFLA_PON_PMAPPER_DSCP_37,
	IFLA_PON_PMAPPER_DSCP_38,
	IFLA_PON_PMAPPER_DSCP_39,
	IFLA_PON_PMAPPER_DSCP_40,
	IFLA_PON_PMAPPER_DSCP_41,
	IFLA_PON_PMAPPER_DSCP_42,
	IFLA_PON_PMAPPER_DSCP_43,
	IFLA_PON_PMAPPER_DSCP_44,
	IFLA_PON_PMAPPER_DSCP_45,
	IFLA_PON_PMAPPER_DSCP_46,
	IFLA_PON_PMAPPER_DSCP_47,
	IFLA_PON_PMAPPER_DSCP_48,
	IFLA_PON_PMAPPER_DSCP_49,
	IFLA_PON_PMAPPER_DSCP_50,
	IFLA_PON_PMAPPER_DSCP_51,
	IFLA_PON_PMAPPER_DSCP_52,
	IFLA_PON_PMAPPER_DSCP_53,
	IFLA_PON_PMAPPER_DSCP_54,
	IFLA_PON_PMAPPER_DSCP_55,
	IFLA_PON_PMAPPER_DSCP_56,
	IFLA_PON_PMAPPER_DSCP_57,
	IFLA_PON_PMAPPER_DSCP_58,
	IFLA_PON_PMAPPER_DSCP_59,
	IFLA_PON_PMAPPER_DSCP_60,
	IFLA_PON_PMAPPER_DSCP_61,
	IFLA_PON_PMAPPER_DSCP_62,
	IFLA_PON_PMAPPER_DSCP_63,
	IFLA_PON_PMAPPER_DSCP_MAX
};

static const struct nla_policy
ltq_pon_net_pmapper_dscp_nl_policy[IFLA_PON_PMAPPER_DSCP_MAX] = {
	[IFLA_PON_PMAPPER_DSCP_0]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_1]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_2]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_3]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_4]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_5]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_6]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_7]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_8]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_9]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_10]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_11]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_12]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_13]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_14]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_15]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_16]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_17]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_18]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_19]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_20]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_21]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_22]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_23]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_24]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_25]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_26]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_27]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_28]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_29]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_30]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_31]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_32]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_33]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_34]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_35]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_36]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_37]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_38]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_39]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_40]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_41]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_42]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_43]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_44]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_45]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_46]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_47]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_48]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_49]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_50]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_51]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_52]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_53]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_54]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_55]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_56]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_57]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_58]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_59]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_60]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_61]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_62]	= { .type = NLA_U8 },
	[IFLA_PON_PMAPPER_DSCP_63]	= { .type = NLA_U8 },
};

/** Returns the maximum size of a IEEE 802.1p mapper configuration NetLink
 * message
 */
static size_t
ltq_pon_net_pmapper_nl_getsize(const struct net_device *pmapper_ndev)
{
	return nla_total_size(sizeof(u8)) +	/* IFLA_PON_PMAPPER_PCP_DEF */
	       DP_PMAP_PCP_NUM * nla_total_size(sizeof(u32)) +
	       nla_total_size(sizeof(u8)) +	/* IFLA_PON_PMAPPER_MODE */
	       /* IFLA_PON_PMAPPER_DSCP */
	       nla_total_size(sizeof(struct nlattr)) +
	       DP_PMAP_DSCP_NUM * nla_total_size(sizeof(u8));
}

/** Fills the NetLink message with the full configuration of this
 * IEEE 802.1p mapper.
 */
static int ltq_pon_net_pmapper_fill_info(struct sk_buff *skb,
					 const struct net_device *pmapper_ndev)
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);
	struct nlattr *dscp;
	dp_subif_t dp_subif;
	int i, ret;

	if (nla_put_u8(skb, IFLA_PON_PMAPPER_PCP_DEF, pmapper->pcpdef))
		return -ENOBUFS;

	if (nla_put_u8(skb, IFLA_PON_PMAPPER_MODE, pmapper->mode))
		return -ENOBUFS;

	/* Do not handle error because this is an uncritical debugging attribute
	 * and we continue to not break the main function
	 */
	ret = dp_get_netif_subifid(pmapper->ndev, NULL, NULL, NULL,
				   &dp_subif, 0);
	if (ret == DP_SUCCESS) {
		if (nla_put_u16(skb, IFLA_PON_PMAPPER_BP, dp_subif.bport))
			return -ENOBUFS;
	}

	for (i = 0; i < DP_PMAP_PCP_NUM; i++) {
		if (pmapper->gem[i]) {
			if (nla_put_u32(skb, IFLA_PON_PMAPPER_PCP_0 + i,
					pmapper->gem[i]->ndev->ifindex))
				return -ENOBUFS;

			/* Do not handle error because this is an uncritical
			 * debugging attribute and we continue to not break the
			 * main function
			 */
			ret = dp_get_netif_subifid(pmapper->gem[i]->ndev, NULL,
						   NULL, NULL, &dp_subif, 0);
			if (ret == DP_SUCCESS) {
				if (nla_put_u32(skb, IFLA_PON_GEM_CTP_0 + i,
						dp_subif.ctp))
					return -ENOBUFS;
			}
		}
	}

	dscp = nla_nest_start(skb, IFLA_PON_PMAPPER_DSCP);
	if (dscp == NULL)
		return -ENOBUFS;

	for (i = 0; i < DP_PMAP_DSCP_NUM; i++) {
		if (pmapper->dscp_map[i] != PON_PMAPPER_DSCP_DEFAULT) {
			if (nla_put_u8(skb, IFLA_PON_PMAPPER_DSCP_0 + i,
					pmapper->dscp_map[i]))
				return -ENOBUFS;
		}
	}
	nla_nest_end(skb, dscp);

	return 0;
}

/** This is called to configure the newly created IEEE 802.1p mapper interface.
 * The interface itself is created by the Linux stack this just does the
 * IEEE 802.1p mapper specific initial configuration.
 */
static void ltq_pon_net_pmapper_setup(struct net_device *pmapper_ndev)
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);

	ether_setup(pmapper_ndev);

	pmapper_ndev->netdev_ops = &ltq_pon_net_pmapper_ops;
	pmapper_ndev->ethtool_ops = &ethtool_ops;
#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
	pmapper_ndev->destructor = free_netdev;
#else
	pmapper_ndev->priv_destructor = free_netdev;
#endif

	pmapper_ndev->needed_headroom = DP_MAX_PMAC_LEN;

	eth_hw_addr_random(pmapper_ndev);

	pmapper_ndev->features = NETIF_F_SG;
#if IS_ENABLED(CONFIG_QOS_TC)
	pmapper_ndev->features |= NETIF_F_HW_TC;
#endif

	pmapper_ndev->hw_features = pmapper_ndev->features;
	pmapper_ndev->vlan_features = pmapper_ndev->features;

	INIT_LIST_HEAD(&pmapper->node);
}

static struct ltq_pon_net_gem *
ltq_pon_net_pmapper_gem_find(struct ltq_pon_net_pmapper *pmapper, u8 gem_idx)
{
	int i;

	for (i = 0; i < DP_PMAP_PCP_NUM; i++) {
		if (pmapper->gem[i] && pmapper->gem[i]->gem_idx == gem_idx)
			return pmapper->gem[i];
	}

	return NULL;
}

static int ltq_pon_net_dp_gem_update(struct ltq_pon_net_gem *gem,
				     struct net_device *master)
{
	struct dp_subif_upd_info dp_subif = {0};
	struct device *dev = gem->hw->dev;
	int err;

	dp_subif.inst = dev->id;
	dp_subif.dp_port = gem->hw->port_id;
	dp_subif.subif = gem->gem_idx;
	if (master) {
		dp_subif.new_dev = master;
		dp_subif.new_ctp_dev = gem->ndev;
	} else {
		dp_subif.new_dev = gem->ndev;
		dp_subif.new_ctp_dev = NULL;
	}
	dp_subif.new_cqm_deq_idx = gem->tcont ? gem->tcont->qos_idx : 0;
	dp_subif.new_num_cqm_deq = 1;
	dp_subif.flags = DP_F_UPDATE_NO_Q_MAP;

	ltq_pon_net_dp_domain_upd(gem, &dp_subif);

	dev_dbg(dev, "dp_update_subif_info(inst: %d, port: %d, subif: %d, new_dev: %s, new_ctp_dev: %s, cqm_deq_idx: %d, num_cqm_deq: %d)\n",
		dp_subif.inst, dp_subif.dp_port, dp_subif.subif,
		dp_subif.new_dev ? dp_subif.new_dev->name : NULL,
		dp_subif.new_ctp_dev ? dp_subif.new_ctp_dev->name : NULL,
		dp_subif.new_cqm_deq_idx, dp_subif.new_num_cqm_deq);

	err = dp_update_subif_info(&dp_subif);
	if (err) {
		dev_err(dev, "dp_update_subif_info, err: %d\n", err);
		return err;
	}

	/* Update the device we used to register to dp */
	gem->dp_ndev = dp_subif.new_dev;

	return 0;
}

static int ltq_pon_net_pmapper_gem_add(struct ltq_pon_net_pmapper *pmapper,
				       int idx, struct ltq_pon_net_gem *gem)
{
	int err;

	if (WARN_ON(idx >= DP_PMAP_PCP_NUM))
		return -EINVAL;

	if (WARN_ON(pmapper->gem[idx]))
		return -ENODEV;

	if (WARN_ON(!gem))
		return -ENODEV;

	/* Register only the first GEM port from the DP library */
	if (ltq_pon_net_pmapper_gem_find(pmapper, gem->gem_idx)) {
		pmapper->gem[idx] = gem;
		return 0;
	}

	pmapper->gem[idx] = gem;

	err = ltq_pon_net_dp_gem_update(gem, pmapper->ndev);
	if (err)
		return err;

	gem->pmapper = pmapper;
	return 0;
}

static int ltq_pon_net_pmapper_gem_remove(struct ltq_pon_net_pmapper *pmapper,
					  int idx)
{
	struct ltq_pon_net_gem *gem;
	int err;

	if (WARN_ON(idx >= DP_PMAP_PCP_NUM))
		return -EINVAL;

	gem = pmapper->gem[idx];
	if (WARN_ON(!gem))
		return -ENODEV;

	pmapper->gem[idx] = NULL;
	dev_put(gem->ndev);

	/* Deregister only the last GEM port from the DP library */
	if (ltq_pon_net_pmapper_gem_find(pmapper, gem->gem_idx))
		return 0;

	err = ltq_pon_net_dp_gem_update(gem, NULL);
	if (err)
		return err;

	gem->pmapper = NULL;
	return 0;
}

/** Adds a mapping of a PCP value to a GEM port and also configures the default
 * PCP value to use for not VLAN tagged traffic.
 */
#if (KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE)
static int ltq_pon_net_pmapper_changelink(struct net_device *pmapper_ndev,
					  struct nlattr *tb[],
					  struct nlattr *data[])
#else
static int ltq_pon_net_pmapper_changelink(struct net_device *pmapper_ndev,
					  struct nlattr *tb[],
					  struct nlattr *data[],
					  struct netlink_ext_ack *extack)
#endif
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);
	struct ltq_pon_net_gem *gem = NULL;
	struct net_device *gem_ndev;
	const struct net_device_ops *netdev_ops;
	int i;
	int err;

	if (data[IFLA_PON_PMAPPER_MODE]) {
		u8 mode = nla_get_u8(data[IFLA_PON_PMAPPER_MODE]);

		switch (mode) {
		case DP_PMAP_PCP:
		case DP_PMAP_DSCP:
		case DP_PMAP_DSCP_ONLY:
			pmapper->mode = mode;
			break;
		default:
			netdev_err(pmapper_ndev, "Invalid pmapper mode: %i\n",
				   mode);
			return -EINVAL;
		}
	}

	if (data[IFLA_PON_PMAPPER_DSCP]) {
		struct nlattr *dscp[IFLA_PON_PMAPPER_DSCP_MAX + 1];

#if (KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE)
		err = nla_parse_nested(dscp, IFLA_PON_PMAPPER_DSCP_MAX,
				       data[IFLA_PON_PMAPPER_DSCP],
				       ltq_pon_net_pmapper_dscp_nl_policy);
#else
		err = nla_parse_nested(dscp, IFLA_PON_PMAPPER_DSCP_MAX,
				       data[IFLA_PON_PMAPPER_DSCP],
				       ltq_pon_net_pmapper_dscp_nl_policy,
				       extack);
#endif
		if (err < 0)
			return err;

		for (i = 0; i < DP_PMAP_DSCP_NUM; i++) {
			struct nlattr *nl_attr;

			nl_attr = dscp[IFLA_PON_PMAPPER_DSCP_0 + i];
			if (!nl_attr)
				continue;

			pmapper->dscp_map[i] = nla_get_u8(nl_attr);
		}
	}

	for (i = 0; i < DP_PMAP_PCP_NUM; i++) {
		u32 gem_ndev_idx;

		if (!data[IFLA_PON_PMAPPER_PCP_0 + i])
			continue;

		gem_ndev_idx = nla_get_u32(data[IFLA_PON_PMAPPER_PCP_0 + i]);

		if (gem_ndev_idx) {
			gem_ndev = dev_get_by_index(pmapper->src_net,
						    gem_ndev_idx);
			if (!gem_ndev) {
				netdev_err(pmapper_ndev, "Invalid/missing gem_ndev!\n");
				return -ENODEV;
			}

			/* check if this is a GEM port device */
			netdev_ops = gem_ndev->netdev_ops;
			if (netdev_ops->ndo_open != &ltq_pon_net_gem_open) {
				dev_put(gem_ndev);
				netdev_err(pmapper_ndev, "Added interface (%s) is not a GEM\n",
						gem_ndev ?
						netdev_name(gem_ndev) : NULL);
				return -ENODEV;
			}
			gem = netdev_priv(gem_ndev);

			/* If this is the same gem port just skip it */
			if (gem == pmapper->gem[i]) {
				dev_put(gem_ndev);
				continue;
			}
		}

		if (pmapper->gem[i]) {
			err = ltq_pon_net_pmapper_gem_remove(pmapper, i);
			if (err)
				return err;
		}
		if (gem_ndev_idx) {
			err = ltq_pon_net_pmapper_gem_add(pmapper, i, gem);
			if (err)
				return err;
		}
	}

	if (data[IFLA_PON_PMAPPER_PCP_DEF])
		pmapper->pcpdef = nla_get_u8(data[IFLA_PON_PMAPPER_PCP_DEF]);

	for (i = 0; i < DP_PMAP_PCP_NUM; i++) {
		if (pmapper->gem[i])
			pmapper->subif[i] = pmapper->gem[i]->gem_idx;
		else
			pmapper->subif[i] = DP_PMAPPER_DISCARD_CTP;
	}
	pmapper->subif_def = pmapper->subif[pmapper->pcpdef];

	return ltq_pon_net_pmapper_set_mapping(pmapper);
}

/** Creates a new IEEE 802.1p mapper on the interface given in IFLA_LINK. */
#if (KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE)
static int ltq_pon_net_pmapper_newlink(struct net *src_net,
				       struct net_device *pmapper_ndev,
				       struct nlattr *tb[],
				       struct nlattr *data[])
#else
static int ltq_pon_net_pmapper_newlink(struct net *src_net,
				       struct net_device *pmapper_ndev,
				       struct nlattr *tb[],
				       struct nlattr *data[],
				       struct netlink_ext_ack *extack)
#endif
{
	struct ltq_pon_net_pmapper *pmapper = netdev_priv(pmapper_ndev);
	struct ltq_pon_net_hw *hw;
	struct net_device *hw_ndev;
	int err;
	int i;

	if (!tb[IFLA_LINK]) {
		netdev_err(pmapper_ndev, "Missing IFLA_LINK!\n");
		return -EINVAL;
	}
	hw_ndev = dev_get_by_index(src_net, nla_get_u32(tb[IFLA_LINK]));
	if (!hw_ndev) {
		netdev_err(pmapper_ndev, "Invalid hw_ndev\n");
		return -ENODEV;
	}

	/* check if this is a pon master device */
	if (hw_ndev->netdev_ops->ndo_open != &ltq_pon_net_mdev_open) {
		netdev_err(pmapper_ndev, "Invalid pmapper device != &ltq_pon_net_mdev_open\n");
		err = -ENODEV;
		goto err_put_dev;
	}

	hw = netdev_priv(hw_ndev);
	pmapper->hw = hw;
	pmapper->ndev = pmapper_ndev;
	pmapper->src_net = src_net;
	pmapper->mode = DP_PMAP_PCP;
	for (i = 0; i < DP_PMAP_DSCP_NUM; i++)
		pmapper->dscp_map[i] = PON_PMAPPER_DSCP_DEFAULT;

	if (hw->dp_mode_registered != DP_F_GPON) {
		netdev_err(pmapper_ndev, "system not in ITU PON mode!\n");
		err = -ENOTSUPP;
		goto err_put_dev;
	}

	list_add(&pmapper->node, &hw->pmapper_list);

	err = register_netdevice(pmapper_ndev);
	if (err)
		goto err_list_remove;

	return 0;

err_list_remove:
	list_del(&pmapper->node);
err_put_dev:
	dev_put(hw_ndev);

	return err;
}

/**
 * Deletes one IEEE 802.1p mapper, deregistration from dp library is done
 * in the uninit callback later.
 */
static void ltq_pon_net_pmapper_dellink(struct net_device *pmapper_ndev,
					struct list_head *head)
{
	unregister_netdevice_queue(pmapper_ndev, head);
}

/** Rtnl ops to create and delete IEEE 802.1p mapper.
 */
static struct rtnl_link_ops ltq_pon_net_pmapper_rtnl __read_mostly = {
	.kind		= "pmapper",
	.priv_size	= sizeof(struct ltq_pon_net_pmapper),

	.get_size	= ltq_pon_net_pmapper_nl_getsize,
	.policy		= ltq_pon_net_pmapper_nl_policy,
	.fill_info	= ltq_pon_net_pmapper_fill_info,

	.newlink	= ltq_pon_net_pmapper_newlink,
	.dellink	= ltq_pon_net_pmapper_dellink,
	.changelink	= ltq_pon_net_pmapper_changelink,
	.setup		= ltq_pon_net_pmapper_setup,
	.maxtype	= IFLA_PON_PMAPPER_MAX,
};

static void ltq_pon_net_mdev_unregister(struct net_device *hw_ndev)
{
	struct ltq_pon_net_hw *hw = netdev_priv(hw_ndev);
	struct ltq_pon_net_pmapper *pmapper;
	struct ltq_pon_net_tcont *tcont;
	struct ltq_pon_net_gem *gem;
	struct ltq_pon_net_iphost *iphost;
	struct list_head *ele, *next;
	LIST_HEAD(list);

	list_for_each_safe(ele, next, &hw->tcont_list) {
		tcont = list_entry(ele, struct ltq_pon_net_tcont, node);
		ltq_pon_net_tcont_dellink(tcont->ndev, &list);
	}

	list_for_each_safe(ele, next, &hw->pmapper_list) {
		pmapper = list_entry(ele, struct ltq_pon_net_pmapper, node);
		ltq_pon_net_pmapper_dellink(pmapper->ndev, &list);
	}

	list_for_each_safe(ele, next, &hw->gem_list) {
		gem = list_entry(ele, struct ltq_pon_net_gem, node);
		ltq_pon_net_gem_dellink(gem->ndev, &list);
	}

	list_for_each_safe(ele, next, &hw->iphost_list) {
		iphost = list_entry(ele, struct ltq_pon_net_iphost, node);
		iphost_dellink(iphost->ndev, &list);
	}

	unregister_netdevice_many(&list);
}

/** Listener which checks if the PON IP hardware interface was unregistered to
 * also remove all the other interfaces.
 */
static int ltq_pon_net_device_event(struct notifier_block *unused,
				    unsigned long event,
				    void *ptr)
{
	struct net_device *ndev = netdev_notifier_info_to_dev(ptr);
	const struct net_device_ops *netdev_ops;

	switch (event) {
	case NETDEV_UNREGISTER:

		/* twiddle thumbs on netns device moves */
		if (ndev->reg_state != NETREG_UNREGISTERING)
			break;

		netdev_ops = ndev->netdev_ops;

		if (netdev_ops->ndo_open == &ltq_pon_net_mdev_open)
			ltq_pon_net_mdev_unregister(ndev);

		break;
	}

	return NOTIFY_DONE;
}

static struct ltq_pon_net_tcont *ltq_pon_net_tcont_by_alloc_id_get(
								u32 alloc_id)
{
	struct ltq_pon_net_tcont *tcont;
	struct list_head *ele;

	list_for_each(ele, &g_hw->tcont_list) {
		tcont = list_entry(ele, struct ltq_pon_net_tcont, node);
		if (tcont->alloc_id == alloc_id)
			return tcont;
	}

	return NULL;
}

static void ltq_pon_net_tcont_gems_activate(struct ltq_pon_net_tcont *tcont)
{
	struct ltq_pon_net_gem *gem;
	struct list_head *ele;

	list_for_each(ele, &tcont->gem_list) {
		gem = list_entry(ele, struct ltq_pon_net_gem, tcont_node);

		if (!gem->alloc_link_ref)
			ltq_pon_net_gem_fw_apply(gem, tcont);
	}
}

static void ltq_pon_net_tcont_gems_deactivate(struct ltq_pon_net_tcont *tcont)
{
	struct ltq_pon_net_gem *gem;
	struct list_head *ele;

	list_for_each(ele, &tcont->gem_list) {
		gem = list_entry(ele, struct ltq_pon_net_gem, tcont_node);
		gem->alloc_link_ref = 0;
	}
}

static void ltq_pon_net_alloc_id_reactivate(struct ltq_pon_net_tcont *tcont)
{
	int err;

	err = ltq_pon_net_qos_idx_link_set(tcont->ndev);
	if (err)
		return;

	ltq_pon_net_tcont_gems_activate(tcont);
}

static void ltq_pon_net_alloc_id_link_event(char *msg, size_t msg_len)
{
	struct ponfw_alloc_id_link *fw_alloc =
					(struct ponfw_alloc_id_link *)msg;
	struct ltq_pon_net_tcont *tcont;

	if (msg_len != sizeof(*fw_alloc)) {
		pr_err("ALLOC_ID_LINK event: wrong message size!\n");
		return;
	}

	if (!g_hw) {
		pr_err("ALLOC_ID_LINK event: PON master interface not available!\n");
		return;
	}

	rtnl_lock();

	pr_debug("pon_eth: received ALLOC_ID_LINK event, alloc_idx: %i, alloc_id: %i, alloc_link_ref: 0x%x, qos_idx: %i\n",
		 fw_alloc->alloc_idx, fw_alloc->alloc_id,
		 fw_alloc->alloc_link_ref, fw_alloc->qos_idx);

	if (fw_alloc->alloc_idx && fw_alloc->qos_idx)
		g_alloc_idx_to_qos_idx[fw_alloc->alloc_idx] = fw_alloc->qos_idx;

	tcont = ltq_pon_net_tcont_by_alloc_id_get(fw_alloc->alloc_id);
	if (!tcont) {
		pr_debug("pon_eth: tcont netdev for alloc_id %i does not exist (yet)\n",
			 fw_alloc->alloc_id);
		rtnl_unlock();
		return;
	}

	/** Attempt to reactivate US connections by requesting QOS index link
	 *  with allocation ID again after PLOAM Allocation ID activation event.
	 */
	if (fw_alloc->link_status == PONFW_ALLOC_ID_LINK_LINK_STATUS_ASSIGNED) {
		ltq_pon_net_alloc_id_reactivate(tcont);
		rtnl_unlock();
		return;
	}

	tcont->alloc_link_ref = fw_alloc->alloc_link_ref;

	ltq_pon_net_tcont_gems_activate(tcont);

	alloc_id_write_update(tcont->alloc_idx);

	pr_debug("ALLOC_ID_LINK event: Alloc idx (%u) is added to counters storage\n",
		 tcont->alloc_idx);

	rtnl_unlock();
}

static void ltq_pon_net_alloc_id_clear_all(void)
{
	struct ltq_pon_net_tcont *tcont;
	struct list_head *ele;

	list_for_each(ele, &g_hw->tcont_list) {
		tcont = list_entry(ele, struct ltq_pon_net_tcont, node);
		ltq_pon_net_qos_idx_unlink(tcont->ndev, NULL);
		ltq_pon_net_tcont_gems_deactivate(tcont);
	}
}

static void ltq_pon_net_alloc_id_unlink_event(char *msg, size_t msg_len,
					      u32 seq)
{
	struct ponfw_alloc_id_unlink *fw_alloc =
					(struct ponfw_alloc_id_unlink *)msg;
	struct ltq_pon_net_tcont *tcont;
	int err;

	if (msg_len != sizeof(*fw_alloc)) {
		pr_err("ALLOC_ID_UNLINK event: wrong message size!\n");
		return;
	}

	if (!g_hw) {
		pr_err("ALLOC_ID_UNLINK event: PON master interface not available!\n");
		return;
	}

	err = pon_mbox_send_ack(PONFW_ALLOC_ID_UNLINK_CMD_ID, PONFW_WRITE, seq);
	if (err < 0) {
		pr_err("ALLOC_ID_UNLINK event: failed to send acknowledgment\n");
		return;
	}

	rtnl_lock();

	if (fw_alloc->all) {
		ltq_pon_net_alloc_id_clear_all();
		rtnl_unlock();
		return;
	}

	tcont = ltq_pon_net_tcont_by_alloc_id_get(fw_alloc->alloc_id);
	if (!tcont) {
		/* When we get an unlink event from the PON FW for a T-CONT
		 * which was not configured over OMCI we still have to ack it
		 * to the PON FW, so the PON FW does not block this index
		 * forever. It looks like under some conditions we get a
		 * ALLOC ID link and then unlink event over PLOAM without
		 * any OMCI configuration for this T-CONT in between.
		 */
		pr_warn("pon_eth: received unexpected ALLOC_ID_UNLINK event alloc_id: %i, alloc_link_ref: 0x%x\n",
			fw_alloc->alloc_id, fw_alloc->alloc_link_ref);
		ltq_pon_net_ack_alloc_unlink(fw_alloc);
		rtnl_unlock();
		return;
	}

	ltq_pon_net_qos_idx_unlink(tcont->ndev, fw_alloc);

	ltq_pon_net_tcont_gems_deactivate(tcont);

	rtnl_unlock();
}

static void ltq_pon_net_o5_tconts_activate(void)
{
	struct ltq_pon_net_tcont *tcont;
	struct list_head *ele;
	int err;

	list_for_each(ele, &g_hw->tcont_list) {
		tcont = list_entry(ele, struct ltq_pon_net_tcont, node);
		if (tcont->tcont_omci)
			continue;

		err = ltq_pon_net_qos_idx_link_set(tcont->ndev);
		if (err)
			continue;
	}
}

static void ltq_pon_net_o5_gems_activate(void)
{
	struct ltq_pon_net_gem *gem;
	struct list_head *ele;
	int err;

	list_for_each(ele, &g_hw->gem_list) {
		gem = list_entry(ele, struct ltq_pon_net_gem, node);

		err = ltq_pon_net_gem_fw_apply(gem, gem->tcont);
		if (err)
			continue;

		netif_carrier_on(gem->ndev);
		gem->valid = true;
	}
}

static void ltq_pon_net_ploam_state_event(char *msg, size_t msg_len)
{
	struct ponfw_ploam_state *ploam = (struct ponfw_ploam_state *)msg;
	struct ltq_pon_net_gem *gem;
	struct list_head *ele;
	bool in_o5;

	if (msg_len != sizeof(*ploam)) {
		pr_err("PLOAM state event: wrong message size!\n");
		return;
	}

	if (!g_hw) {
		pr_err("PLOAM state event: PON master interface not available!\n");
		return;
	}

	rtnl_lock();
	/* The system is in O50, O51 O52 or O60 */
	in_o5 = (ploam->ploam_act >= 50 && ploam->ploam_act <= 60);

	/* We dropped out of O5 */
	if (g_hw->in_o5 && !in_o5) {
		list_for_each(ele, &g_hw->gem_list) {
			gem = list_entry(ele, struct ltq_pon_net_gem, node);
			gem->valid = false;
			netif_carrier_off(gem->ndev);
		}
		g_hw->in_o5 = in_o5;
	}

	/* We just entered O5 */
	if (!g_hw->in_o5 && in_o5) {
		g_hw->in_o5 = in_o5;
		ltq_pon_net_o5_tconts_activate();
		ltq_pon_net_o5_gems_activate();
	}
	rtnl_unlock();
}

/*
 * Listen for PON mode changes and register the driver to DP in ITU PON or
 * in IEEE PON mode. We do not support switching between ITU and IEEE
 * PON Mode, this would need to delete all interfaces.
 */
static int ltq_pon_net_mode_event(enum pon_mode mode)
{
	int ret;

	/* No pon0 device registered e.g. AON mode, skip */
	if (!g_hw)
		return 0;

	rtnl_lock();
	ret = ltq_pon_net_register_dp_port(g_hw, mode);
	rtnl_unlock();

	return ret;
}

static struct notifier_block ltq_pon_net_notifier_block __read_mostly = {
	.notifier_call = ltq_pon_net_device_event,
};

static const struct ltq_pon_net_soc_data prx300_data = {
	.pmac_remove = true,
};

/* FIXME: Replace code name by product name */
static const struct ltq_pon_net_soc_data lgm_data = {
	.pmac_remove = false,
};

static const struct of_device_id ltq_pon_net_match[] = {
	{ .compatible = "lantiq,falcon-mountain-pon", .data = &prx300_data },
	{ .compatible = "intel,prx300-pon", .data = &prx300_data },
	{ .compatible = "intel,lgm-pon", .data = &lgm_data },
	{},
};
MODULE_DEVICE_TABLE(of, ltq_pon_net_match);

static struct platform_driver ltq_pon_net_driver = {
	.probe = ltq_pon_net_probe,
	.remove = ltq_pon_net_remove,
	.driver = {
		.name = "prx300-pon",
		.of_match_table = ltq_pon_net_match,
	},
};

static int __init ltq_pon_net_driver_init(void)
{
	int ret;

	ret = rtnl_link_register(&pon_net_pon_cfg_rtnl);
	if (ret)
		return ret;

	ret = rtnl_link_register(&ltq_pon_net_pmapper_rtnl);
	if (ret)
		goto err_pon_rtnl;

	ret = rtnl_link_register(&ltq_pon_net_tcont_rtnl);
	if (ret)
		goto err_pmapper_rtnl;

	ret = rtnl_link_register(&ltq_pon_net_gem_rtnl);
	if (ret)
		goto err_tcont_rtnl;

	ret = pon_eth_iphost_rtnl_link_register();
	if (ret)
		goto err_gem_rtnl;

	ret = register_netdevice_notifier(&ltq_pon_net_notifier_block);
	if (ret < 0)
		goto err_iphost_rtnl;

	pon_mbox_ploam_state_callback_func_register(
					ltq_pon_net_ploam_state_event);
	pon_mbox_alloc_id_link_callback_register(
					ltq_pon_net_alloc_id_link_event);
	pon_mbox_alloc_id_unlink_callback_register(
					ltq_pon_net_alloc_id_unlink_event);

	pon_mbox_mode_callback_register(ltq_pon_net_mode_event);

	ret = platform_driver_register(&ltq_pon_net_driver);
	if (ret)
		goto err_notifier;

	return 0;

err_notifier:
	pon_mbox_mode_callback_register(NULL);
	pon_mbox_alloc_id_unlink_callback_register(NULL);
	pon_mbox_alloc_id_link_callback_register(NULL);
	pon_mbox_ploam_state_callback_func_register(NULL);
	unregister_netdevice_notifier(&ltq_pon_net_notifier_block);
err_iphost_rtnl:
	pon_eth_iphost_rtnl_link_unregister();
err_gem_rtnl:
	rtnl_link_unregister(&ltq_pon_net_gem_rtnl);
err_tcont_rtnl:
	rtnl_link_unregister(&ltq_pon_net_tcont_rtnl);
err_pmapper_rtnl:
	rtnl_link_unregister(&ltq_pon_net_pmapper_rtnl);
err_pon_rtnl:
	rtnl_link_unregister(&pon_net_pon_cfg_rtnl);

	return ret;
}
module_init(ltq_pon_net_driver_init);

static void __exit ltq_pon_net_driver_exit(void)
{
	pon_mbox_mode_callback_register(NULL);
	pon_mbox_alloc_id_unlink_callback_register(NULL);
	pon_mbox_alloc_id_link_callback_register(NULL);
	pon_mbox_ploam_state_callback_func_register(NULL);
	unregister_netdevice_notifier(&ltq_pon_net_notifier_block);
	rtnl_link_unregister(&ltq_pon_net_gem_rtnl);
	rtnl_link_unregister(&ltq_pon_net_tcont_rtnl);
	rtnl_link_unregister(&ltq_pon_net_pmapper_rtnl);
	pon_eth_iphost_rtnl_link_unregister();

	/*
	 * The cleanup routines (struct rtnl_link_ops->dellink()), often access
	 * struct ltq_pon_net_hw through hw pointer, therefore we must
	 * unregister the PON IP hardware after cleanup was done, so that
	 * elements of struct ltq_pon_net_hw are still valid.
	 */
	platform_driver_unregister(&ltq_pon_net_driver);

	/*
	 * We must call this after platform_driver_unregister(), because
	 * platform_driver_unregister() removes "pon" device. If we don't call
	 * it after, then the rtnl_link_unregister() will loop over all devices
	 * of given type and call struct rtnl_link_ops->dellink(), and it is not
	 * implemented for pon device. Therefore we must call
	 * rtnl_link_unregister() when no pon device exists - i.e. after
	 * platform_driver_unregister()
	 */
	rtnl_link_unregister(&pon_net_pon_cfg_rtnl);
}
module_exit(ltq_pon_net_driver_exit);

MODULE_DESCRIPTION("MaxLinear Pon network driver");
MODULE_LICENSE("GPL");
