/*
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/ethtool.h>
#include <linux/phy.h>

#include "bcm-amac-ethtool.h"
#include "bcm-amac-enet.h"
#include "bcm-amac-core.h"
#include "bcm-amac-regs.h"

#define MOD_NAME		"AMAC"
#define MOD_VERSION		"3.0.0"

#define ETH_GSTRING_LEN    32

#define REG_64_BITS         64
#define REG_32_BITS         32

struct ethtool_keys {
	const char string[ETH_GSTRING_LEN];
};

struct bcm_ethtool_mib {
	/* Warning: do not change the names of the following members
	 * as they are used in macro expansion to formulate
	 * corresponding data routines.
	 */
	u64 port_num;
	u64 tx_octets;
	u64 tx_drop_pkts;
	u64 tx_broadcast_pkts;
	u64 tx_multicast_pkts;
	u64 tx_unicast_pkts;
	u64 tx_collisions;
	u64 tx_single_collision;
	u64 tx_multiple_collision;
	u64 tx_deferred_txmit;
	u64 tx_late_collision;
	u64 tx_excessive_collision;
	u64 tx_frame_in_disc;
	u64 tx_pause_pkts;
	u64 rx_octets;
	u64 rx_undersize_pkts;
	u64 rx_pause_pkts;
	u64 rx_pkts64octets;
	u64 rx_pkts65to127octets;
	u64 rx_pkts128to255octets;
	u64 rx_pkts256to511octets;
	u64 rx_pkts512to1023octets;
	u64 rx_pkts1024tomaxpktoctets;
	u64 rx_oversize_pkts;
	u64 rx_jabbers;
	u64 rx_alignment_errors;
	u64 rx_fcs_errors;
	u64 rx_good_octets;
	u64 rx_drop_pkts;
	u64 rx_unicast_pkts;
	u64 rx_multicast_pkts;
	u64 rx_broadcast_pkts;
	u64 rx_sa_changes;
	u64 rx_fragments;
	u64 rx_jumbo_pktcount;
	u64 rx_symbol_error;
	u64 rx_discard;
};

/* Derrived from 'struct bcm_ethtool_mib' */
static const struct ethtool_keys ethtool_stats_keys[] = {
	{"                    Port"},
	{"                TxOctets"},
	{"              TxDropPkts"},
	{"         TxBroadcastPkts"},
	{"         TxMulticastPkts"},
	{"           TxUnicastPkts"},
	{"            TxCollisions"},
	{"       TxSingleCollision"},
	{"     TxMultipleCollision"},
	{"      TxDeferredTransmit"},
	{"         TxLateCollision"},
	{"    TxExcessiveCollision"},
	{"           TxFrameInDisc"},
	{"             TxPausePkts"},
	{"                RxOctets"},
	{"         RxUndersizePkts"},
	{"             RxPausePkts"},
	{"          RxPkts64Octets"},
	{"     RxPkts65To127Octets"},
	{"    RxPkts128To255Octets"},
	{"    RxPkts256To511Octets"},
	{"   RxPkts512To1023Octets"},
	{"RxPkts1024ToMaxPktOctets"},
	{"          RxOversizePkts"},
	{"               RxJabbers"},
	{"       RxAlignmentErrors"},
	{"             RxFcsErrors"},
	{"            RxGoodOctets"},
	{"              RxDropPkts"},
	{"           RxUnicastPkts"},
	{"         RxMulticastPkts"},
	{"         RxBroadcastPkts"},
	{"             RxSaChanges"},
	{"             RxFragments"},
	{"         RxJumboPktCount"},
	{"           RxSymbolError"},
	{"               RxDiscard"}
};

/* Derrived from 'struct amac_ethtool_stats'
 * The ordering needs to be in sync with
 * 'struct amac_ethtool_stats'
 */
static const struct ethtool_keys amac_ethtool_keys[] = {
	{"rx_packets"},
	{"tx_packets"},
	{"rx_bytes"},
	{"tx_bytes"},
	{"rx_dropped"},
	{"tx_dropped"},
	{"multicast"},
	{"rx_length_errors"},
	{"rx_fifo_errors"},
	{"tx_fifo_errors"},
};

/* Do not change the names of the following members
 * as they are used in macro expansion to formulate
 * corresponding data routines.
 */
struct amac_ethtool_stats {
	u64 rx_packets;
	u64 tx_packets;
	u64 rx_bytes;
	u64 tx_bytes;
	u64 rx_dropped;
	u64 tx_dropped;
	u64 multicast;
	u64 rx_length_errors;
	u64 rx_fifo_errors;
	u64 tx_fifo_errors;
};

static int amac_ethtool_get_settings(struct net_device *ndev,
				     struct ethtool_cmd *cmd)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	u32 speed;

	if (!IS_UNIMAC_PATH) {
		speed = get_speed(privp);
		switch (speed) {
		case SPEED_10G:
			speed = SPEED_10000;
			break;
		case SPEED_2_5G:
			speed = SPEED_2500;
			break;
		case SPEED_1G:
			speed = SPEED_1000;
			break;
		default:
			speed = SPEED_10000;
			break;
		}
		cmd->supported = ADVERTISED_10000baseT_Full |
					ADVERTISED_2500baseX_Full |
					ADVERTISED_1000baseT_Full;
		cmd->supported |= SUPPORTED_Autoneg;
		cmd->supported |= SUPPORTED_FIBRE;

		cmd->advertising = cmd->supported;
		cmd->advertising |= ADVERTISED_Autoneg;
		cmd->advertising |= ADVERTISED_FIBRE;

		cmd->autoneg = AUTONEG_DISABLE;

		cmd->lp_advertising = cmd->supported;
		cmd->duplex = DUPLEX_FULL;
		ethtool_cmd_speed_set(cmd, speed);

		cmd->port = PORT_FIBRE;
		cmd->transceiver = XCVR_INTERNAL;
		return 0;
	}

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	return phy_ethtool_gset(privp->port.ext_port.phydev, cmd);
}

int amac_ethtool_set_settings(struct net_device *ndev,
			      struct ethtool_cmd *cmd)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	return phy_ethtool_sset(privp->port.ext_port.phydev, cmd);
}

static void amac_ethtool_get_drvinfo(struct net_device *ndev,
				     struct ethtool_drvinfo *info)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	strcpy(info->driver, "amac-enet");
	strcpy(info->version, MOD_VERSION);
	strcpy(info->fw_version, "N/A");

	if (privp->port.ext_port.phydev) {
		if (privp->port.ext_port.phydev->mdio.bus)
			strcpy(info->bus_info,
			       privp->port.ext_port.phydev->mdio.bus->name);
	} else {
		strcpy(info->bus_info, "N/A");
	}
}

static int amac_ethtool_nway_reset(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	if (!netif_running(ndev))
		return -EAGAIN;

	return phy_start_aneg(privp->port.ext_port.phydev);
}

static void amac_ethtool_get_strings(struct net_device *ndev,
				     u32 stringset, u8 *buf)
{
	if (stringset == ETH_SS_STATS)
		memcpy(buf, &amac_ethtool_keys,
		       sizeof(amac_ethtool_keys));
}

static int amac_ethtool_get_sset_count(struct net_device *ndev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return ARRAY_SIZE(amac_ethtool_keys);
}

static void amac_ethtool_get_stats(struct net_device *ndev,
				   struct ethtool_stats *estats, u64 *tmp_stats)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	struct amac_ethtool_stats *stats =
		(struct amac_ethtool_stats *)tmp_stats;

	stats->rx_packets = privp->ndev->stats.rx_packets;
	stats->tx_packets = privp->ndev->stats.tx_packets;
	stats->rx_bytes = privp->ndev->stats.rx_bytes;
	stats->tx_bytes = privp->ndev->stats.tx_bytes;
	stats->rx_dropped = privp->ndev->stats.rx_dropped;
	stats->tx_dropped = privp->ndev->stats.tx_dropped;
	stats->multicast = privp->ndev->stats.multicast;
	stats->rx_length_errors = privp->ndev->stats.rx_length_errors;
	stats->rx_fifo_errors = privp->ndev->stats.rx_fifo_errors;
	stats->tx_fifo_errors = privp->ndev->stats.tx_fifo_errors;
}

void amac_ethtool_get_ringparam(struct net_device *ndev,
				struct ethtool_ringparam *ering)
{
	(void)ndev;

	ering->rx_max_pending = DMA_RX_DESC_NUM;
	ering->rx_mini_max_pending = 0;
	ering->rx_jumbo_max_pending = 0;
	ering->tx_max_pending = AMAC_DMA_TX_MAX_QUEUE_LEN;

	ering->rx_pending = DMA_RX_DESC_NUM;
	ering->rx_mini_pending = 0;
	ering->rx_jumbo_pending = 0;
	ering->tx_pending = AMAC_DMA_TX_MAX_QUEUE_LEN;
}

void amac_ethtool_get_pauseparam(struct net_device *ndev,
				 struct ethtool_pauseparam *epause)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	epause->autoneg = privp->port.ext_port.stat.aneg;
	epause->rx_pause = privp->port.ext_port.stat.pause;
	epause->tx_pause = privp->port.ext_port.stat.pause;
}

static const struct ethtool_ops amac_ethtool_ops = {
	.get_settings = amac_ethtool_get_settings,
	.set_settings = amac_ethtool_set_settings,
	.get_drvinfo = amac_ethtool_get_drvinfo,
	.nway_reset = amac_ethtool_nway_reset,
	.get_pauseparam = amac_ethtool_get_pauseparam,
	.get_strings = amac_ethtool_get_strings,
	.get_ethtool_stats = amac_ethtool_get_stats,
	.get_sset_count = amac_ethtool_get_sset_count,
	.get_link = ethtool_op_get_link,
	.get_ringparam = amac_ethtool_get_ringparam
};

void bcm_amac_set_ethtool_ops(struct net_device *ndev)
{
	ndev->ethtool_ops = &amac_ethtool_ops;
}
