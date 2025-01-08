/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (c) 2020 - 2021 MaxLinear, Inc.
 *  Copyright (C) 2018 - 2020 Intel Corporation.
 *
 * The driver implements a PTP device (multiple instances available) for
 * PON Time-Of-Day (TOD) clock.
 * Since the ToD clock is fully controlled by the OLT from the PON side,
 * it has limited capabilities as a PTP clock. The only available functions
 * are TIME_GET and EXTTS. All other standard PTP functions return the
 * -ENOTSUPP ("Not supported") error code.
 * Multiple instances of the /dev/ptp devices are possible as defined in
 * devicetree configuration. All of them are identical and can be used to
 * provide multiple EXTTS sources for user applications.
 */

#include <linux/device.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>

#include <pon/pon_ip_msg.h>
#include <pon/pon_mbox_ikm.h>


/* GPON/XGPON1/XGSPON/NGPON2:
 *       31104 clocks == 100000 ns
 *       The greatest common divisor is 32.
 *       => 972 clocks == 3125 ns.
 */
#define TOD_CLOCKS_TO_NS(c) (((c) * 3125) / 972)
#define TOD_NS_TO_CLOCKS(c) (((c) * 972) / 3125)

#define PPS_TIME_EN  1
#define PPS_TIME_DIS 0

#define INVALID_EXTTS_SYNCE 0xFFFFFFFFFFFFFFFF
#define INVALID_EXTTS_NOSYNCE 0xFFFFFFFFFFFFFFFE

struct pon_ptp_priv {
	/* Linux platform device */
	struct platform_device *pdev;
	/* Linux device */
	struct device *dev;
	/* PTP clock device associated */
	struct ptp_clock *ptp_clock;
	/* PTP clock capabilities */
	struct ptp_clock_info ptp_clk_info;
	/* EXTTS generation switch */
	int extts_enable;
	/* EXTTS event struct */
	struct ptp_clock_event event;
	/* List head for PTP device instances */
	struct list_head ptp_inst;
};

/* A list of created PTP device instances */
LIST_HEAD(ptp_list);

/* PTP clock name index counter */
static int ptp_instance;

/* PON firmware status */
static bool fw_ready;

/*
 * time_get PTP function
 * Returns a snapshot value of a running ToD clock.
 */
static int pon_ptp_time_get(struct ptp_clock_info *ptp,
				struct timespec64 *ts64)
{
	int err;
	struct ponfw_onu_tod_sync fw_tods_out = {0};

	err = pon_mbox_send(PONFW_ONU_TOD_SYNC_CMD_ID, PONFW_READ,
				NULL, 0,
				&fw_tods_out, sizeof(fw_tods_out));
	if (err < 0)
		goto err_dev;

	if (err != sizeof(fw_tods_out)) {
		err = -EINVAL;
		goto err_dev;
	}

	/* Seconds and hundreds microseconds are taken from the
	 * message without change.
	 * Nanoseconds 0 to 99999 are calculated from TOD clock ticks.
	 */
	ts64->tv_sec = (time64_t)fw_tods_out.tod_sec;
	ts64->tv_nsec =
		TOD_CLOCKS_TO_NS(fw_tods_out.tod_clocks) +
				(fw_tods_out.tod_micro * 100000);
	return 0;

err_dev:
	return err;
}

/*
 * time_set function not supported
 */
static int pon_ptp_time_set(struct ptp_clock_info *ptp,
				const struct timespec64 *ts64)
{
	return -EOPNOTSUPP;
}

/*
 * time_adjust function not supported
 */
static int pon_ptp_time_adjust(struct ptp_clock_info *ptp, int64_t delta)
{
	return -EOPNOTSUPP;
}

/*
 * crosstimestamp_get function not supported
 */
static int pon_ptp_cts_get(struct ptp_clock_info *ptp,
			      struct system_device_crosststamp *cts)
{
	return -EOPNOTSUPP;
}

/*
 * verify function not supported
 */
static int pon_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
		      enum ptp_pin_function func, unsigned int chan)
{
	return -EOPNOTSUPP;
}

/*
 * freq_adjust function not supported
 */
static int pon_ptp_freq_adjust(struct ptp_clock_info *ptp, int32_t delta)
{
	/* The PON PTP does not support frequency adjustment.
	 * 0 is returned as a workaround to suppress phc2sys warnings.
	 */
	return 0;
}

/*
 * Get SyncE status of the system.
 */
static int pon_ptp_synce_status_get(uint32_t *synce_stat)
{
	int err;
	struct ponfw_synce_status fw_synce_stat = {0};

	err = pon_mbox_send(PONFW_SYNCE_STATUS_CMD_ID, PONFW_READ,
			NULL, 0,
			&fw_synce_stat, sizeof(fw_synce_stat));
	if (err < 0)
		return err;
	if (err != sizeof(fw_synce_stat)) {
		err = -EINVAL;
		return err;
	}
	*synce_stat = fw_synce_stat.synce_stat;
	return 0;
}

/*
 * This function enables/disables automatic generation of
 * TOD_SYNC message by the PONIP on every 1PPS event.
 * TOD_SYNC message is used for EXTTS event generation.
 */
static int pon_ptp_monitor_config(int en)
{
	int err;
	struct ponfw_monitor_config fw_mon_cfg = {0};

	/* Read the current MONITOR_CONFIG value */
	err = pon_mbox_send(PONFW_MONITOR_CONFIG_CMD_ID, PONFW_READ,
			NULL, 0,
			&fw_mon_cfg, sizeof(fw_mon_cfg));
	if (err < 0)
		return err;
	if (err != sizeof(fw_mon_cfg)) {
		err = -EINVAL;
		return err;
	}

	/* Change it, if necessary*/
	if (fw_mon_cfg.pps_time != en) {
		fw_mon_cfg.pps_time = en;

		err = pon_mbox_send(PONFW_MONITOR_CONFIG_CMD_ID, PONFW_WRITE,
				&fw_mon_cfg, sizeof(fw_mon_cfg),
				NULL, 0);
		if (err < 0)
			return err;
	}
	return 0;
}

/*
 * This function is called by an user application to
 * start or stop EXTTS event generation.
 */
static int pon_ptp_enable(struct ptp_clock_info *ptp,
		      struct ptp_clock_request *rq, int on)
{
	struct pon_ptp_priv *ctx;
	int err;

	list_for_each_entry(ctx, &ptp_list, ptp_inst) {
		if (&ctx->ptp_clk_info == ptp) {
			switch (rq->type) {
			case PTP_CLK_REQ_EXTTS:
				ctx->extts_enable = on;
				if (!fw_ready)
					break;
				err = pon_ptp_monitor_config(PPS_TIME_EN);
				if (err) {
					dev_err(ctx->dev, "Activating pps time report failed: %i\n",
					       err);
					return err;
				}
				break;
			default:
				return -EOPNOTSUPP;
			}
		}
	}
	return 0;
}

/*
 * TDD PTP device capabilities declaration struct.
 */
static const struct ptp_clock_info pon_ptp_clock_info = {
	.owner		= THIS_MODULE,
	/* max_adj has to be differ from 0, otherwise the phc2sys tool cannot be
	 * used to synchronize clocks.
	 */
	.max_adj	= 1,
	.n_alarm	= 0,
	.n_ext_ts	= 1,
	.n_per_out	= 0,
	.pps		= 0,
	.adjfreq	= pon_ptp_freq_adjust,
	.adjtime	= pon_ptp_time_adjust,
	.gettime64	= pon_ptp_time_get,
	.settime64	= pon_ptp_time_set,
	.getcrosststamp = pon_ptp_cts_get,
	.enable		= pon_ptp_enable,
	.verify     = pon_ptp_verify,
};

/*
 * This function generates an EXTTS event on every
 * TOD_SYNC message automatically generated by the PONIP.
 * This function is registered as a callback at
 * pon_mbox driver to intercept TOD_SYNC messages.
 */
static void pon_ptp_extts_event(char *msg, size_t msg_len)
{
	struct pon_ptp_priv *ctx;
	struct ponfw_onu_tod_sync *fw_tods_out;
	uint32_t synce_stat;

	if (msg_len == sizeof(struct ponfw_onu_tod_sync)) {
		fw_tods_out = (struct ponfw_onu_tod_sync *)msg;

		if (pon_ptp_synce_status_get(&synce_stat) != 0) {
			pr_err("Cannot get SyncE status. Assume OFF.\n");
			synce_stat = 0;
		}

		list_for_each_entry(ctx, &ptp_list, ptp_inst) {
			if (!ctx->extts_enable)
				continue;

			/* Prepare external timestamp event */
			ctx->event.type = PTP_CLOCK_EXTTS;
			/* If tod_quality == 1, the time is traceable
			 * and EXTTS event contains a valid actual timestamp.
			 */
			if (fw_tods_out->tod_quality) {
				ctx->event.timestamp =
					(u64)fw_tods_out->tod_sec *
					(u64)NSEC_PER_SEC;
			} else {
				/* If tod_quality == 0, the system has lost
				 * time source and EXTTS events are filled with
				 * special fixed values used to control
				 * phc2sys daemon behavior:
				 * If SyncE is disabled:
				 *   INVALID_EXTTS_NOSYNCE
				 *   phc2sys maintains previous
				 *   frequency adjustment value,
				 * If SyncE is enabled:
				 *   INVALID_EXTTS_SYNCE
				 *   phc2sys sets frequency
				 *   adjustment value to 0.
				 */
				if (synce_stat)
					ctx->event.timestamp =
						INVALID_EXTTS_SYNCE;
				else
					ctx->event.timestamp =
						INVALID_EXTTS_NOSYNCE;
			}
			ctx->event.index = 0;

			/* Fire event */
			ptp_clock_event(ctx->ptp_clock, &ctx->event);
		}
	}
}

/*
 * This function is registered as callback for ploam state changes.
 * It is used to track the initial start of the firmware to do configurations,
 * which are only possible with a running firmware.
 */
static void pon_ptp_psc_event(char *msg, size_t msg_len)
{
	struct ponfw_ploam_state *ploam = (struct ponfw_ploam_state *)msg;
	int err;

	if (msg_len != sizeof(*ploam))
		return;

	/* The previous PLOAM state 0 indicates that PON firmware is ready. */
	if (ploam->ploam_prev == 0) {
		err = pon_ptp_monitor_config(PPS_TIME_EN);
		if (err) {
			pr_err("Activating pps time report failed: %i\n",
			       err);
			return;
		}
		fw_ready = true;
	}
}

static int pon_ptp_probe(struct platform_device *pdev)
{
	struct pon_ptp_priv *ctx;
	struct device *dev = &pdev->dev;

	/* Allocate memory for private PTP device data.
	 */
	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->pdev = pdev;
	ctx->dev = dev;
	platform_set_drvdata(pdev, ctx);

	ctx->ptp_clock = NULL;
	memcpy(&ctx->ptp_clk_info, &pon_ptp_clock_info,
			sizeof(pon_ptp_clock_info));
	snprintf(ctx->ptp_clk_info.name, sizeof(ctx->ptp_clk_info.name),
			"PONIP TOD %d", ++ptp_instance);

	ctx->ptp_clock = ptp_clock_register(&ctx->ptp_clk_info, NULL);
	if (IS_ERR(ctx->ptp_clock)) {
		ctx->ptp_clock = NULL;
		dev_err(dev, "%s PTP clock register failed\n",
			ctx->ptp_clk_info.name);
	} else {
		dev_dbg(dev, "Registered %s PTP clock\n",
			ctx->ptp_clk_info.name);
	}

	/* Add created PTP instance to the list. */
	INIT_LIST_HEAD(&ctx->ptp_inst);
	list_add(&ctx->ptp_inst, &ptp_list);

	return 0;
}

static int pon_ptp_remove(struct platform_device *pdev)
{
	struct pon_ptp_priv *ctx = platform_get_drvdata(pdev);

	/* Disable automatic generation of TOD_SYNC messages
	 * by the PONIP.
	 */
	pon_ptp_monitor_config(PPS_TIME_DIS);

	/* Unregister the PTP clock */
	if (ctx->ptp_clock) {
		ptp_clock_unregister(ctx->ptp_clock);
		dev_dbg(&pdev->dev, "Removed %s PTP clock\n",
			ctx->ptp_clk_info.name);
		ctx->ptp_clock = NULL;
	}
	list_del(&ctx->ptp_inst);

	return 0;
}

static const struct of_device_id pon_ptp_match[] = {
	{ .compatible = "intel,falcon-mountain-pon-ptp" },
	{ .compatible = "intel,prx300-pon-ptp" },
	{ .compatible = "intel,urx800-pon-ptp" },
	{},
};
MODULE_DEVICE_TABLE(of, pon_ptp_match);

static struct platform_driver pon_ptp_driver = {
	.probe = pon_ptp_probe,
	.remove = pon_ptp_remove,
	.driver = {
		.name = "pon_ptp_drv",
		.of_match_table = pon_ptp_match,
	},
};

static int __init pon_ptp_driver_init(void)
{
	int ret;

	ptp_instance = 0;
	fw_ready = false;

	/* Register a callback function at pon_mbox driver for
	 * TOD_SYNC automessages from PONIP with 1PPS timestamps.
	 */
	pon_mbox_pps_callback_register(pon_ptp_extts_event);
	pon_mbox_pps_psc_callback_register(pon_ptp_psc_event);

	ret = platform_driver_register(&pon_ptp_driver);
	return ret;
}

module_init(pon_ptp_driver_init);

static void __exit pon_ptp_driver_exit(void)
{
	/* Unregister the callback function at pon_mbox driver for
	 * TOD_SYNC automessages from PONIP with 1PPS timestamps.
	 */
	pon_mbox_pps_callback_register((void *)NULL);
	pon_mbox_pps_psc_callback_register((void *)NULL);

	platform_driver_unregister(&pon_ptp_driver);
}
module_exit(pon_ptp_driver_exit);

MODULE_DESCRIPTION("MaxLinear PON PTP driver");
MODULE_LICENSE("GPL");
