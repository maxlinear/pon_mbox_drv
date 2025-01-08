/******************************************************************************
 *
 * Copyright (c) 2022 - 2024 MaxLinear, Inc.
 * Copyright (c) 2018 - 2020 Intel Corporation
 *
 * For licensing information, see the file 'LICENSE' in the root folder of
 * this software module.
 *
 ******************************************************************************/

/**
 * \file pon_mbox_ikm.h
 *
 */

#ifndef __PON_MBOX_IKM_H
#define __PON_MBOX_IKM_H

#include <stddef.h>
#include <linux/skbuff.h>

/** \addtogroup PON_MBOX_REFERENCE
 *  @{
 */

/** The message was not acknowledged by the firmware */
#define PON_STATUS_FW_NACK	1009
/** NACK, duplicate entry was rejected */
#define PON_STATUS_FW_DUP_ERR	1013
/** NACK from firmware, because debug is disabled */
#define PON_STATUS_FW_DBG	1014
/** NACK from firmware, because the PON IP FW is in wrong state */
#define PON_STATUS_FW_STATE	1026

/** PON operation modes.
 */
enum pon_mode {
	/** The PON operation mode is unknown. */
	PON_MODE_UNKNOWN = 0,
	/** The PON operation mode is ITU-T G.984 (GPON). */
	PON_MODE_984_GPON = 1,
	/** The PON operation mode is ITU-T G.987 (XG-PON). */
	PON_MODE_987_XGPON = 2,
	/** The PON operation mode is ITU-T G.9807 (XGS-PON). */
	PON_MODE_9807_XGSPON = 3,
	/** The PON operation mode is ITU-T G.989 (NG-PON2 2.5G upstream). */
	PON_MODE_989_NGPON2_2G5 = 4,
	/** The PON operation mode is ITU-T G.989 (NG-PON2 10G upstream). */
	PON_MODE_989_NGPON2_10G = 5,
	/** The AON operation mode. */
	PON_MODE_AON = 9,
};

/** PON datapath flags
 *
 * These are settings configured in device-tree and provided via the pon-mbox
 * driver to userspace.
 */
struct pon_dp_flags {
	/** The datapath expects the FCS in rx packets */
	bool with_rx_fcs;
	/** The datapath expects the FCS in tx packets */
	bool with_tx_fcs;
	/** The datapath expects no timestamp in rx packets */
	bool without_timestamp;
};

/**
 * @brief Send message using pon_mbox, wait for response (blocking).
 *
 * @param cmd_id PON FW command ID
 * @param rw selects read/write (1 - read, 0 write)
 * @param input input data (NULL, if no input is needed)
 * @param input_size input data size (in bytes)
 * @param output output data pointer (NULL, if no output is expected)
 * @param output_size output data size
 *
 * @return Returns the size of the actually received data on success in bytes.
 *	   In case of an error a negative error code is returned.
 *	   - E...			The normal Unix error codes for
 *					general problems
 *	   - PON_STATUS_FW_NACK:		The FW returned a NACK
 *	   - PON_STATUS_FW_DUP_ERR:	The FW returned a NACK_DUP
 *	   - PON_STATUS_FW_DBG:		The FW returned a NACK_DEBUG
 *	   - PON_STATUS_FW_STATE:	The FW returned a NACK_STATE
 */
ssize_t pon_mbox_send(unsigned int cmd_id, unsigned int rw,
		      void *input, size_t input_size,
		      void *output, size_t output_size);

/**
 * @brief Send ack message using pon_mbox.
 *
 * @param cmd_id PON FW command ID
 * @param rw selects read/write (1 - read, 0 write)
 * @param seq sequence number of message received
 *
 * @return Returns 0 if operation succeeded.
 *	   In case of an error a negative error code is returned.
 *	   - E...			The normal Unix error codes for
 *					general problems
 *	   - PON_STATUS_FW_NACK:	The FW returned a NACK
 *	   - PON_STATUS_FW_DUP_ERR:	The FW returned a NACK_DUP
 *	   - PON_STATUS_FW_DBG:		The FW returned a NACK_DEBUG
 *	   - PON_STATUS_FW_STATE:	The FW returned a NACK_STATE
 */
ssize_t pon_mbox_send_ack(unsigned int cmd_id, unsigned int rw, u32 seq);

/**
 * @brief Register a callback function of PTP driver to forward 1PPS timestamps.
 *
 * @param func A pointer to the function consuming 1PPS time stamps.
 */
void pon_mbox_pps_callback_register(void(*func)(char *msg, size_t msg_len));

/**
 * @brief Register a callback function of PTP driver to enable events.
 *
 * @param func A pointer to the function enabling events.
 */
void pon_mbox_pps_psc_callback_register(void(*func)(char *msg, size_t msg_len));

/**
 * @brief Register a callback function of ETH driver to take an action on
 *	  ploam state change,
 *
 * @param func A pointer to the ETH driver function.
 */
void pon_mbox_ploam_state_callback_func_register(void(*func)
					(char *msg, size_t msg_len));

/**
 * @brief Register a callback function of ETH driver to take an action on
 *	  allocation link.
 *
 * @param func A pointer to the ETH driver function.
 */
void pon_mbox_alloc_id_link_callback_register(void(*func)
					(char *msg, size_t msg_len));


/**
 * @brief Register a callback function of ETH driver to take an action on
 *	  allocation unlink.
 *
 * @param func A pointer to the ETH driver function.
 */
void pon_mbox_alloc_id_unlink_callback_register(void(*func)
					(char *msg, size_t msg_len, u32 seq));

/**
 * @brief Register a callback function of ETH driver to take an action
 *	  when the pon mode is changed.
 *
 * @param func A pointer to the ETH driver function.
 */
void pon_mbox_mode_callback_register(int(*func)(enum pon_mode));

/**
 * @brief Returns the Linux interface index for the GEM port of the given skb.
 * This only works for skbs received on a pmapper.
 *
 * @param skb Pointer to skb to work on
 *
 * @return
 *    Positive number: Linux Interface index of GEM port
 *    -ENXIO: Not part of a pmapper or can not find GEM port in pmapper
 *    -ENOENT: no private data
 */
int ltq_pon_net_gem_get(struct sk_buff *skb);

/**
 * @brief Save the datapath flags read from device-tree to the mbox driver
 *
 * @param dp_flags Pointer to the \ref pon_dp_flags structure.
 */
void pon_mbox_save_pon_dp_flags(const struct pon_dp_flags *dp_flags);

/**
 * @brief Counters update function for GEM_PORT_ID firmware message.
 *
 * @param[in] gem_port_idx GEM Port Index
 */
void gem_port_id_write_update(u8 gem_port_idx);

/**
 * @brief Counters update function for GEM_PORT_ID_REMOVE firmware message.
 *
 * @param[in] gem_port_idx GEM Port Index
 */
void gem_port_id_remove_update(u8 gem_port_idx);

/**
 * @brief Counters update function for ALLOC_ID firmware message.
 *
 * @param[in] alloc_idx    Alloc Index
 */
void alloc_id_write_update(u8 alloc_idx);

/**
 * @brief Counters update function for ALLOC_ID_REMOVE firmware message.
 *
 * @param[in] alloc_idx    Alloc Index
 */
void alloc_id_remove_update(u8 alloc_idx);

/**
 * @brief Returns the current PON mode
 *
 * @return PON mode
 */
enum pon_mode pon_mbox_get_pon_mode(void);

/**
 * @brief Read from SFP EEPROM
 *
 * @param[in]  a2     Select the EEPROM (if true, use addr 0x51, otherwise 0x50)
 * @param[in]  addr   Offset in the EEPROM to start reading
 * @param[out] buf    Buffer for the result
 * @param[in]  len    Length to read
 *
 * @return int	Negative error codes, 0 for success
 */
int pon_sfp_read(bool a2, u8 addr, void *buf, size_t len);

/**
 * @brief Write to SFP EEPROM
 *
 * @param[in] a2     Select the EEPROM (if true, use addr 0x51, otherwise 0x50)
 * @param[in] addr   Offset in the EEPROM to start writing
 * @param[in] buf    Buffer with data to write
 * @param[in] len    Length to write
 *
 * @return int	Negative error codes, 0 for success
 */
int pon_sfp_write(bool a2, u8 addr, void *buf, size_t len);

/** @} */

#endif /* __PON_MBOX_IKM_H */
