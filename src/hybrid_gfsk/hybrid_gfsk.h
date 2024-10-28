/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __HYBRID_GFSK_H__
#define __HYBRID_GFSK_H__

#if defined(gAppEnableHybridGenfsk_d) && (gAppEnableHybridGenfsk_d!=0)

#include "EmbeddedTypes.h"

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * \brief Enable or disable hybrid GENFSK module
 *
 */
void hybrid_gfsk_init(void);

/*!
 * \brief Set GENFSK to the default configuration
 *
 */
void hybrid_gfsk_set_default_config(void);

#endif // defined(gAppEnableHybridGenfsk_d) && (gAppEnableHybridGenfsk_d!=0)

#endif // __HYBRID_GFSK_H__