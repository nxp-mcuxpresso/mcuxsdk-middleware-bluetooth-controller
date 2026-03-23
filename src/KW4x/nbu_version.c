/*
 * Copyright 2021-2024 NXP
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "EmbeddedTypes.h"
#include "sw_version.h"
#include "fwk_platform_ics.h"
#include "ll_types.h"
#include "controller_hal.h"

/* NBU build version  */
/*! \cond DOXY_SKIP_TAG */
#define gNbuVerMajor_c   1
#define gNbuVerMinor_c   9
#define gNbuVerPatch_c   34
#define gNbuVerBuildNo_c 0
#define gNbuModuleId_c   0x11
/*! \endcond */

/* =============================================================================
 * Version String
 * =============================================================================
 * RC_STR is constructed from individual components defined by the build system.
 * Format: "RC: <commit>, <user>, <date>, <time>"
 * Example: "RC: 8db3b63c, unknown, 2026/04/01, 10:28:21"
 * =============================================================================
 */
#ifndef COMMIT_INFO
#define COMMIT_INFO ""
#endif

#ifndef BUILD_USER
#define BUILD_USER ""
#endif

#ifndef BUILD_DATE
#define BUILD_DATE ""
#endif

#ifndef BUILD_TIME
#define BUILD_TIME ""
#endif

#define RC_STR "RC: " COMMIT_INFO ", " BUILD_USER ", " BUILD_DATE ", " BUILD_TIME

#define gNbuVerString_c "NBU BLE v" \
                        QUH(gNbuVerMajor_c) "." \
                        QUH(gNbuVerMinor_c) "." \
                        QUH(gNbuVerPatch_c)

/*! \cond DOXY_SKIP_TAG */

/*! \endcond */


#if defined(__IAR_SYSTEMS_ICC__)
#define RegisterNbuInfo(versionNoMajor, versionNoMinor, versionNoPatch, versionNoBuild) \
    _Pragma("location=\".NBU_VERSION_TAGS\"") __root \
    const NbuInfo_t nbu_version = { .versionNumber = {versionNoMajor, versionNoMinor, versionNoPatch}, \
                                    .repo_digest = {VERSION_SHA},\
                                    .repo_tag = {VERSION_TAG}, \
                                    .variant = {VERSION_VARIANT}, \
                                    .build_type = {VERSION_BUILD_TYPE}, \
                                    .versionBuildNo = versionNoBuild}
#elif defined(__CC_ARM)
#define RegisterNbuInfo(versionNoMajor, versionNoMinor, versionNoPatch, versionNoBuild) \
    const NbuInfo_t nbu_version =  { .versionNumber = {versionNoMajor, versionNoMinor, versionNoPatch}, \
                                     .repo_digest = {VERSION_SHA},\
                                     .repo_tag = {VERSION_TAG},\
                                     .variant = {VERSION_VARIANT}, \
                                     .build_type = {VERSION_BUILD_TYPE}, \
                                     .versionBuildNo = versionNoBuild}
#elif defined(__GNUC__)
#define RegisterNbuInfo(versionNoMajor, versionNoMinor, versionNoPatch, versionNoBuild) \
    const NbuInfo_t nbu_version =  { .versionNumber = {versionNoMajor, versionNoMinor, versionNoPatch}, \
                                     .repo_digest = {VERSION_SHA},\
                                     .repo_tag = {VERSION_TAG},\
                                     .variant = {VERSION_VARIANT}, \
                                     .build_type = {VERSION_BUILD_TYPE}, \
                                     .versionBuildNo = versionNoBuild}
#else
#define RegisterNbuInfo(versionNoMajor, versionNoMinor, versionNoPatch, versionNoBuild) \
    const NbuInfo_t nbu_version = {
          .versionNumber = {{versionNoMajor, versionNoMinor, versionNoPatch, versionNoBuild},
          .repo_digest = {VERSION_SHA},\
          .repo_tag = {VERSION_TAG},\
          .variant = {VERSION_VARIANT}, \, 
          .build_type = {VERSION_BUILD_TYPE}, \
          .versionBuildNo = versionNoBuild}
   #warning Unknown/undefined toolchain!
#endif

RegisterNbuInfo(gNbuVerMajor_c, gNbuVerMinor_c, gNbuVerPatch_c, gNbuVerBuildNo_c); /* DO NOT MODIFY */

/* Strings made in the build process */
const char revCtrlStr[RC_STR_LENGTH] = RC_STR;

void NbuGetVersion(uint8_t *output)
{
  output[0] = nbu_version.versionNumber[0];
  output[1] = nbu_version.versionNumber[1];
  output[2] = nbu_version.versionNumber[2];
  output[3] = nbu_version.versionBuildNo;
}
