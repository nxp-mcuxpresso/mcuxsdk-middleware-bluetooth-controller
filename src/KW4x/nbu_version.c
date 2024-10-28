/*
 * Copyright 2021-2024 NXP
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "EmbeddedTypes.h"
#include "sw_version.h"
#include "fwk_platform_ics.h"


/* NBU build version  */
/*! \cond DOXY_SKIP_TAG */
#define gNbuVerMajor_c   1
#define gNbuVerMinor_c   9
#define gNbuVerPatch_c   18
#define gNbuVerBuildNo_c 0
#define gNbuModuleId_c   0x11
/*! \endcond */

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
    const NbuInfo_t nbu_version =  { .versionNumber = {versionNoMajor, versionNoMinor, versionNoPatch, versionNoBuild}, \
                                     .repo_digest = {VERSION_SHA},\
                                     .repo_tag = {VERSION_TAG},\
                                     .variant = {VERSION_VARIANT}, \
                                     .build_type = {VERSION_BUILD_TYPE}, \
                                     .versionBuildNo = versionNoBuild}
#elif defined(__GNUC__)
#define RegisterNbuInfo(versionNoMajor, versionNoMinor, versionNoPatch, versionNoBuild) \
    const NbuInfo_t nbu_version __attribute__((section (".NBU_VERSION_TAGS"), used)) \
      = { .versionNumber = {versionNoMajor, versionNoMinor, versionNoPatch, versionNoBuild}, 
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
