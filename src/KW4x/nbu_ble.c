/*
 * Copyright 2020-2024 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_component_mem_manager.h"
#include "fwk_platform.h"
#include "fwk_platform_ble.h"
#include "fwk_platform_ics.h"
#include "fwk_platform_lowpower.h"
#include "fwk_debug.h"
#include "fwk_rf_sfc.h"
#include "board.h"
#include "nxp2p4_xcvr.h"
#include "fwk_platform_sensors.h"
#include "hci_transport.h"
#include "controller_init.h"

#if defined(gNbu_Hadm_d) && (gNbu_Hadm_d==1)
#include "lcl_hadm_measurement.h"
#endif

#ifdef SIMULATOR
#include "nbu_testcases.h"
#endif

#if defined(gUseIpcTransport_d) && (gUseIpcTransport_d == 1)
#include "ipc.h"
#endif

#if defined(SDK_OS_FREE_RTOS)
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PACKET_INFO_QUEUE_SIZE  8    //Number has to be a power of 2

typedef struct hci_pkt_info_tag
{
    void*               pPacket;
    uint16_t            packetSize;
    hciPacketType_t     packetType;
} hci_pkt_info_t;


#ifdef __COVERAGESCANNER__

#define OPEN_APPEND_CMD_PACKET_TYPE             5U
#define PUTS_CMD_PACKET_TYPE                    6U
#define CLOSE_CMD_PACKET_TYPE                   7U
#define MALLOC_FAILURE_PACKET_TYPE              8U

#define HCI_VENDOR_COV_DATA_WRITE_OPCODE        0xFDF1U
#define HCI_VENDOR_COV_SET_TESTNAME_OPCODE      0xFDF2U
#define HCI_VENDOR_COV_SET_FILENAME_OPCODE      0xFDF3U

#define MAX_NAME_LENGTH                         40U

#define __SECTION(x) __attribute__((section(x)))

#endif /*__COVERAGESCANNER__*/

#if defined(HDI_MODE) && (HDI_MODE == 1)
/* Tie phy switch to PTD3 */
#define PHY_SWITCH_IO (3u)
#endif /*defined(HDI_MODE) && (HDI_MODE == 1)*/

#if defined(gNbu_Hadm_d) && (gNbu_Hadm_d==1)
#define CS_HANDOFF_ENABLED           0U // 0: no, 1: event, 2: procedure
#endif /*defined(gNbu_Hadm_d) && (gNbu_Hadm_d==1)*/

#if defined(CS_HANDOFF_ENABLED) && (CS_HANDOFF_ENABLED != 0)

/* event config pointer */
#if CS_HANDOFF_ENABLED==1
#define CS_HANDOFF_TYPE               BLE_HADM_EventConfig_t
#elif CS_HANDOFF_ENABLED==2
#define CS_HANDOFF_TYPE               const TBleHadmConnection_t
#else
  #errro CS_HANDOFF_ENABLED invalid
#endif
#endif /*defined(CS_HANDOFF_ENABLED) && (CS_HANDOFF_ENABLED != 0)*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if defined(PHY_15_4_ENABLED) && (PHY_15_4_ENABLED == 1)
void init_15_4_Phy(void);
#endif
static void NbuHci_SendPktToHost(unsigned long packetType, void *pPacket, unsigned short packetSize);
#if defined(gUseIpcTransport_d) && (gUseIpcTransport_d == 1)
static void NbuHdi_SendChannelSwitchCmd(unsigned short channel);
#endif
#if defined(HDI_MODE) && (HDI_MODE == 1)
static void NBU_SetPhy(unsigned char rate);
#endif
static bool_t nbu_tasks_init_done = FALSE;
static bool_t isHighZ = FALSE; /*For peak power reduction feature.*/
/*osa start_task*/
void start_task(void *argument);

#if defined(SDK_OS_FREE_RTOS)
void vApplicationStackOverflowHook( TaskHandle_t pxTask,
                                    char * pcTaskName );
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationTickHook( void );
void vFullDemoIdleFunction( void );
void vFullDemoTickHookFunction( void );
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize );
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize );
#endif /*defined(SDK_OS_FREE_RTOS)*/

#ifdef __COVERAGESCANNER__
int   csfputs(const char *s, void *stream);
void *csfopenappend(const char *path);
int   csfclose(void *fp);

#endif /*__COVERAGESCANNER__*/

/*******************************************************************************
 * Private memory declarations
 ******************************************************************************/
static hci_pkt_info_t hciPacketInfo[PACKET_INFO_QUEUE_SIZE];
volatile static uint8_t pendingPktInfo  = 0;
static uint8_t readPktInfoIdx           = 0;
static uint8_t writePktInfoIdx          = 0;
static uint8_t nbrPacketInfoSkipped     = 0; /* for debug */

/* Definition missing from NBU libs
   ThreadX requires this two variables to be set to know the location */
uint32_t *      _tx_initialize_low_level_ptr;
uint32_t *      tx_application_define_ptr;

#ifdef SIMULATOR
uint8 test_HostHciPacket[256] = {0};
#endif

#if defined(gUseIpcTransport_d) && (gUseIpcTransport_d == 1)
const static hal_rpmsg_config_t ipcRpmsgConfig = {
    .local_addr    = 50,
    .remote_addr   = 60,
};

RPMSG_HANDLE_DEFINE(s_IpcRpmsgHandle);

#endif /* gUseIpcTransport_d */

const nbuIntf_t nbuInterface = {
    .nbuHciIntf = NbuHci_SendPktToHost,
#if defined(gUseIpcTransport_d) && (gUseIpcTransport_d == 1)
    .nbuChannelSwitchIntf = NbuHdi_SendChannelSwitchCmd,
#else
    .nbuChannelSwitchIntf = NULL,
#endif
#if defined(gDbg_Enabled_d) && (gDbg_Enabled_d == 1)
    .nbuDbgIoSet = NbuDbg_IO_Set,
#else
    .nbuDbgIoSet = NULL,
#endif /* gDbg_Enabled_d */
#if defined(HDI_MODE) && (HDI_MODE == 1)
    .nbuPhySwitchIntf = NBU_SetPhy,
#else
    .nbuPhySwitchIntf = NULL,
#endif
    .nbuEnterCritical = OSA_InterruptDisable,
    .nbuExitCritical = OSA_InterruptEnable
};

#if defined(SDK_OS_FREE_RTOS)
static StaticTask_t xIdleTaskTCB;
static StaticTask_t xTimerTaskTCB;

static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];
#endif /*defined(SDK_OS_FREE_RTOS)*/

#ifdef __COVERAGESCANNER__
volatile bool __SECTION(".coverage") is_buffer_freed;
static uint32_t stream = 0;
#endif /*__COVERAGESCANNER__*/

#if CS_HANDOFF_ENABLED
CS_HANDOFF_TYPE *p_hadm_config = NULL;

/* event start time in LL timing */
uint32_t ulStartTimeHSlot;
uint16_t ulStartTimeOffsetUs;

/* event start time in TSTMR0 value */
uint64_t ullStartTimeTsTmr;

/* structure used to copy the event config. As the RPMSG payload size is limited,
   the config copy is segmented */
typedef PACKED_STRUCT
{
  uint8_t  packet_type;         /* 0x04: ACL data */
  uint16_t handle;              /* 0xFFFx: invalid handle for config identification, x = segment number */
  uint16_t data_length;         /* payload length in this message */
  uint64_t start_time;          /* event start time for TSTMR0 */
  uint16_t config_size;         /* event config size */
  uint16_t config_offset;       /* starting offset of the config being copied */
  uint16_t copied_size;         /* copied config size */
  uint8_t  config[RL_BUFFER_PAYLOAD_SIZE-32]; /* partial config copied */
} EventConfig_t;
EventConfig_t sHciConfig;
#endif /*CS_HANDOFF_ENABLED*/

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef __COVERAGESCANNER__
int csfputs(const char *s, void *stream)
{
  int return_value = 0;
  uint16_t len = strlen(s);
  is_buffer_freed = FALSE;
  uint16_t status = Hcit_SendPacket((hciPacketType_t)PUTS_CMD_PACKET_TYPE, (char*)s, len);
  while (!is_buffer_freed);
  if (status != 0)
  {
    return_value = -1;
  }
  return return_value;
}

void *csfopenappend(const char *path)
{
  is_buffer_freed = FALSE;
  uint16_t status = Hcit_SendPacket((hciPacketType_t)OPEN_APPEND_CMD_PACKET_TYPE, "OPEN_APPEND_CMD:" , strlen("OPEN_APPEND_CMD:")+1);
  while (!is_buffer_freed);
  is_buffer_freed = FALSE;
  status |= Hcit_SendPacket((hciPacketType_t)OPEN_APPEND_CMD_PACKET_TYPE, (void*)path , strlen(path)+1);
  while (!is_buffer_freed);
  if (status == 0)
  {
    return &stream;
  }
  else
  {
    return NULL;
  }
}

int csfclose(void *fp)
{
  is_buffer_freed = FALSE;
  uint16_t status = Hcit_SendPacket((hciPacketType_t)CLOSE_CMD_PACKET_TYPE, "CLOSE_CMD" , strlen("CLOSE_CMD")+1);
  while (!is_buffer_freed);
  if (status != 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

void malloc_failure(int size)
{
  is_buffer_freed = FALSE;
  Hcit_SendPacket((hciPacketType_t)MALLOC_FAILURE_PACKET_TYPE, &size , sizeof(size));
  while (!is_buffer_freed);
  while(TRUE);
}

void handle_coverage_hci_command(hciPacketType_t type, void* packet)
{
  if (type == gHciCommandPacket_c)
  {
    uint8_t* ptr = (uint8_t*)packet;
    uint16_t opcode = (ptr[1] << 8U) | ptr[0];
    if (opcode == HCI_VENDOR_COV_DATA_WRITE_OPCODE)
    {
      //Save coverage data
      /* Let Coco instrumentation know the test is finished */
      __coveragescanner_save();
    }
    else if ((opcode == HCI_VENDOR_COV_SET_TESTNAME_OPCODE) || (opcode == HCI_VENDOR_COV_SET_FILENAME_OPCODE))
    {
      //Save test name
      static char name[MAX_NAME_LENGTH] = {0};
      uint8_t length_total = ptr[2];
      if (length_total < MAX_NAME_LENGTH)
      {
        uint8_t length = ptr[3];
        memcpy(name, &ptr[4], length);
        /*Fill with backspace*/
        memset(name+length-1,0x20,MAX_NAME_LENGTH-length-1U+1U);
        name[MAX_NAME_LENGTH-1U]=0;
      }
      else
      {
        memcpy(name, &ptr[4], MAX_NAME_LENGTH-1U);
        name[MAX_NAME_LENGTH-1U]=0;
      }

      if (opcode == HCI_VENDOR_COV_SET_TESTNAME_OPCODE)
      {
        __coveragescanner_testname(name);
      }
      else
      {
        __coveragescanner_filename(name);
      }
    }
    else
    {
      //Do nothing specific
    }
  }
}
#endif /*__COVERAGESCANNER__*/

#if defined(HDI_MODE) && (HDI_MODE == 1)
/* Tie phy switch to PTD3 */
#define PHY_SWITCH_IO (3u)
static void NBU_InitPhySwitch(void)
{
  GPIO_Type *gpiod = GPIOD;

  /* Switch direction to output */
  gpiod->PDDR |= 1u << PHY_SWITCH_IO;
  /* Clear output (1Mbps) */
  gpiod->PCOR = 1u << PHY_SWITCH_IO;
}

/* This function is used for 1/2Mbps phy switch */
static void NBU_SetPhy(unsigned char rate)
{
  GPIO_Type *gpiod = GPIOD;

  if (rate == 1)
  {
    /* 2Mbps */
    RADIO_CTRL->FPGA_CTRL |= RADIO_CTRL_FPGA_CTRL_DATA_RATE_SEL_MASK;
    gpiod->PSOR = 1u << PHY_SWITCH_IO;
  }
  else
  {
    /* 1Mbps + LR */
    RADIO_CTRL->FPGA_CTRL &= ~RADIO_CTRL_FPGA_CTRL_DATA_RATE_SEL_MASK;
    gpiod->PCOR = 1u << PHY_SWITCH_IO;
  }
}
#endif

void Hcit_PktReceived(hciPacketType_t type, void* packet, uint16_t size)
{
    PWR_DBG_LOG("Rcv PKT type=%d pkt=%x sz=%d", type, packet, size);
    /* delay processing of HCI commands into idle task as not all NBU tasks are initialized */
    if (nbu_tasks_init_done == TRUE)
    {
#ifdef __COVERAGESCANNER__
      OSA_InterruptDisable();
      handle_coverage_hci_command(type, packet);
      OSA_InterruptEnable();
#endif /*__COVERAGESCANNER__*/
      NbuHci_SendPktToController(type, packet, size);
    }
    else
    {
      /* We are in interrupt context, we can't directly send the packet to the LL
         or we will have issue with ThreadX
         So, we store the packet and wait for Idle to send it */
      uint8_t *pPacketBuffer = MEM_BufferAlloc((uint32_t)size);
      if (pPacketBuffer != NULL)
      {
          FLib_MemCpy(pPacketBuffer, (uint8_t*)packet, size);
      }
      else
      {
         //ERROR: Out of memory
         nbrPacketInfoSkipped++;
      }

      if (pendingPktInfo >= PACKET_INFO_QUEUE_SIZE)
      {
         //ERROR: Message will be lost
         nbrPacketInfoSkipped++;
      }
      if (nbrPacketInfoSkipped > 0)
      {
          assert(0);
          return;
      }
      hciPacketInfo[writePktInfoIdx].packetType = type;
      hciPacketInfo[writePktInfoIdx].pPacket = pPacketBuffer;
      hciPacketInfo[writePktInfoIdx].packetSize = size;
      writePktInfoIdx = (writePktInfoIdx+1)&(PACKET_INFO_QUEUE_SIZE-1);  //modulo PACKET_INFO_QUEUE_SIZE
      pendingPktInfo++;
    }
}

static void NbuHci_SendPktToHost(unsigned long packetType, void *pPacket, unsigned short packetSize)
{
    /* Send packet from CM3 to CM33 over IMU */
#ifdef SIMULATOR
    FLib_MemCpy(test_HostHciPacket, pPacket, packetSize);
#ifdef SIMULATOR_PRINTF
    printf("HCI: --> ");
    for (int i = 0; i < packetSize; i++)
    {
        printf("%02X ", test_HostHciPacket[i]);
    }
    printf("\n");
#endif
#else
    PWR_DBG_LOG("Send PKT type=%d pkt=%x sz=%d", (int)packetType, pPacket, (uint16_t)packetSize);
    Hcit_SendPacket((hciPacketType_t)packetType, pPacket, (uint16_t)packetSize);
#endif
}

#if defined(gUseIpcTransport_d) && (gUseIpcTransport_d == 1)
static void NbuHdi_SendChannelSwitchCmd(unsigned short channel)
{
    Ipc_SendPacket((uint8_t*)&channel, 1U);
}
#endif

#if CS_HANDOFF_ENABLED

/* Callback from link-layer */
#if CS_HANDOFF_ENABLED==1
BLE_HADM_STATUS_t BLE_HADM_SubeventContinue(BLE_HADM_SubeventConfig_t *pConfig, uint32 startTimeHSlot, uint32 startTimeOffsetUs)
#elif CS_HANDOFF_ENABLED==2
BLE_HADM_STATUS_t BLE_HADM_ProcedureContinue(const TBleHadmConnection_t *pConfig, uint32 startTimeHSlot, uint32 startTimeOffsetUs)
#endif
{
    /* save LL CS event start time */
    ulStartTimeHSlot = startTimeHSlot;
    ulStartTimeOffsetUs = (uint16_t)startTimeOffsetUs;

    /* get event config pointer */
    p_hadm_config = pConfig;

    /* The event config is updated after the return of the call, so it is not available yet.
       Return HADM_HAL_ABORTED to request CS proceudre abort in the LL. */
    return HADM_HAL_ABORTED;
}

static void  NBU_HADM_Init(void)
{
}

/* Sample code to send the config to the app core:
    CS Event config size: 1228
      02 00 F0 DE 01 95 64 8A 00 00 00 00 00 CC 04 00 00 D0 01   00 00 00 00 03 00 00 01 5B 13 ...
      02 01 F0 DE 01 95 64 8A 00 00 00 00 00 CC 04 D0 01 D0 01   00 00 00 00 00 00 00 00 00 00 ...
      02 02 F0 3A 01 95 64 8A 00 00 00 00 00 CC 04 A0 03 2C 01   00 00 00 00 00 00 00 00 00 00 ...
    CS Procedure: 204 + 88
      02 00 F0 DA 00 68 C4 7A 00 00 00 00 00 CC 00 00 00 CC 00   80 00 02 00 0F 00 40 00 01 0A ...
      02 00 F8 66 00 68 C4 7A 00 00 00 00 00 58 00 00 00 58 00   00 C0 14 00 E5 B6 03 00 D7 5D ...
*/
static void  NBU_HADM_CopyConfig(void)
{
    #if CS_HANDOFF_ENABLED==1
    // debug only: check if the config is valid, no update here
    BLE_HADM_STATUS_t status = BLE_HADM_EventCheckConfig(p_hadm_config);
    assert(status==HADM_HAL_SUCCESS);
    #endif

    // non optimized sample code to convert LL timing to TSTMR counter value in us
    uint32_t clock;
    uint16_t qus;
    LL_API_GetBleTiming(&clock, &qus);
    uint64_t tstmr = *(uint64_t *)TSTMR0;

    uint64_t current = ((uint64_t)clock*625*2 + (uint64_t)qus) / 4;
    uint64_t start   = (uint64_t)ulStartTimeHSlot*625*2/4 + (uint64_t)ulStartTimeOffsetUs;

    // event start time in ullStartTimeTsTmr
    uint64_t distance = start - current;
    ullStartTimeTsTmr = tstmr + distance;

    // Event config is ready. Send it through RPMSG channel as ACL data messages */
    uint32_t remaining = sizeof(CS_HANDOFF_TYPE); // ~1228 bytes for event, ~204 bytes for procedure
    uint32_t no = 0;
    while(remaining>0)
    {
      /* fill the copy message */
      sHciConfig.packet_type = 0x02;
      sHciConfig.handle = 0xf000 + no;
      uint32_t sz = remaining > sizeof(sHciConfig.config) ? sizeof(sHciConfig.config):remaining;
      sHciConfig.data_length = 8+2+2+2+sz;
      sHciConfig.start_time = ullStartTimeTsTmr;
      sHciConfig.config_size = sizeof(CS_HANDOFF_TYPE);
      sHciConfig.config_offset = sizeof(CS_HANDOFF_TYPE) - remaining;
      sHciConfig.copied_size = sz;
      memcpy(&sHciConfig.config, (uint8_t*)p_hadm_config + sHciConfig.config_offset, sz);
      remaining -= sz;
      no++;

      /* do the copy */
      OSA_InterruptDisable();
      PLATFORM_SendHciMessage((uint8_t*)&sHciConfig, 1+2+2+sHciConfig.data_length);
      OSA_InterruptEnable();
    }

#if CS_HANDOFF_ENABLED==2
    // copy TBleHadmConfiguration_t
    remaining = sizeof(TBleHadmConfiguration_t); // ~88 bytes
    no = 0;
    while(remaining>0)
    {
      /* fill the copy message */
      sHciConfig.packet_type = 0x02;
      sHciConfig.handle = 0xf800 + no;
      uint32_t sz = remaining > sizeof(sHciConfig.config) ? sizeof(sHciConfig.config):remaining;
      sHciConfig.data_length = 8+2+2+2+sz;
      sHciConfig.start_time = ullStartTimeTsTmr;
      sHciConfig.config_size = sizeof(TBleHadmConfiguration_t);
      sHciConfig.config_offset = sizeof(TBleHadmConfiguration_t) - remaining;
      sHciConfig.copied_size = sz;
      memcpy(&sHciConfig.config, (uint8_t*)p_hadm_config->pActiveConfig + sHciConfig.config_offset, sz);
      remaining -= sz;
      no++;

      /* do the copy */
      OSA_InterruptDisable();
      PLATFORM_SendHciMessage((uint8_t*)&sHciConfig, 1+2+2+sHciConfig.data_length);
      OSA_InterruptEnable();
    }
#endif
}
#endif // #if CS_HANDOFF_ENABLED==1 || CS_HANDOFF_ENABLED==2

static void NBU_CheckTemperatureChange(void)
{
#if defined(gNbu_Hadm_d) && (gNbu_Hadm_d==1)
    static int32_t nbu_last_temperature = PLATFORM_SENSOR_UNKNOWN_TEMPERATURE/10;
    int32_t new_temperature;

    /* Simply read cached data into memory, not a sensor polling */
    PLATFORM_GetTemperatureValue(&new_temperature);
    /* Temperature is provided in tenth of Celsisus degrees.
     * We rely on sensor inertia to avoid very frequent updates.
     */
    if (new_temperature/10 != nbu_last_temperature)
    {
        assert(new_temperature < 5000);
        nbu_last_temperature = new_temperature/10;

        /* Inform interested parties */
        lcl_hadm_handle_temperature_change(new_temperature/10);
    }
#endif // gNbu_Hadm_d
}

/* Hook from LL Idle Task */
void NBU_Idle(void)
{
#if defined(CS_HANDOFF_ENABLED) && (CS_HANDOFF_ENABLED!=0)
    if( p_hadm_config != NULL )
    {
        NBU_HADM_CopyConfig();
        p_hadm_config = NULL;
    }
#endif

    NBU_CheckTemperatureChange();

    // Enable logging timestamps - required LL to be enabled - move it to somewhere else
    BOARD_DBGLOGCOUNTERRUNNING();

#if !defined(FPGA_TARGET) || (FPGA_TARGET == 0)
    /* Check if a measure is available and process the result
     * Called under masked interrupts so no more SFA interrupts are received */
    SFC_Process();
#endif /* FPGA_TARGET */

    /* Check if a nbu api indication message is pending and send it to host if it is the case */
    PLATFORM_FwkSrvCheckAndSendNbuApiIndicationInIdle();

    OSA_DisableIRQGlobal();

    if(pendingPktInfo > 0)
    {
        PWR_DBG_LOG("pendingPktInfo=%x", pendingPktInfo);
        pendingPktInfo--;
#ifdef __COVERAGESCANNER__
        handle_coverage_hci_command(hciPacketInfo[readPktInfoIdx].packetType, hciPacketInfo[readPktInfoIdx].pPacket);
#endif /*__COVERAGESCANNER__*/
        OSA_EnableIRQGlobal();

        BOARD_DBGLPIOSET(1u, 0u);

        /* we are not under exception context, we can send the packet now */
        NbuHci_SendPktToController(hciPacketInfo[readPktInfoIdx].packetType, hciPacketInfo[readPktInfoIdx].pPacket, hciPacketInfo[readPktInfoIdx].packetSize);
        MEM_BufferFree(hciPacketInfo[readPktInfoIdx].pPacket);

        readPktInfoIdx = (readPktInfoIdx+1)&(PACKET_INFO_QUEUE_SIZE-1);  //modulo PACKET_INFO_QUEUE_SIZE

        BOARD_DBGLPIOSET(1u, 1u);
    }
    else
    {
        BOARD_DBGLPIOSET(0u, 0u);

#if !defined (SDK_OS_FREE_RTOS)
#if !defined(gNbuJtagCapability)    || (gNbuJtagCapability==0)
        /* Try to go to low power (Deep Sleep), if that's not possible, it will
         * go to WFI only.
         * To keep full debug capability, set gNbuJtagCapability to 1 to avoid
         * Deep Sleep or WFI. */
        PLATFORM_EnterLowPower();
#endif
#endif

        BOARD_DBGLPIOSET(0u, 1u);

        OSA_EnableIRQGlobal();
    }
    nbu_tasks_init_done = TRUE;
#ifdef SIMULATOR
    {
        bool_t test_status = FALSE;
        //test_status = test_scan_privacy();
        //test_status = test_adv_conn_encrypted();
        test_status = test_conn_init();
#if defined(gNbu_Hadm_d) && (gNbu_Hadm_d==1)
        //test_status = test_conn_hadm();
        test_status = test_hadm_test_mode();
#endif
        assert(TRUE==test_status);

        while(1);
    }
#endif

}

/*API allows to read the CI coding indicator just after it is available (after header decoding done).
DATA_RATE = 3 => S2 encoding
DATA_RATE = 2 => S8 encoding
There is another mean to read the CI (though ptBLE_RD->RX_PKT_STATUS.RX_BLE_PKT_STATUS.uRxCodedIndic) but this CI indication is
only valid when packet is entirely received.
*/
uint8_t NbuGetCodedIndicator()
{
  return (((XCVR_2P4GHZ_PHY->STAT0 & GEN4PHY_STAT0_DATA_RATE_MASK) >> GEN4PHY_STAT0_DATA_RATE_SHIFT) == 3 ? 1 : 0);
}

/*API below allow to configure LDOs for peak power reduction purpose KFOURWONE-4185.
Those APIs are being called by LL if peak power reduction feature is enabled in LL.*/
void NbuPwrPeakReductionActivityStart()
{
  XCVR_forceLdoAntEnable();
  isHighZ = FALSE;
}
void NbuPwrPeakReductionActivityStop()
{
  XCVR_setLdoAntHiz();
  isHighZ = TRUE;
}
void NbuPwrPeakReductionDisable()
{
  if (isHighZ)
  {
    XCVR_releaseLdoAntAll();
    isHighZ = FALSE;
  }
}

void NBU_Init()
{
    /* Init MemManager for buffer allocation in serial manager */
    MEM_Init();
#ifndef SIMULATOR
    /* Low level init for the BLE controller */
    PLATFORM_InitBle();
#endif
    /* Init HCI Transport module */
    bleResult_t status = Hcit_Init((hciTransportInterface_t)Hcit_PktReceived);
    assert(status == gHciSuccess_c);
    (void)status;

#ifndef SIMULATOR
    /* Init Framework Intercore Service */
    PLATFORM_FwkSrvInit();

#if defined(PHY_15_4_ENABLED) && (PHY_15_4_ENABLED == 1)
    /* Init 15.4 Phy must be after PLATFORM_FwkSrvInit() for the RNG seeding
       and before SFC_Init() to have the 15.4 RPMSG endpoints initialized without delay */
    init_15_4_Phy();
#endif

#if !defined(FPGA_TARGET) || (FPGA_TARGET == 0)
    /* SFC module requires FwkSrv service to be initialized */
    SFC_Init();
#endif /* FPGA_TARGET */
#endif

#if defined(CS_HANDOFF_ENABLED) && (CS_HANDOFF_ENABLED!=0)
    NBU_HADM_Init();
#endif

#if defined(gUseIpcTransport_d) && (gUseIpcTransport_d == 1)
    Ipc_Init(s_IpcRpmsgHandle, &ipcRpmsgConfig, NULL);
#endif

#if defined(HDI_MODE) && (HDI_MODE == 1)
    NBU_InitPhySwitch();
#endif

#if !defined(gNbuDisableLowpower_d) || (gNbuDisableLowpower_d==0)
        /* Initialize required ressources before requesting low power entry
         * If gNbuDisableLowpower_d is set to 1, this function won't be called so
         * PLATFORM_EnterLowPower will only go to WFI
         * CAUTION: do not move before Controller_RadioInit */
#ifndef SIMULATOR
        PLATFORM_LowPowerInit();
#endif
#endif
}

#if defined(SDK_OS_FREE_RTOS)
void vApplicationMallocFailedHook( void )
{
    assert(0);
    /* Called if a call to pvPortMalloc() fails because there is insufficient
     * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
     * internally by FreeRTOS API functions that create tasks, queues, software
     * timers, and semaphores.  The size of the FreeRTOS heap is set by the
     * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
    taskDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask,
                                    char * pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
     * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     * function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
     * state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/* This function is called once when the Freertos timer task starts */
void vApplicationDaemonTaskStartupHook(void)
{
    NBU_Init();
}
#else
/* This function is called from ThreadX's tx_application_define function
 * If needed, we can create ThreadX objects (tasks, queues...) from there
 * This is also used to configure the Systicks (weren't before) */
void tx_application_define_hook(void)
{
    NBU_Init();
}
#endif

int main(void)
{
    /* Configure FRO192M clock */
#if !defined(FPGA_TARGET) || (FPGA_TARGET == 0)
    PLATFORM_InitFro192M();
#endif

#if !defined(FPGA_TARGET) || (FPGA_TARGET == 0)
    /* By default the NBU runs to 32MHz, set the constraint in the init to
     * prevent the app core to set a slower speed for the NBU on its side */
    PLATFORM_SetFrequencyConstraintFromController(2);
#endif

#if defined(SDK_OS_FREE_RTOS)
#define TICK_RATE_HZ 100U
    SysTick->LOAD = (BOARD_GetSystemCoreClockFreq() / TICK_RATE_HZ) - 1U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
#else
#define TICK_RATE_HZ 1000U
    SysTick->LOAD |= (BOARD_GetSystemCoreClockFreq() / TICK_RATE_HZ) - 1U;
    /* Not enabling the Systicks now, will be done in _tx_thread_schedule */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
#endif

#if !defined(RF_OSC_26MHZ) || (RF_OSC_26MHZ == 0)
    /* bt_eclk should be >=16MHz, so for XO=26MHz select XO while for XO=32MHz XO/2 can be selected
     * This allows to save some power during active mode */
    RADIO_CTRL->RF_CLK_CTRL &= ~RADIO_CTRL_RF_CLK_CTRL_BT_ECLK_DIV_MASK;
    RADIO_CTRL->RF_CLK_CTRL |= RADIO_CTRL_RF_CLK_CTRL_BT_ECLK_DIV(0x1U);
#endif

    /* Init OSA: should be called before any other OSA API*/
    OSA_Init();

    //static  volatile int i = 1;
    //while (i) {}

    Controller_RadioInit();

    /* Debug init */
    BOARD_DBGINITRFACTIVE();
    BOARD_DBGINITDTEST();
    BOARD_DBGCONFIGINITNBU(true);
    /* Init NbuDbg IOs, will configure pinmux and GPIOD
        - need to be done after PLATFORM_RemoteActiveReq() */
    BOARD_DBGINITDBGIO();

#ifdef __COVERAGESCANNER__
    __coveragescanner_set_custom_io(NULL,
        csfputs,
        csfopenappend,
        NULL,
        NULL,
        csfclose,
        NULL);
#endif /*__COVERAGESCANNER__*/

    Controller_SetNbuVersion(nbu_version.repo_digest);

    /* Start LL scheduler */
    Controller_Init(&nbuInterface);  /* never returns */

    /* Won't run here*/
    assert(0);
    return 0;
}
