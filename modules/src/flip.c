#define DEBUG_MODULE "FLIP"

#include "FreeRTOS.h"
#include "task.h"

#include "flip.h"
#include "crtp.h"
#include "motors.h"
#include "system.h"
#include "debug.h"

struct ThrustCrtpValues
{
  uint16_t thrust1;
  uint16_t thrust2;
  uint16_t thrust3;
  uint16_t thrust4;
} __attribute__((packed));

#define THRUSTS_UPDATE_FREQ 500

static bool isInit;
static bool isInactive;
static uint32_t lastUpdate;
static struct ThrustCrtpValues thrustsCmd;

static void flipCrtpCB(CRTPPacket* pk);
void flipTask(void* param);
static void flipWatchdogReset(void);
static void updateThrusts(void);

void flipInit(void)
{
  if(isInit)
    return;

  crtpInit();
  motorsInit();
  crtpRegisterPortCB(CRTP_PORT_FLIP, flipCrtpCB);

  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;
}

bool flipTest(void)
{
  bool pass = true;
  pass &= crtpTest();
  pass &= motorsTest();
  return pass;
}

static void flipCrtpCB(CRTPPacket* pk)
{
  thrustsCmd = *((struct ThrustCrtpValues*)pk->data);
  flipWatchdogReset();
}

void flipWatchdog(void)
{
  uint32_t ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;
  if (ticktimeSinceUpdate > FLIP_WDT_TIMEOUT_SHUTDOWN)
  {
    thrustsCmd.thrust1 = 0;
    thrustsCmd.thrust2 = 0;
    thrustsCmd.thrust3 = 0;
    thrustsCmd.thrust4 = 0;
    isInactive = true;
  }
  else
  {
    isInactive = true;
  }
}

static void flipWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

static void updateThrusts(void)
{
  flipWatchdog();
  motorsSetRatio(MOTOR_M1,(uint32_t) thrustsCmd.thrust1);
  motorsSetRatio(MOTOR_M2,(uint32_t) thrustsCmd.thrust2);
  motorsSetRatio(MOTOR_M3,(uint32_t) thrustsCmd.thrust3);
  motorsSetRatio(MOTOR_M4,(uint32_t) thrustsCmd.thrust4);
}

void flipTask(void* param)
{
  vTaskSetApplicationTaskTag(0, (void*)TASK_FLIP_ID_NBR);
  systemWaitStart();
  uint32_t lastWakeTime = xTaskGetTickCount();
  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(THRUSTS_UPDATE_FREQ));
    updateThrusts();
  }
}
