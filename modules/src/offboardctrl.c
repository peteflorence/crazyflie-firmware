#define DEBUG_MODULE "OFFBOARDCTRL"

#include "math.h"
#include "string.h"
#include "complex.h"

#include "FreeRTOS.h"
#include "task.h"

#include "offboardctrl.h"
#include "crtp.h"
#include "motors.h"
#include "system.h"
#include "sensfusion6.h"
#include "imu.h"
#include "pm.h"
#include "debug.h"

#define A_OFFSET -1.208905335853438f
#define V_MAX 4.0f

struct InputCrtpValues
{
  float input1;
  float input2;
  float input3;
  float input4;
  int type;
} __attribute__((packed));

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;

static float eulerRollActualRad;
static float eulerPitchActualRad;
static float eulerYawActualRad;

static float complex eulerRollComplexRad;
static float complex eulerPitchComplexRad;
static float complex eulerYawComplexRad;

static float eulerRollRealRad;
static float eulerPitchRealRad;
static float eulerYawRealRad;

static Axis3f gyroRad;
static Axis3f gyroRealRad;  

static bool isInit;
static bool isInactive;
static uint32_t lastUpdate;
static struct InputCrtpValues inputCmd;

static float Va = V_MAX;
static float omega1 = 0.0;
static float omega2 = 0.0;
static float omega3 = 0.0;
static float omega4 = 0.0;
static float thrust1 = 0.0;
static float thrust2 = 0.0;
static float thrust3 = 0.0;
static float thrust4 = 0.0;

static CRTPPacket pk;

static void offboardCtrlCrtpCB(CRTPPacket* pk);
static void offboardCtrlWatchdogReset(void);
static void updateSensors(float);
static void updateThrusts(void);
void offboardCtrlTask(void* param);

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

void offboardCtrlInit(void)
{
  if(isInit)
    return;

  crtpInit();
  motorsInit();
  imu6Init();
  sensfusion6Init();
  crtpRegisterPortCB(CRTP_PORT_OFFBOARDCTRL, offboardCtrlCrtpCB);

  eulerRollActual = 0.0;
  eulerPitchActual = 0.0;
  eulerYawActual = 0.0;
  gyro.x = 0.0;
  gyro.y = 0.0;
  gyro.z = 0.0;

  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;

  pk.header = CRTP_HEADER(CRTP_PORT_SENSORS, 0);
  pk.size = 6*4;

  xTaskCreate(offboardCtrlTask, (const signed char * const)"OFFBOARDCTRL",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);
}

bool offboardCtrlTest(void)
{
  bool pass = true;
  pass &= crtpTest();
  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  return pass;
}

static void offboardCtrlCrtpCB(CRTPPacket* inputPk)
{
  inputCmd = *((struct InputCrtpValues*)inputPk->data);
  offboardCtrlWatchdogReset();
}

void offboardCtrlWatchdog(void)
{
  uint32_t ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;
  if (ticktimeSinceUpdate > OFFBOARDCTRL_WDT_TIMEOUT_SHUTDOWN)
  {
    inputCmd.input1 = 0.0;
    inputCmd.input2 = 0.0;
    inputCmd.input3 = 0.0;
    inputCmd.input4 = 0.0;
    inputCmd.type = 1;
    isInactive = true;
  }
  else
  {
    isInactive = false;
  }
}

static void offboardCtrlWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

static void updateThrusts(void)
{
  offboardCtrlWatchdog();

  if (inputCmd.type==1)
  {
    thrust1 = inputCmd.input1;
    thrust2 = inputCmd.input2;
    thrust3 = inputCmd.input3;
    thrust4 = inputCmd.input4; 
  } 
  else if (inputCmd.type==2)
  {
    Va = pmGetBatteryVoltage();
    omega1 = sqrt(max(0.0,inputCmd.input1));
    omega2 = sqrt(max(0.0,inputCmd.input2));
    omega3 = sqrt(max(0.0,inputCmd.input3));
    omega4 = sqrt(max(0.0,inputCmd.input4));
    thrust1 = ((V_MAX/Va)*omega1+A_OFFSET)*10000.0;
    thrust2 = ((V_MAX/Va)*omega2+A_OFFSET)*10000.0;
    thrust3 = ((V_MAX/Va)*omega3+A_OFFSET)*10000.0;
    thrust4 = ((V_MAX/Va)*omega4+A_OFFSET)*10000.0;
  }

  thrust1 = min(65000.0,max(0.0,thrust1));
  thrust2 = min(65000.0,max(0.0,thrust2));
  thrust3 = min(65000.0,max(0.0,thrust3));
  thrust4 = min(65000.0,max(0.0,thrust4));
  motorsSetRatio(MOTOR_M1,(uint16_t) thrust1);
  motorsSetRatio(MOTOR_M2,(uint16_t) thrust2);
  motorsSetRatio(MOTOR_M3,(uint16_t) thrust3);
  motorsSetRatio(MOTOR_M4,(uint16_t) thrust4);
}

static void updateSensors(float dt)
{
  imu6Read(&gyro, &acc);
  if (imu6IsCalibrated())
  {
    sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, dt);
    sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
    
    eulerRollActualRad = eulerRollActual*M_PI/180.0;
    eulerPitchActualRad = eulerPitchActual*M_PI/180.0;
    eulerYawActualRad = eulerYawActual*M_PI/180.0;
    eulerRollComplexRad = 0.0;//log((sqrt(2.0)*sin(eulerPitchActualRad)*0.5*I+cos(eulerRollActualRad)*cos(eulerPitchActualRad)+sqrt(2.0)*cos(eulerPitchActualRad)*sin(eulerRollActualRad)*0.5*I)/fabs(sqrt(2.0)*sin(eulerPitchActualRad)*0.5*I+cos(eulerRollActualRad)*cos(eulerPitchActualRad)+sqrt(2.0)*cos(eulerPitchActualRad)*sin(eulerRollActualRad)*0.5*I))*-I;
    // Note the inversion of the pitch
    eulerPitchComplexRad  = 0.0;//-log((sqrt(pow(cos(eulerRollActualRad),2.0)*pow(cos(eulerPitchActualRad),2.0)+pow(sqrt(2.0)*sin(eulerPitchActualRad)*(1.0/2.0)+sqrt(2.0)*cos(eulerPitchActualRad)*sin(eulerRollActualRad)*(1.0/2.0),2.0))+sqrt(2.0)*sin(eulerPitchActualRad)*0.5*I-sqrt(2.0)*cos(eulerPitchActualRad)*sin(eulerRollActualRad)*0.5*I)/fabs(sqrt(pow(cos(eulerRollActualRad),2.0)*pow(cos(eulerPitchActualRad),2.0)+pow(sqrt(2.0)*sin(eulerPitchActualRad)*(1.0/2.0)+sqrt(2.0)*cos(eulerPitchActualRad)*sin(eulerRollActualRad)*(1.0/2.0),2.0))+sqrt(2.0)*sin(eulerPitchActualRad)*0.5*I-sqrt(2.0)*cos(eulerPitchActualRad)*sin(eulerRollActualRad)*0.5*I))*-I;
    eulerYawComplexRad= 0.0;//log(((cos(eulerYawActualRad)+sin(eulerYawActualRad)*I)*(cos(eulerRollActualRad)*I+cos(eulerPitchActualRad)+sin(eulerRollActualRad)*sin(eulerPitchActualRad)))/fabs((cos(eulerYawActualRad)+sin(eulerYawActualRad)*I)*(cos(eulerRollActualRad)*I+cos(eulerPitchActualRad)+sin(eulerRollActualRad)*sin(eulerPitchActualRad))))*-I;
    eulerRollRealRad = creal(eulerRollComplexRad);
    eulerPitchRealRad = creal(eulerPitchComplexRad);
    eulerYawRealRad = creal(eulerYawComplexRad);

    gyroRad.x = gyro.x*M_PI/180.0;
    gyroRad.y = gyro.y*M_PI/180.0;
    gyroRad.z = gyro.z*M_PI/180.0;
    gyroRealRad.x = sqrt(2.0)*gyroRad.x*(1.0/2.0)-sqrt(2.0)*gyroRad.y*(1.0/2.0);
    gyroRealRad.y = sqrt(2.0)*gyroRad.x*(1.0/2.0)+sqrt(2.0)*gyroRad.y*(1.0/2.0);
    gyroRealRad.z = gyroRad.z;

    memcpy(pk.data,&eulerRollActualRad,4);
    memcpy(pk.data+4,&eulerPitchActualRad,4);
    memcpy(pk.data+8,&eulerYawActualRad,4);
    memcpy(pk.data+12,&(gyroRealRad.x),4);
    memcpy(pk.data+16,&(gyroRealRad.y),4);
    memcpy(pk.data+20,&(gyroRealRad.z),4);
    crtpSendPacketNoWait(&pk);
  }
}

void offboardCtrlTask(void* param)
{
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_OFFBOARDCTRL_ID_NBR);
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount();
  while(1)
  { 
    vTaskDelayUntil(&lastWakeTime, F2T(100.0));
    updateSensors((float)(1.0/100.0));
    updateThrusts();
  }
}