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

#define OFFBOARDCTRL_UPDATE_FREQ 200.0f
#define A_OFFSET -1.208905335853438f
#define V_MAX 4.0f
//#define OFFBOARDCTRL_FORMATION_X

struct InputCrtpValues
{
  float input1;
  float input2;
  float input3;
  float input4;
  float offset;
  int type;
} __attribute__((packed));

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;

static float rpyX[3];
static float rpyN[3];
static float omegaX[3];
static float omegaN[3];

static float IMURotMatrix[3][3] = {{0.7071, -0.7071, 0},
                                   {0.7071,  0.7071, 0},
                                   {0,       0,      1}};

float ROLL_KP = 3.5*180/M_PI;
float PITCH_KP = 3.5*180/M_PI;
float YAW_KP = 0.0;
float ROLL_RATE_KP = 70*180/M_PI;
float PITCH_RATE_KP = 70*180/M_PI; 
float YAW_RATE_KP = 50*180/M_PI;

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
static void rotateGyro(float*, float*);
static void rotateRPY(float*, float*);
void offboardCtrlTask(void* param);
static uint16_t limitThrust(int32_t value);

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
    inputCmd.offset = 0.0;
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
    // "32bits"
    // input is raw thrust values
    thrust1 = inputCmd.input1 + inputCmd.offset;
    thrust2 = inputCmd.input2 + inputCmd.offset;
    thrust3 = inputCmd.input3 + inputCmd.offset;
    thrust4 = inputCmd.input4 + inputCmd.offset; 
  } 
  else if (inputCmd.type==2)
  {
    // "omeguasqu"
    // input is omega square
    Va = pmGetBatteryVoltage();
    omega1 = sqrt(max(0.0,inputCmd.input1 + inputCmd.offset));
    omega2 = sqrt(max(0.0,inputCmd.input2 + inputCmd.offset));
    omega3 = sqrt(max(0.0,inputCmd.input3 + inputCmd.offset));
    omega4 = sqrt(max(0.0,inputCmd.input4 + inputCmd.offset));
    thrust1 = ((V_MAX/Va)*omega1 + A_OFFSET)*10000.0;
    thrust2 = ((V_MAX/Va)*omega2 + A_OFFSET)*10000.0;
    thrust3 = ((V_MAX/Va)*omega3 + A_OFFSET)*10000.0;
    thrust4 = ((V_MAX/Va)*omega4 + A_OFFSET)*10000.0;
  }
  else if (inputCmd.type==3)
  {
    // "onboardpd"
    // input only contains an offset, use onboard PD
    #ifdef OFFBOARDCTRL_FORMATION_X
      thrust1 = 0.5*ROLL_KP*rpyX[0] + 0.5*PITCH_KP*rpyX[1] + YAW_KP*rpyX[2] + 0.5*ROLL_RATE_KP*omegaX[0] + 0.5*PITCH_RATE_KP*omegaX[1] + YAW_RATE_KP*omegaX[2] + inputCmd.offset;
      thrust2 = 0.5*ROLL_KP*rpyX[0] - 0.5*PITCH_KP*rpyX[1] - YAW_KP*rpyX[2] + 0.5*ROLL_RATE_KP*omegaX[0] - 0.5*PITCH_RATE_KP*omegaX[1] - YAW_RATE_KP*omegaX[2] + inputCmd.offset;
      thrust3 = -0.5*ROLL_KP*rpyX[0] - 0.5*PITCH_KP*rpyX[1] + YAW_KP*rpyX[2] - 0.5*ROLL_RATE_KP*omegaX[0] - 0.5*PITCH_RATE_KP*omegaX[1] + YAW_RATE_KP*omegaX[2] + inputCmd.offset;
      thrust4 = -0.5*ROLL_KP*rpyX[0] + 0.5*PITCH_KP*rpyX[1] - YAW_KP*rpyX[2] - 0.5*ROLL_RATE_KP*omegaX[0] + 0.5*PITCH_RATE_KP*omegaX[1] - YAW_RATE_KP*omegaX[2] + inputCmd.offset;
    #else // OFFBOARDCTRL_FORMATION_NORMAL
      thrust1 = PITCH_KP*rpyN[1] + YAW_KP*rpyN[2] + PITCH_RATE_KP*omegaN[1] + YAW_RATE_KP*omegaN[2] + inputCmd.offset;
      thrust2 = ROLL_KP*rpyN[0] - YAW_KP*rpyN[2] + ROLL_RATE_KP*omegaN[0] - YAW_RATE_KP*omegaN[2] + inputCmd.offset;
      thrust3 = -PITCH_KP*rpyN[1] + YAW_KP*rpyN[2] - PITCH_RATE_KP*omegaN[1] + YAW_RATE_KP*omegaN[2] + inputCmd.offset;
      thrust4 = -ROLL_KP*rpyN[0] - YAW_KP*rpyN[2] - ROLL_RATE_KP*omegaN[0] - YAW_RATE_KP*omegaN[2] + inputCmd.offset;
    #endif
  }

  motorsSetRatio(MOTOR_M1,limitThrust((uint32_t)thrust1));
  motorsSetRatio(MOTOR_M2,limitThrust((uint32_t)thrust2));
  motorsSetRatio(MOTOR_M3,limitThrust((uint32_t)thrust3));
  motorsSetRatio(MOTOR_M4,limitThrust((uint32_t)thrust4));
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

static void updateSensors(float dt)
{
  imu6Read(&gyro, &acc);
  if (imu6IsCalibrated())
  {
    sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, dt);
    sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
    
    // make rpy with motor1 being front
    rpyX[0] = eulerRollActual*M_PI/180.0;
    rpyX[1] = eulerPitchActual*M_PI/180.0;
    rpyX[2] = eulerYawActual*M_PI/180.0;
    rotateRPY(rpyX,rpyN);

    // make omega with motor1 being front
    omegaX[0] = gyro.x*M_PI/180.0;
    omegaX[1] = gyro.y*M_PI/180.0;
    omegaX[2] = gyro.z*M_PI/180.0;
    rotateGyro(omegaX,omegaN);

    // pitch is inverted
    rpyN[1] = -rpyN[1];

    memcpy(pk.data,&(rpyN[0]),4);
    memcpy(pk.data+4,&(rpyN[1]),4);
    memcpy(pk.data+8,&(rpyN[2]),4);
    memcpy(pk.data+12,&(omegaN[0]),4);
    memcpy(pk.data+16,&(omegaN[1]),4);
    memcpy(pk.data+20,&(omegaN[2]),4);
    crtpSendPacketNoWait(&pk);
  }
}

static void rotateGyro(float* omega, float* rotomega)
{
  // IMURotMatrix*omega
  int i;
  int j;
  for (i=0;i<3;i++)
  {
    rotomega[i] = 0;
    for (j=0;j<3;j++)
    {
      rotomega[i] += IMURotMatrix[i][j]*omega[j];
    }
  }
}

static void rotateRPY(float* rpy, float* rotrpy)
{

  float rotmat[3][3]; 
  rotmat[0][0] = cos(rpy[2])*cos(rpy[1]);
  rotmat[0][1] = cos(rpy[2])*sin(rpy[1])*sin(rpy[0])-sin(rpy[2])*cos(rpy[0]);
  rotmat[0][2] = cos(rpy[2])*sin(rpy[1])*cos(rpy[0])+sin(rpy[2])*sin(rpy[0]);
  rotmat[1][0] = sin(rpy[2])*cos(rpy[1]);
  rotmat[1][1] = sin(rpy[2])*sin(rpy[1])*sin(rpy[0])+cos(rpy[2])*cos(rpy[0]);
  rotmat[1][2] = sin(rpy[2])*sin(rpy[1])*cos(rpy[0])-cos(rpy[2])*sin(rpy[0]);
  rotmat[2][0] = -sin(rpy[1]);
  rotmat[2][1] = cos(rpy[1])*sin(rpy[0]);
  rotmat[2][2] = cos(rpy[1])*cos(rpy[0]);

  // rotmat*IMURotMatrix
  float rotmatR[3][3];
  int i;
  int j;
  int k;
  for (i=0;i<3;i++)
  {
    for (j=0;j<3;j++)
    {
      rotmatR[i][j] = 0;
      for (k=0;k<3;k++)
      {
        rotmatR[i][j] += rotmat[i][k]*IMURotMatrix[k][j];
      }
    }
  }

  // rotmat2rpy
  rotrpy[0] = atan2(rotmatR[2][1],rotmatR[2][2]);
  rotrpy[1] = atan2(-rotmatR[2][0],sqrt(pow(rotmatR[2][1],2) + pow(rotmatR[2][2],2)));
  rotrpy[2] = atan2(rotmatR[1][0],rotmatR[0][0]);
}

void offboardCtrlTask(void* param)
{
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_OFFBOARDCTRL_ID_NBR);
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount();
  while(1)
  { 
    vTaskDelayUntil(&lastWakeTime, F2T(OFFBOARDCTRL_UPDATE_FREQ));
    updateSensors(1.0/OFFBOARDCTRL_UPDATE_FREQ);
    updateThrusts();
  }
}