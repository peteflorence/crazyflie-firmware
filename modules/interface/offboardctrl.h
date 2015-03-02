#ifndef OFFBOARDCTRL_H_
#define OFFBOARDCTRL_H_
#include <stdint.h>
#include <stdbool.h>
#include "crtp.h"

#define OFFBOARDCTRL_WDT_TIMEOUT_SHUTDOWN M2T(1000)

void offboardCtrlInit(void);
bool offboardCtrlTest(void);
void offboardCtrlTask(void*);
void offboardCtrlCrtpCB(CRTPPacket*);
void getSensorsPacket(CRTPPacket*);

#endif /* OFFBOARDCTRL_H_ */
