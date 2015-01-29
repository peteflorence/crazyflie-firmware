#ifndef FLIP_H_
#define FLIP_H_
#include <stdint.h>
#include <stdbool.h>

#define FLIP_WDT_TIMEOUT_SHUTDOWN M2T(500)

void flipInit(void);
bool flipTest(void);
void flipTask(void*);

#endif /* FLIP_H_ */
