#ifndef __USER_CONFIG__
#define __USER_CONFIG__

#include "mex.h"
#include "string.h"
#include "stdint.h"
#include "tinympc.h"

uint8_t task_init(void);
void task_main(void);

extern float u[N_INPUT];
extern float x_current[N_STATE];

#endif