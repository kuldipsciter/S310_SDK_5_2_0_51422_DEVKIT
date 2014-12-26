#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "nrf.h"
#include "devkit\devkit_constant.h"
#include "AntEventHandler.h"
#include "main.h"
#include "boards/ras_1_6.h"

#define SWITCH_PRESSED     17
#define SWITCH_RELEASED    18

#define MOTION_DETECTED    19
#define MOTION_STOP        20

#define WATER_DETECTED     21
#define WATER_NOT_DETECTED 22


void AntAschyncEvent(void);

