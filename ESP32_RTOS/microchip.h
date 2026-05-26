//================================================================
// Includes
//================================================================

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"

//================================================================
// Defines
//================================================================

#define STACK_DEPTH         1024

//================================================================
// Public Function Declarations
//================================================================

void microchip__init_gpio(void);
void microchip__init_tasks(void);