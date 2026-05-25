#include "microchip.h"
#include <stdio.h>

void app_main() 
{
    microchip__init_gpio();
    microchip__init_tasks();
}