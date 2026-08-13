#include "encoder_data.h"

void encoder_task(void *arg)
{
    while (1)
    {
        int16_t count;

        pcnt_unit_get_count(unit, &count);

        encoder_data__update(count);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}