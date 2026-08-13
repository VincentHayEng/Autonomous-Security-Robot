#include "common_functions.h"
#include "tables.h"

encoder_data_t * p_encoder_data;

void encoder_data__init(void)
{
    p_encoder_data = (encoder_data_t *)allocate_memory(sizeof(encoder_data_t));
}

void encoder_data__update(uint16_t count)
{
    p_encoder_data->count = count;
    p_encoder_data->angle = (count / CPR) * 360;
}