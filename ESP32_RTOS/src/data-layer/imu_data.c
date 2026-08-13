#include "common_functions.h"
#include "tables.h"

imu_data_t * p_imu_data;

void imu_data__init(void)
{
    p_imu_data = (imu_data_t *)allocate_memory(sizeof(imu_data_t));
}

void encoder_data__update(uint16_t count)
{
    p_encoder_data->count = count;
    p_encoder_data->angle = (count / CPR) * 360;
}