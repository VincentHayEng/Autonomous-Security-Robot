// Comment
//
//

#include "table_index.h"

BaseType_t      task_return[NUM_OF_RETURN_VALUES];
TaskHandle_t    task_list[NUM_OF_TASKS];

//================================================================
// Public Function Definitions
//================================================================

void microchip__init_gpio()
{
    ESP_ERROR_CHECK(gpio_set_direction(LED_1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(LED_2, GPIO_MODE_OUTPUT));
    
    ESP_ERROR_CHECK(gpio_pulldown_dis(LED_1));
    ESP_ERROR_CHECK(gpio_pulldown_dis(LED_2));

    ESP_ERROR_CHECK(gpio_pullup_dis(LED_1));
    ESP_ERROR_CHECK(gpio_pullup_dis(LED_2));

    gpio_set_level(LED_1, 0);
    gpio_set_level(LED_2, 0);
}

void microchip__init_tasks()
{
    xTaskCreate(task1, "LED 1 Control", STACK_DEPTH, NULL, 0, &task_list[LED_1_TASK_INDEX]);
    xTaskCreate(task2, "LED 2 Control", STACK_DEPTH, NULL, 0, &task_list[LED_2_TASK_INDEX]);
}
