#include "led_tasks.h"
#include "defines.h"

void task1(void * pvParameters)
{

    uint8_t out;

    for(;;)
    {
        ESP_ERROR_CHECK(gpio_get_level(LED_1));

        out = gpio_get_level(LED_1) ? 1 : 0;
        gpio_set_level(LED_1, !out);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task2(void * pvParameters)
{

    uint8_t out;

    for(;;)
    {
        ESP_ERROR_CHECK(gpio_get_level(LED_2));

        out = gpio_get_level(LED_2) ? 1 : 0;
        gpio_set_level(LED_2, !out);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}