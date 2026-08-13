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

    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =
        (1ULL << LEFT_REN) |
        (1ULL << LEFT_LEN) |
        (1ULL << RIGHT_REN) |
        (1ULL << RIGHT_LEN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

    // Enable both sides of both BTS7960 drivers
    gpio_set_level(LEFT_REN, 1);
    gpio_set_level(LEFT_LEN, 1);
    gpio_set_level(RIGHT_REN, 1);
    gpio_set_level(RIGHT_LEN, 1);

    ledc_timer_config_t timer;
    memset(&timer, 0, sizeof(timer));

    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.timer_num = LEDC_TIMER_0;
    timer.duty_resolution = PWM_RESOLUTION;
    timer.freq_hz = PWM_FREQ;
    timer.clk_cfg = LEDC_AUTO_CLK;

    ledc_timer_config(&timer);

    ledc_channel_config_t ch;
    memset(&ch, 0, sizeof(ch));

    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.timer_sel = LEDC_TIMER_0;
    ch.intr_type = LEDC_INTR_DISABLE;
    ch.duty = 0;
    ch.hpoint = 0;

    ch.channel = LEFT_RPWM_CH;
    ch.gpio_num = LEFT_RPWM;
    ledc_channel_config(&ch);

    ch.channel = LEFT_LPWM_CH;
    ch.gpio_num = LEFT_LPWM;
    ledc_channel_config(&ch);

    ch.channel = RIGHT_RPWM_CH;
    ch.gpio_num = RIGHT_RPWM;
    ledc_channel_config(&ch);

    ch.channel = RIGHT_LPWM_CH;
    ch.gpio_num = RIGHT_LPWM;
    ledc_channel_config(&ch);

    stop_all();

    ESP_LOGI(TAG, "Motor pins initialized");
}

void microchip__init_tasks()
{
    xTaskCreate(task1, "LED 1 Control", STACK_DEPTH, NULL, 0, &task_list[LED_1_TASK_INDEX]);
    xTaskCreate(task2, "LED 2 Control", STACK_DEPTH, NULL, 0, &task_list[LED_2_TASK_INDEX]);
}
