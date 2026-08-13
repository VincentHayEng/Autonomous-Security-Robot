
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

void motor_driver__set_pwm(ledc_channel_t channel, uint8_t duty)
{
    if (duty < 0)
    {
        duty = 0;
    }

    if (duty > PWM_MAX)
    {
        duty = PWM_MAX;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void motor_driver__left_stop()
{
    motor_driver__set_pwm(LEFT_RPWM_CH, 0);
    motor_driver__set_pwm(LEFT_LPWM_CH, 0);
}

void motor_driver__right_stop()
{
    motor_driver__set_pwm(RIGHT_RPWM_CH, 0);
    motor_driver__set_pwm(RIGHT_LPWM_CH, 0);
}

void motor_driver__stop_all()
{
    motor_driver__left_stop();
    motor_driver__right_stop();
}

void motor_driver__left_forward(uint8_t motor_speed)
{
    motor_driver__set_pwm(LEFT_RPWM_CH, motor_speed);
    motor_driver__set_pwm(LEFT_LPWM_CH, 0);
}

void motor_driver__left_reverse(uint8_t motor_speed)
{
    motor_driver__set_pwm(LEFT_RPWM_CH, 0);
    motor_driver__set_pwm(LEFT_LPWM_CH, motor_speed);
}

void motor_driver__right_forward(uint8_t motor_speed)
{
    motor_driver__set_pwm(RIGHT_RPWM_CH, motor_speed);
    motor_driver__set_pwm(RIGHT_LPWM_CH, 0);
}

void motor_driver__right_reverse(uint8_t motor_speed)
{
    motor_driver__set_pwm(RIGHT_RPWM_CH, 0);
    motor_driver__set_pwm(RIGHT_LPWM_CH, motor_speed);
}

void motor_driver__move_forward()
{
    motor_driver__left_forward();
    motor_driver__right_forward();
}

void motor_driver__move_backward()
{
    motor_driver__left_reverse();
    motor_driver__right_reverse();
}

void motor_driver__turn_left()
{
    motor_driver__left_reverse();
    motor_driver__right_forward();
}

void motor_driver__turn_right()
{
    motor_driver__left_forward();
    motor_driver__right_reverse();
}

void motor_driver__only_left()
{
    motor_driver__left_forward();
    motor_driver__right_stop();
}

void motor_driver__only_right()
{
    motor_driver__left_stop();
    motor_driver__right_forward();
}
