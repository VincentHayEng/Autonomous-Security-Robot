#define LED_1   13
#define LED_2   14

// LEFT BTS7960
#define LEFT_RPWM       GPIO_NUM_25
#define LEFT_LPWM       GPIO_NUM_26
#define LEFT_REN        GPIO_NUM_27
#define LEFT_LEN        GPIO_NUM_14

// RIGHT BTS7960
#define RIGHT_RPWM      GPIO_NUM_32
#define RIGHT_LPWM      GPIO_NUM_33
#define RIGHT_REN       GPIO_NUM_18
#define RIGHT_LEN       GPIO_NUM_19

// PWM settings
#define PWM_FREQ        1000
#define PWM_RESOLUTION  LEDC_TIMER_8_BIT
#define PWM_MAX         255

#define LEFT_RPWM_CH    LEDC_CHANNEL_0
#define LEFT_LPWM_CH    LEDC_CHANNEL_1
#define RIGHT_RPWM_CH   LEDC_CHANNEL_2
#define RIGHT_LPWM_CH   LEDC_CHANNEL_3