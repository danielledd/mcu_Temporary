/**
 * @file   main.c
 * @brief  Basic PWM(Pulse-width modulation) output example.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <pin.h>

#define PWM_FREQUENCY (1)  /* PWM frequency in Hz. */
#define PWM_DUTY_CYCLE (60) /* PWM duty-cycle in %. */

/*
 * PER = {f_tim / [(PRS + 1) * f_pwm]} - 1
 * 
 * f_pwm: PWM frequency.
 * f_tim: Timer frequency. The value is 'rcc_apb1_frequency * 2' equal 32MHz in this case.
 * PRS:   PWM timer prescaler.
 * PER:   PWM timer period.
 */
#define PWM_TIMER_PRESCALER (32 - 1)
#define PWM_TIMER_PERIOD (((rcc_apb1_frequency * 2) / ((PWM_TIMER_PRESCALER + 1) * PWM_FREQUENCY)) - 1)


void gpio_setup(void)
{
    /* TIM22-Channel2 on PB5 (depending on the specific STM32 model, GPIO setup may vary). */
    /* Enable GPIOB clock. */
    rcc_periph_clock_enable(RCC_GPIOB);
    /* Set PB5 (for TIM22_CH2) to alternate function mode. */
    gpio_mode_setup(PWM_VDDC_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_VDDC_PIN);
    /* Set alternate function for PB5 to TIM22_CH2. */
    gpio_set_af(PWM_VDDC_PORT, GPIO_AF2, PWM_VDDC_PIN);
    /* Set PB5 to push-pull with 50 MHz speed. */
    gpio_set_output_options(PWM_VDDC_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PWM_VDDC_PIN);
}

void pwm_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM22);

    timer_set_mode(TIM22,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    timer_disable_preload(TIM22);
    timer_continuous_mode(TIM22);

    timer_set_prescaler(TIM22, PWM_TIMER_PRESCALER);
    timer_set_period(TIM22, PWM_TIMER_PERIOD);

    timer_set_oc_mode(TIM22, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_value(TIM22,
                       TIM_OC2,
                       (PWM_TIMER_PERIOD + 1) * (PWM_DUTY_CYCLE / 100.0));

    timer_enable_oc_output(TIM22, TIM_OC2);
    timer_enable_counter(TIM22);
}

int init_pwm(void)
{
    /* Setup system clock = 32MHz. */
    gpio_setup();
    pwm_setup();

    /* Halt. */
    // while (1)
    // {
    //     __asm__("nop");
    // }

    return 0;
}
