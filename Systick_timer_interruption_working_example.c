#include "stm32l476xx.h"

#define SYSTEM_CLOCK 4000000UL  // 4 MHz system clock
volatile uint32_t delay_counter = 0;

// Configure GPIOA pin 5 for LED
void init_LED() {
    // 1. Enable GPIOA clock
    RCC->AHB2ENR |= (1 << 0);

    // 2. Configure PA5 as output
    GPIOA->MODER &= ~(3U << (2 * 5));  // Clear bits 11:10
    GPIOA->MODER |=  (1U << (2 * 5));  // Set bit 10 (MODER5 = 01)
}

// Configure SysTick to generate interrupt every 1 ms
void SysTick_Init(void) {
    SysTick->CTRL = 0;  // Disable SysTick

    // ?? FIXED: The reload value must be (Clock / 1000) - 1 for 1 ms tick,
    // NOT (Clock * delay_ms - 1)
    SysTick->LOAD = (SYSTEM_CLOCK / 1000) - 1;  // 1 ms tick
    SysTick->VAL  = 0;                          // Clear current value

    // Enable SysTick with processor clock and interrupt
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk ;
}

// SysTick interrupt handler
void SysTick_Handler(void) {
    if (delay_counter > 0) {
        delay_counter--;
    }
}

// Blocking delay in milliseconds
void delay_ms(uint32_t ms) {
    delay_counter = ms;
    while (delay_counter > 0);
}

int main(void) {
    __disable_irq();

    init_LED();
    SysTick_Init(); // Configure SysTick for 1ms
    NVIC_SetPriority(SysTick_IRQn, 1);

    __enable_irq();

    while (1) {
        // Turn LED ON
        GPIOA->BSRR = (1 << 5);
        delay_ms(500);
        // Turn LED OFF
        GPIOA->BSRR = (1 << (5 + 16));
        delay_ms(500);
    }
}
