#include "stm32l476xx.h"

void SystemClock_Config(void);
void GPIO_Config(void);
void TIM2_PWM_Config(void);
void delay_ms(uint32_t ms);

int main(void)
{
    SystemClock_Config(); // Configure l'horloge système à 4 MHz (MSI)
    GPIO_Config();        // Configure PA5 pour TIM2_CH1
    TIM2_PWM_Config();    // Configure Timer 2 pour PWM
    
    while (1)
    {
        // Fade in (0 à 999)
        for (uint16_t i = 0; i < 1000; i++)
        {
            TIM2->CCR1 = i;
            delay_ms(2);
        }
        
        // Fade out (999 à 0)
        for (int16_t i = 999; i >= 0; i--)
        {
            TIM2->CCR1 = i;
            delay_ms(2);
        }
    }
}

void SystemClock_Config(void)
{
    // Configure MSI à 4 MHz (déjà la valeur par défaut)
    // Vérifier que MSI est bien à 4 MHz
    RCC->CR |= RCC_CR_MSION;  // Activer MSI
    while ((RCC->CR & RCC_CR_MSIRDY) == 0); // Attendre MSI ready
    
    // MSI range = 4 MHz (0x6)
    RCC->CR &= ~RCC_CR_MSIRANGE;
    RCC->CR |= RCC_CR_MSIRANGE_6; // 4 MHz
    RCC->CR |= RCC_CR_MSIRGSEL;    // Utiliser MSIRANGE du CR
}

void GPIO_Config(void)
{
    // 1. Activer horloge GPIOA
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    
    // 2. Mettre PA5 en mode alternatif AF1 (TIM2_CH1)
    GPIOA->MODER &= ~(3U << (5 * 2));  // Clear bits
    GPIOA->MODER |=  (2U << (5 * 2));  // Mode alternatif (10)
    
    // 3. Configurer la fonction alternative AF1
    GPIOA->AFR[0] &= ~(0xF << (5 * 4)); // Clear AF pour PA5
    GPIOA->AFR[0] |=  (1U << (5 * 4));  // AF1 = TIM2_CH1
    
    // 4. Configurer la vitesse (optionnel, mais recommandé)
    GPIOA->OSPEEDR |= (2U << (5 * 2));  // High speed
}

void TIM2_PWM_Config(void)
{
    // 1. Activer horloge TIM2
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    
    // 2. Configurer prescaler et ARR
    TIM2->PSC = 39;       // 4 MHz / (39+1) = 100 kHz
    TIM2->ARR = 999;      // 100 kHz / 1000 = 100 Hz (10 ms période)
    
    // 3. Configurer canal 1 en PWM mode 1
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;            // Clear mode
    TIM2->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos); // PWM mode 1 (0110)
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;            // Enable preload
    
    // 4. Activer la sortie CH1
    TIM2->CCER |= TIM_CCER_CC1E;
    
    // 5. Activer l'auto-reload preload
    TIM2->CR1 |= TIM_CR1_ARPE;
    
    // 6. Initialiser le duty cycle à 0
    TIM2->CCR1 = 0;
    
    // 7. Générer un événement d'update pour charger les registres
    TIM2->EGR |= TIM_EGR_UG;
    
    // 8. Démarrer le timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

void delay_ms(uint32_t ms)
{
    // Délai approximatif pour 4 MHz
    // Avec un CPU à 4 MHz et optimisation -O0:
    // Environ 1000 cycles par milliseconde
    for (uint32_t i = 0; i < ms * 1000; i++)
    {
        __NOP();
    }
}