#include "stm32l476xx.h"
#include <stdio.h>

volatile uint8_t rx_sel;
volatile uint8_t rx_flag = 0;
volatile uint8_t adc_conversion_complete = 0;
volatile uint16_t adc_value = 0;
int val_CR;
char data[50];

void USART3_Init(void);
void USART3_SendChar(char c);
void USART3_SendString(const char *s);

void configure_GPIOA(void){
    // Activation des horloges ADC et GPIOA
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	
    
    // Configuration de PA0, PA1, PA4, PA6 en mode analogique
    GPIOA->MODER |= (3U << (0*2)) | (3U << (1*2)) | (3U << (4*2)) | (3U << (6*2));
    GPIOA->PUPDR &= ~((3U << (0*2)) | (3U << (1*2)) | (3U << (4*2)) | (3U << (6*2)));
    
    // Connect GPIOA to ADC
    GPIOA->ASCR |= (1U<<0) | (1U<<1) | (1U<<4) | (1U<<6);
}

void init_ADC(void)
{
    ADC123_COMMON->CCR |= (1<<16); // HCLK as input clock, CK_Mode =01
    ADC123_COMMON->CCR &= ~(1<<17);
    ADC123_COMMON->CCR &= ~(1<<21) & ~(1<<20) & ~(1<<19) & ~(1<<18); // input ADC clock divided by 1
    
    val_CR = ADC1->CR;
    
    if ((ADC1->CR & 0x20000000)==0x20000000){ // if in deep-power-down mode then disabled it 
        if ((ADC1->CR & 0x1) == 0x1){ // if ADC enabled then disable it
            ADC1->CR |= (1UL<<1);     // ADDIS(ADC disable command)
        }
        ADC1->CR &= ~(1UL<<29);   // Disable deep-power-down
    }
    
    val_CR = ADC1->CR;
    
    ADC1->SQR1 &= 0xfffffff0; // Single conversion
    
    ADC1->CR |= 0x10000000;  // Enable ADC voltage regulator
    val_CR = ADC1->CR;
    while (!(ADC1->CR & ADC_CR_ADVREGEN));
    
    // ADC calibration
    ADC1->CR &= ~(1UL<<30); // ADCALDIF <- 0
    val_CR = ADC1->CR;
    
    ADC1->CR |= (1UL<<31);  // Start ADC calibration
    while (ADC1->CR & ADC_CR_ADCAL){}; // Wait ADCAL=0
    
    // Enable ADC
    ADC1->ISR |= 1;          // Clear ADRDY bit
    ADC1->CR |= 1;           // Enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    
    val_CR = ADC1->CR;
    
    // *** CONFIGURATION DES INTERRUPTIONS ADC ***
    ADC1->IER |= ADC_IER_EOCIE;  // Enable End Of Conversion interrupt
    
    // Configuration de la priorité (supérieure à USART3)
    NVIC_SetPriority(ADC1_2_IRQn, 3);  // Priorité 3 (plus basse que USART3 qui est à 2)
    NVIC_EnableIRQ(ADC1_2_IRQn);       // Enable ADC interrupt in NVIC
}

void ADC1_StartConversion(uint16_t channel) {
    // Sélection du canal
    ADC1->SQR1 &= ~ADC_SQR1_SQ1_Msk;
    ADC1->SQR1 |= (channel << ADC_SQR1_SQ1_Pos);
    
    // Reset du flag de conversion complète
    adc_conversion_complete = 0;
    
    // Démarrer la conversion
    ADC1->CR |= ADC_CR_ADSTART;
}

// *** HANDLER D'INTERRUPTION ADC ***
void ADC1_2_IRQHandler(void) {
    if (ADC1->ISR & ADC_ISR_EOC) {  // End of conversion
        adc_value = ADC1->DR;        // Lire la valeur (efface aussi le flag EOC)
        adc_conversion_complete = 1; // Signaler que la conversion est terminée
    }
}

void Select_converter(uint8_t t){
    float volt;
    int unit, decimal;
    char u, d;
    uint16_t channel;
    
    // Déterminer le canal à utiliser
    switch(t) {
        case 1: channel = 5;  break;  // PA0 -> ADC_IN5
        case 2: channel = 6;  break;  // PA1 -> ADC_IN6
        case 3: channel = 9;  break;  // PA4 -> ADC_IN9
        case 4: channel = 11; break;  // PA6 -> ADC_IN11
        
    }
    
    // Démarrer la conversion ADC par interruption
    ADC1_StartConversion(channel);
    
    // Attendre la fin de la conversion (flag mis à jour dans l'interruption)
    while (!adc_conversion_complete);
    
    // Calculer la tension
    volt = (adc_value * 3.3f) / 4095.0f;
    unit = (int)volt;                          // Partie entière
    decimal = ((int)((volt) * 10)) % 10;       // Premier chiffre après la virgule
    
    u = '0' + unit;     // Conversion en ASCII
    d = '0' + decimal;  // Conversion en ASCII
    
    // Envoyer les données
    USART3_SendChar(u);
    USART3_SendChar('.');
    USART3_SendChar(d);
    USART3_SendChar('\r');
    USART3_SendChar('\n');
    
    sprintf(data, "volt: %.2f\r\n", volt);
    USART3_SendString(data);
}

int main(void) {
    configure_GPIOA();
    init_ADC();
    USART3_Init();
    
    USART3_SendString("=== USART3 ready @19200 (ADC par interruption) ===\r\n");
    
    __enable_irq(); 
    
    while (1) {
        if(rx_flag){
            // Ne faire la sélection que si on est dans l'intervalle 
            if(rx_sel > 0 && rx_sel < 5)
                Select_converter(rx_sel);
            
            rx_flag = 0; // Reset flag
        }
    }
}

/* ======= INITIALISATION USART3 ======= */
void USART3_Init(void) {
    // 1. Activer horloges GPIOB et USART3
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

    // 2. Configurer PB10 (TX) et PB11 (RX) en mode alternatif AF7
    GPIOB->MODER &= ~((3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->MODER |=  ((2U << (10 * 2)) | (2U << (11 * 2)));

    GPIOB->AFR[1] &= ~((0xF << ((10 - 8) * 4)) | (0xF << ((11 - 8) * 4)));
    GPIOB->AFR[1] |=  ((7U << ((10 - 8) * 4)) | (7U << ((11 - 8) * 4)));

    // 3. Configurer USART3
    USART3->CR1 &= ~USART_CR1_UE;
    USART3->BRR = 4000000 / 19200;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
   
    // Priorité 2 pour USART3 (plus prioritaire que ADC qui est à 3)
    NVIC_SetPriority(USART3_IRQn, 2);  
    NVIC_EnableIRQ(USART3_IRQn);
    
    USART3->CR1 |= USART_CR1_UE;
}

void USART3_IRQHandler(void) {	
    if (USART3->ISR & USART_ISR_RXNE) {
        rx_sel = USART3->RDR & 0xFF;
        rx_sel -= '0'; 
        rx_flag = 1; 
    }
}		
						
/* ======= ENVOI ======= */
void USART3_SendChar(char c) {
	while (!(USART3->ISR & USART_ISR_TXE)){} ;  // Si la donnée n'est pas transferrer au registre, ne rien faire (attendre) 
    USART3->TDR = c;
}

void USART3_SendString(const char *s) {
    while (*s) USART3_SendChar(*s++);
}