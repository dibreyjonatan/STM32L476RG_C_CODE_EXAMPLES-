#include "stm32l476xx.h"
#include <stdio.h>

volatile char rx_char;
volatile uint8_t rx_flag = 0; 

 char data[50];
void USART3_Init(void);
void USART3_SendChar(char c);
void USART3_SendString(const char *s);

int main(void) {
    USART3_Init();
	
    USART3_SendString("=== USART3 ready @19200 ===\r\n");
    
	__enable_irq(); 
    while (1) {
			
			if(rx_flag){
				
				 sprintf(data, "Recu: %c\r\n", rx_char);
            USART3_SendString(data);
            rx_flag = 0; // reset flag
			}
    }
}

/* ======= INITIALISATION USART3 ======= */
// pour l'usart 
//  TXE c'est le flag qui se lève si le registre TDX est vide 
// RNXE c'est le flag qui se lève si le registre RDX est rempli 

void USART3_Init(void) {
    // 1. Activer horloges GPIOB et USART3
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

    // 2. Configurer PB10 (TX) et PB11 (RX) en mode alternatif AF7
    GPIOB->MODER &= ~((3U << (10 * 2)) | (3U << (11 * 2))); // Clear bits
    GPIOB->MODER |=  ((2U << (10 * 2)) | (2U << (11 * 2))); // Mode AF

    GPIOB->AFR[1] &= ~((0xF << ((10 - 8) * 4)) | (0xF << ((11 - 8) * 4)));
    GPIOB->AFR[1] |=  ((7U << ((10 - 8) * 4)) | (7U << ((11 - 8) * 4))); // AF7 = USART3

    // 3. Configurer USART3
    USART3->CR1 &= ~USART_CR1_UE;       // Désactiver USART
    USART3->BRR = 4000000 / 19200;      // Baud rate (si HSI = 4 MHz)
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;       // Activer la transmission seulement
   
 	     NVIC_SetPriority(USART3_IRQn, 2);  
        NVIC_EnableIRQ(USART3_IRQn);
	
	USART3->CR1 |= USART_CR1_UE;        // Activer USART3
}

 

void USART3_IRQHandler(void) {	
    if (USART3->ISR & USART_ISR_RXNE) { // Si nouveau caractère reçu
         	rx_char= USART3->RDR & 0xFF;
			    rx_flag=1 ; 
		}

	}		
						
/* ======= ENVOI ======= */
void USART3_SendChar(char c) {
	
	// Tant que la donné n'est pas envoyé sur le registre TDR, il faut rester dans la boucle 
    while (!(USART3->ISR & USART_ISR_TXE)); // Attente TXE=1
    USART3->TDR = c;
}

void USART3_SendString(const char *s) {
    while (*s) USART3_SendChar(*s++);
}
