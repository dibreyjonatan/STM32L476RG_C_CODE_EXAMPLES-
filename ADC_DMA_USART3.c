/*
This Code is to implement an ADC -> DMA -> USART3 system.
Here a user sends a command to get the value of an ADC converter.
The command creates an interrupt on the USART, which is used to request the specified data from the DMA.
The DMA stores value from the ADC continuously, i.e ADC1 -> RAM.
The DMA controller connects the ADC to the RAM 

*/
#include "stm32l476xx.h"
#include <stdio.h>
#define NUM_CHANNELS 4

volatile uint8_t rx_sel ;
volatile uint8_t rx_flag = 0; 
int val_CR ; 
 char data[50];

volatile uint16_t adc_buffer[NUM_CHANNELS];

void USART3_Init(void);
void USART3_SendChar(char c);
void USART3_SendString(const char *s);

void delay_ms(uint32_t ms);
void DMA_Init(void); 
void ADC_DMA_Init(void){
	
	ADC1->SQR1 &= ~ADC_SQR1_L_Msk;
    ADC1->SQR1 |= (3U << ADC_SQR1_L_Pos);
	
	ADC1->SQR1 &= ~ADC_SQR1_SQ1_Msk;
    ADC1->SQR1 |= (5U << ADC_SQR1_SQ1_Pos);

    ADC1->SQR1 &= ~ADC_SQR1_SQ2_Msk;
    ADC1->SQR1 |= (6U << ADC_SQR1_SQ2_Pos);

    ADC1->SQR1 &= ~ADC_SQR1_SQ3_Msk;
    ADC1->SQR1 |= (9U << ADC_SQR1_SQ3_Pos);

    ADC1->SQR1 &= ~ADC_SQR1_SQ4_Msk;
    ADC1->SQR1 |= (11U << ADC_SQR1_SQ4_Pos);

    /* Temps d'échantillonnage (safe default: SMP = 010 -> env. 24.5 cycles).
       - channels 0..9 dans SMPR1 (3 bits par channel)
       - channels 10..18 dans SMPR2
       Calculation: bit position = 3 * channel_index_in_register
    */
    /* Clear puis set pour CH5, CH6, CH9 dans SMPR1 */
    ADC1->SMPR1 &= ~(7U << (3U * 5U));
    ADC1->SMPR1 &= ~(7U << (3U * 6U));
    ADC1->SMPR1 &= ~(7U << (3U * 9U));
    ADC1->SMPR1 |=  (2U << (3U * 5U));
    ADC1->SMPR1 |=  (2U << (3U * 6U));
    ADC1->SMPR1 |=  (2U << (3U * 9U));

    /* CH11 est dans SMPR2, position 3*(11-10) = 3 */
    ADC1->SMPR2 &= ~(7U << (3U * (11U - 10U)));
    ADC1->SMPR2 |=  (2U << (3U * (11U - 10U)));

    /* Configure ADC pour conversions continues et active DMA circulaire côté ADC */
    ADC1->CFGR |= ADC_CFGR_CONT;    /* continuous mode */
    ADC1->CFGR |= ADC_CFGR_DMAEN;   /* enable DMA request after each EOC */
    ADC1->CFGR |= ADC_CFGR_DMACFG;  /* DMA circular mode on ADC side (periph->memory circular) */

}
void configure_GPIOA(void){
	
	// Activation des horloges ADC et GPIOA
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	
	//configuration de PA0, PA1, PA4, PA6
	 
    GPIOA->MODER |= (3U << (0*2)) | (3U << (1*2)) | (3U << (4*2)) | (3U << (6*2));
    GPIOA->PUPDR &= ~((3U << (0*2)) | (3U << (1*2)) | (3U << (4*2)) | (3U << (6*2)));
	
   
	//connect GPIOA to ADC
	 GPIOA->ASCR |= (1U<<0) | (1U<<1) | (1U<<4) | (1U<<6);
	
}
void init_ADC(void)
{
   
  ADC123_COMMON->CCR |= (1<<16); //HCLK as input clock, CK_Mode =01
  //ADC123_COMMON->CCR &= ~(1<<16); //if PLLSAI1 clock used
  ADC123_COMMON->CCR &= ~(1<<17);

  ADC123_COMMON->CCR &= ~(1<<21) & ~(1<<20) & ~(1<<19) & ~(1<<18); //input ADC clock divided by 1
	val_CR = ADC1->CR; //lecture pour vérifier ADC1->CR;
	
	if ((ADC1->CR & 0x20000000)==0x20000000){ //if in deep-power-down mode then disabled it 
		
		if ((ADC1->CR & 0x1) == 0x1){ //if ADC enabled then disable it to disable deep-power-down mode
			ADC1->CR |= (1UL<<1);         //ADDIS(ADC disable command), bit 1 disable the ADC
		}
	
		ADC1->CR &= ~(1UL<<29);   //Disable deep-power-down by setting DEEPPWD to 0, bit 29
	}
  
	val_CR = ADC1->CR; // lecture pour vérifier ADC1->CR = 0x00000000
	
 
	
	ADC_DMA_Init();
	
  ADC1->CR |= 0x10000000;  //Enable ADC voltage regulator by setting ADVREGEN to 1
	val_CR = ADC1->CR;
	while (!(ADC1->CR & ADC_CR_ADVREGEN));
  //systickDelayMs(1);       //Wait 1 ms to start-up ADC voltage regulator
    
  //ADC calibration(make sure ADCALDIF is in correct state, write 1 when needed calibration in differential input mode)
	ADC1->CR &= ~(1UL<<30); //ADCALDIF <- 0
//  systickDelayMs(10);     //Wait 10 ms to be sure diff calibration disabled ; not mandatory

	val_CR = ADC1->CR; //lecture pour vérifier ADC1->CR = 0x10000000
	
  ADC1->CR |= (1UL<<31);  //Start ADC calibration by setting ADCAL to 1
	while (ADC1->CR & ADC_CR_ADCAL){}; //	During calibration ADCAL=1 then wait ADCAL=0
	
	//systickDelayMs(10);     //Wait 10 ms to be sure ADC calibration is finished
    
    //Enable ADC if configured (calibration, deep power mode off, voltage regulator on)
  ADC1->ISR |= 1;          //Clear the ADC bit in ADC_ISR by writing 1 to ADRDY

  ADC1->CR |= 1;           //Enable ADC by setting ADEN to 1
	while (!(ADC1->ISR & ADC_ISR_ADRDY)); // wait until ADRDY=1
	//while ((ADC1->ISR & ADC_ISR_ADRDY) != (1UL<<0)); // idem, wait until ADRDY=1

	val_CR = ADC1->CR; //lecture pour vérifier ADC1->CR = 0x10000001;
	
}

uint16_t ADC1_Read(uint16_t channel) {
    // 1. Sélection du canal
	  // le canal selectionner sera le premier converti dans la sequence 
	
	 ADC1->SQR1 &= ~ADC_SQR1_SQ1_Msk;
    ADC1->SQR1 |= (channel << ADC_SQR1_SQ1_Pos);
	 // 2. Démarrer la conversion
    ADC1->CR |= ADC_CR_ADSTART;
	    // 3. Attendre la fin
    while (!(ADC1->ISR & ADC_ISR_EOC));
    return ADC1->DR; //lire la donnée qui est comprise entre 0 -4095 
	

}
void Select_value_to_send(uint8_t t){
	float volt=adc_buffer[t] * 3.3f / 4095.0f; ;
	int unit, decimal ; 
	char u, d ; 
			
		         unit = (int)volt;                // partie entière
         decimal = ((int)((volt) * 10))%10 ; // premier chiffre après la virgule

        u = '0' + unit;     // conversion en ASCII
        d = '0' + decimal;  // conversion en ASCII

        USART3_SendChar(u);
        USART3_SendChar('.');
        USART3_SendChar(d);
        USART3_SendChar('\r');
        USART3_SendChar('\n');
		        
			      sprintf(data, "volt: %.2f\r\n",volt);
            USART3_SendString(data);

	
}

int main(void) {
	 
	configure_GPIOA() ;
	
   init_ADC() ;
    USART3_Init();
	  DMA_Init();
    USART3_SendString("=== USART3 ready @19200 ===\r\n");
    USART3_SendString("For PA0 send 1, PA1 send 2, PA4 send 3, PA6 send 4 \r\n");
    
	__enable_irq(); 
    
			
		ADC1->CR |= ADC_CR_ADSTART; //Démarre la conversion 

    /* Boucle principale : lecture du buffer rempli par le DMA et envoi sur UART */
    while (1)
    {
			
			if(rx_flag){
				
				 if(rx_sel>0 && rx_sel<=4)
				 Select_value_to_send(rx_sel-1) ;
				 
				rx_flag=0 ;
			}
    }

}

void DMA_Init(void)
{
    /* activer horloge DMA1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* désactiver le canal avant config (sécurité) */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    while (DMA1_Channel1->CCR & DMA_CCR_EN) {}

    /* adresse du registre DATA de l'ADC */
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;

    /* adresse mémoire : adr du tableau adc_buffer */
    DMA1_Channel1->CMAR = (uint32_t)adc_buffer;

    /* nombre de transferts = nombre de canaux dans la séquence (4) */
    DMA1_Channel1->CNDTR = NUM_CHANNELS;

    /* Reset CCR puis configurer :
       - MINC : increment memory
       - CIRC : circular mode
       - PSIZE/MSIZE = 16 bits (value 01)
    */
    DMA1_Channel1->CCR = 0;
    DMA1_Channel1->CCR |= DMA_CCR_MINC;  /* incrément mémoire */
    DMA1_Channel1->CCR |= DMA_CCR_CIRC;  /* circular */
    /* PSIZE = 01 -> set bit8, MSIZE = 01 -> set bit10 (macro DMA_CCR_PSIZE_0 possible) */
    DMA1_Channel1->CCR &= ~(DMA_CCR_PSIZE | DMA_CCR_MSIZE);
    DMA1_Channel1->CCR |= (1U << 8);   /* PSIZE = 16-bit */
    DMA1_Channel1->CCR |= (1U << 10);  /* MSIZE = 16-bit */

    /* (optionnel) activer TCIE si tu veux une interruption à chaque "bloc" complet */
    /* DMA1_Channel1->CCR |= DMA_CCR_TCIE; */

    /* mapping request: s'assurer que Channel1 est associé à ADC (CSELR) :
       sur la plupart des MCU L4, ADC1 request pour Channel1 = 0 ; on laisse 0.
    */
    DMA1_CSELR->CSELR &= ~DMA_CSELR_C1S_Msk;
    DMA1_CSELR->CSELR |= (0U << DMA_CSELR_C1S_Pos);

    /* enfin activer le canal DMA */
    DMA1_Channel1->CCR |= DMA_CCR_EN;
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
         	rx_sel= USART3->RDR & 0xFF;
			    rx_sel-='0' ; 
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

void delay_ms(uint32_t ms)
{
    /* approximatif : dépend du clock système (ici HSI16 à 16MHz) */
    for (volatile uint32_t i = 0; i < ms * 16000U; ++i) {
        __NOP();
    }
}