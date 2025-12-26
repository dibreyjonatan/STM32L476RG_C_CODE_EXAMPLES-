#include <stm32l476xx.h>

#define MAX_CYCLE_SIZE 20
#define FREQ 4000000

// volatile uint8_t cycle_buffer[MAX_CYCLE_SIZE];
volatile uint8_t cycle_index = 0;
volatile uint8_t cycle_length = 0;
volatile uint8_t state = 0; // 0 = attente longueur, 1 = réception cycle
volatile uint8_t c ;
	
int cycle_buffer[MAX_CYCLE_SIZE]={0, 8, 12, 4, 6, 2, 3, 1} ;
int8_t index=0 ;
int8_t size=8 ; // pour commencer on va utiliser une taille de 8
 volatile uint16_t ARR_temps=7441; // Avec un ARR qui correspond à 240ms pour freq=4MHz et PSC=128


void init_LED(void) {

	 // Activation de l'horloge du peripherique PORTA
	RCC->AHB2ENR|=(1<<0);
	GPIOA->MODER&= 0xFFFC03FF ;
	GPIOA->MODER|=(1<<10) ;
	GPIOA->MODER|=(1<<12);
	GPIOA->MODER|=(1<<14) ;
	GPIOA->MODER|=(1<<16) ;
	
}


void init_BP(void){
	// Activation de l'horloge du peripherique PORTB
	RCC->AHB2ENR|=(1<<1);
	// Entrée numérique 
	GPIOB->MODER&= ~(0xF<<6);
	// On met les buttons en pull up 
	GPIOB->PUPDR &= ~(0xF<<6);
	 GPIOB->PUPDR |= (1<<6) ;
	GPIOB->PUPDR |= (1<<8) ;
	GPIOB->PUPDR |= (1<<10) ;
	GPIOB->PUPDR |= (1<<12) ;

	
}

void update_TIMER2(uint16_t ARR_time) {
    // Désactiver le timer 2 (avant de le configurer)
    TIM2->CR1 &= ~TIM_CR1_CEN;  // Désactivation du compteur (CEN = 0)

    // Réinitialiser le compteur
    TIM2->CNT = 0;

    // Configurer le prescaler (PSC) 
    TIM2->PSC = 128;  // Exemple : division par 128 (ajuster en fonction de ta fréquence)

    // Calcul du registre ARR (Auto-Reload Register)
    // ARR= (time * FREQ) / (PSC + 1) - 1 pour ajuster le délai
    TIM2->ARR =ARR_time ; 
    // Update generation 
	  TIM2->EGR |= TIM_EGR_UG;  
    // Activer le timer 2
    TIM2->CR1 |= TIM_CR1_CEN;  // Activation du compteur (CEN = 1) 
}

// l'erreur était au niveau de l'affectation du TIM2->CR1=1 c'est faux car en faisant cela on écrase d'autres configurations 
// Pareil pour 	TIM2->DIER =1 ; on écrase d'autres configurations 
void init_Interrupt_Timer(void){


	TIM2->DIER |=1 ;  //Activate timer interrupt
 	NVIC_SetPriority(TIM2_IRQn, 4);
		NVIC_EnableIRQ(TIM2_IRQn);  
}
void TIM2_IRQHandler(void){
	  index++ ;
	 if(index==size) index=0 ;
	switch(cycle_buffer[index]){
			case 0 : 
				     GPIOA->BSRR = (1 << (5 + 16));  
			       GPIOA->BSRR = (1 << (6 + 16));  
			GPIOA->BSRR = (1 << (7 + 16));  
			GPIOA->BSRR = (1 << (8 + 16));  
				       break ;
			case 8 : 
				     GPIOA->BSRR = (1 << (5 + 0));  
			       GPIOA->BSRR = (1 << (6 + 16));  
			      GPIOA->BSRR = (1 << (7 + 16));  
			     GPIOA->BSRR = (1 << (8 + 16));  
				       break ;
			case 12: 
				     GPIOA->BSRR = (1 << (5 + 0));  
			       GPIOA->BSRR = (1 << (6 + 0));  
			GPIOA->BSRR = (1 << (7 + 16));  
			GPIOA->BSRR = (1 << (8 + 16));  
				       break ;
			case 4 : 
				     GPIOA->BSRR = (1 << (5 + 16));  
			       GPIOA->BSRR = (1 << (6 + 0));  
			GPIOA->BSRR = (1 << (7 + 16));  
			GPIOA->BSRR = (1 << (8 + 16));  
				       break ;
			case 6 : 
				     GPIOA->BSRR = (1 << (5 + 16));  
			       GPIOA->BSRR = (1 << (6 + 0));  
			GPIOA->BSRR = (1 << (7 + 0));  
			GPIOA->BSRR = (1 << (8 + 16));  
				       break ;
			case 2 : 
				     GPIOA->BSRR = (1 << (5 + 16));  
			       GPIOA->BSRR = (1 << (6 + 16));  
			GPIOA->BSRR = (1 << (7 + 0));  
			GPIOA->BSRR = (1 << (8 + 16));  
				       break ;
			case 3: 
				     GPIOA->BSRR = (1 << (5 + 16));  
			       GPIOA->BSRR = (1 << (6 + 16));  
			GPIOA->BSRR = (1 << (7 + 0));  
			GPIOA->BSRR = (1 << (8 + 0));  
				       break ;
			case 1 : 
				     GPIOA->BSRR = (1 << (5 + 16));  
			       GPIOA->BSRR = (1 << (6 + 16));  
			GPIOA->BSRR = (1 << (7 + 16));  
			GPIOA->BSRR = (1 << (8 + 0));  
				       break ;
	}
	
	TIM2->SR &=0xFE ;
}
void USART3_Init(void) {
    // 1. Activer les horloges pour GPIOB et USART3
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

    // 2. Configurer PB10 (TX) et PB11 (RX) en Alternate Function AF7
    //    MODER: 10 = Alternate function
    GPIOB->MODER &= ~((3U << 20) | (3U << 22));  // Clear MODER10, MODER11
    GPIOB->MODER |=  ((2U << 20) | (2U << 22));  // Set to AF mode

    //    AFRH: bits [3:0] par pin — AF7 pour USART3
	 // AFRH10 et AFRH11 viennent du fait qu'on utilise PB10 et PB11
	 // AFR[1] Correspond à AFRH
    GPIOB->AFR[1] &= ~((0xF << 8) | (0xF << 12));  // Clear AFRH10, AFRH11
    GPIOB->AFR[1] |=  ((7U << 8) | (7U << 12));    // Set AF7

    // 3. Configurer USART3
    USART3->CR1 &= ~USART_CR1_UE;   // Désactiver USART pour configuration
    USART3->BRR = 209;              // Baudrate = 19200 à 4 MHz (Fck / Baud)

    // Activer RX, TX et l'interruption de réception
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    USART3->CR1 |= USART_CR1_UE;    // Activer USART3

    // 4. Activer l’interruption NVIC

		NVIC_SetPriority(USART3_IRQn, 3);  
        NVIC_EnableIRQ(USART3_IRQn);
}


void USART3_IRQHandler(void) {	
    if (USART3->ISR & USART_ISR_RXNE) { // Si nouveau caractère reçu
         	
			TIM2->CR1 &= ~TIM_CR1_CEN; 
			
			c = USART3->RDR ; //& 0xFF;
      c=c-'0' ; // Pour convertir les characters envoyé en numérique 
			 
        if (state == 0) {
            cycle_length = c;          // Premier octet = longueur
            cycle_index = 0;
            if (cycle_length > MAX_CYCLE_SIZE) {
                cycle_length = MAX_CYCLE_SIZE; // sécurité
            }
            state = 1;                 // passer en mode réception cycle
			
			        // Desactiver la chinièree     
					 
        }
		else if (state == 1) {
            cycle_buffer[cycle_index] = c;
			cycle_index++ ;
            if (cycle_index == cycle_length ) {  // vu qu'on commence à 0, la taille c'est cycle_length-1
                state = 0;             // retour à l’état attente longueur
								cycle_index=0 ;  // Pour recommencer à partir de la prémière période du premier cycle 
								index=0 ; //reinitialiser le cycle à 0, la chinièree doit recommencer à la prémière période 
								size=cycle_length ;	
					 TIM2->CR1 |= TIM_CR1_CEN;  // Réactiver la chinièree après reception du cycle 
								
							
            }
        }
    }
			//		TIM2->CR1 |= TIM_CR1_CEN; 

}

void Configure_Increment_BP(void){
	 /* Enable SYSCFG clock! */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	//EXTI3
	// SYSCFG->EXTICR[0]  correspond à EXTICR1 dans la datasheet
	SYSCFG->EXTICR[0] |= (1<<12) ;  //  0001: PB[3] pin 
	EXTI->IMR1 |= (1<<3); /* unmask EXTI3 */
    EXTI->FTSR1 |= (1<<3); /* Activate trigger on falling edge */
	NVIC_EnableIRQ(EXTI3_IRQn);  
	NVIC_SetPriority(EXTI3_IRQn, 1);  
}

void EXTI3_IRQHandler(void){
	//pour le button poussoir increment PB3
	ARR_temps*=2 ;
	TIM2->CR1 &= ~TIM_CR1_CEN; 
	if(ARR_temps > 29767 ) ARR_temps =29767 ; //Ceci est un ARR qui correspond à un temps de 960ms
	 
	update_TIMER2(ARR_temps) ;
	for( int i =0 ; i<20000 ; i++ ) ;
	EXTI->PR1 |= (1<<3 ) ; // mise à 1 pour baisser le drapeau d'interruption 
	
}

void Configure_Decrement_BP(void){
	// EXTI4
	// SYSCFG->EXTICR[1]  correspond à EXTICR2 dans la datasheet 
	SYSCFG->EXTICR[1] |= (1 << 0);  // 0001: PB4 pin (EXTI4)
    EXTI->IMR1 |= (1 << 4);  // Unmask EXTI4  on fait décalage à droite pour la ligne 4
    EXTI->FTSR1 |= (1<<4); /* Activate trigger on falling edge */
    NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_SetPriority(EXTI4_IRQn, 2); 
}

void EXTI4_IRQHandler(void){
	//pour le button poussoir decrement PB4
	ARR_temps/=2 ;
	if(ARR_temps<1860 ) ARR_temps =1860;  // ARR qui correspond à un temps de 60ms 
	
	update_TIMER2(ARR_temps) ;
	for( int i =0 ; i<20000 ; i++ ) ; 
	EXTI->PR1  |= (1<<4) ;  
	
}

void init_TIM2(void) {
	// calcul pout duree =240 ms
   RCC->APB1ENR1 |= (1<<0);
	 TIM2->CR1 &= ~TIM_CR1_CEN; // disable timer2 (à vérifier)
	 TIM2->CNT=0 ;
	 TIM2->PSC= 128 ; // la valeur de PSC
	 TIM2->ARR= 7441 ; // calcul d'arr
	TIM2->CR1 |=TIM_CR1_CEN ; // enable timer2 (à vérifier)
	
	
}
int main() {
	
	init_LED();
	init_BP();
	init_TIM2() ;
	init_Interrupt_Timer(); 
	Configure_Increment_BP();
	Configure_Decrement_BP();
   USART3_Init();
	__enable_irq();  //Pour activer les interruptions globales 
	
	while(1){
		
}
	
	}

