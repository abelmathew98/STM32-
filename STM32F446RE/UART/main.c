#include "stm32f446xx.h"  //Decomposed the .h file
#include<stdlib.h>


//PLL Constant values
#define PLL_M 	4
#define PLL_N		180
#define PLL_P		0    //instead of directly giving the value , we have to follow the value given in REF manual.O corresponds to 2


void UART_SendChar(uint8_t c);
void UART_SendString (char  *c);
uint8_t UART_GetChar(void);
void Delay_us(uint16_t us);
void Delay_ms (uint16_t ms);
void SysClockConfig(void);


void SysClockConfig(void)
{
	 /*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. ENABLE HSE and wait for the HSE to become Ready
	2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	3. Configure the FLASH PREFETCH and the LATENCY Related Settings
	4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
	5. Configure the MAIN PLL
	6. Enable the PLL and wait for it to become ready
	7. Select the Clock Source and wait for it to be set
	
	********************************************************/
	
	//Step 1
	//RCC->CR |=  RCC_CR_HSEON;
	//while(!(RCC->CR & RCC_CR_HSERDY)); // when the HSE is ready, it breaks out of the loop
	RCC->CR |= 1<<16;  
	while (!(RCC->CR & (1<<17)));
	
	//Step 2
	//RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	//PWR->CR |= PWR_CR_VOS; // C=11 corresponds to the Scale MODE 1 RESET VALUE
	RCC->APB1ENR |= 1<<28;
	PWR->CR |= 3<<14; //11
	
	//STEP 3
	//FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN; //  Parameter settings RCC in Cube IDE
	FLASH->ACR = (1<<8) | (1<<9)| (1<<10)| (5<<0);
	//STEP 4
	//AHB PRESCALER
	//RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	
	//APB1 prescaler
	//RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	
	//APB2 prescaler
	//RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	
	// AHB PR
	RCC->CFGR &= ~(1<<4);
	
	// APB1 PR
	RCC->CFGR |= (5<<10);
	
	// APB2 PR
	RCC->CFGR |= (4<<13);
	
	//STEP 5
	// PLLM starts at 0 bit | PLLN starts at 6 | PLLP starts at 16 |PLLSRC(HSE) at 22
	//RCC->PLLCFGR = (PLL_M<<0)|(PLL_N<<6)|(PLL_P<<16)|(RCC_PLLCFGR_PLLSRC_HSE);
	RCC->PLLCFGR = (PLL_M <<0) | (PLL_N << 6) | (PLL_P <<16) | (1<<22);
	//STEP 6
	//RCC->CR |=RCC_CR_PLLON;
	
	//while(!(RCC_CR_PLLRDY));
	
	RCC->CR |= (1<<24);
	while (!(RCC->CR & (1<<25)));
	
	//STEP 7
	//RCC->CFGR |= RCC_CFGR_SW_PLL;
	
	//while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	RCC->CFGR |= (2<<0);
	while (!(RCC->CFGR & (2<<2)));
	
} 

void GPIO_Config(void)
{
	//Step 1 :ENABLE GPIO CLOCK
	RCC->AHB1ENR |= (1<<0);
	
	//SET THE PIN AS OUTPUT
	GPIOA->MODER |= (1<<10); //PIN A5 at bits 11 and 10 as Output (01)
	
	//configure the output mode
	GPIOA->OTYPER &=~(1<<5); //Output type default : push pull
	
	GPIOA->OSPEEDR=0; //DEFAULT LOW SPEED
	
	GPIOA->PUPDR &= ~(1<<10);
	GPIOA->PUPDR &= ~(1<<11);
	

	
}
void TIM6Config (void)
{
	//step1
	RCC->APB1ENR |=(1<<4); //enable the timer6 clock
	//step2
	TIM6->PSC =90-1; // 90MHZ /(90-1)+1 =1Mhz which is 1us
	TIM6->ARR=0xffff;
	
	//ENABLE TIMER
	TIM6->CR1 |=(1<<0);
	while(!(TIM6->SR & (1<<0)));
	
}




static void UART2Config(void)
{
	//Step 1: Enable the UART and GPIO clock (to which RX AND TX are assigned) 
	RCC->APB1ENR |= (1<<17); //ENABLE UART2 CLOCK 
	RCC->AHB1ENR |= (1<<0); //ENABLE GPIO A CLOCK
	
	//STEP 2:  Configure the UART PINs for Alternate Functions
	GPIOA->MODER |= (1<<5)|(0<<4); // bits 5:4 as 1:0 for PIN PA2 Alternate Function
	GPIOA->MODER |= (1<<7)|(0<<6); // bits 7:6 as 1:0 for PIN PA3 Alternate Function
	
	// STEP 3 : ENABLE High speed at the pins PA2 AND PA3
	GPIOA->OSPEEDR |=(1<<5)|(1<<4);
	GPIOA->OSPEEDR |=(1<<7)|(1<<6);
	
	// STEP 4 : ALTERNATE FUNCTION REGISTER AF7 FOR USART IN AFRL REGISTER  AF7 = 0111 
	GPIOA->AFR[0] |= (0<<11)|(1<<10)|(1<<9)|(1<<8);  // AF7 ALTERNATE FUNCTION FOR PIN A2
	GPIOA->AFR[0] |= (0<<15)|(1<<14)|(1<<13)|(1<<12);  //AF7 ALTERNATE FUNCTION FOR PIN A3
	
	//STEP 5 :ENABLE THE UART  : UE BIT USART_CR1 REGISTER TO 1
	USART2->CR1 |=0x00; //clear all bits
	USART2->CR1 |=(1<<13); //  UE=1  USART enabled
	
	//STEP 6 :M BIT WORD LENGTH
	USART2->CR1 &=  ~(1<<12); // M BIT =0  -> 8 BIT 
	
	//STEP 7 : SELECT THE USART BAUD RATE (USART_BRR)
	USART2->BRR |= (7<<0)|(24<<4); //BAUD RATE 115200 ,FREQUENCT OF 45mhz at apb1
	
	//STEP 8 : ENABLE  THE TRANSMITTER/RECIEVER  BY SETTING TE AND RE BITS OF  USART_CR1 REGISTER
	USART2->CR1 |=(1<<2); //RE=1
	USART2->CR1 |=(1<<3); //TE=1
	
	// for interrupt 
	//USART2->CR1 |=(1<<6); //TCIE=1
	//
	//USART2->CR1 |=(1<<5); //RXNEIE=1
} 



void Delay_us(uint16_t us)
{
	TIM6->CNT=0;
	while(TIM6->CNT <us);
}
void Delay_ms (uint16_t ms)
{
	for (uint16_t i=0; i<ms; i++)
	{
		Delay_us (1000); // delay of 1 ms
	}
}
void UART_SendChar(uint8_t c)
{
	//STEP 1 : copy the data to the USART_DR register 
	USART2->DR = c; 
	//STEP 2 : CHECK IF TRANSMISSION IS COMPLETED BY CHECKING IN THE USART_SR (STATUS REGISTER) TC BIT
	while(!(USART2->SR & (1<<6)));
	
}

void UART_SendString (char  *c)
{
	while(*c)
	{
		UART_SendChar(*c++); 
	}		
	
}

uint8_t UART_GetChar(void)
{
	//STEP 1 : TO INITIALIZE A TEMP VARIABLE TO READ 
	uint8_t temp;
	
	//STEP 2 : CHECK FOR RE BIT =1 ; THEN THERE IS A DATA TO BE RECIEVED BY THE RDR
	while(!(USART2->SR & (1<<5)));
	
	//STEP 3 : READ THE DATA FROM DATA REGISTER 
	temp =USART2->DR; //CLEARS THE RXE BIT ALSO
	
	return temp;
	
}

void USART2_IRQHandler(void)
{
	if (USART2->SR & (1<<6))
	{
		GPIOA->BSRR |= (1<<5); 
		Delay_ms(500);
		GPIOA->BSRR |= (1<<21); 
		Delay_ms(500);
	USART2->SR &=(0<<6);
	}

	//if (USART2->SR & (1<<5))
	//{
	//	char temp = USART2->DR;
	//	USART2->DR = temp;
	//	while (!(USART2->SR & (1<<6)));
	//}
}


int main(void)
 {
	SysClockConfig();
	TIM6Config();
	GPIO_Config();
	UART2Config();
	USART2->CR1|=0x80;
	//USART2->CR1 |=(1<<7); //TXEIE=1
	NVIC_SetPriority(USART2_IRQn,1);
	NVIC_EnableIRQ(USART2_IRQn);
	__enable_irq();
	UART_SendString("Hello\n");
	while(1)	
	{
		//uint8_t data = UART_GetChar();
		//UART_SendChar(data);
		
//		Delay_ms(1000);
		
	}
}