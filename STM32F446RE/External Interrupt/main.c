#include "RccConfig.h"
int flag = 0;
int count = 0;
void  EXTI15_10_IRQHandler (void);
void GPIO_Config(void);
void GPIO_Config(void)
{
	//gpio clock for A and C
	RCC->AHB1ENR |=(1<<0); //ENABLE GPIOA CLOCK
	GPIOA->MODER |= (1<<10); 
	GPIOA->OTYPER =0;
	GPIOA->OSPEEDR=0;
	
	RCC->AHB1ENR |=(1<<2); //ENABLE GPIOC CLOCK
	GPIOC->MODER &=(0<<26); 
	GPIOC->PUPDR |= (1<<26);
	
}

void  Interrupt_Config(void)
{
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. Enable the SYSCFG/AFIO bit in RCC register 
	2. Configure the EXTI configuration Register in the SYSCFG/AFIO
	3. Disable the EXTI Mask using Interrupt Mask Register (IMR)
	4. Configure the Rising Edge / Falling Edge Trigger
	5. Set the Interrupt Priority
	6. Enable the interrupt
	
	********************************************************/
	
	RCC->APB2ENR |= (1<<14);  // Enable SYSCNFG
	SYSCFG->EXTICR[3] |= (1<<5);  // Bits[7:6:5:4] = (0:0:1:0)  -> configure EXTI3 line for PC13
	EXTI->IMR |= (1<<13);
	EXTI->RTSR |= (1<<13);  // Enable Rising Edge Trigger for PC13
	EXTI->FTSR &= ~(1<<13);  // Disable Falling Edge Trigger for PC13
	NVIC_SetPriority (EXTI15_10_IRQn, 1);  // Set Priority
	NVIC_EnableIRQ (EXTI15_10_IRQn);  // Enable Interrupt
}


void  EXTI15_10_IRQHandler (void)
{
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	1. Check the Pin, which triggerred the Interrupt
	2. Clear the Interrupt Pending Bit
	
	********************************************************/
	if(EXTI->PR & (1<<13))
	{
		flag=1;
		EXTI->PR |= (1<<13);	
	}
}



void delay_us(uint32_t delay_us)
{
  volatile unsigned int num;
  volatile unsigned int t;


  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  } 
}
//Millisecond delay
void delay_ms(uint16_t delay_ms)
{
  volatile unsigned int num;
  for (num = 0; num < delay_ms; num++)
  {
    delay_us(1000);
  }
}

int main ()
{
	SysClockConfig();
	GPIO_Config();
	Interrupt_Config();
	while(1)
	{
		if(flag)
		{
			delay_ms(1000);
			count++;
			flag=0;
		}
	}
	
}