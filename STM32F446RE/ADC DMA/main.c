


#include "RccConfig.h"
#include "Delay.h"

void DMA_Config(uint32_t srcAdd,uint32_t destAdd, uint8_t size);
void ADC_Init (void);
void ADC_Enable (void);
void ADC_Start (void);
void DMA_Init(void);
void DMA_Config(uint32_t srcAdd,uint32_t destAdd, uint8_t size);

void ADC_Init (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable ADC and GPIO clock
	2. Set the prescalar in the Common Control Register (CCR)
	3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
	4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	5. Set the Sampling Time for the channels in ADC_SMPRx
	6. Set the Regular channel sequence length in ADC_SQR1
	7. Set the Respective GPIO PINs in the Analog Mode
	************************************************/
	
//1. Enable ADC and GPIO clock
	RCC->APB2ENR |= (1<<8);  // enable ADC1 clock
	RCC->AHB1ENR |= (1<<0);  // enable GPIOA clock
	
//2. Set the prescalar in the Common Control Register (CCR)	
	ADC->CCR |= 1<<16;  		 // PCLK2 divide by 4
	
//3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)	
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 &= ~(1<<24);   // 12 bit RES
	
//4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	ADC1->CR2 |= (1<<1);     // enable continuous conversion mode
	ADC1->CR2 |= (1<<10);    // EOC after each conversion
	ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT
	
//5. Set the Sampling Time for the channels	
	ADC1->SMPR2 &= ~((1<<3) | (1<<12));  // Sampling time of 3 cycles for channel 1 and channel 4

//6. Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR1 |= (2<<20);   // SQR1_L =2 for 3 conversions
	
//7. Set the Respective GPIO PINs in the Analog Mode	
	GPIOA->MODER |= (3<<2);  // analog mode for PA 1 (chennel 1)
	GPIOA->MODER |= (3<<8);  // analog mode for PA 4 (channel 4)
	
	/*  ******************************************* B A S I C   S E  T U P ************************* */
	
	//Sampling TIME FOR TEMP SENSOR
	//(480+12.5) cycles /(90/4)MHz =21us 
	//480 cycles corresponds to 111 in ADC_SMPR1
	ADC1->SMPR1 |=(7<<24);
	
	//Set the TSVREFE Bit in the CCR to wake the sensor
	ADC->CCR |=(1<<23);
	
	
	
	
	
	//Enable DMA
	ADC1->CR2 |=(1<<8);
	
	//Enable Continuous  Requests
	ADC1->CR2 |=(1<<9);
	
	// Channel Sequence
	ADC1->SQR3 |=(1<<0); //SEQ1 for Channel 1
	ADC1->SQR3 |=(4<<5); //SEQ2 for Channel 4
	
	//For internal temp sensor
	ADC1->SQR3 |=(18<<10); //SEQ3 for Channel 18
	
	
	
}


void ADC_Enable (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable the ADC by setting ADON bit in CR2
	2. Wait for ADC to stabilize (approx 10us) 
	************************************************/
	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1
	
	uint32_t delay = 10000;
	while (delay--);
}


void ADC_Start (void)
{
	/************** STEPS TO FOLLOW *****************
	2. Clear the Status register
	3. Start the Conversion by Setting the SWSTART bit in CR2
	************************************************/
		
	ADC1->SR = 0;        // clear the status register
	
	ADC1->CR2 |= (1<<30);  // start the conversion
}


void DMA_Init(void)
{
	//Enable the DMA2 Clock
	RCC->AHB1ENR |=(1<<22);
	
	//Select the data direction
	DMA2_Stream0->CR &= ~(3<<6); //Peripheral To Memory
	
	//Select Circular Mode
	DMA2_Stream0->CR |=(1<<8); //CIRC=1
	
	//Enable Memory Address INcrement
	DMA2_Stream0->CR |=(1<<10); //MINC =1
	
	//For 12 bit ADC , WE NEED 16 BIT  data size and memory data size
	DMA2_Stream0->CR |=(1<<11)|(1<<13); //PSIZE=01 ,MSIZE =01
	
	//Select Channel for the stream
	DMA2_Stream0->CR &=~(7<<25); //Channel 0 selected
	
}

void DMA_Config(uint32_t srcAdd,uint32_t destAdd, uint8_t size)
{
	DMA2_Stream0->NDTR =size; //Set the size of the DMA transfer
	
	DMA2_Stream0->NDTR = srcAdd; //Source Address is Peripheral Add
	
	DMA2_Stream0->M0AR = destAdd; //Destination Address is Memory Address
	
	//Enable the DMA stream
	DMA2_Stream0->CR |=(1<<0); //EN=1
	
}


uint16_t RxData[3];
float Temperature;


int main ()
{
	SysClockConfig ();
	TIM6Config ();
	
	ADC_Init ();
	ADC_Enable ();
	DMA_Init ();
	
	DMA_Config ((uint32_t ) &ADC1->DR, (uint32_t) RxData, 3);
	
	ADC_Start ();
	
	while (1)
	{
		
		Temperature = (((float)(3.3*RxData[2]/(float)4095) - 0.76) / 0.0025) + 25;
			
		
	  Delay_ms (1000);
	}
	
}