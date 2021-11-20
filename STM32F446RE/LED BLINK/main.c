#include "stm32f446xx.h"  //Decomposed the .h file
//#define     __IO    volatile             /*!< Defines 'read / write' permissions */
//typedef unsigned           int uint32_t;
//typedef unsigned short     int uint16_t;

//// RCC essential declarations
//typedef struct
//{
//  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
//  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
//  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
//  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
//  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
//  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
//  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
//  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
//  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
//  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
//  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
//  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
//  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
//  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
//  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
//  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
//  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
//  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
//  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
//  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
//  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address ox      ffset: 0x58 */
//  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
//  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
//  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
//  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
//  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
//  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
//  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
//  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
//  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
//  __IO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
//  __IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
//  __IO uint32_t CKGATENR;      /*!< RCC Clocks Gated ENable Register,                            Address offset: 0x90 */
//  __IO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */
//} RCC_TypeDef;

//#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
//#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
//#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL) 
//#define RCC                   ((RCC_TypeDef *) RCC_BASE)


//#define FLASH_BASE            0x08000000UL /*!< FLASH(up to 1 MB) base address in the alias region                         */
//#define SRAM1_BASE            0x20000000UL /*!< SRAM1(112 KB) base address in the alias region                              */
//#define SRAM2_BASE            0x2001C000UL /*!< SRAM2(16 KB) base address in the alias region                              */
//#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
//#define BKPSRAM_BASE          0x40024000UL /*!< Backup SRAM(4 KB) base address in the alias region                         */
//#define FMC_R_BASE            0xA0000000UL /*!< FMC registers base address                                                 */
//#define QSPI_R_BASE           0xA0001000UL /*!< QuadSPI registers base address                                             */
//#define SRAM1_BB_BASE         0x22000000UL /*!< SRAM1(112 KB) base address in the bit-band region                          */
//#define SRAM2_BB_BASE         0x22380000UL /*!< SRAM2(16 KB) base address in the bit-band region                           */
//#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region                             */
//#define BKPSRAM_BB_BASE       0x42480000UL /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
//#define FLASH_END             0x0807FFFFUL /*!< FLASH end address                                                          */
//#define FLASH_OTP_BASE        0x1FFF7800UL /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
//#define FLASH_OTP_END         0x1FFF7A0FUL /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */
//#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */

//typedef struct
//{
//  __IO uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
//  __IO uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
//} PWR_TypeDef;
//#define APB1PERIPH_BASE       PERIPH_BASE
//#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)
//#define PWR                 ((PWR_TypeDef *) PWR_BASE)


//typedef struct
//{
//  __IO uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
//  __IO uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
//  __IO uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
//  __IO uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
//  __IO uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
//  __IO uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
//  __IO uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
//} FLASH_TypeDef;
//#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
//#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
//#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)



//typedef struct
//{
//  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
//  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
//  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
//  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
//  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
//  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
//  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
//  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
//  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
//} GPIO_TypeDef;
//#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
//#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
//#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)














/* **************  INCLUDE DEFINITION END ************************ */

//PLL Constant values
#define PLL_M 	4
#define PLL_N		180
#define PLL_P		0    //instead of directly giving the value , we have to follow the value given in REF manual.O corresponds to 2

void SysClockConfig(void);
void GPIO_Config(void);
void delay(uint32_t time);
void delay_ms(uint16_t delay_ms);
void delay_us(uint32_t delay_us);
void SleepModeActivate(void);

void SleepModeActivate(void)
{
	RCC->APB1ENR |= (RCC_APB1ENR_PWREN);
	
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	
	PWR->CR |= PWR_CR_PDDS;
	
	PWR->CR |= PWR_CR_CWUF;
	
	//PWR->CSR |= PWR_CSR_EWUP1;
	
	__WFI();
	
	
}


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
	GPIOA->OTYPER =0; //Output type default : push pull
	GPIOA->OSPEEDR=0; //DEFAULT LOW SPEED
	
}

void delay()
{
	for(uint32_t i=255;i>0;i--)
	{
		for(int j=240;j>0;j--);
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


int main(void)
{
	SysClockConfig();
	GPIO_Config();
	
	while(1)	
	{
		GPIOA->BSRR |= (1<<5); 
		delay_ms(1000);
		GPIOA->BSRR |= (1<<21); 
		delay_ms(1000);
		SleepModeActivate();
		
	}
}