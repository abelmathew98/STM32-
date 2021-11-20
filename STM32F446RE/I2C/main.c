#include "RccConfig.h"
#include "Delay.h"
#include "I2C.h"


static const uint8_t TMP102_ADDR = 0x48 << 1; // Use 8-bit address
static const uint8_t HTU21D_ADDR = 0x40 << 1;
static const uint8_t REG_TEMP = 0x00;
static const uint8_t REG_TEMP_HTU = 0xE3;
static const uint8_t TMP_REG = 0x0;
										

//void I2C_Config(void)
//{
//	/**** STEPS FOLLOWED  ************
//1. Enable the I2C CLOCK and GPIO CLOCK
//2. Configure the I2C PINs for ALternate Functions
//	a) Select Alternate Function in MODER Register
//	b) Select Open Drain Output 
//	c) Select High SPEED for the PINs
//	d) Select Pull-up for both the Pins
//	e) Configure the Alternate Function in AFR Register
//3. Reset the I2C 	
//4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
//5. Configure the clock control registers
//6. Configure the rise time register
//7. Program the I2C_CR1 register to enable the peripheral
//*/
//	
//	//Enable the GPIOB clock
//	RCC->AHB1ENR |=(1<<1);
//	
//	//Enable the I2C Clock 
//	RCC->APB1ENR |=(1<<21); // Enable I2C clock
//	
//	
//	
//	//Configure the I2C pin for Alternate functions
//	GPIOB->MODER |=(2<<16)|(2<<18);
//	GPIOB->OTYPER |=(1<<8)|(1<<9); //Bit 8 =1 and Bit 9 = 1 open drain
//	GPIOB->OSPEEDR |=(3<<16) |(3<<18);  // HIGH SPEED PIN 8 AND PIN 9 =3
//	GPIOB->PUPDR |=(1<<16) | (1<<18); // PULL UP for pin 8 and 9
//	
//	//Alternate function AFR =AF4
//	GPIOB->AFR[1] |=(4<<0)|(4<<4);
//	
//	//RESET I2C
//	I2C1->CR1 |=(1<<15);
//	I2C1->CR1 &=~(1<<15); //TAKE OUT OF RESET
//	
//	//set the PCLK a.k.a peripheral clock in i2c_cr2 to generate correct timings
//	I2C1->CR2 |=(45<0); ///IN MHz
//	
//	//Configure the clock configuration register
//	I2C1->CCR |= (225<0);
//	
//	//Configure the rise time register
//	I2C1->TRISE |=(46<<0);
//	
//	//Enable I2C peripheral
//	I2C1->CR1 |=(1<<0);//Bit 0 of CR1 register
//	
//}

//void I2C_Start()
//{
//	/**** STEPS FOLLOWED  ************
//1. Enable the ACK
//2. Send the START condition 
//3. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
//*/	
//	I2C1->CR1 |=(1<<15);
//	I2C1->CR1 |= (1<<10); //Enable the ACK bit

//	I2C1->CR1 |= (1<<8);  //Generate start bit

// 	
//	while(!(I2C1->SR1 & (1<<0))); //WAIT FOR SB TO BE SET
//	
//	
//}

//void I2C_Write(uint8_t data)
//{
//	/**** STEPS FOLLOWED  ************
//1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
//2. Send the DATA to the DR Register
//3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
//*/
//	while(!(I2C1->SR1 & (1<<7))); //WAIT FOR TXE bit to be set
//	I2C1->DR =data; // send data to DR 
//	while(!(I2C1->SR1 &(1<<2))); //wait for BTF bit to set
//	
//}

//void I2C_Address(uint8_t Address)
//{
//	/**** STEPS FOLLOWED  ************
//1. Send the Slave Address to the DR Register
//2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
//3. clear the ADDR by reading the SR1 and SR2
//*/	
//	while(!(I2C1->SR1 & (1<<7))); //WAIT FOR TXE bit to be set
//	I2C1->DR= Address;
//	while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
//	uint8_t temp = I2C1->SR1 | I2C1->SR2; //read SR1 and SR2 to clear the ADDR bit
//	while(!(I2C1->SR1 & (1<<7))); //wait for DATA REGISTER TO BE EMPTY TXE BIT
//}


//void I2C_Stop()
//{
//	I2C1->CR1 |=(1<<9); //STOP BIT IN CR1 (I2C)
//	
//}	

//void I2C_WriteMulti (uint8_t *data, uint8_t size)
//{
//	while(!(I2C1->SR1 & (1<<7))); //WAIT FOR TXE bit to be set
//	
//	while(size)
//	{
//		while(!(I2C1->SR1 & (1<<7))); //WAIT FOR TXE bit to be set
//		I2C1->DR= (volatile uint32_t) *data++; //send data
//		size--;

//	}
//	while(!(I2C1->SR1 &(1<<2))); //wait for BTF bit to set
//}


//void I2C_Read(uint8_t Address, uint8_t *buffer, int size)
//{
//	/**** STEPS FOLLOWED  ************
//1. If only 1 BYTE needs to be Read
//	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
//	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
//	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
//	d) Read the data from the DR

//2. If Multiple BYTES needs to be read
//  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
//	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
//	c) Wait for the RXNE (Receive buffer not empty) bit to set
//	d) Read the data from the DR 
//	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
//	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the 
//		 second last data byte (after second last RxNE event)
//	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit 
//	   after reading the second last data byte (after the second last RxNE event)
//*/
//	
//	int remaining =size;
//	
//	if(size==1)
//	{
//		//send the slave address
//		I2C1->DR=Address;
//		Delay_ms(10);
//		while(!(I2C1->SR1 & (1<<1))); //  WAITING FOR ADDR BIT TO SET
//		
//		
//		//Clear the ACK BIT
//		I2C1->CR1 &= ~(1<<10);
//		uint8_t temp= I2C1->SR1 | I2C1->SR2; //Read I2C SR registers to	 clear the ADDR bit
//		I2C_Stop();  // Stop i2c
//		
//		while(!(I2C1->SR1 & (1<<6))); //WAIT FOR RXE to set
//		
//		buffer[size-remaining] = I2C1->DR; //Read the data from Data Register
//		
//	}
//	else
//	{
//		I2C1->DR=Address;
//		while(!(I2C1->SR1 & (1<<1))); //  WAITING FOR ADDR BIT TO SET
//		
//		uint8_t temp= I2C1->SR1 |  I2C1->SR2; //Read I2C SR registers to	 clear the ADDR bit
//		
//		while(remaining>2)
//		{
//			while(!(I2C1->SR1 & (1<<6))); //WAIT FOR RXE to set
//			
//			buffer[size-remaining] = I2C1->DR; //Read the data from Data Register 
//			
//			I2C1->CR1 |= (1<<10); //SET ACK BIT to acknowledge the data bit

//			remaining--;
//		}
//		//READ THE SECOND LAST BYTE
//		while(!(I2C1->SR1 & (1<<6))); //Wait for the RXNE to set
//		buffer[size-remaining] = I2C1->DR; 
//		//Clear the ACK BIT
//		I2C1->CR1 &= ~(1<<10);
//		I2C_Stop();  // Stop i2c
//		remaining--;
//		
//		// READ THE LAST BYTE
//		while(!(I2C1->SR1 & (1<<6))); //Wait for the RXNE to set
//		buffer[size-remaining] = I2C1->DR; //Send DATA
//		
//		
//		
//	}
//	
//	
//}
void I2C_Config (void)
{
/**** STEPS FOLLOWED  ************
1. Enable the I2C CLOCK and GPIO CLOCK
2. Configure the I2C PINs for ALternate Functions
	a) Select Alternate Function in MODER Register
	b) Select Open Drain Output 
	c) Select High SPEED for the PINs
	d) Select Pull-up for both the Pins
	e) Configure the Alternate Function in AFR Register
3. Reset the I2C 	
4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
5. Configure the clock control registers
6. Configure the rise time register
7. Program the I2C_CR1 register to enable the peripheral
*/
	
	// Enable the I2C CLOCK and GPIO CLOCK
	RCC->AHB1ENR |= (1<<1);  // Enable GPIOB CLOCK
	RCC->APB1ENR |= (1<<21);  // enable I2C CLOCK

	
	
	// Configure the I2C PINs for ALternate Functions
	GPIOB->MODER |= (2<<16) | (2<<18);  // Bits (17:16)= 1:0 --> Alternate Function for Pin PB8; Bits (19:18)= 1:0 --> Alternate Function for Pin PB9
	GPIOB->OTYPER |= (1<<8) | (1<<9);  //  Bit8=1, Bit9=1  output open drain
	GPIOB->OSPEEDR |= (3<<16) | (3<<18);  // Bits (17:16)= 1:1 --> High Speed for PIN PB8; Bits (19:18)= 1:1 --> High Speed for PIN PB9
	GPIOB->PUPDR |= (1<<16) | (1<<18);  // Bits (17:16)= 0:1 --> Pull up for PIN PB8; Bits (19:18)= 0:1 --> pull up for PIN PB9
	GPIOB->AFR[1] |= (4<<0) | (4<<4);  // Bits (3:2:1:0) = 0:1:0:0 --> AF4 for pin PB8;  Bits (7:6:5:4) = 0:1:0:0 --> AF4 for pin PB9
	
	
	// Reset the I2C 
	I2C1->CR1 |= (1<<15);
	//I2C1->CR1 &= ~(1<<15);
	
	// Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
	I2C1->CR2 |= (45<<0);  // PCLK1 FREQUENCY in MHz
	
	// Configure the clock control registers
	I2C1->CCR = 225<<0;  // check calculation in PDF
	
	// Configure the rise time register
	I2C1->TRISE = 46;  // check PDF again
	
	// Program the I2C_CR1 register to enable the peripheral
	I2C1->CR1 |= (1<<0);  // Enable I2C
}

void I2C_Start (void)
{
/**** STEPS FOLLOWED  ************
1. Send the START condition 
2. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
*/	

	I2C1->CR1 |= (1<<10);  // Enable the ACK
	I2C1->CR1 |= (1<<8);  // Generate START
	while (!(I2C1->SR1 & (1<<0)));  // Wait fror SB bit to set
}


void I2C_Write (uint8_t data)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Send the DATA to the DR Register
3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/	
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	I2C1->DR = data;
	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF bit to set
}

void I2C_Address (uint8_t Address)
{
/**** STEPS FOLLOWED  ************
1. Send the Slave Address to the DR Register
2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
3. clear the ADDR by reading the SR1 and SR2
*/	
	I2C1->DR = Address;  //  send the address
	while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
	uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
}
	
void I2C_Stop (void)
{
	I2C1->CR1 |= (1<<9);  // Stop I2C
}

void I2C_WriteMulti (uint8_t *data, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Keep Sending DATA to the DR Register after performing the check if the TXE bit is set
3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/	
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set 
	while (size)
	{
		while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set 
		I2C1->DR = (uint32_t )*data++;  // send data
		size--;
	}
	
	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF to set
}

void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. If only 1 BYTE needs to be Read
	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
	d) Read the data from the DR

2. If Multiple BYTES needs to be read
  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
	c) Wait for the RXNE (Receive buffer not empty) bit to set
	d) Read the data from the DR 
	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the 
		 second last data byte (after second last RxNE event)
	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit 
	   after reading the second last data byte (after the second last RxNE event)
*/		
	
	int remaining = size;
	
/**** STEP 1 ****/	
	if (size == 1)
	{
		/**** STEP 1-a ****/	
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		/**** STEP 1-b ****/	
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit 
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2C1->CR1 |= (1<<9);  // Stop I2C

		/**** STEP 1-c ****/	
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		
		/**** STEP 1-d ****/	
		buffer[size-remaining] = I2C1->DR;  // Read the data from the DATA REGISTER
		
	}

/**** STEP 2 ****/		
	else 
	{
		/**** STEP 2-a ****/
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		/**** STEP 2-b ****/
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
		
		while (remaining>2)
		{
			/**** STEP 2-c ****/
			while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
			
			/**** STEP 2-d ****/
			buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer			
			
			/**** STEP 2-e ****/
			I2C1->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received
			
			remaining--;
		}
		
		// Read the SECOND LAST BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;
		
		/**** STEP 2-f ****/
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit 
		
		/**** STEP 2-g ****/
		I2C1->CR1 |= (1<<9);  // Stop I2C
		
		remaining--;
		
		// Read the Last BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer
	}	
	
}

void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
	I2C_Start ();
	I2C_Address (Address);
	I2C_Write (Reg);
	I2C_Start ();  // repeated start
	I2C_Read (Address+0x01, buffer, size);
	I2C_Stop ();
}



int main()
{
	SysClockConfig();
	TIM6Config();
	Delay_ms(10);
	
	I2C_Config();
	Delay_ms(10);
	uint8_t check;
	I2C_Stop ();
	MPU_Read (TMP102_ADDR,TMP_REG, &check, 1);
	
	
	while(1)
	{
		//uint8_t softreset =0xFE;
		//uint8_t temp_cmd =0xE3;
		//uint8_t rxdata[3];
		//I2C_Start();
		//Delay_ms(20);
		//softreset
		//I2C_Address(TMP102_ADDR);
		//I2C_Write(temp_read);
		//I2C_Read(i2c_add,rxdata,2);
		
		
		//I2C_Address(0xE3);
	  //I2C_Read(i2c_add ,rxdata,2);
		//I2C_Write(1<<1);
		//I2C_Stop();
		
	}

}