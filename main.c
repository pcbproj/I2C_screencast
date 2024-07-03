/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

/*
I2C project for lesson
по кнопке S1 происходит запись массива данных из 4-х  байт в I2C EEPROM, начиная с выбранной ячейки памяти.
по кнопке S2 происходит изменение адреса в EEPROM циклически между 8-мью адресами. 
	-- Индикация смещения на LED1-LED3.
по кнопке S3 происходит чтение массива данных (4 байта) из EEPROM, начиная с выбранной ячейки памати и сохранение в массив

*/

#include <stdio.h>
#include <stdlib.h>
#include "stm32f407xx.h"

#define	BTN_PRESS_CNT		4  // счетки опроса кнопки при антидребезге
#define BTN_CHECK_MS		10	// период опроса кнопок при антидребезге

// ------  адресация устройства на шине I2C ------------- 
#define I2C_DEV_ADDR	0xA0 // адрес микросхемы EEPROM = 1010_0000 в бинарном виде. Используются старшие 7 бит
#define I2C_WR_BIT		0x00 // запрос на запись данных в I2C-устройство (в EEPROM)
#define I2C_RD_BIT		0x01 // запрос на чтение данных из I2C-устройство (в EEPROM)
#define I2C_DEV_ADDR_RD	 (I2C_DEV_ADDR + I2C_RD_BIT)	// младший бит выставляем в RD = 1
#define I2C_DEV_ADDR_WR  (I2C_DEV_ADDR + I2C_WR_BIT)	// младший бит выставляем в WR = 0

// ----------- адресация внутри EEPROM -------------
#define EEPROM_WR_START_ADDR	0x08	// запись с 1 ячейки в страницу 2
#define EEPROM_WR_LEN			4	
#define EEPROM_PAGE_LEN_BYTES	8
#define EEPROM_RD_START_ADDR	0x08	// чтение с 1 ячейки в страницу 2
#define EEPROM_RD_LEN			4

char i2c_tx_array[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };	// Массив записываетмый в EEPROM
char i2c_rx_array[8] = {};	// Массив, куда будут читаться данные из EEPROM

uint16_t ms_count = 0;	// счетчик мс для опроса кнопок




void GPIO_Init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	
	//-------- GPIO for buttons -------------------
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD10_0;
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD11_0;
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD12_0;
	   
	//-------- GPIO settings for LED1 LED2 LED3 --------
	GPIOE -> MODER |=GPIO_MODER_MODE13_0;
	GPIOE -> MODER |=GPIO_MODER_MODE14_0;
	GPIOE -> MODER |=GPIO_MODER_MODE15_0;

}


void I2C_Init(void){
	RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;	// включение тактирования модуля I2C1 
  	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	// включение тактирования порта GPIOB (PB8 = SCL, PB9 = SDA)
  	
  	// настройка выводов PB8 и PB9 для работы с модулем I2C1
  	GPIOB -> MODER 	|= 	GPIO_MODER_MODE8_1;		// PB8 в режиме альтернативной функции
  	GPIOB -> MODER 	|= 	GPIO_MODER_MODE9_1;		// PB9 в режиме альтернативной функции

	GPIOB -> AFR[1]	|=	GPIO_AFRH_AFRH0_2;	// для PB8 выбрана альтернативная ф-ия AF4 = I2C1
  	GPIOB -> AFR[1]	|=	GPIO_AFRH_AFRH1_2;	// для PB9 выбрана альтернативная ф-ия AF4 = I2C1
	
	GPIOB -> PUPDR	&=	~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);		// явно прописываем отключение всех подтягивающих резисторов
																		// хотя по умолчанию они и так отключены

	/*======== настройка модуля I2C1 ============
	По умолчанию I2C работает в подчиненном режиме. Интерфейс автоматически переключается с подчиненного на
	мастер, после того как он сгенерирует условие START и от ведущего к ведомому переключается, в случае 
	арбитражного проигрыша или если на шине происходит генерация STOP-условия, что обеспечивает возможность 
	работы с несколькими ведущими устройствами.
	
	  	режим работы					  	  = мастер
	  	скорость передачи				  	  = 100 кбит/сек
	  	адресация устройств на шине I2C 	  = 7 битная
	  	DMA не используется			  	  = Эти биты по умолчанию равны 0
	  	прерывания не используются	  	  = Эти биты по умолчанию равны 0
	  	адрес микросхемы памяти на шине I2C = 0xA0 = 0b1010_0000. Используются старшие 7 бит!
	
	*/ 

	I2C1 -> CR2	|=	(42 << I2C_CR2_FREQ_Pos);  // CR2_FREQ = 42 т.к. Freq_APB1 = 42MHz

	/*====== CCR вычисления: ======
	I2C работает на частоте 100 кГц - Standard mode
	Thigh = CCR * T_plck1
	Tlow = CCR * T_pclk1
	Tsm = 1/(I2C_freq) = 1/100000 = Thigh + Tlow;
	1/100000 = 2 * CCR * T_pclk1
	CCR = 1 / (2*100000*T_pclk1)
	T_pclk1 = 1 / Freq_APB1; 
	Freq_APB1 = 42 MHz
	T_Pclk1 = 1 / 42000000
	CCR = 42000000 / (2*100000) = 210;
  	*/

	I2C1 -> CCR	|=	(210 << I2C_CCR_CCR_Pos);		// 100 КГц
	I2C1 -> CCR	&=	~(I2C_CCR_FS);					// явный сброс бита FS = работа на чатоте 100 кГц (Standard Mode)
	I2C1 -> TRISE |=  (43 << I2C_TRISE_TRISE_Pos);	// значение поля = I2C1_CR2_FREQ + 1 = 42+1 = 43

	I2C1 -> OAR1  &=  ~(I2C_OAR1_ADDMODE);			// использование 7-ми битного адреса устройства на шине I2C

	I2C1 -> CR1	|=	I2C_CR1_PE;						// I2C1 enabled. 
  	I2C1 -> CR1	|=	I2C_CR1_ACK;					// разрешение генерации ACK после приема байтов.
  	/* бит I2C_CR1_ACK можно выставлять в 1 только после включения бита I2C_CR1_PE. 
		 иначе бит I2C_CR1_ACK всегда будет сбрасываться в 0 аппаратно.
  	*/
	
}




int main(void) {
	while (1){
  
	

	}	// while(1)
}	// main()

/*************************** End of file ****************************/
