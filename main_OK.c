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

char S1_cnt = 0; 	 // button S1 press couter
char S2_cnt = 0; 	 // button S2 press couter
char S3_cnt = 0; 	 // button S3 press couter

char S1_state = 0;   // S1 state: 1 = pressed, 0 = released
char S2_state = 0;   // S2 state: 1 = pressed, 0 = released
char S3_state = 0;   // S3 state: 1 = pressed, 0 = released


//FSM флаги состояния, по которым автомат переходит в состояния
char IDLE_flag = 0;
char EEPROM_WRITE_flag = 0;
char EEPROM_READ_flag = 0;
char ADDR_INC_flag = 0;

// флаги сброса состояний, чтобы не циклиться в одном состоянии при долгом сигнале с кнопки
char IDLE_out = 0;
char EEPROM_WRITE_out = 0;
char EEPROM_READ_out = 0;
char ADDR_INC_out = 0;



void RCC_Init();

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

	GPIOB -> OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
	
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


void SysTick_Handler(void){		// прервание от Systick таймера, выполняющееся с периодом 1000 мкс
	ms_count++;
}


void BTN_Check(void){
	if(ms_count > BTN_CHECK_MS){
		ms_count = 0;
		// опрос кнопки S1
		if((GPIOE->IDR & GPIO_IDR_ID10) == 0){
			if(S1_cnt < BTN_PRESS_CNT){
				S1_cnt++;
				S1_state = 0;	// кнопка пока не нажата
			}
			else{
				S1_state = 1;	// кнопка нажата
			}
		}
		else{
			S1_state = 0;
			S1_cnt = 0;
		}

		// опрос кнопки S2
		if((GPIOE->IDR & GPIO_IDR_ID11) == 0){
			if(S2_cnt < BTN_PRESS_CNT){
				S2_cnt++;
				S2_state = 0;	// кнопка пока не нажата
			}
			else{
				S2_state = 1;	// кнопка нажата
			}
		}
		else{
			S2_state = 0;
			S2_cnt = 0;
		}

		// опрос кнопки S3
		if((GPIOE->IDR & GPIO_IDR_ID12) == 0){
			if(S3_cnt < BTN_PRESS_CNT){
				S3_cnt++;
				S3_state = 0;	// кнопка пока не нажата
			}
			else{
				S3_state = 1;	// кнопка нажата
			}
		}
		else{
			S3_state = 0;
			S3_cnt = 0;
		}
	}

}


void State_Flag_Gen(void){
	if(!S1_state) EEPROM_WRITE_out = 0;
	else EEPROM_WRITE_flag = S1_state & ~(EEPROM_WRITE_out);

	if(!S2_state) ADDR_INC_out = 0;
	else ADDR_INC_flag = S2_state & ~(ADDR_INC_out);

	if(!S3_state) EEPROM_READ_out = 0;
	else EEPROM_READ_flag = S3_state & ~(EEPROM_READ_out);
}

void I2C_Start_gen(void){
	I2C1 -> CR1 |= I2C_CR1_START;
	while((I2C1->SR1 & I2C_SR1_SB) == 0){};	// ожидание START-условия на шине I2C
}

void I2C_TxDeviceADDR(char device_address, char RW_bit){
	I2C1 -> DR = (device_address + RW_bit);				// отправить в I2C_DR адрес устройства и бит WR
	while((I2C1 -> SR1 & I2C_SR1_ADDR) == 0){};		// ждем флаг I2C_SR1_ADDR = 1. Пока завершится передача байта адреса
	(void)I2C1 -> SR1; 
	(void)I2C1 -> SR2;	// очистка бита ADDR чтением регистров SR1 SR2

}

void I2C_Stop_Gen(void){
	I2C1 -> CR1 |= I2C_CR1_STOP;
}

void I2C_Write( char start_addr, char data[], uint16_t data_len){
	I2C1 -> CR1	|=	I2C_CR1_ACK;// Включить генерацию битов ACK

	while((I2C1 -> SR2 & I2C_SR2_BUSY) != 0 ){}; // проверить свободна ли шина I2C

	I2C_Start_gen(); // генерация START-условия
	
	I2C_TxDeviceADDR(I2C_DEV_ADDR , I2C_WR_BIT);	// отправить адрес устройства I2C

	I2C1 -> DR = start_addr;	// отправить в I2C_DR адрес начальной ячейки памяти, куда хотим писать данные
	while((I2C1 -> SR1 & I2C_SR1_TXE) == 0){};	// ждем флаг I2C_SR1_TXE = 1. Пока завершится передача байта данных

	// цикл for запись данных в I2C-устройство
	for(uint16_t i = 0; i < data_len; i++){
		I2C1 -> DR = data[i];
		while((I2C1 -> SR1 & I2C_SR1_TXE) == 0){};
	}		

	I2C_Stop_Gen();	// генерация STOP-условия
}

void I2C_Read(char start_addr, char rx_data[], uint16_t data_len){
	I2C1 -> CR1	|=	I2C_CR1_ACK;// Включить генерацию битов ACK
		
	while((I2C1 -> SR2 & I2C_SR2_BUSY) != 0 ){};	// проверить свободна ли шина I2C

	I2C_Start_gen();	// генерация START-условия

	I2C_TxDeviceADDR(I2C_DEV_ADDR , I2C_WR_BIT); // передать адрес устройства I2C и бит WR

	I2C1 -> DR = start_addr; // передать адрес ячейки памяти EEPROM начиная с которой будем читать данные
	while((I2C1 -> SR1 & I2C_SR1_TXE) == 0){};	// ждем флаг I2C_SR1_TXE = 1. Пока завершится передача байта данных
	
	/*========= Пример отправки 2-х байтного адреса ячейки памяти ===============

		I2C1 -> DR = (start_addr >> 8);			// передача старшего байта адреса			
		while((I2C1 -> SR1 & I2C_SR1_TXE) == 0){};	// ожидание отправки байта
		I2C1 -> DR = (start_addr);					// отправка младшего байта адреса	
		while((I2C1 -> SR1 & I2C_SR1_TXE) == 0){};	// ожидание отправки байта
	
	*/


	I2C_Start_gen(); // повторное START-условие

	I2C_TxDeviceADDR(I2C_DEV_ADDR , I2C_RD_BIT); // передать адрес устройства I2C и бит RD

	// цикл чтения данных data_len-1
	for(uint16_t i=0; i < data_len-1; i++ ){
		while((I2C1 ->SR1 & I2C_SR1_RXNE ) == 0 ){};
		rx_data[i] = I2C1->DR;
	}
	

	I2C1->CR1  &=  ~(I2C_CR1_ACK); // отключить генерацию ACK

	// принять байт data_len
	while((I2C1 ->SR1 & I2C_SR1_RXNE ) == 0 ){};
	rx_data[data_len-1] = I2C1->DR;

	I2C_Stop_Gen(); // генерация STOP-условия
}



int main(void) {

	enum states{
		IDLE = 0,
		EEPROM_WRITE,
		EEPROM_READ,
		ADDR_INC
	};

	enum states FSM_state = IDLE;

	char eeprom_addr = 0;	// адрес чтения или записи в EEPROM
	char addr_offset = 0;	// смещение адреса EEPROM относительно начального значения

	RCC_Init();

	GPIO_Init();

	I2C_Init();
	
	SysTick_Config(84000);		// настройка SysTick таймера на время отрабатывания = 1 мс
								// 84000 = (AHB_freq / время_отрабатывания_таймера_в_мкс)
								// 84000 = 84_000_000 Гц / 1000 мкс; 
	
	//----- turn off leds ---------
	GPIOE->BSRR	|=	GPIO_BSRR_BS13;
	GPIOE->BSRR	|=	GPIO_BSRR_BS14;
	GPIOE->BSRR	|=	GPIO_BSRR_BS15;


	while (1){
  
		BTN_Check();
		
		State_Flag_Gen();

		//-------- FSM блок переключения состояний ---------
		if(EEPROM_WRITE_flag) FSM_state = EEPROM_WRITE;
		else{
			if(ADDR_INC_flag) FSM_state = ADDR_INC;
			else{
				if (EEPROM_READ_flag) FSM_state = EEPROM_READ;
				else FSM_state = IDLE;
			}
		}

		//--------- FSM блок реализации основной логики -------
		switch(FSM_state){
		case IDLE:
			break;

		case EEPROM_WRITE:
			I2C_Write(eeprom_addr, i2c_tx_array, EEPROM_WR_LEN);// Запись массива данных в EEPROM по адресу eeprom_addr
			EEPROM_WRITE_out = 1;
			break;
	
		case ADDR_INC:
			if(addr_offset < EEPROM_WR_LEN) addr_offset++;
			else addr_offset = 0;
			ADDR_INC_out = 1;
			break;

		case EEPROM_READ:
			I2C_Read(eeprom_addr, i2c_rx_array, EEPROM_RD_LEN);	// Чтение массива данных из EEPROM по адресу eeprom_addr
			EEPROM_READ_out = 1;
			break;
		}	// switch(FSM_state)

	
		eeprom_addr = EEPROM_RD_START_ADDR + addr_offset;	// вычисление адреса в EEPROM через базовый адрес
		GPIOE -> ODR = ( (~(addr_offset & 0x07)) << 13);	// индикация смещения адреса на LEDS

	}	// while(1)
}	// main()

/*************************** End of file ****************************/
