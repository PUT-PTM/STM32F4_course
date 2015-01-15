#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define I2C1_SLAVE_ADDRESS7 0x30
uint8_t zmienna;

void init_I2C1(void);
void I2C_slave_start(I2C_TypeDef* I2Cx);
void I2C_slave_stop(I2C_TypeDef* I2Cx);
uint8_t I2C_slave_read(I2C_TypeDef* I2Cx);


// SLAVE
int main(void) {
	SystemInit();
	SystemCoreClockUpdate();

	// Inicjalizacja I2C
	init_I2C1();

	// Glowna petla odczytujaca dane
	while (1) {
		// START Slave
		I2C_slave_start(I2C1);
		// Read slave
		zmienna = I2C_slave_read(I2C1);
		// Zakonczenie transmisji
		I2C_slave_stop(I2C1);
	}
}

/*
 * Init
 */
void init_I2C1(void) {
	// W³¹czenie zegara I2C
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// W³¹czenie zegara od GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// Ustawienie pinów SCL (PB8) i SDA (PB7)
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // Open-drain
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Po³¹czenie I2C z pinami
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); // SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

	// Ustawienie I2C
	I2C_InitTypeDef I2C_InitStruct;
	// Predkosc transmisji - 100kHz
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	// Wybór I2C (inne mo¿liwosci dotycz¹ SMBUS)
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	// Wspó³czynnik wype³nienia 50%
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	// Adres w trybie slave
	I2C_InitStruct.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;
	// ACK - potwierdzenie odbioru
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	 // D³ugosc adresacji - 7bitów
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// init I2C1
	I2C_Init(I2C1, &I2C_InitStruct);

	// W³aczenie I2C1
	I2C_Cmd(I2C1, ENABLE);
}

/*
 * SLAVE code
 */

/* Oczekiwanie na wywo³anie urz¹dzenia SLAVE  */
void I2C_slave_start(I2C_TypeDef* I2Cx) {
	/* Oczekiwanie na wywo³anie adresu urz¹dzenia SLAVE  */
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED));
}

/* Odczyt bajtu przez urz¹dzenie SLAVE */
uint8_t I2C_slave_read(I2C_TypeDef* I2Cx) {
	// Oczekiwania a¿ odebranie bajtu danych zostanie zakoñczone
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_BYTE_RECEIVED));
	// Odczyt odebranych danych z I2C
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* Realizacja zakoñczenia transmisji przez urz¹dzenie SLAVE */
void I2C_slave_stop(I2C_TypeDef* I2Cx) {
	/* Wykrycie zakoñczenia transmisji przez urz¹dzenie MASTER */
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_STOP_DETECTED));

	/* Clear I2C2 STOPF flag: read operation to I2C_SR1 followed by a
	 write operation to I2C_CR1 */
	(void) (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
	I2C_Cmd(I2Cx, ENABLE);
}

