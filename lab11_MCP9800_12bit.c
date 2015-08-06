#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define MCP9800_ADDRESS (0x48 << 1)
#define I2C1_SLAVE_ADDRESS7 0x30
uint8_t zmienna1, zmienna2;
float zmienna3;

void init_I2C1(void);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_stop(I2C_TypeDef* I2Cx);
uint8_t I2C_read(I2C_TypeDef* I2Cx, uint8_t ACK);


int main(void) {
	SystemInit();
	SystemCoreClockUpdate();

	// Inicjalizacja I2C
	init_I2C1();

	// Rozpoczecie transmisji z uC do czujnika
	I2C_start(I2C1, MCP9800_ADDRESS, I2C_Direction_Transmitter);
	I2C_write(I2C1,0x01);	// ustaw adres rejestru Configuration Register
	I2C_write(I2C1,0x30);	// ustaw rozdzielczosc na 12 bit
	// Zakonczenie transmisji
	I2C_stop(I2C1);

	// Rozpoczecie transmisji z uC do czujnika (czy to jest konieczne?)
	I2C_start(I2C1, MCP9800_ADDRESS, I2C_Direction_Transmitter);
	I2C_write(I2C1, 0x00); // Ustawienie rejestru z temperatur¹
	// Zakonczenie transmisji
	I2C_stop(I2C1);

	// Glowna petla odczytujaca dane
	while (1) {
		// Rozpoczecie transmisji odbierajaca dane z MPC9800
		I2C_start(I2C1, MCP9800_ADDRESS, I2C_Direction_Receiver);
		// Odczyt 2 bajtów danych
		zmienna1 = I2C_read(I2C1, ENABLE);
		zmienna2 = I2C_read(I2C1, DISABLE);
		// Zakonczenie transmisji
		I2C_stop(I2C1);
		// Zebranie danych -> zmienna3 to temperatura z dok³adnoci¹ do 0.5 stopnia
		zmienna3 = zmienna1 + zmienna2/256.0;
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
 * MASTER code
 */

/* I2Cx --> w naszym pzypadku I2C1
 * address --> 7-bitowy adres urzadzenia
 * direction --> kierunek transmisji danych:
 * I2C_Direction_Tranmitter dla transmisji danych z urzadzenia Master
 * I2C_Direction_Receiver dla odbioru danych prze urz¹dzenie Master
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
	// Oczekiwanie na zwolnienie magistrali I2C, ¿eby nie powodowaæ kolizji
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		;
	// Wygenerowanie sygna³u START
	I2C_GenerateSTART(I2Cx, ENABLE);

	// Oczekiwanie na potwierdzenie od Slave'a, ¿e odebra³ sygna³ START
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		;

	// Adresacja urz¹dzenia Slave i wybór kierunku transmisji danych
	I2C_Send7bitAddress(I2Cx, address, direction);

	// Oczekiwanie na potwierdzenie kierunku transmisji
	if (direction == I2C_Direction_Transmitter) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	} else if (direction == I2C_Direction_Receiver) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/* Wysy³anie danych po I2C */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
	I2C_SendData(I2Cx, data);
	// Oczekiwanie na zakoñczenie transmisji
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* Funkcja odbieraj¹ca 1 bajt danych oraz w zale¿nosci od parametru ACK:
 * - je¿eli ENABLE -> potwierdzaj¹ca odbiór (oznacza chêæ odbioru jeszcze 1 bajtu)
 * - je¿eli DISABLE -> NIE potwierdzaj¹ca odbioru (zakoñczenie odczytu)*/

uint8_t I2C_read(I2C_TypeDef* I2Cx, uint8_t ACK) {
	// W³¹czenie/Wy³¹czenie potwierdzenia odbioru
	I2C_AcknowledgeConfig(I2Cx, ACK);
	// Oczekiwania a¿ odebranie bajtu danych zostanie zakoñczone
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
	// Odczyt odebranych danych z I2C
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* Zakoñczenie transmisji i zwolnienie magistrali */
void I2C_stop(I2C_TypeDef* I2Cx) {
	// Wygenerowanie sygna³u STOP
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

