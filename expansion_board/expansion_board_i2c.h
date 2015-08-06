#ifndef _EXPANSION_BOARD_I2C_H_
#define _EXPANSION_BOARD_I2C_H_

#include <stdint.h>
#include "stm32f4xx_i2c.h"

// I2C code based on:
// http://eliaselectronics.com/
// Github:
// https://github.com/devthrash/STM32F4-examples

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);

void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);

void I2C_stop(I2C_TypeDef* I2Cx);

void I2C_slave_start(I2C_TypeDef* I2Cx);

void I2C_slave_stop(I2C_TypeDef* I2Cx);

uint8_t I2C_slave_read_ack(I2C_TypeDef* I2Cx);

uint8_t I2C_slave_read_nack(I2C_TypeDef* I2Cx);

void init_I2C1(void);

#endif
