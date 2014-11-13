STM32F4_course
==============

Git repository for Basics of Microprocessors Technology course hosted by Institute of Control and Information Engineering on  Poznan University of Technology.

## Description

The main file (main.c) allows to choose (by commenting/uncommenting appropriate functions) example code for each class (eg. lab01(); lab02();). Each function is wrapped in infinite loop so only one will be executed. All the instructions for classes can be found on Google Drive (only in Polish language):
https://drive.google.com/open?id=0B7GyMR8rCWn4OUJHOVZHYXhNZ2M&authuser=0

In addition example code for testing expansion board is provided. It uses FreeRTOS operating system to test all the peripherals:
 * main board leds (4) and button - leds are turning on and off and button changes the direction,
 * expansion board leds (4) and buttons (4) - each button enables to toggle the corresponding led,
 * rgb led - fading effect,
 * potentiometer and small speaker - generating sinus wave and changing the frequency using potentiometer,
 * i2c thermometer and serial port - sending the measured temperature through the serial port.

Sample video with some functions shown can be seen:

## How to compile the code

FreeRTOS in this example require hard FPU, so following steps should be used:
 * in <b>View->Configuartion</b> menu, in <b>Compile</b> tab <b>FPU hard</b> option should be choosen,
 * in the same tab in <b>Includepaths</b> path to libraries should be added; in my case it was: 
C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2014q3\arm-none-eabi\lib\armv7e-m\fpu
 * in <b>Link</b> tab in <b>Linked Libraries</b> <b>m</b> libarary should be added.

## IDE and Compiler Setup

This project is created with free CooCox IDE and GNU Tools for ARM.

## Attributions

FreeRTOS code based on:
https://github.com/hmph/FreeRTOS-template-for-STM32F4
and following fork:
https://github.com/tocchi02/FreeRTOS-template-for-STM32F4

My changes:
Some clean-ups, added more task, some changes to FreeRTOSConfig.h - heap size was too low for more than just few simple tasks.

I2C functions from:
http://eliaselectronics.com/
https://github.com/devthrash/STM32F4-examples

Fading RGB based on:
http://www.instructables.com/id/Digispark-RGB-LED-Fader/step4/Arduino-Program/

Details about STM32F4, FPU and CooCox:
http://virtual-shed.blogspot.com/2012/12/fpu-on-stm32f4-with-coocox_18.html

## License

MIT

