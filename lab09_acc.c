#include "stm32f4xx_conf.h"
#include "stm32f4_discovery_lis302dl.h"

int8_t accX = 5;
int8_t accY = 55;
int8_t accZ = -5;

int lab09_acc(void)
{
	SystemInit();

	LIS302DL_InitTypeDef  LIS302DL_InitStruct;
	/* Set configuration of LIS302DL*/
	LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
	LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
	LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
	LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
	LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
	LIS302DL_Init(&LIS302DL_InitStruct);

	/* Required delay for the MEMS Accelerometer: Turn-on time = 3/Output data Rate = 3/100 = 30ms */
	// TODO
	uint16_t i;
	for(i=0; i<10000; ++i)
	{
		uint16_t j;
		for(j=0; j<1000; ++j)
		{
			asm("nop");
		}
	}

	// WHO_AM_I (0Fh)


	int8_t przyspieszenie_x;
	int8_t przyspieszenie_y;
	int8_t przyspieszenie_z;

	for(;;)
	{
		LIS302DL_Read(&przyspieszenie_x, LIS302DL_OUT_X_ADDR, 1);
		LIS302DL_Read(&przyspieszenie_y, LIS302DL_OUT_Y_ADDR, 1);
		LIS302DL_Read(&przyspieszenie_z, LIS302DL_OUT_Z_ADDR, 1);

		LIS302DL_Read(&przyspieszenie_x, LIS302DL_OUT_X_ADDR, 1);
		if(przyspieszenie_x>127)
		{
			przyspieszenie_x=przyspieszenie_x-1;
			przyspieszenie_x=(~przyspieszenie_x)&0xFF;
			przyspieszenie_x=-przyspieszenie_x;
			accZ = -100;
		}
		LIS302DL_Read(&przyspieszenie_y, LIS302DL_OUT_Y_ADDR, 1);
		if(przyspieszenie_y>127)
		{
			przyspieszenie_y=przyspieszenie_y-1;
			przyspieszenie_y=(~przyspieszenie_y)&0xFF;
			przyspieszenie_y=-przyspieszenie_y;
			accZ = -100;
		}
		LIS302DL_Read(&przyspieszenie_z, LIS302DL_OUT_Z_ADDR, 1);
		if(przyspieszenie_z>127)
		{
			przyspieszenie_z=przyspieszenie_z-1;
			przyspieszenie_z=(~przyspieszenie_z)&0xFF;
			przyspieszenie_z=-przyspieszenie_z;
			accZ = -100;
		}

		accX = przyspieszenie_x;
		accY = przyspieszenie_y;



		/*
		accX = przyspieszenie_x;
		if(przyspieszenie_x > 127)
		{
			accY = -(~(przyspieszenie_x-1));
		}
		else
		{
			accY = przyspieszenie_x;
		}

		if(accX != accY)
		{
			accZ = 100;
		}
		else
		{
			accZ = 0;
		}
		*/

	}

	return 0;
}
