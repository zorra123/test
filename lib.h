#pragma once
#include "stm32f0xx_hal.h"
#define __Num_Katode 8
#define __Num_Anode 4
#define __SetBit BSRR
#define __ResetBit BRR

enum
{
	PORT_A = 1,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E,
	PORT_F	
};
struct MyGroup
{
	//USE ONLY IN APPLIB NOT IN APP!!!!
	uint8_t buffer[__Num_Katode][__Num_Anode];  //the last line is for anod. Ex: buf[__Num_Katode][0] is mean that katode 0 is on, etc.
	//USE ONLY IN APPLIB NOT IN APP!!!!
	void(*ptrSetHl)(struct MyGroup *group); //readBuffer
	//USE ONLY IN APPLIB NOT IN APP!!!!
	void(*ptrWriteBuff)(struct MyGroup *group, uint32_t *number, uint8_t max_bit_depth);   //WriteBuff
	//USE ONLY IN APPLIB NOT IN APP!!!!
	struct MyKatode
	{
		uint8_t pin[__Num_Katode];
		GPIO_TypeDef *port[__Num_Katode];
	}Katode;
	//USE ONLY IN APPLIB NOT IN APP!!!!
	struct MyAnode
	{
		uint8_t pin[__Num_Anode];
		GPIO_TypeDef *port[__Num_Anode];
	}Anode;
};
//USE ONLY IN APPLIB NOT IN APP!!!!
//before stat to using u must config __Num_Katode && __Num_Anode in lib.h!!!
//for each group u must reconfig init_katod and init_anod!!!
//so if u use 2 or more groups with common anod, u must duplicate anod
void configLib(struct MyGroup *, uint8_t *, uint8_t*);

//void config(struct MyGroup *, uint8_t *, uint8_t *);