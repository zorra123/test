#include "AppLib.h"
void config(struct MyGroup *group, uint8_t num_group)
{
	uint8_t init_katod[2][2][__Num_Katode] = {
		9,
		2,
		9,
		4,
		5,
		6,
		7,
		8,
		PORT_C,
		PORT_B,
		PORT_C,
		PORT_D,
		PORT_E,
		PORT_F,
		PORT_B,
		PORT_B, 
		1,
		2,
		9,
		4,
		5,
		6,
		7,
		8,
		PORT_A,
		PORT_B,
		PORT_C,
		PORT_D,
		PORT_E,
		PORT_F,
		PORT_B,
		PORT_B,
	};
	uint8_t init_anod[2][2][__Num_Anode] = {
		9,
		10,
		11,
		12,
		PORT_F,
		PORT_E,
		PORT_D,
		PORT_C,
		9,
		10,
		11,
		12,
		PORT_F,
		PORT_E,
		PORT_D,
		PORT_C
	};
	for (int i = 0; i < num_group; i++)
	{		
		configLib(&group[i], &init_katod[i][0][0], &init_anod[i][0][0]);
	}
}

void writeBuffer(struct MyGroup *groups, uint32_t *numbers, uint8_t num_group, uint8_t max_bit_depth)
{
	for (int i = 0; i < num_group; i++)
	{
		groups[i].ptrWriteBuff(&groups[i], &numbers[i], max_bit_depth);
	}
}

void readBuffer(struct MyGroup *groups, uint8_t num_groups)
{
	for (int i = 0; i < num_groups; i++)
	{
		groups[0].ptrSetHl(&groups[0]);
	}
}