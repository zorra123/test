#include "lib.h"

enum
{
	A,
	B,
	C,
	D,
	E,
	F,
	G,
	H
};

void readBufferLib(struct MyGroup *);
void writeBuffLib(struct MyGroup *group, uint32_t *number, uint8_t max_bit_depth);
void configLib(struct MyGroup *group, uint8_t *katod, uint8_t *anod)
{
	int i = 0, j = 0;
	for (j = 0; j < __Num_Anode + 1; j++)//buff = NULL for defolt
	{
		for (i = 0; i < __Num_Katode; i++)
		{
			group->buffer[i][j] = NULL;
		}
	}
	for (i = 0; i < __Num_Katode; i++)//filling katode
	{
		group->Katode.pin[i] = katod[i];
		switch (katod[__Num_Katode + i])
		{
		case(PORT_A):
			group->Katode.port[i] = GPIOA;
			break;
		case(PORT_B):
			group->Katode.port[i] = GPIOB;
			break;
		case(PORT_C):
			group->Katode.port[i] = GPIOC;
			break;
		case(PORT_D):
			group->Katode.port[i] = GPIOD;
			break;
		case(PORT_E):
			group->Katode.port[i] = GPIOE;
			break;
		case(PORT_F):
			group->Katode.port[i] = GPIOF;
			break;
		default:
			group->Katode.port[i] = NULL;
			break;
		}
	}
	for (i = 0; i <  __Num_Anode; i++)//filling anode
	{
		group->Anode.pin[i] = anod[i];
		switch (anod[__Num_Anode + i])
		{
		case(PORT_A):
			group->Anode.port[i] = GPIOA;
			break;
		case(PORT_B):
			group->Anode.port[i] = GPIOB;
			break;
		case(PORT_C):
			group->Anode.port[i] = GPIOC;
			break;
		case(PORT_D):
			group->Anode.port[i] = GPIOD;
			break;
		case(PORT_E):
			group->Anode.port[i] = GPIOE;
			break;
		case(PORT_F):
			group->Anode.port[i] = GPIOF;
			break;
		default:
			group->Anode.port[i] = NULL;
			break;
		}
	}
	group->ptrWriteBuff = &writeBuffLib;
	group->ptrSetHl = &readBufferLib;
}
uint16_t Pow(uint8_t x, uint8_t n) {
	return n == 0 ? 1 : x * Pow(x, n - 1);
}
void writeBuffLib(struct MyGroup *group, uint32_t *number, uint8_t max_bit_depth)
{
	int i = 0;
	uint8_t array[max_bit_depth];
	uint32_t tmp = *number;
	for (i = 0; i < max_bit_depth; i++)
	{
		array[i] = tmp / Pow(10, max_bit_depth - (i + 1));
		tmp = tmp % Pow(10, max_bit_depth - (i + 1));
	}
	for (i = 0; i < max_bit_depth; i++)
	{
		switch (array[i])
		{
		case 1:
			group->buffer[B][i] = 1;
			group->buffer[C][i] = 1;
			break;
		case 2:
			group->buffer[A][i] = 1;
			group->buffer[B][i] = 1;
			group->buffer[G][i] = 1;
			group->buffer[E][i] = 1;
			group->buffer[D][i] = 1;
			break;
			
		case 3:			
			group->buffer[A][i] = 1;
			group->buffer[B][i] = 1;
			group->buffer[G][i] = 1;
			group->buffer[C][i] = 1;
			group->buffer[D][i] = 1;
			break;
		case 4:
			group->buffer[F][i] = 1;
			group->buffer[G][i] = 1;
			group->buffer[B][i] = 1;
			group->buffer[C][i] = 1;			
			break;
		case 5:
			group->buffer[A][i] = 1;
			group->buffer[F][i] = 1;
			group->buffer[G][i] = 1;
			group->buffer[C][i] = 1;
			group->buffer[D][i] = 1;			
			break;
		case 6:
			group->buffer[A][i] = 1;
			group->buffer[F][i] = 1;
			group->buffer[G][i] = 1;
			group->buffer[C][i] = 1;
			group->buffer[D][i] = 1;
			group->buffer[E][i] = 1;
			break;
		case 7:
			group->buffer[A][i] = 1;
			group->buffer[B][i] = 1;
			group->buffer[C][i] = 1;			
			break;
		case 8:
			group->buffer[A][i] = 1;
			group->buffer[B][i] = 1;
			group->buffer[C][i] = 1;
			group->buffer[D][i] = 1;
			group->buffer[E][i] = 1;
			group->buffer[F][i] = 1;
			group->buffer[G][i] = 1;
			break;
		case 9:
			group->buffer[A][i] = 1;
			group->buffer[B][i] = 1;
			group->buffer[C][i] = 1;
			group->buffer[D][i] = 1;
			group->buffer[F][i] = 1;
			group->buffer[G][i] = 1;			
			break;
		case 0:
			group->buffer[A][i] = 1;
			group->buffer[B][i] = 1;
			group->buffer[C][i] = 1;
			group->buffer[D][i] = 1;
			group->buffer[E][i] = 1;
			group->buffer[F][i] = 1;
			break;			
		default: //will be show E aka error
			group->buffer[A][i] = 1;
			group->buffer[F][i] = 1;
			group->buffer[G][i] = 1;
			group->buffer[E][i] = 1;
			group->buffer[D][i] = 1;			
			break;
		}
	}
}
void readBufferLib(struct MyGroup *group)
{
	int i = 0, j = 0;
	for (j = 0; j < __Num_Anode + 1; j++)
	{
		group->Anode.port[j]->__SetBit = 1 << group->Anode.pin[j]; //turn on katode aka GPIO[N]->bsrr =...
		for (i = 0; i < __Num_Katode; i++)
		{
			if (group->buffer[i][j])
			{
				group->Katode.port[i]->__SetBit = 1 << group->Katode.pin[i];  //turn on anode
			}
			else
			{
				group->Katode.port[i]->__ResetBit = 0 << group->Katode.pin[i];  //turn off anode
			}
		}
		group->Anode.port[j]->__ResetBit = 0 << group->Anode.pin[j];   //turn off katode
	}
	for (i = 0; i < __Num_Katode; i++) //turn off all katode
	{
		group->Katode.port[i]->__ResetBit = 0 << group->Katode.pin[i];
	}
}