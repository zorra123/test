#pragma once
#include "lib.h"
//USE IN APP
void config(struct MyGroup *group, uint8_t num_group);
//USE IN APP
void writeBuffer(struct MyGroup *groups, uint32_t *numbers, uint8_t num_group, uint8_t max_bit_depth);
//USE IN APP
//it maybe in timer. Ex:
//readBuffer(&group,2);
void readBuffer(struct MyGroup *groups, uint8_t num_groups);