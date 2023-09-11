//
// Created by 陈骏骏 on 2023/7/9.
//

#ifndef PRODUCTMANUALTEST_12864_H
#define PRODUCTMANUALTEST_12864_H

#include "main.h"
#include "string.h"

#define FIRST_LINE 0x80
#define SECOND_LINE 0x90
#define THIRD_LINE 0x88
#define FOURTH_LINE 0x98

extern uint8_t unusual[4];
extern uint8_t normal[4];

void LCDInit(void);
void DisplayCharacter(uint8_t addr,uint16_t character,uint8_t count);
void DisplayChineseCharacter(uint8_t addr,uint8_t *character,uint8_t count);

#endif //PRODUCTMANUALTEST_12864_H
