//
// Created by 陈骏骏 on 2023/5/3.
//

#ifndef PRODUCTMANUALTEST_KEY_H
#define PRODUCTMANUALTEST_KEY_H

#include "main.h"
#include "12864.h"

/** 按键按下标置宏
 * 按键按下为高电平，设置 KEY_ON=1， KEY_OFF=0
 * 若按键按下为低电平，把宏设置成KEY_ON=0 ，KEY_OFF=1 即可
 */
#define KEY_ON    1
#define KEY_OFF   0
#define STEP_DIGITAL 1
#define LOOP_DIGITAL 0

//当前步长
extern uint16_t currentStepSize;
//当前循环次数
extern uint16_t currentCycleCount;

uint8_t General_Key_Scan(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin);
void Operation_Key_Scan(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin,uint8_t step,uint8_t step_loop);

#endif //PRODUCTMANUALTEST_KEY_H
