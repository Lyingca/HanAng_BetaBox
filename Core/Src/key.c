//
// Created by 陈骏骏 on 2023/5/3.
//

#include "key.h"

//当前步长
uint16_t currentStepSize;

void Update_Data(uint8_t step);

/**
 * 普通按键的检测
 * @param GPIOx
 * @param GPIO_Pin
 * @return
 */
uint8_t General_Key_Scan(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
    /*检测是否有按键按下 */
    if (HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )
    {
        /*等待按键释放 */
        while (HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);
        return KEY_ON;
    }
    else
    {
        return KEY_OFF;
    }
}

/**
 * 运算按键的检测
 * @param GPIOx
 * @param GPIO_Pin
 * @param Number_Of_Symbols
 */
void Operation_Key_Scan(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin,uint8_t step)
{
    /*检测是否有按键按下 */
    if (HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )
    {
        /*等待按键释放 */
        while (HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON)
        {
            Update_Data(step);
            //延迟50ms,再次检测
            HAL_Delay(50);
        }
    }
}

/**
 * 更新LED的数字
 * @param step
 * @param hi2c
 * @return
 */
void Update_Data(uint8_t step)
{
    if (step)
    {
        if (currentStepSize == 999)
        {
            currentStepSize = 0;
        }
        else
        {
            ++currentStepSize;
        }
    }
    else
    {
        if (currentStepSize == 0)
        {
            currentStepSize = 999;
        }
        else
        {
            --currentStepSize;
        }
    }
    DisplayCharacter(FOURTH_LINE + 5,currentStepSize,3);
}