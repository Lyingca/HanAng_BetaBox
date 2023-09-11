//
// Created by 陈骏骏 on 2023/7/9.
//
#include "12864.h"

//端口高电平	 OK
#define LCD_RS_H		HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_SET)	//	(对应显示屏的第4脚)
#define	LCD_WR_H		HAL_GPIO_WritePin(LCD_WR_GPIO_Port,LCD_WR_Pin,GPIO_PIN_SET)	//
#define	LCD_E_H			HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin,GPIO_PIN_SET)
#define	LCD_D0_H		HAL_GPIO_WritePin(LCD_D0_GPIO_Port,LCD_D0_Pin,GPIO_PIN_SET)
#define	LCD_D1_H		HAL_GPIO_WritePin(LCD_D1_GPIO_Port,LCD_D1_Pin,GPIO_PIN_SET)
#define	LCD_D2_H		HAL_GPIO_WritePin(LCD_D2_GPIO_Port,LCD_D2_Pin,GPIO_PIN_SET)
#define	LCD_D3_H		HAL_GPIO_WritePin(LCD_D3_GPIO_Port,LCD_D3_Pin,GPIO_PIN_SET)
#define	LCD_D4_H		HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,GPIO_PIN_SET)
#define	LCD_D5_H		HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,GPIO_PIN_SET)
#define LCD_D6_H		HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,GPIO_PIN_SET)
#define LCD_D7_H		HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,GPIO_PIN_SET)
#define LCD_PSB_H		HAL_GPIO_WritePin(LCD_PSB_GPIO_Port,LCD_PSB_Pin,GPIO_PIN_SET)
//NC
#define LCD_RST_H		HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET)	//	(对应显示屏的第17脚)
//vee

//端口低电平
#define LCD_RS_L		HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_RESET)	//	(对应显示屏的第4脚)
#define	LCD_WR_L		HAL_GPIO_WritePin(LCD_WR_GPIO_Port,LCD_WR_Pin,GPIO_PIN_RESET)
#define	LCD_E_L			HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin,GPIO_PIN_RESET)
#define	LCD_D0_L		HAL_GPIO_WritePin(LCD_D0_GPIO_Port,LCD_D0_Pin,GPIO_PIN_RESET)
#define	LCD_D1_L		HAL_GPIO_WritePin(LCD_D1_GPIO_Port,LCD_D1_Pin,GPIO_PIN_RESET)
#define	LCD_D2_L		HAL_GPIO_WritePin(LCD_D2_GPIO_Port,LCD_D2_Pin,GPIO_PIN_RESET)
#define	LCD_D3_L		HAL_GPIO_WritePin(LCD_D3_GPIO_Port,LCD_D3_Pin,GPIO_PIN_RESET)
#define	LCD_D4_L		HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,GPIO_PIN_RESET)
#define	LCD_D5_L		HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,GPIO_PIN_RESET)
#define LCD_D6_L		HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,GPIO_PIN_RESET)
#define LCD_D7_L		HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,GPIO_PIN_RESET)
#define LCD_PSB_L		HAL_GPIO_WritePin(LCD_PSB_GPIO_Port,LCD_PSB_Pin,GPIO_PIN_RESET)
//NC
#define LCD_RST_L		HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET)	//	(对应显示屏的第17脚)

/*
 * uint8_t step_init[] = "目标步数：0";这种方式会有问题，因为项目采用UTF-8编码，所以汉字占3个字节
 * 而显示屏使用的是GB2312的汉字编码，一个汉字占2个字节，所以程序在下载到芯片中后，显示屏现实的是乱码
 * 最终采用硬编码的形式，将文字对应的16进制放到数组中
 * 或者将项目编码方式改成GB2312
 */
uint8_t LINCommunication[] = {0x4c, 0x49, 0x4e, 0x20, 0xcd, 0xa8, 0xd0, 0xc5, 0xa3, 0xba};
uint8_t current_step_init[] = {0xb5, 0xb1, 0xc7, 0xb0, 0xb2, 0xbd, 0xca, 0xfd, 0xa3, 0xba};
uint8_t InitializationState[] = {0xb3, 0xf5, 0xca, 0xbc, 0xbb, 0xaf, 0xa3, 0xba};
uint8_t EXVState[] = {0x45, 0x58, 0x56, 0x20, 0xd7, 0xb4, 0xcc, 0xac, 0xa3, 0xba};
uint8_t unusual[] = {0xd2, 0xec, 0xb3, 0xa3};
uint8_t normal[] = {0xd5, 0xfd, 0xb3, 0xa3};

//因为字节的位是乱的，用到这个函数
void PORT_Assignment(uint8_t WriteData)
{
    if (WriteData & 0x01)  LCD_D0_H;	else	LCD_D0_L;
    if (WriteData & 0x02)  LCD_D1_H;    else	LCD_D1_L;
    if (WriteData & 0x04)  LCD_D2_H;    else	LCD_D2_L;
    if (WriteData & 0x08)  LCD_D3_H;    else	LCD_D3_L;
    if (WriteData & 0x10)  LCD_D4_H;    else	LCD_D4_L;
    if (WriteData & 0x20)  LCD_D5_H;    else	LCD_D5_L;
    if (WriteData & 0x40)  LCD_D6_H;    else	LCD_D6_L;
    if (WriteData & 0x80)  LCD_D7_H;    else	LCD_D7_L;
}

//读忙标志,
void RDBF(void)
{
    uint8_t temp;
    LCD_RS_L;	// LCD_RS_L	指令
    LCD_WR_H;	// LCD_WR_H	读

    while(1)
    {

        LCD_E_H;
        		//读状态字
        LCD_E_L;
        if ((temp&1)==0) break;
    }
}

//写数据到指令寄存器
void WRCommand_M68(uint8_t comm)
{
    LCD_RS_L;
    LCD_WR_L;
    PORT_Assignment(comm);
    ms_Delay(5);
    LCD_E_H;
    ms_Delay(5);
    LCD_E_L;
    ms_Delay(5);
}

//写数据到数据寄存器
void WRData_M68(uint8_t TEMP)
{
    LCD_RS_H;
    LCD_WR_L;
    PORT_Assignment(TEMP);
    ms_Delay(5);
    LCD_E_H;
    ms_Delay(5);
    LCD_E_L;
    ms_Delay(5);
}

//addr为汉字显示位置,*character汉字指针;count为输入汉字串字符数
void DisplayChineseCharacter(uint8_t addr,uint8_t *character,uint8_t count)
{
    uint8_t i;
    WRCommand_M68(addr);	//设定DDRAM地址
    ms_Delay(5);
    for(i = 0;i < count;i++)
    {
        WRData_M68(*(character + i));
        ms_Delay(5);
    }
}

//addr为半宽字符首个地址,character为首个半宽字符代码,count为需要输入字符个数
void DisplayCharacter(uint8_t addr,uint16_t character,uint8_t count)
{
    uint8_t num[8] = {0};
    uint16_t test_num = 0;
    uint8_t index = 7,digital = 0;
    test_num = character;
    while(test_num)
    {
         digital = test_num % 10;
         test_num /= 10;
         num[index--] = digital;
    }
    WRCommand_M68(addr);	//设定DDRAM地址
    ms_Delay(5);
    for (int j = 8 - count; j < 8; ++j) {
        WRData_M68(0x30 + num[j]);
        ms_Delay(5);
    }
}

//初始化LCD-8位接口
void LCDInit(void)
{
    LCD_RST_H;
    LCD_RST_L;
    HAL_Delay(40);
    LCD_RST_H;		//硬复位
    HAL_Delay(1);
    LCD_PSB_H;
    WRCommand_M68(0x30);//基本指令集,8位
    HAL_Delay(1);

    WRCommand_M68(0x0C);	//显示状态开关：整体显示开，光标显示关，光标显示反白关
    HAL_Delay(1);
    WRCommand_M68(0x01);	//清除显示DDRAM
    HAL_Delay(11);
    WRCommand_M68(0x06);	//启始点设定：光标右移
    HAL_Delay(1);
    WRCommand_M68(0x02);//地址归零
    HAL_Delay(1);

    uint8_t size = sizeof(LINCommunication) / sizeof(uint8_t);
    DisplayChineseCharacter(0x80,LINCommunication,size);
    size = sizeof(current_step_init) / sizeof(uint8_t);
    DisplayChineseCharacter(0x90,current_step_init,size);
    size = sizeof(InitializationState) / sizeof(uint8_t);
    DisplayChineseCharacter(0x88,InitializationState,size);
//    size = sizeof(EXVState) / sizeof(uint8_t);
//    DisplayChineseCharacter(0x98,EXVState,size);
}