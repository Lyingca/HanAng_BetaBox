//
// Created by 陈骏骏 on 2023/5/3.
//

#include "LIN.h"
#include "usart.h"
#include "key.h"
#include <string.h>

//LIN同步帧字节
uint8_t  SYNC_Frame = 0x55;

//LIN接收数据缓存
uint8_t pLINRxBuff[LIN_RX_MAXSIZE];
//LIN发送数据缓存
uint8_t pLINTxBuff[LIN_TX_MAXSIZE];
//当前测试的电机步长
uint16_t EXV_Test_Step;
//发送读取帧的标志位
uint8_t LIN_Read_Flag = ENABLE;
//发送写帧的标志位
uint8_t LIN_Send_Flag = DISABLE;
//保存LIN芯片的信息
struct LIN_Chip_Msg
{
    //读PID
    uint8_t read_PID;
    //写PID
    uint8_t write_PID;
    //电机运动使能
    uint8_t EXV_Move_Enable;
    //初始化请求
    uint8_t EXV_Init_Request;
    //非初始化请求
    uint8_t EXV_Not_Init_Request;
};
//初始化LIN芯片信息
struct LIN_Chip_Msg chip[3] = {
        {LIN_PID_53_0x35,LIN_PID_52_0x34,0xFF,0xFD,0xFC},
        {LIN_PID_55_0x37,LIN_PID_54_0x36,0xFF,0xFD,0xFC},
        {LIN_PID_32_0x20,LIN_PID_16_0x10,0xFF,0xFD,0xFC}
};
//芯片编号
uint8_t chip_Num = 0;
//电机完成动作标志位
uint8_t EXV_finished = 0;

/****************************************************************************************
** 函数名称: LINCheckSum----标准校验
** 功能描述: 计算并返回LIN校验值
** 参    数:  uint8_t *buf：需要计算的数组
			        uint8_t lens：数组长度
** 返 回 值:   uint8_t ckm: 计算结果
****************************************************************************************/
uint8_t LIN_Check_Sum(uint8_t *buf, uint8_t lens)
{
    uint8_t i, ckm = 0;
    uint16_t chm1 = 0;
    for(i = 1; i < lens; i++)
    {
        chm1 += *(buf+i);
    }
    ckm = chm1 / 256;
    ckm = ckm + chm1 % 256;
    ckm = 0xFF - ckm;
    return ckm;
}
/****************************************************************************************
** 函数名称: LINCheckSumEn----增强校验
** 功能描述: 计算并返回LIN校验值
** 参    数:  uint8_t *buf：需要计算的数组
			        uint8_t lens：数组长度
** 返 回 值:   uint8_t ckm: 计算结果
****************************************************************************************/
uint8_t LIN_Check_Sum_En(uint8_t *buf, uint8_t lens)
{
    uint8_t i, ckm = 0;
    uint16_t chm1 = 0;
    for(i = 0; i < lens; i++)
    {
        chm1 += *(buf+i);
    }
    ckm = ~(chm1 % 255);
    return ckm;
}
/****************************************************************************************
** 函数名称: Lin_Tx_PID_Data
** 功能描述: LIN发送数据帧
** 参    数: *buf:数组地址；buf[0]=PID
			       lens:数据长度,不含校验字节
			       CK_Mode: 校验类型增强型LIN_CK_ENHANCED=1：基本LIN_CK_STANDARD=0
             Timeout (0xffff)不做时间限制
** 返 回 值: 无
****************************************************************************************/
void LIN_Tx_PID_Data(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t lens, LIN_CK_Mode CK_Mode)
{
    if(CK_Mode == LIN_CK_STANDARD)
    {
        //arr[i] = *(arr + i)
        //计算标准型校验码，不计算PID
        *(buf + lens) = LIN_Check_Sum(buf, LIN_CHECK_STD_NUM);
    }
    else
    {
        //计算增强型校验码,连PID一起校验
        *(buf + lens) = LIN_Check_Sum_En(buf, LIN_CHECK_EN_NUM);
    }

    //发送同步间隔段
    HAL_LIN_SendBreak(huart);
    //发送同步段
    HAL_UART_Transmit(huart,&SYNC_Frame,1,HAL_MAX_DELAY);
    //发送PID,数据内容和校验
    HAL_UART_Transmit(huart,buf,LIN_TX_MAXSIZE,HAL_MAX_DELAY);
}
/****************************************************************************************
** 函数名称: Lin_Tx_PID
** 功能描述: LIN发送报文头，PID，读取从机状态信息
** 参    数: PID, Timeout (0xffff)不做时间限制
** 返 回 值: 无
****************************************************************************************/
void LIN_Tx_PID(UART_HandleTypeDef *huart, uint8_t PID)
{
    //发送间隔帧
    HAL_LIN_SendBreak(huart);
    //发送同步帧
    HAL_UART_Transmit(huart,&SYNC_Frame,1,HAL_MAX_DELAY);
    HAL_UART_Transmit(huart,&PID,1,HAL_MAX_DELAY);
}

void Data_To_LIN(uint16_t step,uint8_t init_enable)
{
    LIN_Send_Flag = DISABLE;
    uint8_t index = 0;
    EXV_Test_Step = step;

    pLINTxBuff[index++] = chip[chip_Num].write_PID;
    pLINTxBuff[index++] = step & 0xFF;
    pLINTxBuff[index++] = step >> 8;
    pLINTxBuff[index++] = chip[chip_Num].EXV_Move_Enable;
    if(init_enable)
    {
        pLINTxBuff[index++] = chip[chip_Num].EXV_Init_Request;
    }
    else
    {
        pLINTxBuff[index++] = chip[chip_Num].EXV_Not_Init_Request;
    }
    //剩余的字节数有0xFF填充
    while(index < LIN_TX_MAXSIZE - 1)
    {
        pLINTxBuff[index++] = 0xFF;
    }
    LIN_Send_Flag = ENABLE;
}

void Finished_LIN(uint8_t send,uint8_t read)
{
    LIN_Send_Flag = send;
    LIN_Read_Flag = read;
}

/**
 * 发送LIN数据，包括读取帧和写帧
 */
void Send_LIN_Data()
{
    if(LIN_Send_Flag)
    {
        LIN_Tx_PID_Data(&huart2,pLINTxBuff,LIN_TX_MAXSIZE - 1,LIN_CK_ENHANCED);
        HAL_Delay(20);
    }
    if(LIN_Read_Flag)
    {
        LIN_Tx_PID(&huart2, chip[chip_Num].read_PID);
        HAL_Delay(50);
    }
}

/**
 * 数据处理函数
 */
void LIN_Data_Process(uint8_t RxLength)
{
    //电机转动步长
    uint16_t EXV_Run_Step = 0;
    //通过校验位-校验数据
    uint8_t ckm = 0;
    //pLINRxBuff + 2表示从接收的第3个数据开始，因为接收数组第1个是同步间隔段，第2个是同步段（0x55）
    ckm = LIN_Check_Sum_En(pLINRxBuff + 2,LIN_CHECK_EN_NUM);
    //检查电机与测试板之间的连接是否正常
    if (RxLength < LIN_RX_MAXSIZE)
    {
        //LCD显示屏第一行显示LIN通信异常
        DisplayChineseCharacter(FIRST_LINE + 5,unusual,sizeof(unusual) / sizeof(uint8_t));
    }
    //如果校验不通过，丢弃这帧数据
    else if(ckm != pLINRxBuff[LIN_RX_MAXSIZE - 1] || pLINRxBuff[2] == chip[chip_Num].write_PID)
    {
        //无操作
    }
    //校验LIN通信故障反馈
    else if((pLINRxBuff[3] & EXV_F_RESP_COMP) == EXV_F_RESP_ERROR)
    {
        //无操作
    }
    //检查初始化状态，解决反馈数据中以E2，E3开始的数据帧
    else if((pLINRxBuff[3] & EXV_ST_INIT_COMP) == EXV_ST_INIT_NOT || (pLINRxBuff[3] & EXV_ST_INIT_COMP) == EXV_ST_INIT_PROCESS)
    {
        //LCD显示屏第三行显示初始化
        DisplayChineseCharacter(THIRD_LINE + 4,"Init    ", strlen("Init    "));
    }
    //电机停止转动
    else if((pLINRxBuff[3] & EXV_ST_RUN_COMP) == EXV_ST_RUN_NOT_MOVE)
    {
        //LCD显示屏第四行显示EXV停止运动
//        DisplayChineseCharacter(FOURTH_LINE + 5,"Stop  ", strlen("Stop  "));
        //LCD显示屏第三行显示初始化结束
        DisplayChineseCharacter(THIRD_LINE + 4,"Finished", strlen("Finished"));
        //计算电机转动步长，步长低字节在前高字节在后
        EXV_Run_Step = (pLINRxBuff[6] << 8) | pLINRxBuff[5];
        if(EXV_Run_Step == EXV_Test_Step)
        {
            EXV_finished = 1;
            DisplayCharacter(SECOND_LINE + 5,EXV_Run_Step,3);
        }
    }
    else
    {
        //LCD显示屏第三行显示初始化结束
        DisplayChineseCharacter(THIRD_LINE + 4,"Finished", strlen("Finished"));
        //计算电机转动步长，步长低字节在前高字节在后
        EXV_Run_Step = (pLINRxBuff[6] << 8) | pLINRxBuff[5];
        //LCD显示屏第二行显示当前步数
        DisplayCharacter(SECOND_LINE + 5,EXV_Run_Step,3);
    }

    if(RxLength == LIN_RX_MAXSIZE)
    {
        //LCD显示屏第一行显示LIN通信正常
        DisplayChineseCharacter(FIRST_LINE + 5,normal, sizeof(normal) / sizeof(uint8_t));
    }

    //这帧数据解析完成，清空接收缓存数据
    memset(pLINRxBuff,0,LIN_RX_MAXSIZE);
}