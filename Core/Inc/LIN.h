//
// Created by 陈骏骏 on 2023/5/3.
//

#ifndef PRODUCTMANUALTEST_LIN_H
#define PRODUCTMANUALTEST_LIN_H

#include "main.h"
#include "12864.h"
#include "key.h"

//一个同步间隔段 + 1个同步字节（0x55） + PID（1个字节） + （数据帧 （8个字节）  +  校验和（1个字节））
#define LIN_RX_MAXSIZE 12
//PID（1个字节） + （数据帧 （8个字节）  +  校验和（1个字节））
#define LIN_TX_MAXSIZE 10
//不算PID，只有8个数据
#define LIN_CHECK_STD_NUM 8
//PID+8个数据
#define LIN_CHECK_EN_NUM 9

extern uint8_t pLINRxBuff[LIN_RX_MAXSIZE];
extern uint8_t EXV_finished;

//LIN校验模式
typedef enum
{
    //标准校验
    LIN_CK_STANDARD = 0x00U,
    //增强校验
    LIN_CK_ENHANCED = 0x01U
}LIN_CK_Mode;

//LIN_PID+P0+P1,LSB,PID列表
typedef enum
{
    LIN_PID_00_0x00 = 0x80,
    LIN_PID_01_0x01 = 0xC1,
    LIN_PID_02_0x02 = 0x42,
    LIN_PID_03_0x03 = 0x03,
    LIN_PID_04_0x04 = 0xC4,
    LIN_PID_05_0x05 = 0x85,
    LIN_PID_06_0x06 = 0x06,
    LIN_PID_07_0x07 = 0x47,
    LIN_PID_08_0x08 = 0x08,
    LIN_PID_09_0x09 = 0x49,
    LIN_PID_10_0x0A = 0xCA,
    LIN_PID_11_0x0B = 0x8B,
    LIN_PID_12_0x0C = 0x4C,
    LIN_PID_13_0x0D = 0x0D,
    LIN_PID_14_0x0E = 0x8E,
    LIN_PID_15_0x0F = 0xCF,
    LIN_PID_16_0x10 = 0x50,
    LIN_PID_17_0x11 = 0x11,
    LIN_PID_18_0x12 = 0x92,
    LIN_PID_19_0x13 = 0xD3,
    LIN_PID_20_0x14 = 0x14,
    LIN_PID_21_0x15 = 0x55,
    LIN_PID_22_0x16 = 0xD6,
    LIN_PID_23_0x17 = 0x97,
    LIN_PID_24_0x18 = 0xD8,
    LIN_PID_25_0x19 = 0x99,
    LIN_PID_26_0x1A = 0x1A,
    LIN_PID_27_0x1B = 0x5B,
    LIN_PID_28_0x1C = 0x9C,
    LIN_PID_29_0x1D = 0xDD,
    LIN_PID_30_0x1E = 0x5E,
    LIN_PID_31_0x1F = 0x1F,
    LIN_PID_32_0x20 = 0x20,
    LIN_PID_33_0x21 = 0x61,
    LIN_PID_34_0x22 = 0xE2,
    LIN_PID_35_0x23 = 0xA3,
    LIN_PID_36_0x24 = 0x64,
    LIN_PID_37_0x25 = 0x25,
    LIN_PID_38_0x26 = 0xA6,
    LIN_PID_39_0x27 = 0xE7,
    LIN_PID_40_0x28 = 0xA8,
    LIN_PID_41_0x29 = 0xE9,
    LIN_PID_42_0x2A = 0x6A,
    LIN_PID_43_0x2B = 0x2B,
    LIN_PID_44_0x2C = 0xEC,
    LIN_PID_45_0x2D = 0xAD,
    LIN_PID_46_0x2E = 0x2E,
    LIN_PID_47_0x2F = 0x6F,
    LIN_PID_48_0x30 = 0xF0,
    LIN_PID_49_0x31 = 0xB1,
    LIN_PID_50_0x32 = 0x32,
    LIN_PID_51_0x33 = 0x73,
    LIN_PID_52_0x34 = 0xB4,
    LIN_PID_53_0x35 = 0xF5,
    LIN_PID_54_0x36 = 0x76,
    LIN_PID_55_0x37 = 0x37,
    LIN_PID_56_0x38 = 0x78,
    LIN_PID_57_0x39 = 0x39,
    LIN_PID_58_0x3A = 0xBA,
    LIN_PID_59_0x3B = 0xFB,
    LIN_PID_60_0x3C = 0x3C,
    LIN_PID_61_0x3D = 0x7D,
    LIN_PID_62_0x3E = 0xFE,
    LIN_PID_63_0x3F = 0xBF
} LIN_PID_List;

//LIN通信故障反馈
typedef enum
{
    EXV_F_RESP_NO_ERROR = 0x00,
    EXV_F_RESP_ERROR = 0x01
}EXV_F_Response;

//初始化状态反馈
typedef enum
{
    EXV_ST_INIT_NOT = 0x00,
    EXV_ST_INIT_PROCESS = 0x04,
    EXV_ST_INIT_SUCCESS = 0x08
}EXV_St_Initial;

//运行状态反馈
typedef enum
{
    EXV_ST_RUN_NOT_MOVE = 0x00,
    EXV_ST_RUN_MOVING = 0x10
}EXV_St_Running;

//故障状态
typedef enum
{
    EXV_ST_FAULT_NOT = 0x00,
    EXV_ST_FAULT_SHORTED = 0x01,
    EXV_ST_FAULT_OPENLOAD = 0x02,
    EXV_ST_FAULT_OVERTEMP = 0x03,
    EXV_ST_FAULT_ACTUATORFAULT = 0x05
}EXV_St_Fault;

//电压状态
typedef enum
{
    EXV_ST_VOLTAGE_OK = 0x00,
    EXV_ST_VOLTAGE_OVER = 0x10,
    EXV_ST_VOLTAGE_UNDER = 0x20
}EXV_St_Voltage;

//温度状态
typedef enum
{
    EXV_OVERTEMP_OK = 0x00,
    EXV_OVERTEMP_OVER = 0x40
}EXV_W_OverTemp;

//电机信息比较值
typedef enum
{
    EXV_F_RESP_COMP = 0x01,
    EXV_ST_INIT_COMP = 0x0C,
    EXV_ST_RUN_COMP = 0x10,
    EXV_ST_FAULT_COMP = 0x0F,
    EXV_ST_VOLTAGE_COMP = 0x30,
    EXV_OVERTEMP_COMP = 0xC0
}EXV_St_Comp;

typedef enum
{
    EXV_RESP_OK = 0x384,
    EXV_RESP_ERROR = 0x38D,
    EXV_RESP_LIN_COMM_ERROR = 0x385,
    EXV_RESP_MC_SHORT = 0x386,
    EXV_RESP_MC_LOADOPEN = 0x387,
    EXV_RESP_SHUTDOWN = 0x388,
    EXV_RESP_ACTUATOR_FAULT = 0x389,
    EXV_RESP_OVER_VOLTAGE = 0x38A,
    EXV_RESP_UNDER_VOLTAGE = 0x38B,
    EXV_RESP_OVERTEMP = 0x38C,
    EXV_RESP_CHIP_ERROR = 0x38E
}EXV_Resp_Rsult;

void Data_To_LIN(uint16_t step,uint8_t init_enable);
void Finished_LIN(uint8_t send,uint8_t read);
void Send_LIN_Data();
void LIN_Data_Process(uint8_t RxLength);

#endif //PRODUCTMANUALTEST_LIN_H
