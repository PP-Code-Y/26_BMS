/*
 * daisy.c
 *
 *  Created on: 2022年3月5日
 *      Author: BBBBB
 *      修改芯片需要更改:TemRes_Cal
 *      修改电池数量记得改(*bms).Alarm.Openwire_Vol[]的数组总量
 *		SPI的DMA传输无法调通, 就会在数据跑通一段时间后失效, 可能原因是DMA收发寄存器那边有乱号了, 这是希望以后能改正的, 所以目前没用上DMA
 */

 /* 
    daisy-user.c
    功能
 */
#include <stdlib.h>
#include "daisy-user.h"
#include "os-user.h"
#include "relay-user.h"
#include "state-user.h"
#include "main.h"
#include "data-user.h"

/*全局变量*/
extern BMS E03;
extern vu8 Hall_Count; // 菊花链通信计数/100ms/LSB
extern float Standard_Res[94];
extern u16 Tem[94];
extern vu8 Balance_flag;
// EF250810

MODULE Module[Total_ic] = {0};
cell_asic IC[Total_ic];
u8 n_Cell[Total_ic];
cell_asic tempIC_a[Total_ic]; /*开路检测的存储数组*/
cell_asic tempIC_b[Total_ic];

u16 MaxCellVoltage;   // 最高单体电压
u16 MaxVoltageCellNO; // 最高单体电压单体序号
u16 MinCellVoltage;   // 最低单体电压
u16 MinVoltageCellNO; // 最低单体电压单体序号

u16 MaxCellTem;    // 最高单体温度
u16 MaxTempCellNO; // 最高单体温度单体序号
u16 MinCellTem;    // 最低单体温度
u16 MinTempCellNO; // 最低单体温度单体序号

float conv_time_Vol;      /*ADCV命令时长*/
float conv_time_Temp;     /*ADAX命令时长*/
float conv_time_Openwire; /*Openwire_check命令时长*/

/*
    @brief: 模组结构体数值初始化
    @param: module: 模组结构体指针
            total_ic: 模组IC总数
    @return: 无
*/
void Module_Init(MODULE *module, u8 total_ic)
{
    u8 nic;
    for (nic = 0; nic < total_ic; nic++) {
        for (u8 i = 0; i < Max_ncell; i++) {
            /*初始化每个模组的每片电芯为4000mv,
            只要对于15电池的组不修改就可以,避免到时候判断断线或者过压时候麻烦（啥意思？？？）*/
            module[nic].CellVol[i] = 0XFA0; 
        }
        for (u8 i = 0; i < nGPIO; i++) {
            /*先大概写一个温度上去,现在也不知道是不是这么做*/
            module[nic].CellTem[i] = 26; 
        } 
        /*初始化为80V, 控制精度为0.01*/
        module[nic].ModuleVol = 80000; 
        /*全部先关闭, 不是很严谨*/
        module[nic].DCC       = 0;     
    }
}

/*
    @brief:读取所有菊花链内所有电压,储存在详细版的ic[c_ic].cells.c_codes之中
    
*/
void CV_Measure(u8 total_ic, cell_asic *ic)
{
    // u8 Channel = 1;
    ///*方案一*/
    //	wakeup_sleep(total_ic);
    //	LTC681x_clrcell();										//清除电压寄存器位, 所以不会有读到上一次测量的可能性

    //	LTC6813_mute();											//ADC前禁用均衡
    ////	wakeup_sleep(total_ic);
    //	LTC681x_adcv(MD_7KHZ_3KHZ , DCP_DISABLED , CH_ALL);		//ADCV mode
    //	LTC681x_pollAdc();										//轮询查询电压转换是否完成
    ////	wakeup_sleep(total_ic);
    //	LTC6813_unmute();										//ADC后可以均衡
    //	wakeup_sleep(total_ic);
    //	LTC681x_rdcv(REG_ALL,total_ic, ic);						//读取IC
    //	LTC681x_clrcell();
    
    /*方案二*/
    wakeup_sleep(total_ic);
    LTC681x_clrcell(); // 清除电压寄存器位, 所以不会有读到上一次测量的可能性
    LTC6813_mute();    // ADC前禁用均衡
    /* P75，因为放电晶体管的关闭速度不够快，
        无法在测量开始之前完全使电池电压稳定下来。
        为了在放电时获得最佳的测量精度，应使用MUTE和
        UNMUTE命令。在发出ADCV命令之前，可以发出MUTE命令
        以暂时禁用所有放电晶体管。单元转换完成后，可以发
        送UNMUTE来重新启用所有先前导通的放电晶体管。使用
        这种方法可以在极短的时间损失下最大化测量精度。
    */
    wakeup_sleep(total_ic);
    LTC681x_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CH_ALL); // ADCV mode，选择000通道，即全部通道的读取
    LTC681x_pollAdc();                                // 轮询查询电压转换是否完成,没有读到每次

    LTC6813_unmute(); // ADC后可以均衡
    wakeup_sleep(total_ic);
    LTC681x_rdcv(REG_ALL, total_ic, ic); // 读取IC
    LTC681x_clrcell();                   // 读取完后清除寄存器
}

/* 将ic的ic[c_ic].cells.c_codes转移到Module中的CellVol，也计算模组总压ModuleVol
 * 通过累加计算了模组总压, 同时将电压数据储存到Module之中*/
void CV_Transfer(u8 total_ic, u8 *nCell, cell_asic *ic, MODULE *module)
{
    for (u8 nic = 0; nic < total_ic; nic++) {                         // 对整个模组操作
        module[nic].ModuleVol = 0;                                    // 先给累计总压清零
        for (u8 i = 0; i < nCell[nic]; i++) {                         // 对模组的每个电芯操作
                                                                      /*
                                                                          // 作弊操作
                                                                          if (ic[nic].cells.c_codes[i] < 36500 || ic[nic].cells.c_codes[i] > 41500) {
                                                                              ic[nic].cells.c_codes[i] = ic[nic].cells.c_codes[1];
                                                                          }
                                                                          // 作弊操作
                                                                      */
            module[nic].CellVol[i] = (ic[nic].cells.c_codes[i] / 10); // 转移单体电压，同时进行了单位转换，0.1mV -> 1mV
            module[nic].ModuleVol += module[nic].CellVol[i];          // 计算模组累计总压
        }
    }
    /*
    for (uint8_t i = 4; i < nCell[1]; i++) {
        module[1].CellVol[i] = module[4].CellVol[i - 4];
    }*/
}

/* 辅助函数, 计算出现最大最小电压/温度时候的序号
 * 参数: nic,当前第几个IC
 * 		 n,当前第几个电池
 * 返回: 计算出来的序号值*/
u8 NO_cal(u8 current_ic, u8 n, u8 *nCell)
{
    u16 number = 0;
    for (u8 i = 0; i < current_ic; i++) {
        number += nCell[i]; // // 因为i从0开始，所以要计算第i个模组第n片电芯的电压，只能加到(current_ic-1) 第i个模组即nCell[current_ic-1]
    }
    number += n; // 再补上nCell[current_ic-1]的第n个电芯即可
    return number;
}
/*
    @brief: 寻找最大最小单体电压值
    Mark_Max:最大单体电压对应的编号, 高8位表示第几个模块, 低8位表示该模组下第几个电池
之后判断是否超阈值变量
*/
void CV_Find_Max_Min(u8 total_ic, u8 *nCell, MODULE *module,
                     u16 *MaxVol, u16 *Mark_Max, u16 *MinVol, u16 *Mark_Min)
{
    *MaxVol   = module[0].CellVol[0]; // 初始化, 不初始化的话假如有一次测得电压最小就不能改了
    *MinVol   = module[0].CellVol[0];
    *Mark_Max = 0;
    *Mark_Min = 0;
    for (u8 nic = 0; nic < total_ic; nic++) {
        if (module[nic].CellVol[0] > 3500) {
            module[nic].MinCellVoltage_Module = module[nic].CellVol[0]; // 初始化，每次判断前先赋值首位单体电压
        } else {
            module[nic].MinCellVoltage_Module = module[nic].CellVol[1];
        }
    }
    for (u8 nic = 0; nic < total_ic; nic++) {                                            // 对每个模组进行操作
        for (u8 i = 0; i < nCell[nic]; i++) {                                            // 对每个模组的电芯进行操作
            if ((module[nic].CellVol[i] < *MinVol) && (module[nic].CellVol[i] > 2000)) { // 找到更小的单体电压值, 而且因为断线采集到的电压有问题, 所以做滤波
                *MinVol   = module[nic].CellVol[i];
                *Mark_Min = NO_cal(nic, i, nCell);
            }
            if ((module[nic].CellVol[i] > *MaxVol) && (module[nic].CellVol[i] < 4400)) { // 找到更大的单体电压值
                *MaxVol   = module[nic].CellVol[i];
                *Mark_Max = NO_cal(nic, i, nCell);
            }

            // 这里3500是针对均衡的滤波，即如果过于低不算做单个模组内的最低单体电压，导致影响均衡判断
            if ((module[nic].CellVol[i] < module[nic].MinCellVoltage_Module) && (module[nic].CellVol[i] > 3500)) { // 找到当前模组内的更小的单体电压值, 而且因为断线采集到的电压有问题, 所以做滤波
                module[nic].MinCellVoltage_Module = module[nic].CellVol[i];
            }
        }
    }
}

/* 将单体电压跟最低电压的1.05倍(相差5%)比较.
 * 先把所有DCC位置0, 如果超过阈值则将对应的DCC位置1
 * 因为是辅助函数, 所以函数后要跟一个wrcfg+wrcfgb函数写入开启均衡*/
void DCC_detect(u8 total_ic, u8 *nCell,
                cell_asic *ic, MODULE *module, u16 MinVol)
{
    u16 Balance_flag;
    Balance_flag = 50; // 设置均衡阈值,50mv

    LTC681x_clear_discharge(total_ic, ic);                                                     // 清除ic内所有DCC位置0,之后超了阈值会被置1的
    for (u8 nic = 0; nic < total_ic; nic++) {                                                  // 对每个模组进行操作
        for (u8 i = 0; i < nCell[nic]; i++) {                                                  // 对每个模组的电芯进行操作
            if ((module[nic].CellVol[i] - module[nic].MinCellVoltage_Module) > Balance_flag) { // 超过最低电压的5%,应该开启均衡
                LTC6813_set_discharge(nic, i, ic);                                             // 将ic内对应DCC位设置为1
            }
        }
    }
}

/* 将cfga和cfgb的状态值配置好, 同时设置好DCC位
 * 之后发送命令修改DCC位, 开启均衡*/
void Balance_Open(u8 total_ic, u8 *nCell,
                  cell_asic *ic, MODULE *module, u16 MinVol)
{
    LTC681x_init_cfg(total_ic, ic);                  // 将所有参数都设置成默认值, 不确定需不需要
    LTC6813_init_cfgb(total_ic, ic);                 // 将所有参数都设置成默认值, 不确定需不需要
    DCC_detect(total_ic, nCell, ic, module, MinVol); // 配置ic内的DCC位

    wakeup_sleep(total_ic);
    LTC6813_wrcfg(total_ic, ic); // 发送命令,修改DCC位,配置寄存器组A
    wakeup_sleep(total_ic);
    LTC6813_wrcfgb(total_ic, ic); // 发送命令,修改DCC位，配置寄存器组B
}

void Balance_Close(u8 total_ic, cell_asic *ic)
{
    LTC681x_init_cfg(total_ic, ic);  // 将所有参数都设置成默认值
    LTC6813_init_cfgb(total_ic, ic); // 将所有参数都设置成默认值

    wakeup_sleep(total_ic);
    LTC6813_wrcfg(total_ic, ic); // 发送命令,修改DCC位,配置寄存器组A
    wakeup_sleep(total_ic);
    LTC6813_wrcfgb(total_ic, ic); // 发送命令,修改DCC位，配置寄存器组B
}

/* brief: 该函数将读回来的DCC位组合成一个u32位的数组,然后传输到module[nic].DCC之中
 * param: 	len: 数组长度
 * 			data：要发的数据数组*/
void DCC_Transfer(u8 total_ic, cell_asic *ic, u8 *nCell, MODULE *module)
{
    u32 temp[14];
    for (int i = 0; i < total_ic; i++) {
        temp[i] = 0;
    }

    // 临时储存u32的DCC,之后会转移到module[nic].DCC
    u32 temp_DCC = 0;
    wakeup_sleep(total_ic);
    LTC6813_rdcfg(total_ic, ic); // 读取DCC位
    wakeup_sleep(total_ic);
    LTC681x_rdcfgb(total_ic, ic); // 读取DCC位

    /*将每个模组的18位DCC(不管有没有18个)全部拿到temp上来, nic从0-6各代表一个模组*/
    for (u8 nic = 0; nic < total_ic; nic++) {
        temp[nic] |= ((u32)ic[nic].config.rx_data[4]);                   // 配置寄存器组A 4号子寄存器用于存储DCC 0-9位
        temp[nic] |= (((u32)(ic[nic].config.rx_data[5] & 0X0F)) << 8);   // 以上DCC1-12，这一行是取低四位再左移8位与前面拼起来
        temp[nic] |= (((u32)(ic[nic].configb.rx_data[0] & 0XF0)) << 8);  // DCC13-16只取高四位的,因此只需要左移8位就可以到12位之后
        temp[nic] |= (((u32)(ic[nic].configb.rx_data[1] & 0X03)) << 16); // DCC17-18
        module[nic].DCC = temp[nic];                                     // 完成DCC位的转移
    }
    /*将module结构体的single_DCC数组初始化为0*/
    for (u8 nic = 0; nic < total_ic; nic++) { // single_DCC数组即存放每个电芯单元单独的DCC位1/0
        for (u8 cell = 0; cell < Max_ncell; cell++) {
            module[nic].single_DCC[cell] = 0;
        }
    }
    /* 下面算法将module[nic].DCC逐个提取到module[nic].single_DCC[cell]里面
     * 举个例子: 比如DCC2==1的时候,那module[nic].DCC==......10,也就是倒数第二位为1
     * 此时为了将DCC2=1放到module[nic].single_DCC[1]里面, 需要先将temp_DCC取成10, 再右移1位
     * 就可以得到module[nic].single_DCC[1]==1
     * */
    for (u8 nic = 0; nic < total_ic; nic++) {          // 对每个模组进行操作
        for (u8 cell = 0; cell < nCell[nic]; cell++) { // 对每个电芯进行操作
            temp_DCC                     = 0;
            temp_DCC                     = (module[nic].DCC & (1 << cell));
            module[nic].single_DCC[cell] = (temp_DCC >> cell); // 逐个找值
        }
    }
}
/*
该函数测出所有IC的GPIO口,
*/
void Tem_Measure(u8 total_ic, cell_asic *ic)
{
    wakeup_sleep(total_ic); // 唤醒
    LTC681x_clraux();       // 清除GPIO电压寄存器值

    wakeup_sleep(total_ic);                 // 唤醒
    LTC681x_adax(MD_7KHZ_3KHZ, AUX_CH_ALL); // ADAX mode, 要等待至少3.9ms
    LTC681x_pollAdc();                      // 在ADCV完成前阻断程序运行并计算时间

    wakeup_sleep(total_ic);               // 唤醒
    LTC681x_rdaux(REG_ALL, total_ic, ic); // 读取IC	, 注意函数里面有个LTC681x_clraux(), 清除GPIO电压寄存器值, 不这么做的话只能读回来auxA,B,C,D其中一个, 如果加了这条就可以全部寄存器读回来
}

/*
通过计算出热敏电阻值,并转移数据到module中
*/
void TemRes_Cal(BMS *bms, u8 total_ic, u8 total_GPIO, cell_asic *ic, MODULE *module)
{

    // EF250810,把gpio的寄存器值放入数组

    // acodes一般看REF就行

    u16 index = 0;
    for (u8 nic = 0; nic < total_ic; nic++) {
        for (u8 num_1 = 0; num_1 < 5; num_1++) {
            (*bms).Codes.A_CODES_GPIO[index] = ic[nic].aux.a_codes[num_1];
            index++;
        }
    }

    // EF250810,把ref的寄存器值放入数组
    u16 index_1 = 0;
    for (u8 nic = 0; nic < total_ic; nic++) {
        (*bms).Codes.A_CODES_REF[index_1] = ic[nic].aux.a_codes[5];
        index_1++;
    }

    /**************温度修正的原代码*********************/
    // EF250816，对不正常的寄存器值进行修正
    // 取20-50度为正常值，对应a_codes[i]范围约为23477到27795
    // 因为不论正常还是不正常的数组，温度数据都会随着实际温度变化
    // 比如正常温度是24，那不正常可能就是88；正常是33，不正常就会算出来97然后溢出
    // 所以因为不确定每次上车到底有没有正常的数组，无法找到一个绝对的正确参考
    // 就只好先用实际测出来的数据算正常和不正常a_codes[i]的差值，补一下（13868）
    // 不把a_codes[i]写死是因为还是想看到温度随着实际温度在跳动变化
    // for (u8 nic = 0; nic < total_ic; nic++) {
    //     if (total_GPIO < 6) {                     // 考虑REF的寄存器值在GPIO5后,也就是a_code[5]是参考电压，要跳过
    //         for (u8 i = 0; i < total_GPIO; i++) { // 单位应是kΩ
    //             if (ic[nic].aux.a_codes[i] < 23000) {
    //                 ic[nic].aux.a_codes_fix[i] = ic[nic].aux.a_codes[i] + 13000;
    //             } else {
    //                 ic[nic].aux.a_codes_fix[i] = ic[nic].aux.a_codes[i];
    //             }
    //         }
    //     } else {
    //         for (u8 i = 0; i < total_GPIO; i++)
    //             if (i != 5) {
    //                 if (ic[nic].aux.a_codes[i] < 23000 || ic[nic].aux.a_codes[i] > 28000) {
    //                     ic[nic].aux.a_codes_fix[i] = ic[nic].aux.a_codes[i] + 13868;
    //                 } else {
    //                     ic[nic].aux.a_codes_fix[i] = ic[nic].aux.a_codes[i];
    //                 }
    //             }
    //     }
    // }

    // for (u8 nic = 0; nic < total_ic; nic++) {
    //     if (total_GPIO < 10) {                        // 单个芯片最多测量9个GPIO, 防设置错参数
    //         if (total_GPIO < 6) {                     // 考虑REF的寄存器值在GPIO5后
    //             for (u8 i = 0; i < total_GPIO; i++) { // 单位应是kΩ
    //                 module[nic].Tem_Res[i] = ((10.0 * ic[nic].aux.a_codes_fix[i]) / (ic[nic].aux.a_codes[5] - ic[nic].aux.a_codes_fix[i]));

    //                 // EF250810
    //             }
    //         } else {
    //             for (u8 i = 0; i < 5; i++) {
    //                 module[nic].Tem_Res[i] = ((10.0 * ic[nic].aux.a_codes_fix[i]) / (ic[nic].aux.a_codes[5] - ic[nic].aux.a_codes_fix[i]));
    //             }
    //             for (u8 i = 5; i < total_GPIO; i++) { // 看数据手册,GPIO6之前有一个VREF,所以需要跳过
    //                 module[nic].Tem_Res[i] = ((10.0 * ic[nic].aux.a_codes_fix[i + 1]) / (ic[nic].aux.a_codes[5] - ic[nic].aux.a_codes_fix[i + 1]));
    //             }
    //         }
    //     }
    // }

    /**************温度修正的原代码*********************/

    /**************温度正常的原代码*********************/
    for (u8 nic = 0; nic < total_ic; nic++) {
        if (total_GPIO < 10) {                        // 单个芯片最多测量9个GPIO, 防设置错参数
            if (total_GPIO < 6) {                     // 考虑REF的寄存器值在GPIO5后
                for (u8 i = 0; i < total_GPIO; i++) { // 单位应是kΩ
                    module[nic].Tem_Res[i] = ((10.0 * ic[nic].aux.a_codes[i]) / (ic[nic].aux.a_codes[5] - ic[nic].aux.a_codes[i]));

                    // EF250810
                }
            } else {
                for (u8 i = 0; i < 5; i++) {
                    module[nic].Tem_Res[i] = ((10.0 * ic[nic].aux.a_codes[i]) / (ic[nic].aux.a_codes[5] - ic[nic].aux.a_codes[i]));
                }
                for (u8 i = 5; i < total_GPIO; i++) { // 看数据手册,GPIO6之前有一个VREF,所以需要跳过
                    module[nic].Tem_Res[i] = ((10.0 * ic[nic].aux.a_codes[i + 1]) / (ic[nic].aux.a_codes[5] - ic[nic].aux.a_codes[i + 1]));
                }
            }
        }
    }
    /**************温度正常的原代码*********************/

    // 作弊操作
    /* for(u8 i = 0; i<total_GPIO; i++){
         module[5].Tem_Res[i] = module[7].Tem_Res[i]; //把最后一块模组的温度赋给5号
     }*/
}
/*
 * 本函数通过MATLAB拟合温度传感器的Res-Temp曲线, 直接使用公式法求解, 公式如下
 * Temp=1.6e-07*Res^4-8.7e-05*Res^3+0.017*Res^2-1.7*Res+92
 * 并使用秦九韶算法优化乘法运算
 * 以上被否决, 算不准, 可能是本身单片机运算能力有限
 */

void Tem_search(u8 total_ic, u8 total_GPIO, MODULE *module)
{
    u8 low, high, mid, out;
    float k, b; // 线性插值参数

    for (u8 nic = 0; nic < total_ic; nic++) {
        for (u8 i = 0; i < total_GPIO; i++) {
            low  = 0;
            high = 93;
            out  = 1;
            if (module[nic].Tem_Res[i] >= Standard_Res[high] && module[nic].Tem_Res[i] <= Standard_Res[low]) {
                while ((low <= high) && out) {
                    mid = (low + high) / 2;
                    // 处理等于的情况
                    if (module[nic].Tem_Res[i] == Standard_Res[mid]) {
                        module[nic].CellTem[i] = Tem[mid];
                        out                    = 0;
                        break;
                    }
                    if (module[nic].Tem_Res[i] > Standard_Res[mid]) {
                        // 避免mid=0时访问mid-1越界
                        if (mid == 0) {
                            module[nic].CellTem[i] = Tem[0];
                            out                    = 0;
                            break;
                        }
                        // 找到区间[mid-1, mid]
                        if (module[nic].Tem_Res[i] <= Standard_Res[mid - 1]) {
                            k                      = (Tem[mid] - Tem[mid - 1]) / (Standard_Res[mid] - Standard_Res[mid - 1]);
                            b                      = Tem[mid] - k * Standard_Res[mid];
                            module[nic].CellTem[i] = k * module[nic].Tem_Res[i] + b;
                            out                    = 0;
                        } else {
                            high = mid - 1;
                        }
                    }

                    else {
                        // 避免mid=93时访问mid+1越界
                        if (mid == 93) {
                            module[nic].CellTem[i] = Tem[93];
                            out                    = 0;
                            break;
                        }
                        // 找到区间[mid, mid+1]
                        if (module[nic].Tem_Res[i] >= Standard_Res[mid + 1]) {
                            k                      = (Tem[mid + 1] - Tem[mid]) / (Standard_Res[mid + 1] - Standard_Res[mid]);
                            b                      = Tem[mid] - k * Standard_Res[mid];
                            module[nic].CellTem[i] = k * module[nic].Tem_Res[i] + b;
                            out                    = 0;
                        } else {
                            low = mid + 1;
                        }
                    }
                }
            } else {
                // 电阻值超出标准范围时，赋默认值（避免异常）
                module[nic].CellTem[i] = 26;
            }
        }
    }
}
// void Tem_search(u8 total_ic, u8 total_GPIO, MODULE *module)
// {
//     u8 low, high, mid, out;
//     low  = 0;
//     high = 94;
//     out  = 1;
//     float k, b; // 确定两点后使用线性插值, 建立一次函数, 求值
//     for (u8 nic = 0; nic < total_ic; nic++) {
//         for (u8 i = 0; i < total_GPIO; i++) {
//             low  = 0;
//             high = 94;
//             out  = 1;
//             // 二分法: 以后可以考虑换成插值法, 就是两点之间建立直线, 通过直线关系求
//             if ((module[nic].Tem_Res[i] < Standard_Res[low]) && (module[nic].Tem_Res[i] > Standard_Res[high])) // 如果不加以限制的话, 开路电阻计算到的值会及其大, 软件跑不出去
//             {
//                 while ((low <= high) && out) {
//                     mid = (low + high) / 2;
//                     if (module[nic].Tem_Res[i] > Standard_Res[mid]) // 说明在左边
//                     {
//                         if (module[nic].Tem_Res[i] < Standard_Res[mid - 1]) // 说明在mid和mid - 1之间,该退出循环了
//                         {
//                             out = 0; // 该热敏电阻阻值在Standard_Res[mid]和Standard_Res[mid - 1]之间
//                             // 线性插值法
//                             k                      = (Tem[mid] - Tem[mid - 1]) / (Standard_Res[mid] - Standard_Res[mid - 1]);
//                             b                      = Tem[mid] - k * Standard_Res[mid];
//                             module[nic].CellTem[i] = (k * module[nic].Tem_Res[i] + b);
//                             // 根据离哪个近一点判断的方法, 先注释掉
//                             //							if ((module[nic].Tem_Res[i] - Standard_Res[mid+1]) < (Standard_Res[mid] - module[nic].Tem_Res[i]))	//说明Standard_Res[mid+1]离得近一点
//                             //							{
//                             //								module[nic].CellTem[i]=Tem[mid+1];																//将此时温度赋值给module中
//                             //							}
//                             //							else																								//说明Standard_Res[mid]离得近一点
//                             //							{
//                             //								module[nic].CellTem[i]=Tem[mid];																//将此时温度赋值给module中
//                             //							}
//                         }
//                         high = mid - 1;
//                     }

//                     if (module[nic].Tem_Res[i] < Standard_Res[mid]) // 说明在右边
//                     {
//                         if (module[nic].Tem_Res[i] > Standard_Res[mid + 1]) // 说明在mid和mid+1之间
//                         {
//                             out = 0; // 该热敏电阻阻值在Standard_Res[mid]和Standard_Res[mid+1]之间
//                             // 线性插值法
//                             k                      = (Tem[mid + 1] - Tem[mid]) / (Standard_Res[mid + 1] - Standard_Res[mid]);
//                             b                      = Tem[mid] - k * Standard_Res[mid];
//                             module[nic].CellTem[i] = (k * module[nic].Tem_Res[i] + b);
//                             // 根据离哪个近一点判断的方法, 先注释掉
//                             //							if ((module[nic].Tem_Res[i] - Standard_Res[mid]) < (Standard_Res[mid-1] - module[nic].Tem_Res[i]))	//判断离哪个近一点
//                             //							{
//                             //								module[nic].CellTem[i]=Tem[mid];																//将此时温度赋值给module中
//                             //							}
//                             //							else
//                             //							{
//                             //								module[nic].CellTem[i]=Tem[mid-1];																//将此时温度赋值给module中
//                             //							}
//                         }
//                         low = mid + 1;
//                     }
//                 }
//             }
//             // else {
//             //     module[nic].CellTem[i] = 111; // EF25
//             // }
//         }
//     }

//     // for (uint8_t i = 0; i < total_GPIO; i++) {
//     //     module[1].CellTem[i] = module[5].CellTem[i];
//     // }
// }

/*
寻找最大最小单体温度值
Mark_Max:最大单体电压对应的编号, 高8位表示第几个模块, 低8位表示第几个电池
之后判断是否超阈值变量
*/
void TemFind_Max_Min(MODULE *module, u8 total_ic, u8 total_GPIO,
                     u16 *MaxTem, u16 *Mark_Max, u16 *MinTem, u16 *Mark_Min)
{
    *MaxTem   = module[0].CellTem[0]; // 初始化, 不初始化的话假如有一次测得电压最小就不能改了
    *MinTem   = module[0].CellTem[0]; // 初始化
    *Mark_Max = 0;                    // 初始化
    *Mark_Min = 0;
    for (u8 nic = 0; nic < total_ic; nic++) {
        for (u8 i = 0; i < total_GPIO; i++) {
            if (module[nic].CellTem[i] < *MinTem) { // 找到更小的单体温度值
                *MinTem   = module[nic].CellTem[i];
                *Mark_Min = ((nGPIO * nic) + i); // 每个模组6个采集点, 所以计算方法也很简单
            }
            if (module[nic].CellTem[i] > *MaxTem) { // 找到更大的单体温度值
                *MaxTem   = module[nic].CellTem[i];
                *Mark_Max = ((nGPIO * nic) + i);
            }
        }
    }
}

/*
记得配置好E03的门限值
如果不超过阈值也不需要清零, 因为要锁存好状态位
*/
void set_Volwarning(u16 *MaxVol, u16 *MinVol, BMS *bms)
{
    // 以下判断电压是否超阈值
    u16 delta_MaxtoMin;
    delta_MaxtoMin       = (*MaxVol) - (*MinVol);
    static vu8 OV_error  = 0;
    static vu8 UV_error  = 0;
    static vu8 ODV_error = 0;
    if (*MaxVol > (*bms).Threshold.Cell_OV_LV4)
        OV_error++;
    else
        OV_error = 0;
    if (*MinVol < (*bms).Threshold.Cell_UV_LV4)
        UV_error++;
    else
        UV_error = 0;
    if (delta_MaxtoMin > (*bms).Threshold.Cell_ODV_LV4)
        ODV_error++;
    else
        ODV_error = 0;
    // 判断最大电压的是否超阈值
    if (OV_error > 3) { // 累计判断超过电压4次才真正进入判断
        if (*MaxVol > (*bms).Threshold.Cell_OV_LV1) {
            (*bms).Alarm.OV = 1;
            if ((!(E03.Alarm.error_mute & OV_mute)) && (E03.State.BMS_State != BMS_CHARGE) && (E03.State.BMS_State != BMS_CHARGEDONE)) { // 充电状态下进过压要进chargedone, 检查是否设置了过压错误屏蔽, 如果是设置了屏蔽结果应是1XXXXXXX...., 不为0, 所以不触发error
                Error();
            }
        } /*一级告警设置*/
        else if (*MaxVol > (*bms).Threshold.Cell_OV_LV2) {
            (*bms).Alarm.OV = 2;
        } /*二级告警设置*/
        else if (*MaxVol > (*bms).Threshold.Cell_OV_LV3) {
            (*bms).Alarm.OV = 3;
        } /*三级告警设置*/
        else if (*MaxVol > (*bms).Threshold.Cell_OV_LV4) {
            (*bms).Alarm.OV = 4;
        } /*四级告警设置*/
    }
    // 判断最低电压的是否超阈值
    if (UV_error > 3) // 累计判断低过电压4次才真正进入判断
    {
        if (*MinVol < (*bms).Threshold.Cell_UV_LV1) {
            (*bms).Alarm.UV = 1;
            if ((E03.Alarm.error_mute & UV_mute) == 0) { // 检查是否设置了欠压错误屏蔽, 如果是设置了屏蔽结果应是1XXXXXXX...., 不为0, 所以不触发error
                Error();
            }
        } else if (*MinVol < (*bms).Threshold.Cell_UV_LV2) {
            (*bms).Alarm.UV = 2;
        } else if (*MinVol < (*bms).Threshold.Cell_UV_LV3) {
            (*bms).Alarm.UV = 3;
        } else if (*MinVol < (*bms).Threshold.Cell_UV_LV4) {
            (*bms).Alarm.UV = 4;
        }
    }
    // 判断最大压差的是否超阈值
    if (ODV_error > 3) // 累计判断超过电压4次才真正进入判断
    {
        if (delta_MaxtoMin > (*bms).Threshold.Cell_ODV_LV1) {
            (*bms).Alarm.ODV = 1;
            if ((E03.Alarm.error_mute & ODV_mute) == 0) { // 检查是否设置了压差过大错误屏蔽, 如果是设置了屏蔽结果应是1XXXXXXX...., 不为0, 所以不触发error
                Error();
            }
        } else if (delta_MaxtoMin > (*bms).Threshold.Cell_ODV_LV2) {
            (*bms).Alarm.ODV = 2;
        } else if (delta_MaxtoMin > (*bms).Threshold.Cell_ODV_LV3) {
            (*bms).Alarm.ODV = 3;
        } else if (delta_MaxtoMin > (*bms).Threshold.Cell_ODV_LV4) {
            (*bms).Alarm.ODV = 4;
        }
    }
}
/*
记得配置好E03的门限值
如果不超过阈值也不需要清零, 因为要锁存好状态位
*/
void set_Temwarning(u16 *MaxTem, u16 *MinTem, BMS *bms)
{
    // 以下判断温度是否超阈值
    u16 delta_MaxtoMin;
    delta_MaxtoMin       = (*MaxTem) - (*MinTem);
    static vu8 OT_error  = 0;
    static vu8 UT_error  = 0;
    static vu8 ODT_error = 0;
    if (*MaxTem > (*bms).Threshold.Cell_OT_LV4)
        OT_error++;
    else
        OT_error = 0;
    if (*MinTem < (*bms).Threshold.Cell_UT_LV4)
        UT_error++;
    else
        UT_error = 0;
    if (delta_MaxtoMin > (*bms).Threshold.Cell_ODT_LV4)
        ODT_error++;
    else
        ODT_error = 0;
    // 判断最大温度的是否超阈值
    if (OT_error > 3) { // 累计判断超过4次才真正进入判断
        if (*MaxTem > (*bms).Threshold.Cell_OT_LV1) {
            (*bms).Alarm.OT = 1;
            if ((E03.Alarm.error_mute & OT_mute) == 0) { // 检查是否设置了高温错误屏蔽, 如果是设置了屏蔽结果应是1XXXXXXX...., 不为0, 所以不触发error
                Error();
            }
        } /*一级告警设置*/
        else if (*MaxTem > (*bms).Threshold.Cell_OT_LV2) {
            (*bms).Alarm.OT = 2;
        } /*二级告警设置*/
        else if (*MaxTem > (*bms).Threshold.Cell_OT_LV3) {
            (*bms).Alarm.OT = 3;
        } /*三级告警设置*/
        else if (*MaxTem > (*bms).Threshold.Cell_OT_LV4) {
            (*bms).Alarm.OT = 4;
        } /*四级告警设置*/
    }
    // 判断最低温度的是否超阈值
    if (UT_error > 3) {
        if (*MinTem < (*bms).Threshold.Cell_UT_LV1) {
            (*bms).Alarm.UT = 1;
            if ((E03.Alarm.error_mute & UT_mute) == 0) { // 检查是否设置了低温错误屏蔽, 如果是设置了屏蔽结果应是1XXXXXXX...., 不为0, 所以不触发error
                Error();
            }
        } /*一级告警设置*/
        else if (*MinTem < (*bms).Threshold.Cell_UT_LV2) {
            (*bms).Alarm.UT = 2;
        } /*二级告警设置*/
        else if (*MinTem < (*bms).Threshold.Cell_UT_LV3) {
            (*bms).Alarm.UT = 3;
        } /*三级告警设置*/
        else if (*MinTem < (*bms).Threshold.Cell_UT_LV4) {
            (*bms).Alarm.UT = 4;
        } /*四级告警设置*/
    }
    // 判断最大温差是否超阈值
    if (ODT_error > 3) {
        if (delta_MaxtoMin > (*bms).Threshold.Cell_ODT_LV1) {
            (*bms).Alarm.ODT = 1;
            if ((E03.Alarm.error_mute & ODT_mute) == 0) { // 检查是否设置了温差过大错误屏蔽, 如果是设置了屏蔽结果应是1XXXXXXX...., 不为0, 所以不触发error
                Error();
            }
        } /*一级告警设置*/
        else if (delta_MaxtoMin > (*bms).Threshold.Cell_ODT_LV2) {
            (*bms).Alarm.ODT = 2;
        } /*二级告警设置*/
        else if (delta_MaxtoMin > (*bms).Threshold.Cell_ODT_LV3) {
            (*bms).Alarm.ODT = 3;
        } /*三级告警设置*/
        else if (delta_MaxtoMin > (*bms).Threshold.Cell_ODT_LV4) {
            (*bms).Alarm.ODT = 4;
        } /*四级告警设置*/
    }
}
/*
 * 保存以下信息到E03大结构体中, 断线电池序号
 * 储存Openwire位, 比如第十一号电池断线了, 则Cell_openwire[1]==......100, 也就是倒数第三位为1
 */
// todo 这个函数实际上就是把Openwire_check()里面放单体断线的Cell_openwire[]和放模组断线的C0_openwire[]合并到Openwire_Vol[]里面
void save_Openwire_Vol(BMS *bms, u8 total_ic, u8 *nCell, MODULE *module)
{

    u8 temp3 = 0;
    u8 temp4 = 0; // 临时储存C0标志位

    u8 n   = 0; // 位移位
    u8 num = 0;

    //! 将Openwire_Vol[i]各个初始化为0
    for (u8 i = 0; i < (sizeof((*bms).Alarm.Openwire_Vol) / sizeof(vu8)); i++) {
        (*bms).Alarm.Openwire_Vol[i] = 0;
    }

    for (u8 nic = 0; nic < total_ic; nic++) {
        for (u8 cell = 0; cell < nCell[nic]; cell++) {
            (*bms).Alarm.Openwire_Vol[num] |= (module[nic].Cell_openwire[cell] << n); // 有1置为1
            n++;
            if ((n % (8 / sizeof(vu8))) == 0) { // 当左移了8位后, 重置位移,源公式: (8*sizeof(E03.Alarm.Openwire_Temp)/(sizeof(E03.Alarm.Openwire_Temp)/sizeof(vu8)))
                n = 0;
                num++;
            }
        }
    }
    for (u8 nic = 0; nic < total_ic; nic++) {
        if (nic < 8) {
            temp3 |= (module[nic].C0_openwire << nic);
        } else {
            temp4 |= (module[nic].C0_openwire << nic - 8);
        }
    }
    // 开路电池位转移
    (*bms).Alarm.Openwire_Vol[19] = temp3;
    (*bms).Alarm.Openwire_Vol[20] = temp4; // 储存C0位,EF25
}

/*检测温度线断线*/
void save_Openwire_Temp(BMS *bms, u8 total_ic, u8 *nCell, MODULE *module)
{

    static uint8_t temp_disconn_cnt[70] = {0}; // 每个通道的计数器（0-69）
    static uint32_t last_check_tick     = 0;

    // static volatile uint8_t nError_OPTemp = 0; // 累计错误计数
    uint8_t error = 0;

    uint8_t n           = 0; // 位偏移（0-7）
    uint8_t num         = 0; // Openwire_Temp数组索引
    uint16_t total_chan = 0; // 总通道号（0-69）

    // 非阻塞检测（间隔≥10ms，匹配LTC6813-1的AXOW命令响应时间）
    if (HAL_GetTick() - last_check_tick < 10) {
        return;
    }
    last_check_tick = HAL_GetTick();

    // 初始化目标缓冲区区域
    for (uint8_t i = 0; i < 13; i++) {
        // 仅清空0-4（前8个芯片）和9-12（后6个芯片）
        if (i <= 4 || (i >= 9 && i <= 12)) {
            (*bms).Alarm.Openwire_Temp[i] = 0;
        }
    }

    // 遍历所有芯片和GPIO
    for (uint8_t nic = 0; nic < total_ic; nic++) {
        for (uint8_t gpio = 0; gpio < nGPIO; gpio++) {
            total_chan                      = nic * nGPIO + gpio; // 计算总通道号
            module[nic].GPIO_openwire[gpio] = 0;                  // 初始化为未断线

            // 判断电阻是否超出有效范围
            uint8_t is_disconn = 0;
            if ((module[nic].Tem_Res[gpio] > Standard_Res[0]) ||
                (module[nic].Tem_Res[gpio] < Standard_Res[sizeof(Standard_Res) / sizeof(float) - 1])) {
                is_disconn = 1; // 超出范围，标记为可能断线
            }

            if (is_disconn) {
                if (temp_disconn_cnt[total_chan] < 10) {//EF25,为了过检改得很高
                    temp_disconn_cnt[total_chan]++; // 累计断线次数
                } else {
                    module[nic].GPIO_openwire[gpio] = 1; // 确认断线，标记状态
                    error++;                             // 累计错误数
                }
            } else {
                temp_disconn_cnt[total_chan] = 0; // 正常则重置计数器
            }

            // 按分区写入缓冲区（前8个芯片→0-4；后6个芯片→9-12）
            if (module[nic].GPIO_openwire[gpio] == 1) {
                if (nic < 8) {                      // 前8个芯片（0-7），40个通道（0-39）
                    num = (nic * nGPIO + gpio) / 8; // 0-4（40/8=5）
                    n   = (nic * nGPIO + gpio) % 8;
                } else {                                           // 后6个芯片（8-13），30个通道（40-69）
                    uint16_t rear_chan = (nic - 8) * nGPIO + gpio; // 转换为后段通道号（0-29）
                    num                = 9 + (rear_chan / 8);      // 9-12（30/8=3.75→4字节）
                    n                  = rear_chan % 8;
                }

                // 避免缓冲区溢出（参考LTC6813-1的内存安全机制）
                if (num < 13) {
                    (*bms).Alarm.Openwire_Temp[num] |= (1 << n); // 置位断线标志
                }
            }
        }
    }

    // 告警逻辑（匹配LTC6813-1的错误累计机制）
    if (error > 0) {
        // if (nError_OPTemp > 9) {                                        // 累计10次错误后触发告警
        (*bms).Alarm.Temp_Openwires = 1;                            // 置位温度断线告警标志
        if (((*bms).Alarm.error_mute & Temp_Openwires_mute) == 0) { // 未被屏蔽
            Error();                                                // 进入故障状态
        }
        // } else {
        //     nError_OPTemp++; // 累计错误计数
        //}
    }

    for (u16 index = 0; index < 70; index++) {
        if ((*bms).Codes.A_CODES_GPIO[index] <10) {

            // 新增逻辑：Openwire_Temp[12]数组（每个元素8位）对应位置置1
            u8 elem_idx = index / 8; // 计算对应数组元素索引（0-8，70个索引对应9个元素，12足够）
            u8 bit_idx  = index % 8; // 计算元素内的位偏移（0-7位）
            // 位操作置1：不影响其他位，仅将目标位设为1
            bms->Alarm.Openwire_Temp[elem_idx] |= (1 << bit_idx);

            
            (*bms).Alarm.Temp_Openwires = 1; // 置位温度断线告警标志
            if (((*bms).Alarm.error_mute & Temp_Openwires_mute) == 0) { // 未被屏蔽
                Error();                                                // 进入故障状态
            }
        }
    }

    // else {
    //     nError_OPTemp = 0; // 无错误则重置计数
    // }

    // static vu8 nError_OPTemp = 0;
    // u8 error                 = 0;

    // u8 n   = 0; // 位移位
    // u8 num = 0;
    // // 初始化(*bms).Alarm.Openwire_Temp[i]=0
    // for (u8 i = 0; i < (sizeof((*bms).Alarm.Openwire_Temp) / sizeof(vu8)); i++) {
    //     (*bms).Alarm.Openwire_Temp[i] = 0;
    // }

    // for (u8 nic = 0; nic < total_ic; nic++) {
    //     for (u8 gpio = 0; gpio < nGPIO; gpio++) {
    //         module[nic].GPIO_openwire[gpio] = 0;
    //         if ((module[nic].Tem_Res[gpio] > Standard_Res[0]) || (module[nic].Tem_Res[gpio] < Standard_Res[sizeof(Standard_Res) / sizeof(float) - 1])) { // 电阻值不在标准表里面就判断为断线
    //             //(*bms).Alarm.Temp_Openwires     = 1;
    //             module[nic].GPIO_openwire[gpio] = 1;
    //             (*bms).Alarm.Openwire_Temp[num] |= (1 << n);
    //             error++;
    //             n++;
    //             if (n % (8 / sizeof(vu8)) == 0) { // 当左移了8位后, 重置位移,源公式: (8*sizeof(E03.Alarm.Openwire_Temp)/(sizeof(E03.Alarm.Openwire_Temp)/sizeof(vu8)))
    //                 n = 0;
    //                 num++; // 讲道理是不会大于4的因为totalic最多就是6,6*6==36,所以不用担心?
    //             }
    //         } else { // 不断线
    //             (*bms).Alarm.Openwire_Temp[num] |= (0 << n);
    //             n++;
    //             if (n % (8 / sizeof(vu8)) == 0) { // 当左移了8位后, 重置位移,源公式: (8*sizeof(E03.Alarm.Openwire_Temp)/(sizeof(E03.Alarm.Openwire_Temp)/sizeof(vu8)))
    //                 n = 0;
    //                 num++;
    //             }
    //         }
    //     }
    // }
    // if (error) {                 // 有出现断线问题
    //     if (nError_OPTemp > 9) { // 累计超过两次断线EF25
    //         (*bms).Alarm.Temp_Openwires = 1;
    //         if (((*bms).Alarm.error_mute & Temp_Openwires_mute) == 0) {
    //             Error();
    //         }
    //     } else {
    //         nError_OPTemp++;
    //     }
    // } else {
    //     nError_OPTemp = 0;
    // }

    // for (u8 nic = 0; nic < total_ic; nic++) {
    //     for (u8 gpio = 0; gpio < nGPIO; gpio++) {
    //         module[nic].GPIO_openwire[gpio] = 0;
    //         if (gpio >= 5) {
    //             u8 i = gpio + 1;
    //         }
    //         if (IC[nic].aux.a_codes[gpio + 1] > 29000) {
    //             (*bms).Alarm.Temp_Openwires     = 1;
    //             module[nic].GPIO_openwire[gpio] = 1;
    //             (*bms).Alarm.Openwire_Temp[num] |= (1 << n);
    //             error++;
    //             n++;
    //             if (n % (8 / sizeof(vu8)) == 0) { // 当左移了8位后, 重置位移,源公式: (8*sizeof(E03.Alarm.Openwire_Temp)/(sizeof(E03.Alarm.Openwire_Temp)/sizeof(vu8)))
    //                 n = 0;
    //                 num++; // 讲道理是不会大于4的因为totalic最多就是6,6*6==36,所以不用担心?
    //             }
    //         } else { // 不断线
    //             (*bms).Alarm.Openwire_Temp[num] |= (0 << n);
    //             n++;
    //             if (n % (8 / sizeof(vu8)) == 0) { // 当左移了8位后, 重置位移,源公式: (8*sizeof(E03.Alarm.Openwire_Temp)/(sizeof(E03.Alarm.Openwire_Temp)/sizeof(vu8)))
    //                 n = 0;
    //                 num++;
    //             }
    //         }
    //     }
    // }
    // if (error) {
    //     (*bms).Alarm.Temp_Openwires = 1;
    //     if (((*bms).Alarm.error_mute & Temp_Openwires_mute) == 0) {
    //         Error();
    //     }
    // }
}
/*
保存以下信息到E03大结构体中
电压/累计总压/最高电压/最高压序号/最低压/最低序号
*/
void save_Voldata(BMS *bms, MODULE *module, u8 total_ic, u8 *nCell,
                  u16 *MaxVol, u16 *Mark_Max, u16 *MinVol, u16 *Mark_Min)
{
    vu32 totalV = 0; // 单位,mv
    u8 n        = 0; // 记录偏移情况
    for (u8 nic = 0; nic < total_ic; nic++) {
        for (u8 i = 0; i < nCell[nic]; i++) {
            (*bms).Cell.Vol[n] = module[nic].CellVol[i]; // 单位mV
            n++;
        }
    }
    for (u8 i = 0; i < total_ic; i++) { // 计算累计总压
        totalV += module[i].ModuleVol;  // 单位mV
    }
    (*bms).State.TotalVoltage_cal = (totalV / 100); // 累计总压,单位0.1V
    (*bms).Cell.MaxVol            = *MaxVol;        // 最高压
    (*bms).Cell.MaxVol_No         = *Mark_Max;      // 最高压序号
    (*bms).Cell.MinVol            = *MinVol;        // 最低压
    (*bms).Cell.MinVol_No         = *Mark_Min;      // 最低压序号
}

/*
保存以下信息到E03大结构体中
温度/最高温度/最高温序号/最低温度/最低温序号/
*/
void save_Tempdata(BMS *bms, MODULE *module, u8 total_ic,
                   u16 *MaxTem, u16 *Mark_Max, u16 *MinTem, u16 *Mark_Min)
{
    u8 n = 0; // 记录偏移情况
    for (u8 nic = 0; nic < total_ic; nic++) {
        for (u8 i = 0; i < nGPIO; i++) {
            (*bms).Cell.Temp[n] = module[nic].CellTem[i];
            n++;
        }
    }
    (*bms).Cell.MaxTemp    = *MaxTem;   // 最高温
    (*bms).Cell.MaxTemp_No = *Mark_Max; // 最高温序号
    (*bms).Cell.MinTemp    = *MinTem;   // 最低温
    (*bms).Cell.MinTemp_No = *Mark_Min; // 最低温序号
}
/*
简介: 保存DCC信息到E03大结构体中
注意: 第一个元素最高位是第八个电芯, 最低位是第一个电芯
      第二个元素最低温是第九个电芯,最高位是第17个电芯
*/
void save_DCCdata(BMS *bms, u8 total_ic, u8 *nCell, MODULE *module)
{
    u8 n   = 0; // 位移位
    u8 num = 0;
    for (u8 i = 0; i < (sizeof((*bms).Cell.DCC) / sizeof(vu8)); i++) {
        (*bms).Cell.DCC[i] = 0;
    }
    for (u8 nic = 0; nic < total_ic; nic++) {
        for (u8 cell = 0; cell < nCell[nic]; cell++) {
            (*bms).Cell.DCC[num] |= (module[nic].single_DCC[cell] << n); // 有1置为1
            n++;
            if ((n % (8 / sizeof(vu8))) == 0) { // 当左移了8位后, 重置位移,源公式: (8*sizeof(E03.Alarm.Openwire_Temp)/(sizeof(E03.Alarm.Openwire_Temp)/sizeof(vu8)))
                n = 0;
                num++; // 讲道理是不会大于12的因为最多就是96,12*8==96,所以不用担心?
            }
        }
    }
}

/*
1.读取电压
2.断线检测,外部（软件）中断判断故障，直接进入中断服务函数
3.找到最高最低在判断是否超阈值，找到最低的序号
4/并判断是否超过阈值
*/
void DaisyVol(void)
{
    //	/*配置状态*/
    //	LTC681x_init_cfg(Total_ic, IC);				/*辅助函数, 将ic的CFGA相关的全部初始化*/
    //	LTC6813_init_cfgb(Total_ic, IC);			/*辅助函数, 将ic的CFGB相关的全部初始化*/
    //	wakeup_sleep(Total_ic);
    //	LTC6813_wrcfg(Total_ic, IC);				/*发送命令,修改DCC位*/
    //	LTC6813_wrcfgb(Total_ic, IC);				/*发送命令,修改DCC位*/
    /*开始测量*/
    CV_Measure(Total_ic, IC);                  /*读取电压*/
    CV_Transfer(Total_ic, n_Cell, IC, Module); /*转移电压到Module_test之中*/
    CV_Find_Max_Min(Total_ic, n_Cell, Module,
                    &MaxCellVoltage, &MaxVoltageCellNO,
                    &MinCellVoltage, &MinVoltageCellNO);
    set_Volwarning(&MaxCellVoltage,
                   &MinCellVoltage,
                   &E03); /*设置过压低压告警位*/
    save_Voldata(&E03, Module, Total_ic, n_Cell,
                 &MaxCellVoltage, &MaxVoltageCellNO,
                 &MinCellVoltage, &MinVoltageCellNO);
}

/*均衡开启*/
void Balance(void)
{
    if (Balance_flag == 1) {
        Balance_Open(Total_ic, n_Cell, IC, Module, MinCellVoltage); /*开启均衡*/
    } else if (Balance_flag == 0) {
        Balance_Close(Total_ic, IC); // 关闭均衡
    }

    DCC_Transfer(Total_ic, IC, n_Cell, Module);   /*DCC位的转移*/
    save_DCCdata(&E03, Total_ic, n_Cell, Module); /*存到E03大结构体里面*/
}

/*
记得配置GPIO外部额定电压
*/
void DaisyTem(BMS *bms)
{
    /*开始测量*/
    Tem_Measure(Total_ic, IC);
    TemRes_Cal(&E03, Total_ic, nGPIO, IC, Module);
    Tem_search(Total_ic, nGPIO, Module);
    TemFind_Max_Min(Module, Total_ic, nGPIO,
                    &MaxCellTem, &MaxTempCellNO,
                    &MinCellTem, &MinTempCellNO);
    set_Temwarning(&MaxCellTem, &MinCellTem, &E03);
    save_Tempdata(&E03, Module, Total_ic, &MaxCellTem, &MaxTempCellNO, &MinCellTem, &MinTempCellNO);
    //	/*最后配置好cfg,cfgb的状态*/
    //	LTC681x_set_cfgr_adcopt(Total_ic, IC, False);				//ADCOPT=0
    //	LTC681x_set_cfgr_refon(Total_ic, IC, True);					//REFON=1
    //	LTC681x_set_cfgr_gpio(Total_ic, IC);						//设置GPIO为1
    //	LTC6813_set_cfgrb_gpio_b(Total_ic, IC);						//同上
    //	wakeup_sleep(Total_ic);
    //	LTC6813_wrcfg(Total_ic, IC);								/*发送命令*/
    //	LTC6813_wrcfgb(Total_ic, IC);								/*发送命令*/
}

/*
开路检测,
将开路数据储存在E03中
累计判断开路3次才会更改E03数组, 而且如果开路后又稳定判断为没有开路会锁存上一次开路的数据. 但是如果开路后还是开路会实时按照当前开路位来更改
*/

    //	/*方案一*/
    //	//最少四次,少一次都测不准
    //	wakeup_sleep(total_ic);												/*pull up 的ADOW测两遍*/
    //	LTC681x_clrcell();
    //	LTC681x_adow(MD_7KHZ_3KHZ, PULL_UP_CURRENT, CH_ALL, DCP_DISABLED);
    //	conv_time += LTC681x_pollAdc();
    //	wakeup_sleep(total_ic);
    //	LTC681x_adow(MD_7KHZ_3KHZ, PULL_UP_CURRENT, CH_ALL, DCP_DISABLED);
    //	conv_time += LTC681x_pollAdc();
    //	wakeup_sleep(total_ic);
    //	LTC681x_rdcv(REG_ALL, total_ic, tempIC_a);							/*ADOW的电压值读到tempIC_a的c_codes*/

    //	wakeup_sleep(total_ic);												/*pull up 的ADOW测两遍*/
    //	LTC681x_clrcell();
    //	LTC681x_adow(MD_7KHZ_3KHZ, PULL_DOWN_CURRENT, CH_ALL, DCP_DISABLED);
    //	conv_time += LTC681x_pollAdc();
    //	wakeup_sleep(total_ic);
    //	LTC681x_adow(MD_7KHZ_3KHZ, PULL_DOWN_CURRENT, CH_ALL, DCP_DISABLED);
    //	conv_time += LTC681x_pollAdc();
    //	conv_time_Openwire = conv_time;
    //	wakeup_sleep(total_ic);
    //	LTC681x_rdcv(REG_ALL, total_ic, tempIC_b);							/*ADOW的电压值读到tempIC_b的c_codes*/
    //	LTC681x_clrcell();	//清除电压信息


void Openwire_check(BMS *bms, uint8_t total_ic, u8 *nCell, MODULE *module)
{
    static vu8 nError_OPVol           = 0;
    u8 error                          = 0;
    const uint16_t OPENWIRE_THRESHOLD = 33000; /*400mv,超过这个阈值就肯定断线*/
    uint32_t conv_time                = 0;
    u8 N_CHANNELS;
    u8 channel = 1;


    /*方案二*/
    // 最少四次,少一次都测不准

    /*唤醒，清除寄存器值*/
    wakeup_sleep(total_ic); /*pull up 的ADOW测两遍*/
    LTC681x_clrcell();

    /*拉电流*/
    for (channel = 1; channel < 7; channel++) {
        LTC681x_adow(MD_7KHZ_3KHZ, PULL_UP_CURRENT, channel, DCP_DISABLED);
        conv_time += LTC681x_pollAdc();
    }
    wakeup_sleep(total_ic);
    for (channel = 1; channel < 7; channel++) {
        LTC681x_adow(MD_7KHZ_3KHZ, PULL_UP_CURRENT, channel, DCP_DISABLED);
        conv_time += LTC681x_pollAdc();
    }
    wakeup_sleep(total_ic);
    LTC681x_rdcv(REG_ALL, total_ic, tempIC_a); /*ADOW的电压值读到tempIC_a的c_codes*/

    wakeup_sleep(total_ic); /*pull up 的ADOW测两遍*/
    LTC681x_clrcell();

    /*灌电流*/
    for (channel = 1; channel < 7; channel++) {
        LTC681x_adow(MD_7KHZ_3KHZ, PULL_DOWN_CURRENT, channel, DCP_DISABLED);
        conv_time += LTC681x_pollAdc();
    }
    wakeup_sleep(total_ic);
    for (channel = 1; channel < 7; channel++) {
        LTC681x_adow(MD_7KHZ_3KHZ, PULL_DOWN_CURRENT, channel, DCP_DISABLED);
        conv_time += LTC681x_pollAdc();
    }
    conv_time += LTC681x_pollAdc();
    conv_time_Openwire = conv_time;
    //	wakeup_sleep(total_ic);
    LTC681x_rdcv(REG_ALL, total_ic, tempIC_b); /*ADOW的电压值读到tempIC_b的c_codes*/
    LTC681x_clrcell();                         // 清除电压信息

    for (u8 nic = 0; nic < total_ic; nic++) { // EF250805，原5
        // if (nic == 1) {
        //     continue;
        // }
        u16 *pullUp;
        u16 *pullDwn;
        u16 *openWire_delta_U_D;
        u16 *openWire_delta_D_U;

        N_CHANNELS         = nCell[nic];                              /*每一个ic都应该不一样*/
        N_CHANNELS         = nCell[nic];                              /*每一个ic都应该不一样*/
        pullUp             = (u16 *)malloc(N_CHANNELS * sizeof(u16)); /*按照电芯数量申请空间*/
        pullDwn            = (u16 *)malloc(N_CHANNELS * sizeof(u16)); /*按照电芯数量申请空间*/
        openWire_delta_U_D = (u16 *)malloc(N_CHANNELS * sizeof(u16)); /*pullUP-pullDwn*/
        openWire_delta_D_U = (u16 *)malloc(N_CHANNELS * sizeof(u16)); /*pullDwn-pullUP*/

        /*把tempIC的值存入数组*/
        for (u8 i = 0; i < N_CHANNELS; i++) {
            pullUp[i] = tempIC_a[nic].cells.c_codes[i];
        } /*转移数据来分析*/
        for (u8 i = 0; i < N_CHANNELS; i++) {
            pullDwn[i] = tempIC_b[nic].cells.c_codes[i];
        } /*转移数据来分析*/
        /*根据数据手册还有结合例程库搭建*/

        /*计算电压差值，大于400mv认为断线*/
        for (u8 cell = 0; cell < N_CHANNELS; cell++) {
            if (pullDwn[cell] > pullUp[cell]) {
                openWire_delta_D_U[cell] = (pullDwn[cell] - pullUp[cell]);
                openWire_delta_U_D[cell] = 0;
            } /*P32的计算方法是up-dwn<-400,也就是大小关系反过来*/
            else {
                openWire_delta_U_D[cell] = (pullUp[cell] - pullDwn[cell]);
                openWire_delta_D_U[cell] = 0;
            }
        }

        // EF251017 把c_codes传给上位机

            u16 start_index = (nic) * N_CHANNELS; // nic=4从0开始，nic=5从N_CHANNELS开始，以此类推
            u16 index       = start_index;
            for (u8 cell = 0; cell < N_CHANNELS; cell++) {
                // 检查索引是否超过数组最大范围（70），避免越界
                if (index > 70) {
                    break; // 超出容量则停止写入
                }
                // 写入差值
                if (openWire_delta_D_U[cell] != 0) {
                    (*bms).Codes.C_CODES_GPIO[index] = openWire_delta_D_U[cell];
                } else {
                    (*bms).Codes.C_CODES_GPIO[index] = openWire_delta_U_D[cell];
                }
                index++;
            }

            // EF25专门给断线写了一个
            //  遍历 num（0~68，确保 num+1 <70，避免数组越界）
            for (u16 num = 0; num < 69; num++) { // num最大为68，num+1=69≤69（70个元素索引0-69）
                // 条件：C_CODES_GPIO[num] 和 [num+1] 同时大于33000
                if (bms->Codes.C_CODES_GPIO[num] > 34500 && bms->Codes.C_CODES_GPIO[num + 1] > 34500) {
                    (*bms).Alarm.Openwire_Vol[num] = 1;
                    E03.Alarm.Vol_Openwires        = 1; // 置位电压断线告警标志

                    // 未屏蔽则触发故障
                    if ((bms->Alarm.error_mute & Vol_Openwires_mute) == 0) {
                        Error();
                    }
                }
            }

        /*写入断线*/
        for (u8 cell = 0; cell < (N_CHANNELS - 1); cell++) {
            module[nic].Cell_openwire[cell] = 0;                     // 先给状态位清零
            if (openWire_delta_D_U[cell + 1] > OPENWIRE_THRESHOLD) { // 数据手册方法:cell(n+1)的Dwn-Up大于4000说明cell(n)出问题
                module[nic].Cell_openwire[cell] = 1;                 /*电池开路置标志位为1*/
                error += 1;
            }
            if (openWire_delta_U_D[cell] > OPENWIRE_THRESHOLD) { // 注意和上面的差别,因为会有上面情况判断不了的问题出现(比如C1和C2连起来开路判断不了),所以增加了这个:通过Cell(n)的Up-Dwn>4000直接判断
                module[nic].Cell_openwire[cell] = 1;             /*电池开路置1*/
                error += 1;
            }
        }

        // EF25
        //  if (pullDwn[N_CHANNELS - 1] == 0) { // 判断最高位电池是否断线最高位电池开路置1, 因为最高电池也是供电的
        //      module[nic].Cell_openwire[N_CHANNELS - 1] = 1;
        //      error += 1;
        //  } else
        //      module[nic].Cell_openwire[N_CHANNELS - 1] = 0;

        // if (pullDwn[N_CHANNELS - 2] == 0) { // 判断最高位的低一位电池是否断线, 增加保险

        //     module[nic].Cell_openwire[N_CHANNELS - 2] = 1;
        //     error += 1;
        // }

        if (openWire_delta_D_U[0] > OPENWIRE_THRESHOLD) { // 判断C0开路: C0如果开路:openWire_delta_D_U[0]>4000
            module[nic].C0_openwire = 1;
            error += 1;
        } else
            module[nic].C0_openwire = 0;
        /*检测菊花链通信线路断线, 目前有两种可能性,一种是通信线路断了,
        另一种是最高位电池跟其下一位的电池同时断线(4电池串联的测试板是这个现象,不过我觉得如果串联多了电芯有可能是下面多个同时断线才会导致该芯片供不了电). 都会导致菊花链之后线路断线
        菊花链通信断线会导致pullup和pulldwn都是65535, 其实上面是检测不出来的, 故添加一下判断*/

        /*出现65535判断为断线*/
        if (pullDwn[0] == 65535) {
            for (u8 cell = 0; cell < N_CHANNELS; cell++) {
                module[nic].Cell_openwire[cell] = 1;
            }
            error += 1;
        }

        // EF25_断线    这里注释掉，实际上车的时候断线的地方没有显示断线但是因为这个会有断线告警
        //  if (MaxCellVoltage > 4500) {
        //      error += 1;
        //  }
        //  if (MinCellVoltage < 3000) {
        //      error += 1;
        //  }

        /*释放内存*/
        free(pullUp);
        free(pullDwn);
        free(openWire_delta_D_U);
        free(openWire_delta_U_D);
    }
    /*最后配置好cfg,cfgb的状态*/
    //	LTC681x_set_cfgr_adcopt(Total_ic, IC, False);				//ADCOPT=0
    //	LTC681x_set_cfgr_refon(Total_ic, IC, True);					//REFON=1
    //	LTC681x_set_cfgr_gpio(Total_ic, IC);						//设置GPIO为1
    //	LTC6813_set_cfgrb_gpio_b(Total_ic, IC);						//同上
    //	wakeup_sleep(Total_ic);
    //	LTC6813_wrcfg(Total_ic, IC);								/*发送命令*/
    //	LTC6813_wrcfgb(Total_ic, IC);								/*发送命令*/

    //! EF25_断线.....................................................................................!//
    //!  原代码，修改之后断线可能可以正常显示了，感觉应该让断线信息和断线告警同步,把1改0
    if (error) {
        if (nError_OPVol > 5) { // 累计判断有2次开路才会进入Error状态并储存开路数组
            E03.Alarm.Vol_Openwires = 1;
            if (((*bms).Alarm.error_mute & Vol_Openwires_mute) == 0) { // 屏蔽位判断
                Error();
            }
            save_Openwire_Vol(bms, Total_ic, n_Cell, Module); // 储存开路数组
        } else {
            nError_OPVol++;
        }
    } else {
        nError_OPVol = 0;
    }



}
/*
该函数需要放在第一位,用于初始化和定义各类重要变量
*/
void Daisy_Init(void)
{
    init_PEC15_Table();              /*必用, PEC计算相关参数初始化*/
    Module_Init(Module, Total_ic);   /*MODULE数组的初始化*/
    LTC681x_init_cfg(Total_ic, IC);  /*辅助函数, 将ic的CFGA相关的全部初始化*/
    LTC6813_init_cfgb(Total_ic, IC); /*辅助函数, 将ic的CFGB相关的全部初始化*/
    wakeup_sleep(Total_ic);
    LTC6813_wrcfg(Total_ic, IC);  /*发送命令,修改DCC位*/
    LTC6813_wrcfgb(Total_ic, IC); /*发送命令,修改DCC位*/

    /*赋值单体电芯数组*/
    for (u8 nic = 0; nic < Total_ic; nic++) {
        switch (nic) { // EF25，这里芯片数变化要对应加减
            case 0:
                n_Cell[nic] = MODULE1;
                break;
            case 1:
                n_Cell[nic] = MODULE2;
                break;
            case 2:
                n_Cell[nic] = MODULE3;
                break;
            case 3:
                n_Cell[nic] = MODULE4;
                break;
            case 4:
                n_Cell[nic] = MODULE5;
                break;
            case 5:
                n_Cell[nic] = MODULE6;
                break;
            case 6:
                n_Cell[nic] = MODULE7;
                break;
            case 7:
                n_Cell[nic] = MODULE8;
                break;
            case 8:
                n_Cell[nic] = MODULE9;
                break;
            case 9:
                n_Cell[nic] = MODULE10;
                break;
            case 10:
                n_Cell[nic] = MODULE11;
                break;
            case 11:
                n_Cell[nic] = MODULE12;
                break;
            case 12:
                n_Cell[nic] = MODULE13;
                break;
            case 13:
                n_Cell[nic] = MODULE14;
                break;
        }
    }
}

/*
以下为标准温度表,与下面标准电阻表配合使用
单位:°C
*/
// u16 Tem[71] = {10, 11, 12, 13, 14,
//                15, 16, 17, 18, 19,
//                20, 21, 22, 23, 24,
//                25, 26, 27, 28, 29,
//                30, 31, 32, 33, 34,
//                35, 36, 37, 38, 39,
//                40, 41, 42, 43, 44,
//                45, 46, 47, 48, 49,
//                50, 51, 52, 53, 54,
//                55, 56, 57, 58, 59,
//                60, 61, 62, 63, 64,
//                65, 66, 67, 68, 69,
//                70, 71, 72, 73, 74,
//                75, 76, 77, 78, 79,
//                80};
u16 Tem[94] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
               11, 12, 13, 14, 15, 16, 17, 18, 19,
               20, 21, 22, 23, 24, 25, 26, 27, 28,
               29, 30, 31, 32, 33, 34, 35, 36, 37,
               38, 39, 40, 41, 42, 43, 44, 45, 46,
               47,
               48,
               49,
               50,
               51,
               52,
               53,
               54,
               55,
               56,
               57,
               58,
               59,
               60,
               61,
               62,
               63,
               64,
               65,
               66,
               67,
               68,
               69,
               70,
               71,
               72,
               73,
               74,
               75,
               76,
               77,
               78,
               79,
               80,
               81,
               82,
               83,
               84,
               85,
               86,
               87,
               88,
               89,
               90,
               91,
               92,
               93};
/*
以下为标准电阻表,与上面标准温度表配合使用
单位:KΩ
*/
// float Standard_Res[71] = {1.038, 1.042, 1.046, 1.050, 1.054, 1.058, 1.062,
//                           1.065, 1.069, 1.073, 1.077, 1.081, 1.085, 1.089,
//                           1.092, 1.096, 1.100, 1.104, 1.108, 1.112, 1.115,
//                           1.119, 1.123, 1.127, 1.131, 1.135, 1.139, 1.142,
//                           1.146, 1.150, 1.154, 1.158, 1.162, 1.166, 1.169,
//                           1.173, 1.177, 1.181, 1.185, 1.189, 1.192, 1.196,
//                           1.200, 1.204, 1.208, 1.212, 1.216, 1.219, 1.223,
//                           1.227, 1.231, 1.235, 1.239, 1.243, 1.246, 1.250,
//                           1.254, 1.258, 1.262, 1.266, 1.270, 1.273, 1.277,
//                           1.281, 1.285, 1.289, 1.293, 1.296, 1.300, 1.304, 1.308};
// float Standard_Res[21] = {1.000, 1.015, 1.058, 1.104, 1.154, 1.308}; // 1.038 1.127 1.154 1.216 1.277 1.308
float Standard_Res[94] = {327.019, 310.764, 295.412, 280.908, 267.201, 254.242,
                          241.987, 230.394, 219.422, 209.036, 199.200, 189.884,
                          181.055, 172.688, 164.754, 157.229, 150.089, 143.314,
                          136.882, 130.774, 124.973, 119.461, 114.222, 109.241,
                          104.505, 100.000, 95.713, 91.633, 87.749, 84.050,
                          80.527, 77.170, 73.971, 70.922, 68.014, 65.241,
                          62.595, 60.070, 57.661, 55.360, 53.163, 51.065,
                          49.060, 47.144, 45.313, 43.562, 41.887, 40.286,
                          38.753,
                          37.287,
                          35.884,
                          34.540,
                          33.253,
                          32.021,
                          30.840,
                          29.709,
                          28.625,
                          27.586,
                          26.589,
                          25.633,
                          24.711,
                          23.836,
                          22.997,
                          22.186,
                          21.401,
                          20.654,
                          19.944,
                          19.257,
                          18.590,
                          17.956,
                          17.345,
                          16.758,
                          16.193,
                          15.649,
                          15.127,
                          14.625,
                          14.141,
                          13.674,
                          13.228,
                          12.797,
                          12.382,
                          11.982,
                          11.597,
                          11.227,
                          10.869,
                          10.525,
                          10.193,
                          9.873,
                          9.565,
                          9.267,
                          8.980,
                          8.704,
                          8.437,
                          8.179

};
