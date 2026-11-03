#ifndef SC8815_H
#define SC8815_H

#include <Arduino.h>
#include <Wire.h>

// =============================================================================
// SC8815 高效同步双向 Buck-Boost 充电控制芯片 (I2C 接口) 驱动库
// 参考资料: SC8815 Datasheet (SOUTHCHIP SEMICONDUCTOR)
//   - 支持 1~6 节锂电池充电管理: 涓流 / 恒流(CC) / 恒压(CV) / 截止(EOC) / 回充;
//   - 支持反向放电 (OTG): 电池经升/降压输出至 VBUS, 输出范围 3V~36V;
//   - VBUS / VBAT 工作范围: 2.7V ~ 36V (瞬时耐压 40V);
//   - 集成 10-bit ADC: 实时回读 VBUS/VBAT 电压、IBUS/IBAT 电流及 ADIN 电压;
//   - 保护: VBUS/VBAT 过压、VBUS 短路折返、过温 (165°C, 迟滞 15°C) 等;
//   - 功率路径管理: PGATE (外接 PMOS 栅极驱动) / GPO 开漏输出。
// =============================================================================

// I2C 从机地址 (SC8815 地址固定, 芯片无 ADDR 地址选择引脚)
// 手册 8.12.1 节: 7 位地址 0x74; 8 位地址: 写 0xE8 / 读 0xE9
#define SC8815_I2C_ADDR 0x74

// =============================================================================
// 寄存器地址定义 (依据手册第 10 章 "Register Map")
// 标注 [PSTOP] 的配置项, 手册要求 "Set this bit during PSTOP pin is high",
// 即需在 PSTOP 拉高(待机)期间写入, 运行中动态修改可能不生效。
// =============================================================================
#define SCREG_VBAT_SET 0x00       // [PSTOP] 电池配置: IRCOMP / VBAT_SEL / CSEL / VCELL_SET
#define SCREG_VBUSREF_I_SET 0x01  // 内部FB模式 VBUS 参考电压设定 (高 8 位)
#define SCREG_VBUSREF_I_SET2 0x02 // 内部FB模式 VBUS 参考电压设定 (低 2 位, 位于 bit7:6)
#define SCREG_VBUSREF_E_SET 0x03  // 外部FB模式 VBUS 参考电压设定 (高 8 位)
#define SCREG_VBUSREF_E_SET2 0x04 // 外部FB模式 VBUS 参考电压设定 (低 2 位, 位于 bit7:6)
#define SCREG_IBUS_LIM_SET 0x05   // IBUS 限流门限 (充电/放电模式均有效)
#define SCREG_IBAT_LIM_SET 0x06   // IBAT 限流门限 (充电/放电模式均有效)
#define SCREG_VINREG_SET 0x07     // VINREG 动态输入电压门限 (充电模式)
#define SCREG_RATIO 0x08          // [PSTOP] 采样/监测比例: IBAT_RATIO / IBUS_RATIO / VBAT_MON_RATIO / VBUS_RATIO
#define SCREG_CTRL0_SET 0x09      // EN_OTG / VINREG_RATIO / FREQ_SET / DT_SET
#define SCREG_CTRL1_SET 0x0A      // ICHAR_SEL / DIS_TRICKLE / DIS_TERM / FB_SEL / TRICKLE_SET / DIS_OVP
#define SCREG_CTRL2_SET 0x0B      // FACTORY / EN_DITHER / SLEW_SET
#define SCREG_CTRL3_SET 0x0C      // EN_PGATE / GPO_CTRL / AD_START / ILIM_BW / LOOP / SFB / EOC / PFM
#define SCREG_VBUS_FB_VALUE 0x0D  // R: ADC 回读 VBUS 电压高 8 位
#define SCREG_VBUS_FB_VALUE2 0x0E // R: ADC 回读 VBUS 电压低 2 位 (位于 bit7:6)
#define SCREG_VBAT_FB_VALUE 0x0F  // R: ADC 回读 VBAT 电压高 8 位
#define SCREG_VBAT_FB_VALUE2 0x10 // R: ADC 回读 VBAT 电压低 2 位 (位于 bit7:6)
#define SCREG_IBUS_VALUE 0x11     // R: ADC 回读 IBUS 电流高 8 位
#define SCREG_IBUS_VALUE2 0x12    // R: ADC 回读 IBUS 电流低 2 位 (位于 bit7:6)
#define SCREG_IBAT_VALUE 0x13     // R: ADC 回读 IBAT 电流高 8 位
#define SCREG_IBAT_VALUE2 0x14    // R: ADC 回读 IBAT 电流低 2 位 (位于 bit7:6)
#define SCREG_ADIN_VALUE 0x15     // R: ADC 回读 ADIN 引脚电压高 8 位
#define SCREG_ADIN_VALUE2 0x16    // R: ADC 回读 ADIN 引脚电压低 2 位 (位于 bit7:6)
#define SCREG_STATUS 0x17         // R: 实时状态: AC_OK / INDET / VBUS_SHORT / OTP / EOC
#define SCREG_MASK 0x19           // R/W: 中断屏蔽: 对应位置 1 屏蔽 INT 引脚输出

// =============================================================================
// 枚举配置选项定义 (位域布局与手册寄存器表完全一致)
// =============================================================================

// 0x00 VBAT_SET[7:6] 电池内阻补偿 (Battery Impedance Compensation)
// 手册 8.1.7 节: VBAT_cmp = VBAT_set + min(IBAT × IRCOMP, VCLAMP)
// 其中 VCLAMP 固定为 125mV 最大补偿值。
// 警告: 若设置的 IRCOMP 大于电池实际内阻, 将导致过充, 请结合电池参数谨慎评估!
enum SCBAT_IRCOMP
{
    SCBAT_IRCOMP_0mR = 0x00,  // 00: 0 mΩ (复位默认)
    SCBAT_IRCOMP_20mR = 0x40, // 01: 20 mΩ
    SCBAT_IRCOMP_40mR = 0x80, // 10: 40 mΩ
    SCBAT_IRCOMP_80mR = 0xC0  // 11: 80 mΩ
};

// 0x00 VBAT_SET[5] 电池电压设定方式选择
// 手册 8.1.3 节:
//   内部设定: VBATS 引脚接 VBAT, 目标电压由 CSEL + VCELL_SET 决定;
//   外部设定: VBATS 引脚接电阻分压, VBAT = 1.203V × (1 + Rup/Rdown), 参考基准 1.2V
enum SCBAT_VBAT_SEL
{
    SCBAT_VBAT_SEL_Internal = 0x00, // 0: 内部设定 (复位默认), VBATS 引脚接 VBAT 端
    SCBAT_VBAT_SEL_External = 0x20  // 1: 外部设定, VBATS 引脚接电阻分压网络
};

// 0x00 VBAT_SET[4:3] 电池串数 (仅内部设定方式有效)
// 注意: 手册该位仅支持 1S~4S 内部设定, 更多节数需使用外部设定方式
enum SCBAT_CSEL
{
    SCBAT_CSEL_1S = 0x00, // 00: 1 节 (复位默认)
    SCBAT_CSEL_2S = 0x08, // 01: 2 节串联
    SCBAT_CSEL_3S = 0x10, // 10: 3 节串联
    SCBAT_CSEL_4S = 0x18  // 11: 4 节串联
};

// 0x00 VBAT_SET[2:0] 单节电池充电截止电压 (仅内部设定方式有效)
// 手册 Table 1: 000=4.1V, 001=4.2V(默认), 010=4.25V, 011=4.3V,
//               100=4.35V, 101=4.4V, 110=4.45V, 111=4.5V
enum SCBAT_VCELL
{
    SCBAT_VCELL_4v1 = 0x00,  // 000: 4.10V
    SCBAT_VCELL_4v2 = 0x01,  // 001: 4.20V (复位默认)
    SCBAT_VCELL_4v25 = 0x02, // 010: 4.25V
    SCBAT_VCELL_4v3 = 0x03,  // 011: 4.30V
    SCBAT_VCELL_4v35 = 0x04, // 100: 4.35V
    SCBAT_VCELL_4v4 = 0x05,  // 101: 4.40V
    SCBAT_VCELL_4v45 = 0x06, // 110: 4.45V
    SCBAT_VCELL_4v5 = 0x07   // 111: 4.50V
};

// 0x08 RATIO[4] IBAT 限流设置倍率 (同时决定 IBAT ADC 换算倍率)
// 手册 Table 6: IBAT_LIM(A) = (IBAT_LIM_SET + 1)/256 × IBAT_RATIO × 10mΩ/RS2
enum SCHWI_IBAT_RATIO
{
    SCHWI_IBAT_RATIO_6x = 0x00, // 0: 6x
    SCHWI_IBAT_RATIO_12x = 0x10 // 1: 12x (复位默认)
};

// 0x08 RATIO[3:2] IBUS 限流设置倍率 (同时决定 IBUS ADC 换算倍率)
// 手册 Table 5: IBUS_LIM(A) = (IBUS_LIM_SET + 1)/256 × IBUS_RATIO × 10mΩ/RS1
// 注意: 手册规定 00 与 11 为禁止值, 合法值为 01(6x) / 10(3x)
enum SCHWI_IBUS_RATIO
{
    SCHWI_IBUS_RATIO_6x = 0x04, // 01: 6x
    SCHWI_IBUS_RATIO_3x = 0x08  // 10: 3x (复位默认)
};

// 0x08 RATIO[1] VBAT ADC 监测比例
// 手册 Table 8: VBAT = (4×VBAT_FB_VALUE + VBAT_FB_VALUE2 + 1) × VBAT_MON_RATIO × 2mV
// 1S/2S 电池应用 (VBAT < 9V) 应设为 5x
enum SCHWI_VBAT_RATIO
{
    SCHWI_VBAT_RATIO_12_5x = 0x00, // 0: 12.5x (复位默认)
    SCHWI_VBAT_RATIO_5x = 0x02     // 1: 5x
};

// 0x08 RATIO[0] VBUS 电压设定与 ADC 监测比例 (设定与回读共用)
// 手册 8.2 节: VBUS < 10.24V 建议 5x (步进 10mV); VBUS > 10.24V 建议 12.5x (步进 25mV)
enum SCHWI_VBUS_RATIO
{
    SCHWI_VBUS_RATIO_12_5x = 0x00, // 0: 12.5x (复位默认)
    SCHWI_VBUS_RATIO_5x = 0x01     // 1: 5x
};

// 0x09 CTRL0[4] VINREG 门限倍率 (VBUS < 12V 时建议设为 40x)
// 手册 Table 7: VINREG = (VINREG_SET + 1) × VINREG_RATIO (mV)
enum SCHWI_VINREG_RATIO
{
    SCHWI_VINREG_RATIO_100x = 0x00, // 0: 100x (复位默认, 最大可设 25.6V)
    SCHWI_VINREG_RATIO_40x = 0x10   // 1: 40x (最大可设 10.24V)
};

// 0x09 CTRL0[3:2] 开关频率 [PSTOP]
enum SCHWI_FREQ
{
    SCHWI_FREQ_150KHz = 0x00,   // 00: 150kHz
    SCHWI_FREQ_300KHz_1 = 0x04, // 01: 300kHz (复位默认)
    SCHWI_FREQ_300KHz_2 = 0x08, // 10: 300kHz
    SCHWI_FREQ_450KHz = 0x0C    // 11: 450kHz
};

// 0x09 CTRL0[1:0] 开关死区时间 [PSTOP]
// 手册 9.4 节: 若 MOS 栅极串阻 > 10Ω 导致开关变慢, 需相应增大死区避免直通
enum SCHWI_DT
{
    SCHWI_DT_20ns = 0x00, // 00: 20ns (复位默认)
    SCHWI_DT_40ns = 0x01, // 01: 40ns
    SCHWI_DT_60ns = 0x02, // 10: 60ns
    SCHWI_DT_80ns = 0x03  // 11: 80ns
};

// 0x0A CTRL1[7] 充电电流基准选择 [PSTOP]
// 决定涓流充电电流与 EOC 截止电流以 IBUS (输入侧) 还是 IBAT (电池侧) 为基准
enum SCHWI_ICHAR
{
    SCHWI_ICHAR_IBUS = 0x00, // 0: 以 IBUS 为充电电流基准 (复位默认)
    SCHWI_ICHAR_IBAT = 0x80  // 1: 以 IBAT 为充电电流基准
};

// 0x0A CTRL1[6] 涓流充电使能 [PSTOP]
// 涓流阶段电流: ICHAR=IBUS 时降为 IBUS 限流的 22%; ICHAR=IBAT 时降为 IBAT 限流的 10%
enum SCHWI_TRICKLE
{
    SCHWI_TRICKLE_Enable = 0x00, // 0: 使能涓流充电 (复位默认)
    SCHWI_TRICKLE_Disable = 0x40 // 1: 关闭涓流充电
};

// 0x0A CTRL1[5] 充电自动截止控制 [PSTOP]
// 手册 8.1.4 节: EOC 条件同时满足 (电压 > 98% 目标值 且 电流 < 门限) 后停止充电;
// 若禁用截止, 则维持恒压充电
enum SCHWI_TERM
{
    SCHWI_TERM_Enable = 0x00, // 0: EOC 后自动终止充电 (复位默认)
    SCHWI_TERM_Disable = 0x20 // 1: 关闭自动终止, EOC 后继续恒压维持
};

// 0x0A CTRL1[4] VBUS 反馈方式 (仅放电/OTG 模式有效) [PSTOP]
// 手册 8.2 节:
//   内部: VBUS = VBUSREF_I × VBUS_RATIO, 参考电压可经 I2C 动态改变;
//   外部: VBUS = VBUSREF_E × (1 + Rup/Rdown), FB 引脚接电阻分压网络
enum SCHWI_FB
{
    SCHWI_FB_Internal = 0x00, // 0: 内部设定 (VBUSREF_I + VBUS_RATIO) (复位默认)
    SCHWI_FB_External = 0x10  // 1: 外部设定 (FB 引脚电阻分压)
};

// 0x0A CTRL1[3] 涓流充电门限 (以 VBAT 目标电压的百分比计) [PSTOP]
// 手册 8.1.1 节: 可设为 60% 或 70% 目标电压
enum SCHWI_TRICKLE_SET
{
    SCHWI_TRICKLE_SET_70 = 0x00, // 0: 70% VBAT 目标电压 (复位默认)
    SCHWI_TRICKLE_SET_60 = 0x08  // 1: 60% VBAT 目标电压
};

// 0x0A CTRL1[2] 放电模式过压保护 (VBUS 超过目标值 10% 时停止开关)
enum SCHWI_OVP
{
    SCHWI_OVP_Enable = 0x00, // 0: 使能 OVP (复位默认)
    SCHWI_OVP_Disable = 0x04 // 1: 关闭 OVP
};

// 0x0B CTRL2[2] 开关频率抖动使能 [PSTOP]
// 手册 8.7 节: 使能后频率在 ±5% 范围渐变以改善 EMC;
// 此时 PGATE/DITHER 引脚仅用作抖动电容 (典型 100nF), 栅极驱动功能失效
// T_dither = 120mV × C / 10μA
enum SCHWI_DITHER
{
    SCHWI_DITHER_Disable = 0x00, // 0: 关闭抖动, PGATE 用作 PMOS 栅极控制 (复位默认)
    SCHWI_DITHER_Enable = 0x04   // 1: 使能 ±5% 频率抖动
};

// 0x0B CTRL2[1:0] VBUS 动态调节斜率 [PSTOP]
// 手册 8.2.2 节: 参考电压变化率, 与 2mV DAC 步进配合使用
enum SCHWI_SLEW
{
    SCHWI_SLEW_1mV_us = 0x00, // 00: 1mV/μs
    SCHWI_SLEW_2mV_us = 0x01, // 01: 2mV/μs (复位默认)
    SCHWI_SLEW_4mV_us = 0x02, // 10: 4mV/μs
    SCHWI_SLEW_8mV_us = 0x03  // 11: 8mV/μs
};

// 0x0C CTRL3[5] ADC 转换使能
// 手册 8.3 节: 使能后可回读 VBUS/VBAT 电压与 IBUS/IBAT 电流;
// 待机模式下开启 ADC 会增加 0.5~1mA 静态电流
enum SCHWI_ADC
{
    SCHWI_ADC_Disable = 0x00, // 0: 停止 ADC 转换 (复位默认)
    SCHWI_ADC_Enable = 0x20   // 1: 启动 ADC 转换
};

// 0x0C CTRL3[4] 限流环带宽 [PSTOP]
enum SCHWI_ILIM_BW
{
    SCHWI_ILIM_BW_5KHz = 0x00,   // 0: 5kHz (复位默认, 环路响应快)
    SCHWI_ILIM_BW_1_25KHz = 0x10 // 1: 1.25kHz (带宽低, 抗采样噪声能力更强)
};

// 0x0C CTRL3[3] 环路响应 [PSTOP]
enum SCHWI_LOOP
{
    SCHWI_LOOP_Normal = 0x00, // 0: 常规环路响应 (复位默认)
    SCHWI_LOOP_Fast = 0x08    // 1: 增强环路响应
};

// 0x0C CTRL3[2] VBUS 短路时 IBUS/IBAT 限流折返 (仅放电模式, 保护芯片)
// 手册 8.11.3 节: VBUS < 1V 时, IBUS 折返至设定值的 22%, IBAT 折返至 10%
// 带载启动场景: 需先设为 Disable 关闭折返, 启动完成后再恢复 Enable 以获得短路保护
enum SCHWI_SFB
{
    SCHWI_SFB_Enable = 0x00, // 0: 使能折返保护 (复位默认)
    SCHWI_SFB_Disable = 0x04 // 1: 关闭折返 (带载启动时使用)
};

// 0x0C CTRL3[1] EOC 截止电流门限 [PSTOP]
// 手册 8.1.4 节: 截止电流 = 充电限流值的 1/25 或 1/10
enum SCHWI_EOC
{
    SCHWI_EOC_1_25 = 0x00, // 0: 充电电流降至限流值的 1/25
    SCHWI_EOC_1_10 = 0x02  // 1: 充电电流降至限流值的 1/10 (复位默认)
};

// 0x0C CTRL3[0] 轻载 PFM 模式 (仅放电/OTG 模式)
// 手册 8.2.3 节: PFM 可提高轻载效率, 代价是输出纹波略大
enum SCHWI_PFM
{
    SCHWI_PFM_Disable = 0x00, // 0: 关闭 PFM, 全程 PWM (纹波更小) (复位默认)
    SCHWI_PFM_Enable = 0x01   // 1: 使能 PFM (轻载效率更高)
};

// 电池参数配置结构体 (对应 0x00 VBAT_SET 寄存器)
// 注意: 手册要求该寄存器在 PSTOP 拉高 (待机) 期间写入
struct BatteryConfig
{
    SCBAT_IRCOMP IRCOMP;     // 电池内阻补偿 (0~80mΩ)
    SCBAT_VBAT_SEL VBAT_SEL; // 电池电压设定方式 (内部 / 外部)
    SCBAT_CSEL CSEL;         // 电池串数 (1S~4S, 内部设定时有效)
    SCBAT_VCELL VCELL;       // 单节充电截止电压 (4.1V~4.5V, 内部设定时有效)
};

// 硬件运行参数配置结构体
struct HardwareConfig
{
    SCHWI_IBAT_RATIO IBAT_RATIO;     // [PSTOP] IBAT 限流/ADC 倍率 (6x/12x)
    SCHWI_IBUS_RATIO IBUS_RATIO;     // [PSTOP] IBUS 限流/ADC 倍率 (6x/3x)
    SCHWI_VBAT_RATIO VBAT_RATIO;     // [PSTOP] VBAT ADC 监测比例 (12.5x/5x)
    SCHWI_VBUS_RATIO VBUS_RATIO;     // [PSTOP] VBUS 设定与监测比例 (12.5x/5x)
    SCHWI_VINREG_RATIO VINREG_Ratio; // VINREG 门限倍率 (100x/40x)
    SCHWI_FREQ SW_FREQ;              // [PSTOP] 开关频率 (150k~450kHz)
    SCHWI_DT DeadTime;               // [PSTOP] 开关死区时间 (20~80ns)
    SCHWI_ICHAR ICHAR;               // [PSTOP] 充电电流基准 (IBUS/IBAT)
    SCHWI_TRICKLE TRICKLE;           // [PSTOP] 涓流充电使能
    SCHWI_TERM TERM;                 // [PSTOP] EOC 自动截止使能
    SCHWI_FB FB_Mode;                // [PSTOP] VBUS 反馈模式 (内部/外部)
    SCHWI_TRICKLE_SET TRICKLE_SET;   // [PSTOP] 涓流门限 (70%/60% 目标电压)
    SCHWI_OVP OVP;                   // 放电过压保护使能
    SCHWI_DITHER DITHER;             // [PSTOP] 频率抖动使能
    SCHWI_SLEW SLEW_SET;             // [PSTOP] VBUS 动态斜率 (1~8mV/μs)
    SCHWI_ADC ADC_SCAN;              // ADC 转换开关
    SCHWI_ILIM_BW ILIM_BW;           // [PSTOP] 限流环带宽 (5k/1.25kHz)
    SCHWI_LOOP LOOP;                 // [PSTOP] 环路响应 (常规/增强)
    SCHWI_SFB ShortFoldBack;         // 短路折返 (Enable=折返保护有效)
    SCHWI_EOC EOC;                   // [PSTOP] EOC 电流门限 (1/25 或 1/10)
    SCHWI_PFM PFM;                   // 轻载 PFM 使能
};

// 中断/状态位结构体 (对应 0x17 STATUS 与 0x19 MASK 的同名位)
// 中断触发机制 (手册 8.12.2 节):
//   AC_OK      : 状态跳变 (上升沿或下降沿) 触发 1ms 低脉冲 INT;
//   INDET      : 仅上升沿触发 INT; STATUS 中该位为"读后自动清除"型;
//   VBUS_SHORT : 逻辑高电平期间持续拉低 INT;
//   OTP / EOC  : 状态跳变 (上升沿或下降沿) 触发 1ms 低脉冲 INT。
// 作为 configInterruptMask 入参时: true = 屏蔽 (禁止) 对应中断输出;
// 作为 readInterruptStatus 出参时: true = 状态位有效。
struct InterruptStatus
{
    bool AC_OK;      // AC 适配器插入 (ACIN 引脚 > 3V)
    bool INDET;      // USB-A 端口负载插入检测 (读后清)
    bool VBUS_SHORT; // 放电模式 VBUS 短路故障 (VBUS < 1V)
    bool OTP;        // 过温保护 (结温 > 165°C)
    bool EOC;        // 充电结束条件满足
};

// =============================================================================
// SC8815 驱动类
// =============================================================================
class SC8815
{
public:
    // 构造函数: 可指定硬件 Wire 接口与 I2C 从机地址
    SC8815(TwoWire &wire = Wire, uint8_t addr = SC8815_I2C_ADDR);

    /**
     * @brief 初始化芯片接口并记录外部硬件参数
     * @param vbusRshunt_mOhm VBUS 侧采样电阻 RS1 阻值 (mΩ, 手册推荐 10mΩ)
     * @param battRshunt_mOhm VBAT 侧采样电阻 RS2 阻值 (mΩ, 手册推荐 5/10mΩ)
     * @param fbRup 外部 FB 分压上拉电阻 (kΩ)
     * @param fbRdown 外部 FB 分压下拉电阻 (kΩ)
     * @param vbat_fbRup 外部 VBAT 分压上拉电阻 (kΩ)
     * @param vbat_fbRdown 外部 VBAT 分压下拉电阻 (kΩ)
     * @return true: 芯片应答正常; false: 芯片无应答 (请检查连线与地址)
     * @note SC8815 无 ID 寄存器, 仅通过 I2C 应答探测; 调用前需先执行 Wire.begin()
     */
    bool begin(uint16_t vbusRshunt_mOhm = 10, uint16_t battRshunt_mOhm = 10, double vbus_fbRup = 100.0, double vbus_fbRdown = 10.0, double vbat_fbRup = 100.0, double vbat_fbRdown = 10.0);

    // 配置电池参数 (0x00 VBAT_SET), 需在 PSTOP 拉高(待机)期间调用
    void configBattery(const BatteryConfig &config);
    // 批量配置硬件运行参数 (0x08 RATIO + 0x09~0x0C CTRL0~3), 建议在 PSTOP 拉高期间调用
    void configHardware(const HardwareConfig &config);
    // 配置中断屏蔽 (0x19 MASK), 入参 true = 屏蔽对应中断输出
    void configInterruptMask(const InterruptStatus &status);
    // 读取实时中断状态 (0x17 STATUS)
    void readInterruptStatus(InterruptStatus &status);

    // =========================================================================
    // ADC 传感器数据读取 (自动换算为 mV / mA)
    // =========================================================================
    uint16_t readVbusVoltage(); // 读取 VBUS 实际电压 (mV)
    uint16_t readVbusCurrent(); // 读取 VBUS 实际电流 (mA)
    uint16_t readBattVoltage(); // 读取电池实际电压 (mV)
    uint16_t readBattCurrent(); // 读取电池实际电流 (mA)
    uint16_t readAdinVoltage(); // 读取 ADIN 引脚电压 (mV, 范围 ≤ 2048mV)

    // =========================================================================
    // 参数设置函数
    // =========================================================================
    void setOutputVoltage(uint16_t newVolt_mV);       // 设置放电(OTG)目标电压 (内部/外部 FB 模式均有效, 越界自动钳位)
    void setBusCurrentLimit(uint16_t newIlim_mA);     // 设置 VBUS 限流 (mA, 下限 300mA, 越界自动钳位)
    void setBatteryCurrentLimit(uint16_t newIlim_mA); // 设置电池端限流 (mA, 下限 300mA, 越界自动钳位)
    void setVinregVoltage(uint16_t newVolt_mV);       // 设置 VINREG 动态输入门限 (mV, 越界自动钳位)

    // =========================================================================
    // 参数查询函数
    // =========================================================================
    uint16_t getOutputVoltage();       // 获取当前设定的输出目标电压 (mV)
    uint16_t getBusCurrentLimit();     // 获取当前设定的 VBUS 限流门限 (mA)
    uint16_t getBatteryCurrentLimit(); // 获取当前设定的 VBAT 限流门限 (mA)
    uint16_t getVinregVoltage();       // 获取 VINREG 设定电压 (mV)
    uint16_t getMaxOutputVoltage();    // 获取当前 FB 配置下的最高输出电压上限 (mV)
    uint16_t getOutputVoltageStep();   // 获取当前 FB 配置下的电压单步调节精度 (mV)

    // =========================================================================
    // 控制开关设置
    // =========================================================================
    void setOtgEnable(bool enable);     // 开启/关闭 OTG 放电模式 (true: 放电, false: 充电)
    void setVinregRatio40x(bool is40x); // 设置 VINREG 倍率 (true: 40x, false: 100x)
    void setOvpEnable(bool enable);     // 开启/关闭 过压保护 OVP (仅放电模式)
    void setPgateEnable(bool enable);   // 开启/关闭 PGATE 驱动 (true: 拉低导通外部 PMOS)
    void setGpoEnable(bool enable);     // 开启/关闭 GPO 引脚 (true: 内部拉低输出)
    void setAdcEnable(bool enable);     // 开启/关闭 ADC 转换功能
    void setSfbEnable(bool enable);     // 开启/关闭 短路折返保护 (true: 折返有效)
    void setPfmEnable(bool enable);     // 开启/关闭 轻载 PFM 省电模式

    // =========================================================================
    // 状态查询
    // =========================================================================
    bool isOtgEnabled();      // [CTRL0.7] OTG 放电模式是否使能
    uint8_t getVinregRatio(); // [CTRL0.4] 返回 40 或 100
    bool isOvpEnabled();      // [CTRL1.2] OVP 是否使能 (该位 0 为使能)
    bool isPgateEnabled();    // [CTRL3.7] PGATE 驱动是否使能
    bool isGpoEnabled();      // [CTRL3.6] GPO 输出是否使能
    bool isAdcEnabled();      // [CTRL3.5] ADC 转换是否使能
    bool isSfbEnabled();      // [CTRL3.2] 短路折返是否使能 (该位 0 为使能)
    bool isPfmEnabled();      // [CTRL3.0] PFM 模式是否使能

private:
    TwoWire *_wire;
    uint8_t _addr;
    uint16_t _vbusRshunt; // VBUS 端采样电阻 RS1 阻值 (mΩ)
    uint16_t _battRshunt; // VBAT 端采样电阻 RS2 阻值 (mΩ)
    double _vbus_fbRup;    // 外部VBUS FB 分压上拉电阻 (kΩ)
    double _vbus_fbRdown;  // 外部VBUS FB 分压下拉电阻 (kΩ)

    double _adin_fbRup;   // 外部 ADIN FB 分压上拉电阻 (kΩ)
    double _adin_fbRdown; // 外部 ADIN FB 分压下拉电阻 (kΩ)

    bool writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
};

#endif // SC8815_H
