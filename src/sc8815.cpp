#include "sc8815.h"

// =============================================================================
// 本驱动所有换算公式均依据 SC8815 Datasheet 第 10 章 Register Map, 约定如下:
//
//   [电压 DAC]  10-bit 参考码 N = 4×SET + SET2 + 1 (SET 为高 8 位, SET2 为低 2 位)
//               参考电压  VREF = N × 2mV, 范围 2mV ~ 2048mV
//               内部 FB:  VBUS = VREF × VBUS_RATIO   (VBUS_RATIO: 5x / 12.5x)
//               外部 FB:  VBUS = VREF × (1 + Rup/Rdown)
//   [电压 ADC]  VBUS = (4×VBUS_FB_VALUE + VBUS_FB_VALUE2 + 1) × VBUS_RATIO × 2mV
//               VBAT = (4×VBAT_FB_VALUE + VBAT_FB_VALUE2 + 1) × VBAT_MON_RATIO × 2mV
//   [电流 ADC]  IBUS(A) = (N × 2 / 1200) × IBUS_RATIO × 10mΩ/RS1
//               IBAT(A) = (N × 2 / 1200) × IBAT_RATIO × 10mΩ/RS2
//   [限流门限]  ILIM(A) = (SET + 1)/256 × RATIO × 10mΩ/RS
//   [VINREG]    VINREG = (VINREG_SET + 1) × VINREG_RATIO
//
// 其中 N = 4×高8位寄存器 + 低2位寄存器(bit7:6) + 1
// =============================================================================

SC8815::SC8815(TwoWire &wire, uint8_t addr)
    : _wire(&wire), _addr(addr), _vbusRshunt(10), _battRshunt(10), _fbRup(100.0), _fbRdown(10.0) {}

// 初始化: 记录采样电阻与 FB 分压参数, 并探测芯片 I2C 应答
// 注意: 调用前需先完成 Wire.begin() 与引脚映射
bool SC8815::begin(uint16_t vbusRshunt_mOhm, uint16_t battRshunt_mOhm, double fbRup, double fbRdown)
{
    _vbusRshunt = vbusRshunt_mOhm;
    _battRshunt = battRshunt_mOhm;
    _fbRup = fbRup;
    _fbRdown = fbRdown;

    // SC8815 无芯片 ID 寄存器 (手册未定义), 仅通过 I2C 从机地址应答探测
    _wire->beginTransmission(_addr);
    return (_wire->endTransmission() == 0);
}

// 写单个寄存器 (手册 8.12.1.6 单写时序: S + 地址(写) + 寄存器 + 数据 + P)
// @return true: 总线传输成功 (收到 ACK); false: 无应答
bool SC8815::writeRegister(uint8_t reg, uint8_t data)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(data);
    return (_wire->endTransmission() == 0);
}

// 读单个寄存器 (手册 8.12.1.6 单读时序, 使用重复起始条件)
// @return 寄存器值; 总线异常时返回 0
uint8_t SC8815::readRegister(uint8_t reg)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) // false: 发送重复起始条件, 保持总线占用
        return 0;
    if (_wire->requestFrom(_addr, (uint8_t)1) != 1)
        return 0;
    return _wire->read();
}

// =============================================================================
// 配置函数
// =============================================================================

// 配置电池参数 (0x00 VBAT_SET)
// 位域: [7:6]IRCOMP [5]VBAT_SEL [4:3]CSEL [2:0]VCELL_SET
// 手册要求: 以上各位均需在 PSTOP 拉高 (待机) 期间写入
void SC8815::configBattery(const BatteryConfig &config)
{
    // 四组字段位域互不重叠, 直接按位或拼接
    uint8_t tmp = config.IRCOMP | config.VBAT_SEL | config.CSEL | config.VCELL;
    writeRegister(SCREG_VBAT_SET, tmp);
}

// 批量配置硬件运行参数 (0x08 RATIO 与 0x09~0x0C CTRL0~CTRL3)
// 采用"读-改-写"保护手册中标注 "Internal use. Don't overwrite" 的保留位:
//   0x08 RATIO : 保留 [7:5] (其中 bit5 复位值为 1, 不可覆盖)
//   0x09 CTRL0 : 保留 [6:5] 及 EN_OTG[7] (OTG 开关由 setOtgEnable 单独管理)
//   0x0A CTRL1 : 保留 [1:0] (其中 bit0 复位值为 1)
//   0x0B CTRL2 : 保留 [7:4], 并置 FACTORY[3]=1 (手册要求上电后写入该位)
//   0x0C CTRL3 : 保留 EN_PGATE[7] / GPO_CTRL[6] (由 setPgateEnable 等单独控制)
void SC8815::configHardware(const HardwareConfig &config)
{
    // 1. 采样/监测比例 (0x08 RATIO)
    uint8_t tmp = readRegister(SCREG_RATIO) & 0xE0;
    tmp |= config.IBAT_RATIO | config.IBUS_RATIO | config.VBAT_RATIO | config.VBUS_RATIO;
    writeRegister(SCREG_RATIO, tmp);

    // 2. 模式与频率 (0x09 CTRL0), 保留 EN_OTG 与保留位 [6:5]
    tmp = readRegister(SCREG_CTRL0_SET) & 0xE0;
    tmp |= config.VINREG_Ratio | config.SW_FREQ | config.DeadTime;
    writeRegister(SCREG_CTRL0_SET, tmp);

    // 3. 充电/放电控制 (0x0A CTRL1), 保留位 [1:0]
    tmp = readRegister(SCREG_CTRL1_SET) & 0x03;
    tmp |= config.ICHAR | config.TRICKLE | config.TERM | config.FB_Mode | config.TRICKLE_SET | config.OVP;
    writeRegister(SCREG_CTRL1_SET, tmp);

    // 4. 抖动与斜率 (0x0B CTRL2), 保留 [7:4], FACTORY[3] 必须置 1
    tmp = (readRegister(SCREG_CTRL2_SET) & 0xF0) | 0x08;
    tmp |= config.DITHER | config.SLEW_SET;
    writeRegister(SCREG_CTRL2_SET, tmp);

    // 5. 驱动/ADC/保护 (0x0C CTRL3), 保留 EN_PGATE[7] 与 GPO_CTRL[6]
    tmp = readRegister(SCREG_CTRL3_SET) & 0xC0;
    tmp |= config.ADC_SCAN | config.ILIM_BW | config.LOOP | config.ShortFoldBack | config.EOC | config.PFM;
    writeRegister(SCREG_CTRL3_SET, tmp);

    delay(5); // 等待配置写入生效
}

// 配置中断屏蔽 (0x19 MASK)
// 位域: [7]保留(复位1) [6]AC_OK [5]INDET [4]保留 [3]VBUS_SHORT [2]OTP [1]EOC [0]保留
// 入参 status 中某位为 true 表示"屏蔽"对应中断 (状态位仍会在 0x17 STATUS 中正常置位,
// 只是不再触发 INT 引脚脉冲), 详见手册 8.12.2 节
void SC8815::configInterruptMask(const InterruptStatus &status)
{
    uint8_t tmp = readRegister(SCREG_MASK) & 0x91; // 保留 bit7 / bit4 / bit0 原值
    tmp |= 0x01;                                   // 手册要求: bit0 上电后需写 1
    tmp |= status.AC_OK ? 0x40 : 0x00;
    tmp |= status.INDET ? 0x20 : 0x00;
    tmp |= status.VBUS_SHORT ? 0x08 : 0x00;
    tmp |= status.OTP ? 0x04 : 0x00;
    tmp |= status.EOC ? 0x02 : 0x00;
    writeRegister(SCREG_MASK, tmp);
}

// 读取实时中断状态 (0x17 STATUS)
// 位域: [6]AC_OK [5]INDET [3]VBUS_SHORT [2]OTP [1]EOC
// 注意: 除 INDET (读后自动清除) 外, 其余位均反映实时状态
void SC8815::readInterruptStatus(InterruptStatus &status)
{
    uint8_t tmp = readRegister(SCREG_STATUS);
    status.AC_OK = (tmp & 0x40) >> 6;
    status.INDET = (tmp & 0x20) >> 5;
    status.VBUS_SHORT = (tmp & 0x08) >> 3;
    status.OTP = (tmp & 0x04) >> 2;
    status.EOC = (tmp & 0x02) >> 1;
}

// =============================================================================
// ADC 数据读取 (自动换算, 单位 mV / mA)
// =============================================================================

// 读取 VBUS 实际电压 (mV)
// 手册公式: VBUS = (4×VBUS_FB_VALUE + VBUS_FB_VALUE2 + 1) × VBUS_RATIO × 2mV
uint16_t SC8815::readVbusVoltage()
{
    // RATIO[0]: VBUS 监测比例 (5x / 12.5x)
    double ratio = ((readRegister(SCREG_RATIO) & 0x01) == 1) ? 5.0 : 12.5;
    uint8_t tmp1 = readRegister(SCREG_VBUS_FB_VALUE);       // 高 8 位
    uint8_t tmp2 = readRegister(SCREG_VBUS_FB_VALUE2) >> 6; // 低 2 位
    // 先做整数乘法再乘比例, 保证 12.5x 时结果精确 (N×2×ratio 必为整数)
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * 2 * ratio);
}

// 读取 IBUS 实际电流 (mA)
// 手册公式: IBUS(A) = ((4×IBUS_VALUE + IBUS_VALUE2 + 1) × 2 / 1200) × IBUS_RATIO × 10mΩ/RS1
// 换算为 mA 后化简: IBUS(mA) = 50 × IBUS_RATIO × N / (3 × RS1)
uint16_t SC8815::readVbusCurrent()
{
    // RATIO[3:2]: 01=6x, 10=3x
    uint32_t ratio = ((readRegister(SCREG_RATIO) & 0x0C) == 0x04) ? 6 : 3;
    uint8_t tmp1 = readRegister(SCREG_IBUS_VALUE);
    uint8_t tmp2 = readRegister(SCREG_IBUS_VALUE2) >> 6;
    // 使用 32 位中间量, 避免 16 位平台上 50×ratio×N (最大 307200) 溢出
    return (uint16_t)(((uint32_t)50 * ratio * (4 * tmp1 + tmp2 + 1)) / (3 * _vbusRshunt));
}

// 读取 VBAT 实际电压 (mV)
// 手册公式: VBAT = (4×VBAT_FB_VALUE + VBAT_FB_VALUE2 + 1) × VBAT_MON_RATIO × 2mV
uint16_t SC8815::readBattVoltage()
{
    // RATIO[1]: VBAT 监测比例 (12.5x / 5x)
    double ratio = ((readRegister(SCREG_RATIO) & 0x02) == 0x02) ? 5.0 : 12.5;
    uint8_t tmp1 = readRegister(SCREG_VBAT_FB_VALUE);
    uint8_t tmp2 = readRegister(SCREG_VBAT_FB_VALUE2) >> 6;
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * 2 * ratio);
}

// 读取 IBAT 实际电流 (mA)
// 手册公式: IBAT(A) = ((4×IBAT_VALUE + IBAT_VALUE2 + 1) × 2 / 1200) × IBAT_RATIO × 10mΩ/RS2
// 换算为 mA 后化简: IBAT(mA) = 50 × IBAT_RATIO × N / (3 × RS2)
uint16_t SC8815::readBattCurrent()
{
    // RATIO[4]: 0=6x, 1=12x
    uint32_t ratio = ((readRegister(SCREG_RATIO) & 0x10) == 0x10) ? 12 : 6;
    uint8_t tmp1 = readRegister(SCREG_IBAT_VALUE);
    uint8_t tmp2 = readRegister(SCREG_IBAT_VALUE2) >> 6;
    return (uint16_t)(((uint32_t)50 * ratio * (4 * tmp1 + tmp2 + 1)) / (3 * _battRshunt));
}

// 读取 ADIN 引脚电压 (mV), 量程 ≤ 2048mV, 分辨率 2mV
// 手册公式: VADIN = (4×ADIN_VALUE + ADIN_VALUE2 + 1) × 2mV
uint16_t SC8815::readAdinVoltage()
{
    uint8_t tmp1 = readRegister(SCREG_ADIN_VALUE);
    uint8_t tmp2 = readRegister(SCREG_ADIN_VALUE2) >> 6;
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * 2);
}

// =============================================================================
// 参数设置函数
// =============================================================================

/**
 * @brief 设置放电(OTG)模式 VBUS 目标电压 (自动兼容内部 / 外部 FB 模式)
 * @param newVolt_mV 期望设置的电压值 (单位: mV)
 *
 * 手册公式 (10-bit DAC):
 *   参考码 N = 4×SET + SET2 + 1, 有效范围 1 ~ 1024 (对应 2mV ~ 2048mV)
 *   内部 FB: VBUS = N × 2mV × VBUS_RATIO
 *   外部 FB: VBUS = N × 2mV × (1 + Rup/Rdown)
 * 反解步骤:
 *   N = VBUS / (ratio × 2mV) -> SET = (N-1)/4 (高 8 位), SET2 = (N-1)%4 (低 2 位)
 * 越界处理: N 自动钳位到 1~1024, 防止写入回绕
 */
void SC8815::setOutputVoltage(uint16_t newVolt_mV)
{
    double ratio;
    bool external = (readRegister(SCREG_CTRL1_SET) & 0x10) != 0; // CTRL1[4] FB_SEL

    if (external)
        ratio = (_fbRup / _fbRdown) + 1.0;                                 // 外部分压比
    else
        ratio = ((readRegister(SCREG_RATIO) & 0x01) == 1) ? 5.0 : 12.5;    // 内部 RATIO[0]

    // 计算 10-bit 参考码 (四舍五入), 并钳位到 [1, 1024]
    uint32_t code = (uint32_t)((newVolt_mV / ratio) / 2.0 + 0.5);
    if (code < 1)
        code = 1;
    if (code > 1024)
        code = 1024;

    uint8_t set_h = (uint8_t)((code - 1) / 4); // 高 8 位 -> *_SET 寄存器
    uint8_t set_l = (uint8_t)((code - 1) % 4); // 低 2 位 -> *_SET2 寄存器 bit7:6

    if (external)
    {
        // 外部 FB 模式: 0x03 / 0x04
        writeRegister(SCREG_VBUSREF_E_SET, set_h);
        writeRegister(SCREG_VBUSREF_E_SET2, set_l << 6);
    }
    else
    {
        // 内部 FB 模式: 0x01 / 0x02
        writeRegister(SCREG_VBUSREF_I_SET, set_h);
        writeRegister(SCREG_VBUSREF_I_SET2, set_l << 6);
    }
}

// 设置 IBUS 限流门限 (mA), 充电 / 放电模式均有效
// 手册公式: IBUS_LIM(A) = (IBUS_LIM_SET + 1)/256 × IBUS_RATIO × 10mΩ/RS1
// 反解:     IBUS_LIM_SET + 1 = Ilim(mA) × 16 × RS1 / (625 × IBUS_RATIO)
// 手册约束: 限流值不得低于 300mA; 上限为 256 档 (RS1=10mΩ 时 3x: 3A, 6x: 6A)
void SC8815::setBusCurrentLimit(uint16_t newIlim_mA)
{
    uint32_t ratio = ((readRegister(SCREG_RATIO) & 0x0C) == 0x04) ? 6 : 3;
    uint32_t maxIlim_mA = (uint32_t)10000 * ratio / _vbusRshunt; // (SET+1)=256 档对应的限流

    // 越界钳位: 下限 300mA, 上限为满量程
    if (newIlim_mA < 300)
        newIlim_mA = 300;
    if (newIlim_mA > maxIlim_mA)
        newIlim_mA = (uint16_t)maxIlim_mA;

    // 反解 SET+1 (四舍五入), 再写入 SET
    uint32_t code = ((uint32_t)newIlim_mA * 16 * _vbusRshunt + (625UL * ratio) / 2) / (625UL * ratio);
    writeRegister(SCREG_IBUS_LIM_SET, (uint8_t)(code - 1));
}

// 设置电池端限流门限 (mA), 充电 / 放电模式均有效
// 手册公式: IBAT_LIM(A) = (IBAT_LIM_SET + 1)/256 × IBAT_RATIO × 10mΩ/RS2
// 反解:     IBAT_LIM_SET + 1 = Ilim(mA) × 16 × RS2 / (625 × IBAT_RATIO)
// 手册约束: 限流值不得低于 300mA; 上限为 256 档 (RS2=10mΩ 时 6x: 6A, 12x: 12A)
void SC8815::setBatteryCurrentLimit(uint16_t newIlim_mA)
{
    uint32_t ratio = ((readRegister(SCREG_RATIO) & 0x10) == 0x10) ? 12 : 6;
    uint32_t maxIlim_mA = (uint32_t)10000 * ratio / _battRshunt; // 注意: 使用 VBAT 侧采样电阻 RS2

    if (newIlim_mA < 300)
        newIlim_mA = 300;
    if (newIlim_mA > maxIlim_mA)
        newIlim_mA = (uint16_t)maxIlim_mA;

    uint32_t code = ((uint32_t)newIlim_mA * 16 * _battRshunt + (625UL * ratio) / 2) / (625UL * ratio);
    writeRegister(SCREG_IBAT_LIM_SET, (uint8_t)(code - 1));
}

// 设置 VINREG 动态输入电压门限 (mV), 充电模式下 VBUS 被拉低至该门限时自动降低充电电流
// 手册公式: VINREG = (VINREG_SET + 1) × VINREG_RATIO
// 反解:     VINREG_SET = VINREG(mV)/RATIO - 1, SET 范围 0~255, 即最大 256 × RATIO
void SC8815::setVinregVoltage(uint16_t newVolt_mV)
{
    uint16_t ratio = ((readRegister(SCREG_CTRL0_SET) & 0x10) == 0x10) ? 40 : 100; // CTRL0[4]
    uint16_t maxVolt_mV = 256 * ratio;

    // 越界钳位到 [1×RATIO, 256×RATIO]
    if (newVolt_mV < ratio)
        newVolt_mV = ratio;
    if (newVolt_mV > maxVolt_mV)
        newVolt_mV = maxVolt_mV;

    // 反解 SET+1 (四舍五入), 再写入 SET
    uint32_t code = ((uint32_t)newVolt_mV + ratio / 2) / ratio;
    writeRegister(SCREG_VINREG_SET, (uint8_t)(code - 1));
}

// =============================================================================
// 参数查询函数
// =============================================================================

// 获取当前设定的输出目标电压 (mV)
// 与 setOutputVoltage 相同的正解公式: VBUS = N × 2mV × ratio
uint16_t SC8815::getOutputVoltage()
{
    uint16_t tmp1, tmp2;
    double ratio;

    if (readRegister(SCREG_CTRL1_SET) & 0x10)
    {
        // 外部 FB 模式: 读取 0x03 / 0x04, 结合外部分压比
        tmp1 = readRegister(SCREG_VBUSREF_E_SET);
        tmp2 = readRegister(SCREG_VBUSREF_E_SET2) >> 6;
        ratio = (_fbRup / _fbRdown) + 1.0;
    }
    else
    {
        // 内部 FB 模式: 读取 0x01 / 0x02, 结合 VBUS_RATIO
        tmp1 = readRegister(SCREG_VBUSREF_I_SET);
        tmp2 = readRegister(SCREG_VBUSREF_I_SET2) >> 6;
        ratio = ((readRegister(SCREG_RATIO) & 0x01) == 1) ? 5.0 : 12.5;
    }
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * 2 * ratio);
}

// 获取设定的 IBUS 限流门限 (mA)
// 正解公式: Ilim(mA) = (SET + 1) × 625 × IBUS_RATIO / (16 × RS1)
uint16_t SC8815::getBusCurrentLimit()
{
    uint32_t ratio = ((readRegister(SCREG_RATIO) & 0x0C) == 0x04) ? 6 : 3;
    uint8_t tmp = readRegister(SCREG_IBUS_LIM_SET);
    return (uint16_t)((625UL * ratio * (tmp + 1)) / (16 * _vbusRshunt));
}

// 获取设定的 VBAT 限流门限 (mA)
// 正解公式: Ilim(mA) = (SET + 1) × 625 × IBAT_RATIO / (16 × RS2)
uint16_t SC8815::getBatteryCurrentLimit()
{
    uint32_t ratio = ((readRegister(SCREG_RATIO) & 0x10) == 0x10) ? 12 : 6;
    uint8_t tmp = readRegister(SCREG_IBAT_LIM_SET);
    return (uint16_t)((625UL * ratio * (tmp + 1)) / (16 * _battRshunt));
}

// 获取设定的 VINREG 电压 (mV)
// 手册公式: VINREG = (VINREG_SET + 1) × VINREG_RATIO
uint16_t SC8815::getVinregVoltage()
{
    uint16_t ratio = ((readRegister(SCREG_CTRL0_SET) & 0x10) == 0x10) ? 40 : 100;
    uint8_t tmp = readRegister(SCREG_VINREG_SET);
    return (uint16_t)((tmp + 1) * ratio);
}

// 获取当前 FB 配置下可输出的最高电压 (mV)
// 10-bit 参考电压上限 2048mV, 故上限 = 2048 × ratio
uint16_t SC8815::getMaxOutputVoltage()
{
    double ratio = (readRegister(SCREG_CTRL1_SET) & 0x10)
                       ? ((_fbRup / _fbRdown) + 1.0)
                       : (((readRegister(SCREG_RATIO) & 0x01) == 1) ? 5.0 : 12.5);
    return (uint16_t)(2048 * ratio);
}

// 获取当前 FB 配置下的电压单步调节精度 (mV)
// DAC 分辨率 2mV × ratio (内部 12.5x 时 25mV/step, 5x 时 10mV/step)
uint16_t SC8815::getOutputVoltageStep()
{
    double ratio = (readRegister(SCREG_CTRL1_SET) & 0x10)
                       ? ((_fbRup / _fbRdown) + 1.0)
                       : (((readRegister(SCREG_RATIO) & 0x01) == 1) ? 5.0 : 12.5);
    return (uint16_t)(2 * ratio);
}

// =============================================================================
// 开关及位操作函数实现
// =============================================================================

// 使能/关闭 OTG 放电模式 (0x09 CTRL0[7] EN_OTG)
// 0: 充电模式 (VBUS -> VBAT, 复位默认); 1: 放电模式 (VBAT -> VBUS)
void SC8815::setOtgEnable(bool enable)
{
    uint8_t val = readRegister(SCREG_CTRL0_SET);
    writeRegister(SCREG_CTRL0_SET, enable ? (val | 0x80) : (val & 0x7F));
}

// 设置 VINREG 倍率 (0x09 CTRL0[4])
// 0 = 100x (最大 25.6V); 1 = 40x (最大 10.24V), VBUS < 12V 时建议 40x
void SC8815::setVinregRatio40x(bool is40x)
{
    uint8_t val = readRegister(SCREG_CTRL0_SET);
    writeRegister(SCREG_CTRL0_SET, is40x ? (val | 0x10) : (val & 0xEF));
}

// 使能/禁用放电模式过压保护 (0x0A CTRL1[2], 该位 0 为使能, 1 为禁用)
// 使能时 VBUS 超过目标值 10% 即停止开关 (手册 8.11.1 节)
void SC8815::setOvpEnable(bool enable)
{
    uint8_t val = readRegister(SCREG_CTRL1_SET);
    writeRegister(SCREG_CTRL1_SET, enable ? (val & 0xFB) : (val | 0x04));
}

// 使能/关闭 PGATE 驱动输出 (0x0C CTRL3[7] EN_PGATE)
// 1: PGATE 拉低 (经 6kΩ 下拉) 导通外部 PMOS; 0: PGATE 经 20kΩ 上拉至 VBUS 关断 PMOS
void SC8815::setPgateEnable(bool enable)
{
    uint8_t val = readRegister(SCREG_CTRL3_SET);
    writeRegister(SCREG_CTRL3_SET, enable ? (val | 0x80) : (val & 0x7F));
}

// 使能/关闭 GPO 开漏输出 (0x0C CTRL3[6] GPO_CTRL)
// 1: GPO 内部 6kΩ 拉低; 0: 高阻 (需外部上拉)
void SC8815::setGpoEnable(bool enable)
{
    uint8_t val = readRegister(SCREG_CTRL3_SET);
    writeRegister(SCREG_CTRL3_SET, enable ? (val | 0x40) : (val & 0xBF));
}

// 使能/关闭 ADC 转换 (0x0C CTRL3[5] AD_START)
// 开启后可读取 VBUS/VBAT 电压与 IBUS/IBAT 电流 ADC 寄存器
void SC8815::setAdcEnable(bool enable)
{
    uint8_t val = readRegister(SCREG_CTRL3_SET);
    writeRegister(SCREG_CTRL3_SET, enable ? (val | 0x20) : (val & 0xDF));
}

// 使能/关闭短路折返保护 (0x0C CTRL3[2] DIS_ShortFoldBack, 该位 0 为使能, 1 为禁用)
// 使能时: VBUS < 1V (短路) 则 IBUS 限流折返至 22%, IBAT 折返至 10% (手册 8.11.3 节)
void SC8815::setSfbEnable(bool enable)
{
    uint8_t val = readRegister(SCREG_CTRL3_SET);
    writeRegister(SCREG_CTRL3_SET, enable ? (val & 0xFB) : (val | 0x04));
}

// 使能/关闭轻载 PFM 模式 (0x0C CTRL3[0] EN_PFM, 仅放电模式有效)
void SC8815::setPfmEnable(bool enable)
{
    uint8_t val = readRegister(SCREG_CTRL3_SET);
    writeRegister(SCREG_CTRL3_SET, enable ? (val | 0x01) : (val & 0xFE));
}

// =============================================================================
// 状态查询实现 (读取对应控制寄存器并解析控制位)
// =============================================================================

// OTG 放电模式是否使能 (CTRL0[7])
bool SC8815::isOtgEnabled() { return (readRegister(SCREG_CTRL0_SET) & 0x80) != 0; }

// VINREG 当前倍率: 返回 40 或 100 (CTRL0[4])
uint8_t SC8815::getVinregRatio() { return (readRegister(SCREG_CTRL0_SET) & 0x10) ? 40 : 100; }

// OVP 是否使能 (CTRL1[2], 该位为 0 表示使能)
bool SC8815::isOvpEnabled() { return (readRegister(SCREG_CTRL1_SET) & 0x04) == 0; }

// PGATE 驱动是否使能 (CTRL3[7])
bool SC8815::isPgateEnabled() { return (readRegister(SCREG_CTRL3_SET) & 0x80) != 0; }

// GPO 输出是否使能 (CTRL3[6])
bool SC8815::isGpoEnabled() { return (readRegister(SCREG_CTRL3_SET) & 0x40) != 0; }

// ADC 转换是否使能 (CTRL3[5])
bool SC8815::isAdcEnabled() { return (readRegister(SCREG_CTRL3_SET) & 0x20) != 0; }

// 短路折返保护是否使能 (CTRL3[2], 该位为 0 表示使能)
bool SC8815::isSfbEnabled() { return (readRegister(SCREG_CTRL3_SET) & 0x04) == 0; }

// PFM 模式是否使能 (CTRL3[0])
bool SC8815::isPfmEnabled() { return (readRegister(SCREG_CTRL3_SET) & 0x01) != 0; }
