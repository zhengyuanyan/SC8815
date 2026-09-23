#include <Arduino.h>
#include <Wire.h>
#include "sc8815/src/sc8815.h"

#define I2C_CLOCK 400000
#define SC8815_B_INT_PIN 3
#define SC8815_B_SDA_PIN 4
#define SC8815_B_SCL_PIN 5
#define SC8815_B_PSTOP_PIN 6 // 统一更名为 PSTOP (Power Stop)
#define SC8815_B_CE_PIN 7    // CE (Chip Enable)

#define OUT_TARGET_VOLTAGE_MV 29000 // KNX 总线目标电压 30V
#define OUT_MAX_CURRENT_MA 1280      // KNX 总线最大电流 640mA

const uint16_t VBUS_RSHUNT_MOHM = 10; // VBUS 线路采样电阻 (10 mΩ)
const uint16_t BATT_RSHUNT_MOHM = 10; // VBAT 线路采样电阻 (10 mΩ)

const double FB_RUP_KOHM = 200.0;  // 外部 FB 分压网络 - 上拉电阻 (200 kΩ)
const double FB_RDOWN_KOHM = 13.3; // 外部 FB 分压网络 - 下拉电阻 (13.3 kΩ)

static uint32_t last_print = 0;

SC8815 BUS_PWR(Wire, SC8815_I2C_ADDR);

// CE (Chip Enable): 低电平使能芯片，高电平关闭 IC
void SC8815_B_PIN_CE(bool enable)
{
  digitalWrite(SC8815_B_CE_PIN, enable ? LOW : HIGH);
}

// PSTOP (Power Stop): 低电平使能 PWM 开关，高电平强制 Standby 关断功率输出
void SC8815_B_PIN_PSTOP(bool enable)
{
  digitalWrite(SC8815_B_PSTOP_PIN, enable ? LOW : HIGH);
}
void setup()
{
  Serial.begin(115200);
  delay(2000); // 建议适当缩短等待时间

  Serial.println("初始化 SC8815...");

  // 初始化硬件控制 GPIO 引脚
  pinMode(SC8815_B_PSTOP_PIN, OUTPUT);
  pinMode(SC8815_B_CE_PIN, OUTPUT);

  // 先保持 PSTOP 关断（高电平），开启 CE 芯片供电
  SC8815_B_PIN_PSTOP(false); // 强制输出 HIGH，关闭 MOSFET 驱动
  SC8815_B_PIN_CE(true);     // 输出 LOW，使能 IC 供电
  delay(10);                 // 等待内部 LDO 稳定

  Wire.setSDA(SC8815_B_SDA_PIN);
  Wire.setSCL(SC8815_B_SCL_PIN);
  Wire.setClock(I2C_CLOCK);
  Wire.begin();

  // 1. 初始化驱动并传递硬件分压阻值（用于精准计算外部 FB 模式下的 DAC 输出电压）
  if (!BUS_PWR.begin(VBUS_RSHUNT_MOHM, BATT_RSHUNT_MOHM, FB_RUP_KOHM, FB_RDOWN_KOHM))
  {
    Serial.println("错误: SC8815 芯片未响应，请检查 I2C 连线或从机地址!");
    while (1)
      delay(100);
  }

  // 2. 配置电池参数 (PSTOP 保持高电平/待机状态下写入, 手册要求)
  // 【必须配置】若保持 POR 默认值 (1S / 4.2V), 放电(OTG)模式下 VBAT 过压
  // 保护门限仅约 4.2V×1.055≈4.4V (手册 8.11.2 节: VBAT OVP 在充/放电模式
  // 均有效), 13.1V 的 4S 电池上电即触发 OVP, IC 停止开关, OTG 无输出!
  BatteryConfig batCfg;
  batCfg.IRCOMP = SCBAT_IRCOMP_0mR;          // 禁用内阻补偿 (0 mΩ)
  batCfg.VBAT_SEL = SCBAT_VBAT_SEL_Internal; // VBATS 引脚接 VBAT, 内部采样
  batCfg.CSEL = SCBAT_CSEL_4S;               // 按实际电池组设置: 4S (可选 1S~4S)
  batCfg.VCELL = SCBAT_VCELL_4v2;            // 单节截止 4.2V (4S 目标 16.8V)
  BUS_PWR.configBattery(batCfg);

  // 3. 配置硬件运行模式（核心：使能外部 FB）
  HardwareConfig hwCfg = {};
  hwCfg.IBAT_RATIO = SCHWI_IBAT_RATIO_6x;
  hwCfg.IBUS_RATIO = SCHWI_IBUS_RATIO_3x;
  hwCfg.VBAT_RATIO = SCHWI_VBAT_RATIO_12_5x;
  hwCfg.VBUS_RATIO = SCHWI_VBUS_RATIO_12_5x;
  hwCfg.VINREG_Ratio = SCHWI_VINREG_RATIO_100x;
  hwCfg.SW_FREQ = SCHWI_FREQ_450KHz; // 开关频率 450kHz
  hwCfg.DeadTime = SCHWI_DT_40ns;      // 死区时间 40ns
  hwCfg.ICHAR = SCHWI_ICHAR_IBUS;
  hwCfg.TRICKLE = SCHWI_TRICKLE_Disable;
  hwCfg.TERM = SCHWI_TERM_Disable;

  // 【关键步骤】设置反馈模式为：外部 FB 模式
  hwCfg.FB_Mode = SCHWI_FB_External;

  hwCfg.TRICKLE_SET = SCHWI_TRICKLE_SET_70;
  hwCfg.OVP = SCHWI_OVP_Enable; // 开启过压保护
  hwCfg.DITHER = SCHWI_DITHER_Disable;
  hwCfg.SLEW_SET = SCHWI_SLEW_1mV_us;
  hwCfg.ADC_SCAN = SCHWI_ADC_Enable; // 使能 ADC，以便实时读取 VBUS/IBUS
  hwCfg.ILIM_BW = SCHWI_ILIM_BW_1_25KHz;
  hwCfg.LOOP = SCHWI_LOOP_Normal;
  // 带载启动期间必须关闭短路折返 (手册 8.2.1 节): 若 VBUS 起始低于 1V 时
  // 折返使能, IBUS/IBAT 限流降至 22%/10%, 带载输出可能永远无法建立;
  // 启动完成后调用 setSfbEnable(true) 恢复短路保护功能
  hwCfg.ShortFoldBack = SCHWI_SFB_Disable;
  hwCfg.EOC = SCHWI_EOC_1_25;
  hwCfg.PFM = SCHWI_PFM_Disable;

  BUS_PWR.configHardware(hwCfg);

  // 4. 设置 OTG 模式下的保护限流与目标输出电压
  BUS_PWR.setBusCurrentLimit(OUT_MAX_CURRENT_MA);    // 设置 OTG 输出限流 640mA 
  BUS_PWR.setOutputVoltage(OUT_TARGET_VOLTAGE_MV);   // 设置目标输出电压为 30000mV (30V, 外部FB分压)
  delay(10);

  BUS_PWR.setOtgEnable(true);

  delay(20);
  // 3. 最后释放 PSTOP (拉低为 LOW)，解锁 PWM 功率输出
  SC8815_B_PIN_PSTOP(true); // 传入 true 会输出 LOW 电平

  // 4. 等待软启动建立: VBUS 斜率 1mV/μs, 0→30V 约需 30ms, 此处留足余量
  delay(200);

  // 5. 启动自检: 读取 STATUS, 输出建立后再恢复短路折返保护
  {
    InterruptStatus st;
    BUS_PWR.readInterruptStatus(st);
    Serial.printf("[STATUS] AC_OK:%d INDET:%d VBUS_SHORT:%d OTP:%d EOC:%d\n",
                  st.AC_OK, st.INDET, st.VBUS_SHORT, st.OTP, st.EOC);

    if (BUS_PWR.readVbusVoltage() >= 20000)
    {
      // 输出已建立: 恢复短路折返保护 (手册 8.2.1 节建议流程)
      BUS_PWR.setSfbEnable(true);
      Serial.println("OTG 输出已建立, 短路折返保护已恢复");
    }
    else
    {
      Serial.println("[警告] VBUS 输出未建立! 请检查电池 CSEL 配置、外部 FB 分压、MOS 与电感等硬件");
    }
  }

  Serial.println("==========================================");
  Serial.println("SC8815 OTG 外部 FB 模式配置完成!");
  Serial.printf("当前配置下的最大支持电压上限: %u mV\n", BUS_PWR.getMaxOutputVoltage());
  Serial.printf("当前配置下的电压调节步进精度: %u mV\n", BUS_PWR.getOutputVoltageStep());

  Serial.println("==========================================");

  BUS_PWR.setPgateEnable(true);
}

void loop()
{
  // 每 1000ms 刷新一次
  uint32_t now = millis();
  if (millis() - last_print >= 1000)
  {
    last_print = now;
    if (BUS_PWR.isOtgEnabled())
    {
      // uint16_t vbus = BUS_PWR.readVbusVoltage();     // 读取 VBUS 输出电压
      uint16_t ibus = BUS_PWR.readVbusCurrent();     // 读取 VBUS 输出电流
      uint16_t setVolt = BUS_PWR.getOutputVoltage(); // 读取设置的控制电压

      // 注意: VBUS 内部 ADC 在 12.5x 比例下量程上限为 25.6V,
      // 30V 输出时会饱和显示 25600 mV (属正常现象, 非故障);
      // 若需精确监测 30V, 可将分压后的电压接入 ADIN 引脚读取
      // Serial.printf("[VBUS] 设定位: %u mV | 实际 VBUS: %u mV | 实际输出电流: %u mA\n",
      //               setVolt, vbus, ibus);
      Serial.printf("[VBUS] 设定位: %u mV | 实际输出电流: %u mA\n", setVolt, ibus);
     

      InterruptStatus st;
      BUS_PWR.readInterruptStatus(st);
      if (st.VBUS_SHORT || st.OTP)
        Serial.printf("[FAULT] VBUS_SHORT=%d OTP=%d\n", st.VBUS_SHORT, st.OTP);

      uint16_t vbat = BUS_PWR.readBattVoltage(); // 读取 VBAT 输入电压
      uint16_t ibat = BUS_PWR.readBattCurrent(); // 读取 IBAT 输入电流

      Serial.printf("[VBAT] 实际 VBAT: %u mV | 实际输入电流: %u mA\n", vbat, ibat);       
    }
    else
    {
      Serial.println("[警告] OTG 模式未使能或由于异常保护退出!");
    }
  }
}