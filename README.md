## 注意事项
- **PSTOP 引脚**：修改 VBAT_SET、CTRL0 等关键寄存器前，必须将 PSTOP 拉高，修改完成后拉低。
- **CE 引脚**：芯片使能引脚，通常低电平有效，需根据实际电路确认。
- **采样电阻**：根据实际电路修改 `SC8815_VBUS_RSHUNT_MOHM` 和 `SC8815_VBAT_RSHUNT_MOHM` 宏。
- **ADC 换算系数**：`readVbusVoltage()` 等函数中的 LSB 值（如 40mV/LSB）为典型值，实际使用时建议用万用表校准。
- **I2C 地址**：默认 0x74（7 位），可通过 `i2c_scanner` 确认。