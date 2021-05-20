# wattius_bms
Node to communicate with Wattius BMS using Modbus

## Installation

```
git clone https://github.com/RobotnikAutomation/wattius_bms.git
```

## Parameters


- **ip_address** (String, localhost): IP address of the device
- **port** (Int, 512): Port to communicate thorough Modbus
- **soc_input_register** (Int, 1000): SOC Input register
- **voltage_input_register** (Int, 1001): Voltage input register
- **current_input_register** (Int, 1002): Current input register
