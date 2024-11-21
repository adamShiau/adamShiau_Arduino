#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\README.md"
# IMU_PIG_V2 使用說明

## 需要注意的設定:

### 接收cmd時使用的port設定:

需在IMU_PIG_DEFINE.h 裡針對使用的port類型作 define,

|                   | Port name |   Rx pin   |   Tx pin   |
|:------------------|:----------|:----------:|:----------:|
| UART_SERIAL_5_CMD | mySerial5 |     D5     |     D6     |
| UART_USB_CMD      | Serial    | nano33 USB | nano33 USB |
| UART_RS422_CMD    | Serial1   |     D2     |     D1     |

```commandline
/*** UART port***/
// #define UART_SERIAL_5_CMD
// #define UART_USB_CMD
#define UART_RS422_CMD
```

### SYS pin 電路設定

SYS_TRIG pin 的設定，電路圖接到 net "SYNC_3.3V"，在IMU-V3 PCB接到D12

```commandline
/*** trig pin***/
#define SYS_TRIG 12
```

### ADXL_MUX pin 電路設定

電路圖接到 net "ADXL_MUX"，在IMU-V3 PCB接到D17

```commandline
int pin_scl_mux = 17;

Adxl355 adxl355(pin_scl_mux);
```