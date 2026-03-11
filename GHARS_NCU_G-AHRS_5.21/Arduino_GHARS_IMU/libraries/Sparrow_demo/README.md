# Sparrow_read class 使用說明

## Author: Adam Shiau
## data: 5/19/2022

### class usage

此 class 之 constructor 須把 Serial object 丟入，若是使用Serial、Serial1 可直接丟入:

```commandline
Sparrow_read sparrow(Serial);
```

若是使用SERCOM mux則須在丟入前先declare:

```commandline
Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

Sparrow_read sparrow(mySerial5);

void SERCOM0_Handler()
{
	mySerial5.IrqHandler();
}
```

### 讀取數據之method:

宣告一個 6-byte 空間後代入 object.readData method，

fog data 會回傳至此變數空間，前4 byte為gyro rate，後 2 byte 為溫度，

data update rate = 100Hz 為 Quantaser 公司出廠設定無法變更

```commandline
byte data[6];

sparrow.readData(data);
```