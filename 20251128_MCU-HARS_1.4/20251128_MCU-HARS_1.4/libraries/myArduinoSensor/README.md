# myArduinoSensor

## 2025/03/21

### myUARTSensor

#### Xsens Reader

1. ***INS_PAV_QUT()***: includes the IMU outputs.
2. **LEN_XBUS**: Increase LEN_XBUS from 100 to 200.

### myMessage

1. ***convert2sign_8B( )***: Fix the problem that bytes start with LSB (Little-endian).

2. ***appendValue2Str( )***: Increase the stability for memory managment, add the judgement of invalid value such as nan or inf. And make ***appendValues2Str( )*** depends on  ***appendValue2Str( )***.

### example

1. Add Xsens, ADIS16505-2, GYPRO4300 reading examples.
