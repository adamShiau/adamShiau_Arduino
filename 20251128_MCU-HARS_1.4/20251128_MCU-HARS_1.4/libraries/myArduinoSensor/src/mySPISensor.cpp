#include "mySPISensor.h"

ADIS16505::ADIS16505(){}

ADIS16505::ADIS16505(uint16_t CS, uint16_t DRDY, u_int8_t RST) : DRDY(DRDY), CS(CS), RST(RST){
    SPI.begin();    
    pinMode(DRDY, INPUT);
    pinMode(CS, OUTPUT);
    pinMode(RST, OUTPUT);
    digitalWrite(RST, HIGH);
    deselect();
}

void ADIS16505::select() {
  SPISettings IMUSettings(1000000, MSBFIRST, SPI_MODE3);
  SPI.beginTransaction(IMUSettings);
  digitalWrite(CS, LOW); // Set CS low to enable device
}

void ADIS16505::deselect() {
  SPI.endTransaction();
  digitalWrite(CS, HIGH); // Set CS high to disable device
}

void ADIS16505::initSensor(){
    digitalWrite(RST, LOW);
    delay(10);
    digitalWrite(RST, HIGH);
    delay(500);

    regWrite(MSC_CTRL, 0x02C1);
    regWrite(FILT_CTRL, 0x04);
    regWrite(DEC_RATE, 0x0013);
    delay(100);

    Serial.print("PROD_ID  :");Serial.println(readReg(PROD_ID), HEX);
    Serial.print("DIAG_STAT:");Serial.println(readReg(DIAG_STAT), HEX);
    Serial.print("MSC_CTRL :");Serial.println(readReg(MSC_CTRL), HEX);
    Serial.print("FILT_CTRL:");Serial.println(readReg(FILT_CTRL), HEX);
    Serial.print("DEC_RATE :");Serial.println(readReg(DEC_RATE), HEX);
    Serial.print("RANG_MDL :");Serial.println(readReg(RANG_MDL), HEX);
    Serial.println();
    delay(500);
}

bool ADIS16505::checkDRDY(){
    return (digitalRead(DRDY) == HIGH);
}

bool ADIS16505::readData(float (&omg)[3], float (&acc)[3], float &temp, int32_t &ct){
    //Read the temperature data
    uint8_t burstdata[32];
    int32_t burstwords[10];

    select();
    SPI.transfer(0x68);
    SPI.transfer(0x00);
    // Read Burst Data
    for (int i=0;i<32;i++){
        burstdata[i] = SPI.transfer(0x00);
    }
    deselect();

    // Read Burst Data
    burstwords[0] = ((burstdata[0] << 8) | (burstdata[1] & 0xFF));
    burstwords[1] = ((burstdata[4] << 24) | (burstdata[5] << 16) | (burstdata[2] << 8) | (burstdata[3] & 0xFF));
    burstwords[2] = ((burstdata[8] << 24) | (burstdata[9] << 16) | (burstdata[6] << 8) | (burstdata[7] & 0xFF));
    burstwords[3] = ((burstdata[12] << 24) | (burstdata[13] << 16) | (burstdata[10] << 8) | (burstdata[11] & 0xFF));
    burstwords[4] = ((burstdata[16] << 24) | (burstdata[17] << 16) | (burstdata[14] << 8) | (burstdata[15] & 0xFF));
    burstwords[5] = ((burstdata[20] << 24) | (burstdata[21] << 16) | (burstdata[18] << 8) | (burstdata[19] & 0xFF));
    burstwords[6] = ((burstdata[24] << 24) | (burstdata[25] << 16) | (burstdata[22] << 8) | (burstdata[23] & 0xFF));
    burstwords[7] = ((burstdata[26] << 8) | (burstdata[27] & 0xFF));
    burstwords[8] = ((burstdata[28] << 8) | (burstdata[29] & 0xFF));
    burstwords[9] = ((burstdata[30] << 8) | (burstdata[31] & 0xFF));

    temp = burstwords[7] * 0.1;
    ct = burstwords[8];
    for (int i=0;i<3;i++){
        omg[i] = burstwords[i+1] / GYRO_32bit_SF;
        acc[i] = burstwords[i+4] / ACCL_32bit_SF;
    }
    
    if (burstwords[8] > counter){
        counter = burstwords[8];
        return true;
    }
    else{
        counter = burstwords[8];
        return false;
    } 
}

int16_t ADIS16505::readReg(uint8_t regAddr){
    // Write register address to be read
    select();
    SPI.transfer(regAddr); // Write address over SPI bus
    SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
    deselect();
    delayMicroseconds(20);

    // Read data from requested register
    select();
    uint8_t _msbData = SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
    uint8_t _lsbData = SPI.transfer(0x00); // Send (0x00) and place lower byte into variable
    deselect();
    delayMicroseconds(20);
    
    int16_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
    // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

    return(_dataOut);
}

int ADIS16505::regWrite(uint8_t regAddr, int16_t regData){
    // Write register address and data
    uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
    uint16_t lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
    uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

    // Split words into chars
    uint8_t highBytehighWord = (highWord >> 8);
    uint8_t lowBytehighWord = (highWord & 0xFF);
    uint8_t highBytelowWord = (lowWord >> 8);
    uint8_t lowBytelowWord = (lowWord & 0xFF);

    // Write highWord to SPI bus
    select();
    SPI.transfer(highBytelowWord); // Write high byte from low word to SPI bus
    SPI.transfer(lowBytelowWord); // Write low byte from low word to SPI bus
    deselect();

    delayMicroseconds(20);; // Delay to not violate read rate 

    // Write lowWord to SPI bus
    select();
    SPI.transfer(highBytehighWord); // Write high byte from high word to SPI bus
    SPI.transfer(lowBytehighWord); // Write low byte from high word to SPI bus
    deselect();

    delayMicroseconds(20);; // Delay to not violate read rate 

    return(1);
}

bool ADIS16505::checksum(uint8_t * burstArray, uint8_t num_bytes){
    uint16_t s = 0;
    for (int i = 0; i < num_bytes; i++) // Checksum value is not part of the sum!!
    {   
        s += burstArray[i];
    }
    uint16_t cks = ((burstArray[num_bytes] << 8) | (burstArray[num_bytes+1] & 0xFF));
    return (s == cks);
}


GYPRO4300::GYPRO4300(){}

GYPRO4300::GYPRO4300(uint16_t CS, uint16_t DRDY, uint16_t EN, uint16_t VDDIO, int WS) : DRDY(DRDY), CS(CS), EN(EN), VDDIO(VDDIO), WS(WS){
    SPI.begin();
    // SPI.setDataMode(SPI_MODE0);           // Set SPI communication MODE0 : CPOL=0 CPHA=0
	// SPI.setClockDivider(SPI_CLOCK_DIV16); // Set SPI Frequency at 1 MHz
	// SPI.setBitOrder(MSBFIRST);            // Set bit order : Most Significant Bit First 	

    Serial.println("Setting PIN definition");
    pinMode(EN, OUTPUT);
	pinMode(DRDY, INPUT);             // Set DRDY (Data Ready) Pin as Input
	pinMode(CS, OUTPUT);             // Set CS (SPI Chip select) Pin as Output
    pinMode(VDDIO, OUTPUT);           // Set VDDIO Pin as Output
  
	digitalWrite(EN, LOW);           // Set EN (Enable) pin to Low Level (Active Low) (To Reset the sensor)
	digitalWrite(CS, HIGH);          // Set CS (SPI Chip select) pin to High Level (Active Low)
    digitalWrite(VDDIO, LOW);         // Set VDDIO pin to High Level (Active Low)

    select();
    deselect();

    delay(1000);
	digitalWrite(EN, HIGH);
    Serial.println("set EN pin high");
    delay(1000);

    uint32_t TestASIC1 = readReg(0x00);
    uint32_t TestASIC2 = readReg(0x40);
    Serial.print("TestASIC1: ");Serial.println(TestASIC1, HEX);
    Serial.print("TestASIC2: ");Serial.println(TestASIC2, HEX);

    if ((TestASIC1 == TestASIC2) && (TestASIC1 != 0)) {
        digitalWrite(VDDIO, LOW);       // Set VDDIO pin to Low Level (Active High)
    }
    else {
        digitalWrite(VDDIO, HIGH);      // Set VDDIO pin to High Level (Active High)
        Serial.println("set VDDIO high");
    }		
}

void GYPRO4300::select() {
  SPISettings IMUSettings(1000000, MSBFIRST, SPI_MODE0);
  SPI.beginTransaction(IMUSettings);
  digitalWrite(CS, LOW); // Set CS low to enable device
}

void GYPRO4300::deselect() {
  SPI.endTransaction();
  digitalWrite(CS, HIGH); // Set CS high to disable device
}

void GYPRO4300::initSensor(){
    Serial.println("Initialize Parameters");
    writeReg(4759542, 0x26);        // fix the bug of bad register value
    writeReg(2215945084, 0x3D);     // adjust date rate to about 900Hz
    // writeReg(2215945056, 0x3D);     // adjust date rate to about 600Hz

    Serial.print("UID: ");Serial.println((readReg(0x03) & 0x3FFFFFFE) >> 1);
    Serial.print("ODRSEL: ");Serial.println(30000. / (((readReg(0x3D) & 0xFF) >> 2) + 1), 0);
    Serial.print("GOUT_SEL: ");Serial.println((readReg(0x3D) & 0x80000000) >> 31);
    Serial.print("MTPSLOTNB (BIN): ");Serial.println((readReg(0x3D) & 0x1F00) >> 8, BIN);
    Serial.print("TOUT_SEL: ");Serial.println((readReg(0x04) & 8) >> 3);
    Serial.print("TOUT_SEL_O: ");Serial.println((readReg(0x04) & 0xFFFC0000) >> 18, HEX);
    Serial.print("TOUT_SEL_G: ");Serial.println((readReg(0x04) & 0x3FFF0) >> 4, HEX);
    // Serial.print("SF4: ");Serial.println(readReg(0x48) & 0x3FFFF);
    // Serial.print("SF3: ");Serial.println(readReg(0x46) & 0x7FFFF);
    // Serial.print("SF2: ");Serial.println(readReg(0x44) & 0xFFFFF);
    // Serial.print("SF1: ");Serial.println(readReg(0x42) & 0x1FFFFFFF);
    // Serial.print("SF0: ");Serial.println(readReg(0x3F) & 0x3FFFFFFF);
    // Serial.print("B4: ");Serial.println(readReg(0x47) & 0x3FFFF);
    // Serial.print("B3: ");Serial.println(readReg(0x45) & 0x7FFFF);
    // Serial.print("B2: ");Serial.println(readReg(0x43) & 0x7FFFF);
    // Serial.print("B1: ");Serial.println(readReg(0x41) & 0x1FFFFFFF);
    // Serial.print("B0: ");Serial.println(readReg(0x4E) & 0x7FFFFF);
    Serial.print("TMID: ");Serial.println(readReg(0x40) & 0x7FFFF);
    Serial.println();
    
    // for (int i=0;i<=0x52;i++){
    //     Serial.print(i, HEX);Serial.print(" ");Serial.println(readReg(i), HEX);
    // }
    // Serial.println();
}

void GYPRO4300::disableCalibrationMode(){
    writeReg(0x414A33C,0x3D);
    Serial.print("GOUT_SEL: ");
    Serial.println((readReg(0x3D) & 0x80000000) >> 31);
}

bool GYPRO4300::checkDRDY(){
    return (digitalRead(DRDY) == HIGH);
}

bool GYPRO4300::readData(float &omg, float &temp){
    uint8_t byte_read[8];
    readOutput(0x50, byte_read, 8);                // Read the 6 bytes of the Sensor Output (Acceleration + Temperature + Selft Test)
    bool dataReady = ((byte_read[0] & 0b10000000) == 128); // Check Data Ready bit
    
    if (dataReady) {
        int32_t rate_int;                          // Define Rate 32-bits variable
        uint16_t temp_int;
        uint8_t ST;
        float omg0, temp0;
        
        byte_read[0] &= 0b01111111;       // Bitmask to have only bits 6 to 0 of the first byte
        ST = byte_read[3] & 0b00000001;
        byte_read[3] &= 0b10000000;       // Bitmask to have only bits 7 of the fourth byte
        
        if (ST == 0) { return false; }

        rate_int = (byte_read[0] << 24) | (byte_read[1] << 16) | (byte_read[2] << 8) | byte_read[3];
        rate_int = rate_int >> 7;
        if (byte_read[0] & 0b01000000){rate_int -= 0x1000000;}
        omg0 = float(rate_int) / 10000.;

        temp_int = (byte_read[4] << 8) | byte_read[5];
        temp0 = float(temp_int-8000) / 85 + 25.0;

        vec_omg.push(omg0);
        // vec_temp.push(temp0);
        sum_omg += omg0;
        // sum_temp += temp0;
        counter++;

        if (vec_omg.size() > WS){
            sum_omg -= vec_omg.front();
            // sum_temp -= vec_temp.front();
            vec_omg.pop();
            // vec_temp.pop();
        }

        if (counter == WS){
            omg = sum_omg / WS;
            temp = temp0;
            // temp = sum_temp / WS;
            counter = 0;
            return true;
        }
    }
    return false;
}

void GYPRO4300::readOutput(uint8_t address, uint8_t Buffer_Sensor[], uint8_t Buffer_Size) {	
    select();
    SPI.transfer(address);                     // reading command : 0x5 + start address : 0x0 --> 0x50
	for (uint8_t i=0 ; i < Buffer_Size ; i++) {
        Buffer_Sensor[i] = SPI.transfer(0);
    }
    deselect();
}

uint32_t GYPRO4300::readReg(uint32_t regAddr){
    select();		// Set CS to Low Level (Active)
	SPI.transfer(0x7C);        	 	// Send 0x7 (write command) and 0xC (SPI register address) -> 0x7C
	SPI.transfer(regAddr);  		// Send The system register adress
	SPI.transfer(0x01);				// Send System register command (0x01 : read from system register)
	digitalWrite(CS, HIGH);

	delay(10);

	uint8_t rspi[4] = {};

	digitalWrite(CS, LOW);          // Set CS to Low Level (Active)
	SPI.transfer(0x58);;                   // Send 0x5 (read command) and 0x8 (SPI register address) -> 0x58
	rspi[0] = SPI.transfer(0x00);     // Read first incoming byte
	rspi[1] = SPI.transfer(0x00);     // Read second incoming byte
	rspi[2] = SPI.transfer(0x00);     // Read third incoming byte
	rspi[3] = SPI.transfer(0x00);     // Read last incoming byte
	deselect();          // Set CS to high Level (Inactive)

	return (rspi[0] << 24) | (rspi[1] << 16) | (rspi[2] << 8) | rspi[3];          // Return the data
}


void GYPRO4300::writeReg(uint32_t Data, uint32_t Address) {
	uint8_t wspi[4] = {};

	wspi[0] = (Data & 0xff000000) >> 24;  // Detachment of the data into 4 bytes
	wspi[1] = (Data & 0x00ff0000) >> 16;
	wspi[2] = (Data & 0x0000ff00) >>  8;
	wspi[3] = (Data & 0x000000ff)      ;

	select();
	SPI.transfer(0x78);           	// Send 0x7 (write command) and 0x8 (SPI register address) -> 0x78
	SPI.transfer(wspi[0]);         	// Send first byte of the data
	SPI.transfer(wspi[1]);         	// Send second byte of the data
	SPI.transfer(wspi[2]);         	// Send third byte of the data
	SPI.transfer(wspi[3]);         	// Send last byte of the data
	SPI.transfer(Address);  		// Send The system register adress
	SPI.transfer(0x02);           	// Send System register command (0x02 : write to system register)
	deselect();  	// Set CS to high Level (Inactive)

    Serial.print(0x78, HEX); Serial.print(" ");
    Serial.print(wspi[0], HEX); Serial.print(" ");
    Serial.print(wspi[1], HEX); Serial.print(" ");
    Serial.print(wspi[2], HEX); Serial.print(" ");
    Serial.print(wspi[3], HEX); Serial.print(" ");
    Serial.print(Address, HEX); Serial.print(" ");
    Serial.print(0x02, HEX); Serial.print(" ");
    Serial.println();
}