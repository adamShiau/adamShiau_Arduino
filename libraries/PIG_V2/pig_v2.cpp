#include "pig_v2.h"

#define CHECK_BYTE		170
#define CHECK_BYTE2		171
#define CHECK_BYTE3		172

#define MOD_FREQ_ADDR			0
#define MOD_AMP_H_ADDR  		1
#define MOD_AMP_L_ADDR  		2
#define ERR_OFFSET_ADDR 		3
#define POLARITY_ADDR  			4
#define WAIT_CNT_ADDR  			5
#define ERR_TH_ADDR  			6
#define ERR_AVG_ADDR  			7
#define TIMER_RST_ADDR  		8
#define GAIN1_ADDR  			9
#define GAIN2_ADDR  			10
#define FB_ON_ADDR  			11
#define CONST_STEP_ADDR  		12
#define FPGA_Q_ADDR				13
#define FPGA_R_ADDR  			14
#define DAC_GAIN_ADDR  			50
#define DATA_INT_DELAY_ADDR 	98
#define DATA_OUT_START_ADDR		99

#define INT_SYNC	1
#define EXT_SYNC 	1<<1
#define STOP_SYNC 	1<<2


PIG::PIG(Stream &p ) : port(p)
{
	
}

PIG::~PIG()
{
	
}

void PIG::init()
{
	Serial.begin(230400);
	p_time_cnt = 0;
}

void PIG::sendCmd(unsigned char addr, unsigned int value)
{
	port.write(addr);
	port.write(value>>24 & 0xFF);
	port.write(value>>16 & 0xFF);
	port.write(value>>8 & 0xFF);
	port.write(value & 0xFF);
	delay(1);
}

char PIG::setSyncMode(unsigned int CTRLREG)
{
	char run_fog_flag = 0;
	
	switch(CTRLREG) {
		case INT_SYNC: {
			sendCmd(DATA_OUT_START_ADDR, 1);
			run_fog_flag = 1;
			break;
		}
		case EXT_SYNC: {
			sendCmd(DATA_OUT_START_ADDR, 2);
			run_fog_flag = 1;
			break;
		}
		case STOP_SYNC: {
			sendCmd(DATA_OUT_START_ADDR, 0);
			resetFakeDataTime();
			run_fog_flag = 0;
			break;
		}
		default: break;
	}
	return run_fog_flag;
}

void PIG::printVal(char name[], int val)
{
	Serial.print(name);
	Serial.print(": ");
	Serial.println(val);
}

void PIG::readData(unsigned char header[2], unsigned char data[14])
{
    alignHeader_2B(header);
    port.readBytes(data, 14);
    // printData(data);
}

void PIG::printData(unsigned char data[14])
{
    int err, step, pd_T;
	uint32_t time;

    time = data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3];
    err = data[4]<<24 | data[5]<<16 | data[6]<<8 | data[7];
	step = data[8]<<24 | data[9]<<16 | data[10]<<8 | data[11];
    pd_T = data[12] + data[13]>>7;

    Serial.print(port.available());
    Serial.print(", ");
	Serial.print(time);
    Serial.print(", ");
    Serial.print(err);
    Serial.print(", ");
    Serial.println(step);
}

//void PIG::convert2Sign_4B(unsigned char data[4]):
//    shift_data = (datain[1] << 8 | datain[0])
//    if (datain[1] >> 7) == 1:
//        return shift_data - (1 << 16)
//    else:
//        return shift_data

void PIG::readFakeDataCRC(unsigned char header[4], unsigned char data[17])
{
	unsigned int time;
	
	time = millis();
	checkFakeHeader(header);
	data[0] = time>>24;
	data[1] = time>>16;
	data[2] = time>>8;
	data[3] = time;
	for(int i=4; i<16; i++) 
		data[i] = i;
	data[16] = 255;
}

void PIG::readDataCRC(unsigned char header[4], unsigned char data[17])
{
	alignHeader_4B(header);
	port.readBytes(data, 17);
}

unsigned char* PIG::checkFakeHeader(unsigned char headerArr[4])
{
	headerArr[0] = KVH_HEADER[0];
	headerArr[1] = KVH_HEADER[1];
	headerArr[2] = KVH_HEADER[2];
	headerArr[3] = KVH_HEADER[3];
}

unsigned char* PIG::alignHeader_2B(unsigned char headerArr[2])
{
	unsigned char header[2];

	port.readBytes(headerArr, 2);
	// Serial.print("In: ");
	// Serial.print(headerArr[0], HEX);
	// Serial.print(", ");
	// Serial.println(headerArr[1], HEX);
	while(1)
	{

		if( (headerArr[0] == PIG_HEADER[0]) &&
		    headerArr[1] == PIG_HEADER[1]
		    )
		{
			p_time_cnt = 0;
           Serial.print("PASS: ");
           Serial.print(headerArr[0], HEX);
           Serial.print(",");
           Serial.println(headerArr[1], HEX);
		    return headerArr ;
		}


		else {
			p_time_cnt++;
			headerArr[0] = headerArr[1];
			headerArr[1] = port.read();
			
           // Serial.print("FAIL: ");
			// Serial.print(headerArr[0], HEX);
           // Serial.print(", ");
           // Serial.println(headerArr[1], HEX);
		   Serial.print("p_time_cnt: ");
		   Serial.println(p_time_cnt);
		   if(p_time_cnt > 10000) 
		   {
				digitalWrite(29, HIGH); //trigger signal to PIG
				EIC->CONFIG[1].bit.SENSE7 = 0; ////set interrupt condition to NONE
		   }

		   delayMicroseconds(100);
		}
	}
}

unsigned char* PIG::alignHeader_4B(unsigned char headerArr[4])
{
	unsigned char header[4];

	port.readBytes(headerArr, 4);
	while(1)
	{
		if( (headerArr[0] == KVH_HEADER[0]) &&
		    headerArr[1] == KVH_HEADER[1] &&
		    headerArr[2] == KVH_HEADER[2] &&
		    headerArr[3] == KVH_HEADER[3]
		    )
		   {
		    return headerArr ;
		   }



		else {
			headerArr[0] = headerArr[1];
			headerArr[1] = headerArr[2];
			headerArr[2] = headerArr[3];
			headerArr[3] = port.read();
		}
	}
}

//unsigned char* PIG::alignHeader_4B(unsigned char headerArr[4])
//{
//	unsigned char header[4], hold;
//
//	port.readBytes(headerArr, 4);
//	hold = 1;
//	while(hold)
//	{
//		if(	(headerArr[0] == HEADER[0]) &&
//			(headerArr[1] == HEADER[1]) &&
//			(headerArr[2] == HEADER[2]) &&
//			(headerArr[3] == HEADER[3])
//			){
//				hold = 0;
//				return headerArr ;
//			}
//		else {
//			headerArr[0] = headerArr[1];
//			headerArr[1] = headerArr[2];
//			headerArr[2] = headerArr[3];
//			headerArr[3] = port.read();
//		}
//	}
//}

void PIG::resetFakeDataTime()
{
	p_time_cnt = 0;
}

void PIG::readFakeData(unsigned char data[16])
{
	data[0] = p_time_cnt >> 24;
	data[1] = p_time_cnt >> 16;
	data[2] = p_time_cnt >> 8;
	data[3] = p_time_cnt ;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 1;
	data[8] = 0;
	data[9] = 0;
	data[10] = 0;
	data[11] = 2;
	data[12] = 0;
	data[13] = 0;
	data[14] = 0;
	data[15] = 3;
	p_time_cnt++;
}

void PIG::readData_debug(unsigned char data[16])
{
	unsigned char val1, val2, val3;
	unsigned int time, PD_T;
	int err, step;
	
	port.readBytes(&val1, 1);
	port.readBytes(&val3, 1);
	while(val1 != CHECK_BYTE or val3 != CHECK_BYTE3){
		val1 = val3;
		port.readBytes(&val3, 1);
	}
	port.readBytes(data, 16);
	port.readBytes(&val2, 1);
	time = data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3];
	err  = (int)(data[4]<<24 | data[5]<<16 | data[6]<<8 | data[7]);
	step = (int)(data[8]<<24 | data[9]<<16 | data[10]<<8 | data[11]);
	PD_T = data[12]<<24 | data[13]<<16 | data[14]<<8 | data[15];
	printVal("time: ", time);
	printVal("err: ", err);
	printVal("val1: ", val1);
	printVal("val3: ", val3);
	printVal("val2: ", val2);
}