


void setup()
{
	Serial.begin(115200);
	Serial1.begin(115200);
}

void loop()
{
	IRQ_UART_ISR();
	// if (Serial.available()) {
		// byte inChar = Serial.read();
		// Serial.write(inChar);
	// }  
  

}

byte rx_cnt = 0, uart_cmd;
unsigned int uart_value;
bool uart_complete = 0;

void IRQ_UART_ISR()
{
	if (Serial.available()){
		switch(rx_cnt) {
		case 0: {
			uart_cmd = Serial.read();
			rx_cnt = 1;
			break;
		}
		case 1: {
			uart_value = Serial.read()<<24;
			rx_cnt = 2;
			break;
		}
		case 2: {
			uart_value |= Serial.read()<<16;
			rx_cnt = 3;
			break;
		}
		case 3: {
			uart_value |= Serial.read()<<8;
			rx_cnt = 4;
			break;
		}
		case 4: {
			uart_value |= Serial.read();
			rx_cnt = 0;
			uart_complete = 1;
			break;
		}
		}
		// Serial.print(rx_cnt);
		// Serial.print(", ");
		// Serial.println(uart_complete);
	}
	
	if(uart_complete) {
		uart_complete = 0;
		Serial1.write(uart_cmd);
		Serial1.write(uart_value>>24);
		Serial1.write(uart_value>>16);
		Serial1.write(uart_value>>8);
		Serial1.write(uart_value);
		// switch(uart_cmd){
		// case 0: IOWR(VARSET_BASE, O_VAR_FREQ, uart_value); break;
		// case 1: IOWR(VARSET_BASE, O_VAR_AMP_H, uart_value); break;
		// case 2: IOWR(VARSET_BASE, O_VAR_AMP_L, uart_value); break;
		// case 3: IOWR(VARSET_BASE, O_VAR_OFFSET, uart_value); break;
		// case 4: IOWR(VARSET_BASE, O_VAR_POLARITY, uart_value); break;
		// case 5: IOWR(VARSET_BASE, O_VAR_WAITCNT, uart_value); break;
		// case 6: IOWR(VARSET_BASE, O_VAR_ERRTH, uart_value); break;
		// case 7: IOWR(VARSET_BASE, O_VAR_AVGSEL, uart_value); break;
		// case 8: {
				// IOWR(VARSET_BASE, O_VAR_TIMER_RST, 1);
				// IOWR(VARSET_BASE, O_VAR_TIMER_RST, 0);
				// break;
		// }
		// case 9: IOWR(VARSET_BASE, O_VAR_GAIN1_SEL, uart_value); break;
		// case 10: IOWR(VARSET_BASE, O_VAR_GAIN2_SEL, uart_value); break;
		// case 11: IOWR(VARSET_BASE, O_VAR_FB_ON, uart_value); break;
		// case 12: IOWR(VARSET_BASE, O_VAR_CONST_STEP, uart_value); break;
		// case 13: IOWR(VARSET_BASE, O_VAR_KAL_Q, uart_value); break;
		// case 14: IOWR(VARSET_BASE, O_VAR_KAL_R, uart_value); break;
		// case 50: {
			// IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_ADDA_BASE, (DAC1_GAIN_LSB_ADDR<<8) | (uart_value&0xFF)); usleep (10);
			// IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_ADDA_BASE, (DAC1_GAIN_MSB_ADDR<<8) | (uart_value>>8)); usleep (10);
			// IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_ADDA_BASE, (DAC2_GAIN_LSB_ADDR<<8) | (uart_value&0xFF)); usleep (10);
			// IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_ADDA_BASE, (DAC2_GAIN_MSB_ADDR<<8) | (uart_value>>8)); usleep (10);
			// break;
		// }
		// case 98: delay_time = uart_value; break;
		// case 99: start_flag = uart_value; break;
		// }
	}
}
