/*  Watchdog timer support for Arduino Zero
 *  by Richard Hole  December 19, 2015
 */

/*  Functions included:
 *  
 *  setupWDT(uint8_t period);
 *              initializes WDT in NORMAL mode
 *              initializes WDT period, 
 *                  valid values: 0x0 to 0xB (0-11)
 *                  see 17.18.2 and Table 17-5 in Atmel SAM D21G Datasheet
 *              enables WDT timer
 *  
 *  resetWDT();
 *              resets the WDT timer to prevent automatic system reset on timeout
 *  
 *  systemReset();
 *              forces an immediate system reset
 *              MUST first have WDT enabled 
 */
static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

void setup() {
  Serial.begin(230400);
  delay(2000);
  Serial.println("\nWDT demo\n");

  setupWDT( 11 ); // initialize and activate WDT with maximum period
  
  uint32_t begin= millis(); // keep track of time since WDT enabled
  
  while(millis()-begin <= 10000){ // loop up to 10 seconds or until WDT resets the board
    // this loop will print the number of millis() since WDT enabled
    // if WDT times out, the MCU will reset and the program begins again.
    
    delay(10);

    // to see WDT work, comment (//) the next line, or uncomment to see WDT reset work
    resetWDT();

    // the following line demonstrates a forced system reset at time>1000
    // uncomment the next line to activate this demo
    // if( millis()-begin >1000 ) systemReset();

    Serial.println( millis()-begin ); // print the elapsed time in milliSeconds
  //     Serial.println(GCLK->CLKCTRL.reg, HEX);
  // Serial.println(GCLK->GENCTRL.reg, HEX);

    // watchdog reset is seen by the program restarting and showing the "WDT demo" message
    
  }
  Serial.println("NO WDT reset occurred");
  disableWDT();
  
  while(1); // wait forever
}

void loop() {
  // no loop code required for the WDT test
}

//============= resetWDT ===================================================== 
void resetWDT() {
  // reset the WDT watchdog timer.
  // this must be called before the WDT resets the system
  WDT->CLEAR.reg= 0xA5; // reset the WDT
  WDTsync(); 
  // Serial.println("resetWDT");
}

//============= systemReset ================================================== 
void systemReset() {
  // use the WDT watchdog timer to force a system reset.
  // WDT MUST be running for this to work
  WDT->CLEAR.reg= 0x00; // system reset via WDT
  WDTsync(); 
}

//============= setupWDT =====================================================
void setupWDT( uint8_t period) {
  // initialize the WDT watchdog timer

  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required

  WDT->CONFIG.reg = min(period,11); // see Table 17-5 Timeout Period (valid values 0-11)

  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync(); 
  Serial.println("setupWDT");
}

//============= disable WDT =====================================================
void disableWDT() {
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required
  Serial.println("disableWDT");
}

