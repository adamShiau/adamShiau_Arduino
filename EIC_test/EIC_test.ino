volatile int cnt = 0;
//byte tt=0;

void setup() {
  // put your setup code here, to run once:
//  pinMode(2, INPUT_PULLUP);
  attachInterrupt(2, ISR_test, RISING);
  pinMode(3, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
//Serial.println("hi");
//delay(1000);
//Serial.println(EIC->INTFLAG.reg, HEX);
}

void ISR_test()
{
  EIC->CONFIG[1].reg = 0;
  do_somthing();
//  noInterrupts();
  Serial.println(g_APinDescription[2].ulExtInt);
  Serial.println(EIC->CONFIG[0].reg, HEX);
  Serial.println(EIC->CONFIG[1].reg, HEX);
  Serial.println(cnt);
  Serial.println();
  
  cnt++;
}

void do_somthing()
{
//  flag = 0;
  
  for(int i=0; i<65535; i++) {
    Serial.println(i);
  }
  EIC->CONFIG[1].reg = 0x100;
//  delay(10);
//  flag = 1;
}
