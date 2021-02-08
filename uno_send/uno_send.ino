long cnt = 0;
byte cnt0, cnt1, cnt2, cnt3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  cnt0 = cnt;
  cnt1 = cnt >> 8;
  cnt2 = cnt >> 16;
  cnt3 = cnt >> 24;
  Serial.write(0xC0);
  Serial.write(0xC0);
  Serial.write(cnt3);
  Serial.write(cnt2);
  Serial.write(cnt1);
  Serial.write(cnt0);
//  Serial.write(7);
//  Serial.write(8);
//  Serial.write(9);
//  Serial.write(10);
//  Serial.write(11);
//  Serial.write(12);
  cnt++;
  delay(10);
}
