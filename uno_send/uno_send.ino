long cnt = 12345;
byte cnt0, cnt1, cnt2, cnt3;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  cnt0 = cnt;
  cnt1 = cnt >> 8;
  cnt2 = cnt >> 16;
  cnt3 = cnt >> 24;
 // Serial.write(0xC1);
  Serial1.write(0xAB);
  Serial1.write(cnt3);
  Serial1.write(cnt2);
  Serial1.write(cnt1);
  Serial1.write(cnt0);
// Serial.write(1);
// Serial.write(2);
// Serial.write(3);
// Serial.write(4);
//  Serial.write(11);
//  Serial.write(12);
  cnt++;
  delayMicroseconds(500);
}
