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
  Serial.write(cnt3);
  Serial.write(cnt2);
  Serial.write(cnt1);
  Serial.write(cnt0);
  cnt++;
  delay(10);
}
