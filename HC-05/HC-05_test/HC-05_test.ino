void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
}

int cnt=0;
void loop() {
  // put your main code here, to run repeatedly:
  Serial1.println(cnt);
  cnt++;
  delay(10);
}
