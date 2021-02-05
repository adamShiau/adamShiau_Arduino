char val;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(38400);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    val = Serial.read();
    Serial1.print(val);
  }

  if(Serial1.available()) {
    val = Serial1.read();
    Serial.print(val);
  }
}
