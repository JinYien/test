void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int MxValue = analogRead(A0);
  int MyValue = analogRead(A1);
  int MzValue = analogRead(A2);

  Serial.print(MxValue);
  Serial.print(',');
  Serial.print(MyValue);
  Serial.print(',');
  Serial.println(MzValue);

}
