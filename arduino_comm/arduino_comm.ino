void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  while (!Serial) {
    ; // wait for serial port to connect.
  }

}

void loop() {
  char buffer[16];
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    int size = Serial.readBytesUntil('\n', buffer, 12);
    Serial.print(buffer);
  }
  
}
