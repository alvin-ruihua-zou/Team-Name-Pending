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
    int size = Serial.readBytesUntil('0', buffer, 1);
    Serial.print(buffer);
    memset(buffer, 0,16);
    while (Serial.available()>0){
        char t = Serial.read();
    }
  }
  
}
