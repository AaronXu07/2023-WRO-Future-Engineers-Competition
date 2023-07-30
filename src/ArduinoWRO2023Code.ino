#include <Servo.h>

Servo esc;
Servo ser;
int led = 13;

const int BUFFER_SIZE = 5;
char buf[BUFFER_SIZE];

void setup(){

  pinMode(led, OUTPUT);

  Serial.begin(115200);
  ser.attach(9); //blue wire is servo
  ser.write(98);
  esc.attach(10); //white wire is motor
  esc.writeMicroseconds(1500);

  digitalWrite(led, HIGH);
  delay(6000);//mandatory delay waiting for motor to be ready
  digitalWrite(led, LOW);
}

void loop() {
  while(Serial.available() > 0) {


    Serial.readBytes(buf, BUFFER_SIZE); //reading command from rasppi
    buf[4] = '\0';
    int command = atoi(buf);

    //Serial.println(command);
    
    if(command <= 2000)
    {
      esc.writeMicroseconds(command);
    }
    else
    {
      ser.write(command - 2000);
    }
  }
}

